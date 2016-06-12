/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <unordered_map>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "debug.hpp"
// UAVCAN
#include <uavcan/protocol/dynamic_node_id_server/distributed.hpp>
// UAVCAN Linux drivers
#include <uavcan_linux/uavcan_linux.hpp>
// UAVCAN POSIX drivers
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>

namespace
{

constexpr int MaxNumLastEvents = 30;
constexpr int MinUpdateInterval = 100;

uavcan_linux::NodePtr initNode(const std::vector<std::string>& ifaces, uavcan::NodeID nid, const std::string& name)
{
    const auto app_id = uavcan_linux::makeApplicationID(uavcan_linux::MachineIDReader().read(),  name,  nid.get());

    uavcan::protocol::HardwareVersion hwver;
    std::copy(app_id.begin(), app_id.end(), hwver.unique_id.begin());
    std::cout << hwver << std::endl;

    auto node = uavcan_linux::makeNode(ifaces, name.c_str(), uavcan::protocol::SoftwareVersion(), hwver, nid);

    node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);
    node->setModeOperational();

    return node;
}


class EventTracer : public uavcan_posix::dynamic_node_id_server::FileEventTracer
{
public:
    struct RecentEvent
    {
        const uavcan::MonotonicDuration time_since_startup;
        const uavcan::UtcTime utc_timestamp;
        const uavcan::dynamic_node_id_server::TraceCode code;
        const std::int64_t argument;

        RecentEvent(uavcan::MonotonicDuration arg_time_since_startup,
                    uavcan::UtcTime arg_utc_timestamp,
                    uavcan::dynamic_node_id_server::TraceCode arg_code,
                    std::int64_t arg_argument)
            : time_since_startup(arg_time_since_startup)
            , utc_timestamp(arg_utc_timestamp)
            , code(arg_code)
            , argument(arg_argument)
        { }

        uavcan::MakeString<81>::Type toString() const   // Heapless return
        {
            char timebuf[12] = { };
            {
                const std::time_t rawtime = utc_timestamp.toUSec() * 1e-6;
                const auto tm = std::localtime(&rawtime);
                std::strftime(timebuf, 10, "%H:%M:%S.", tm);
                std::snprintf(&timebuf[9], 3, "%02u", static_cast<unsigned>((utc_timestamp.toMSec() % 1000U) / 10U));
            }

            decltype(toString()) out;
            out.resize(out.capacity());
            // coverity[overflow : FALSE]
            (void)std::snprintf(reinterpret_cast<char*>(out.begin()), out.size() - 1U,
                                "%-11s  %-28s %-20lld %016llx",
                                timebuf,
                                getEventName(code),
                                static_cast<long long>(argument),
                                static_cast<long long>(argument));
            return out;
        }

        static const char* getTableHeader()
        {
            // Matches the string format above
            return "Timestamp    Event name                   Argument (dec)       Argument (hex)";
        }
    };

    struct EventStatisticsRecord
    {
        std::uint64_t count;
        uavcan::MonotonicTime last_occurence;

        EventStatisticsRecord()
            : count(0)
        { }

        void hit(uavcan::MonotonicTime ts)
        {
            count++;
            last_occurence = ts;
        }
    };

private:
    struct EnumKeyHash
    {
        template <typename T>
        std::size_t operator()(T t) const { return static_cast<std::size_t>(t); }
    };

    uavcan_linux::SystemClock clock_;
    const uavcan::MonotonicTime started_at_ = clock_.getMonotonic();
    const unsigned num_last_events_;

    std::deque<RecentEvent> last_events_;
    std::unordered_map<uavcan::dynamic_node_id_server::TraceCode, EventStatisticsRecord, EnumKeyHash> event_counters_;

    bool had_events_ = false;

    void onEvent(uavcan::dynamic_node_id_server::TraceCode code, std::int64_t argument) override
    {
        uavcan_posix::dynamic_node_id_server::FileEventTracer::onEvent(code, argument);

        had_events_ = true;

        const auto ts_m = clock_.getMonotonic();
        const auto ts_utc = clock_.getUtc();
        const auto time_since_startup = ts_m - started_at_;

        last_events_.emplace_front(time_since_startup, ts_utc, code, argument);
        if (last_events_.size() > num_last_events_)
        {
            last_events_.pop_back();
        }

        event_counters_[code].hit(ts_m);
    }

public:
    EventTracer(unsigned num_last_events_to_keep)
        : num_last_events_(num_last_events_to_keep)
    { }

    using uavcan_posix::dynamic_node_id_server::FileEventTracer::init;

    const RecentEvent& getEventByIndex(unsigned index) const { return last_events_.at(index); }

    unsigned getNumEvents() const { return last_events_.size(); }

    const decltype(event_counters_)& getEventCounters() const { return event_counters_; }

    bool hadEvents()
    {
        if (had_events_)
        {
            had_events_ = false;
            return true;
        }
        return false;
    }
};


::winsize getTerminalSize()
{
    auto w = ::winsize();
    ENFORCE(0 >= ioctl(STDOUT_FILENO, TIOCGWINSZ, &w));
    ENFORCE(w.ws_col > 0 && w.ws_row > 0);
    return w;
}


std::vector<std::pair<uavcan::dynamic_node_id_server::TraceCode, EventTracer::EventStatisticsRecord>>
collectRelevantEvents(const EventTracer& event_tracer, const unsigned num_events)
{
    // First, creating a vector of pairs (event code, count)
    typedef std::pair<uavcan::dynamic_node_id_server::TraceCode, EventTracer::EventStatisticsRecord> Pair;
    const auto counters = event_tracer.getEventCounters();
    std::vector<Pair> pairs(counters.size());
    std::copy(counters.begin(), counters.end(), pairs.begin());

    // Now, sorting the pairs so that the most recent ones are on top of the list
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return a.second.last_occurence > b.second.last_occurence;
    });

    // Cutting the oldest events away
    pairs.resize(std::min(num_events, unsigned(pairs.size())));

    // Sorting so that the most frequent events are on top of the list
    std::stable_sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return a.second.count > b.second.count;
    });

    return pairs;
}

enum class CLIColor : unsigned
{
    Red     = 31,
    Green   = 32,
    Yellow  = 33,
    Blue    = 34,
    Magenta = 35,
    Cyan    = 36,
    White   = 37,
    Default = 39
};

CLIColor getColorHash(unsigned value) { return CLIColor(31 + value % 7); }

class CLIColorizer
{
    const CLIColor color_;
public:
    explicit CLIColorizer(CLIColor c) : color_(c)
    {
        std::printf("\033[%um", static_cast<unsigned>(color_));
    }

    ~CLIColorizer()
    {
        std::printf("\033[%um", static_cast<unsigned>(CLIColor::Default));
    }
};


void redraw(const uavcan_linux::NodePtr& node,
            const uavcan::MonotonicTime timestamp,
            const EventTracer& event_tracer,
            const uavcan::dynamic_node_id_server::DistributedServer& server)
{
    using uavcan::dynamic_node_id_server::distributed::RaftCore;

    /*
     * Constants that are permanent for the designed UI layout
     */
    constexpr unsigned NumRelevantEvents = 17;
    constexpr unsigned NumRowsWithoutEvents = 3;

    /*
     * Collecting the data
     */
    const unsigned num_rows = getTerminalSize().ws_row;

    const auto relevant_events = collectRelevantEvents(event_tracer, NumRelevantEvents);

    const uavcan::dynamic_node_id_server::distributed::StateReport report(server);

    const auto time_since_last_activity = timestamp - report.last_activity_timestamp;

    /*
     * Basic rendering functions
     */
    unsigned next_relevant_event_index = 0;

    const auto render_next_event_counter = [&]()
    {
        const char* event_name = "";
        char event_count_str[10] = { };
        CLIColor event_color = CLIColor::Default;

        if (next_relevant_event_index < relevant_events.size())
        {
            const auto e = relevant_events[next_relevant_event_index];
            event_name = uavcan::dynamic_node_id_server::IEventTracer::getEventName(e.first);
            std::snprintf(event_count_str, sizeof(event_count_str) - 1U, "%llu",
                          static_cast<unsigned long long>(e.second.count));
            event_color = getColorHash(static_cast<unsigned>(e.first));
        }
        next_relevant_event_index++;

        std::printf(" | ");
        CLIColorizer izer(event_color);
        std::printf("%-29s %-9s\n", event_name, event_count_str);
    };

    const auto render_top_str = [&](const char* local_state_name, const char* local_state_value, CLIColor color)
    {
        {
            CLIColorizer izer(color);
            std::printf("%-20s %-16s", local_state_name, local_state_value);
        }
        render_next_event_counter();
    };

    const auto render_top_int = [&](const char* local_state_name, long long local_state_value, CLIColor color)
    {
        char buf[21];
        std::snprintf(buf, sizeof(buf) - 1U, "%lld", local_state_value);
        render_top_str(local_state_name, buf, color);
    };

    const auto raft_state_to_string = [](uavcan::dynamic_node_id_server::distributed::RaftCore::ServerState s)
    {
        switch (s)
        {
        case RaftCore::ServerStateFollower:  return "Follower";
        case RaftCore::ServerStateCandidate: return "Candidate";
        case RaftCore::ServerStateLeader:    return "Leader";
        default:                             return "BADSTATE";
        }
    };

    const auto duration_to_string = [](uavcan::MonotonicDuration dur)
    {
        uavcan::MakeString<16>::Type str;                       // This is much faster than std::string
        str.appendFormatted("%.1f", dur.toUSec() / 1e6);
        return str;
    };

    const auto colorize_if = [](bool condition, CLIColor color)
    {
        return condition ? color : CLIColor::Default;
    };

    /*
     * Rendering the data to the CLI
     */
    std::printf("\x1b[1J"); // Clear screen from the current cursor position to the beginning
    std::printf("\x1b[H");  // Move cursor to the coordinates 1,1

    // Local state and relevant event counters - two columns
    std::printf("        Local state                   |         Event counters\n");

    render_top_int("Node ID",
                   node->getNodeID().get(),
                   CLIColor::Default);

    render_top_str("State",
                   raft_state_to_string(report.state),
                   (report.state == RaftCore::ServerStateCandidate) ? CLIColor::Magenta :
                       (report.state == RaftCore::ServerStateLeader) ? CLIColor::Green :
                           CLIColor::Default);

    render_top_int("Last log index",
                   report.last_log_index,
                   CLIColor::Default);

    render_top_int("Commit index",
                   report.commit_index,
                   colorize_if(report.commit_index != report.last_log_index, CLIColor::Magenta));

    render_top_int("Last log term",
                   report.last_log_term,
                   CLIColor::Default);

    render_top_int("Current term",
                   report.current_term,
                   CLIColor::Default);

    render_top_int("Voted for",
                   report.voted_for.get(),
                   CLIColor::Default);

    render_top_str("Since activity",
                   duration_to_string(time_since_last_activity).c_str(),
                   CLIColor::Default);

    render_top_str("Random timeout",
                   duration_to_string(report.randomized_timeout).c_str(),
                   CLIColor::Default);

    render_top_int("Unknown nodes",
                   report.num_unknown_nodes,
                   colorize_if(report.num_unknown_nodes != 0, CLIColor::Magenta));

    render_top_int("Node failures",
                   node->getInternalFailureCount(),
                   colorize_if(node->getInternalFailureCount() != 0, CLIColor::Magenta));

    const bool all_allocated = server.guessIfAllDynamicNodesAreAllocated();
    render_top_str("All allocated",
                   all_allocated ? "Yes": "No",
                   colorize_if(!all_allocated, CLIColor::Magenta));

    // Empty line before the next block
    std::printf("                                     ");
    render_next_event_counter();

    // Followers block
    std::printf("        Followers                    ");
    render_next_event_counter();

    const auto render_followers_state = [&](const char* name,
                                            const std::function<int (std::uint8_t)> value_getter,
                                            const std::function<CLIColor (std::uint8_t)> color_getter)
    {
        std::printf("%-17s", name);
        for (std::uint8_t i = 0; i < 4; i++)
        {
            if (i < (report.cluster_size - 1))
            {
                CLIColorizer colorizer(color_getter(i));
                const auto value = value_getter(i);
                if (value >= 0)
                {
                    std::printf("%-5d", value);
                }
                else
                {
                    std::printf("N/A  ");
                }
            }
            else
            {
                std::printf("     ");
            }
        }
        render_next_event_counter();
    };

    const auto follower_color_getter = [&](std::uint8_t i)
    {
        if (report.state != RaftCore::ServerStateLeader)              { return CLIColor::Default; }
        if (!report.followers[i].node_id.isValid())                   { return CLIColor::Red; }
        if (report.followers[i].match_index != report.last_log_index ||
            report.followers[i].next_index <= report.last_log_index)
        {
            return CLIColor::Magenta;
        }
        return CLIColor::Default;
    };

    render_followers_state("Node ID", [&](std::uint8_t i)
                           {
                               const auto nid = report.followers[i].node_id;
                               return nid.isValid() ? nid.get() : -1;
                           },
                           follower_color_getter);

    render_followers_state("Next index",
                           [&](std::uint8_t i) { return report.followers[i].next_index; },
                           follower_color_getter);

    render_followers_state("Match index",
                           [&](std::uint8_t i) { return report.followers[i].match_index; },
                           follower_color_getter);

    assert(next_relevant_event_index == NumRelevantEvents);     // Ensuring that all events can be printed

    // Separator
    std::printf("--------------------------------------+----------------------------------------\n");

    // Event log
    std::printf("%s\n", EventTracer::RecentEvent::getTableHeader());
    const int num_events_to_render = static_cast<int>(num_rows) -
                                     static_cast<int>(next_relevant_event_index) -
                                     static_cast<int>(NumRowsWithoutEvents) -
                                     1; // This allows to keep the last line empty for stdout or UAVCAN_TRACE()
    for (int i = 0;
         i < num_events_to_render && i < static_cast<int>(event_tracer.getNumEvents());
         i++)
    {
        const auto e = event_tracer.getEventByIndex(i);
        CLIColorizer colorizer(getColorHash(static_cast<unsigned>(e.code)));
        std::printf("%s\n", e.toString().c_str());
    }

    std::fflush(stdout);
}


void runForever(const uavcan_linux::NodePtr& node,
                const std::uint8_t cluster_size,
                const std::string& event_log_file,
                const std::string& persistent_storage_path)
{
    /*
     * Event tracer
     */
    EventTracer event_tracer(MaxNumLastEvents);
    ENFORCE(0 <= event_tracer.init(event_log_file.c_str()));

    /*
     * Storage backend
     */
    uavcan_posix::dynamic_node_id_server::FileStorageBackend storage_backend;
    ENFORCE(0 <= storage_backend.init(persistent_storage_path.c_str()));

    /*
     * Server
     */
    uavcan::dynamic_node_id_server::DistributedServer server(*node, storage_backend, event_tracer);

    const int server_init_res = server.init(node->getNodeStatusProvider().getHardwareVersion().unique_id, cluster_size);
    if (server_init_res < 0)
    {
        throw std::runtime_error("Failed to start the server; error " + std::to_string(server_init_res));
    }

    /*
     * Preparing the CLI
     */
    std::printf("\x1b[2J"); // Clear entire screen; this will preserve initialization output in the scrollback

    /*
     * Spinning the node
     */
    uavcan::MonotonicTime last_redraw_at;

    while (true)
    {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(MinUpdateInterval));
        if (res < 0)
        {
            std::cerr << "Spin error: " << res << std::endl;
        }

        const auto ts = node->getMonotonicTime();

        if (event_tracer.hadEvents() || (ts - last_redraw_at).toMSec() > 1000)
        {
            last_redraw_at = ts;
            redraw(node, ts, event_tracer, server);
        }
    }
}

struct Options
{
    uavcan::NodeID node_id;
    std::vector<std::string> ifaces;
    std::uint8_t cluster_size = 0;
    std::string storage_path;
};

Options parseOptions(int argc, const char** argv)
{
    const char* const executable_name = *argv++;
    argc--;

    const auto enforce = [executable_name](bool condition, const char* error_text) {
        if (!condition)
        {
            std::cerr << error_text << "\n"
                      << "Usage:\n\t"
                      << executable_name
                      << " <node-id> <can-iface-name-1> [can-iface-name-N...] [-c <cluster-size>] -s <storage-path>"
                      << std::endl;
            std::exit(1);
        }
    };

    enforce(argc >= 3, "Not enough arguments");

    /*
     * Node ID is always at the first position
     */
    argc--;
    const int node_id = std::stoi(*argv++);
    enforce(node_id >= 1 && node_id <= 127, "Invalid node ID");

    Options out;
    out.node_id = uavcan::NodeID(std::uint8_t(node_id));

    while (argc --> 0)
    {
        const std::string token(*argv++);

        if (token[0] != '-')
        {
            out.ifaces.push_back(token);
        }
        else if (token[1] == 'c')
        {
            int cluster_size = 0;
            if (token.length() > 2)     // -c2
            {
                cluster_size = std::stoi(token.c_str() + 2);
            }
            else                        // -c 2
            {
                enforce(argc --> 0, "Expected cluster size");
                cluster_size = std::stoi(*argv++);
            }
            enforce(cluster_size >= 1 &&
                    cluster_size <= uavcan::dynamic_node_id_server::distributed::ClusterManager::MaxClusterSize,
                    "Invalid cluster size");
            out.cluster_size = std::uint8_t(cluster_size);
        }
        else if (token[1] == 's')
        {
            if (token.length() > 2)     // -s/foo/bar
            {
                out.storage_path = token.c_str() + 2;
            }
            else                        // -s /foo/bar
            {
                enforce(argc --> 0, "Expected storage path");
                out.storage_path = *argv++;
            }
        }
        else
        {
            enforce(false, "Unexpected argument");
        }
    }

    enforce(!out.storage_path.empty(), "Invalid storage path");

    return out;
}

}

int main(int argc, const char** argv)
{
    try
    {
        std::srand(std::time(nullptr));

        if (isatty(STDOUT_FILENO) != 1)
        {
            std::cerr << "This application cannot run if stdout is not associated with a terminal" << std::endl;
            std::exit(1);
        }

        auto options = parseOptions(argc, argv);

        std::cout << "Self node ID: " << int(options.node_id.get()) << "\n"
                     "Cluster size: " << int(options.cluster_size) << "\n"
                     "Storage path: " << options.storage_path << "\n"
                     "Num ifaces:   " << options.ifaces.size() << "\n"
#ifdef NDEBUG
                     "Build mode:   Release"
#else
                     "Build mode:   Debug"
#endif
                     << std::endl;

        /*
         * Preparing the storage directory
         */
        options.storage_path += "/node_" + std::to_string(options.node_id.get());

        int system_res = std::system(("mkdir -p '" + options.storage_path + "' &>/dev/null").c_str());
        (void)system_res;

        const auto event_log_file = options.storage_path + "/events.log";
        const auto storage_path   = options.storage_path + "/storage/";

        /*
         * Starting the node
         */
        auto node = initNode(options.ifaces, options.node_id, "org.uavcan.linux_app.dynamic_node_id_server");
        runForever(node, options.cluster_size, event_log_file, storage_path);
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
