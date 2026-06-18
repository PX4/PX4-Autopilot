#include <rl_tools/ui_server/server.h>
#include <rl_tools/operations/cpu.h>

#include <CLI/CLI.hpp>

#include <thread>
#include <chrono>
#include <boost/dll.hpp>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

int main(int argc, char* argv[]) {
    using namespace rlt::ui_server;

    using DEVICE = rlt::devices::DefaultCPU;
    DEVICE device;

    CLI::App app;
    uint16_t port = 13337; // default is 13337 so it does not clash with other services at e.g. 8000 or 8888
    std::string ip = "127.0.0.1", static_path_stub = "static/ui_server", simulator = "generic", scenario = "default";
    bool verbose = false;
#if defined(RL_TOOLS_RELEASE_WINDOWS)
    static_path_stub = "../share/rl_tools/static/ui_server";
#elif defined(RL_TOOLS_RELEASE_MACOS)
    static_path_stub = boost::dll::program_location().parent_path().string() + "/../share/rl_tools/static/ui_server";
#elif defined(RL_TOOLS_RELEASE_LINUX)
    static_path_stub = "../share/rl_tools/static/ui_server";
#endif
    app.add_option("--ip", ip, "IP address");
    app.add_option("--port", port, "Port");
    app.add_option("--static", static_path_stub, "path to the static directory");
    app.add_option("--simulator", simulator, "simulator [multirotor, car]");
    app.add_option("--scenario", scenario, "scenario [default, ...]");
    app.add_flag("-v,--verbose", verbose, "Verbose output");

    CLI11_PARSE(app, argc, argv);


    std::string static_path = static_path_stub + "/" + simulator;
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    std::cout << "Static path: " << static_path << std::endl;

    rlt::ui_server::UIServer server;

    rlt::init(device, server, ip, port, static_path, scenario, verbose);

    while(server.running){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}
