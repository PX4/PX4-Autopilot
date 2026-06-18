#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>

#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/td3/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/td3/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

// for testing compile-time overheads
#ifdef NLOHMANN
#include <nlohmann/json.hpp>
#endif
#ifdef CJSON
#include <cjson/cJSON.h>
#endif
#ifdef BOOST_JSON
#include <boost/json.hpp>
#endif
#ifdef CLI11
#include <CLI/CLI.hpp>
#endif
#ifdef CXXOPTS
#include <cxxopts.hpp>
#endif
#ifdef BOOST_OPTS
#include <boost/program_options.hpp>
#endif
#ifdef CARGS
#include <cargs.h>
#endif


namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI STEP_LIMIT = 20000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
};
using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CORE_CONFIG>{
    static constexpr TI NUM_EVALUATION_EPISODES = 100;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

int main(int argc, char** argv) {
#ifdef NLOHMANN
    nlohmann::json j = nlohmann::json::parse("{\"hello\": \"moin\"}");
    std::cout << j["hello"] << std::endl;
    std::string to;
    j["hello"].get_to(to);
    j["test"] = "yo";
    std::cout << j.dump() << std::endl;
#endif
#ifdef CJSON
    cJSON *root = cJSON_Parse("{\"hello\": \"moin\"}");
    cJSON *hello = cJSON_GetObjectItem(root, "hello");
    std::cout << hello->valuestring << std::endl;
    cJSON_AddStringToObject(root, "test", "yo");
    std::cout << cJSON_PrintUnformatted(root) << std::endl;
    cJSON_Delete(root);
#endif
#ifdef BOOST_JSON
    boost::json::value j = boost::json::parse("{\"hello\": \"moin\"}");
    std::cout << j.at("hello").as_string() << std::endl;
    boost::json::object o = j.as_object();
    o["test"] = "yo";
    std::cout << o << std::endl;
#endif
#ifdef CLI11
    {
        TI seed = 0;
        std::string extrack_base_path = "extrack";
        std::string extrack_experiment_path = "extrack-experiment";
        CLI::App app{"rl_zoo"};
        app.add_option("-s,--seed", seed, "seed");
        app.add_option("-e,--extrack", extrack_base_path, "extrack");
        app.add_option("--ee,--extrack-experiment", extrack_experiment_path, "extrack-experiment");
        CLI11_PARSE(app, argc, argv);
    };
#endif
#ifdef CXXOPTS
    {
        cxxopts::Options options("rl_zoo", "RL Zoo");
        options.add_options()
            ("s,seed", "seed", cxxopts::value<TI>()->default_value("0"))
            ("e,extrack", "extrack", cxxopts::value<std::string>()->default_value("extrack"))
            ("ee,extrack-experiment", "extrack-experiment", cxxopts::value<std::string>()->default_value("extrack-experiment"));
        auto result = options.parse(argc, argv);
    }
#endif
#ifdef BOOST_OPTS
    {
        TI seed = 0;
        std::string extrack_base_path = "extrack";
        std::string extrack_experiment_path = "extrack-experiment";
        boost::program_options::options_description desc("Options");
        desc.add_options()
            ("seed,s", boost::program_options::value<TI>(&seed)->default_value(0), "seed")
            ("extrack,e", boost::program_options::value<std::string>(&extrack_base_path)->default_value("extrack"), "extrack")
            ("extrack-experiment,ee", boost::program_options::value<std::string>(&extrack_experiment_path)->default_value("extrack-experiment"), "extrack-experiment");
        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);
    }
#endif

    DEVICE device;
    TI seed = 0;
    if (argc > 1) {
        seed = std::atoi(argv[1]);
    }
    std::cerr << "seed: " << seed << std::endl;
    rlt::malloc(device);
    rlt::init(device);
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
    while(!rlt::step(device, ts)){
        if(ts.step == 5000){
            std::cout << "steppin yourself > callbacks 'n' hooks: " << ts.step << std::endl;
        }
    }
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
    rlt::free(device, device.logger);
#endif
    rlt::free(device);
    return 0;
}
