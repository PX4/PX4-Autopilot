#include <rl_tools/operations/cpu.h>
#include <rl_tools/rl/environments/l2f/operations_generic.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <filesystem>
namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI = DEVICE::index_t;
using T = double;
constexpr bool DYNAMIC_ALLOCATION = true;

using PARAMETER_FACTORY = rlt::rl::environments::l2f::parameters::DEFAULT_PARAMETERS_FACTORY<T, TI, rlt::rl::environments::l2f::parameters::DEFAULT_DOMAIN_RANDOMIZATION_OPTIONS<true>>;
using ENVIRONMENT = rlt::rl::environments::Multirotor<rlt::rl::environments::l2f::Specification<T, TI,  PARAMETER_FACTORY::STATIC_PARAMETERS>>;

int main(int argc, char** argv){
    DEVICE device;
    rlt::init(device);
    RNG rng;
    rlt::malloc(device, rng);
    TI seed = 0;
    TI N = 1000;
    rlt::init(device, rng, seed);
    ENVIRONMENT env;
    ENVIRONMENT::Parameters params;
    rlt::init(device, env);

    std::filesystem::path output_path_registry = "./static/l2f-studio/blob/registry";
    if (!std::filesystem::exists(output_path_registry)){
        std::cerr << "Output path does not exist: " << output_path_registry << std::endl;
        std::cerr << "CWD: " << std::filesystem::current_path() << std::endl;
        return 1;
    }
    std::vector<std::tuple<std::string, std::string, ENVIRONMENT::Parameters::Dynamics>> registry;
    auto permute_rotors_px4_to_cf = [&device, &env](const auto& dynamics){
        auto copy = dynamics;
        rlt::permute_rotors(device, env, copy, 0, 3, 1, 2);
        return copy;
    };
    registry.emplace_back("crazyflie" , "b75f5120e17783744a8fac5e1ab69c2dce10f0e3", rlt::rl::environments::l2f::parameters::dynamics::crazyflie<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>);
    registry.emplace_back("x500"      , "9602ffc2ffb77f62c4cf6fdc78fe67d32088870d",  permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::x500::real<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));
    registry.emplace_back("mrs"       , "", permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::mrs<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));
    registry.emplace_back("fs"        , "", permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::fs::base<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));
    registry.emplace_back("arpl"      , "775ba8559aeed800dbcdab93806601e39d84fede", permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::arpl<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));
    registry.emplace_back("flightmare", "", permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::flightmare<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));
    registry.emplace_back("soft"      , "", permute_rotors_px4_to_cf(rlt::rl::environments::l2f::parameters::dynamics::soft<ENVIRONMENT::SPEC::T, ENVIRONMENT::SPEC::TI>));

    rlt::initial_parameters(device, env, params);
    std::ofstream index(output_path_registry / ("index.json"));
    for (const auto& [name, model_hash, dynamics] : registry){
        params.dynamics = dynamics;
        std::ofstream output(output_path_registry / (name + ".json"));
        auto params_copy = params;
        std::string json_string = rlt::json(device, env, params_copy);
        if (model_hash != ""){
            nlohmann::json json_ui;
            json_ui["enable"] = true;
            json_ui["name"] = name;
            json_ui["model"] = model_hash;
            auto json_par = nlohmann::json::parse(json_string);
            json_par["ui"] = json_ui;
            json_string = json_par.dump();
        }
        output << json_string;
        output.close();
        index << name << std::endl;
    }
    index.close();
}
