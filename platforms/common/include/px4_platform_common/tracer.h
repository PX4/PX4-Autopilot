#pragma once

#include "nlohmann/json.hpp"
#include <fstream>
#include <iostream>
#include <mutex>
#include <stacktrace>
#include <tuple>
#include <utility>

#define PX4_TRACER

template <typename F, std::size_t... Indices, typename... Args>
void FunctionImpl(F &&f, std::index_sequence<Indices...>, Args &&...args) {
  (f(Indices, std::forward<Args>(args)), ...);
}


template <typename F, typename... Args, typename... ArgsRes>
std::tuple<ArgsRes...> Function(F &&f, Args &&...args) {
  std::tuple<ArgsRes...> result;

  [&]<auto... Is>(std::index_sequence<Is...>) {
    ([&]() { std::get<Is>(result) = f(Is, args); }(), ...);
  }(std::index_sequence_for<Args...>{});
  return result;
}

class Tracer {
  using json = nlohmann::json;

public:
  inline static Tracer *instance = nullptr;
  inline static std::mutex mutex{};

  static Tracer *get() {
    if (instance != nullptr) return instance;
    const std::lock_guard<std::mutex> lock(mutex);
    if (instance == nullptr) instance = new Tracer();
    return instance;
  }
  Tracer() : ofs("/tmp/tracer.json", std::ios::out) {
    std::ifstream ifs("/tmp/tracer_conf.json");
    conf = json::parse(ifs);
  }

  template <typename... Args> bool check_entry(const json::array_t &tb, Args&&... args) {
  auto result= std::make_tuple([](auto &&x){return false; }(args)...);

  [&]<auto... Is>(std::index_sequence<Is...>) {
    ([&]() { std::get<Is>(result) = 

		 [&]() {
      if (tb.size() <= Is) return true;
      auto v = tb[Is];
      if (v.is_array()) {
        auto varray = v.template get<std::vector<std::remove_reference_t<Args>>>();
        if (varray.size() == 0) return true;
        for (auto x : varray) {
          if (x == args) return true;
        }
        return false;
      } else {
        auto x = v.template get<std::remove_reference_t<Args>>();
        // std::cout << "Trying " <<x << " against " << arg << std::endl;
        return x == args;
      }
			}();
		 
		 }(), ...);
  }(std::index_sequence_for<Args...>{});
    return std::apply([](auto &&...tmp) { return (... && tmp); }, result);
  }

  template <typename... Args> bool filter(const std::string &opname, Args&&... args) {
		if (conf.count(opname) == 0) return false;
    auto tb = conf[opname].template get<json::array_t>();
    return std::any_of(tb.begin(), tb.end(), [&](auto &&e) {
      return this->template check_entry<Args...>(e.template get<json::array_t>(), args...);
    });
  }

  template <typename... Args> void maybe_dump(Args&&... args) {
    if (filter(args...)) dump(args...);
  }

  template <typename... Args> void dump(const std::string &op, Args&&... args) {

    auto bt = std::stacktrace::current();
    json j = { { "op", op },
               { "args", std::tuple<Args...>(args...) },
               { "bt", std::to_string(bt) } };
    auto sx = j.dump(2);

    const std::lock_guard<std::mutex> lock(mutex);
    ofs << sx << "\n+++++++\n";
    ofs.flush();
  }

private:
  json conf;
  std::ofstream ofs;
};
