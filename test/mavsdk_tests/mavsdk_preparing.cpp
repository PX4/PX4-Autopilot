
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param/param.h>
#include <atomic>
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

template<typename Rep>
bool poll_condition_with_timeout(std::function<bool()> fun, std::chrono::duration<Rep> duration)
{
	static constexpr unsigned check_resolution = 100;

	const std::chrono::microseconds duration_us(duration);
	const auto start_time = std::chrono::steady_clock::now();

	while (!fun()) {
		std::this_thread::sleep_for(duration_us / check_resolution);
		const auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
					     start_time);

		if (elapsed_time_us > duration_us) {
			std::cout << "Timeout, waiting for the vehicle for "
				  << elapsed_time_us.count() * std::chrono::steady_clock::period::num
				  / static_cast<double>(std::chrono::steady_clock::period::den)
				  << " seconds\n";
			return false;
		}
	}

	return true;
}

int main(int argc, char **argv)
{
	const std::string arg_url{"--url"};
	const std::string arg_command{"--command"};
	const std::string command_set_param{"set_sys_autostart"};
	const std::string command_check{"check"};
	const std::string command_reboot{"reboot"};

	std::string connection_url{};
	std::string command{};
	int param_value{};
	mavsdk::Mavsdk mavsdk{};

	for (int i = 0; i < argc; ++i) {
		const std::string argv_string(argv[i]);

		if (argv_string == arg_url && (argc > (i + 1))) {
			connection_url = argv[i + 1];
			i++;

		} else if (argv_string == arg_command && (argc > (i + 1))) {
			command = argv[i + 1];
			i++;

			if (command == command_set_param) {
				if ((argc > (i + 1))) {
					param_value = std::atoi(argv[i + 1]);
					i++;

				} else {
					command.erase();
				}
			}
		}

	}

	if (connection_url.empty()) {
		std::cerr << "No connection URL  was supplied" << std::endl;
		return 1;
	}

	std::cout << "Connection url " << connection_url << std::endl;
	ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connect was failed" << std::endl;
		return 1;
	}

	std::cout << "Waiting for system connect" << std::endl;


	auto res_polling = poll_condition_with_timeout([&]() { return mavsdk.systems().size() > 0;}, std::chrono::seconds(25));

	if (not res_polling) {
		std::cerr << "Polling was failed" << std::endl;
		return 1;
	}

	auto system = mavsdk.systems().at(0);

	if (!system) {
		std::cerr << "The system wasn't be found" << std::endl;
		return 1;
	}

	if (command_check == command) {
		std::cout << "Success! The connection was checked." << std::endl;
		return 0;
	}

	if (command_set_param == command) {
		//TODO: For some reason when we set the new value it sometimes takes several attempts to change a parameter.
		int attempt = 1;
		const int max_attemp = 5;
		const auto param_name = "SYS_AUTOSTART";

		for (attempt = 0; attempt <= max_attemp; attempt++) {
			using namespace std::chrono_literals;
			auto param = Param(system);


			if (param.set_param_int(param_name, param_value) != Param::Result::Success) {
				std::cerr << "Fail setting param: " << param_name << ", value: "
					  << param_value << std::endl;
				return 1;
			}

			auto res = param.get_param_int(param_name);

			if (res.first == Param::Result::Success && res.second == param_value) {
				break;
			}

			std::this_thread::sleep_for(1s);
		}

		if (attempt <= max_attemp) {
			std::cout << "Param " << param_name << " was successly set after " << attempt
				  << " attempt. Tne new value " << param_value << std::endl;

		} else {
			std::cerr << "Fail. Setting param: " << param_name << " wasn't set to value: "
				  << param_value << "after " << max_attemp << " attempts. " << std::endl;
			return 1;
		}
	}

	if (command_reboot == command) {
		auto action = Action{system};

		if (Action::Result::Success != action.reboot()) {
			std::cerr << "Reboot doesn't work" << std::endl;
			return 1;
		}

		std::cout << "Rebooting...\n";
	}

	return 0;
}
