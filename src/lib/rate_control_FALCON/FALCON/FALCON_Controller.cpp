#include "FALCON_Controller.hpp"
#include <px4_platform_common/defines.h>
#include <fstream>

FALCON_Controller::FALCON_Controller(float proportional_gain, float integral_gain, float saturation_positive, float saturation_negative, float integral_limit){

    /* std::ifstream file("/home/davis.jeffrey/px4_falcon/PX4-Autopilot/src/lib/rate_control/FALCON/Controller_Config.json");


    if (!file.is_open()) {
        std::cerr << "Error: Could not open Controller_Config.json" << std::endl;
        return;
    }

    _config = nlohmann::json::parse(file);

    _proportional_gain = _config[controller][axis]["proportional_gain"];
    _integral_gain = _config[controller][axis]["integral_gain"];
    _saturation_positive = _config[controller][axis]["saturation_positive"];
    _saturation_negative = _config[controller][axis]["saturation_negative"];
    _integral_limit = _config[controller][axis]["integral_limit"];

    std::cout << "proportional_gain: " << _proportional_gain << std::endl;
    std::cout << "integral_gain: " << _integral_gain << std::endl;
    std::cout << "saturation_positive: " << _saturation_positive << std::endl;
    std::cout << "saturation_negative: " << _saturation_negative << std::endl;
    std::cout << "integral_limit: " << _integral_limit << std::endl;

    file.close(); */

    _proportional_gain = proportional_gain;
    _integral_gain = integral_gain;
    _saturation_positive = saturation_positive;
    _saturation_negative = saturation_negative;
    _integral_limit = integral_limit;
}

void FALCON_Controller::update_integral(float &rate_error, float dt)
{
    // prevent further positive control saturation
    if (_saturation_positive < 0.001f) {
        rate_error = math::min(rate_error, 0.f);
    }

    // prevent further negative control saturation
    if (_saturation_negative < 0.001f) {
        rate_error = math::max(rate_error, 0.f);
    }

    float i_factor = rate_error / math::radians(400.f);
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

    // Perform the integration using a first order method
    float rate_i = _rate_int + i_factor * _integral_gain * rate_error * dt;

    // do not propagate the result if out of range or invalid
    if (PX4_ISFINITE(rate_i)) {
        _rate_int = math::constrain(rate_i, -_integral_limit, _integral_limit);
    }
}

void FALCON_Controller::log_state() const
{

    return;
    /* // 1. Create an output file stream object and open the file.
    // "example.csv" will be created in the directory where the program is run.
    std::ofstream outputFile;
    outputFile.open("example.csv"); // This overwrites the file if it exists by default.

    // Check if the file was opened successfully
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }

    // 2. Write the header row, separating values with commas and ending with a newline.
    outputFile << "Name" << "," << "Age" << "," << "Gender" << "\n";
    
    // 3. Write data rows in a similar fashion.
    outputFile << "John" << "," << 30 << "," << "Male" << "\n";
    outputFile << "Jane" << "," << 25 << "," << "Female" << "\n";
    outputFile << "Bob" << "," << 40 << "," << "Male" << "\n";

    // 4. Close the file.
    outputFile.close();

    std::cout << "Data written to example.csv successfully." << std::endl; */
}

