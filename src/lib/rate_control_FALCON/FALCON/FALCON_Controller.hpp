#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class FALCON_Controller
{

    public:
        FALCON_Controller(float proportional_gain, float integral_gain, float saturation_positive, float saturation_negative, float integral_limit);

        ~FALCON_Controller() = default;

        virtual float update(float state, float state_setpoint, const float dt, const bool landed) = 0;

        void update_integral(float &rate_error, float dt);

        void log_state() const;


        float _proportional_gain;
        float _integral_gain;
        float _integral_value;
        float _saturation_positive;
        float _saturation_negative;
        float _integral_limit;
        float _rate_int;
        

    private:
        /* data */
        


        
};