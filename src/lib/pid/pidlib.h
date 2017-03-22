#ifndef _PID_H_
#define _PID_H_

class __EXPORT Pid
{
private:
    bool  _param_set;
    float _dt;
    float _max;
    float _min;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;
    float _max_i;


public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    Pid();
    ~Pid(){}

    /**
     * @brief      Returns the manipulated variable given a setpoint and current process value
     *
     * @param[in]  setpoint  The setpoint to track
     * @param[in]  control_feedback        control value to adjust
     *
     * @return     { description_of_the_return_value }
     */
    float calculate( float setpoint, float control_feedback );
    
    /**
     * @brief      updates the gains of the controller
     *
     * @param[in]  dt     delta time
     * @param[in]  max    Maximum output allowed
     * @param[in]  min    minimum value allowed
     * @param[in]  Kp     Proportional gain
     * @param[in]  Kd     Derivative gain
     * @param[in]  Ki     Integrative gain
     * @param[in]  max_i  Maximum value allowed for the integrator (anti windup)
     */
    void update_gains(float time, float a_max, float a_min, float Kp, float Kd, float Ki, float a_max_i );

    void update_dt(float time);

    /***************************** GETTERS ******************************/

    float dt() { return _dt; }
    float max() { return _max;}
    float min() { return _min;}
    float kp() { return _Kp;}
    float kd() { return _Kd;}
    float ki() { return _Ki;}
    float pre_error() { return _pre_error ;}
    float integral() { return _integral; }
    float max_i() { return _max_i; }


    
};

#endif