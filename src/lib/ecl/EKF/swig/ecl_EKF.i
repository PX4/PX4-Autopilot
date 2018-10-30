// SWIG Wrapper for the ecl's EKF
%module(directors="1") ecl_EKF
%feature("autodoc", "3");

%include "inttypes.i"
%include "std_vector.i"
%include "std_string.i"
%include "typemaps.i"

// Include headers in the SWIG generated C++ file
%{
    #define SWIG_FILE_WITH_INIT
    #include <iostream>
    #include <EKF/ekf.h>
    #include <geo/geo.h>
%}

%include "numpy.i"
%init %{
    import_array();
%}

%apply (float ARGOUT_ARRAY1[ANY]) {(float out[3])};
%apply (float ARGOUT_ARRAY1[ANY]) {(float bias[3])};
%apply (float ARGOUT_ARRAY1[ANY]) {(float out[24])};
%apply (float IN_ARRAY1[ANY]) {(float delta_ang[3]), (float delta_vel[3])};
%apply (float IN_ARRAY1[ANY]) {(float mag_data[3])};

%inline {
    struct ekf_control_mode_flags_t {
        bool tilt_align; // 0 - true if the filter tilt alignment is complete
        bool yaw_align; // 1 - true if the filter yaw alignment is complete
        bool gps; // 2 - true if GPS measurements are being fused
        bool opt_flow; // 3 - true if optical flow measurements are being fused
        bool mag_hdg; // 4 - true if a simple magnetic yaw heading is being fused
        bool mag_3D; // 5 - true if 3-axis magnetometer measurement are being fused
        bool mag_dec; // 6 - true if synthetic magnetic declination measurements are being fused
        bool in_air; // 7 - true when the vehicle is airborne
        bool wind; // 8 - true when wind velocity is being estimated
        bool baro_hgt; // 9 - true when baro height is being fused as a primary height reference
        bool rng_hgt; // 10 - true when range finder height is being fused as a primary height reference
        bool gps_hgt; // 11 - true when GPS height is being fused as a primary height reference
        bool ev_pos; // 12 - true when local position data from external vision is being fused
        bool ev_yaw; // 13 - true when yaw data from external vision measurements is being fused
        bool ev_hgt; // 14 - true when height data from external vision measurements is being fused
        bool fuse_beta; // 15 - true when synthetic sideslip measurements are being fused
        bool update_mag_states_only; // 16 - true when only the magnetometer states are updated by the magnetometer
        bool fixed_wing; // 17 - true when the vehicle is operating as a fixed wing vehicle
        std::string __repr__() {
            std::stringstream ss;
            ss << "[tilt_align: " << tilt_align << "\n";
            ss << " yaw_align: " << yaw_align << "\n";
            ss << " gps: " << gps << "\n";
            ss << " opt_flow: " << opt_flow << "\n";
            ss << " mag_hdg: " << mag_hdg << "\n";
            ss << " mag_3D: " << mag_3D << "\n";
            ss << " mag_dec: " << mag_dec << "\n";
            ss << " in_air: " << in_air << "\n";
            ss << " wind: " << wind << "\n";
            ss << " baro_hgt: " << baro_hgt << "\n";
            ss << " rng_hgt: " << rng_hgt << "\n";
            ss << " gps_hgt: " << gps_hgt << "\n";
            ss << " ev_pos: " << ev_pos << "\n";
            ss << " ev_yaw: " << ev_yaw << "\n";
            ss << " ev_hgt: " << ev_hgt << "\n";
            ss << " fuse_beta: " << fuse_beta << "\n";
            ss << " update_mag_states_only: " << update_mag_states_only << "\n";
            ss << " fixed_wing: " << fixed_wing << "]\n";
            return std::string(ss.str());
        }
    };

    struct ekf_fault_status_flags_t {
        bool bad_mag_x;	// 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
        bool bad_mag_y;	// 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
        bool bad_mag_z;	// 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
	bool bad_hdg;	// 3 - true if the fusion of the heading angle has encountered a numerical error
        bool bad_mag_decl;	// 4 - true if the fusion of the magnetic declination has encountered a numerical error
        bool bad_airspeed;	// 5 - true if fusion of the airspeed has encountered a numerical error
        bool bad_sideslip;	// 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
        bool bad_optflow_X;	// 7 - true if fusion of the optical flow X axis has encountered a numerical error
        bool bad_optflow_Y;	// 8 - true if fusion of the optical flow Y axis has encountered a numerical error
        bool bad_vel_N;	// 9 - true if fusion of the North velocity has encountered a numerical error
        bool bad_vel_E;	// 10 - true if fusion of the East velocity has encountered a numerical error
        bool bad_vel_D;	// 11 - true if fusion of the Down velocity has encountered a numerical error
        bool bad_pos_N;	// 12 - true if fusion of the North position has encountered a numerical error
        bool bad_pos_E;	// 13 - true if fusion of the East position has encountered a numerical error
        bool bad_pos_D;	// 14 - true if fusion of the Down position has encountered a numerical error
        bool bad_acc_bias;	// 15 - true if bad delta velocity bias estimates have been detected
        std::string __repr__() {
            std::stringstream ss;
            ss << "[bad_mag_x: " << bad_mag_x << "\n";
            ss << " bad_mag_y: " << bad_mag_y << "\n";
            ss << " bad_mag_z: " << bad_mag_z << "\n";
	    ss << " bad_hdg: " << bad_hdg << "\n";
            ss << " bad_mag_decl: " << bad_mag_decl << "\n";
            ss << " bad_airspeed: " << bad_airspeed << "\n";
            ss << " bad_sideslip: " << bad_sideslip << "\n";
            ss << " bad_optflow_X: " << bad_optflow_X << "\n";
            ss << " bad_optflow_Y: " << bad_optflow_Y << "\n";
            ss << " bad_vel_N: " << bad_vel_N << "\n";
            ss << " bad_vel_E: " << bad_vel_E << "\n";
            ss << " bad_vel_D: " << bad_vel_D << "\n";
            ss << " bad_pos_N: " << bad_pos_N << "\n";
            ss << " bad_pos_E: " << bad_pos_E << "\n";
            ss << " bad_pos_D: " << bad_pos_D << "\n";
            ss << " bad_acc_bias: " << bad_acc_bias << "]\n";
            return std::string(ss.str());
        }
    };

    struct ekf_imu_sample_t {
        float delta_ang_x;	// delta angle in body frame (integrated gyro measurements)
        float delta_ang_y;	// delta angle in body frame (integrated gyro measurements)
        float delta_ang_z;	// delta angle in body frame (integrated gyro measurements)
        float delta_vel_x;	// delta velocity in body frame (integrated accelerometer measurements)
        float delta_vel_y;	// delta velocity in body frame (integrated accelerometer measurements)
        float delta_vel_z;	// delta velocity in body frame (integrated accelerometer measurements)
        float delta_ang_dt;	// delta angle integration period in seconds
        float delta_vel_dt;	// delta velocity integration period in seconds
        uint64_t time_us;	// timestamp in microseconds
        std::string __repr__() {
            std::stringstream ss;
            ss << "[delta_ang_x: " << delta_ang_x << "\n";
            ss << " delta_ang_y: " << delta_ang_y << "\n";
            ss << " delta_ang_z: " << delta_ang_z << "\n";
            ss << " delta_vel_x: " << delta_vel_x << "\n";
            ss << " delta_vel_y: " << delta_vel_y << "\n";
            ss << " delta_vel_z: " << delta_vel_z << "\n";
            ss << " delta_ang_dt: " << delta_ang_dt << "\n";
            ss << " delta_vel_dt: " << delta_vel_dt << "\n";
            ss << " time_us: " << time_us << "]\n";
            return std::string(ss.str());
        }
    };

    static float last_mag_data[3];
    static float last_imu_delta_ang[3];
    static float last_imu_delta_vel[3];

    const float one_g = CONSTANTS_ONE_G;
}

// Tell swig to wrap ecl classes
%include <matrix/Vector3.hpp>
%include <matrix/Vector2.hpp>
%include <matrix/Quaternion.hpp>
%include <matrix/Dcm.hpp>
%include <matrix/Euler.hpp>
%include <matrix/SquareMatrix.hpp>
%include <matrix/helper_functions.hpp>
%include <EKF/common.h>
%include <EKF/estimator_interface.h>
%include <EKF/ekf.h>

%extend Ekf {
    void set_imu_data(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt,  float delta_ang[3], float delta_vel[3]) {
        for (int i = 0; i < 3; ++i) {
            last_imu_delta_ang[i] = delta_ang[i];
            last_imu_delta_vel[i] = delta_vel[i];
        }
        self->setIMUData(time_usec, delta_ang_dt, delta_vel_dt,  last_imu_delta_ang, last_imu_delta_vel);
    }

    void set_mag_data(uint64_t time_usec, float mag_data[3]) {
        for (int i = 0; i < 3; ++i) {
            last_mag_data[i] = mag_data[i];
        }
        self->setMagData(time_usec, last_mag_data);
    }

    void set_baro_data(uint64_t time_usec, float baro_data) {
        self->setBaroData(time_usec, baro_data);
    }

    %rename (get_control_mode) get_control_mode_;
    ekf_control_mode_flags_t get_control_mode_() {
        filter_control_status_u result_union;
        self->get_control_mode(&result_union.value);

        ekf_control_mode_flags_t result;
        result.tilt_align = result_union.flags.tilt_align; // 0 - true if the filter tilt alignment is complete
        result.yaw_align = result_union.flags.yaw_align; // 1 - true if the filter yaw alignment is complete
        result.gps = result_union.flags.gps; // 2 - true if GPS measurements are being fused
        result.opt_flow = result_union.flags.opt_flow; // 3 - true if optical flow measurements are being fused
        result.mag_hdg = result_union.flags.mag_hdg; // 4 - true if a simple magnetic yaw heading is being fused
        result.mag_3D = result_union.flags.mag_3D; // 5 - true if 3-axis magnetometer measurement are being fused
        result.mag_dec = result_union.flags.mag_dec; // 6 - true if synthetic magnetic declination measurements are being fused
        result.in_air = result_union.flags.in_air; // 7 - true when the vehicle is airborne
        result.wind = result_union.flags.wind; // 8 - true when wind velocity is being estimated
        result.baro_hgt = result_union.flags.baro_hgt; // 9 - true when baro height is being fused as a primary height reference
        result.rng_hgt = result_union.flags.rng_hgt; // 10 - true when range finder height is being fused as a primary height reference
        result.gps_hgt = result_union.flags.gps_hgt; // 11 - true when GPS height is being fused as a primary height reference
        result.ev_pos = result_union.flags.ev_pos; // 12 - true when local position data from external vision is being fused
        result.ev_yaw = result_union.flags.ev_yaw; // 13 - true when yaw data from external vision measurements is being fused
        result.ev_hgt = result_union.flags.ev_hgt; // 14 - true when height data from external vision measurements is being fused
        result.fuse_beta = result_union.flags.fuse_beta; // 15 - true when synthetic sideslip measurements are being fused
        result.update_mag_states_only = result_union.flags.update_mag_states_only; // 16 - true when only the magnetometer states are updated by the magnetometer
        result.fixed_wing = result_union.flags.fixed_wing; // 17 - true when the vehicle is operating as a fixed wing vehicle
        return result;
    }

   %rename (get_filter_fault_status) get_filter_fault_status_;
   ekf_fault_status_flags_t get_filter_fault_status_() {
        fault_status_u result_union;
        self->get_filter_fault_status(&result_union.value);

        ekf_fault_status_flags_t result;
        result.bad_mag_x = result_union.flags.bad_mag_x;
        result.bad_mag_y = result_union.flags.bad_mag_y;
        result.bad_mag_z = result_union.flags.bad_mag_z;
	result.bad_hdg = result_union.flags.bad_hdg;
        result.bad_mag_decl = result_union.flags.bad_mag_decl;
        result.bad_airspeed = result_union.flags.bad_airspeed;
        result.bad_sideslip = result_union.flags.bad_sideslip;
        result.bad_optflow_X = result_union.flags.bad_optflow_X;
        result.bad_optflow_Y = result_union.flags.bad_optflow_Y;
        result.bad_vel_N = result_union.flags.bad_vel_N;
        result.bad_vel_E = result_union.flags.bad_vel_E;
        result.bad_vel_D = result_union.flags.bad_vel_D;
        result.bad_pos_N = result_union.flags.bad_pos_N;
        result.bad_pos_E = result_union.flags.bad_pos_E;
        result.bad_pos_D = result_union.flags.bad_pos_D;
        result.bad_acc_bias = result_union.flags.bad_acc_bias;
        return result;
   }

   %rename (get_imu_sample_delayed) get_imu_sample_delayed_;
   ekf_imu_sample_t get_imu_sample_delayed_() {
       imuSample result_sample = self->get_imu_sample_delayed();
       ekf_imu_sample_t result;
       result.delta_ang_x = result_sample.delta_ang(0);
       result.delta_ang_y = result_sample.delta_ang(1);
       result.delta_ang_z = result_sample.delta_ang(2);
       result.delta_vel_x = result_sample.delta_vel(0);
       result.delta_vel_y = result_sample.delta_vel(1);
       result.delta_vel_z = result_sample.delta_vel(2);
       result.delta_ang_dt = result_sample.delta_ang_dt;
       result.delta_vel_dt = result_sample.delta_vel_dt;
       result.time_us = result_sample.time_us;
       return result;
   }

   %rename (get_position) get_position_;
   void get_position_(float out[3]) {
       return self->get_position(out);
   };

   %rename (get_velocity) get_velocity_;
   void get_velocity_(float out[3]) {
       return self->get_velocity(out);
   };

   %rename (get_state_delayed) get_state_delayed_;
   void get_state_delayed_(float out[24]) {
       return self->get_state_delayed(out);
   }
   void get_quaternion(float out[4]) {
       return self->copy_quaternion(out);
   }
}

// Let SWIG instantiate vector templates
%template(vector_int) std::vector<int>;
%template(vector_double) std::vector<double>;
%template(vector_float) std::vector<float>;
