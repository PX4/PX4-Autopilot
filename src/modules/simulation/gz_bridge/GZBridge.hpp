/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "GZMixingInterfaceESC.hpp"
#include "GZMixingInterfaceServo.hpp"
#include "GZMixingInterfaceWheel.hpp"
#include "GZGimbal.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/scene.pb.h>
// Custom PX4 proto
#include <opticalflow.pb.h>
#include <gz/msgs/int32.pb.h>

using namespace time_literals;

class GZBridge : public ModuleBase<GZBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

    GZBridge(const std::string &world, const std::string &model_name);
    ~GZBridge() override;

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    int init();

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:

    void Run() override;

    bool subscribeClock(bool required);
    bool subscribePoseInfo(bool required);
    bool subscribeImu(bool required);
    bool subscribeMag(bool required);
    bool subscribeOdometry(bool required);
    bool subscribeLaserScan(bool required);
    bool subscribeDistanceSensor(bool required);
    bool subscribeAirspeed(bool required);
    bool subscribeAirPressure(bool required);
    bool subscribeNavsat(bool required);
    bool subscribeOpticalFlow(bool required);

    // Ataques UAVJamSim
    bool subscribeAttacks(bool required);
    void gpsAttackCallback(const gz::msgs::Vector3d &msg);
    void gpsRotAttackCallback(const gz::msgs::Vector3d &msg);
    void imuAttackCallback(const gz::msgs::Vector3d &msg);
    void motorAttackCallback(const gz::msgs::Vector3d &msg);
    void magAttackCallback(const gz::msgs::Vector3d &msg);
    void lidarAttackCallback(const gz::msgs::Vector3d &msg);
    void streamAttackCallback(const gz::msgs::Int32 &msg);
    void sonarAttackCallback(const gz::msgs::Vector3d &msg);
    void baroAttackCallback(const gz::msgs::Vector3d &msg);

    // Ataques de jamming GPS
    void jammingAttackCallback(const gz::msgs::Vector3d &msg);

    // Mitigacao GPS
    void attackMitigationCallback(const gz::msgs::Vector3d &msg);

    void clockCallback(const gz::msgs::Clock &msg);
    void airspeedCallback(const gz::msgs::AirSpeed &msg);
    void airPressureCallback(const gz::msgs::FluidPressure &msg);
    void imuCallback(const gz::msgs::IMU &msg);
    void poseInfoCallback(const gz::msgs::Pose_V &msg);
    void odometryCallback(const gz::msgs::OdometryWithCovariance &msg);
    void navSatCallback(const gz::msgs::NavSat &msg);
    void laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg);
    void laserScanCallback(const gz::msgs::LaserScan &msg);
    void opticalFlowCallback(const px4::msgs::OpticalFlow &msg);
    void magnetometerCallback(const gz::msgs::Magnetometer &msg);

    // Publica pseudo-GPS sintetico durante blackout usando dead-reckoning local.
    void publishBlackoutPseudoGPS(uint64_t timestamp);

    // Pouso unico para qualquer anomalia GPS mitigada.
    void checkGpsAutoLand(uint64_t timestamp);
    void checkGpsAutoDisarm(uint64_t timestamp);
    void publishGpsLandCommand(uint64_t timestamp);
    void publishGpsDisarmCommand(uint64_t timestamp, bool force_disarm);

    static void rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU);

    static float generate_wgn();

    void addGpsNoise(double &latitude, double &longitude, double &altitude,
             float &vel_north, float &vel_east, float &vel_down);

    uORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Publication<distance_sensor_s>          _distance_sensor_pub{ORB_ID(distance_sensor)};
    uORB::Publication<differential_pressure_s>    _differential_pressure_pub{ORB_ID(differential_pressure)};
    uORB::Publication<obstacle_distance_s>        _obstacle_distance_pub{ORB_ID(obstacle_distance)};
    uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
    uORB::Publication<vehicle_attitude_s>         _attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
    uORB::Publication<vehicle_global_position_s>  _gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
    uORB::Publication<vehicle_local_position_s>   _lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};
    uORB::PublicationMulti<sensor_gps_s>          _sensor_gps_pub{ORB_ID(sensor_gps)};
    uORB::PublicationMulti<sensor_baro_s>         _sensor_baro_pub{ORB_ID(sensor_baro)};
    uORB::PublicationMulti<sensor_accel_s>        _sensor_accel_pub{ORB_ID(sensor_accel)};
    uORB::PublicationMulti<sensor_gyro_s>         _sensor_gyro_pub{ORB_ID(sensor_gyro)};
    uORB::PublicationMulti<sensor_mag_s>          _sensor_mag_pub{ORB_ID(sensor_mag)};
    uORB::PublicationMulti<vehicle_odometry_s>    _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
    uORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};
    uORB::Publication<vehicle_command_s>          _vehicle_command_pub{ORB_ID(vehicle_command)};
    uORB::Subscription                            _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
    uORB::Subscription                            _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

    GZMixingInterfaceESC   _mixing_interface_esc{_node};
    GZMixingInterfaceServo _mixing_interface_servo{_node};
    GZMixingInterfaceWheel _mixing_interface_wheel{_node};

    GZGimbal _gimbal{_node};

    MapProjection _pos_ref{};
    double _alt_ref{};

    matrix::Vector3d _position_prev{};
    matrix::Vector3d _velocity_prev{};
    matrix::Quatf _q_nb_prev{1.0f, 0.0f, 0.0f, 0.0f};  // quaternion NED<-body anterior, para calculo correto de velocidade angular
    hrt_abstime _timestamp_prev{};

    const std::string _world_name;
    const std::string _model_name;

    float _temperature{288.15};  // 15 degrees

    bool _realtime_clock_set{false};
    gz::transport::Node _node;

    // Ataques UAVJamSim
    gz::math::Vector3d _gps_attack_offset{0.0, 0.0, 0.0};
    gz::math::Vector3d _gps_attack_rot{0.0, 0.0, 0.0};

    // Ataque de IMU
    bool _imu_attack_enabled{false};
    bool _imu_disabled{false};
    double _imu_temp_offsets[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double _imu_active_offsets[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Ataque de magnetometro
    int _mag_attack_option{0};

    // Ataque de lidar
    int _lidar_attack_option{0};
    double _lidar_distance_offset{0.0};

    // Ataque de stream
    int _stream_attack_option{0};
    gz::transport::Node::Publisher _stream_cmd_pub;

    // Ataque de sonar
    int _sonar_attack_option{0};
    double _sonar_distance_offset{0.0};

    // Ataque de barometro
    int _baro_attack_option{0};
    double _baro_alt_offset{0.0};

    // Ataques de jamming GPS
    int _jamming_attack_type{0}; // 0=OFF, 1=ruido, 2=blackout, 3=pulsado
    float _jamming_intensity{0.0f};

    // Ataque 3: jamming pulsado
    enum class PulsedJamState {
        IDLE = 0,
        RAMP_UP,
        JAM,
        DECAY,
        VALID
    };

    PulsedJamState _pjam_state{PulsedJamState::IDLE};

    uint64_t _pjam_state_enter_us{0};
    uint64_t _pjam_ramp_us{500000ULL};
    uint64_t _pjam_on_us{1000000ULL};
    uint64_t _pjam_decay_us{800000ULL};
    uint64_t _pjam_off_us{2000000ULL};

    float _pjam_noise_factor{0.0f};

    bool _jamming_intermittent_active{false};
    bool _jamming_pulsed_initialized{false};

    // Mitigacao GPS
    bool _attack_mitigation_enabled{false};

    // Mitigacao GPS - Filtro de Kalman com NIS

    // Ruido de processo do filtro
    static constexpr double JMIT_Q_POS        = 0.04;    // [m2/step]
    static constexpr double JMIT_Q_VEL        = 0.25;    // [(m/s)2/step]
    static constexpr double JMIT_Q_ALT        = 0.09;    // [m2/step]

    // Ruido nominal de medicao GPS
    static constexpr double JMIT_R_POS_CLEAN  = 0.64;    // [m2]   ~0.8 m
    static constexpr double JMIT_R_VEL_CLEAN  = 0.0025;  // [(m/s)2]
    static constexpr double JMIT_R_ALT_CLEAN  = 2.56;    // [m2]   ~1.6 m

    // Limiar NIS: 2(1-DOF, p=0.001) = 10.83.
    static constexpr double JMIT_NIS_THRESHOLD = 10.83;

    // Limites de velocidade aceitos para fontes auxiliares de dead-reckoning.
    static constexpr float JMIT_DR_VEL_MAX_M_S   = 15.0f;  // [m/s] horizontal
    static constexpr float JMIT_DR_VEL_Z_MAX_M_S =  5.0f;  // [m/s] vertical

    // Limites de sanidade da odometria visual usada como fonte auxiliar.
    static constexpr float VIO_MAX_POS_VAR_M2 = 25.0f;

    // Covariancia maxima da estimativa
    static constexpr float  JMIT_P_MAX         = 625.0f;  // [m2]   = 25 m

    // ======== MITIGACAO BLACKOUT GPS - CONSTANTES ========
    // Tempo maximo em que a mitigacao mantem a pseudo-medicao GPS durante blackout.
    static constexpr uint64_t BLACKOUT_MAX_HOLD_US = 300000000ULL; // 300 s

    // Qualidade degradada usada durante a pseudo-medicao GPS.
    static constexpr float BLACKOUT_EPH_DEGRADE  = 3.0f; // [m]
    static constexpr float BLACKOUT_EPV_DEGRADE  = 5.0f; // [m]
    static constexpr float BLACKOUT_HDOP_DEGRADE = 1.5f;
    static constexpr float BLACKOUT_VDOP_DEGRADE = 2.5f;

    // Frequencia de publicacao da pseudo-medicao durante blackout (10 Hz).
    static constexpr uint64_t BLACKOUT_PUB_INTERVAL_US = 100000ULL; // 0.1 s

    // Janela maxima para considerar a odometria visual recente durante blackout.
    static constexpr uint64_t BLACKOUT_VIO_TIMEOUT_US = 500000ULL; // 0.5 s

    // Tempo limite usado para identificar ausencia de atualizacoes GPS.
    static constexpr uint64_t BLACKOUT_GPS_TIMEOUT_US = 1500000ULL; // 1.5 s

    // ======== POUSO AUTOMATICO GPS - CONSTANTES ========
    static constexpr float AUTO_LAND_DEST_RADIUS_M = 1.2f;
    static constexpr float AUTO_LAND_MAX_SPEED_M_S = 0.25f;
    static constexpr float AUTO_LAND_ANOMALY_DEST_RADIUS_M = 1.2f;
    static constexpr float AUTO_LAND_ANOMALY_MAX_SPEED_M_S = 0.35f;
    static constexpr float AUTO_LAND_SEVERE_ANOMALY_DEST_RADIUS_M = 2.0f;
    static constexpr float AUTO_LAND_SEVERE_ANOMALY_MAX_SPEED_M_S = 1.0f;
    static constexpr uint64_t AUTO_LAND_STABLE_US = 3000000ULL;
    static constexpr uint64_t AUTO_LAND_PULSE_BRIDGE_US = 6000000ULL;
    static constexpr uint64_t AUTO_LAND_SEVERE_STABLE_US = 3500000ULL;
    static constexpr float AUTO_LAND_FINAL_MAX_SPEED_M_S = 0.70f;
    static constexpr uint64_t AUTO_LAND_FINAL_MAX_WAIT_US = 8000000ULL;
    static constexpr double AUTO_LAND_SEVERE_NIS_THRESHOLD = 50000.0;
    static constexpr uint64_t AUTO_LAND_MIN_BLACKOUT_US = 500000ULL;
    static constexpr uint64_t AUTO_LAND_MIN_NOISE_US = 3000000ULL;
    static constexpr double AUTO_LAND_DESCENT_RATE_M_S = 0.08;
    static constexpr double AUTO_LAND_TARGET_AGL_M = 0.30;
    static constexpr double AUTO_LAND_TARGET_GPS_ANOMALY_AGL_M = 0.25;
    static constexpr uint64_t AUTO_LAND_GROUND_DISARM_MIN_US = 0ULL;
    static constexpr uint64_t AUTO_LAND_GROUND_SENSOR_TIMEOUT_US = 1000000ULL;
    static constexpr float AUTO_LAND_GROUND_DISARM_DIST_M = 0.08f;
    static constexpr double AUTO_LAND_SIM_GROUND_DISARM_AGL_M = 0.08;
    static constexpr float AUTO_LAND_MAX_PITCH_RAD = 0.0872665f;
    static constexpr float AUTO_LAND_MAX_ROLL_RAD = 0.0872665f;
    static constexpr uint64_t AUTO_LAND_ATT_LOG_INTERVAL_US = 500000ULL;

    // Estado do filtro
    double _kf_n_m{0.0};     // Norte estimado [m] relativo a ref.
    double _kf_e_m{0.0};     // Leste estimado [m]
    double _kf_alt{0.0};     // Altitude estimada [m]
    float  _kf_vel_n{0.0f};
    float  _kf_vel_e{0.0f};
    float  _kf_vel_d{0.0f};

    // Covariancias do filtro
    double _kf_P_n{1.0};
    double _kf_P_e{1.0};
    double _kf_P_alt{4.0};
    float  _kf_P_vn{0.1f};
    float  _kf_P_ve{0.1f};
    float  _kf_P_vd{0.1f};

    bool     _kf_initialized{false};
    uint64_t _kf_last_us{0};
    uint64_t _kf_attack_dur_us{0};
    bool     _kf_attack_first{true};

    // Pseudo-medicao GPS publicada durante a mitigacao
    double _jmit_pub_n_m{0.0};
    double _jmit_pub_e_m{0.0};
    double _jmit_pub_alt{0.0};
    float  _jmit_pub_vel_n{0.0f};
    float  _jmit_pub_vel_e{0.0f};
    float  _jmit_pub_vel_d{0.0f};

    // Altitude barometrica auxiliar
    float _jmit_baro_alt_m{0.0f};
    bool _jmit_baro_valid{false};
    uint64_t _jmit_baro_timestamp{0};

    // Alinhamento entre altitude barometrica e altitude MSL
    double _jmit_baro_alt_offset{0.0};
    bool _jmit_baro_alt_offset_valid{false};

    // ======== MITIGACAO BLACKOUT GPS - ESTADO ========
    // Indica se ha uma ancora de blackout inicializada e pronta para publicacao.
    bool     _blackout_anchor_initialized{false};
    // Posicao ancora capturada no inicio do blackout (coordenadas locais [m]).
    double   _blackout_anchor_n_m{0.0};
    double   _blackout_anchor_e_m{0.0};
    double   _blackout_anchor_alt{0.0};
    // Posicao estimada do drone durante o blackout (propagada por dead-reckoning).
    double   _blackout_dr_n_m{0.0};
    double   _blackout_dr_e_m{0.0};
    double   _blackout_dr_alt{0.0};
    // Velocidade no inicio do blackout, usada para o dead-reckoning.
    float    _blackout_dr_vel_n{0.0f};
    float    _blackout_dr_vel_e{0.0f};
    float    _blackout_dr_vel_d{0.0f};
    // Timestamps para controle do dead-reckoning e da frequencia de publicacao.
    uint64_t _blackout_start_us{0};
    uint64_t _blackout_last_pub_us{0};
    uint64_t _blackout_last_dr_us{0};
    bool     _blackout_detected{false};

    // Ultima medicao GPS real publicada. A deteccao de blackout usa timeout deste cache.
    bool     _gps_real_valid{false};
    uint64_t _gps_real_last_us{0};
    double   _gps_real_n_m{0.0};
    double   _gps_real_e_m{0.0};
    double   _gps_real_alt{0.0};
    float    _gps_real_vel_n{0.0f};
    float    _gps_real_vel_e{0.0f};
    float    _gps_real_vel_d{0.0f};

    // Cache de odometria visual usado como fonte primaria durante blackout.
    bool     _vio_valid{false};
    uint64_t _vio_timestamp{0};
    double   _vio_n_m{0.0};
    double   _vio_e_m{0.0};
    double   _vio_d_m{0.0};
    double   _vio_alt_msl{0.0};
    float    _vio_vel_n{0.0f};
    float    _vio_vel_e{0.0f};
    float    _vio_vel_d{0.0f};

    // Posicao local e altitude MSL usadas no criterio de chegada ao destino.
    uORB::Subscription _lpos_ekf2_sub{ORB_ID(vehicle_local_position)};

    // Velocidade horizontal estimada usada no criterio de chegada ao destino.
    float _imu_dr_vel_n{0.0f};
    float _imu_dr_vel_e{0.0f};
    float _imu_dr_vel_d{0.0f};

    // Altitude local convertida para MSL para a pseudo-medicao GPS.
    double   _jmit_lpos_alt_msl{0.0};
    bool     _jmit_lpos_alt_valid{false};
    uint64_t _jmit_lpos_timestamp{0};

    // Posicao local usada para detectar chegada ao destino durante blackout.
    double   _lpos_n_m{0.0};
    double   _lpos_e_m{0.0};
    double   _lpos_alt_msl{0.0};
    bool     _lpos_xy_valid{false};
    bool     _lpos_z_valid{false};
    uint64_t _lpos_timestamp{0};
    float    _lpos_ground_speed{0.0f};
    float    _lpos_heading_rad{0.0f};   // Heading atual do drone [rad NED], atualizado via lpos.

    // Cache do sensor de distancia apontado para baixo.
    float    _ground_distance_m{NAN};
    bool     _ground_distance_valid{false};
    uint64_t _ground_distance_timestamp{0};

    double   _sim_agl_m{NAN};
    bool     _sim_agl_valid{false};
    uint64_t _sim_agl_timestamp{0};

    float    _sim_roll_rad{NAN};
    float    _sim_pitch_rad{NAN};
    bool     _sim_att_valid{false};
    uint64_t _sim_att_timestamp{0};

    // Contador de confirmacoes consecutivas de contato com o solo via AGL simulado.
    // Exige N amostras consecutivas abaixo do limiar antes de autorizar o desarme forcado.
    uint32_t _sim_agl_ground_count{0};
    static constexpr uint32_t SIM_AGL_GROUND_CONFIRM_COUNT = 1;

    // Estado do pouso GPS mitigado.
    bool     _gps_auto_land_sent{false};
    bool     _gps_auto_disarm_sent{false};
    bool     _gps_landing_active{false};
    bool     _gps_land_source_blackout{false};
    bool     _gps_land_source_severe_anomaly{false};
    uint64_t _gps_auto_land_arrival_us{0};
    uint64_t _gps_auto_land_last_arrived_us{0};
    uint64_t _gps_land_hold_start_us{0};
    double   _gps_land_hold_n_m{0.0};
    double   _gps_land_hold_e_m{0.0};
    double   _gps_land_start_alt_msl{0.0};
    double   _gps_land_target_alt_msl{0.0};
    double   _gps_land_last_alt_msl{0.0};
    bool     _gps_land_last_alt_valid{false};
    uint64_t _gps_land_last_update_us{0};
    bool     _gps_anomaly_severe{false};
    uint64_t _gps_anomaly_severe_us{0};

    // Heading capturado no momento da chegada ao destino para uso no comando LAND.
    // Garante que o PX4 receba um heading explicito e pouse sem inclinar.
    float    _gps_land_heading_rad{0.0f};
    bool     _gps_land_heading_valid{false};

    // Modelo de ruido nominal do GPS
    float _gps_pos_noise_n = 0.0f;
    float _gps_pos_noise_e = 0.0f;
    float _gps_pos_noise_d = 0.0f;
    float _gps_vel_noise_n = 0.0f;
    float _gps_vel_noise_e = 0.0f;
    float _gps_vel_noise_d = 0.0f;
    const float _pos_noise_amplitude = 0.8f;    // [m]
    const float _pos_random_walk = 0.01f;
    const float _pos_markov_time = 0.95f;
    const float _vel_noise_amplitude = 0.05f;   // [m/s]
    const float _vel_noise_density = 0.2f;
    const float _vel_markov_time = 0.85f;

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used,
        (ParamInt<px4::params::SIM_GZ_EN_LIDAR>) _sim_gz_en_lidar,
        (ParamInt<px4::params::SIM_GZ_EN_FLOW>) _sim_gz_en_flow,
        (ParamInt<px4::params::SIM_GZ_EN_ASPD>) _sim_gz_en_aspd,
        (ParamInt<px4::params::SIM_GZ_EN_BARO>) _sim_gz_en_baro,
        (ParamInt<px4::params::SIM_GZ_EN_ODOM>) _sim_gz_en_odom,
        (ParamInt<px4::params::SIM_GZ_EN_GPS>) _sim_gz_en_gps
    )
};