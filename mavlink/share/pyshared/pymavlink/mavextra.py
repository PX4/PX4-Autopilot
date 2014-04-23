#!/usr/bin/env python
'''
useful extra functions for use by mavlink clients

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import os, sys
from math import *

try:
    # rotmat doesn't work on Python3.2 yet
    from rotmat import Vector3, Matrix3
except Exception:
    pass


def kmh(mps):
    '''convert m/s to Km/h'''
    return mps*3.6

def altitude(SCALED_PRESSURE, ground_pressure=None, ground_temp=None):
    '''calculate barometric altitude'''
    from pymavlink import mavutil
    self = mavutil.mavfile_global
    if ground_pressure is None:
        if self.param('GND_ABS_PRESS', None) is None:
            return 0
        ground_pressure = self.param('GND_ABS_PRESS', 1)
    if ground_temp is None:
        ground_temp = self.param('GND_TEMP', 0)
    scaling = ground_pressure / (SCALED_PRESSURE.press_abs*100.0)
    temp = ground_temp + 273.15
    return log(scaling) * temp * 29271.267 * 0.001

def altitude2(SCALED_PRESSURE, ground_pressure=None, ground_temp=None):
    '''calculate barometric altitude'''
    from pymavlink import mavutil
    self = mavutil.mavfile_global
    if ground_pressure is None:
        if self.param('GND_ABS_PRESS', None) is None:
            return 0
        ground_pressure = self.param('GND_ABS_PRESS', 1)
    if ground_temp is None:
        ground_temp = self.param('GND_TEMP', 0)
    scaling = SCALED_PRESSURE.press_abs*100.0 / ground_pressure
    temp = ground_temp + 273.15
    return 153.8462 * temp * (1.0 - exp(0.190259 * log(scaling)))

def mag_heading(RAW_IMU, ATTITUDE, declination=None, SENSOR_OFFSETS=None, ofs=None):
    '''calculate heading from raw magnetometer'''
    if declination is None:
        import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z

    # go via a DCM matrix to match the APM calculation
    dcm_matrix = rotation(ATTITUDE)
    cos_pitch_sq = 1.0-(dcm_matrix.c.x*dcm_matrix.c.x)
    headY = mag_y * dcm_matrix.c.z - mag_z * dcm_matrix.c.y
    headX = mag_x * cos_pitch_sq - dcm_matrix.c.x * (mag_y * dcm_matrix.c.y + mag_z * dcm_matrix.c.z)

    heading = degrees(atan2(-headY,headX)) + declination
    if heading < 0:
        heading += 360
    return heading

def mag_heading_motors(RAW_IMU, ATTITUDE, declination, SENSOR_OFFSETS, ofs, SERVO_OUTPUT_RAW, motor_ofs):
    '''calculate heading from raw magnetometer'''
    ofs = get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs)

    if declination is None:
        import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z

    headX = mag_x*cos(ATTITUDE.pitch) + mag_y*sin(ATTITUDE.roll)*sin(ATTITUDE.pitch) + mag_z*cos(ATTITUDE.roll)*sin(ATTITUDE.pitch)
    headY = mag_y*cos(ATTITUDE.roll) - mag_z*sin(ATTITUDE.roll)
    heading = degrees(atan2(-headY,headX)) + declination
    if heading < 0:
        heading += 360
    return heading

def mag_field(RAW_IMU, SENSOR_OFFSETS=None, ofs=None):
    '''calculate magnetic field strength from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs):
    '''calculate magnetic field strength from raw magnetometer'''
    import mavutil
    self = mavutil.mavfile_global

    m = SERVO_OUTPUT_RAW
    motor_pwm = m.servo1_raw + m.servo2_raw + m.servo3_raw + m.servo4_raw
    motor_pwm *= 0.25
    rc3_min = self.param('RC3_MIN', 1100)
    rc3_max = self.param('RC3_MAX', 1900)
    motor = (motor_pwm - rc3_min) / (rc3_max - rc3_min)
    if motor > 1.0:
        motor = 1.0
    if motor < 0.0:
        motor = 0.0

    motor_offsets0 = motor_ofs[0] * motor
    motor_offsets1 = motor_ofs[1] * motor
    motor_offsets2 = motor_ofs[2] * motor
    ofs = (ofs[0] + motor_offsets0, ofs[1] + motor_offsets1, ofs[2] + motor_offsets2)

    return ofs

def mag_field_motors(RAW_IMU, SENSOR_OFFSETS, ofs, SERVO_OUTPUT_RAW, motor_ofs):
    '''calculate magnetic field strength from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag

    ofs = get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs)

    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def angle_diff(angle1, angle2):
    '''show the difference between two angles in degrees'''
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

average_data = {}

def average(var, key, N):
    '''average over N points'''
    global average_data
    if not key in average_data:
        average_data[key] = [var]*N
        return var
    average_data[key].pop(0)
    average_data[key].append(var)
    return sum(average_data[key])/N

derivative_data = {}

def second_derivative_5(var, key):
    '''5 point 2nd derivative'''
    global derivative_data
    import mavutil
    tnow = mavutil.mavfile_global.timestamp

    if not key in derivative_data:
        derivative_data[key] = (tnow, [var]*5)
        return 0
    (last_time, data) = derivative_data[key]
    data.pop(0)
    data.append(var)
    derivative_data[key] = (tnow, data)
    h = (tnow - last_time)
    # N=5 2nd derivative from
    # http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    ret = ((data[4] + data[0]) - 2*data[2]) / (4*h**2)
    return ret

def second_derivative_9(var, key):
    '''9 point 2nd derivative'''
    global derivative_data
    import mavutil
    tnow = mavutil.mavfile_global.timestamp

    if not key in derivative_data:
        derivative_data[key] = (tnow, [var]*9)
        return 0
    (last_time, data) = derivative_data[key]
    data.pop(0)
    data.append(var)
    derivative_data[key] = (tnow, data)
    h = (tnow - last_time)
    # N=5 2nd derivative from
    # http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    f = data
    ret = ((f[8] + f[0]) + 4*(f[7] + f[1]) + 4*(f[6]+f[2]) - 4*(f[5]+f[3]) - 10*f[4])/(64*h**2)
    return ret

lowpass_data = {}

def lowpass(var, key, factor):
    '''a simple lowpass filter'''
    global lowpass_data
    if not key in lowpass_data:
        lowpass_data[key] = var
    else:
        lowpass_data[key] = factor*lowpass_data[key] + (1.0 - factor)*var
    return lowpass_data[key]

last_diff = {}

def diff(var, key):
    '''calculate differences between values'''
    global last_diff
    ret = 0
    if not key in last_diff:
        last_diff[key] = var
        return 0
    ret = var - last_diff[key]
    last_diff[key] = var
    return ret

last_delta = {}

def delta(var, key, tusec=None):
    '''calculate slope'''
    global last_delta
    if tusec is not None:
        tnow = tusec * 1.0e-6
    else:
        import mavutil
        tnow = mavutil.mavfile_global.timestamp
    dv = 0
    ret = 0
    if key in last_delta:
        (last_v, last_t, last_ret) = last_delta[key]
        if last_t == tnow:
            return last_ret
        if tnow == last_t:
            ret = 0
        else:
            ret = (var - last_v) / (tnow - last_t)
    last_delta[key] = (var, tnow, ret)
    return ret

def delta_angle(var, key, tusec=None):
    '''calculate slope of an angle'''
    global last_delta
    if tusec is not None:
        tnow = tusec * 1.0e-6
    else:
        import mavutil
        tnow = mavutil.mavfile_global.timestamp
    dv = 0
    ret = 0
    if key in last_delta:
        (last_v, last_t, last_ret) = last_delta[key]
        if last_t == tnow:
            return last_ret
        if tnow == last_t:
            ret = 0
        else:
            dv = var - last_v
            if dv > 180:
                dv -= 360
            if dv < -180:
                dv += 360
            ret = dv / (tnow - last_t)
    last_delta[key] = (var, tnow, ret)
    return ret

def roll_estimate(RAW_IMU,GPS_RAW_INT=None,ATTITUDE=None,SENSOR_OFFSETS=None, ofs=None, mul=None,smooth=0.7):
    '''estimate roll from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if ATTITUDE is not None and GPS_RAW_INT is not None:
        ry -= ATTITUDE.yawspeed * GPS_RAW_INT.vel*0.01
        rz += ATTITUDE.pitchspeed * GPS_RAW_INT.vel*0.01
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(-asin(ry/sqrt(rx**2+ry**2+rz**2))),'_roll',smooth)

def pitch_estimate(RAW_IMU, GPS_RAW_INT=None,ATTITUDE=None, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if ATTITUDE is not None and GPS_RAW_INT is not None:
        ry -= ATTITUDE.yawspeed * GPS_RAW_INT.vel*0.01
        rz += ATTITUDE.pitchspeed * GPS_RAW_INT.vel*0.01
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(asin(rx/sqrt(rx**2+ry**2+rz**2))),'_pitch',smooth)

def rotation(ATTITUDE):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(ATTITUDE.roll, ATTITUDE.pitch, ATTITUDE.yaw)
    return r

def mag_rotation(RAW_IMU, inclination, declination):
    '''return an attitude rotation matrix that is consistent with the current mag
       vector'''
    m_body = Vector3(RAW_IMU.xmag, RAW_IMU.ymag, RAW_IMU.zmag)
    m_earth = Vector3(m_body.length(), 0, 0)

    r = Matrix3()
    r.from_euler(0, -radians(inclination), radians(declination))
    m_earth = r * m_earth

    r.from_two_vectors(m_earth, m_body)
    return r

def mag_yaw(RAW_IMU, inclination, declination):
    '''estimate yaw from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    y = degrees(y)
    if y < 0:
        y += 360
    return y

def mag_pitch(RAW_IMU, inclination, declination):
    '''estimate pithc from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    return degrees(p)

def mag_roll(RAW_IMU, inclination, declination):
    '''estimate roll from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    return degrees(r)

def expected_mag(RAW_IMU, ATTITUDE, inclination, declination):
    '''return expected mag vector'''
    m_body = Vector3(RAW_IMU.xmag, RAW_IMU.ymag, RAW_IMU.zmag)
    field_strength = m_body.length()

    m = rotation(ATTITUDE)

    r = Matrix3()
    r.from_euler(0, -radians(inclination), radians(declination))
    m_earth = r * Vector3(field_strength, 0, 0)

    return m.transposed() * m_earth

def mag_discrepancy(RAW_IMU, ATTITUDE, inclination, declination=None):
    '''give the magnitude of the discrepancy between observed and expected magnetic field'''
    if declination is None:
        import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    expected = expected_mag(RAW_IMU, ATTITUDE, inclination, declination)
    mag = Vector3(RAW_IMU.xmag, RAW_IMU.ymag, RAW_IMU.zmag)
    return degrees(expected.angle(mag))


def mag_inclination(RAW_IMU, ATTITUDE, declination=None):
    '''give the magnitude of the discrepancy between observed and expected magnetic field'''
    if declination is None:
        import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    r = rotation(ATTITUDE)
    mag1 = Vector3(RAW_IMU.xmag, RAW_IMU.ymag, RAW_IMU.zmag)
    mag1 = r * mag1
    mag2 = Vector3(cos(radians(declination)), sin(radians(declination)), 0)
    inclination = degrees(mag1.angle(mag2))
    if RAW_IMU.zmag < 0:
        inclination = -inclination
    return inclination

def expected_magx(RAW_IMU, ATTITUDE, inclination, declination):
    '''estimate  from mag'''
    v = expected_mag(RAW_IMU, ATTITUDE, inclination, declination)
    return v.x

def expected_magy(RAW_IMU, ATTITUDE, inclination, declination):
    '''estimate  from mag'''
    v = expected_mag(RAW_IMU, ATTITUDE, inclination, declination)
    return v.y

def expected_magz(RAW_IMU, ATTITUDE, inclination, declination):
    '''estimate  from mag'''
    v = expected_mag(RAW_IMU, ATTITUDE, inclination, declination)
    return v.z

def gravity(RAW_IMU, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(sqrt(rx**2+ry**2+rz**2),'_gravity',smooth)



def pitch_sim(SIMSTATE, GPS_RAW):
    '''estimate pitch from SIMSTATE accels'''
    xacc = SIMSTATE.xacc - lowpass(delta(GPS_RAW.v,"v")*6.6, "v", 0.9)
    zacc = SIMSTATE.zacc
    zacc += SIMSTATE.ygyro * GPS_RAW.v;
    if xacc/zacc >= 1:
        return 0
    if xacc/zacc <= -1:
        return -0
    return degrees(-asin(xacc/zacc))

def distance_two(GPS_RAW1, GPS_RAW2):
    '''distance between two points'''
    if hasattr(GPS_RAW1, 'Lat'):
        lat1 = radians(GPS_RAW1.Lat)
        lat2 = radians(GPS_RAW2.Lat)
        lon1 = radians(GPS_RAW1.Lng)
        lon2 = radians(GPS_RAW2.Lng)
    elif hasattr(GPS_RAW1, 'cog'):
        lat1 = radians(GPS_RAW1.lat)*1.0e-7
        lat2 = radians(GPS_RAW2.lat)*1.0e-7
        lon1 = radians(GPS_RAW1.lon)*1.0e-7
        lon2 = radians(GPS_RAW2.lon)*1.0e-7
    else:
        lat1 = radians(GPS_RAW1.lat)
        lat2 = radians(GPS_RAW2.lat)
        lon1 = radians(GPS_RAW1.lon)
        lon2 = radians(GPS_RAW2.lon)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
    return 6371 * 1000 * c


first_fix = None

def distance_home(GPS_RAW):
    '''distance from first fix point'''
    global first_fix
    if GPS_RAW.fix_type < 2:
        return 0
    if first_fix == None or first_fix.fix_type < 2:
        first_fix = GPS_RAW
        return 0
    return distance_two(GPS_RAW, first_fix)

def sawtooth(ATTITUDE, amplitude=2.0, period=5.0):
    '''sawtooth pattern based on uptime'''
    mins = (ATTITUDE.usec * 1.0e-6)/60
    p = fmod(mins, period*2)
    if p < period:
        return amplitude * (p/period)
    return amplitude * (period - (p-period))/period

def rate_of_turn(speed, bank):
    '''return expected rate of turn in degrees/s for given speed in m/s and
       bank angle in degrees'''
    if abs(speed) < 2 or abs(bank) > 80:
        return 0
    ret = degrees(9.81*tan(radians(bank))/speed)
    return ret

def wingloading(bank):
    '''return expected wing loading factor for a bank angle in radians'''
    return 1.0/cos(bank)

def airspeed(VFR_HUD, ratio=None, used_ratio=None):
    '''recompute airspeed with a different ARSPD_RATIO'''
    import mavutil
    mav = mavutil.mavfile_global
    if ratio is None:
        ratio = 1.9936 # APM default
    if used_ratio is None:
        if 'ARSPD_RATIO' in mav.params:
            used_ratio = mav.params['ARSPD_RATIO']
        else:
            print("no ARSPD_RATIO in mav.params")
            used_ratio = ratio
    airspeed_pressure = (VFR_HUD.airspeed**2) / used_ratio
    airspeed = sqrt(airspeed_pressure * ratio)
    return airspeed

def airspeed_ratio(VFR_HUD):
    '''recompute airspeed with a different ARSPD_RATIO'''
    import mavutil
    mav = mavutil.mavfile_global
    airspeed_pressure = (VFR_HUD.airspeed**2) / ratio
    airspeed = sqrt(airspeed_pressure * ratio)
    return airspeed

def airspeed_voltage(VFR_HUD, ratio=None):
    '''back-calculate the voltage the airspeed sensor must have seen'''
    import mavutil
    mav = mavutil.mavfile_global
    if ratio is None:
        ratio = 1.9936 # APM default
    if 'ARSPD_RATIO' in mav.params:
        used_ratio = mav.params['ARSPD_RATIO']
    else:
        used_ratio = ratio
    if 'ARSPD_OFFSET' in mav.params:
        offset = mav.params['ARSPD_OFFSET']
    else:
        return -1
    airspeed_pressure = (pow(VFR_HUD.airspeed,2)) / used_ratio
    raw = airspeed_pressure + offset
    SCALING_OLD_CALIBRATION = 204.8
    voltage = 5.0 * raw / 4096
    return voltage


def earth_rates(ATTITUDE):
    '''return angular velocities in earth frame'''
    from math import sin, cos, tan, fabs

    p     = ATTITUDE.rollspeed
    q     = ATTITUDE.pitchspeed
    r     = ATTITUDE.yawspeed
    phi   = ATTITUDE.roll
    theta = ATTITUDE.pitch
    psi   = ATTITUDE.yaw

    phiDot   = p + tan(theta)*(q*sin(phi) + r*cos(phi))
    thetaDot = q*cos(phi) - r*sin(phi)
    if fabs(cos(theta)) < 1.0e-20:
        theta += 1.0e-10
    psiDot   = (q*sin(phi) + r*cos(phi))/cos(theta)
    return (phiDot, thetaDot, psiDot)

def roll_rate(ATTITUDE):
    '''return roll rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return phiDot

def pitch_rate(ATTITUDE):
    '''return pitch rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return thetaDot

def yaw_rate(ATTITUDE):
    '''return yaw rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return psiDot


def gps_velocity(GLOBAL_POSITION_INT):
    '''return GPS velocity vector'''
    return Vector3(GLOBAL_POSITION_INT.vx, GLOBAL_POSITION_INT.vy, GLOBAL_POSITION_INT.vz) * 0.01


def gps_velocity_old(GPS_RAW_INT):
    '''return GPS velocity vector'''
    return Vector3(GPS_RAW_INT.vel*0.01*cos(radians(GPS_RAW_INT.cog*0.01)),
                   GPS_RAW_INT.vel*0.01*sin(radians(GPS_RAW_INT.cog*0.01)), 0)

def gps_velocity_body(GPS_RAW_INT, ATTITUDE):
    '''return GPS velocity vector in body frame'''
    r = rotation(ATTITUDE)
    return r.transposed() * Vector3(GPS_RAW_INT.vel*0.01*cos(radians(GPS_RAW_INT.cog*0.01)),
                                    GPS_RAW_INT.vel*0.01*sin(radians(GPS_RAW_INT.cog*0.01)),
                                    -tan(ATTITUDE.pitch)*GPS_RAW_INT.vel*0.01)

def earth_accel(RAW_IMU,ATTITUDE):
    '''return earth frame acceleration vector'''
    r = rotation(ATTITUDE)
    accel = Vector3(RAW_IMU.xacc, RAW_IMU.yacc, RAW_IMU.zacc) * 9.81 * 0.001
    return r * accel

def earth_gyro(RAW_IMU,ATTITUDE):
    '''return earth frame gyro vector'''
    r = rotation(ATTITUDE)
    accel = Vector3(degrees(RAW_IMU.xgyro), degrees(RAW_IMU.ygyro), degrees(RAW_IMU.zgyro)) * 0.001
    return r * accel

def airspeed_energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD):
    '''return airspeed energy error matching APM internals
    This is positive when we are going too slow
    '''
    aspeed_cm = VFR_HUD.airspeed*100
    target_airspeed = NAV_CONTROLLER_OUTPUT.aspd_error + aspeed_cm
    airspeed_energy_error = ((target_airspeed*target_airspeed) - (aspeed_cm*aspeed_cm))*0.00005
    return airspeed_energy_error


def energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD):
    '''return energy error matching APM internals
    This is positive when we are too low or going too slow
    '''
    aspeed_energy_error = airspeed_energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD)
    alt_error = NAV_CONTROLLER_OUTPUT.alt_error*100
    energy_error = aspeed_energy_error + alt_error*0.098
    return energy_error

def rover_turn_circle(SERVO_OUTPUT_RAW):
    '''return turning circle (diameter) in meters for steering_angle in degrees
    '''

    # this matches Toms slash
    max_wheel_turn = 35
    wheelbase      = 0.335
    wheeltrack     = 0.296

    steering_angle = max_wheel_turn * (SERVO_OUTPUT_RAW.servo1_raw - 1500) / 400.0
    theta = radians(steering_angle)
    return (wheeltrack/2) + (wheelbase/sin(theta))

def rover_yaw_rate(VFR_HUD, SERVO_OUTPUT_RAW):
    '''return yaw rate in degrees/second given steering_angle and speed'''
    max_wheel_turn=35
    speed = VFR_HUD.groundspeed
    # assume 1100 to 1900 PWM on steering
    steering_angle = max_wheel_turn * (SERVO_OUTPUT_RAW.servo1_raw - 1500) / 400.0
    if abs(steering_angle) < 1.0e-6 or abs(speed) < 1.0e-6:
        return 0
    d = rover_turn_circle(SERVO_OUTPUT_RAW)
    c = pi * d
    t = c / speed
    rate = 360.0 / t
    return rate

def rover_lat_accel(VFR_HUD, SERVO_OUTPUT_RAW):
    '''return lateral acceleration in m/s/s'''
    speed = VFR_HUD.groundspeed
    yaw_rate = rover_yaw_rate(VFR_HUD, SERVO_OUTPUT_RAW)
    accel = radians(yaw_rate) * speed
    return accel


def demix1(servo1, servo2):
    '''de-mix a mixed servo output'''
    s1 = servo1 - 1500
    s2 = servo2 - 1500
    out1 = (s1+s2)/2
    out2 = (s1-s2)/2
    return out1+1500

def demix2(servo1, servo2):
    '''de-mix a mixed servo output'''
    s1 = servo1 - 1500
    s2 = servo2 - 1500
    out1 = (s1+s2)/2
    out2 = (s1-s2)/2
    return out2+1500

def wrap_180(angle):
    if angle > 180:
        angle -= 360.0
    if angle < -180:
        angle += 360.0
    return angle

    
def wrap_360(angle):
    if angle > 360:
        angle -= 360.0
    if angle < 0:
        angle += 360.0
    return angle

class DCM_State(object):
    '''DCM state object'''
    def __init__(self, roll, pitch, yaw):
        self.dcm = Matrix3()
        self.dcm2 = Matrix3()
        self.dcm.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.dcm2.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.mag = Vector3()
        self.gyro = Vector3()
        self.accel = Vector3()
        self.gps = None
        self.rate = 50.0
        self.kp = 0.2
        self.kp_yaw = 0.3
        self.omega_P = Vector3()
        self.omega_P_yaw = Vector3()
        self.omega_I = Vector3() # (-0.00199045287445, -0.00653007719666, -0.00714212376624)
        self.omega_I_sum = Vector3()
        self.omega_I_sum_time = 0
        self.omega = Vector3()
        self.ra_sum = Vector3()
        self.last_delta_angle = Vector3()
        self.last_velocity = Vector3()
        (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()
        (self.roll2, self.pitch2, self.yaw2) = self.dcm2.to_euler()
        
    def update(self, gyro, accel, mag, GPS):
        if self.gyro != gyro or self.accel != accel:
            delta_angle = (gyro+self.omega_I) / self.rate
            self.dcm.rotate(delta_angle)
            correction = self.last_delta_angle % delta_angle
            #print (delta_angle - self.last_delta_angle) * 58.0
            corrected_delta = delta_angle + 0.0833333 * correction
            self.dcm2.rotate(corrected_delta)
            self.last_delta_angle = delta_angle

            self.dcm.normalize()
            self.dcm2.normalize()

            self.gyro = gyro
            self.accel = accel
            (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()
            (self.roll2, self.pitch2, self.yaw2) = self.dcm2.to_euler()

dcm_state = None

def DCM_update(IMU, ATT, MAG, GPS):
    '''implement full DCM system'''
    global dcm_state
    if dcm_state is None:
        dcm_state = DCM_State(ATT.Roll, ATT.Pitch, ATT.Yaw)

    mag   = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)
    gyro  = Vector3(IMU.GyrX, IMU.GyrY, IMU.GyrZ)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    accel2 = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    dcm_state.update(gyro, accel, mag, GPS)
    return dcm_state

class PX4_State(object):
    '''PX4 DCM state object'''
    def __init__(self, roll, pitch, yaw, timestamp):
        self.dcm = Matrix3()
        self.dcm.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.gyro = Vector3()
        self.accel = Vector3()
        self.timestamp = timestamp
        (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()
        
    def update(self, gyro, accel, timestamp):
        if self.gyro != gyro or self.accel != accel:
            delta_angle = gyro * (timestamp - self.timestamp)
            self.timestamp = timestamp
            self.dcm.rotate(delta_angle)
            self.dcm.normalize()
            self.gyro = gyro
            self.accel = accel
            (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()

px4_state = None

def PX4_update(IMU, ATT):
    '''implement full DCM using PX4 native SD log data'''
    global px4_state
    if px4_state is None:
        px4_state = PX4_State(degrees(ATT.Roll), degrees(ATT.Pitch), degrees(ATT.Yaw), IMU._timestamp)

    gyro  = Vector3(IMU.GyroX, IMU.GyroY, IMU.GyroZ)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    px4_state.update(gyro, accel, IMU._timestamp)
    return px4_state

_downsample_N = 0

def downsample(N):
    '''conditional that is true on every Nth sample'''
    global _downsample_N
    _downsample_N = (_downsample_N + 1) % N
    return _downsample_N == 0

def armed(HEARTBEAT):
    '''return 1 if armed, 0 if not'''
    from pymavlink import mavutil
    if HEARTBEAT.type == mavutil.mavlink.MAV_TYPE_GCS:
        self = mavutil.mavfile_global
        if self.motors_armed():
            return 1
        return 0
    if HEARTBEAT.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return 1
    return 0

def rotation_df(ATT):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(radians(ATT.Roll), radians(ATT.Pitch), radians(ATT.Yaw))
    return r

def rotation2(AHRS2):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(AHRS2.roll, AHRS2.pitch, AHRS2.yaw)
    return r

def earth_accel2(RAW_IMU,ATTITUDE):
    '''return earth frame acceleration vector from AHRS2'''
    r = rotation2(ATTITUDE)
    accel = Vector3(RAW_IMU.xacc, RAW_IMU.yacc, RAW_IMU.zacc) * 9.81 * 0.001
    return r * accel

def earth_accel_df(IMU,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    return r * accel

def earth_accel2_df(IMU,IMU2,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel1 = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    accel2 = Vector3(IMU2.AccX, IMU2.AccY, IMU2.AccZ)
    accel = 0.5 * (accel1 + accel2)
    return r * accel

def gps_velocity_df(GPS):
    '''return GPS velocity vector'''
    vx = GPS.Spd * cos(radians(GPS.GCrs))
    vy = GPS.Spd * sin(radians(GPS.GCrs))
    return Vector3(vx, vy, GPS.VZ)

def distance_gps2(GPS, GPS2):
    '''distance between two points'''
    if GPS.TimeMS != GPS2.TimeMS:
        # reject messages not time aligned
        return None
    return distance_two(GPS, GPS2)


radius_of_earth = 6378100.0 # in meters

def wrap_valid_longitude(lon):
  ''' wrap a longitude value around to always have a value in the range
      [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
  '''
  return (((lon + 180.0) % 360.0) - 180.0)

def gps_newpos(lat, lon, bearing, distance):
	'''extrapolate latitude/longitude given a heading and distance
	thanks to http://www.movable-type.co.uk/scripts/latlong.html
	'''
        import math
	lat1 = math.radians(lat)
	lon1 = math.radians(lon)
	brng = math.radians(bearing)
	dr = distance/radius_of_earth

	lat2 = math.asin(math.sin(lat1)*math.cos(dr) +
                         math.cos(lat1)*math.sin(dr)*math.cos(brng))
	lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dr)*math.cos(lat1), 
                                 math.cos(dr)-math.sin(lat1)*math.sin(lat2))
	return (math.degrees(lat2), wrap_valid_longitude(math.degrees(lon2)))

def gps_offset(lat, lon, east, north):
	'''return new lat/lon after moving east/north
	by the given number of meters'''
        import math
	bearing = math.degrees(math.atan2(east, north))
	distance = math.sqrt(east**2 + north**2)
	return gps_newpos(lat, lon, bearing, distance)

ekf_home = None

def ekf1_pos(EKF1):
    '''calculate EKF position when EKF disabled'''
    global ekf_home
    from pymavlink import mavutil
    self = mavutil.mavfile_global
    if ekf_home is None:
        if not 'GPS' in self.messages or self.messages['GPS'].Status != 3:
            return None
        ekf_home = self.messages['GPS']
        (ekf_home.Lat, ekf_home.Lng) = gps_offset(ekf_home.Lat, ekf_home.Lng, -EKF1.PE, -EKF1.PN)
    (lat,lon) = gps_offset(ekf_home.Lat, ekf_home.Lng, EKF1.PE, EKF1.PN)
    return (lat, lon)

