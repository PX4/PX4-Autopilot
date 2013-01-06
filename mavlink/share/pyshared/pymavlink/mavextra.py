#!/usr/bin/env python
'''
useful extra functions for use by mavlink clients

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import os, sys
from math import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'examples'))

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
    import mavutil
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
    if hasattr(GPS_RAW1, 'cog'):
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

def airspeed(VFR_HUD, ratio=None):
    '''recompute airspeed with a different ARSPD_RATIO'''
    import mavutil
    mav = mavutil.mavfile_global
    if ratio is None:
        ratio = 1.98 # APM default
    if 'ARSPD_RATIO' in mav.params:
        used_ratio = mav.params['ARSPD_RATIO']
    else:
        used_ratio = ratio
    airspeed_pressure = (VFR_HUD.airspeed**2) / used_ratio
    airspeed = sqrt(airspeed_pressure * ratio)
    return airspeed


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


def gps_velocity(GPS_RAW_INT):
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

