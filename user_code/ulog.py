import numpy as np
import pandas as pd
from pyulog import ULog
from scipy.signal import butter, filtfilt, savgol_filter
from scipy.spatial.transform import Rotation as R
import os

# 低通滤波
def lowpass(x, fs_hz, fc_hz, order=3):
    if len(x) < 10:
        return x
    b, a = butter(order, fc_hz/(0.5*fs_hz), btype='low')
    return filtfilt(b, a, x, axis=0)

# 平滑导数（角速度微分）
def smooth_diff(x, fs_hz, window=51, poly=3):
    if len(x) < window:
        return np.gradient(x, 1.0/fs_hz, axis=0)
    return savgol_filter(x, window_length=window, polyorder=poly, deriv=1, delta=1.0/fs_hz, axis=0)

# 将某个 ULog topic 转为 DataFrame（含时间戳 s）
def topic_to_df(ulog, name):
    ds = [d for d in ulog.data_list if d.name == name]
    if not ds:
        return None
    d = ds[0]
    df = pd.DataFrame(d.data)
    if 'timestamp' not in df:
        return None
    df['t'] = df['timestamp'] * 1e-6  # 转秒
    return df

# 合并到统一时间轴（以 IMU 为主）
def merge_asof_multi(base_df, others, on='t', tol=0.01):
    out = base_df.copy()
    for k, df in others.items():
        if df is None or df.empty:
            continue
        out = pd.merge_asof(out.sort_values(on), df.sort_values(on), on=on, direction='nearest', tolerance=tol, suffixes=('', f'_{k}'))
    return out

def main(path, mass_kg=1.0, fs_imu=400.0, fc_low=40.0):
    ulog = ULog(path)

    # 读取常用话题（若字段名不同，稍作调整）
    imu = topic_to_df(ulog, 'vehicle_imu')  # 含 accel_m_s2 或 delta_velocity
    ang = topic_to_df(ulog, 'vehicle_angular_velocity')  # xyz/xyz[0..2] or rollspeed/pitchspeed/yawspeed
    att = topic_to_df(ulog, 'vehicle_attitude')          # q[0..3] or q
    lpos = topic_to_df(ulog, 'vehicle_local_position')
    air = topic_to_df(ulog, 'vehicle_air_data')
    # 兼容不同日志下的执行器输出话题，避免对 DataFrame 使用 "or"（布尔歧义）
    amot = topic_to_df(ulog, 'actuator_motors')
    if amot is None or amot.empty:
        amot = topic_to_df(ulog, 'actuator_outputs')
        if amot is None or amot.empty:
            amot = topic_to_df(ulog, 'actuator_outputs_sim')
    asrv = topic_to_df(ulog, 'actuator_servos')
    gps = topic_to_df(ulog, 'vehicle_gps_position')
    batt = topic_to_df(ulog, 'battery_status')

    # 统一 IMU 时间轴：按 IMU 采样率重采样
    if imu is None or imu.empty:
        raise RuntimeError('vehicle_imu not found')
    t0, t1 = imu['t'].iloc[0], imu['t'].iloc[-1]
    t_grid = np.arange(t0, t1, 1.0/fs_imu)
    base = pd.DataFrame({'t': t_grid})

    # 将其它话题按时间最近对齐到 IMU 轴上
    merged = merge_asof_multi(base, {
        'imu': imu,
        'ang': ang,
        'att': att,
        'lpos': lpos,
        'air': air,
        'amot': amot,
        'asrv': asrv,
        'gps': gps,
        'batt': batt
    }, on='t', tol=0.02)

    # 提取加速度（机体系）：优先 accel_m_s2；否则由 delta_velocity / dt 近似
    if {'accel_x', 'accel_y', 'accel_z'}.issubset(merged.columns):
        acc_b = merged[['accel_x', 'accel_y', 'accel_z']].to_numpy()
    elif {'delta_velocity[0]', 'delta_velocity[1]', 'delta_velocity[2]', 'delta_velocity_dt'}.issubset(merged.columns):
        dv = merged[['delta_velocity[0]', 'delta_velocity[1]', 'delta_velocity[2]']].to_numpy()
        dt = merged['delta_velocity_dt'].to_numpy().reshape(-1, 1)
        acc_b = np.divide(dv, np.maximum(dt, 1e-4))
    else:
        raise RuntimeError('No accelerometer fields in vehicle_imu')

    # 角速度（机体系）
    if {'xyz[0]','xyz[1]','xyz[2]'}.issubset(merged.columns):
        omega_b = merged[['xyz[0]','xyz[1]','xyz[2]']].to_numpy()
    elif {'rollspeed','pitchspeed','yawspeed'}.issubset(merged.columns):
        omega_b = merged[['rollspeed','pitchspeed','yawspeed']].to_numpy()
    else:
        omega_b = np.zeros((len(merged), 3))

    # 四元数 -> 旋转矩阵（世界->机体），并给出 Rbw 用于坐标变换
    if {'q[0]','q[1]','q[2]','q[3]'}.issubset(merged.columns):
        q = merged[['q[0]','q[1]','q[2]','q[3]']].to_numpy()
    elif {'q[0]','q[1]','q[2]','q[3]','timestamp'}.issubset(att.columns) if att is not None else False:
        q = att[['q[0]','q[1]','q[2]','q[3]']].to_numpy()
    else:
        q = None

    # 低通滤波
    acc_b_f = lowpass(acc_b, fs_hz=fs_imu, fc_hz=fc_low)
    omega_b_f = lowpass(omega_b, fs_hz=fs_imu, fc_hz=fc_low)

    # 角加速度
    omegad_b = smooth_diff(omega_b_f, fs_hz=fs_imu, window=int(min(51, len(omega_b_f)//2*2-1)), poly=3)

    # 比力（机体系）：fb ≈ acc_b_f（若需要扣重力项，可用 Rbw*g 进一步构造）
    merged[['acc_bx','acc_by','acc_bz']] = acc_b_f
    merged[['omg_bx','omg_by','omg_bz']] = omega_b_f
    merged[['omgd_bx','omgd_by','omgd_bz']] = omegad_b

    # 选取关键信息保存（后续反演和建模用）
    cols_keep = ['t','acc_bx','acc_by','acc_bz','omg_bx','omg_by','omg_bz','omgd_bx','omgd_by','omgd_bz']
    for c in ['vehicle_air_data.baro_temp','vehicle_air_data.baro_pres']:  # 占位示例
        pass
    # 可按需追加执行量/姿态/位置等列

    # 导出到 user_processed 目录，按输入文件名命名
    out = merged[cols_keep].copy()
    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'user_processed'))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(path))[0]
    out_parquet = os.path.join(out_dir, f'{base}_preprocessed.parquet')
    out_csv = os.path.join(out_dir, f'{base}_preprocessed.csv')
    out.to_parquet(out_parquet, index=False)
    out.to_csv(out_csv, index=False)
    print(f'Saved: {out_parquet} / {out_csv}')

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print('Usage: python preprocess_ulog.py /path/to/log.ulg')
        sys.exit(1)
    main(sys.argv[1], mass_kg=1.0, fs_imu=400.0, fc_low=40.0)
