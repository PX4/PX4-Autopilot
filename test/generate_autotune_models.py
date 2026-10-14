#!/usr/bin/env python3
"""Generate documented Gazebo stress models for Autotune response experiments.

These are synthetic variants, not models of a particular production aircraft.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def variant(source, destination, name, length_scale, mass_scale, inertia_extra, motor_up, motor_down,
            gyro_noise_scale=1.):
    tree = ET.parse(source / 'x500_base/model.sdf')
    model = tree.getroot().find('model')
    model.set('name', name)
    for pose in model.iter('pose'):
        values = [float(v) for v in pose.text.split()]
        pose.text = ' '.join(str(v * length_scale if k < 3 else v) for k, v in enumerate(values))
    for node in model.findall('.//geometry/mesh/scale') + model.findall('.//geometry/box/size'):
        node.text = ' '.join(str(float(v) * length_scale) for v in node.text.split())
    for tag in ['radius', 'length']:
        for node in model.findall(f'.//geometry/*/{tag}'):
            node.text = str(float(node.text) * length_scale)
    for link in model.findall('link'):
        mass = link.find('inertial/mass')
        mass.text = str(float(mass.text) * mass_scale)
        for node in link.find('inertial/inertia'):
            node.text = str(float(node.text) * mass_scale * length_scale**2 * inertia_extra)
    for node in model.findall('.//imu/angular_velocity/*/noise/stddev'):
        node.text = str(float(node.text) * gyro_noise_scale)
    motors = ET.parse(source / 'x500/model.sdf').getroot().find('model')
    for original in motors.findall('plugin'):
        plugin = copy.deepcopy(original)
        if plugin.find('motorNumber') is not None:
            changes = dict(timeConstantUp=motor_up, timeConstantDown=motor_down,
                           maxRotVelocity=1000/length_scale,
                           motorConstant=float(plugin.findtext('motorConstant'))*mass_scale*length_scale**2,
                           momentConstant=float(plugin.findtext('momentConstant'))*length_scale,
                           rotorDragCoefficient=float(plugin.findtext('rotorDragCoefficient'))*length_scale**3,
                           rollingMomentCoefficient=float(plugin.findtext('rollingMomentCoefficient'))*length_scale**4)
            for tag, value in changes.items():
                plugin.find(tag).text = str(value)
        model.append(plugin)
    folder = destination / name
    folder.mkdir()
    ET.indent(tree, space='  ')
    tree.write(folder/'model.sdf', encoding='utf-8', xml_declaration=True)
    mass = sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    metadata = dict(name=name, description='Synthetic stress model; no production-aircraft fidelity claim',
                    length_scale=length_scale, mass_scale=mass_scale, inertia_extra=inertia_extra,
                    total_mass_kg=mass, motor_time_constant_up_s=motor_up, motor_time_constant_down_s=motor_down,
                    base_inertia={v.tag: float(v.text) for v in model.find('link/inertial/inertia')},
                    max_motor_velocity_rad_s=1000/length_scale,
                    gyro_noise_standard_deviation_scale=gyro_noise_scale,
                    sdf_sha256=hashlib.sha256((folder/'model.sdf').read_bytes()).hexdigest())
    (folder/'description.json').write_text(json.dumps(metadata, indent=2)+'\n')
    (folder/'model.config').write_text(
        f'<?xml version="1.0"?><model><name>{name}</name><version>1.0</version>'
        '<sdf version="1.9">model.sdf</sdf></model>\n')
    print(json.dumps(metadata, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('Tools/simulation/gz/models'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for source in args.source.iterdir():
        if source.is_dir():
            (args.output/source.name).symlink_to(source.resolve(), target_is_directory=True)
    base = ET.parse(args.source/'x500_base/model.sdf').getroot().find('model')
    mass = sum(float(link.findtext('inertial/mass')) for link in base.findall('link'))
    variant(args.source, args.output, 'x500_slow', 1., 1., 4., .1, .15)
    variant(args.source, args.output, 'x500_1000kg', 10., 1000/mass, 1., .2, .3)
    variant(args.source, args.output, 'x500_noisy', 1., 1., 1., .0125, .025, 30.)
    slow = dict(MC_ROLL_P=2., MC_PITCH_P=2., MC_YAW_P=1., IMU_GYRO_CUTOFF=20., IMU_DGYRO_CUTOFF=10.)
    (args.output/'x500_slow/initial-params.json').write_text(json.dumps(slow, indent=2)+'\n')
    heavy = dict(MC_ROLL_P=.8, MC_PITCH_P=.8, MC_YAW_P=.3, MC_ROLLRATE_I=.05, MC_PITCHRATE_I=.05,
                 MC_ROLLRATE_D=.02, MC_PITCHRATE_D=.02, MC_YAWRATE_I=.02, IMU_GYRO_CUTOFF=8.,
                 IMU_DGYRO_CUTOFF=5., MC_YAW_TQ_CUTOFF=.7, MPC_XY_P=.3, MPC_XY_VEL_P_ACC=1.,
                 MPC_XY_VEL_I_ACC=.1, MPC_XY_VEL_D_ACC=.05, MPC_ACC_HOR_MAX=1., MPC_XY_VEL_MAX=2.)
    for motor, (x, y, km) in enumerate([(1.74, 1.74, .5), (-1.74, -1.74, .5), (1.74, -1.74, -.5), (-1.74, 1.74, -.5)]):
        heavy.update({f'SIM_GZ_EC_MIN{motor+1}': 15, f'SIM_GZ_EC_MAX{motor+1}': 100,
                      f'CA_ROTOR{motor}_PX': x, f'CA_ROTOR{motor}_PY': y, f'CA_ROTOR{motor}_KM': km})
    (args.output/'x500_1000kg/initial-params.json').write_text(json.dumps(heavy, indent=2)+'\n')


if __name__ == '__main__':
    main()
