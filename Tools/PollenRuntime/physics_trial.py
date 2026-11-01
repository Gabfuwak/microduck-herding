#!/usr/bin/env python3
"""Run the packaged Pollen core against the project's real MuJoCo robot model."""
import argparse
import ctypes as C
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('abi', ROOT / 'Tools/PollenControl/validate.py')
abi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(abi)
NAMES = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
         'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'right_hip_yaw',
         'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle']
WIRE = np.array([i if i < 9 else i + 1 for i in range(14)])
HOME = np.array(abi.HOME)[WIRE]

def trial(models, forward, left=0, yaw=0):
    xml = ET.fromstring((ROOT / 'Assets/Robots/MicroDuck/MuJoCo/scene.xml').read_text())
    ET.SubElement(xml.find('worldbody'), 'geom', name='trial_floor', type='plane',
                  size='20 20 .1', pos='0 0 0', contype='1', conaffinity='1',
                  friction='1 0.005 0.0001')
    assets = {p.name.removesuffix('.bytes'): p.read_bytes()
              for p in (ROOT / 'Assets/Robots/MicroDuck/MuJoCo/assets').glob('*.bytes')}
    model = mj.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'), assets)
    data = mj.MjData(model)
    qpos = [model.joint(n).qposadr[0] for n in NAMES]
    qvel = [model.joint(n).dofadr[0] for n in NAMES]
    acts = [model.actuator(n).id for n in NAMES]
    root = model.body('trunk_base').id
    gyro = model.sensor('imu_ang_vel').adr[0]
    gain = model.actuator_gainprm[acts, 0].copy()
    bias = model.actuator_biasprm[acts, 1].copy()
    model.opt.timestep = .005
    model.opt.gravity[:] = [0, 0, -9.81]
    data.qpos[:7] = [0, 0, .119182773, 1, 0, 0, 0]
    data.qpos[qpos] = HOME
    data.ctrl[acts] = HOME
    mj.mj_forward(model, data)
    native = abi.Native(ROOT / 'Assets/Plugins/Linux/x86_64/libmicroduck_control.so',
                        models, ROOT / 'Assets/Plugins/Linux/x86_64/libonnxruntime.so')
    records = []
    start = None
    max_joint_speed = 0
    try:
        for tick in range(701):
            sensors = abi.Sensors()
            for i, wire in enumerate(WIRE):
                sensors.positions[wire] = data.qpos[qpos[i]]
                sensors.velocities[wire] = data.qvel[qvel[i]]
                sensors.currents_ma[wire] = abs(data.actuator_force[acts[i]]) * 100
            sensors.gyro[:] = data.sensordata[gyro:gyro+3]
            sensors.gravity[:] = data.xmat[root].reshape(3, 3).T @ np.array([0, 0, -1.])
            sensors.quat[:] = data.xquat[root]
            command = abi.Command(forward, left, yaw) if tick >= 500 else abi.Command()
            out = abi.Output()
            native.check(native.lib.md_step(native.handle, C.byref(sensors), C.byref(command),
                                            round(data.time*1e6), C.byref(out)))
            ratio = out.gain / 200 if out.torque else 0
            model.actuator_gainprm[acts, 0] = gain * ratio
            model.actuator_biasprm[acts, 1] = bias * ratio
            data.ctrl[acts] = np.array(out.targets)[WIRE]
            if tick == 500:
                start = data.qpos[:3].copy()
            if tick % 25 == 0:
                records.append(dict(t=round(data.time, 3), position=data.qpos[:3].tolist(),
                                    rotation=data.qpos[3:7].tolist(), velocity=data.qvel[:3].tolist(),
                                    gravity=list(sensors.gravity), gyro=list(sensors.gyro),
                                    gain=out.gain, torque=out.torque, state=out.state,
                                    fallen=out.fallen, targets=list(out.targets),
                                    joints=data.qpos[qpos].tolist(), contacts=data.ncon))
            max_joint_speed = max(max_joint_speed, float(np.max(np.abs(data.qvel[qvel]))))
            if tick < 700:
                for _ in range(4):
                    mj.mj_step(model, data)
        return dict(command=[forward, left, yaw], delta=(data.qpos[:3]-start).tolist(),
                    max_joint_speed=max_joint_speed, records=records)
    finally:
        native.close()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=ROOT / 'Builds/PollenRuntime/physics-trial.json')
    p.add_argument('--forward', type=float, nargs='+', default=[0, .18, .3],
                   help='Forward commands to compare after 10 s settling, for 4 s each')
    args = p.parse_args()
    if mj.mj_version() != 3015000:
        raise RuntimeError("Use MuJoCo 3.15.0 to match the Unity runtime.")
    with tempfile.TemporaryDirectory(prefix='pollen-models-') as stage:
        for asset in (ROOT / 'Assets/Robots/MicroDuck/Policies/Pollen').glob('*.bytes'):
            (Path(stage) / asset.name.removesuffix('.bytes')).write_bytes(asset.read_bytes())
        results = [trial(Path(stage), command) for command in args.forward]
    report = dict(mujoco=mj.__version__, native_control_sha256=hashlib.sha256((ROOT / 'Assets/Plugins/Linux/x86_64/libmicroduck_control.so').read_bytes()).hexdigest(), scene_sha256=hashlib.sha256((ROOT / 'Assets/Robots/MicroDuck/MuJoCo/scene.xml').read_bytes()).hexdigest(), results=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({**{k:v for k,v in report.items() if k!='results'},
                     'results':[{k:v for k,v in r.items() if k!='records'} for r in results]}))

if __name__ == '__main__':
    main()
