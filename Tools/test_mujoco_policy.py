#!/usr/bin/env python3
"""Reference rollout and Unity ONNX parity check.
Run: uv run --python 3.12 --with mujoco==3.15.0 --with onnxruntime python Tools/test_mujoco_policy.py
Unity's sandbox writes Library/MicroDuckPolicySnapshot.json at t=10 seconds.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import onnxruntime as ort

project = Path(__file__).resolve().parents[1]
asset = project / 'Assets/StreamingAssets/MicroDuck'
xml = ET.parse(asset / 'robot.xml')
xml.getroot().find('compiler').set('meshdir', str(asset / 'assets'))
model = mujoco.MjModel.from_xml_string(ET.tostring(xml.getroot(), encoding='unicode'))
data = mujoco.MjData(model)
policy = ort.InferenceSession(str(project / 'Assets/Robots/MicroDuck/Mujoco/BEST_alpha_walking.onnx'), providers=['CPUExecutionProvider'])
names = ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle','neck_pitch','head_pitch','head_yaw','head_roll','right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle']
pose = np.array([0,-0.0872664626,-0.457924,-0.004940,0.452984,0.3490658504,0.3490658504,0,0,0,0.0872664626,0.457924,0.004940,-0.452984], dtype=np.float32)
qadr = [model.jnt_qposadr[model.joint(n).id] for n in names]
vadr = [model.jnt_dofadr[model.joint(n).id] for n in names]
aids = [model.actuator(n).id for n in names]
trunk = model.body('trunk_base').id
gyro = model.sensor_adr[model.sensor('imu_ang_vel').id]
data.qpos[2] = 0.12
data.qpos[qadr] = pose
data.ctrl[aids] = pose
mujoco.mj_forward(model,data)
last = np.zeros(14, np.float32)
min_height = 1
for step in range(1500):
    gravity = data.xmat[trunk].reshape(3,3).T @ np.array([0,0,-1])
    cmd = [0.3,0,0.4] if step >= 250 else [0,0,0]
    obs = np.concatenate([data.sensordata[gyro:gyro+3],gravity,data.qpos[qadr]-pose,data.qvel[vadr],last,cmd,np.zeros(10)]).astype(np.float32)[None]
    last = policy.run(None,{policy.get_inputs()[0].name:obs})[0][0]
    data.ctrl[aids] = pose + last
    for _ in range(4): mujoco.mj_step(model,data)
    min_height = min(min_height,data.qpos[2])
    assert np.isfinite(data.qpos).all()
    assert gravity[2] < -0.5, f'Fell at {data.time:.2f}s'
print(f'Reference 30s: min trunk height={min_height:.4f}m, position={data.qpos[:3]}')
assert np.linalg.norm(data.qpos[:2]) > 0.05, 'Walking produced no displacement'
snapshot = project / 'Library/MicroDuckPolicySnapshot.json'
if snapshot.exists():
    sample = json.loads(snapshot.read_text())
    actual = np.array(sample['actions'])
    expected = policy.run(None,{policy.get_inputs()[0].name:np.array(sample['observations'],np.float32)[None]})[0][0]
    error = np.max(np.abs(actual-expected))
    print(f'Unity policy max absolute error vs ONNX Runtime: {error:.8f}')
    print('Unity actions:',actual)
    print('Reference actions:',expected)
    assert error < 1e-3, 'Unity inference differs from ONNX Runtime'
