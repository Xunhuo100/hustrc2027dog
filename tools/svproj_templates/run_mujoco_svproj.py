#!/usr/bin/env python3
"""Open the CAD course with Go2W under standing PD control."""
import argparse
import json
from pathlib import Path
import time
import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true', help='Open the interactive MuJoCo viewer')
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    root = Path(__file__).resolve().parents[2]
    path = root/'unitree_robots/go2_w/urdf/scene_svproj.xml'
    model = mujoco.MjModel.from_xml_path(str(path))
    state = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, state, 0)
    mujoco.mj_forward(model, state)
    joints = model.actuator_trnid[:, 0]
    qa, va = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    reference = state.qpos[qa].copy()
    wheels = np.array(['foot' in mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in joints])
    def step():
        torque = 40*(reference-state.qpos[qa])-1.5*state.qvel[va]
        torque[wheels] = -.5*state.qvel[va[wheels]]
        state.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        mujoco.mj_step(model, state)
        if not np.isfinite(state.qpos).all() or not np.isfinite(state.qvel).all():
            raise RuntimeError('Nonfinite simulation state')
    if args.viewer:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(model, state) as viewer:
            viewer.cam.lookat[:] = [.5, .5, .35]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3., 130., -30.
            viewer.opt.geomgroup[3] = 0
            while viewer.is_running() and state.time < args.seconds:
                start = time.monotonic()
                step()
                viewer.sync()
                remaining = model.opt.timestep-(time.monotonic()-start)
                if remaining > 0:
                    time.sleep(remaining)
    else:
        while state.time < args.seconds:
            step()
    print(json.dumps({'scene':str(path), 'simulated_seconds':float(state.time),
                      'base_xyz':state.qpos[:3].tolist(), 'controller':'standing PD; wheels velocity zero'}))


if __name__ == '__main__':
    main()
