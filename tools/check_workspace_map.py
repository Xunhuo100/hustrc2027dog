#!/usr/bin/env python3
"""Verify portable map loading, actual hole collisions, and both robot scenes."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]


def render(model, state, destination, robot_view=False):
    from PIL import Image
    with mujoco.Renderer(model, height=900, width=1400) as renderer:
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.5, .5, .2] if robot_view else [5, -1.5, .15]
        camera.distance = 2.4 if robot_view else 15
        camera.azimuth = 130 if robot_view else 90
        camera.elevation = -25 if robot_view else -65
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        renderer.update_scene(state, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(destination)


def check(models, output):
    output.mkdir(parents=True, exist_ok=True)
    course = models/'terrains/svproj_course.xml'
    tree = ET.parse(course).getroot()
    assert tree.tag == 'mujoco'
    assert all('file' not in element.attrib for element in tree.iter())
    meshes = tree.findall('./asset/mesh')
    visual = [mesh for mesh in meshes if mesh.get('name', '').startswith('svproj_visual_')]
    collision = [mesh for mesh in meshes if mesh.get('name', '').startswith('svproj_collision_')]
    assert len(visual) == 28 and len(collision) == 327
    # Loading from a different directory with just this XML proves map portability.
    with tempfile.TemporaryDirectory(prefix='svproj-map-') as directory:
        moved = Path(directory)/'map.xml'
        shutil.copy2(course, moved)
        terrain_model = mujoco.MjModel.from_xml_path(str(moved))
    assert terrain_model.nmesh == 355 and terrain_model.nu == 0
    terrain_state = mujoco.MjData(terrain_model)
    mujoco.mj_forward(terrain_model, terrain_state)
    render(terrain_model, terrain_state, output/'overview.png')
    # Exercise compiled convex collision shapes, rather than just visual meshes.
    metadata = json.loads((models/'terrains/svproj_course.json').read_text())
    tire = np.mean(metadata['solids'][4]['bounds_m'], axis=0)
    probes = [('frame_opening', .5, .5, 0.), ('wall_top', 5.64, .5, .3),
              ('tire_opening', float(tire[0]), float(tire[1]), 0.)]
    ET.SubElement(tree, 'option', timestep='.002')
    world = tree.find('worldbody')
    for name, x, y, _ in probes:
        body = ET.SubElement(world, 'body', name=name, pos=f'{x} {y} 1')
        ET.SubElement(body, 'joint', type='slide', axis='0 0 1')
        ET.SubElement(body, 'geom', type='sphere', size='.02', mass='.02')
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding='unicode'))
    state = mujoco.MjData(model)
    for _ in range(2000):
        mujoco.mj_step(model, state)
    collisions = {}
    for name, _, _, expected in probes:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        z = float(state.xpos[body, 2]-.02)
        assert abs(z-expected) < .005, (name, z, expected)
        collisions[name] = {'surface_z':z, 'expected_z':expected}
    scenes = {}
    for robot in ['go2w', 'hustdog_v11']:
        path = models/robot/'urdf/scene_svproj.xml'
        model = mujoco.MjModel.from_xml_path(str(path.resolve()))
        state = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, state, 0)
        mujoco.mj_forward(model, state)
        assert model.nu == 16
        np.testing.assert_allclose(state.qpos[:2], metadata['spawn'][:2])
        planes = np.count_nonzero(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE)
        assert planes == 1
        for contact in state.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ''
                     for g in [contact.geom1, contact.geom2]]
            assert not any(name.startswith('svproj_collision_') for name in names), names
        render(model, state, output/(robot+'.png'), robot_view=True)
        joints = model.actuator_trnid[:, 0]
        qa, va = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
        reference = state.qpos[qa].copy()
        wheels = np.array(['foot' in mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in joints])
        steps = 5000 if robot == 'go2w' else 1000
        for _ in range(steps):
            if robot == 'go2w':
                torque = 40*(reference-state.qpos[qa])-1.5*state.qvel[va]
                torque[wheels] = -.5*state.qvel[va[wheels]]
                state.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
            else:
                # Preserve the imported preview's position actuators and joint zero pose.
                state.ctrl[:] = reference
            mujoco.mj_step(model, state)
            assert np.isfinite(state.qpos).all() and np.isfinite(state.qvel).all(), robot
        assert all(warning.number == 0 for warning in state.warning), robot
        if robot == 'go2w':
            assert state.qpos[2] > .25
        scenes[robot] = {'path':str(path), 'passed':True, 'actuators':model.nu,
                         'joints_including_free_base':model.njnt, 'ground_planes':int(planes),
                         'simulated_seconds':float(state.time), 'warnings':0,
                         'initial_map_contacts':0, 'final_base_xyz':state.qpos[:3].tolist(),
                         'control':'standing PD' if robot == 'go2w' else 'original position actuator preview'}
    # The current ROS-independent Go2W backend also needs to accept the new entry point.
    sys.path.insert(0, str(WORKSPACE/'src/wheel_leg_mujoco'))
    from wheel_leg_mujoco.mujoco_backend import MujocoRobot
    backend = MujocoRobot(str((models/'go2w/urdf/scene_svproj.xml').resolve()), timestep=.002)
    command = np.zeros((16, 6))
    command[:, 0] = 1
    command[:, 1] = backend.joint_state()[0]
    command[:, 3:5] = [40., 1.5]
    command[3::4, 3:5] = [0., .5]
    for _ in range(500):
        backend.step(command)
        assert np.isfinite(backend.data.qpos).all()
    report = {'passed':True, 'mujoco_version':mujoco.__version__, 'portable_map':True,
              'visual_solids':28, 'convex_collision_parts':327,
              'map_bytes':course.stat().st_size, 'collision_probes':collisions,
              'scenes':scenes, 'go2w_backend_steps':500,
              'hustdog_note':'Model load and original position-control preview verified; Go2W ROS/policy mapping is not applicable.'}
    (output/'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', type=Path, default=WORKSPACE/'src/wheel_leg_mujoco/models')
    parser.add_argument('--output', type=Path, default=WORKSPACE/'docs/svproj_workspace_validation')
    args = parser.parse_args()
    check(args.models.resolve(), args.output.resolve())
