#!/usr/bin/env python3
"""Check source fidelity, real collision holes, and a Go2W standing rollout."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np


def validate(root, source, assets):
    out = root/'fields/svproj_course/validation'
    out.mkdir(exist_ok=True)
    course = root/'fields/svproj_course'
    manifest = json.loads((assets/'manifest.json').read_text())
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest['input_sha256']
    from convert_svproj_course import read_project
    solids, _, robot = read_project(source)
    assert len(solids) == len(manifest['solids']) == 28 and len(robot['joints']) == 0
    with np.load(assets/'terrain.npz', allow_pickle=False) as data:
        vertices, triangles = data['vertices'], data['triangles']
        nvertices = sum(len(v) for v, _ in solids)
        np.testing.assert_allclose(vertices[:nvertices], np.vstack([v for v, _ in solids]), atol=5e-7)
        offset, faces = 0, []
        for v, f in solids:
            faces.append(f + offset)
            offset += len(v)
        np.testing.assert_array_equal(triangles[:-2], np.vstack(faces))
        origin, scale = data['grid_origin'], float(data['horizontal_scale'])
        def scan(x, y):
            p = np.rint((np.array([x, y])-origin)/scale).astype(int)
            return float(data['heightsamples'][tuple(p)] * data['vertical_scale'])
        tire = np.mean(manifest['solids'][4]['bounds_m'], axis=0)
        probes = [('frame_opening', .5, .5, 0.),
                  ('wall_top', 5.64, .5, .30),
                  ('tire_opening', float(tire[0]), float(tire[1]), 0.)]
        for name, x, y, expected in probes:
            assert abs(scan(x, y)-expected) < .002, (name, scan(x, y))
    # Spheres physically fall onto the compiled convex collision parts. A ray
    # against a mesh's original triangles alone would not test convexification.
    test = ET.parse(course/'course.xml').getroot()
    test.tag = 'mujoco'
    for mesh in test.findall('.//mesh'):
        mesh.set('file', str(course/mesh.get('file')))
    ET.SubElement(test, 'option', timestep='.002')
    world = test.find('worldbody')
    ET.SubElement(world, 'geom', name='ground', type='plane', size='20 20 .1')
    for name, x, y, _ in probes:
        body = ET.SubElement(world, 'body', name=name, pos=f'{x} {y} 1')
        ET.SubElement(body, 'joint', type='slide', axis='0 0 1')
        ET.SubElement(body, 'geom', type='sphere', size='.02', mass='.02')
    model = mujoco.MjModel.from_xml_string(ET.tostring(test, encoding='unicode'))
    state = mujoco.MjData(model)
    for _ in range(2000):
        mujoco.mj_step(model, state)
    collision_results = {}
    for name, x, y, expected in probes:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        height = float(state.xpos[bid, 2]-.02)
        assert abs(height-expected) < .005, (name, height, expected)
        collision_results[name] = {'surface_z':height, 'expected_z':expected}
    path = root/'unitree_robots/go2_w/urdf/scene_svproj.xml'
    model = mujoco.MjModel.from_xml_path(str(path))
    state = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, state, 0)
    mujoco.mj_forward(model, state)
    initial_obstacle_contacts = []
    for contact in state.contact:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ''
                 for g in [contact.geom1, contact.geom2]]
        if any(n.startswith('svproj_collision') for n in names):
            initial_obstacle_contacts.append(names)
    assert not initial_obstacle_contacts, initial_obstacle_contacts
    reference = state.qpos.copy()
    joints = model.actuator_trnid[:, 0]
    qa, va = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    wheels = np.array(['foot' in mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in joints])
    heights = []
    for _ in range(5000):
        torque = 40*(reference[qa]-state.qpos[qa])-1.5*state.qvel[va]
        torque[wheels] = -.5*state.qvel[va[wheels]]
        state.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        mujoco.mj_step(model, state)
        assert np.isfinite(state.qpos).all() and np.isfinite(state.qvel).all()
        heights.append(float(state.qpos[2]))
    assert min(heights) > .25, min(heights)
    assert all(w.number == 0 for w in state.warning), state.warning
    results = {'passed':True, 'source_sha256':manifest['input_sha256'], 'solids':len(solids),
               'source_joints':0, 'convex_parts':sum(s['convex_parts'] for s in manifest['solids']),
               'isaac_mesh_vertices':len(vertices), 'isaac_mesh_triangles':len(triangles),
               'mujoco_version':mujoco.__version__, 'physical_collision_probes':collision_results,
               'go2w_stand':{'seconds':float(state.time),'base_z_min':min(heights),
                             'base_z_max':max(heights),'final_base_xyz':state.qpos[:3].tolist(),
                             'warnings':0, 'initial_obstacle_contacts':0}}
    scene = mujoco.MjModel.from_xml_path(str(course/'scene.xml'))
    scene_state = mujoco.MjData(scene)
    mujoco.mj_forward(scene, scene_state)
    from PIL import Image
    with mujoco.Renderer(scene, height=900, width=1400) as renderer:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [5, -1.5, .15]
        cam.distance, cam.azimuth, cam.elevation = 15, 90, -65
        opt = mujoco.MjvOption()
        opt.geomgroup[3] = 0
        renderer.update_scene(scene_state, camera=cam, scene_option=opt)
        Image.fromarray(renderer.render()).save(out/'overview.png')
    (out/'mujoco_report.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mujoco-root', type=Path, default=Path.home()/'dogMujoco2027')
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1]/'robot.svproj')
    parser.add_argument('--assets', type=Path, default=Path(__file__).resolve().parents[1]/'assets/svproj_course')
    args = parser.parse_args()
    validate(args.mujoco_root, args.source, args.assets)
