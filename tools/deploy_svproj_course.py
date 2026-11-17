#!/usr/bin/env python3
"""Deploy converted course assets and isolated tasks to the three local projects."""
import argparse
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def save_xml(element, path):
    ET.indent(element)
    ET.ElementTree(element).write(path, encoding='utf-8', xml_declaration=True)


def deploy(source, mujoco_root, him_root, parkour_root):
    templates = Path(__file__).resolve().parent / 'svproj_templates'
    readme = templates.parent.parent/'docs/svproj_course_deployment.md'
    course = mujoco_root/'fields/svproj_course'
    shutil.copytree(source, course, dirs_exist_ok=True)
    shutil.copy2(templates/'run_mujoco_svproj.py', course/'run.py')
    if readme.is_file():
        shutil.copy2(readme, course/'README.md')
    # Make the include portable; the standalone scene is in this directory.
    tree = ET.parse(course/'course.xml')
    for mesh in tree.findall('.//mesh'):
        mesh.set('file', mesh.get('file').removeprefix('svproj_course/'))
    save_xml(tree.getroot(), course/'course.xml')
    scene = ET.Element('mujoco', model='svproj obstacle course')
    ET.SubElement(scene, 'include', file='course.xml')
    ET.SubElement(scene, 'option', timestep='0.002', gravity='0 0 -9.81')
    ET.SubElement(scene, 'statistic', center='5 -1.5 0.4', extent='8')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='1400', offheight='900')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='4 -1 12', dir='0 0 -1', directional='true')
    ET.SubElement(world, 'geom', name='svproj_ground', type='plane', size='16 10 .1',
                  rgba='.18 .22 .24 1', friction='.8 .02 .001')
    save_xml(scene, course/'scene.xml')
    # A separate entry point alongside the existing robot scenes preserves the SDK workflow.
    robot_dir = mujoco_root/'unitree_robots/go2_w/urdf'
    robot = ET.parse(robot_dir/'go2w.xml').getroot()
    meshdir = (robot_dir / robot.find('compiler').get('meshdir')).resolve()
    robot.find('compiler').attrib.pop('meshdir', None)
    spawn = json.loads((source/'manifest.json').read_text())['spawn']
    base = robot.find("./worldbody/body[@name='base']")
    base_pos = [float(v) for v in base.get('pos').split()]
    base_pos[:2] = spawn[:2]
    base.set('pos', ' '.join(map(str, base_pos)))
    for key in robot.findall('./keyframe/key'):
        qpos = key.get('qpos').split()
        qpos[:2] = [str(v) for v in spawn[:2]]
        key.set('qpos', ' '.join(qpos))
    for mesh in robot.findall('.//asset/mesh'):
        mesh.set('file', os.path.relpath(meshdir/mesh.get('file'), robot_dir))
    include = ET.SubElement(robot, 'include', file=os.path.relpath(course/'course.xml', robot_dir))
    # Mesh paths are relative to the main model, even when the mesh is in an include.
    robot_course = ET.parse(course/'course.xml').getroot()
    for mesh in robot_course.findall('.//mesh'):
        mesh.set('file', os.path.relpath(course/mesh.get('file'), robot_dir))
    save_xml(robot_course, robot_dir/'svproj_course_include.xml')
    include.set('file', 'svproj_course_include.xml')
    world = robot.find('worldbody')
    ET.SubElement(world, 'geom', name='svproj_ground', type='plane', size='16 10 .1',
                  rgba='.18 .22 .24 1', friction='.8 .02 .001')
    ET.SubElement(world, 'light', pos='4 -1 12', dir='0 0 -1', directional='true')
    ET.SubElement(robot, 'statistic', center='5 -1.5 .4', extent='8')
    visual = ET.SubElement(robot, 'visual')
    ET.SubElement(visual, 'global', offwidth='1400', offheight='900')
    robot.find('option').set('timestep', '.002')
    save_xml(robot, robot_dir/'scene_svproj.xml')
    for root, kind in [(him_root, 'him'), (parkour_root, 'parkour')]:
        gym = root/'legged_gym'
        target = gym/'resources/terrains/svproj_course'
        target.mkdir(parents=True, exist_ok=True)
        if readme.is_file():
            shutil.copy2(readme, target/'README.md')
        for name in ['terrain.npz', 'manifest.json']:
            shutil.copy2(source/name, target/name)
        shutil.copy2(templates/'svproj_terrain.py', gym/'legged_gym/utils/svproj_terrain.py')
        envdir = gym/'legged_gym/envs/svproj'
        envdir.mkdir(exist_ok=True)
        (envdir/'__init__.py').touch()
        shutil.copy2(templates/(kind+'_svproj.py'), envdir/'course.py')
        tools = root/'tools'
        tools.mkdir(exist_ok=True)
        shutil.copy2(templates/'check_svproj.py', tools/'check_svproj.py')
        registry = gym/'legged_gym/envs/__init__.py'
        text = registry.read_text()
        marker = '# Converted robot.svproj obstacle course'
        if marker not in text:
            if kind == 'him':
                addition = ('from .svproj.course import HIMSVProjRobot, HIMSVProjCfg, HIMSVProjCfgPPO\n'
                            'task_registry.register("him_go2w_svproj", HIMSVProjRobot, HIMSVProjCfg(), HIMSVProjCfgPPO())\n')
            else:
                addition = ('from .svproj.course import ParkourSVProjRobot, ParkourSVProjCfg, ParkourSVProjCfgPPO\n'
                            'task_registry.register("go2w_svproj", ParkourSVProjRobot, ParkourSVProjCfg(), ParkourSVProjCfgPPO())\n')
            registry.write_text(text+'\n'+marker+'\n'+addition)
    print(json.dumps({'mujoco':str(course/'scene.xml'),
                      'mujoco_robot':str(robot_dir/'scene_svproj.xml'),
                      'himloco':str(him_root),'extreme_parkour':str(parkour_root)}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, default=Path(__file__).resolve().parents[1]/'assets/svproj_course')
    parser.add_argument('--mujoco-root', type=Path, default=Path.home()/'dogMujoco2027')
    parser.add_argument('--him-root', type=Path, default=Path.home()/'_PaperWorks/HIMLoco')
    parser.add_argument('--parkour-root', type=Path, default=Path.home()/'_PaperWorks/extreme-parkour')
    args = parser.parse_args()
    deploy(args.assets, args.mujoco_root, args.him_root, args.parkour_root)
