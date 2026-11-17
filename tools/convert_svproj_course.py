#!/usr/bin/env python3
"""Export the supplied STEP Viewer scene, not an invented articulated robot.

Conversion dependencies: numpy, scipy, coacd. The binary geometry in svproj is
already tessellated; no SolidWorks installation or STEP importer is required.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.spatial import ConvexHull


# Proper rotation (det=+1): CAD Y is up, CAD units are mm.
# Ground CAD Y=-75 mm becomes Z=0 m. X/Y stay in the original course layout.
ROTATION = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
ORIGIN = np.array([-2., -5.])
TILE_SIZE = np.array([14., 7.])
SPAWN = np.array([.5, .5, 0.])
# Suggested route; these points are authored here, not stored in the CAD file.
GOALS = np.array([[1.3, .5, 0], [4.9, .5, 0], [6.1, .5, 0],
                  [9.9, .5, 0], [10.5, -1.8, 0], [9.58, -3.6, 0],
                  [6.5, -3.6, 0], [3.5, -3.6, 0], [1.5, -3.6, 0],
                  [-.5, -3.6, 0]])


def read_array(binary, spec, dtype, columns):
    expected = np.dtype(dtype).itemsize * spec['count']
    if expected != spec['byteLength'] or spec['offset'] + expected > len(binary):
        raise ValueError('Invalid geometry buffer descriptor')
    return np.frombuffer(binary, dtype, count=spec['count'],
                         offset=spec['offset']).reshape(-1, columns).copy()


def read_project(path):
    with zipfile.ZipFile(path) as archive:
        manifest = json.loads(archive.read('manifest.json'))
        if manifest['format'] != 'step-viewer-project' or manifest['formatVersion'] != 1:
            raise ValueError('Unsupported svproj format')
        source = archive.read(manifest['source']['entry'])
        if hashlib.sha256(source).hexdigest() != manifest['source']['sha256']:
            raise ValueError('STEP source checksum mismatch')
        geometry = json.loads(archive.read(manifest['geometry']['manifestEntry']))
        binary = archive.read(manifest['geometry']['dataEntry'])
        state = json.loads(archive.read(manifest['stateEntry']))
    solids = []
    for index, solid in enumerate(geometry['solids']):
        vertices = read_array(binary, solid['arrays']['positions'], '<f4', 3).astype(float)
        faces = read_array(binary, solid['arrays']['indices'], '<u4', 3)
        if not np.isfinite(vertices).all() or faces.max() >= len(vertices):
            raise ValueError('Nonfinite vertices or invalid triangle indices')
        vertices = vertices @ ROTATION.T * .001
        vertices[:, 2] += .075
        # Weld duplicated face vertices before convex decomposition.
        vertices, inverse = np.unique(np.round(vertices, 7), axis=0, return_inverse=True)
        faces = inverse[faces]
        area = np.linalg.norm(np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                                      vertices[faces[:, 2]] - vertices[faces[:, 0]]), axis=1)
        faces = faces[area > 1.e-12]
        volume = np.sum(np.einsum('ij,ij->i', vertices[faces[:, 0]],
                        np.cross(vertices[faces[:, 1]], vertices[faces[:, 2]]))) / 6
        if volume < 0:
            faces = faces[:, [0, 2, 1]]
        solids.append((vertices, faces))
    robot = state.get('urdf', {}).get('robot', {})
    return solids, manifest, robot


def write_obj(path, vertices, faces):
    with path.open('w') as stream:
        for vertex in vertices:
            stream.write('v %.8f %.8f %.8f\n' % tuple(vertex))
        for face in faces:
            stream.write('f %d %d %d\n' % tuple(face + 1))


def heightmap(solids, horizontal_scale):
    """Rasterize highest triangle intersection, keeping ring holes open.

    This is the policy height scan, not the exact collision mesh. Vertical faces
    and undersides cannot be represented in a height scan.
    """
    shape = np.rint(TILE_SIZE / horizontal_scale).astype(int) + 1
    heights = np.zeros(tuple(shape), np.float64)
    for vertices, faces in solids:
        for face in faces:
            triangle = vertices[face]
            lower = np.maximum(0, np.ceil((triangle[:, :2].min(0)-ORIGIN)/horizontal_scale-1.e-6).astype(int))
            upper = np.minimum(shape-1, np.floor((triangle[:, :2].max(0)-ORIGIN)/horizontal_scale+1.e-6).astype(int))
            if np.any(lower > upper):
                continue
            a, b, c = triangle
            denominator = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
            if abs(denominator) < 1.e-12:
                continue
            xx, yy = np.meshgrid(ORIGIN[0]+np.arange(lower[0],upper[0]+1)*horizontal_scale,
                                 ORIGIN[1]+np.arange(lower[1],upper[1]+1)*horizontal_scale, indexing='ij')
            wa = ((b[1]-c[1])*(xx-c[0]) + (c[0]-b[0])*(yy-c[1])) / denominator
            wb = ((c[1]-a[1])*(xx-c[0]) + (a[0]-c[0])*(yy-c[1])) / denominator
            wc = 1-wa-wb
            inside = (wa >= -1.e-6) & (wb >= -1.e-6) & (wc >= -1.e-6)
            zz = np.where(inside, wa*a[2]+wb*b[2]+wc*c[2], 0)
            part = heights[lower[0]:upper[0]+1, lower[1]:upper[1]+1]
            np.maximum(part, zz, out=part)
    return np.rint(heights / .001).astype(np.int16)


def convex_parts(vertices, faces, cache):
    import coacd
    center = (vertices.max(0) + vertices.min(0)) / 2
    local = vertices - center
    for old_local, old_parts in cache:
        if local.shape == old_local.shape and np.max(np.abs(local-old_local)) < 2.e-5:
            return [(v+center, f) for v,f in old_parts]
    triangle = local[faces]
    volume = abs(np.sum(np.einsum('ij,ij->i', triangle[:,0],
                 np.cross(triangle[:,1],triangle[:,2]))) / 6)
    hull = ConvexHull(local)
    if abs(hull.volume-volume) < max(1.e-9, hull.volume*1.e-4):
        # Original triangles on a convex solid need no approximation.
        parts = [(local, faces)]
    else:
        coacd.set_log_level('warn')
        parts = coacd.run_coacd(coacd.Mesh(local, faces), threshold=.015,
                 preprocess_resolution=40, resolution=2000, mcts_iterations=60,
                 max_convex_hull=64, seed=0)
    cache.append((local, parts))
    return [(v+center, f) for v,f in parts]


def export(source, output, scale):
    output.mkdir(parents=True, exist_ok=True)
    (output/'visual').mkdir(exist_ok=True)
    (output/'collision').mkdir(exist_ok=True)
    solids, project, robot = read_project(source)
    vertices, faces, offset = [], [], 0
    assets = ET.Element('mujocoinclude')
    asset = ET.SubElement(assets, 'asset')
    world = ET.SubElement(assets, 'worldbody')
    cache, summary = [], []
    for i, (v,f) in enumerate(solids):
        write_obj(output/'visual'/('solid_%02d.obj'%i), v, f)
        name = 'svproj_visual_%02d'%i
        ET.SubElement(asset,'mesh',name=name,file='svproj_course/visual/solid_%02d.obj'%i)
        ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0',
                      group='2',rgba='0.42 0.55 0.65 1')
        parts = convex_parts(v,f,cache)
        for j, (cv,cf) in enumerate(parts):
            name = 'svproj_collision_%02d_%02d'%(i,j)
            file = 'collision/solid_%02d_part_%02d.obj'%(i,j)
            write_obj(output/file,cv,np.asarray(cf))
            ET.SubElement(asset,'mesh',name=name,file='svproj_course/'+file)
            ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,group='3',rgba='0 0 0 0',
                          friction='0.8 0.02 0.001',condim='3')
        vertices.append(v); faces.append(f+offset); offset += len(v)
        summary.append({'id':i,'vertices':len(v),'triangles':len(f),
                        'bounds_m':[v.min(0).tolist(),v.max(0).tolist()],
                        'convex_parts':len(parts)})
        print('solid %02d: %d triangles, %d collision parts'%(i,len(f),len(parts)),flush=True)
    ET.indent(assets)
    ET.ElementTree(assets).write(output/'course.xml',encoding='utf-8',xml_declaration=True)
    # Add ground only once to the exact triangle mesh for PhysX.
    ground = np.array([[-2,-5,0],[12,-5,0],[12,2,0],[-2,2,0]],float)
    vertices.append(ground); faces.append(np.array([[0,1,2],[0,2,3]])+offset)
    all_vertices=np.vstack(vertices).astype(np.float32)
    all_faces=np.vstack(faces).astype(np.uint32)
    raw=heightmap(solids,scale)
    np.savez_compressed(output/'terrain.npz',vertices=all_vertices,triangles=all_faces,
        heightsamples=raw,horizontal_scale=scale,vertical_scale=.001,
        grid_origin=ORIGIN,tile_size=TILE_SIZE,spawn=SPAWN,goals=GOALS)
    metadata={'input':str(source.name),'input_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'step_sha256':project['source']['sha256'],'kind':'static_obstacle_course',
              'source_joint_count':len(robot.get('joints',[])),
              'source_link_count':len(robot.get('links',[])),
              'units':'metres','transform':'x=cad_x/1000; y=-cad_z/1000; z=(cad_y+75)/1000',
              'grid_origin':ORIGIN.tolist(),'tile_size':TILE_SIZE.tolist(),
              'spawn':SPAWN.tolist(),'goals_are_authored':True,'goals':GOALS.tolist(),
              'horizontal_scale':scale,'vertical_scale':.001,
              'mujoco_collision':'CoACD convex decomposition; threshold=0.015, seed=0',
              'isaacgym_collision':'original triangles + added ground',
              'solids':summary}
    (output/'manifest.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    print('Exported',len(solids),'solids,',len(all_faces),'triangles to',output,flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,default=Path('assets/svproj_course'))
    parser.add_argument('--horizontal-scale',type=float,default=.025)
    args=parser.parse_args()
    if args.horizontal_scale <= 0:
        parser.error('horizontal-scale must be positive')
    export(args.source,args.output,args.horizontal_scale)
