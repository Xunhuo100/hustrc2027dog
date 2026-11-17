"""Exact CAD triangle collisions with a separate policy height scan (metres)."""
from pathlib import Path
import numpy as np
from legged_gym import LEGGED_GYM_ROOT_DIR


class SVProjTerrain:
    def __init__(self, cfg, num_envs):
        self.cfg = cfg
        path = Path(cfg.svproj_file.format(LEGGED_GYM_ROOT_DIR=LEGGED_GYM_ROOT_DIR))
        with np.load(path, allow_pickle=False) as data:
            vertices = data['vertices'].copy()
            triangles = data['triangles'].copy()
            heights = data['heightsamples'].copy()
            self.grid_origin = data['grid_origin'].copy()
            self.source_spawn = data['spawn'].copy()
            size = data['tile_size'].copy()
            goals = data['goals'].copy()
            hscale, vscale = float(data['horizontal_scale']), float(data['vertical_scale'])
        if not np.isclose(cfg.horizontal_scale, hscale) or not np.isclose(cfg.vertical_scale, vscale):
            raise ValueError('svproj terrain scale must match terrain.npz')
        if cfg.border_size != 0 or cfg.mesh_type != 'trimesh':
            raise ValueError('svproj requires trimesh and border_size=0')
        if not np.allclose(size, [cfg.terrain_length, cfg.terrain_width]):
            raise ValueError('svproj tile dimensions must match terrain.npz')
        if triangles.max() >= len(vertices) or not np.isfinite(vertices).all():
            raise ValueError('Invalid svproj triangle mesh')
        if hasattr(cfg, 'num_goals') and cfg.num_goals != len(goals):
            raise ValueError('svproj num_goals must match terrain.npz')
        rows, cols = int(cfg.num_rows), int(cfg.num_cols)
        if min(rows, cols) < 1:
            raise ValueError('svproj tile counts must be positive')
        nx, ny = np.rint(size / hscale).astype(int)
        if heights.shape != (nx + 1, ny + 1):
            raise ValueError('Unexpected svproj height scan dimensions')
        self.env_length, self.env_width = size
        self.env_origins = np.zeros((rows, cols, 3), dtype=np.float32)
        self.goals = np.zeros((rows, cols, len(goals), 3), dtype=np.float32)
        self.terrain_type = np.zeros((rows, cols), dtype=np.float32)
        self.heightsamples = np.zeros((rows * nx + 1, cols * ny + 1), dtype=np.int16)
        vv, ff = [], []
        for row in range(rows):
            for col in range(cols):
                offset = np.array([row * size[0] - self.grid_origin[0],
                                   col * size[1] - self.grid_origin[1], 0.], dtype=np.float32)
                vv.append(vertices + offset)
                ff.append(triangles + (row * cols + col) * len(vertices))
                self.env_origins[row, col] = self.source_spawn + offset
                self.goals[row, col] = goals + offset
                self.heightsamples[row*nx:(row+1)*nx+1, col*ny:(col+1)*ny+1] = heights
        self.vertices = np.ascontiguousarray(np.concatenate(vv), dtype=np.float32)
        self.triangles = np.ascontiguousarray(np.concatenate(ff), dtype=np.uint32)
        self.height_field_raw = self.heightsamples
        self.tot_rows, self.tot_cols = self.heightsamples.shape
        # Used by Extreme Parkour's feet-edge reward, independent of collisions.
        self.x_edge_mask = np.zeros_like(self.heightsamples, dtype=bool)
        edges = np.abs(np.diff(self.heightsamples.astype(np.int32), axis=0)) * vscale > .06
        self.x_edge_mask[:-1] |= edges
        self.x_edge_mask[1:] |= edges


class SVProjSceneMixin:
    """Use this terrain without changing either framework's existing tasks."""
    def create_sim(self):
        self.up_axis_idx = 2
        if getattr(getattr(self.cfg, 'depth', None), 'use_camera', False):
            self.graphics_device_id = self.sim_device_id
        self.sim = self.gym.create_sim(self.sim_device_id, self.graphics_device_id,
                                       self.physics_engine, self.sim_params)
        if self.sim is None:
            raise RuntimeError('Isaac Gym failed to create simulation')
        self.terrain = SVProjTerrain(self.cfg.terrain, self.num_envs)
        self._create_trimesh()
        self._create_envs()

    def check_termination(self):
        super().check_termination()
        # Base frameworks normally assume straight lanes; this course makes a U turn.
        local = self.root_states[:, :2] - self.env_origins[:, :2]
        lower = self.terrain.grid_origin - self.terrain.source_spawn[:2]
        outside = ((local[:, 0] < float(lower[0]) + .1)
                   | (local[:, 0] > float(lower[0]) + self.terrain.env_length - .1)
                   | (local[:, 1] < float(lower[1]) + .1)
                   | (local[:, 1] > float(lower[1]) + self.terrain.env_width - .1))
        self.reset_buf |= outside
