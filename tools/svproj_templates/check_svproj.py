#!/usr/bin/env python3
"""Exercise terrain alignment, every goal transition, resets, and vector steps.

Run with the project's conda Python and --task TASK --headless --num_envs 16.
"""
import json
from pathlib import Path
import isaacgym  # Isaac Gym must be imported before torch.
import numpy as np
import torch
import legged_gym.envs
from legged_gym import LEGGED_GYM_ROOT_DIR
from legged_gym.utils import get_args, task_registry

args = get_args()
if args.task not in ['him_go2w_svproj', 'go2w_svproj']:
    raise ValueError('This check requires a svproj task')
cfg, _ = task_registry.get_cfgs(args.task)
cfg.noise.add_noise = False
cfg.domain_rand.push_robots = False
if hasattr(cfg.domain_rand, 'disturbance'):
    cfg.domain_rand.disturbance = False
if hasattr(cfg.domain_rand, 'randomize_base_com'):
    cfg.domain_rand.randomize_base_com = False
cfg.domain_rand.randomize_base_mass = False
env, cfg = task_registry.make_env(args.task, args=args, env_cfg=cfg)
report = {'task':args.task, 'num_envs':env.num_envs}
try:
    env.reset()
    assert torch.isfinite(env.obs_buf).all()
    assert torch.max(torch.abs(env.root_states[:, :2]-env.env_origins[:, :2])) < .1
    with np.load(cfg.terrain.svproj_file.format(LEGGED_GYM_ROOT_DIR=LEGGED_GYM_ROOT_DIR), allow_pickle=False) as data:
        v = data['vertices']
        origin = data['grid_origin']
        size = data['tile_size']
        goals = data['goals']
        count = len(v)
        tile = 0
        for row in range(cfg.terrain.num_rows):
            for col in range(cfg.terrain.num_cols):
                shift = np.array([row*size[0]-origin[0], col*size[1]-origin[1], 0.])
                np.testing.assert_allclose(env.terrain.vertices[tile*count:(tile+1)*count], v+shift, atol=5e-6)
                np.testing.assert_allclose(env.terrain.goals[row,col], goals+shift, atol=5e-6)
                tile += 1
    report['tiles'] = tile
    report['height_scan_shape'] = list(env.height_samples.shape)
    # Place all robots at each goal. This also covers the last padded goal and
    # guards against stale target buffers while changing direction on a U turn.
    ids = torch.arange(env.num_envs, device=env.device)
    ids32 = ids.to(torch.int32)
    is_him = hasattr(env, 'course_goal_idx')
    all_goals = torch.as_tensor(env.terrain.goals, device=env.device)[env.terrain_levels, env.terrain_types]
    for k in range(cfg.terrain.num_goals):
        if is_him:
            env.course_goal_idx[:] = k
        else:
            env.cur_goal_idx[:] = k
            env.cur_goals = env._gather_cur_goals()
            env.next_goals = env._gather_cur_goals(future=1)
            env.reach_goal_timer[:] = cfg.env.reach_goal_delay/env.dt+1
        env.root_states[:, :2] = all_goals[:, k, :2]
        env.root_states[:, 7:13] = 0.
        if is_him:
            env._post_physics_step_callback()
            assert torch.all(env.course_goal_idx == k+1), k
        else:
            env._update_goals()
            assert torch.all(env.cur_goal_idx == k+1), k
    env.check_termination()
    assert env.reset_buf.all()
    env.reset_idx(ids)
    assert torch.all((env.course_goal_idx if is_him else env.cur_goal_idx) == 0)
    torch.testing.assert_allclose(env.root_states[:, :2], env.env_origins[:, :2])
    # Explicitly test leaving the tile instead of sampling a neighbouring tile.
    env.root_states[:, 0] = env.env_origins[:, 0] + 100
    env.check_termination()
    assert env.reset_buf.all()
    env.reset_idx(ids)
    total_resets = 0
    for step in range(300):
        actions = torch.zeros(env.num_envs, env.num_actions, device=env.device)
        obs, priv, rewards, resets, extras = env.step(actions)[:5]
        assert torch.isfinite(obs).all() and torch.isfinite(rewards).all(), step
        if priv is not None:
            assert torch.isfinite(priv).all(), step
        assert torch.isfinite(env.root_states).all(), step
        total_resets += int(resets.sum())
    report.update(passed=True, vector_steps=300, goal_transitions=cfg.terrain.num_goals,
                  observations=list(env.obs_buf.shape), resets_during_zero_actions=total_resets,
                  sampled_heights_min=float(env.measured_heights.min()),
                  sampled_heights_max=float(env.measured_heights.max()))
    out = Path(LEGGED_GYM_ROOT_DIR)/'resources/terrains/svproj_course/validation'
    out.mkdir(exist_ok=True)
    (out/'simulation_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print('SVPROJ_CHECK', json.dumps(report))
finally:
    env.gym.destroy_sim(env.sim)
