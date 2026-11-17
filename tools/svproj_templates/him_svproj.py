"""Go2W HIM task on the converted svproj obstacle course."""
import torch
from isaacgym import gymtorch
from legged_gym.envs.go2_w.him_go2w_config import HIMGo2WRoughCfg, HIMGo2WRoughCfgPPO
from legged_gym.envs.go2_w.him_go2w_env import HIMGo2WRobot
from legged_gym.utils.svproj_terrain import SVProjSceneMixin


class HIMSVProjCfg(HIMGo2WRoughCfg):
    class env(HIMGo2WRoughCfg.env):
        num_envs = 64
        episode_length_s = 120

    class terrain(HIMGo2WRoughCfg.terrain):
        svproj_file = '{LEGGED_GYM_ROOT_DIR}/resources/terrains/svproj_course/terrain.npz'
        mesh_type = 'trimesh'
        horizontal_scale = .025
        vertical_scale = .001
        border_size = 0
        terrain_length = 14.
        terrain_width = 7.
        num_rows = 2
        num_cols = 2
        max_init_terrain_level = 0
        curriculum = False
        measure_heights = True
        num_goals = 10

    class commands(HIMGo2WRoughCfg.commands):
        curriculum = False
        heading_command = False
        resampling_time = 120.
        class ranges(HIMGo2WRoughCfg.commands.ranges):
            lin_vel_x = [.4, .7]
            lin_vel_y = [0., 0.]
            ang_vel_yaw = [-1., 1.]


class HIMSVProjCfgPPO(HIMGo2WRoughCfgPPO):
    class runner(HIMGo2WRoughCfgPPO.runner):
        experiment_name = 'him_go2w_svproj'
        num_steps_per_env = 24
        save_interval = 100


class HIMSVProjRobot(SVProjSceneMixin, HIMGo2WRobot):
    def _init_buffers(self):
        super()._init_buffers()
        self.course_goal_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.course_goals = torch.as_tensor(self.terrain.goals, device=self.device)
        self.course_speed = .55

    def reset_idx(self, env_ids):
        if len(env_ids):
            self.course_goal_idx[env_ids] = 0
        super().reset_idx(env_ids)

    def _reset_root_states(self, env_ids):
        # The parent's +/-1m reset can place a robot in the start frame.
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, :3] += self.env_origins[env_ids]
        self.root_states[env_ids, 7:13] = 0.
        ids = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.root_states), gymtorch.unwrap_tensor(ids), len(ids))

    def _post_physics_step_callback(self):
        super()._post_physics_step_callback()
        goals = self.course_goals[self.terrain_levels, self.terrain_types]
        ids = torch.arange(self.num_envs, device=self.device)
        index = self.course_goal_idx.clamp(max=goals.shape[1]-1)
        delta = goals[ids, index, :2] - self.root_states[:, :2]
        reached = torch.linalg.norm(delta, dim=1) < .35
        self.course_goal_idx += reached.long()
        index = self.course_goal_idx.clamp(max=goals.shape[1]-1)
        delta = goals[ids, index, :2] - self.root_states[:, :2]
        q = self.base_quat
        yaw = torch.atan2(2*(q[:, 3]*q[:, 2]+q[:, 0]*q[:, 1]),
                          1-2*(q[:, 1]**2+q[:, 2]**2))
        error = torch.atan2(torch.sin(torch.atan2(delta[:, 1], delta[:, 0])-yaw),
                            torch.cos(torch.atan2(delta[:, 1], delta[:, 0])-yaw))
        self.commands[:, 0] = self.course_speed * torch.cos(error).clamp(min=0.)
        self.commands[:, 1] = 0.
        self.commands[:, 2] = (1.5*error).clamp(-1., 1.)

    def check_termination(self):
        super().check_termination()
        complete = self.course_goal_idx >= self.cfg.terrain.num_goals
        self.time_out_buf |= complete
        self.reset_buf |= complete
