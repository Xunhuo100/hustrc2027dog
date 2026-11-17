"""Go2W Extreme Parkour teacher task with exact CAD triangle collisions."""
from legged_gym.envs.go2_w.go2w_parkour_config import Go2wParkourCfg, Go2wParkourCfgPPO
from legged_gym.envs.go2_w.go2w_env import Go2wRobot
from legged_gym.utils.svproj_terrain import SVProjSceneMixin


class ParkourSVProjCfg(Go2wParkourCfg):
    class env(Go2wParkourCfg.env):
        num_envs = 64
        episode_length_s = 120
        randomize_start_pos = False
        randomize_start_y = False
        randomize_start_yaw = False
        next_goal_threshold = .35

    class terrain(Go2wParkourCfg.terrain):
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

    class commands(Go2wParkourCfg.commands):
        curriculum = False
        class ranges(Go2wParkourCfg.commands.ranges):
            lin_vel_x = [.4, .7]

    class depth(Go2wParkourCfg.depth):
        use_camera = False


class ParkourSVProjCfgPPO(Go2wParkourCfgPPO):
    class runner(Go2wParkourCfgPPO.runner):
        experiment_name = 'parkour_go2w_svproj'
        save_interval = 100


class ParkourSVProjRobot(SVProjSceneMixin, Go2wRobot):
    pass
