import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src/wheel_leg_mujoco/models/go2w/urdf"
sys.path.insert(0, str(ROOT / "src/himloco_policy"))
sys.path.insert(0, str(ROOT / "src/wheel_leg_mujoco"))
from himloco_policy.policy_core import HimPolicyCore, DEFAULT_POSITION, projected_gravity


def observe(core, **changes):
    values = dict(position=DEFAULT_POSITION, velocity=np.zeros(16),
                  quaternion=[1, 0, 0, 0], angular_velocity=[0, 0, 0], command=[0, 0, 0])
    values.update(changes)
    return core.observe(**values).reshape(6, 57)


def test_observation_contract_and_wheel_mask():
    core = HimPolicyCore()
    position = np.asarray(DEFAULT_POSITION) + 0.2
    position[3::4] = 10000
    history = observe(core, position=position, velocity=np.arange(16),
                      angular_velocity=[1, 2, 3], command=[0.5, -0.2, 0.8])
    np.testing.assert_allclose(history[0, :9], [1, -0.4, 0.2, 0.25, 0.5, 0.75, 0, 0, -1])
    expected = np.full(16, 0.2)
    expected[3::4] = 0
    np.testing.assert_allclose(history[0, 9:25], expected, atol=1e-7)
    np.testing.assert_allclose(history[0, 25:41], np.arange(16) * 0.05)
    assert np.count_nonzero(history[0, 41:]) == 0
    assert np.count_nonzero(history[1:]) == 0


def test_history_newest_first_and_session_reset():
    core = HimPolicyCore()
    first = observe(core, command=[0.1, 0, 0])[0].copy()
    core.action_to_targets(np.ones(16))
    second = observe(core, command=[0.2, 0, 0])
    np.testing.assert_array_equal(second[1], first)
    np.testing.assert_array_equal(second[0, 41:], np.ones(16))
    core.reset()
    assert np.count_nonzero(core.history) == 0
    assert np.count_nonzero(core.last_action) == 0


def test_gravity_uses_inverse_rotation_wxyz():
    root = np.sqrt(0.5)
    np.testing.assert_allclose(projected_gravity([1, 0, 0, 0]), [0, 0, -1])
    np.testing.assert_allclose(projected_gravity([root, 0, root, 0]), [1, 0, 0], atol=2e-7)
    np.testing.assert_allclose(projected_gravity([root, root, 0, 0]), [0, -1, 0], atol=2e-7)


def test_separate_leg_position_and_wheel_velocity_targets():
    core = HimPolicyCore()
    action = np.zeros(16)
    action[0] = 1.0
    action[3::4] = [1, -2, 3, -4]
    position, velocity, kp, kd = core.action_to_targets(action)
    assert position[0] == pytest.approx(0.35)
    np.testing.assert_array_equal(position[3::4], np.zeros(4))
    np.testing.assert_array_equal(velocity[3::4], [10, -20, 30, -40])
    np.testing.assert_array_equal(kp[3::4], np.zeros(4))
    np.testing.assert_allclose(kd[3::4], np.full(4, 0.3))


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_input_is_rejected(value):
    core = HimPolicyCore()
    with pytest.raises(ValueError):
        observe(core, angular_velocity=[value, 0, 0])
    with pytest.raises(ValueError):
        core.action_to_targets([value] * 16)


def test_invalid_dimensions_and_quaternion_are_rejected():
    with pytest.raises(ValueError):
        HimPolicyCore({"history_length": 5})
    with pytest.raises(ValueError):
        observe(HimPolicyCore(), quaternion=[0, 0, 0, 0])
    with pytest.raises(ValueError):
        HimPolicyCore().action_to_targets(np.zeros(12))


def test_targets_respect_go2w_joint_and_wheel_limits():
    core = HimPolicyCore()
    q, dq, kp, kd = core.action_to_targets(np.full(16, 1e6))
    assert q[0] <= 1.0472
    assert q[1] <= 3.4907
    assert q[9] <= 4.5379
    assert q[2] <= -0.83776
    assert np.max(np.abs(dq)) <= 100
    np.testing.assert_array_equal(core.last_action, np.full(16, 100))


@pytest.mark.parametrize("scene", ["scene_flat.xml", "scene_fields.xml"])
def test_mujoco_model_mapping_and_freefall_imu(scene):
    pytest.importorskip("mujoco")
    from wheel_leg_mujoco.mujoco_backend import MujocoRobot
    xml = MODEL_DIR / scene
    robot = MujocoRobot(str(xml))
    assert len(set(robot.actuator_ids)) == 16
    np.testing.assert_allclose(robot.joint_state()[0], DEFAULT_POSITION)
    for _ in range(5):
        robot.step(np.zeros((16, 6)))
    np.testing.assert_allclose(robot.imu_state()[2], np.zeros(3), atol=1e-6)


@pytest.mark.parametrize("quaternion", [[1, 0, 0, 0], [0.5, 0.5, 0.5, 0.5]])
def test_mujoco_gyro_uses_base_axes_instead_of_inertia_axes(quaternion):
    mujoco = pytest.importorskip("mujoco")
    from wheel_leg_mujoco.mujoco_backend import MujocoRobot
    xml = MODEL_DIR / "scene_flat.xml"
    robot = MujocoRobot(str(xml))
    robot.data.qpos[3:7] = quaternion
    # MuJoCo free joint 的后三个速度就是机体坐标系角速度。
    robot.data.qvel[3:6] = [0.7, -0.4, 0.2]
    mujoco.mj_forward(robot.model, robot.data)
    q, gyro, acceleration = robot.imu_state()
    np.testing.assert_allclose(q, quaternion)
    np.testing.assert_allclose(gyro, [0.7, -0.4, 0.2], atol=1e-12)


def test_parkour_profile_and_checkpoint_match_training():
    torch = pytest.importorskip("torch")
    yaml = pytest.importorskip("yaml")
    package = ROOT / "src/himloco_policy"
    profiles = {}
    for package_name, node in [("robot_ctrl", "robot_ctrl"), ("himloco_policy", "himloco_policy"),
                               ("wheel_leg_mujoco", "go2w_mujoco")]:
        profile_path = ROOT / "src" / package_name / "config/parkour.yaml"
        profiles[node] = yaml.safe_load(profile_path.read_text())[node]
    expected = [0.0, 0.8, -1.5, 0.0] * 4
    for name in ["robot_ctrl", "himloco_policy", "go2w_mujoco"]:
        assert profiles[name]["ros__parameters"]["default_position"] == expected
    for name in ["robot_ctrl", "himloco_policy"]:
        parameters = profiles[name]["ros__parameters"]
        assert [parameters[k] for k in ["leg_kp", "leg_kd", "wheel_kd"]] == [40.0, 1.0, 0.5]
    checkpoint = (Path.home() / "_PaperWorks/HIMLoco/legged_gym/logs/him_go2w_parkour"
                  / "Oct24_15-45-13_/model_13500.pt")
    if not checkpoint.is_file():
        pytest.skip("Local training checkpoint missing")
    weights = torch.load(checkpoint, map_location="cpu", weights_only=False)["model_state_dict"]
    model = torch.jit.load(str(package / "policies/parkour.pt"), map_location="cpu")
    for name, value in model.state_dict().items():
        training_name = name
        if name.startswith("estimator."):
            training_name = name.replace("estimator.", "estimator.encoder.", 1)
        torch.testing.assert_close(value, weights[training_name], rtol=0, atol=0)


def test_bundled_torchscript_accepts_real_observation():
    torch = pytest.importorskip("torch")
    model = torch.jit.load(str(ROOT / "src/himloco_policy/policies/policy.pt"), map_location="cpu").eval()
    history = observe(HimPolicyCore()).reshape(1, 342)
    with torch.no_grad():
        result = model(torch.from_numpy(history))
    assert result.shape == (1, 16)
    assert torch.isfinite(result).all()
