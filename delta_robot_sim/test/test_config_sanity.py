from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLLERS = ROOT / "config" / "ros2_controllers.yaml"
SPAWN_LAUNCH = ROOT / "launch" / "delta_robot_spawn.launch.py"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_joint_trajectory_controller_has_gains_for_all_joints() -> None:
    text = _read(CONTROLLERS)
    joints = ["jbf1", "jbf2", "jbf3", "Bevelj1", "Bevelj2", "Tj1", "BeveljEE"]
    missing = [joint for joint in joints if f"{joint}: {{" not in text]
    assert not missing, f"Missing PID gains for joints: {missing}"


def test_spawn_launch_has_placeholder_and_file_sanity_checks() -> None:
    text = _read(SPAWN_LAUNCH)

    assert "Expected placeholder '__DELTA_ROBOT_SIM_SHARE__' not found" in text
    assert "Missing robot model SDF" in text
    assert "Missing box SDF" in text
    assert "Missing simulation world" in text
    assert "Missing Gazebo GUI config" in text
    assert "Missing ROS-Gazebo bridge config" in text
    assert "Missing RViz config" in text
