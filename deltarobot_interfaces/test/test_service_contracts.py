from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRV_DIR = ROOT / "srv"
MSG_DIR = ROOT / "msg"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_convert_to_joint_trajectory_has_structured_status_fields() -> None:
    text = _read(SRV_DIR / "ConvertToJointTrajectory.srv")
    assert "bool success" in text
    assert "int32 error_code" in text
    assert "string error_message" in text


def test_convert_to_joint_vel_trajectory_has_structured_status_fields() -> None:
    text = _read(SRV_DIR / "ConvertToJointVelTrajectory.srv")
    assert "bool success" in text
    assert "int32 error_code" in text
    assert "string error_message" in text


def test_move_services_have_error_fields() -> None:
    move_to_point = _read(SRV_DIR / "MoveToPoint.srv")
    move_to_configuration = _read(SRV_DIR / "MoveToConfiguration.srv")

    assert "int32 error_code" in move_to_point
    assert "string error_message" in move_to_point
    assert "int32 error_code" in move_to_configuration
    assert "string error_message" in move_to_configuration


def test_play_demo_trajectory_uses_enum_like_demo_type() -> None:
    text = _read(SRV_DIR / "PlayDemoTrajectory.srv")
    assert "uint8 DEMO_UP_DOWN=0" in text
    assert "uint8 DEMO_PRINGLE=1" in text
    assert "uint8 DEMO_AXES=2" in text
    assert "uint8 DEMO_CIRCLE=3" in text
    assert "uint8 DEMO_SCAN=4" in text
    assert "uint8 demo_type" in text


def test_robot_config_and_fk_ik_reference_cartesian_units() -> None:
    robot_config = _read(MSG_DIR / "RobotConfig.msg")
    delta_fk = _read(SRV_DIR / "DeltaFK.srv")
    delta_ik = _read(SRV_DIR / "DeltaIK.srv")

    assert "cartesian_units" in robot_config
    assert "cartesian_units" in delta_fk
    assert "cartesian_units" in delta_ik
