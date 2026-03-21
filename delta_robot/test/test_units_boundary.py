from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "delta_config.yaml"
KIN_CPP_PATH = ROOT / "src" / "kinematics.cpp"
KIN_HPP_PATH = ROOT / "include" / "delta_robot" / "kinematics.hpp"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_cartesian_units_parameter_is_present_in_config() -> None:
    text = _read(CONFIG_PATH)
    assert "cartesian_units:" in text


def test_kinematics_declares_and_reads_cartesian_units_parameter() -> None:
    text = _read(KIN_CPP_PATH)
    assert 'declare_parameter("cartesian_units", "mm")' in text
    assert 'cartesian_units = this->get_parameter("cartesian_units").as_string()' in text


def test_kinematics_has_boundary_conversion_helpers() -> None:
    hpp = _read(KIN_HPP_PATH)
    cpp = _read(KIN_CPP_PATH)
    assert "Point toInternalUnits" in hpp
    assert "Point fromInternalUnits" in hpp
    assert "bool DeltaKinematics::isCartesianUnitsSupported" in cpp
    assert "Point DeltaKinematics::toInternalUnits" in cpp
    assert "Point DeltaKinematics::fromInternalUnits" in cpp


def test_boundary_conversion_used_in_fk_ik_paths() -> None:
    text = _read(KIN_CPP_PATH)
    assert "this->toInternalUnits(request->solution)" in text
    assert "this->fromInternalUnits(position)" in text
    assert "this->toInternalUnits(point)" in text
