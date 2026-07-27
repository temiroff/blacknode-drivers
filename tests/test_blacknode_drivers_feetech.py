import math
import runpy
from pathlib import Path
from types import SimpleNamespace

import pytest

import blacknode  # noqa: F401 - triggers extension package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import (
    _PACKAGE_REGISTRY,
    adapter_dependency_plan,
    set_adapter_enabled,
    set_component_enabled,
)
from blacknode.pkg.blacknode_drivers.feetech import bus


def test_feetech_component_registers_expected_nodes():
    info = _PACKAGE_REGISTRY["blacknode-drivers"]

    assert info.ok
    assert info.layer == "drivers"
    assert info.component_mode is True
    assert info.enabled_components == ["feetech"]
    assert info.components["feetech"]["capabilities"] == [
        "driver.feetech",
        "driver.serial-servo",
        "robot.joint-driver",
    ]
    assert info.pip_dependencies == ["feetech-servo-sdk>=1.0"]
    assert {"FeetechBusConfig", "FeetechBusProbe"}.issubset(info.node_types)
    assert _NODE_REGISTRY["FeetechBusConfig"]._bn_component == "feetech"
    assert info.components["feetech"]["adapters"]["ros2"]["enabled"] is False


def test_feetech_ros2_adapter_resolves_layer_dependencies_and_stays_disarmed():
    plan = adapter_dependency_plan("blacknode-drivers", "feetech", "ros2")
    assert [(item["package"], item["component"], item.get("adapter", "")) for item in plan["plan"]] == [
        ("blacknode-drivers", "feetech", ""),
        ("blacknode-ros2", "core", ""),
        ("blacknode-drivers", "feetech", "ros2"),
    ]

    try:
        info = set_adapter_enabled("blacknode-drivers", "feetech", "ros2", True)
        assert "FeetechROS2Adapter" in info.node_types
        assert info.enabled_components == ["feetech"]
        assert info.enabled_adapters == ["feetech/ros2"]
        result = _NODE_REGISTRY["FeetechROS2Adapter"]({"config": {"port": "COM7"}})
        assert result["adapter"]["available"] is True
        assert result["adapter"]["motion_armed"] is False
        assert result["adapter"]["config"] == {"port": "COM7"}
        with pytest.raises(ValueError, match="blacknode-drivers/feetech adapter ros2"):
            set_component_enabled("blacknode-ros2", "core", False)
    finally:
        set_adapter_enabled("blacknode-drivers", "feetech", "ros2", False)


def test_feetech_config_is_inert_and_accepts_robot_profile():
    profile = {
        "id": "test_arm",
        "joints": [
            {
                "id": "shoulder",
                "servo_id": 1,
                "min_deg": -90,
                "max_deg": 90,
                "safe_min_deg": -75,
                "safe_max_deg": 70,
                "home_ticks": 2100,
                "invert": True,
            }
        ],
    }

    result = _NODE_REGISTRY["FeetechBusConfig"]({
        "profile": profile,
        "port": "COM7",
        "baudrate": 1_000_000,
    })

    assert result["ready"] is True
    assert result["config"]["driver"] == "feetech"
    assert result["config"]["motion_armed"] is False
    assert result["config"]["joints"] == [{
        "name": "shoulder",
        "servo_id": 1,
        "min_deg": -75.0,
        "max_deg": 70.0,
        "home_ticks": 2100,
        "invert": True,
    }]
    assert "configuration opens no hardware" in result["report"]


def test_feetech_probe_requires_explicit_confirmation(monkeypatch):
    monkeypatch.setattr(
        bus,
        "probe_bus",
        lambda _config: (_ for _ in ()).throw(AssertionError("hardware must stay closed")),
    )

    result = _NODE_REGISTRY["FeetechBusProbe"]({
        "config": {"port": "COM7"},
        "confirm_read_only": False,
    })

    assert result["connected"] is False
    assert "BLOCKED" in result["report"]


def test_joint_parsing_conversion_and_validation():
    joints = bus.parse_joint_map(
        "shoulder:1:-90:90,gripper:6:-10:80",
        {"shoulder": 2100},
        {"shoulder"},
    )
    shoulder = joints["shoulder"]

    assert shoulder.home_ticks == 2100
    assert shoulder.invert is True
    assert bus.ticks_to_degrees(shoulder.home_ticks, shoulder) == 0.0
    assert bus.degrees_to_ticks(10.0, shoulder) < shoulder.home_ticks
    assert bus.clamp_degrees(999.0, shoulder) == 90.0
    assert math.isclose(
        bus.ticks_to_degrees(bus.degrees_to_ticks(45.0, joints["gripper"]), joints["gripper"]),
        45.0,
    )

    with pytest.raises(ValueError, match="duplicated"):
        bus.parse_joint_map("a:1:-10:10,b:1:-10:10")
    with pytest.raises(ValueError, match="minimum"):
        bus.parse_joint_map("a:1:10:-10")


def test_read_only_probe_never_calls_write_methods():
    calls = []

    class Port:
        def openPort(self):
            calls.append("open")
            return True

        def setBaudRate(self, baudrate):
            calls.append(("baud", baudrate))
            return True

        def closePort(self):
            calls.append("close")

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, address):
            calls.append(("read", servo_id, address))
            return 2048 + servo_id, 0, 0

    fake_sdk = SimpleNamespace(
        COMM_SUCCESS=0,
        PortHandler=lambda _name: Port(),
        PacketHandler=lambda _protocol: Packet(),
    )
    config = {
        "port": "COM7",
        "baudrate": 1_000_000,
        "joints": [
            bus.JointSpec("shoulder", 1, -90, 90).to_dict(),
            bus.JointSpec("gripper", 6, -10, 80).to_dict(),
        ],
    }

    result = bus.probe_bus(config, sdk=fake_sdk)

    assert result["ok"] is True
    assert set(result["readings"]) == {"shoulder", "gripper"}
    assert calls == [
        "open",
        ("baud", 1_000_000),
        ("read", 1, bus.ADDR_PRESENT_POSITION[0]),
        ("read", 6, bus.ADDR_PRESENT_POSITION[0]),
        "close",
    ]


def test_torque_enable_reads_and_seeds_every_joint_before_holding():
    joints = bus.parse_joint_map("shoulder:1:-90:90,gripper:6:-10:80")
    calls = []

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, _address):
            calls.append(("read", servo_id))
            return 2000 + servo_id, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, ticks):
            calls.append(("goal", servo_id, ticks))
            return 0, 0

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("torque", servo_id, enabled))
            return 0, 0

    ok, positions, error = bus.enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is True
    assert error == ""
    assert positions == {"shoulder": 2001, "gripper": 2006}
    assert calls == [
        ("read", 1),
        ("read", 6),
        ("goal", 1, 2001),
        ("goal", 6, 2006),
        ("torque", 1, 1),
        ("torque", 6, 1),
    ]


def test_partial_seed_failure_returns_all_joints_to_torque_off():
    joints = bus.parse_joint_map("shoulder:1:-90:90,gripper:6:-10:80")
    torque_writes = []

    class Packet:
        def read2ByteTxRx(self, _port, _servo_id, _address):
            return 2048, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, _ticks):
            return (1, 0) if servo_id == 6 else (0, 0)

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            torque_writes.append((servo_id, enabled))
            return 0, 0

    ok, _positions, error = bus.enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is False
    assert "could not seed Goal_Position for gripper" in error
    assert torque_writes == [(1, 0), (6, 0)]


def test_driver_startup_disables_torque_before_reading_pose():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joints = runtime["parse_joint_map"](
        "shoulder:1:-90:90,gripper:6:-10:80",
        {},
        set(),
    )
    calls = []

    class Packet:
        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("torque", servo_id, enabled))
            return 0, 0

        def read2ByteTxRx(self, _port, servo_id, _address):
            calls.append(("read", servo_id))
            return 2000 + servo_id, 0, 0

    ok, positions, error = runtime["_prepare_released_startup"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        joints,
    )

    assert ok is True
    assert positions == {"shoulder": 2001, "gripper": 2006}
    assert error == ""
    assert calls == [
        ("torque", 1, 0),
        ("torque", 6, 0),
        ("read", 1),
        ("read", 6),
    ]


def test_driver_goal_seed_accepts_readback_after_lost_write_ack():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    calls = []

    class Packet:
        def write2ByteTxRx(self, _port, servo_id, address, ticks):
            calls.append(("write", servo_id, address, ticks))
            return 1, 0

        def read2ByteTxRx(self, _port, servo_id, address):
            calls.append(("readback", servo_id, address))
            return 2048, 0, 0

    seeded = runtime["_write_goal"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        5,
        2048,
        confirm=True,
    )

    assert seeded is True
    assert calls == [
        ("write", 5, runtime["ADDR_GOAL_POSITION"][0], 2048),
        ("readback", 5, runtime["ADDR_GOAL_POSITION"][0]),
    ]


def test_driver_goal_seed_retries_then_fails_without_confirmation():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    writes = []

    class Packet:
        def write2ByteTxRx(self, _port, servo_id, _address, ticks):
            writes.append((servo_id, ticks))
            return 1, 0

        def read2ByteTxRx(self, _port, _servo_id, _address):
            return 1024, 0, 0

    seeded = runtime["_write_goal"](
        SimpleNamespace(COMM_SUCCESS=0),
        Packet(),
        object(),
        5,
        2048,
        confirm=True,
    )

    assert seeded is False
    assert writes == [(5, 2048), (5, 2048), (5, 2048)]


def test_driver_ros_node_name_is_unique_per_robot_topic_namespace():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    node_name = runtime["ros_node_name"]

    assert node_name("/leader/joint_states") == "blacknode_feetech_bus_driver_leader"
    assert node_name("/follower/joint_states") == "blacknode_feetech_bus_driver_follower"
    assert node_name("/joint_states") == "blacknode_feetech_bus_driver"
    assert node_name("/ignored", "arm 1/driver") == "arm_1_driver"
    assert node_name("/ignored", "123-driver") == "blacknode_123_driver"


def test_read_only_driver_advertises_no_command_authority():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)

    config = runtime["_config_payload"](
        {"shoulder": joint},
        torque_enabled=True,
        commands_allowed=False,
    )

    assert config["torque_enabled"] is True
    assert config["commands_allowed"] is False


def test_position_writes_are_clamped_at_driver_boundary():
    joints = bus.parse_joint_map("shoulder:1:-30:40")
    writes = []
    packet = SimpleNamespace(
        write2ByteTxRx=lambda _port, servo_id, _address, ticks: writes.append(
            (servo_id, ticks)
        ) or (0, 0)
    )

    ok, error = bus.write_joint_positions(
        SimpleNamespace(COMM_SUCCESS=0),
        packet,
        object(),
        joints,
        {"shoulder": 999.0},
    )

    assert ok is True
    assert error == ""
    assert writes == [(1, bus.degrees_to_ticks(40.0, joints["shoulder"]))]


def test_deployed_driver_reports_read_only_joint_and_torque_telemetry():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)
    published = []

    class Publisher:
        def publish_robot_state(self, positions, **metadata):
            published.append((positions, metadata))

    runtime["_publish_deployment_state"](
        Publisher(),
        {"shoulder": 2048},
        {"shoulder": joint},
        {"torque_enabled": False, "last_error": ""},
    )

    assert published == [(
        {"shoulder": 0.0},
        {
            "torque_enabled": False,
            "connected": True,
            "position_unit": "degree",
            "error": "",
            "joint_limits": {"shoulder": (-90.0, 90.0)},
        },
    )]


def test_deployed_driver_keeps_publishing_with_legacy_runtime():
    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "components"
        / "feetech"
        / "adapters"
        / "ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    runtime = runpy.run_path(str(runtime_path))
    joint = runtime["JointSpec"]("shoulder", 1, -90.0, 90.0)
    published = []

    class LegacyPublisher:
        def publish_robot_state(
            self,
            positions,
            *,
            torque_enabled,
            connected,
            position_unit,
            error,
        ):
            published.append((
                positions,
                torque_enabled,
                connected,
                position_unit,
                error,
            ))

    runtime["_publish_deployment_state"](
        LegacyPublisher(),
        {"shoulder": 2048},
        {"shoulder": joint},
        {"torque_enabled": False, "last_error": ""},
    )

    assert published == [
        ({"shoulder": 0.0}, False, True, "degree", ""),
    ]
