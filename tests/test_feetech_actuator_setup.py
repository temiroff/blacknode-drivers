from types import SimpleNamespace

import pytest
import blacknode  # noqa: F401
from blacknode.pkg.blacknode_drivers.feetech import actuator_setup as setup


class FakeBus:
    def __init__(self, ids=(1,)):
        self.devices = {i: {3: 777, 5: i, 6: 0, 9: 0, 11: 4095, 16: 1000, 33: 0,
                            40: 0, 55: 1, 56: 2048, 62: 120, 63: 25, 65: 0} for i in ids}
        self.writes = []
        self.closed = False
        self.lose_id_ack = False
        self.fail_lock = False
        self.corrupt_id = None
        self.fail_id_write = False
        self.sdk = SimpleNamespace(COMM_SUCCESS=0, COMM_RX_TIMEOUT=-6,
                                   PortHandler=lambda name: self, PacketHandler=lambda protocol: self)

    def openPort(self): return True
    def setBaudRate(self, rate): return True
    def closePort(self): self.closed = True
    def ping(self, port, servo_id):
        if servo_id == self.corrupt_id: return 0, -7, 0
        return (self.devices[servo_id][3], 0, 0) if servo_id in self.devices else (0, -6, 0)
    def read1ByteTxRx(self, port, servo_id, address):
        return (self.devices[servo_id][address], 0, 0) if servo_id in self.devices else (0, -6, 0)
    read2ByteTxRx = read1ByteTxRx
    def write1ByteTxRx(self, port, servo_id, address, value):
        self.writes.append((servo_id, address, value))
        if address == 55 and value == 1 and self.fail_lock: return -6, 0
        if address == 5 and self.fail_id_write: return -6, 0
        self.devices[servo_id][address] = value
        if address == 5:
            self.devices[value] = self.devices.pop(servo_id)
            if self.lose_id_ack: return -6, 0
        return 0, 0


CONFIG = {"port": "fake", "baudrate": 1000000}
EXPECTED = {"servo_id": 1, "model_number": 777}


def test_live_position_reads_only_selected_servo_without_scanning_or_writes():
    fake = FakeBus((1, 6))
    fake.ping = lambda *args: pytest.fail("Position feedback must not scan")
    for ticks in (500, 3500):
        fake.devices[6][56] = ticks
        row = setup.read_position(CONFIG, 6, fake.sdk)
        assert row["servo_id"] == row["reported_id"] == 6
        assert row["raw_position"] == ticks and row["torque_enabled"] is False
        assert row["position_range"] == {"min": 0, "max": 4095}
    assert not fake.writes and fake.closed


def test_live_position_keeps_measured_ticks_with_hardware_warning():
    fake = FakeBus()
    fake.devices[1][65] = 32
    row = setup.read_position(CONFIG, 1, fake.sdk)
    assert row["raw_position"] == 2048 and row["hardware_error_flags"] == 32
    assert row["hardware_errors"] and not fake.writes and fake.closed


def test_live_position_failure_closes_connection():
    fake = FakeBus()
    with pytest.raises(RuntimeError, match="did not respond"):
        setup.read_position(CONFIG, 6, fake.sdk)
    assert not fake.writes and fake.closed


def test_scan_all_ids_including_zero_and_253_without_writes():
    fake = FakeBus((0, 1, 253))
    result = setup.scan(CONFIG, fake.sdk)
    assert [r["servo_id"] for r in result["actuators"]] == [0, 1, 253]
    assert result["actuators"][0]["settings"]["max_position_ticks"] == 4095
    assert not result["actuators"][0]["settings_errors"]
    assert fake.writes == [] and fake.closed


def test_scan_preserves_good_ids_before_and_after_unreadable_reply():
    fake = FakeBus((1, 2, 3, 253))
    fake.corrupt_id = 2
    rows = setup.scan(CONFIG, fake.sdk)["actuators"]
    assert [row["servo_id"] for row in rows] == [1, 2, 3, 253]
    assert rows[0]["raw_position"] == rows[2]["raw_position"] == 2048
    assert rows[1]["discovery_status"] == "unreadable"
    assert rows[1]["model_number"] is None and not rows[1]["assignment_supported"]
    assert "Possible shared ID" in rows[1]["errors"][0]
    assert not fake.writes and fake.closed


def test_unreadable_bus_is_still_strict_before_torque_release():
    fake = FakeBus((1,))
    fake.corrupt_id = 2
    with pytest.raises(RuntimeError, match="could not be decoded"):
        setup.release(CONFIG, EXPECTED, fake.sdk)
    assert not fake.writes and fake.closed


@pytest.mark.parametrize("margin", [0, 20])
def test_calibration_context_targets_only_selected_servo_with_exact_tick_conversion(margin):
    state = {"servo_id": 3, "model_number": 777, "points": {"min": 1000, "home": 2000, "max": 3000},
             "safety_margin_ticks": margin}
    ctx = setup.build_test_context({**CONFIG, "hardware_id": "physical"}, state, {"settings": {"operating_mode": 0}})
    joints = setup.bus.joints_from_profile(ctx["profile"])
    assert set(joints) == {"actuator"} and joints["actuator"].servo_id == 3
    assert setup.bus.degrees_to_ticks(joints["actuator"].min_deg, joints["actuator"]) == 1000 + margin
    assert setup.bus.degrees_to_ticks(joints["actuator"].max_deg, joints["actuator"]) == 3000 - margin
    assert ctx["calibration"]["hardware_id"] == "physical"
    assert ctx["profile"]["joints"][0]["velocity_limit"] == 180
    assert ctx["position_target_mode"] is True
    with pytest.raises(ValueError, match="Position mode"):
        setup.build_test_context({**CONFIG, "hardware_id": "physical"}, state, {"settings": {"operating_mode": 1}})


@pytest.mark.parametrize("lose_ack", [False, True])
def test_assign_verifies_new_id_relocks_and_never_moves(lose_ack):
    fake = FakeBus()
    fake.lose_id_ack = lose_ack
    result = setup.assign(CONFIG, EXPECTED, 6, fake.sdk)
    assert result["assigned"] and result["new_id"] == 6
    assert fake.writes == [(1, 55, 0), (1, 5, 6), (6, 55, 1)]
    assert fake.devices[6][40] == 0 and fake.devices[6][55] == 1 and fake.closed


@pytest.mark.parametrize("change", ["multiple", "torque", "warning", "model", "changed", "corrupt"])
def test_preflight_blocks_writes(change):
    fake = FakeBus((1, 2) if change == "multiple" else (1,))
    if change == "torque": fake.devices[1][40] = 1
    if change == "warning": fake.devices[1][65] = 0x20
    if change == "model": fake.devices[1][3] = 123
    if change == "changed": fake.devices[1][5] = 7
    if change == "corrupt": fake.corrupt_id = 99
    with pytest.raises((ValueError, RuntimeError)):
        setup.assign(CONFIG, EXPECTED, 6, fake.sdk)
    assert fake.writes == [] and fake.closed


def test_failed_id_write_relocks_original_and_does_not_claim_success():
    fake = FakeBus()
    fake.fail_id_write = True
    with pytest.raises(RuntimeError): setup.assign(CONFIG, EXPECTED, 6, fake.sdk)
    assert fake.devices[1][55] == 1 and fake.closed


def test_relock_failure_reports_uncertain_state():
    fake = FakeBus()
    fake.fail_lock = True
    with pytest.raises(RuntimeError, match="relock could not be verified"):
        setup.assign(CONFIG, EXPECTED, 6, fake.sdk)
    assert fake.closed


def test_repeated_already_assigned_id_does_not_write():
    fake = FakeBus((6,))
    result = setup.assign(CONFIG, {"servo_id": 6, "model_number": 777}, 6, fake.sdk)
    assert result["assigned"] and fake.writes == []


@pytest.mark.parametrize("destination", [0, 254, -1, True, 1.5, "6"])
def test_invalid_destination_rejected_before_open(destination):
    fake = FakeBus()
    with pytest.raises(ValueError): setup.assign(CONFIG, EXPECTED, destination, fake.sdk)
    assert fake.writes == []


def test_torque_release_reads_back_off_and_never_enables():
    fake = FakeBus()
    fake.devices[1][40] = 1
    fake.devices[1][65] = 0x20
    result = setup.release(CONFIG, EXPECTED, fake.sdk)
    assert result["released"] and result["actuators"][0]["torque_enabled"] is False
    assert fake.writes == [(1, 40, 0)] and fake.closed


def test_release_refuses_multiple_actuators():
    fake = FakeBus((1, 2))
    with pytest.raises(ValueError): setup.release(CONFIG, EXPECTED, fake.sdk)
    assert fake.writes == []


@pytest.mark.parametrize("provider_name", ["mock", "feetech"])
def test_shared_setup_provider_contract(provider_name):
    if provider_name == "mock":
        robot_setup = pytest.importorskip("blacknode.pkg.blacknode_robot.actuator_setup")
        robot_setup._mock_rows.clear()
        provider = robot_setup.robot_actuator_setup_mock_provider._bn_robot_actuator_setup_provider
    else:
        fake = FakeBus()
        provider = {
            "scan": lambda config: setup.scan(config, fake.sdk),
            "assign": lambda config, expected, new_id: setup.assign(config, expected, new_id, fake.sdk),
            "release": lambda config, expected: setup.release(config, expected, fake.sdk),
        }
    scanned = provider["scan"](CONFIG)
    row = scanned["actuators"][0]
    assert row["servo_id"] == 1 and row["torque_enabled"] is False
    assert not row["hardware_error_flags"] and not row["errors"]
    released = provider["release"](CONFIG, row)
    assert released["released"] and released["actuators"][0]["torque_enabled"] is False
    assigned = provider["assign"](CONFIG, row, 6)
    assert assigned["assigned"] and assigned["old_id"] == 1 and assigned["new_id"] == 6
    assert assigned["actuators"][0]["servo_id"] == 6
    assert assigned["actuators"][0]["torque_enabled"] is False
    assert provider["scan"](CONFIG)["actuators"][0]["servo_id"] == 6
