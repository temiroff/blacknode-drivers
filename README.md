# blacknode-drivers

`blacknode-drivers` contains concrete physical bus and firmware providers. The current public component is `feetech`, covering STS/SMS serial-bus setup, read-only probing, calibration primitives, monitoring, and safe joint motion.

## Components and nodes

| Surface | Purpose |
|---|---|
| `feetech` | Default driver component using `scservo_sdk` |
| `feetech/ros2` | Optional ROS 2 and rosbridge process adapter |
| `FeetechBusConfig` | Validate a bus and joint map without opening hardware |
| `FeetechBusProbe` | Explicitly confirmed, read-only position probe |
| `FeetechCalibrationProvider` | Profile-selected calibration and motion provider |
| `FeetechRawMonitorProvider` | Bounded read-only servo discovery and diagnostics |
| `FeetechActuatorSetupProvider` | Full ID discovery and verified, isolated STS3215 ID assignment |
| `FeetechROS2Adapter` | Connect the driver to the standard ROS 2 interface |

Install the repository, enable the required component or adapter in **Packages**, then press **Install prerequisites**. The `feetech-bus-config.json` template provides a portable configuration example.

## Safety

- Configuration is inert; probing is read-only and requires confirmation.
- Torque enable seeds every joint from fresh feedback before activation.
- Commands are clamped again at the driver boundary.
- Communication loss, partial failure, parent-process exit, and graceful shutdown return configured joints to torque-off and verify the physical torque registers.
- Support the robot before releasing torque; an unsupported arm may fall.
- Physical limits are never discovered by driving into hard stops.

## Verification

The **Actuator Setup** workflow in `blacknode-robot` selects this provider through
USB hardware matching, with profile bindings supported for saved workflows.
Scanning probes IDs 0–253 at the operator-selected baud rate and returns a settings
snapshot for each responding actuator. Known STS3215 settings include baud code,
position limits, max torque, operating mode and EEPROM lock.
Discovery preserves valid replies and returns an unresolved address record for
each unreadable reply, then continues scanning. Such records have
`discovery_status: unreadable`, an unknown model and disabled assignment.
Assignment and torque release retain strict preflight scans; unresolved replies
never authorize writes.
`read_position(config, servo_id)` reads one known actuator's tick position,
torque state, identity and hardware warnings, then closes the bus. Calibration
controls use these bounded read-only samples to follow hand movement without
repeating full ID discovery or writing any registers.
Armed session feedback reads torque, position and health in one contiguous
packet. Setup sends the latest position target directly with a bounded 180°/s
speed, acceleration 50 and zero goal time, retaining fresh command checks.
The packet follows [Feetech's WritePosEx contract](https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py).
Goal, speed, acceleration and time are verified by readback; no EEPROM is changed.

For per-servo calibrated testing, `build_test_context(config, state, row)` translates
captured tick limits into a one-joint profile and hardware-bound calibration for
the existing motion controller. It verifies STS3215 position mode and a valid
single-revolution range. The motion provider seeds the measured pose before
torque and preserves its existing limit, feedback and shutdown checks.
Assignment requires one physically isolated STS3215 (model 777), complete
warning-free feedback and torque off. It rescans immediately before the write,
unlocks EEPROM, sets the ID, relocks it and verifies the new ID, model, torque
and disappearance of the old address. Lost write acknowledgements are resolved
by readback. Uncertain relock or identity results block success and require an
isolated rescan. No position or torque-enable register is written.
The servo card's **Release torque** control writes torque-off and
independently reads it back. Hardware warnings are tolerated only for verified
torque release; ID programming stays strict.

Serial connections use exclusive ownership on Windows and POSIX. Stop other
software and managed sessions using the adapter before setup. A scan cannot
prove that two actuators do not share the same address: the physical-isolation
confirmation is mandatory. Register references:
[Feetech control tables](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py)
and [STS3215 setup guidance](https://www.waveshare.com/wiki/ST3215_Servo).

```powershell
python -m pytest packages/blacknode-drivers/tests
python -m blacknode.cli validate packages/blacknode-drivers/components/feetech/templates/feetech-bus-config.json
```

Routine tests use mocks. See [AGENTS.md](AGENTS.md) for hardware-test reporting and driver boundaries.
