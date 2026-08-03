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
| `FeetechROS2Adapter` | Connect the driver to the standard ROS 2 interface |

Install the repository, enable the required component or adapter in **Packages**, then press **Install prerequisites**. The `feetech-bus-config.json` template provides a portable configuration example.

## Safety

- Configuration is inert; probing is read-only and requires confirmation.
- Torque enable seeds every joint from fresh feedback before activation.
- Commands are clamped again at the driver boundary.
- Communication loss, partial failure, and shutdown return configured joints to torque-off.
- Support the robot before releasing torque; an unsupported arm may fall.
- Physical limits are never discovered by driving into hard stops.

## Verification

```powershell
python -m pytest packages/blacknode-drivers/tests
python -m blacknode.cli validate packages/blacknode-drivers/components/feetech/templates/feetech-bus-config.json
```

Routine tests use mocks. See [AGENTS.md](AGENTS.md) for hardware-test reporting and driver boundaries.
