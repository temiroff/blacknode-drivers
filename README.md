# blacknode-drivers

Physical hardware-driver components for Blacknode robotics workflows.

This layer repository groups independently selectable vendor and firmware
adapters. Installing the repository makes its component catalog available;
Blacknode loads nodes and dependencies only for enabled components.

## Components

| Component | Status | Capability |
|---|---|---|
| `feetech` | Available | Feetech STS/SMS bus configuration, read-only probing, and torque-safe bus primitives |
| `stm32` | Planned | STM32 firmware bridge |
| `serial` | Planned | Generic serial hardware helpers |
| `can` | Planned | CAN bus adapters |
| `vendor-adapters` | Planned | Additional physical hardware SDK adapters |

The Feetech component is enabled by default while this is the repository's
first component. Manage it explicitly with:

```powershell
blacknode packages components blacknode-drivers
blacknode packages enable blacknode-drivers feetech
blacknode packages setup blacknode-drivers
blacknode packages disable blacknode-drivers feetech
```

## Feetech nodes

- `FeetechBusConfig` validates a serial-bus and joint map without opening the
  port. It accepts either a Blacknode robot profile or a compact joint-map
  string.
- `FeetechBusProbe` performs an explicitly confirmed, read-only position probe.
  It never writes goal position or torque registers.

The component keeps `scservo_sdk` imports deferred until a probe or hardware
runtime is invoked, so package discovery works on development machines without
the SDK or attached servos.

## Ownership boundary

`blacknode-drivers` owns physical protocol access and final driver-boundary
safeguards. `blacknode-robot` owns robot models, profiles, calibration, and
capability contracts. `blacknode-ros2` owns ROS graph and transport behavior.
A later `feetech-ros2` adapter component will connect this bus implementation
to the ROS interface after cross-package component dependencies are available.

## Safety

- Configuration is inert and opens no hardware.
- Probing is read-only and requires `confirm_read_only=true`.
- Torque enable primitives read every joint, seed the current positions as
  goals while torque is off, and then enable torque.
- Any partial failure returns every configured joint to torque-off.
- Commands are clamped to configured safe ranges at the bus boundary.
- Physical torque and motion paths have not been exercised as part of routine
  automated tests.

## Tests

From the Blacknode repository root:

```powershell
python -m pytest packages/blacknode-drivers/tests
python -m blacknode.cli validate packages/blacknode-drivers/templates/feetech-bus-config.json
```

## License

Apache-2.0, same as Blacknode.
