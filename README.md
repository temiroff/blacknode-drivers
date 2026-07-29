# blacknode-drivers

Physical drivers are selectable components named after the device or firmware
family they control. `feetech` owns its bus contract, safeguards, runtimes, and
optional adapters. Its nested `ros2` adapter depends on
`blacknode-ros2/core`; enabling that adapter resolves the ROS dependency while
the component remains simply `feetech`.

This layer repository groups independently selectable vendor and firmware
adapters. Installing the repository makes its component catalog available;
Blacknode loads nodes and dependencies only for enabled components.

## Components

| Component | Status | Capability |
|---|---|---|
| `feetech` | Available | Feetech STS/SMS bus configuration, read-only probing, and torque-safe bus primitives |

Feetech is currently the only public driver component. STM32, DaMiao,
CubeMars, and RealSense become selectable components when their usable driver
code, dependencies, safeguards, unavailable-state reporting, and tests exist.

Serial, CAN, and USB are internal transport mechanisms rather than selectable
components. Shared transport helpers belong under `internal/transports/` and
are consumed by an implemented concrete driver such as `feetech`.

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

`blacknode-drivers` owns concrete physical drivers, their internal protocol
access, and final driver-boundary safeguards. `blacknode-robot` owns robot
models, profiles, calibration, and capability contracts. `blacknode-ros2` owns
ROS graph and transport behavior. The optional `ros2` adapter nested under
`feetech` connects this driver to the ROS interface while hardware ownership
stays inside the Feetech component.

Driver feedback is normalized into
`blacknode-robot/contracts` `JointState`, `DeviceState`, and `FaultState`
before it reaches deployment telemetry, workflows, or safety consumers.
ROS `sensor_msgs/msg/JointState` exists only at the nested ROS adapter boundary.
Positions and calibrated limits are reported in radians. The Feetech runtime
also reports per-servo temperature and voltage, active hardware-error flags,
complete-feedback age, communication timeout count, serial packet-error count,
and packet-error rate. These diagnostics are passive reads and never authorize
motion.

## Driver safety boundary

Concrete drivers enforce the final hardware boundary:

- communication timeout and stale-command suppression;
- vendor fault, temperature, voltage, and serial packet-error reporting;
- final calibrated-range clamping;
- idempotent torque enable and disable;
- torque disable during normal or fault shutdown.

`blacknode-motion` independently enforces ownership, authorization, joint and
velocity limits, and collision checks. A physical or firmware emergency stop
remains the final independent layer.

When the Feetech ROS 2 driver runs inside Blacknode Runtime 0.3.9 or newer, it
also publishes the same read-only joint and torque state to the runtime's local
deployment telemetry bridge. This lets the editor monitor the robot after the
deployment takes ownership of the serial bus. Outside a managed deployment the
bridge is absent and the driver continues normally.
Runtime 0.3.16 or newer also receives each configured joint's calibrated limits,
allowing monitor charts to use a stable physical scale.

## Safety

- Configuration is inert and opens no hardware.
- Probing is read-only and requires `confirm_read_only=true`.
- Driver startup disables torque on every configured joint before reporting
  state. Support the robot because a previously holding arm may become limp.
- Torque enable primitives read every joint, seed the current positions as
  goals while torque is off, and then enable torque.
- Goal seeding retries the same current-position write and accepts a matching
  register readback when a half-duplex acknowledgement is lost.
- Torque changes retry after communication loss and require either a successful
  write acknowledgement or a matching torque-register readback.
- Any partial failure returns every configured joint to torque-off.
- Commands are clamped to configured safe ranges at the bus boundary.
- Physical torque and motion paths have not been exercised as part of routine
  automated tests.

## Tests

From the Blacknode repository root:

```powershell
python -m pytest packages/blacknode-drivers/tests
python -m blacknode.cli validate packages/blacknode-drivers/components/feetech/templates/feetech-bus-config.json
```

## License

Apache-2.0, same as Blacknode.
