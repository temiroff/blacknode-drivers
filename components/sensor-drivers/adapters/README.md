# Adapters

Transport adapters for the `sensor-drivers` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.sensor-drivers.adapters.ros2]
    description = "ROS 2 adapter for sensor-drivers."
    default = false
    capabilities = ["adapter.sensor-drivers.ros2"]
    nodes = ["components/sensor-drivers/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
