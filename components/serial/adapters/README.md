# Adapters

Transport adapters for the `serial` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.serial.adapters.ros2]
    description = "ROS 2 adapter for serial."
    default = false
    capabilities = ["adapter.serial.ros2"]
    nodes = ["components/serial/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
