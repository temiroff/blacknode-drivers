# Adapters

Transport adapters for the `can` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.can.adapters.ros2]
    description = "ROS 2 adapter for can."
    default = false
    capabilities = ["adapter.can.ros2"]
    nodes = ["components/can/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
