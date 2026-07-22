# Adapters

Transport adapters for the `feetech-ros2` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.feetech-ros2.adapters.ros2]
    description = "ROS 2 adapter for feetech-ros2."
    default = false
    capabilities = ["adapter.feetech-ros2.ros2"]
    nodes = ["components/feetech-ros2/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
