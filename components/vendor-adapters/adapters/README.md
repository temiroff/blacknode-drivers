# Adapters

Transport adapters for the `vendor-adapters` component of `blacknode-drivers`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.vendor-adapters.adapters.ros2]
    description = "ROS 2 adapter for vendor-adapters."
    default = false
    capabilities = ["adapter.vendor-adapters.ros2"]
    nodes = ["components/vendor-adapters/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
