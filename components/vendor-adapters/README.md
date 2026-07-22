# Vendor Adapters

Component of `blacknode-drivers`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="vendor-adapters", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.vendor-adapters]
    nodes = ["components/vendor-adapters/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
