# concrete_block_rviz_plugins

RViz panels for the concrete-block stack. Scoped to host panels for the whole stack (perception, world model, BT control) — not just perception.

## Panels

| Class | Header | Purpose |
|---|---|---|
| `concrete_block_rviz_plugins/PerceptionControlPanel` | `perception_control_panel.hpp` | Buttons to trigger perception test modes via `concrete_block_world_model_interfaces` services |

Plugin descriptor: [rviz_plugins.xml](rviz_plugins.xml).

## Use

Add the panel from RViz: *Panels → Add New Panel → concrete_block_rviz_plugins/PerceptionControlPanel*. The stack-default layout in [`config.rviz`](../../../config.rviz) already wires it up.

## Build

```bash
colcon build --packages-select concrete_block_rviz_plugins --symlink-install
source install/setup.bash
```

Adding a new panel: subclass `rviz_common::Panel`, register it in `rviz_plugins.xml` under the existing `<library>`, install the header from `CMakeLists.txt`.
