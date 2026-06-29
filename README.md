# concrete_block_rviz_plugins

RViz panels for the concrete-block stack. Scoped to host panels for the whole stack (perception, world model, BT control) — not just perception.

## Responsibilities

- Give the operator GUI buttons inside RViz that call stack services directly (no terminal), for driving perception test modes and wall setup.

## Panels

| Class | Source | Calls | Status |
|---|---|---|---|
| `concrete_block_rviz_plugins/PerceptionControlPanel` | `perception_control_panel.cpp` | `RunPoseEstimation` ([world_model_interfaces](../concrete_block_world_model_interfaces/)) | **registered & built** |
| *(wall assembly panel)* | `wall_assembly_panel.cpp` | `ConfirmWallOrigin` ([assembly_interfaces](../concrete_block_assembly_interfaces/)) | source present but **not in `CMakeLists.txt` SRCS nor `rviz_plugins.xml`** — dormant/WIP |

Plugin descriptor: [rviz_plugins.xml](rviz_plugins.xml).

## Dependencies & interactions

- **Depends on:** [concrete_block_world_model_interfaces](../concrete_block_world_model_interfaces/) (the only stack interface listed in `package.xml` / `CMakeLists.txt`), plus `rviz_common`, `pluginlib`, `Qt5`.
- **Talks to:** [concrete_block_world_model](../concrete_block_world_model/)'s `world_model_node` (service calls from the perception panel).
- The dormant wall panel would additionally need `concrete_block_assembly_interfaces` added to `package.xml` / `CMakeLists.txt` before it can build — it is **not** a current dependency.

## Use

Add the panel from RViz: *Panels → Add New Panel → concrete_block_rviz_plugins/PerceptionControlPanel*. The stack-default layout in [`config.rviz`](../../../config.rviz) already wires it up.

## Build

```bash
colcon build --packages-select concrete_block_rviz_plugins --symlink-install
source install/setup.bash
```

Adding a new panel: subclass `rviz_common::Panel`, register it in `rviz_plugins.xml` under the existing `<library>`, install the header from `CMakeLists.txt`.
