# ROS 2 迁移实施计划（目标 A：ROS 2 化仿真）

- 状态：**计划中**，尚未开始实施
- 目标：把当前采集流程跑进 ROS 2 节点，**保留 PyBullet 作为渲染与碰撞后端**
- 不在本次范围：MoveIt2 / Gazebo / 真机接入（见第 9 节的后续分支）
- 对照基线：提交 `0f290b8`（myCobot 630 Pro + 开口烤箱工位，39 项测试通过）

---

## 1. 为什么选目标 A

三个候选目标的工作量与影响差别很大：

| 目标 | 含义 | 替换 PyBullet | 现有基线是否失效 |
|---|---|---|---|
| **A. ROS 2 化仿真**（本文档） | 采集流程跑在 ROS 2 节点里，PyBullet 仍是渲染/碰撞后端 | 否 | **否** |
| B. MoveIt2 + Gazebo | 用 MoveIt2 做 IK/碰撞/规划，Gazebo 或 Isaac 做传感器仿真 | 是 | **是** |
| C. 接真机 | 仿真换成真机驱动 + 真实 D405 | 部分 | 是（数据层面） |

选 A 的理由：

1. **不破坏已验证的基线**。`outputs/demo`（8 视点）、`docs/validation.md`、五类负向工况的结论全部继续有效。
2. **风险最低**。碰撞判定仍是 PyBullet 的凸包 + SRDF 允许矩阵，不会因为换成 FCL 三角网格而改变拒绝/接受结果。
3. **为后续分支铺路**。A 做完之后，B 只是替换后端实现，C 只是替换驱动与相机源，接口不用再改。

---

## 2. 现状评估

### 2.1 PyBullet 的耦合面很小

实测依赖分布（`grep` + 逐文件核对）：

| 模块 | 行数 | 依赖 | 迁移影响 |
|---|---:|---|---|
| `geometry.py` | 133 | numpy / scipy | **零改动** |
| `kinematics.py` | 90 | numpy / scipy | **零改动** |
| `config.py` | 137 | numpy | **零改动** |
| `motion.py` | 78 | numpy（回调 `world.collision`） | **零改动** |
| `report.py` | 46 | Pillow | **零改动** |
| `comparison.py` | 31 | numpy | **零改动** |
| `acquisition.py` | 155 | numpy / Pillow（回调 `world.render`） | **零改动** |
| `planner.py` | 208 | numpy / scipy | 删一行未使用的 `import pybullet as p` |
| **`world.py`** | **359** | **pybullet** | **唯一实际替换点** |
| `cli.py` | 145 | pybullet（仅 GUI 事件循环） | 被 launch / node 取代 |

结论：**约 90% 的 Python 代码可直接复用**，迁移成本集中在 I/O 边界。

### 2.2 现有接口已可复用

| 现有函数 | ROS 2 中的角色 |
|---|---|
| `geometry_check(cfg, T_BC, index)` | 规划节点的纯几何门禁，原样调用 |
| `kinematics.Kinematics.solve()` | IK 服务，或 MoveIt2 `GetPositionIK` 的 fallback |
| `motion.plan()` / `edge_free()` / `path_free()` | 规划与复核，原样调用 |
| `World.collision(q)` | 碰撞判定，原样调用 |
| `World.render(T_BC)` | 相机节点出图，原样调用 |
| `inspect_capture()` / `save_capture()` | 采集节点的质量验收与落盘 |
| `planner.run()` | 规划 + 采集主循环，包成节点内部一次调用 |

### 2.3 需要补齐的缺口

| 缺口 | 现状 | 迁移时必须处理 |
|---|---|---|
| **无速度/稳定判据** | 只有 FK 到位残差检查（`ik_position_tolerance_m`） | A 阶段不阻塞（仿真无动力学）；C 阶段必须补 `max\|dq/dt\| < ε_v` 持续 τ |
| **无时间戳** | 图像、位姿、TF 之间没有时间对齐概念 | 必须引入 `header.stamp`，否则下游无法做同步 |
| **无并发** | 单进程顺序执行 | 规划节点的服务需防止重入（见 §6） |
| **结果目录布局** | `outputs/<run>/hXX_cXXXX/` + `rgb/` + 各 json | **保持完全不变**，避免下游脚本失效 |

---

## 3. 目标架构

```
ros2_ws/src/
├── viewpoint_sim_core/            ament_python｜无 ROS 依赖，纯计算
│   └── viewpoint_sim_core/
│       ├── geometry.py
│       ├── kinematics.py
│       ├── config.py
│       ├── motion.py
│       ├── comparison.py
│       └── report.py
├── viewpoint_sim_interfaces/      ament_cmake｜msg / srv / action
├── viewpoint_sim_sim/             PyBullet 后端
│   ├── viewpoint_sim_sim/world.py          ← 原 world.py 原样搬入
│   └── viewpoint_sim_sim/nodes/
│       ├── sim_world_node.py               驱动关节 + 发布 TF
│       └── view_camera_node.py             渲染 + 发布 RGB-D
├── viewpoint_sim_planner/
│   ├── viewpoint_sim_planner/planner.py    ← 原 planner.py（去掉 CLI 依赖）
│   └── viewpoint_sim_planner/nodes/
│       ├── viewpoint_planner_node.py       PlanViewpoints 服务
│       └── capture_node.py                 CaptureView action
├── viewpoint_sim_bringup/
│   ├── launch/{sim,plan_capture,full}.launch.py
│   ├── config/viewpoint_sim.yaml
│   └── rviz/viewpoint_sim.rviz
└── viewpoint_sim_robot/           预留：真机驱动（本次不实现）
```

### 3.1 包职责与依赖方向

```
interfaces  ←  core  ←  sim
                    ←  planner  ←  bringup
```

- `core` **不 import rclpy**，保持可在无 ROS 环境单测
- `sim` 依赖 `core` + `pybullet` + `rclpy`
- `planner` 依赖 `core` + `interfaces` + `rclpy`（`pybullet` 通过 `sim` 的节点获得，不直接 import）
- `bringup` 只放 launch / params，无代码

---

## 4. 接口设计

### 4.1 服务：`PlanViewpoints.srv`

一次请求返回全部位姿。**同步**语义，因为采样本身是确定性的、耗时可预期。

```
# request
float64[]  heights_above_tray_m     # 覆盖配置；空表示用 config 里的值
int32      target_per_height        # <=0 表示用 config
float64    z_perturbation_m         # <0 表示用 config
bool       dry_run                  # true = 只规划不采集
---
# response
bool                   complete
geometry_msgs/Pose[]   camera_poses     # T_BC，base_link 系
float64[]              heights_above_tray_m
string                 summary_json     # 与现有 summary.json 同结构
string                 output_dir
```

### 4.2 动作：`CaptureView.action`

长任务，需要进度反馈：单个视点要依次经过 IK、碰撞、路径规划、到位复核、渲染、质量验收与落盘。

```
# goal
string                    view_id
geometry_msgs/PoseStamped target_camera_pose
string                    output_dir
---
# feedback
string   stage          # IK | COLLISION | PLAN | EXECUTE | ARRIVE | CAPTURE | QUALITY | SAVE
float32  progress       # 0..1
---
# result
bool     success
string   folder
string   metadata_json
string   failure_reason
```

### 4.3 话题

| 话题 | 类型 | 发布者 | 消费者 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `sim_world_node`（或真机驱动） | 所有节点 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` + `sim_world_node` | 所有节点 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `view_camera_node` | `capture_node` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `view_camera_node` | `capture_node` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `view_camera_node` | `capture_node` |
| `/viewpoint/current_target` | `geometry_msgs/PoseStamped` | `capture_node` | 调试 / rviz |
| `/viewpoint/validity` | `moveit_msgs/GetStateValidity`（可选） | `planner` | 调试 |

### 4.4 TF 映射（替换现有手算变换）

| 现有代码 | ROS 2 等价 |
|---|---|
| `config.mount_transform(cfg)` | 静态 TF `flange` → `camera_optical_frame` |
| `world.flange_pose()` | `tf2.lookupTransform('base_link','flange', t)` |
| `world.camera_pose()` | `tf2.lookupTransform('base_link','camera_optical_frame', t)` |
| `T_BF = T_BC @ inv(T_FC)` | 由 TF 反查，或 `core` 计算后发布 |
| `cfg["robot"]["flange_link"]` = `flange` | URDF 里同名 link，`robot_state_publisher` 直接发布 |

坐标系约定：项目已是光学系 X 右 / Y 下 / Z 前，与 REP-103 一致。`camera_link` → `camera_optical_frame` 用固定变换 `[-π/2, 0, -π/2]`（RPY）。

### 4.5 参数

`configs/default.json` **不拆分、不改造**，由 `bringup/config/viewpoint_sim.yaml` 里的单个字符串参数指向其路径：

```yaml
viewpoint_planner_node:
  ros__parameters:
    config_path: "package://viewpoint_sim_bringup/config/default.json"
    output_dir: "outputs/ros_run"
```

理由：现有 `validate_config()` 已覆盖全部约束，把 JSON 拆成 ROS params 会复制一套校验逻辑，容易分叉。

---

## 5. 分阶段实施

### 阶段 0：核心库剥离（1–2 人日）

| 动作 | 验收 |
|---|---|
| 新建 `viewpoint_sim_core`，搬入 geometry/kinematics/config/motion/comparison/report | `python -c "import viewpoint_sim_core"` 在**无 rclpy** 环境可用 |
| `world.py` 搬入 `viewpoint_sim_sim`，内部零改动，仅调整 import | `World(cfg)` 行为不变 |
| 删除 `planner.py` 的 `import pybullet as p`（从未使用） | — |
| 保留现有 `viewpoint-sim` CLI 与 `scripts/*.py` 可用 | `unittest discover` 39 项全绿；`check` / `run` / `replay` 输出与 `0f290b8` 一致 |

**关键约束**：此阶段**不引入任何 ROS 依赖**，只做目录与 import 调整。目的是先拿到一个"行为零变更"的重构提交。

### 阶段 1：ROS 2 骨架 + 仿真节点（3–4 人日）

| 动作 | 交付物 |
|---|---|
| 建 `viewpoint_sim_interfaces` | 3 个接口文件；`ros2 interface show` 通过 |
| `sim_world_node` | 订阅 `/joint_states` → `world.set_q()` → 发布 `/tf`、`/joint_states`（回环用于记录） |
| `view_camera_node` | 用 `world.render()` 出图 → 转 `Image` / `CameraInfo`；timestamp 取节点时钟 |
| 静态 TF | URDF 走 `robot_state_publisher`；`flange` → `camera_optical_frame` 走 `static_transform_publisher` |
| launch + rviz | `ros2 launch viewpoint_sim_bringup sim.launch.py` 能在 rviz 看到模型、TF 树与深度点云 |

**验收门槛**：

1. `ros2 topic hz /camera/depth/image_rect_raw` 稳定在配置帧率
2. `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` 的数值与 `world.camera_pose()` 一致（误差 < 1e-6）
3. 发布同一组 `JointState` 两次，得到逐像素相同的 RGB（确定性未丢失）

### 阶段 2：规划节点 + 采集节点（3–4 人日）

| 动作 | 交付物 |
|---|---|
| `viewpoint_planner_node` | `PlanViewpoints` 服务，内部调 `planner.run()`（或拆分后的 `plan_views()`） |
| `capture_node` | `CaptureView` action server，内部调 `inspect_capture()` + `save_capture()` |
| 结果目录布局**保持不变** | `outputs/<run>/hXX_cXXXX/`、`rgb/`、`summary.json`、`views.json`、`report.html` |
| `verify_results.py` 保持可用 | 纯离线读文件，不依赖 ROS |
| 可选：保留 CLI 入口 | `ros2 run viewpoint_sim_planner cli -- run --headless` 便于无 ROS 环境调试 |

**验收门槛**：

1. 一次 `ros2 service call /plan_viewpoints` + 8 次 `ros2 action send_goal /capture_view` 产出的目录，与当前 `outputs/demo` **同结构**且 `verify_results.py` PASS
2. 五类负向工况（`scripts/run_negative_cases.py`）仍然全部 0 有效视点
3. 同一 `seed` 下两次运行产出相同的 `view_ids`（可复现性未丢失）

### 阶段 3（收尾）：文档与打包（1 人日）

| 动作 | 交付物 |
|---|---|
| `ros2_ws` 的 `colcon build` 说明 | README 新增「ROS 2 运行」章节 |
| 逐包 README | 每个包说明职责、依赖、启动方式 |
| 保留非 ROS 运行路径 | 现有 CLI 继续可用，两条路径共享 `core` |

---

## 6. 关键风险与对策

| 风险 | 严重度 | 对策 |
|---|---|---|
| **可复现性丢失**（最需要防范） | **高** | 采样 + IK + 规划 + 质量判定必须留在**同一次函数调用**里。ROS 2 只做 I/O 边界，不得把决策拆成跨节点异步步骤。验收用「两次运行 `view_ids` 相同」卡住 |
| PyBullet 客户端跨线程安全性未知 | 高 | 每个节点**独占一个** `PhysicsClientId`；不在回调里并发调用 `collision()`；规划节点内部串行化（见下） |
| 服务重入导致状态串扰 | 高 | `PlanViewpoints` 用 `ReentrantCallbackGroup` **关闭**，加互斥锁；规划期间拒绝第二个请求 |
| Python 版本约束 | 中 | `requires-python = ">=3.10,<3.12"` 只匹配 **Humble（22.04）与 Iron（23.04）**。用 **Jazzy（24.04 / Python 3.12）** 需放宽上界并重跑测试 |
| 图像与位姿时间戳不一致 | 中 | 所有消息用同一节点时钟；`capture_node` 用 `message_filters::ApproximateTime`；记录实际偏差到 `metadata.json` |
| DDS 大图像带宽 | 中 | 深度图 640×360×2 B ≈ 460 KB/帧；本地用 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 或 `use_intra_process_comms=True` |
| `outputs/` 相对路径在多节点下歧义 | 中 | `output_dir` 用绝对路径或在 launch 里 `$(find-pkg-share ...)` 统一推导 |
| launch 参数与 JSON 双份配置漂移 | 低 | JSON 仍是唯一真源；YAML 只存路径 |
| 临时目录 `tempfile.TemporaryDirectory(prefix="viewpoint_sim_")` 生命周期 | 低 | `World` 是上下文管理器，节点退出时随对象销毁；节点需在 `destroy_node()` 显式关闭 |

---

## 7. 工作量估算

| 阶段 | 人日 |
|---|---|
| 0 核心库剥离 | 1–2 |
| 1 骨架 + 仿真节点 | 3–4 |
| 2 规划 + 采集节点 | 3–4 |
| 3 文档与打包 | 1 |
| **合计（目标 A）** | **8–11** |

不含 B（MoveIt2，另需 5–8）与 C（真机，另需 2–3）。

---

## 8. 阶段 0 的最小验证清单

这是唯一可以立刻开始、且风险最低的步骤。完成后应能回答"是"：

- [ ] `python -m unittest discover -s tests` 39 项通过
- [ ] `python -m viewpoint_sim check` 输出 `"configuration_valid": true`
- [ ] `python -m viewpoint_sim run --headless --output /tmp/a` 得到 8 个视点
- [ ] `/tmp/a/views.json` 的 `id` 列表与 `outputs/demo/views.json` **完全相同**
- [ ] `python scripts/verify_results.py /tmp/a` 输出 PASS
- [ ] `python scripts/run_negative_cases.py /tmp/neg` 五类工况全部 0 有效
- [ ] `git diff` 中**没有任何数值、阈值或算法改动**（只有 import 路径）

---

## 9. 后续分支（不在本次范围）

| 分支 | 触发条件 | 主要工作 |
|---|---|---|
| **B. MoveIt2 后端** | 需要标准规划栈 / 与真机共用配置 | 复用现有 SRDF（`arm_group` 已定义 `base_link → J6_Link`，9 对 `disable_collisions`）；IK 换 `GetPositionIK`，碰撞换 FCL，规划换 OMPL。**注意 FCL 是三角网格、PyBullet 是凸包**，结果会变，需重跑全部基线与负向工况 |
| **C. 真机接入** | 硬件就绪 | `pymycobot` 驱动 + `FollowJointTrajectory`；`realsense2_camera`；**必须补速度稳定判据**（当前只有 FK 残差）；用真实 TF 而非目标位姿做几何复核 |

---

## 10. 需要确认的决策

开工前需明确：

1. **ROS 2 版本**：Ubuntu 22.04 + Humble（贴合现有 Python 约束），还是 24.04 + Jazzy（需改 `requires-python`）？
2. **是否保留 CLI**：目标 A 完成后，现有 `viewpoint-sim run/replay/check` 是否继续维护？（建议保留，便于无 ROS 环境调试与 CI）
3. **`outputs/` 归属**：ROS 节点写工程内 `outputs/`，还是改为可配置的绝对路径？（建议后者，用 launch 参数注入）
4. **是否现在就引入 `core` 拆包**：阶段 0 会把 `viewpoint_sim` 拆成 4 个 Python 包，工程结构变化较大。若希望保持单包，可把 `core` 作为子模块 `viewpoint_sim.core`，迁移成本更低但未来接 MoveIt2 时耦合更紧。
