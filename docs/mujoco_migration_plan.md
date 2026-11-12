# 迁移到 MuJoCo：实施方案

把当前基于 PyBullet 的仿真后端换成 MuJoCo，同时保持**算法层零改动、结果可比对**。

- 现状基线：提交 `63b5cfb`，`viewpoint_sim/` 1385 行，39 项测试，PyBullet 3.2.7
- 相关：代码结构见 `docs/simulation_architecture.md`，参数含义见 `docs/config_reference.md`
- 本文只解决"怎么换引擎"，不涉及规格功能补齐（那部分见 `workspace/task_breakdown.md`）

---

## 1. 迁移原则

四条约束决定了整个方案：

| # | 原则 | 理由 |
|---|---|---|
| 1 | **算法层不动** | FK / IK / RRT / 几何判据全在自有代码里，换引擎不应改变这些行为 |
| 2 | **双后端并存** | 只有让两个引擎跑同一份配置，才能用"逐位/逐像素差异"做验收，而不是靠眼看 |
| 3 | **配置格式不变** | `configs/default.json` 是唯一输入契约，MJCF 由配置生成而非另立一份 |
| 4 | **失败要能定位到引擎差异** | 每个 API 替换都记录"语义是否等价"，不等价的显式列出 |

因为原则 1 成立，所以有一件很有力的事：**`q` 序列、`T_BC`、`views.json` 的 ID 与数值应与换引擎前完全一致**（除非碰撞判定边缘翻转）。这让验收可以写成断言，而不是主观评价。

---

## 2. 现状：PyBullet 依赖面

`import pybullet` 出现在 3 个文件，实际调用集中在 `world.py`。

| 文件 | 用途 | 改动量 |
|---|---|---|
| `viewpoint_sim/world.py` | 场景、碰撞、渲染全部 | **重写**（359 行） |
| `viewpoint_sim/cli.py` | `replay` 末尾的 GUI 事件循环（`isConnected`） | 20 行 |
| `tests/test_world.py` | 遮挡/碰撞测试直接调用 `getVisualShapeData`、`getLinkState`、`removeBody` | 3 处 |

`world.py` 用到的 API（共 16 类）：

| # | PyBullet 调用 | 次数 | 用途 |
|---:|---|---:|---|
| 1 | `connect(GUI / DIRECT)` | 1 | 客户端 |
| 2 | `setGravity(0,0,0)` | 1 | 关掉重力 |
| 3 | `loadURDF(useFixedBase, URDF_USE_SELF_COLLISION)` | 1 | 机器人 |
| 4 | `getNumJoints` / `getJointInfo` | 1+ | link 名 ↔ 索引 |
| 5 | `createCollisionShape(Array)` / `createVisualShape(Array)` | 3 处 | box / sphere / 多 box 组合 |
| 6 | `createMultiBody` | 3 处 | 静态体 |
| 7 | `getVisualShapeData` | 2 处 | 隔离渲染的颜色快照 |
| 8 | `resetJointState` | 1 | 位姿写入 |
| 9 | `getLinkState(computeForwardKinematics)` | 2 处 | 法兰位姿 |
| 10 | `getClosestPoints` | 2 处 | 自碰撞 + 环境碰撞 |
| 11 | `computeViewMatrix` + `getCameraImage` | 1 | 渲染 |
| 12 | `changeVisualShape` | 2 处 | 隔离渲染 |
| 13 | `addUserDebugLine` / `addUserDebugText` | 3 处 | ROI/托盘/视锥可视化 |
| 14 | `configureDebugVisualizer` / `resetDebugVisualizerCamera` | 2 处 | GUI 设置 |
| 15 | `isConnected` / `disconnect` | 3 处 | 生命周期 |
| 16 | `ER_TINY_RENDERER` / `ER_SEGMENTATION_MASK_*` | 2 处 | 渲染器与分割 |

**没有被用到的**：动力学、`stepSimulation`、力矩控制、`getContactPoints` 的力信息。这是好消息——仿真是纯运动学的，迁移不涉及动力学语义。

---

## 3. 能力对照

### 3.1 一一对应（语义等价）

| PyBullet | MuJoCo | 备注 |
|---|---|---|
| `setGravity(0,0,0)` | `<option gravity="0 0 0"/>` | |
| `resetJointState(q)` | `data.qpos[jnt_qposadr] = q` + `mj_forward` | 必须先 forward 才能读 `xpos` |
| `getLinkState(...)[4:6]` | `data.xpos[body]` / `data.xquat[body]` | MuJoCo 是全局体坐标系，无需 `computeForwardKinematics` 参数 |
| `createCollisionShape(GEOM_BOX)` | `<geom type="box" size="半尺寸">` | size 是半尺寸，与 Bullet 一致 |
| `createCollisionShapeArray`（多 box 一个 body） | 一个 `<body>` 下多个 `<geom>` | **MuJoCo 更自然**，无需手工拼 array |
| `GEOM_SPHERE` | `<geom type="sphere" size="r">` | |
| `ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX` → 取 body id | `Renderer.enable_segmentation_rendering()` → geom id | 需自建 geom→body 映射 |
| `getClosestPoints(a,b,margin)` | `mj_geomDistance` 或 `<sensor><distance>` | 逐对查询，见 §3.3 |
| SRDF `disable_collisions` 允许矩阵 | `<contact><exclude body1 body2>` | 精确对应（body 级） |
| URDF 父子关节豁免 | `filterparent`（默认开启） | **自动获得**，无需再建 |
| `addUserDebugLine` | `viewer.user_scn` + `mjv_initGeom(mjGEOM_LINE)` | 需自己维护 ngeom |
| `addUserDebugText` | **无等价物** | 需改为 `user_scn` 之外的方案，见 §3.4 |
| `isConnected` 事件循环 | `viewer.is_running()` + `viewer.sync()` | |

### 3.2 需要转换（语义近似，必须验证）

| 项 | PyBullet | MuJoCo | 风险 |
|---|---|---|---|
| **机器人模型** | `loadURDF` 原生 | MuJoCo 解析器能读 URDF，但字段覆盖有限（`<transmission>`、`<gazebo>` 被忽略；固定关节默认被 fuse） | **中**：需要 `fusestatic=false` 才保住 link 结构 |
| **网格碰撞** | STL → 凸包 | STL 二进制可直接读，碰撞同样用**凸包** | 低（行为一致） |
| **相机坐标系** | 光学系 `+Z` 前、`+Y` 下、`+X` 右 | 相机系 `−Z` 前、`+Y` 上、`+X` 右 | **高**：`T_BC` 必须转换，见 §3.5 |
| **深度图** | 非线性深度缓冲，代码已有反算公式 | `Renderer` 深度也是归一化值，但 `znear/zfar` 由 `vis.map.znear × stat.extent` 推导 | **高**：`far` 不再是常数 2.0 m，公式要重标定 |
| **离屏渲染** | `ER_TINY_RENDERER`，纯 CPU，无 GL 依赖 | 经典渲染器**需要 OpenGL 上下文**（`MUJOCO_GL=egl/osmesa/glfw`） | **高**：无 GPU 服务器/WSL2 可能起不来 |

### 3.3 碰撞判定的等价性

PyBullet 侧：`getClosestPoints(robot, body, distance=0.002)`，返回所有距离小于 2 mm 的最近点对；代码用 `c[3]/c[4]` 拿 link 索引、`c[5]/c[6]` 拿两点的世界坐标、`c[8]` 拿带符号距离（用于法兰/支架的局部豁免）。

MuJoCo 侧有两条路：

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A：`mj_geomDistance`** | 对每个 geom 对调用 `mujoco.mj_geomDistance(m,d,g1,g2,distmax,fromto)` | 与 Bullet 语义最接近，能拿带符号距离与接触点 | 需要 geom 级笛卡尔积循环（机器人 9 geom × 环境 12 geom ≈ 100 次/位姿） |
| **B：`mj_forward` + `data.contact`** | 设 `geom margin=0.002`，读 `data.contact` 的 `dist` 与 `pos` | 一次 forward 出全部接触，走引擎 broadphase，快 | 只报告 `margin` 以内的对；`<exclude>` 之后拿不到被排除的项（正好符合需求）；顺序不稳定 |

**建议：主用 B，B 无法覆盖的局部豁免用 A 补。** 理由：`data.contact` 天然只包含"margin 以内"的对，与 `distance=0.002` 完全对应；而法兰/支架那 12 mm 圆盘豁免需要接触点坐标，`data.contact[i].pos` 已提供，够用。

必须先确认的一件事：**两个 geom 的 `margin` 如何合成（取较大值还是相加）必须实测确认**。若为相加，2 mm 会变成 4 mm，误拒显著增加。稳妥做法是把 margin 集中设在机器人 geom 上（`class="robot"`），环境 geom 保持 0，这样无论哪种合成规则结果都是 2 mm。这一点在验收里用"与 PyBullet 同 `q` 的判定逐条对比"来卡。

### 3.4 可视化辅助物的损失

`draw_scene()` 画 ROI 线框、托盘边框、采样高度平面与 `h=200 mm` 文字；`draw_view()` 画视锥。

MuJoCo 方案：

- **线框**：`viewer.user_scn` 里填 `mjGEOM_LINE`/`mjGEOM_CAPSULE`，每帧重建（`ngeom=0` 后重填）。
- **文字**：`mjvGeom` 不支持文字。替代方案：把高度值打印到终端，或用 `viewer` 自己的 UI。**建议直接放弃文字**，只保留线框——它不影响任何判定。

### 3.5 相机与内参的映射（最容易出错的地方）

我们的约定（`geometry.py` 头注释）：光学系 `+X` 右、`+Y` 下、`+Z` 前，OpenCV 式针孔 `u = fx·x/z + cx`。

MuJoCo 相机：`−Z` 前、`+X` 右、`+Y` 上，内参用 `focalpixel=[fx, fy]`、`principalpixel`（相对图像中心的偏移）、`resolution=[w,h]`。

转换关系：

```
R_mj = diag(1, -1, -1) @ R_opt        # 翻转 Y、Z
t_mj = t_opt
principalpixel = [ cx - (w-1)/2 , (h-1)/2 - cy ]   # 符号需用测试确认
```

两处必须用**单元测试**而不是推理来定：

1. **主点偏移符号**：`principalpixel` 的 y 正方向（相对 MuJoCo 的 `+Y` 上）与我们的 `cy`（向下）相反，容易差一个负号。
2. **像素中心约定**：我们的 `+0.5` 半像素修正（`projection_matrix()`）是为 OpenGL 光栅中心服务的；MuJoCo 是否已有同样的半个像素偏置，必须投影一个已知点比出来。

**验证方法**：取一个固定位姿，把同一组 `box_vertices` 用 `geometry.project()` 与 MuJoCo 渲染出的分割图质心对比，误差应 < 1.5 px（与现有 `test_off_center_intrinsics` 同一标准）。

若内建相机模型无法表达偏心主点，退路是**手工构造 `MjvGLCamera`**：直接填 `pos/forward/up/frustum_center/frustum_width/frustum_bottom/frustum_top/near/far`，完全按我们的内参算。这条路不依赖 MuJoCo 的相机语义，代价是要自己做视锥计算。**建议先做 0.5 天的 spike 确定走哪条。**

---

## 4. 目标架构：后端抽象

新增一层薄接口，让 `planner` / `acquisition` / `motion` / 测试都不知道引擎是谁。

```
viewpoint_sim/
  backends/
    __init__.py        # make_world(cfg, gui, backend="pybullet"|"mujoco")
    base.py            # World 契约（抽象基类 + 文档）
    bullet_world.py    # 从现有 world.py 平移
    mujoco_world.py    # 新实现
    mjcf.py            # config → MJCF 文本（含相机夹具注入）
  world.py             # 保留为兼容别名：from .backends import make_world
```

契约（与现有 `World` 公开方法一致，不改一个签名）：

| 方法 | 契约 |
|---|---|
| `set_q(q)` | 写入 6 关节角，保证 `flange_pose()` 立刻反映 |
| `flange_pose()` | 4×4，Base 系 |
| `camera_pose()` | `flange_pose() @ mount` |
| `collision(q)` | 返回 `None` 或原因串（`joint_limit` / `self_collision:a:b` / `environment_collision:name`） |
| `render(T_BC, camera=None)` | `(rgb uint8 HxWx3, depth_m float HxW, body_id int HxW)` |
| `isolated_food_masks(T_BC)` | 每食材一张布尔图 |
| `overview()` | 第三人称预览图 |
| `box` / `boxes` | 测试用的动态障碍物构造 |
| `visuals` | 隔离渲染的颜色快照（测试会 `extend`） |
| `cid` | 引擎原生句柄（测试目前直接用它，见下） |

**测试的适配**：`tests/test_world.py` 有 3 处直接调用 PyBullet（`getVisualShapeData`、`getLinkState`、`removeBody`）。改造方式：在契约里补两个小方法 `visual_shapes(body)` 与 `link_position(link)`、`remove_body(body)`，测试改用契约方法，从而同时覆盖两个后端。

---

## 5. 分阶段计划

| 阶段 | 内容 | 交付物 | 预估 | 前置 |
|---|---|---|---|---|
| **0. Spike** | 装 MuJoCo；确认 `MUJOCO_GL=egl` 下 `Renderer` 能出图；读 URDF 能否保住 link 结构；`mj_geomDistance` 是否存在（版本相关） | `docs/mujoco_spike.md`（结论 + 3 个最小脚本） | 0.5–1 人日 | — |
| **1. 抽象层** | 新建 `backends/`，把现有 `world.py` 平移为 `bullet_world.py`，补契约方法，39 项测试全绿 | 重构提交，`outputs/run_630` 逐位复现 | 1 人日 | — |
| **2. 模型与场景** | `mjcf.py` 由配置生成 MJCF；机器人 + 相机夹具 + 烤箱 + 层架 + 托盘 + 食材；`fusestatic=false`；`<exclude>` 从 SRDF 生成 | `mujoco_world.py` 能构造完整场景，`check` 通过 | 2 人日 | 0,1 |
| **3. 运动学与碰撞** | `set_q` / `flange_pose` / `collision`；`margin` 集中在机器人一侧；法兰/支架局部豁免 | 与 PyBullet 在同一批 `q` 上判定一致（差异清单化） | 1.5 人日 | 2 |
| **4. 渲染** | `GLContext` + `Renderer`；RGB / 深度 / 分割；内参映射与深度反算标定 | 图像指标：mask IoU > 0.99、深度差 < 1 mm | 2 人日 | 2 |
| **5. 隔离渲染与 GUI** | `isolated_food_masks`（`geom_rgba` alpha 置 0 后恢复）；`overview()`；`user_scn` 线框；`replay` 事件循环 | 可见率指标与 PyBullet 差 < 1% | 1 人日 | 3,4 |
| **6. 验收与切换** | 双后端回归脚本；CLI `--backend`；文档；默认后端切换（或保持可配） | `docs/mujoco_validation.md` | 1 人日 | 全部 |

合计约 **9–10 人日**，其中阶段 0 的结论会决定阶段 4 的走法（内建相机 vs 手工 `MjvGLCamera`），偏差可达 ±2 天。

### 阶段验收标准

**阶段 3（碰撞）**——同一份 `configs/default.json`，从 `outputs/run_630/views.json` 取全部 8 条路径的所有插值点（约 3000 个 `q`），两个后端分别判定：

- 判定完全一致：> 99%
- 不一致的必须逐条归类到 3 种原因之一（凸包近似差异 / `margin` 合成差异 / 浮点边界），并记录在案

**阶段 4–5（渲染）**——对 8 个已接受视点：

| 指标 | 阈值 |
|---|---|
| 食材 mask IoU | > 0.99 |
| 深度图中位数误差 | < 1 mm |
| 可见率（`visible_fraction`） | 相对差 < 1% |
| 烤盘占比 `tray_ratio` | 相对差 < 0.5% |
| 深度有效率 `valid_depth_fraction` | 差 < 0.5% |

**阶段 6（端到端）**——`seed` 不变重跑：

- 视点 ID 列表、`q_actual_rad`、`T_BC_actual` 与 `outputs/run_630` **完全一致**（IK 是自有代码，本就应该一致）
- 若有不一致，必须能归因到碰撞判定翻转，不能是"渲染变了所以质量检查拒了"

---

## 6. 风险与对策

| # | 风险 | 影响 | 对策 |
|---:|---|---|---|
| 1 | **无 OpenGL 环境**：MuJoCo 经典渲染必须建 GL 上下文，没有 PyBullet TinyRenderer 那样的纯 CPU 路径 | 阶段 4 完全阻塞 | 阶段 0 先验证 `MUJOCO_GL=egl` 与 `osmesa`；若都不可用，评估 `mujoco.mjrf`（Filament）或改用外部渲染器 |
| 2 | **像素级不可复现**：GL 渲染受驱动/GPU 影响，换机器可能差几个灰阶 | 破坏"逐位复现"的既有承诺 | 明确区分：**几何/判定层逐位复现**，**像素层允许容差**；`docs/validation.md` 需相应措辞 |
| 3 | **URDF 直接加载丢结构**：`fusestatic` 默认把固定关节 fuse 进父体 | 碰撞对、link 名、相机夹具挂载全变 | 生成 MJCF 而非直接读 URDF；至少显式 `fusestatic="false"` |
| 4 | **`margin` 合成规则不明**：两个 geom 的 margin 取大还是相加未确认 | 若相加，2 mm 间隙变成 4 mm，误拒增加 | margin 只设在机器人 class（环境为 0），两种规则下结果都是 2 mm |
| 5 | **深度归一化基数不同**：`znear/zfar` 是 `stat.extent` 的倍数 | 深度图整体偏移，可见率与量程判据全错 | 不用公式推导，用"已知距离平面 → 读数"标定反算系数 |
| 6 | **相机坐标系手性**：`−Z` 前 + `+Y` 上 | 图像上下颠倒、左右镜像 | 投影测试 + 分割图质心对比；不要靠推理 |
| 7 | **`mj_geomDistance` 版本依赖** | 方案 A 不可用 | 先查版本；不可用则全走 `data.contact` |
| 8 | **名字冲突**：MJCF 里 link/body/geom 名不能重复 | 编译报错 | 生成时加前缀（`r_` / `env_` / `food_`） |

---

## 7. 依赖与环境变更

```diff
- dependencies = ["numpy>=1.24,<3", "scipy>=1.10,<2", "Pillow>=10,<13", "pybullet==3.2.7"]
+ dependencies = ["numpy>=1.24,<3", "scipy>=1.10,<2", "Pillow>=10,<13",
+                 "pybullet==3.2.7", "mujoco>=3.0,<4"]
```

- Python 3.11 满足要求，`requires-python` 无需改。
- 无 GPU 时设置 `MUJOCO_GL=egl`（或 `osmesa`），写进 `scripts/setup.sh` 与 README 故障排查段。
- `requirements-lock.txt` 需重新生成。

---

## 8. 明确不做的事

| 不做 | 理由 |
|---|---|
| 引入动力学仿真 | 现有流程全程关重力、纯运动学；加动力学会改变所有路径与判定 |
| 用 MuJoCo 自带 IK | 自有 IK 是确定性的、有奇异值判据，换掉会破坏复现 |
| 用 MuJoCo 渲染替代图像质量验收 | 验收逻辑在 `acquisition.py`，与引擎无关 |
| 直接删除 PyBullet 后端 | 至少在阶段 6 之前保留，作为回归基准 |
| 迁移过程中补规格功能 | 两件事同时做会让"差异归因"失效 |

---

## 9. 建议的第一件事

先做**阶段 0 的三个 spike**，因为它们决定后面 8 人日怎么走：

1. `MUJOCO_GL=egl` 下 `Renderer` 能否在本机出 RGB + 深度 + 分割（5 行代码，10 分钟）
2. `MjModel.from_xml_path("assets/mycobot_630_pro/mycobot.urdf")` 后 `body` 数量、名字是否与 URDF 一致（验证 `fusestatic` 行为）
3. `mujoco.__version__` 与 `hasattr(mujoco, "mj_geomDistance")`

三个结果出来之前，不建议动 `backends/`。
