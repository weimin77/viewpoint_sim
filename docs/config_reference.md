# `configs/default.json` 参数说明

本文件逐项说明 myCobot 630 Pro 的全部配置参数。对应的机器人模型与资产见 `assets/mycobot_630_pro/`，流程性说明见 `README.md` 第 3 节，验证结果见 `docs/validation.md`。

所有长度为**米**，角度为**弧度**；键名带 `_deg` 的值用**度**。所有坐标都在固定的 Base 参考系 B 中。改完配置后先跑校验：

```bash
.venv/bin/python -m viewpoint_sim check --config configs/default.json
```

`validate_config()` 会拒绝非法组合（下文每个参数注明约束），**不会静默忽略**。但未知键不会被检查——键名拼错会导致该参数失效而没有任何提示。

---

## 0. 顶层

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `units` | `"m_rad"` | 单位声明。**必须**是这个字面量，否则报 `All lengths must use meters, angles radians` |
| `description` | 示例工位说明 | 仅作文档，程序不读取 |
| `seed` | `20261106` | 随机种子。同一配置 + 同一种子必须复现同一批视点；内部拆成采样 / IK / 路径三路独立随机流，改动算法不会漂移采样序列 |

---

## 1. `robot` — 机器人模型与 IK

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `urdf` | `assets/mycobot_630_pro/mycobot.urdf` | URDF 相对路径。关节名、轴、限位全部从这里读取，代码不硬编码型号 |
| `srdf` | `assets/mycobot_630_pro/mycobot.srdf` | SRDF 相对路径，只用到其中的 `disable_collisions` 允许矩阵 |
| `flange_link` | `"flange"` | 法兰 link 名。630 上游 URDF 没有独立法兰，本项目在 `J6_Link` 工具板上补充了 `flange`（见 `provenance.json`）。IK 目标、`T_BF`、相机夹具挂载都以它为准 |
| `base_position_B_m` | `[0, 0, 0]` | 底座在 B 系的位置（米）。630 底座本身高 0.16 m，直接坐在台面上，所以是 0。仅用于摆放模拟机器人，不改工位 |
| `home_q_rad` | `[0, -0.4, 0.8, -1.2, 0, 0]` | 准备姿态，6 个关节角（弧度）。**必须无碰撞**（`check` 会检查），且是每条采集路径的起点 |
| `ik_starts` | `8` | IK 多初值数：1 个当前位姿种子 + 1 个关节中位 + `ik_starts-2` 个随机初值。调大更易找到解，但每次调用更慢 |
| `ik_position_tolerance_m` | `0.0005` | IK 解的位置误差上限（米）。到位后会用实际 FK 再复核，超限直接抛错 |
| `ik_rotation_tolerance_deg` | `0.1` | IK 解的姿态误差上限（度） |
| `ik_characteristic_length_m` | `0.4` | 特征长度：旋转残差与雅可比旋转行乘以该值，使平移/旋转量级可比。630 臂展约 630 mm，取 0.4 |
| `joint_limit_margin_rad` | `0.01` | 关节限位向内收缩量（弧度）。IK 边界、碰撞检查、路径规划都用它。0 表示允许贴到机械限位，不安全 |
| `min_scaled_jacobian_singular_value` | `0.003` | 奇异阈值：缩放雅可比最小奇异值低于此值的解被丢弃。调大更保守（解更少） |

> `ik_characteristic_length_m` 与臂展强相关。换机器人或大幅改工位后应重新标定，否则旋转项权重不合适，IK 会偏向"位置准但姿态差"或反之。

---

## 2. `camera` — 内参、量程与安装外参

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `description` | 占位说明 | 仅作文档 |
| `rgb.width` / `height` | `640` / `360` | 图像尺寸（像素）。四者与 `fx/fy` 必须为正 |
| `rgb.fx` / `fy` | `337.2` / `324.7` | 焦距（像素）。当前按 D405 标称视场估算，**不是实测内参** |
| `rgb.cx` / `cy` | `319.5` / `179.5` | 主点（像素），必须满足 `0 ≤ cx < width`、`0 ≤ cy < height` |
| `depth.*` | 与 `rgb` 完全相同 | 深度内参。校验强制与 `rgb` 逐字段相等，否则报错 |
| `T_CD` | 单位阵 | 深度系→彩色系的 4×4 变换。校验强制 `≈I`，即要求硬件已对齐的同内参 RGB-D |
| `depth_range_m` | `[0.07, 0.5]` | 深度有效量程（米，光学 Z）。ROI 任一点超出即拒绝候选；须 `lo < hi`。50 cm 是 D405 的"理想范围"上端，超出精度下降但非无效，高位工况可考虑放宽 |
| `mount.translation_F_m` | `[0, 0, 0.059]` | 光心在法兰系中的位置（米）。630 沿工具轴外移 59 mm——**占位值**，接入真实支架后必须重测 |
| `mount.rpy_FC_rad` | `[0, 0, 0]` | 相机系相对法兰系的固定 xyz 欧拉角（弧度）。`[0,0,0]` 表示光轴 = 法兰 +Z（工具轴） = 轴向安装 |
| `mount.description` | 占位说明 | 仅作文档 |
| `housing_size_m` | `[0.042, 0.042, 0.023]` | 相机外壳碰撞体尺寸（米）。只参与碰撞，不影响光学 |
| `bracket_size_m` | `[0.012, 0.012, 0.036]` | 支架碰撞体尺寸（米）。长度应覆盖 `translation_F_m` 的轴向偏移 |

> `mount` 决定相机朝向：当前是轴向安装（光轴 = 法兰 +Z，即工具轴）。若换成侧向支架，`rpy_FC_rad` 必须同步改，否则相机会指向错误方向，表现为大面积 `food_out_of_rgb_fov` 拒绝——症状容易误导，排查时先核对这个键。

---

## 3. `roi` — 采集工位几何

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `food_min_B` | `[0.31, -0.06, 0.02]` | 食材三维包围盒的最小角（米） |
| `food_max_B` | `[0.49, 0.06, 0.08]` | 最大角（米）。必须逐分量 `min < max`，否则报 `Food ROI must have positive dimensions` |
| `margin_m` | `0.002` | 空间裕量（米）：ROI 六向各扩此值后再做入镜与量程检查。用于覆盖定位误差，**不是图像边距** |
| `tray_z_B_m` | `0.02` | 托盘参考平面高度（B 系 Z，米）。相机光心 Z = 该值 + 高度 + δ；深度下限小于它 |
| `tray_corners_B` | X=[0.3,0.5]、Y=[−0.162,0.162]、Z=0.02 | 托盘四角（米），按顺序构成闭合四边形。决定烤盘投影占比与联合裁剪框，**必须与 `scene.tray` 实际几何一致** |
| `teach_timestamp` | `"example-not-measured"` | 示教时间戳。**当前无代码读取**，仅供人工记录 |
| `tcp_probe_offset_m` | `[0, 0, 0.1]` | 探针 TCP 偏移（米）。**当前无代码读取，也无校核逻辑** |

> `roi` 与 `scene` 是两套独立描述：`roi` 决定算法判据，`scene` 决定渲染与碰撞。两者不一致不会报错，但会导致"投影框/占比"和"画面里的托盘/食材"对不上。改一边必须同步另一边。

---

## 4. `sampling` — 随机采样与筛选

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `x_B_m` | `[0.33, 0.47]` | 光心 X 均匀采样区间（米），须 `min < max` |
| `y_B_m` | `[-0.07, 0.07]` | 光心 Y 均匀采样区间（米），须 `min < max`。调大提高多样性，但入镜/可达通过率下降 |
| `heights_above_tray_m` | `[0.2]` | 采样高度列表（相对托盘，米），**必须全为正**、非空。只有一个高度，Z 方向变化全部来自 `z_perturbation_m` |
| `z_perturbation_m` | `0.02` | 可选 Z 扰动幅度 Δ（米）：每个候选额外抽 δ ~ U(−Δ, +Δ)，光心 Z = `tray_z_B_m + h + δ`。约束 `0 ≤ Δ < min(heights)`；当前 0.02 < 0.2 合法，即 Z 在 0.200–0.240 m 之间浮动。δ 逐候选记入 `views.json`，逐高度记入 `summary.json` 的 `heights[].z_perturbations_m` |
| `target_per_height` | `4` | 每个高度要采集的有效视点数 N，须 ≥ 1。**总输出量 = 层数 × N**，当前 = 1 × 4 |
| `max_attempts_per_height` | `250` | 每个高度的候选尝试上限，须 ≥ 1。不够会导致 `complete=false`、进程退出码 2；调参后先看 `summary.json` 的拒绝原因分布 |
| `max_tilt_deg` | `40` | 光轴相对 Base −Z 的最大倾角（度）。**当前几何下永不生效**：由 XY 范围与 ROI 中心反算，含 δ 时可达上限约 33.4°，实测最大 23.3°。属诊断项；若设得比可达上限小，会整层静默全拒 |
| `min_axis_angle_deg` | `6` | 视角去重阈值（度）：任意两个已接受视点的光轴夹角必须 ≥ 该值。取值 `[0, 180]`，0 表示关闭去重 |
| `deduplicate_across_heights` | `true` | 去重作用域：`true` 跨所有高度全局比对，`false` 每层内部独立比对。当前只有一层，两种取值等价 |
| `reference_up_B` | `[0, 1, 0]` | Look-at 的参考上方向（B 系）。需非零；与视线近似平行时（`\|d·u\| > 0.98`）自动换轴避免退化 |
| `image_margin_fraction` | `[0.05, 0.05]` | 图像安全边距，占宽/高的比例 `[水平, 竖直]`，用于"完整入镜"判定。5% 是按几何可行性下调的实测值，**不是误差预算结论** |
| `tray_min_ratio_by_height` | `[0.0]` | 烤盘投影占比硬阈值，**长度必须等于 `heights_above_tray_m`**（当前都是 1 项），否则运行时报 `must have one entry per configured height`。0 表示该高度不启用 |

> **高度与扰动的关系**：先按 `heights_above_tray_m` 分层，每层再在 `[h−Δ, h+Δ]` 内抖动。当前单层 0.2 m + Δ=0.02，实际 Z 覆盖 0.200–0.240 m。若恢复多层，建议 Δ 不超过层间距的一半，否则层间覆盖范围交叠，"分层"会失去意义。
>
> **δ 不影响硬约束**：δ 只改采样高度，几何入镜、深度量程、IK、碰撞、路径、图像质量全部照常执行。δ 把相机推到托盘附近时会被 `geometry_check` 拒绝，这是预期行为。

---

## 5. `motion` — 碰撞检查与回放

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `collision_margin_m` | `0.002` | 碰撞安全间隙（米），须 ≥ 0。向所有几何对膨胀此距离 |
| `edge_resolution_rad` | `0.025` | 关节插值最大步长（弧度），须 > 0。路径碰撞按该步长离散采样；调大更快但可能漏检薄壁穿透 |
| `rrt_step_rad` | `0.18` | RRT-Connect 扩展步长（弧度） |
| `rrt_iterations` | `500` | RRT 迭代上限。上限内找不到路径只代表本次规划失败，不证明空间中无路 |
| `replay_seconds_per_edge` | `0.05` | `run --gui` 采集回放时每步休眠时间（秒）。仅影响观感，不影响运动学结果 |
| `replay_speed_rad_per_s` | `1.2` | `replay` 的关节空间恒速（弧度/秒），须 > 0 |
| `replay_pause_seconds` | `0.5` | `replay` 在每个采集位姿的停顿（秒），须 ≥ 0，此时绘制视锥 |

---

## 6. `quality` — 图像质量验收

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `min_visible_fraction_per_food` | `0.9` | 每块食材至少该比例的独立投影轮廓可见（分母是无遮挡参考轮廓，已计入食材间互相遮挡）。0–1。不达标拒绝原因 `occlusion` |
| `min_valid_depth_fraction` | `0.98` | 食材可见区域内有效深度像素的最低比例。不达标 `invalid_depth` |
| `min_food_pixels` | `100` | 每块食材最少可见像素数。不达标 `insufficient_food_pixels` |
| `min_food_bbox_side_px` | `6` | 食材外接框最短边长（像素）。不达标同上 |
| `crop.enabled` | `false` | 是否额外输出联合裁剪 `crop_*`。必须是布尔值。**默认关闭**，主输出恒为全幅 640×360 |
| `crop.padding_px` | `10` | 裁剪内边距（像素）。须 ≥ 0 且 < `min(width, height)/2` = 180。裁剪不做缩放，主点按偏移更新 |

> 旧配置兼容：缺 `crop` 键时按关闭处理，`quality.crop_padding_px`（旧拼写）作为内边距兜底。

---

## 7. `scene` — 仿真场景几何

所有 box 的 `position_B_m` 是**几何中心**，`size_m` 是**全尺寸**（不是半尺寸）。

### 7.1 `table` / `tray`

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `table.position_B_m` | `[0.4, 0, -0.086]` | 台面中心（米）。台面顶 Z = −0.086 + 0.1/2 = **−0.036** |
| `table.size_m` | `[0.6, 0.5, 0.1]` | 台面全尺寸 |
| `tray.position_B_m` | `[0.4, 0, 0.014]` | 托盘中心 |
| `tray.size_m` | `[0.2, 0.324, 0.012]` | 托盘全尺寸。**顶面必须等于 `roi.tray_z_B_m`**（=0.02），否则校验报错；底面落在层架上 |

### 7.2 `oven` — 烤箱腔体

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `_comment` | 说明文字 | 仅作文档，程序不读取（含 `_` 前缀的键都不参与计算） |
| `x_m` | `[0.24, 0.62]` | 腔体 X 范围（米）。−X 面是门洞（机器人侧），+X 面是前侧挡板。须 `x0 < x1` |
| `y_m` | `[-0.176, 0.176]` | 腔体外侧 Y 范围（米）。左右侧壁各占 `wall_thickness_m` |
| `floor_top_z_m` | `-0.03` | 底板顶面（米）。必须低于 `roi.tray_z_B_m` |
| `floor_thickness_m` | `0.006` | 底板厚度（米），须 > 0 |
| `wall_thickness_m` | `0.012` | 侧壁与前侧挡板厚度（米），须 > 0 且 `2t < min(x1−x0, y1−y0)`（否则没腔体） |
| `height_m` | `0.19` | 侧壁高度（米），须 > 0。壁顶 Z = `floor_top_z_m + height_m` = 0.16 |
| `top` | `false` | 是否加盖。默认不加：盖子会挡住 200 mm 基准高度上浮后的取景 |

生成的四面结构（程序按约束推导，不是手填坐标）：

- 底板：覆盖整个 `x_m × y_m`
- 左右侧壁：沿 X 贯穿整个腔深，厚 `wall_thickness_m`
- 前侧挡板：位于 +X，**宽度 = 腔体外宽 `y_m` 全域**，因此与左右侧壁在角部重叠，不留缝
- −X 面开口（机器人进出与取景）

校验会检查托盘完全落在腔体内（三面墙内、前挡板内侧），否则报 `scene.tray must fit inside the oven cavity`。

### 7.3 `rack` — 挂壁钢丝层架

| 键 | 当前值 | 含义与约束 |
|---|---|---|
| `_comment` | 说明文字 | 仅作文档 |
| `margin_m` | `0.002` | 层架比托盘每边外扩的余量（米），须 ≥ 0。层架 X 范围 = 托盘 X ± 此值，并夹到腔内 |
| `wire_size_m` | `0.003` | 钢丝截面边长（米），须 > 0。层架在 Z 方向只占这一个厚度 |
| `longitudinal_rails` | `3` | 纵条数，须 ≥ 2。**最外两根贴在左右侧壁内表面上，是承载件** |
| `cross_slats` | `5` | 横档数，须 ≥ 1。横档两端顶到两侧壁，贯通腔内全宽 |

层架**挂在侧壁上、无落地支腿**：底面离腔底 35 mm，顶面 = 托盘底面（自动推导，不需手填）。校验要求腔底与托盘底之间留出大于 `2×wire_size_m` 的空间，否则报 `Oven floor is too close to the tray for a shelf`。

### 7.4 `obstacles`

`[]`，空数组。每项为 `{name, position_B_m, size_m}`，会作为额外静态障碍参与碰撞。真实烤箱的门框、层架、线缆可在此补充。

### 7.5 `foods` — 食材

每项：

| 键 | 含义与约束 |
|---|---|
| `name` | 名称，用于碰撞报告与 `metrics[].name` |
| `shape` | 只能是 `"box"` 或 `"sphere"`，其他值报 `Only box/sphere food shapes are supported` |
| `position_B_m` | 几何中心（米） |
| `size_m` | 仅 `box` 需要，全尺寸（米） |
| `radius_m` | 仅 `sphere` 需要，半径（米） |
| `rgba` | 0–1 的颜色，仅影响渲染外观 |

约束：每块食材必须**完整落在未加裕量的 Food ROI 内**（`food_min_B`/`food_max_B`），否则校验报 `Food X exceeds the unpadded Food ROI`。实例 ID 按列表顺序分配（1..N），与 `instance.png` 一致。

当前三块食材底面都在 Z = 0.02（= 托盘上表面），即"坐在托盘上"。

---

## 8. 调整时的常用联动

| 想改什么 | 需要同步修改 |
|---|---|
| 托盘尺寸 | `scene.tray.size_m` + `roi.tray_corners_B`（两者的 X/Y 范围必须一致） |
| 托盘高度 | `scene.tray.position_B_m[2]` + `roi.tray_z_B_m`（保持顶面相等）+ 所有食材 Z |
| 腔体尺寸 | `scene.oven.x_m/y_m` + `scene.tray` 必须仍落在腔内 + `sampling.x_B_m/y_B_m` 采样范围 |
| 高度列表 | `sampling.tray_min_ratio_by_height` 必须变成等长 |
| 机器人型号 | `robot.urdf/srdf/flange_link/base_position_B_m/home_q_rad/ik_characteristic_length_m` + `camera.mount`（安装方向随末端坐标系变），并按需替换 `assets/` 下的模型 |
| 启用真实支架 | `camera.mount` + `camera.housing_size_m` + `camera.bracket_size_m`，并复核自碰撞豁免（当前豁免要求接触点距法兰轴 <12 mm、轴向 <3 mm） |

---

## 9. 校验覆盖范围（当前实现）

`check --config` 会验证：单位字面量、相机尺寸/焦距/主点、RGB 与 Depth 内参一致性、`T_CD≈I`、ROI 正尺寸、深度量程、计数与高度非空、去重角度范围、高度为正、Z 扰动类型/非负/小于最小高度、XY 区间正向、碰撞与回放参数、裁剪开关与内边距、托盘顶面等于 `roi.tray_z_B_m`、烤箱尺寸/壁厚/腔体、托盘落在腔内、层架参数与空间、食材落在 ROI 内。

`check` 还会实例化场景并检查 home 姿态是否碰撞，输出关节名列表；home 碰撞时退出码为 2。

**不会**验证的：未知键（拼错静默失效）、`roi` 与 `scene` 的语义一致性、`tray_corners_B` 与 `scene.tray` 的一致性、`tcp_probe_offset_m`（无代码读取）、`max_tilt_deg` 是否可达。
