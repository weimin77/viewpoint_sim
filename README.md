# 机械臂多高度随机视点采集仿真

按《视点规划技术调研》的随机采样方案实现的可运行工程。使用 **大象机器人 myCobot 630 Pro 官方 URDF + PyBullet + 腕部理想 RGB-D 相机**，完成从候选生成到图像、深度、实例 Mask 和路径的导出。

| 模型 | 资产 | 配置 | 示例输出 |
|---|---|---|---|
| myCobot 630 Pro（臂展 630 mm） | `assets/mycobot_630_pro/` | `configs/default.json` | `outputs/demo/` |

先打开随包附带的 `outputs/demo/report.html`，无需安装依赖即可浏览已运行的结果。三维运动预览为同目录下的 `motion_preview.gif`。运行和修改仿真需要按下文安装 Python 依赖。

参数逐项说明见 `docs/config_reference.md`。

## 1. 当前实现范围

| 文档要求 | 工程实现 | 位置 |
|---|---|---|
| Food 3D ROI 与独立 Tray ROI | Base 坐标下的包围盒与托盘四角，外加空间裕量 | `configs/default.json` |
| 固定相机光心高度 | `camera_z_B = tray_z_B + h`，每个高度独立随机采样 | `planner.py` |
| 可选 Z 扰动 Δ | `camera_z_B = tray_z_B + h + δ`，`δ ~ U(−Δ, +Δ)`；默认启用 Δ=0.02 m，δ 逐候选记录 | `planner.py` |
| XY 均匀随机采样 | 独立随机流、固定种子、最大尝试次数 | `planner.py` |
| Look-at | 光学系 X 右、Y 下、Z 前，参考方向退化处理 | `geometry.py` |
| 完整入镜 | 8 顶点针孔投影、5% 图像边距、RGB/Depth 共同视场 | `planner.py` |
| 深度量程 | 逐候选检查整个扩张 ROI 的光学 Z | `planner.py` |
| 烤盘投影占比 | 多边形与画面求交，可按高度设置硬阈值；输出占比排序 | `geometry.py`、`summary.json` |
| 相机到法兰变换 | `T_BF = T_BC @ inverse(T_FC)`，同时处理平移和旋转 | `geometry.py`、`planner.py` |
| IK / 限位 / 奇异状态 | 官方 URDF 正运动学、带边界多初值 SE(3) 逆解、雅可比阈值 | `kinematics.py` |
| 终点碰撞 | 官方网格的凸碰撞体、SRDF、相机和支架、工位结构 | `world.py` |
| 独立运动路径检查 | 关节直连优先，失败使用 RRT-Connect；整段插值碰撞检查 | `motion.py` |
| 视角多样性 | 光轴最小夹角，可跨高度去重；使用实际 FK 位姿复核 | `planner.py` |
| 食材实际可见性 | 完整场景可见像素 / 单食材独立渲染轮廓，含食材间互相遮挡 | `acquisition.py` |
| 到位、采集与质量检查 | 沿已验路径进行运动学回放、FK 到位误差复核、理想图像质量验收 | `planner.py` |
| 输出尺寸 | 始终输出全幅 640×360；联合裁剪为可选附加产物（默认关闭）| `acquisition.py` |
| 高度变化方式对比 | 相同 XY 的直接平移与重新 Look-at；仅做几何对比，标注未做机器人检查 | `height_comparison.json` |
| 不足 N 个有效位姿 | 输出失败统计、`complete=false`，命令退出码 2，保留合格样本 | `summary.json` |

文件名为 `viewpoint_sim/` 包内对应模块。当前采集按“随机候选中先通过先接受”执行；烤盘占比排序供结果选片，不重新排列已经验证的执行路径。

## 2. 安装与启动

### Ubuntu / WSL2

推荐 **Python 3.11**。本次环境的 Python 3.12 安装 PyBullet 3.2.7 会走源码编译，因此脚本用 uv 创建独立 3.11 环境。需要联网下载 Python 和四个依赖，机械臂模型已包含在工程内。

先安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)。若已安装 pipx，可用 `pipx install uv`，再确认 `uv --version` 能运行。

在解压后的工程目录运行：

```bash
bash scripts/setup.sh

# 生成一批新的数据；输出目录须为空，避免覆盖已有采集
.venv/bin/python -m viewpoint_sim run --headless --output outputs/my_run

# 回放随包附带的已验证路径（无桌面环境必须加 --headless）
.venv/bin/python -m viewpoint_sim replay --result outputs/demo --headless
```

也可以在采集时显示三维窗口：

```bash
.venv/bin/python -m viewpoint_sim run --gui --output outputs/gui_run
```

绿色线框为 Food ROI，黄色为 Tray ROI，蓝色矩形为各采样高度，绿色小视锥为拍照视点。规划查询会临时改变关节状态；检查实际运动时使用 `replay`。

GUI 视角操作（PyBullet 仅在按住修饰键时移动相机，`run --gui` 与 `replay` 都适用）：

| 操作 | 效果 |
|---|---|
| `Ctrl` 或 `Alt` + 左键拖动 | 绕观察点旋转（改变 yaw / pitch）|
| `Ctrl` 或 `Alt` + 中键拖动 | 平移观察点（沿屏幕上下、左右移动视野）|
| `Ctrl` + 右键拖动 | 拉近 / 拉远（改变相机到观察点的距离）|
| 滚轮 | 拉远 / 拉近 |

不按修饰键时左键拖动不会旋转视角，也不会拖动场景物体。PyBullet 把相机距离钳制在 `[1, 1000]` 米，默认初始距离 0.62 m 已小于下限，所以第一次“放大”会先跳到 1.0 m 再继续。缩放和旋转在动画播放期间同样有效；`replay` 只在启动时设置一次相机，不会覆盖你调整后的视角。

`replay` 按 `motion.replay_speed_rad_per_s`（默认 1.2 rad/s）在关节空间恒速播放，到达每个采集位姿后停顿 `motion.replay_pause_seconds`（默认 0.5 s），此时绘制视锥。路径碰撞复核在独立的无窗口 PyBullet 客户端上完成，因此复核不会让可见机器人在动画开始前先瞬移到轨迹终点。

本次在 Linux 的 **DIRECT 模式与 CPU TinyRenderer** 上运行和验证。GUI 需要本地图形桌面；WSL2 需可用的 WSLg。若 GUI 无法启动，先用 `--headless` 生成数据并打开 HTML 结果页。

### Windows PowerShell

安装 [uv](https://docs.astral.sh/uv/getting-started/installation/) 后，在工程目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup.ps1
.venv/Scripts/python.exe -m viewpoint_sim run --headless --output outputs/my_run
.venv/Scripts/python.exe -m viewpoint_sim replay --result outputs/demo
```

Windows 启动脚本已提供，尚未在本次 Linux 执行环境中运行验证。

### 浏览输出

运行后双击 `outputs/my_run/report.html`。页面不依赖 CDN、服务器或联网，可切换高度、查看工位图、相机投影、RGB、Mask、质量指标和拒绝原因；开启裁剪时还会多出「裁剪」标签页。

### 生成三维运动预览 GIF

`run` 只负责采样、校验与导出数据，**不生成动画**。GIF 由 `replay` 从结果目录里已保存的路径重放得到：

```bash
.venv/bin/python -m viewpoint_sim replay --result outputs/my_run \
    --headless --gif outputs/my_run/motion_preview.gif
```

- **`--headless` 必须加**：不加会尝试打开 PyBullet GUI 窗口，然后进入「等待窗口关闭」的循环，在没有图形桌面的环境里表现为卡住（Ctrl+C 中断后退出码为 1）。
- `--gif` 的路径自己指定；`run` 不会自动生成，也不认识这个参数。
- 想先看窗口再导出可以省略 `--headless` 并保留 `--gif`，两者可以同时用。
- GIF 是「第三人称场景 + 相机画面」的双视图（1000×440），按 `motion.replay_speed_rad_per_s` 恒速播放，每个采集位姿停顿 `motion.replay_pause_seconds`。
- 典型规模：12 个视点约 490 帧、30 s、约 6 MB；生成耗时与播放时长相当（约 30 s）。实际帧数随路径长度变化。
- 随包附带的 `outputs/demo/motion_preview.gif` 就是这样生成的；每个新的结果目录需各自执行一次本命令。

需要停顿时间更短或速度更快时，改 `motion.replay_speed_rad_per_s` 与 `motion.replay_pause_seconds` 后重跑 `replay`；这两个键只影响回放，不改变已导出的数据。

## 3. 默认场景与需要替换的参数

默认是 **前侧开口、顶部敞开的示例烤箱工位**：腔体由底板、左右侧壁和机器人正前方（+X）的前侧挡板构成，左右侧壁之间挂着一层钢丝层架，层架上放托盘，托盘上放三块几何食材。按 myCobot 630 Pro 的臂展布置。

前侧挡板与左右侧壁在角部**重叠**（挡板宽度等于腔体外宽、位置与侧壁末端对齐），因此不会出现缝隙；−X 面是机器人进出与相机取景的门洞。层架的两条外侧钢条贴在左右侧壁内表面上、横档贯通腔内全宽，因此层架是**挂在侧壁上**的，不落地；层架顶面直接取托盘底面，所以“托盘放在层架上、食材放在托盘上”由构造保证，不会因为改尺寸而错位。

顶部默认敞开：`scene.oven.top = false`。加上盖板会挡住 200 mm 基准高度上浮后的取景，只在改用更低视角时才有意义。

| 参数 | 示例值 / 含义 |
|---|---|
| 烤箱腔体 | X=[240,620]，Y=[−176,176] mm |
| 烤箱壁厚 / 底板 | 侧壁与挡板 12 mm，底板 6 mm；腔底标高 Z = −30 mm，壁顶 Z = 160 mm |
| 开口面 | −X（机器人侧）与顶面；其余三面封闭 |
| 钢丝层架 | 204 × 328 mm（横档贯通腔内全宽）。3 根纵向钢条（最外两根贴在左右侧壁内表面上）+ 5 根横档，丝径 3 mm；层架顶面 = 托盘底面，离腔底 35 mm，**无落地支腿** |
| 托盘 | 200 × 324 × 12 mm，四边各比层架内缩 2 mm，即几乎铺满层架；表面 Base Z = 20 mm |
| 食材 | 坐在托盘上表面（Z = 20 mm），与 `roi.food_min_B` 的 Z 下限一致 |

### 3.1 630 Pro 的模型改动与换型方式

**本工程对官方 URDF 的两处改动**（详见 `assets/mycobot_630_pro/provenance.json`）：

1. 上游 URDF 的 XML 声明写成 `version="2.0"`，不是合法 XML 版本，任何解析器都会拒绝，因此 `mycobot.urdf` 改为 `1.0`。
2. 上游没有独立法兰 link，链终止在 `J6_Link`。`J6_Link` 的工具板法向为 J6 的 −Y，且 joint6 转轴同为 (0,−1,0)；据此补充了固定 link/joint `flange` / `joint6_flange`，位于工具板外 2 mm。六个关节的原点、轴与限位均未改动。

与之对应，`robot.flange_link` 填 `flange`，`base_position_B_m` 为 `[0,0,0]`（底座高 0.16 m，直接坐在台面上），`ik_characteristic_length_m` 取 0.4（臂展约 630 mm），`home_q_rad` 为 `[0,-0.4,0.8,-1.2,0,0]`。

相机支架与外壳尺寸是**占位几何**，官方模型不含末端夹具；接入真实支架后必须重新填写 `camera.mount`、`camera.housing_size_m`、`camera.bracket_size_m` 并复核可达性与自碰撞（当前豁免要求接触点距法兰轴 <12 mm、轴向 <3 mm）。

代码本身不绑定型号：`kinematics.py` 直接从 URDF 读取关节名、轴与限位，`world.py` 从 SRDF 读取允许碰撞矩阵。若要换成别的机器人：

1. 在 `assets/` 下新建模型目录，放入 URDF、SRDF、网格与 `provenance.json`；
2. 改 `robot.urdf / srdf / flange_link / base_position_B_m / home_q_rad / ik_characteristic_length_m`；
3. 按新末端坐标系重写 `camera.mount`，并按需调整 `roi` 与 `scene`；
4. 运行 `check` 确认 home 姿态无碰撞，再跑一次完整采集。

`configs/default.json` 每个参数的逐项说明（含义、单位、取值范围、校验规则、联动项）见 **`docs/config_reference.md`**。

| 参数 | 示例值 / 含义 |
|---|---|
| 托盘 | 200 × 324 × 12 mm，底面落在层架上，上表面 = `roi.tray_z_B_m` = 20 mm |
| Food ROI | X=[310,490]，Y=[-60,60]，Z=[20,80] mm |
| 空间裕量 | ROI 向各方向扩 2 mm |
| 相机基准高度 | 200 mm（单层） |
| Z 扰动幅度 Δ | 0.02 m（±20 mm）；每候选独立抽 δ ~ U(−Δ, +Δ)，须满足 Δ < 最小配置高度，实际光心 Z 覆盖 0.200–0.240 m |
| XY 采样范围 | X=[330,470]，Y=[-70,70] mm |
| 相机画面 | 640 × 360，理想无畸变针孔 |
| 内参 | 从标称视场近似，**不是 D405 实测内参** |
| 深度范围 | 光学 Z 70–500 mm，实际有效性另需实测 |
| 相机安装 | 光心在法兰 `[0,0,59]` mm，RPY=`[0,0,0]`（轴向，光轴 = 法兰 +Z = 工具轴）；占位支架，须替换为实际外参 |
| 视角去重 | 至少 6°，`deduplicate_across_heights` 控制是否跨高度 |
| 每高度数量 | 8，最多尝试 250 次 |
| 图像边距 | 水平、竖直各 5% |
| 输出裁剪 | 默认关闭（全幅输出）；`quality.crop.padding_px=10` 仅在内边距开启时生效 |
| 可见率阈值 | 每块食材至少 90% 的独立投影轮廓可见 |
| 路径碰撞 | 2 mm 间隙；关节插值最大步长 0.025 rad |
| 回放 | 关节空间恒速 1.2 rad/s；每个采集位姿停顿 0.5 s |

所有 JSON 长度单位是 **米**，角度单位是 **弧度**。所有坐标都在固定 Base 参考系中；`base_position_B_m` 仅用于调整模拟机器人在该工位参考系中的位置，默认为原点。

先复制 `configs/default.json` 为自己的配置，再运行：

```bash
.venv/bin/python -m viewpoint_sim check --config configs/my_workcell.json
.venv/bin/python -m viewpoint_sim run --config configs/my_workcell.json --headless --output outputs/my_workcell
```

关键修改位置：

1. 用实际 TCP 探针尖端示教坐标设置 `roi.food_min_B / food_max_B` 和 `tray_corners_B`，确认 `tray_z_B_m`。
2. 更新 `scene.tray`、`scene.obstacles` 和 `scene.foods`，保证模拟几何与 ROI 对应。
3. 用 SDK 内参替换 `camera.rgb / depth`，用支架测量或标定结果替换 `camera.mount`。
4. 修改高度列表后，同步修改等长的 `tray_min_ratio_by_height`；0 表示不启用该高度的原图占比硬阈值。
5. 用 `robot.home_q_rad` 设置实际准备姿态，再检查整个转移路径。
6. 需要 Z 扰动消融时设置 `sampling.z_perturbation_m`（米）。该值必须小于最小配置高度；每个候选的 δ 写入 `views.json` 的 `z_perturbation_m`，逐高度记录在 `summary.json` 的 `heights[].z_perturbations_m`。δ 只改变采样高度，几何、深度量程、IK、碰撞、路径与图像质量检查全部照常执行。

首版支持 **已经对齐且内参相同的 RGB/Depth**，`T_CD=I`。配置校验会拒绝独立光学系或不同内参，避免直接把未对齐深度当作像素对齐数据。畸变也未模拟。

相机安装方向和偏移会影响可达性。myCobot 630 Pro 使用本项目补充的 `flange` 坐标系作为法兰，附加占位相机与支架；换上真实支架或夹爪后需要重新校验可达范围、自碰撞豁免与工作高度。

## 4. 输出格式

每个有效位姿对应 `hXX_cXXXX/`：

| 文件 | 内容 |
|---|---|
| `rgb.png` | 原始 RGB |
| `depth_mm.png` | uint16，光学 Z 毫米，0 表示背景或量程无效 |
| `depth_z_m.npy` | float32，渲染的几何光学 Z 米值，尚未按量程置零 |
| `instance.png` | uint16，可见实例 ID；0=背景，1..N 对应配置食材顺序 |
| `food_mask.png` | 二值可见食材 Mask |
| `crop_*.png` | **可选**：`quality.crop.enabled=true` 时额外输出的联合裁剪 RGB、Depth、实例与二值 Mask；默认不生成 |
| `overlay.png` | Food/Tray ROI 投影与裁剪框 |
| `overview.png` | 拍照时机械臂和工位的第三人称渲染 |
| `metadata.json` | 目标/实际相机和法兰位姿、安装外参、关节角、路径、`z_perturbation_m`、`camera_height_above_tray_m`、内参、`image_size_xy`、`roi_bbox_xyxy`、质量指标；开启裁剪时另有 `crop_bbox_*` / `K_crop` / `tray_ratio_cropped` |

运行根目录保存 `config.json`、`views.json`、`summary.json`、`candidates.jsonl`、`height_comparison.json` 和 `report.html`。

主输出 `rgb.png` / `depth_mm.png` / `instance.png` / `food_mask.png` **始终是全幅**，尺寸恒等于 `camera.rgb` 配置，因此每个视点的图像尺寸完全一致。`roi_bbox_xyxy` 是 Food + Tray 投影的并集外接框（仅记录，不裁剪），可直接用于下游定位。

裁剪是可选项：`quality.crop.enabled` 为 `true` 时额外输出 `crop_*`，其外接框 = `roi_bbox_xyxy` 加 `quality.crop.padding_px` 内边距并夹到图像边界，因此**裁剪尺寸逐视点不同**。裁剪不做缩放；主点按裁剪偏移更新，焦距不变。Mask 表示实际可见表面，不包含被遮住区域的推测标签。

若调整模型细节，建议把关节插值步长减半后重新验证关键路径。离散采样不能证明两个检测采样点之间不存在穿透。

### 4.1 汇总的 RGB 目录

`run` 结束时会自动把全部采集到的 RGB 复制到结果根目录的 `rgb/`，下游只需指向这一个目录，不必逐个遍历 `hXX_cXXXX/`：

```
outputs/demo/
  rgb/
    000_h00_c0016.png
    001_h00_c0017.png
    ...
    manifest.csv
  h00_c0016/        ← 每个视点仍保留完整数据（RGB-D、Mask、metadata…）
  views.json
  summary.json
```

| 文件 | 内容 |
|---|---|
| `NNN_hXX_cXXXX.png` | RGB 副本，`NNN` 为 `views.json` 的采集顺序，字母序即时间序 |
| `manifest.csv` | `file,id,height_index,height_above_tray_m,z_perturbation_m,camera_height_above_tray_m,folder` |

`summary.json` 中的 `rgb_folder` 记录该目录名。源视点目录保持只读，`rgb/` 只是副本。

需要其它图像类型、软链接或写入结果目录之外时，用脚本：

```bash
# 追加导出 Mask 与深度，不复制只建相对软链接
.venv/bin/python scripts/collect_rgb.py outputs/demo --kinds rgb,food_mask,depth_mm --link

# 写到结果目录之外（默认目标是 <result>/rgb）
.venv/bin/python scripts/collect_rgb.py outputs/demo --output datasets/rgb
```

| 参数 | 说明 |
|---|---|
| `--kinds` | `rgb, depth_mm, instance, food_mask, overlay, overview`，默认 `rgb`；非 RGB 类型文件名加后缀 |
| `--link` | 建相对软链接，省磁盘但源目录不能删；默认复制 |
| `--force` | 目标目录非空时允许写入，默认拒绝，避免新旧文件混在一起 |

## 5. 已执行的验证

默认种子 `20261106`，单层基准高度 200 mm、Δ=0.02 m、每层 8 个视点，共采到 8 个视点。每个样本都重新检查了实际 FK 位姿、整段路径、深度范围、可见性、全幅尺寸一致性、Mask/深度一致性，以及裁剪开关的两端行为。

| 指标 | 值 |
|---|---|
| 采样层数 × N | 1 × 8 |
| Z 扰动 Δ | 0.02 m（实际光心 Z 覆盖 0.2145–0.2360 m） |
| 有效视点 | 8 / 8 |
| 尝试次数 | 15 |
| 最小实际光轴夹角 | 6.40° |
| 烤盘占比范围 | 44.9%–52.9% |
| 相机倾角范围 | 10.7°–25.2° |
| 运行耗时 | 5.9 s |
| `verify_results.py` | PASS |

15 次尝试里有 7 次 `food_out_of_rgb_fov` 被拒，且**全部来自 δ 为负的候选**（−1.3 至 −17.4 mm）：相机降低使 ROI 在画面中变大，角点越过 5% 边距。这印证 δ 只改变采样位置、不绕过任何硬约束。没有出现 `ik_or_singularity` 拒绝。

验证命令：

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/verify_results.py outputs/demo

# 新目录中运行五类不可能/不合格工况，不能将它们强行接受
.venv/bin/python scripts/run_negative_cases.py outputs/my_negative_cases
```

测试含：Look-at 退化、安装外参变换闭合、凸包完整入镜、多边形裁切、裁剪主点更新、裁剪开关默认值与旧配置兜底、全幅导出尺寸、光学深度、偏置主点渲染、遮挡、自碰撞/环境碰撞、终点均安全但中段碰撞、RRT 绕行、Z 扰动的幅度边界与零振幅可复现性、扰动候选仍受几何约束、URDF 关节链/限位/法兰一致性，以及烤箱/层架场景几何。`docs/validation.md` 记录本次结果。

## 6. 仿真的边界

- 机械臂按关节配置进行运动学回放，没有模拟力矩、负载、动态跟踪误差或控制器稳定时间。
- 深度由理想渲染得到，没有模拟真实双目匹配、无纹理、反光、黑色材质、热环境、噪声和孔洞。
- 可见率依赖仿真中已知的食材几何；真实采集时须换为视觉或深度质量检查，不能直接套用仿真真值。
- 碰撞按官方网格的凸包近似进行检测；允许矩阵来自随包 SRDF，并额外按 URDF 把相邻连杆（父子关节）视为不可碰撞。额外只豁免相机安装连接件的接触；腕部与支架在法兰平面的微小非穿透安装接触使用局部位置条件。
- RRT 在迭代上限内没找到路径，仅代表本次规划失败，不证明空间中绝对没有路径。
- **630 Pro 的相机支架与外壳是占位几何**，官方模型不含末端夹具；法兰坐标系由本项目按 J6_Link 工具板法向补充。接入真实支架后必须重新填写 `camera.mount` 并复核自碰撞豁免条件——该豁免目前要求接触点距法兰轴 <12 mm、轴向 <3 mm，换成更宽的支架后需要重新评估。
- 标定误差、相机参数、真实安装方向、线缆和真实烤箱几何仍需补齐。本工程用于方案验证和数据生成，不构成真机安全验证。

## 7. 模型来源与许可

官方模型来自 [Elephant Robotics mycobot_ros](https://github.com/elephantrobotics/mycobot_ros)，固定提交 `c0d6fbe8f7d9b18b46a9fcb0b041e5bc2d66196b`（myCobot 630 Pro / Pro630）。下载来源、SHA-256 与改动记录见 `assets/mycobot_630_pro/provenance.json`。

`assets/mycobot_630_pro/original.urdf` 保留官方 `pro630.urdf`（该文件的 XML 声明为 `version="2.0"`，不是合法 XML，无法直接解析）。`mycobot.urdf` 做了三处改动：XML 声明改为 `1.0`、网格路径改为相对路径、在 `J6_Link` 工具板上补充固定 `flange` 坐标系。六个关节的原点、轴与限位未改动。

官方网格本身就是米制二进制 STL，无需转换；模型不携带贴图，因此仿真中机械臂使用默认灰度材质。

模型及 SRDF 的原始 BSD-3-Clause 许可见 `assets/mycobot_630_pro/LICENSE`。

离线结果页内嵌了 Noto Sans CJK 中文字体的界面字符子集，许可及来源保留在 `assets/fonts/`，减少系统缺少中文字体导致的方框字符。

参考：[PyBullet 官方指南](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html)、[D405 官方规格](https://www.realsenseai.com/product-family/d405-series/)、[uv Python 管理](https://docs.astral.sh/uv/guides/install-python/)。

