# MuJoCo + myCobot Pro 630 视点规划仿真

这是此前 PyBullet + PiPER 工程的独立迁移版本。仿真、碰撞距离查询、相机渲染和位置伺服均使用 **MuJoCo 3.3.7**，机械臂使用大象机器人官方 **myCobot Pro 630** URDF 和网格。工程不依赖 PyBullet。

本次请求中的“Pro 360”按此前已经确认的 **Pro 630** 处理。模型目录、报告和元数据均明确记录 Pro 630；本工程没有把其他机器人改名为“360”。如果实物型号不同，需要对应的官方模型。

默认是顶部开放，带背板、侧壁、内部导轨和金属线架的烤箱示例。托盘、Food ROI、相机安装和内腔高度仍为可修改的示例参数。

## 先查看已有结果

无需安装依赖，直接打开：

- `outputs/demo/report.html`：默认三高度采集、模型与 RGB-D/Mask 查看。
- `outputs/demo/motion_preview.gif`：运动路径与相机画面的同步预览。
- `outputs/jitter_and_crop/report.html`：启用 Z 扰动与可选裁剪的独立示例。

报告自包含中文界面字体，不依赖联网脚本。默认未生成裁剪图时，裁剪按钮不可用。

## Ubuntu / WSL 启动

已安装 uv 后，在**新工程目录**运行：

```bash
bash scripts/setup.sh

# 交互式 MuJoCo 三维回放；结束后关闭窗口
.venv/bin/python -m viewpoint_sim replay --result outputs/demo

# 使用位置伺服与物理步进回放
.venv/bin/python -m viewpoint_sim replay --result outputs/demo --physics

# 重新采集，输出目录需为空
.venv/bin/python -m viewpoint_sim run --headless --output outputs/my_run

# 显示三维窗口采集
.venv/bin/python -m viewpoint_sim run --gui --output outputs/my_gui_run
```

安装脚本默认创建 Python 3.12 环境，并使用清华 PyPI 镜像与 120 秒读取超时。可以自行指定其他源：

```bash
UV_DEFAULT_INDEX=https://pypi.org/simple bash scripts/setup.sh
```

无显示器的 Linux 默认使用 EGL；可显式运行：

```bash
MUJOCO_GL=egl .venv/bin/python -m viewpoint_sim run --headless --output outputs/my_egl_run
```

若提示系统缺少 EGL/OpenGL 动态库，Ubuntu 可安装 `libegl1 libgl1 libgl1-mesa-dri`。`MUJOCO_GL=osmesa` 是可选软件渲染路线，需要系统提供 OSMesa。WSL 图形窗口需要可用的 WSLg。显示环境和远程服务器环境分别选择对应后端。

本次实际验证的是 Linux EGL 离屏渲染与物理执行；交互窗口及 Windows 启动脚本需在本地图形环境运行。

## Windows PowerShell

```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup.ps1
.venv/Scripts/python.exe -m viewpoint_sim replay --result outputs/demo
.venv/Scripts/python.exe -m viewpoint_sim run --headless --output outputs/my_run
```

## 独立 MJCF 模型和自由查看

```bash
.venv/bin/python -m viewpoint_sim view
.venv/bin/python -m viewpoint_sim export-model --output models/default_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf models/default_scene.xml
```

`models/default_scene.xml` 引用相对路径的 OBJ 网格，可连同工程直接移动。使用独立查看器时，按 MuJoCo 的界面操作显示控制面板、切换相机和调整 actuator 控制量。碰撞凸包几何为 group 3，正常画面应关闭它；视觉模型为 group 1，工位为 group 0，食材为 group 2。

## 已迁移及新增内容

| 模块 | 实现 |
|---|---|
| 机械臂 | 官方 Pro 630 六关节、关节原点、轴方向、限位与视觉网格 |
| 格式转换 | COLLADA 场景变换 + 毫米到米转换，输出可供 MuJoCo 加载的 OBJ |
| 运动学 | URDF 正运动学、多初值有界 IK；与 MuJoCo 刚体位姿交叉验证 |
| 工位 | 顶部开放的采集烤箱工位、内部层架和导轨、托盘与几何食材 |
| 采样 | XY 均匀随机采样，基准高度加可选 Z 扰动 |
| 朝向 | Look-at；光学系 X 右、Y 下、Z 前，转换为 MuJoCo 相机系 |
| 成像检查 | Food ROI 投影、图像边距、光学 Z 量程、烤盘投影占比 |
| 碰撞 | MuJoCo 原生有符号距离；分开检查终点与插值路径 |
| 路径 | 关节直连优先，失败时 RRT-Connect；每条边重新检查 |
| 执行 | 可选运动学回放或 MuJoCo 位置伺服、物理步进、到位稳定判定 |
| 数据 | RGB、米制深度、毫米深度 PNG、原始分辨率实例/二值 Mask |
| 裁剪 | `none` 默认关闭；`tray_roi` 开启后另存同步裁剪副本 |
| 可见性 | 由已知模型计算的诊断指标；默认 `report`，不强制无遮挡 |
| 记录 | 实际光学位姿、法兰位姿、关节角、轨迹、基准高度、扰动量与执行状态 |

裁剪开关与 Z 扰动默认值遵循后来修订的技术方案。跨高度去重默认关闭，高度内部仍保持最小夹角要求。

## 修改场景与采集参数

默认文件为 `configs/default.json`，单位是米和弧度。建议先复制配置，再生成新输出。

| 配置项 | 默认示例 |
|---|---|
| 机器人 | `myCobot Pro 630`，法兰取官方 `link6` 坐标系 |
| 相机 | D405 风格的已对齐无畸变针孔，640×360；内参为示例 |
| 光心安装 | 法兰 `[0.075,0,0]` m，RPY `[0,π/2,0]`；光轴沿法兰 +X |
| 工位 | 深度 400 mm × 宽度 500 mm，侧壁高 645 mm；前方与顶部开放 |
| 托盘表面 | Base Z=95 mm；300×230 mm |
| Food ROI | X=[235,465]、Y=[-80,80]、Z=[95,165] mm，外扩 3 mm |
| 默认采集高度 | 距托盘 300、360、420 mm；每高度 4 个视点 |
| Z 扰动 | `sampling.z_jitter_m=0` |
| 裁剪 | `postprocess.crop_mode="none"` |
| 深度范围 | 光学 Z=[70,500] mm |
| 图片边距 | 水平和竖直各 10% |
| 图像深度有效率阈值 | 可见食材像素至少 98% 在配置量程内；理想渲染指标 |
| 执行模式 | `motion.execution_mode="position_control"` |
| 碰撞间隙 | 2 mm，路径采样步长 0.025 rad，均为示例 |

默认高度沿用 300/360/420 mm。顶部没有盖板，也不施加顶部碰撞限制；背板、侧壁、导轨和层架仍参与碰撞检查。

相机安装外参和末端坐标都是完整刚体变换，目标法兰由 `T_BF = T_BC @ inverse(T_FC)` 求得。示例准备姿态不等于真机上电位置；真机接入时应从实读状态重新规划。

启用可选分支：

```bash
.venv/bin/python -m viewpoint_sim run --config configs/jitter_and_crop.json --headless --output outputs/my_jitter_run
```

该配置在 330 mm 基准高度上加入 ±8 mm 均匀扰动，启用 `tray_roi`，目标两个样本；每个扰动位姿仍执行完整检查。原始数据保留。

## 数据格式

每个有效视点位于 `hXX_cXXXX/`：

- `rgb.png`：原始 RGB。
- `depth_mm.png`：uint16 光学 Z 毫米，0 表示背景或量程无效。
- `depth_z_m.npy`：float32 渲染光学 Z 米值，保留量程过滤前的值。
- `instance.png`：uint16 可见实例 ID；0=背景，1..N 对应配置顺序。
- `food_mask.png`：原始分辨率二值 Mask。
- `overlay.png / overview.png`：投影检查和第三人称工位图。
- `metadata.json`：相机/法兰目标与实际位姿、关节目标与实际值、路径、实际采样高度、扰动量、伺服步数和到位误差。
- `crop_*.png`：仅在显式启用裁剪时生成；同一裁剪框作用于 RGB/Depth/Mask，内参同步更新。

运行根目录包含 `config.json`、`views.json`、`summary.json`、`candidates.jsonl`、高度变化几何对比与 `report.html`。采样不足会保存 `complete=false` 并退出 2；硬约束不会自动放宽。

默认没有统一 resize 或 letterbox；裁剪分支是可选的原生像素裁剪，不预设统一模型输入尺寸。

## 验证

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/verify_results.py outputs/demo
.venv/bin/python scripts/verify_results.py outputs/jitter_and_crop
```

验证包括：单位换算、URDF/MuJoCo FK、光学坐标转换、非居中内参、米制深度、实例分割、可选裁剪、伺服稳定、开放顶部、夹具碰撞，以及“终点无碰撞但中间路径碰撞”。本次运行的具体结果见 `docs/validation.md`。

## 动力学与真机的边界

官方描述文件没有提供完整惯性信息，部分速度字段为 0。本工程保留运动学定义，同时为 MuJoCo 位置伺服增加了明确的**示例质量、惯量、阻尼、伺服增益和重力补偿**。这些用于验证程序流程，不用于评估真机力矩、负载、跟踪性能或安全制动。

碰撞采用官方视觉网格的凸包近似和随包 SRDF 允许矩阵。连续物理步进时也检查碰撞，但规划边的检查仍是离散采样。线缆、实物夹具、标定误差、控制器和真实深度噪声需根据实际平台补充。

相机输出是几何真值渲染，不等同于真实 D405 的双目匹配结果。实例 Mask 来自 MuJoCo 几何体 ID；真实自动标注需要独立的视觉或深度算法。默认可见率仅用于诊断，不以已知仿真真值替代实拍保证。

## 来源与许可

模型来自大象机器人 [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2)，固定提交 `d42ff61a78122c79246623391540d75738b03b23` 的 `mycobot_description/urdf/mycobot_pro_630`。原始 URDF/DAE、转换后的 OBJ、官方 SRDF、许可与 SHA-256 均保留在 `assets/mycobot_pro_630/`。模型采用 BSD 2-Clause，见该目录 `LICENSE`。

官方 URDF 和 SDK 的零位、限位及版本映射仍需在实物上核对；本工程没有假定它们可直接一一作为真机控制参数。

参考：[MuJoCo 3.3.7 XML](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html)、[MuJoCo Python](https://mujoco.readthedocs.io/en/3.3.7/python.html)、[官方 Pro 630 URDF](https://github.com/elephantrobotics/mycobot_ros2/blob/d42ff61a78122c79246623391540d75738b03b23/mycobot_description/urdf/mycobot_pro_630/mycobot_pro_630.urdf)。
