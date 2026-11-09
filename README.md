# 机械臂多高度随机视点采集仿真

按《视点规划技术调研》的随机采样方案实现的可运行工程。使用 **PiPER 官方 URDF + PyBullet + 腕部理想 RGB-D 相机**，完成从候选生成到图像、深度、实例 Mask 和路径的导出。

先打开随包附带的 `outputs/demo/report.html`，无需安装依赖即可浏览本次已运行的结果。三维运动预览为 `outputs/demo/motion_preview.gif`。运行和修改仿真需要按下文安装 Python 依赖。

## 1. 当前实现范围

| 文档要求 | 工程实现 | 位置 |
|---|---|---|
| Food 3D ROI 与独立 Tray ROI | Base 坐标下的包围盒与托盘四角，外加空间裕量 | `configs/default.json` |
| 固定相机光心高度 | `camera_z_B = tray_z_B + h`，每个高度独立随机采样 | `planner.py` |
| XY 均匀随机采样 | 独立随机流、固定种子、最大尝试次数 | `planner.py` |
| Look-at | 光学系 X 右、Y 下、Z 前，参考方向退化处理 | `geometry.py` |
| 完整入镜 | 8 顶点针孔投影、10% 图像边距、RGB/Depth 共同视场 | `planner.py` |
| 深度量程 | 逐候选检查整个扩张 ROI 的光学 Z | `planner.py` |
| 烤盘投影占比 | 多边形与画面求交，可按高度设置硬阈值；输出占比排序 | `geometry.py`、`summary.json` |
| 相机到法兰变换 | `T_BF = T_BC @ inverse(T_FC)`，同时处理平移和旋转 | `geometry.py`、`planner.py` |
| IK / 限位 / 奇异状态 | 官方 URDF 正运动学、带边界多初值 SE(3) 逆解、雅可比阈值 | `kinematics.py` |
| 终点碰撞 | 官方网格的凸碰撞体、SRDF、相机和支架、工位结构 | `world.py` |
| 独立运动路径检查 | 关节直连优先，失败使用 RRT-Connect；整段插值碰撞检查 | `motion.py` |
| 视角多样性 | 光轴最小夹角，可跨高度去重；使用实际 FK 位姿复核 | `planner.py` |
| 食材实际可见性 | 完整场景可见像素 / 单食材独立渲染轮廓，含食材间互相遮挡 | `acquisition.py` |
| 到位、采集与质量检查 | 沿已验路径进行运动学回放、FK 到位误差复核、理想图像质量验收 | `planner.py` |
| 联合裁剪 | Food + Tray 投影外接框，同步裁剪 RGB、Depth、Mask，更新主点 | `acquisition.py` |
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

# 打开三维窗口，回放随包附带的已验证路径
.venv/bin/python -m viewpoint_sim replay --result outputs/demo
```

也可以在采集时显示三维窗口：

```bash
.venv/bin/python -m viewpoint_sim run --gui --output outputs/gui_run
```

GUI 中可旋转、平移和缩放观察工位；绿色线框为 Food ROI，黄色为 Tray ROI，蓝色矩形为各采样高度，绿色小视锥为拍照视点。规划查询会临时改变关节状态；检查实际运动时使用 `replay`。

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

运行后双击 `outputs/my_run/report.html`。页面不依赖 CDN、服务器或联网，可切换高度、查看工位图、相机投影、RGB、Mask、裁剪结果、质量指标和拒绝原因。

## 3. 默认场景与需要替换的参数

默认是 **开放顶部、三面侧壁的示例采集工位**，包含一个托盘、三个几何食材。它用于先验证算法流程；真实烤箱的顶部、门框、相机支架与线缆需要根据实际结构补充。

| 参数 | 示例值 / 含义 |
|---|---|
| 托盘 | 300 × 230 mm，表面 Base Z = 20 mm |
| Food ROI | X=[235,465]，Y=[-80,80]，Z=[20,90] mm |
| 空间裕量 | ROI 向各方向扩 3 mm |
| 相机高度 | 距托盘 300、360、420 mm |
| XY 采样范围 | X=[200,470]，Y=[-150,150] mm |
| 相机画面 | 640 × 360，理想无畸变针孔 |
| 内参 | 从标称视场近似，**不是 D405 实测内参** |
| 深度范围 | 光学 Z 70–500 mm，实际有效性另需实测 |
| 相机安装 | 光心在法兰 `[0,0,70]` mm；RPY=`[π,-π/2,0]` |
| 安装方向 | 示例侧向安装，使相机光轴为法兰 +X；须替换为实际支架外参 |
| 视角去重 | 至少 6°，跨高度生效 |
| 每高度数量 | 4，最多尝试 250 次 |
| 图像边距 | 水平、竖直各 10% |
| 可见率阈值 | 每块食材至少 90% 的独立投影轮廓可见 |
| 路径碰撞 | 2 mm 间隙；关节插值最大步长 0.025 rad |

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

首版支持 **已经对齐且内参相同的 RGB/Depth**，`T_CD=I`。配置校验会拒绝独立光学系或不同内参，避免直接把未对齐深度当作像素对齐数据。畸变也未模拟。

相机安装方向和偏移会影响可达性。默认 PiPER 移除了夹爪，装上示例相机与支架；保留夹爪时需要补回模型并重新校验。

## 4. 输出格式

每个有效位姿对应 `hXX_cXXXX/`：

| 文件 | 内容 |
|---|---|
| `rgb.png` | 原始 RGB |
| `depth_mm.png` | uint16，光学 Z 毫米，0 表示背景或量程无效 |
| `depth_z_m.npy` | float32，渲染的几何光学 Z 米值，尚未按量程置零 |
| `instance.png` | uint16，可见实例 ID；0=背景，1..N 对应配置食材顺序 |
| `food_mask.png` | 二值可见食材 Mask |
| `crop_*.png` | 同一裁剪框下的 RGB、Depth、实例与二值 Mask |
| `overlay.png` | Food/Tray ROI 投影与裁剪框 |
| `overview.png` | 拍照时机械臂和工位的第三人称渲染 |
| `metadata.json` | 目标/实际相机和法兰位姿、安装外参、关节角、路径、内参、裁剪框、质量指标 |

运行根目录保存 `config.json`、`views.json`、`summary.json`、`candidates.jsonl`、`height_comparison.json` 和 `report.html`。

`crop_bbox_xyxy` 的右下边界为 Python 切片的排他边界。裁剪不做缩放；主点按裁剪偏移更新，焦距不变。Mask 表示实际可见表面，不包含被遮住区域的推测标签。

若调整模型细节，建议把关节插值步长减半后重新验证关键路径。离散采样不能证明两个检测采样点之间不存在穿透。

## 5. 已执行的验证

本次默认种子 `20261106`，三个高度共采到 12 个视点。每个样本都重新检查了实际 FK 位姿、整段路径、深度范围、可见性和裁剪一致性。

验证命令：

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/verify_results.py outputs/demo

# 新目录中运行五类不可能/不合格工况，不能将它们强行接受
.venv/bin/python scripts/run_negative_cases.py outputs/my_negative_cases

# 从已验证路径生成第三人称与相机画面的同步预览
.venv/bin/python -m viewpoint_sim replay --result outputs/demo --headless --gif outputs/my_preview.gif
```

测试含：Look-at 退化、非零安装外参、凸包完整入镜、多边形裁切、裁剪主点更新、光学深度、偏置主点渲染、遮挡、自碰撞/环境碰撞、终点均安全但中段碰撞，以及 RRT 绕行。`docs/validation.md` 记录本次结果。

## 6. 仿真的边界

- 机械臂按关节配置进行运动学回放，没有模拟力矩、负载、动态跟踪误差或控制器稳定时间。
- 深度由理想渲染得到，没有模拟真实双目匹配、无纹理、反光、黑色材质、热环境、噪声和孔洞。
- 可见率依赖仿真中已知的食材几何；真实采集时须换为视觉或深度质量检查，不能直接套用仿真真值。
- 碰撞按官方网格的凸包近似进行检测；允许矩阵来自随包 SRDF。额外只豁免相机安装连接件的接触；腕部与支架在法兰平面的微小非穿透安装接触使用局部位置条件。
- RRT 在迭代上限内没找到路径，仅代表本次规划失败，不证明空间中绝对没有路径。
- 标定误差、相机参数、真实安装方向、线缆和真实烤箱几何仍需补齐。本工程用于方案验证和数据生成，不构成真机安全验证。

## 7. 模型来源与许可

官方模型来自 [AgileX piper_ros](https://github.com/agilexrobotics/piper_ros)，固定提交 `ac41fcbcdda598f01b51cf6175ed9a24d0dacadc`。`assets/piper/original.urdf` 保留原始文件；`piper.urdf` 仅移除夹爪、调整资源路径、补上 dummy_link 的零质量惯性，没有修改六个关节的运动学参数。

该版本 URDF 的固件参数说明见官方 README；接入真机前确认固件与 URDF 版本匹配。模型及 SRDF 的原始 MIT 许可见 `assets/piper/LICENSE`，下载来源与 SHA-256 记录见 `provenance.json`。

离线结果页内嵌了 Noto Sans CJK 中文字体的界面字符子集，许可及来源保留在 `assets/fonts/`，减少系统缺少中文字体导致的方框字符。

参考：[PyBullet 官方指南](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html)、[D405 官方规格](https://www.realsenseai.com/product-family/d405-series/)、[uv Python 管理](https://docs.astral.sh/uv/guides/install-python/)。
