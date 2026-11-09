# 本次验证记录

日期：2026-11-09（北京时间）。运行环境：Linux x86_64，Python 3.11.17，PyBullet 3.2.7，CPU TinyRenderer。依赖精确版本见 `requirements-lock.txt`。

本轮把机器人模型从 AgileX PiPER 换成了大象机器人 **myCobot 280 M5**（官方 URDF，臂展约 280 mm），并按新臂展重排了工位：托盘 200 × 150 mm、Food ROI 180 × 120 × 60 mm、相机高度 220/280/340 mm、XY 采样 X=[110,250]、Y=[-70,70] mm。

## 默认示例

| 拍摄高度（相对托盘） | 有效视点 | 尝试次数 | 原图烤盘占比范围 | 裁剪后烤盘占比范围 |
|---|---:|---:|---:|---:|
| 220 mm | 4/4 | 24 | 26.1%–28.8% | 50.3%–54.1% |
| 280 mm | 4/4 | 8 | 17.4%–17.8% | 59.4%–62.9% |
| 340 mm | 4/4 | 29 | 11.4%–12.2% | 60.5%–68.2% |

合计 12 个视点，最小实际光轴夹角 6.1144°。整个 Food ROI 的投影、深度范围和整段转移路径均通过复核。相机实际到位误差最大 1.2e-8 m / 1.5e-5°。

默认示例中，每块食材的可见率和可见区域的有效深度率均为 100%。这是几何真值渲染的结果，不代表真实 D405 的实拍深度有效率。

## 拒绝统计（默认示例内部）

| 高度 | 拒绝原因与次数 |
|---|---|
| 220 mm | ik_or_singularity: 10，food_out_of_rgb_fov: 6，self_collision: 2，duplicate_axis: 2 |
| 280 mm | duplicate_axis: 4 |
| 340 mm | duplicate_axis: 23，ik_or_singularity: 2 |

340 mm 时可用姿态明显变少（臂展接近上限、光轴方向趋于一致），因此重复光轴拒绝占多数；系统仍按“先通过先接受”处理，没有放宽任何约束。

## 自动检查

- 15 项 unittest 检查全部通过（`docs/test_results.txt`）。
- 独立读取导出文件，复核 12 个视点的实际位姿、变换闭合、高度、夹角、轨迹连续性、整段碰撞、RGB/Depth/Mask 裁剪一致性（`docs/export_verification.txt`）。
- 专门加入了终点均安全而直线中段碰撞的工况，直连被拒绝，RRT 找到并验证了替代路径。
- 专门测试非相邻机械臂连杆的自碰撞，结果正确拒绝。
- 相邻连杆（URDF 父子关节，如 `g_base`–`joint1`）按 MoveIt 约定自动加入允许矩阵，避免底座固定连接被判成自碰撞。

## 拒绝不合格工况

每个工况限制最多 8 次尝试，目标数量 1，高度取配置中间值（280 mm）；以下全部得到 0 个有效视点，`complete=false`。没有放宽硬约束。

| 工况 | 拒绝原因与次数 |
|---|---|
| too_low | food_out_of_rgb_fov: 8 |
| beyond_depth | depth_range: 8 |
| impossible_tray_ratio | tray_ratio: 8 |
| opaque_roof | occlusion: 6, self_collision: 2 |
| unreachable | ik_or_singularity: 8 |

`opaque_roof` 的挡板位置与尺寸现在由配置的 ROI 推导，换工位后仍覆盖整个 ROI。

## 相同 XY 的高度变化对比

此表只比较几何朝向；这些对照位姿没有通过完整的机器人路径筛选。

| 高度 | 方法 | 目标中心偏离主点 | 几何约束 |
|---|---|---:|---|
| 220 mm | translate | 0.00 px | 通过 |
| 220 mm | relook | 0.00 px | 通过 |
| 280 mm | translate | 27.26 px | 通过 |
| 280 mm | relook | 0.00 px | 通过 |
| 340 mm | translate | 44.85 px | 通过 |
| 340 mm | relook | 0.00 px | 通过 |

## 尚需本地 / 实物验证

PyBullet 图形窗口、Windows 启动脚本和真机控制尚未在本次环境验证。真实相机畸变、深度噪声、表面反光、热环境、线缆、控制器到位、真实支架与烤箱尺寸需要补充。官方网格转 STL 后不带贴图，仅影响可视化外观，不影响几何或碰撞。
