# XY 采样 · 独立可运行的最小示例

这个目录把"在有效区域内做 XY 随机采样、按设定高度生成拍摄视点"这一步**单独实现**，
目标有两个：

1. 让你**直观看到采样过程**（点位怎么落下、哪些被拒、视锥长什么样）；
2. 让你能**一口气读完代码**，看清这一步到底做了什么算术。

它只依赖 numpy 和 Pillow，**不加载 MuJoCo、不需要 GPU、不需要显示器**，秒级跑完。

```
examples/xy_sampling/
  README.md      ← 你正在读的这份
  sample_xy.py   ← 全部实现，约 500 行，每段都有注释
```

---

## 一、怎么跑

```bash
# 默认：3 个高度 × 每高度 4 个视点，输出到 outputs/my_xy_sampling/
.venv/bin/python examples/xy_sampling/sample_xy.py

# 换种子（换一批点位）
.venv/bin/python examples/xy_sampling/sample_xy.py --seed 1

# 只要图片不要 GIF
.venv/bin/python examples/xy_sampling/sample_xy.py --no-gif

# 自证：用同一种子重算，与真实运行的每次尝试逐点比对（前缀检查，见下文）
.venv/bin/python examples/xy_sampling/sample_xy.py --verify outputs/demo
```

`--output` 默认是 `outputs/my_xy_sampling`，正好命中 `.gitignore` 里的
`outputs/my_*/`，不会把产物提交进仓库。重复运行会覆盖同名文件。

---

## 二、会得到哪些图

| 文件 | 看什么 |
|---|---|
| `topview.png` | **俯视图**。虚线框 = XY 采样矩形，绿框 = Food ROI 足迹，黄框 = 托盘，灰点 = 被拒候选，绿点 = 接受视点（附一条指向 ROI 中心的短箭头）。 |
| `sampling.gif` | **逐次接受的动画**。每接受一个视点就推进一帧，左上角显示 `accepted n/12`，最后一帧是全貌。 |
| `frustum_hXX_cNNNN.png` | **单个视点的三维示意**。能看到相机点、四条射向 ROI 角点的视线、绿色光轴、托盘与 ROI 线框。 |
| `fov_hXX_cNNNN.png` | **相机画面**。640×360 画布上画出 ROI 的**投影轮廓**（绿）、托盘投影（黄）、10% 安全边距（灰框）——一眼看出"为什么这个点能过判据"。 |
| `candidates.csv` | 每次尝试一行：`id, height, eye_x/y/z, accepted, reason, tray_ratio`。 |
| `summary.txt` | 逐高度的接受率与拒绝原因计数，口径与 `run` 写出的 `summary.json` 一致。 |

---

## 三、代码怎么读

`sample_xy.py` 按数据流从上到下分成五段，每段开头有分隔线和一句说明。

### ① `sample_eye()` —— 两次均匀抽取

```python
jitter = jitter_rng.uniform(-Δ, Δ) if Δ else 0.0   # 先抽，且 Δ=0 时不抽
x = rng.uniform(*x_B_m)                            # 再抽 X
y = rng.uniform(*y_B_m)                            # 最后抽 Y
z = tray_z + h + jitter
```

两处顺序是**刻意的**，不是随手写的：

- **jitter 先抽、且 Δ=0 时一次都不抽**。这样把 Z 扰动关掉之后，历史种子仍然逐位复现；
- **X 在 Y 之前**。改一个顺序就换了一批点。

生产代码在 `viewpoint_sim/planner.py` 的 `run()` 里做同样两次抽取，用的是
`SeedSequence(seed).spawn(4)` 的第 0 和第 3 路随机流。本示例照搬这个 spawn 顺序，
所以两边的前若干次尝试逐位相同（具体能对多少，见下文 `--verify` 的说明）。

### ② `pose_from_eye()` —— 朝向

```python
center = (food_min_B + food_max_B) / 2
T_BC   = look_at(eye, center, reference_up_B)
```

`look_at` 在 `viewpoint_sim/geometry.py`，构造光学系 `+X 右 / +Y 下 / +Z 前`。
它**不检查食材是否入镜**——那是下一段的事。两者分开，是这份代码最重要的结构决定：
"朝向生成"与"成像判据"互不因果。

### ③ `check_geometry()` —— 六道门，顺序固定

| # | 判据 | 拒绝原因 | 直觉 |
|---:|---|---|---|
| 1 | 8 个 ROI 角点全部落在 RGB 画面内（含 10% 边距） | `food_out_of_rgb_fov` | 拍不全 |
| 2 | 同上，对深度相机（`T_CD`，本例为单位阵） | `food_out_of_depth_fov` | 深度拍不全 |
| 3 | 全部角点的光学 Z 落在 `[0.07, 0.5]` m | `depth_range` | 太近或太远 |
| 4 | 光轴与竖直向下夹角 ≤ 40° | `tilt` | 太斜 |
| 5 | 托盘四角都在相机前方（`z > 0`） | `tray_behind_camera` | 托盘在背后 |
| 6 | 托盘投影占画面比例 ≥ 该高度阈值 | `tray_ratio` | 托盘太小 |

第 6 道之后 `check_geometry` 还会返回一组度量（`tilt_deg`、`roi_depth_*`、
`tray_ratio_raw`），CSV 与图中显示的就是这些值。

顺序**不是随便排的**：越便宜的判据越靠前，一次投影就能拒掉的，不会走到后面的
多边形裁剪。

### ④ 主循环 —— 去重放在最后

生产代码把 `duplicate_axis` 去重放在 IK **之前**，因为一次点积比一次 IK 便宜得多。
在本示例里，前面的判据本来就便宜，所以去重放在最后，语义完全一致：

```python
compare = all_axes if deduplicate_across_heights else axes
if any(angle_deg(T_BC[:3, 2], d) < min_axis_angle_deg for d in compare):
    reason = "duplicate_axis"
```

### ⑤ 绘制 —— 纯 Pillow

- `draw_topview()`：把 Base 系的 XY 映射成像素，`+X` 朝上；
- `draw_frustum_3d()`：**复用 `geometry.project()`**，先用 `look_at` 造一条虚拟观察机位，
  再把三维点投影上去，等于借用了整套针孔相机数学；
- `draw_fov()`：把 ROI 与托盘的投影顶点直接画在 640×360 上。

  这里有个容易踩的细节：**ROI 的投影轮廓要取凸包，不能按角点下标顺序连**。
  `box_vertices()` 返回的 8 个角点是 `itertools.product` 的顺序（二进制计数），
  相邻两个下标在空间上常常是"对角"，按这个顺序连起来会画成一个自交的星形。
  正确画法是先做凸包（`_convex_hull()`，Andrew monotone chain），再连线。

  为什么要取凸包还有个几何原因：**盒子的投影轮廓一般不是四边形**。盒子有 12 条棱，
  斜视时可见 6 条，轮廓是**六边形**；只有接近正对某个面时才会退化成四边形。
  本工程 12 个视点里两种情况都有：`h00_c0003`、`h01_c0002`、`h01_c0003`、
  `h02_c0001`~`h02_c0003` 是 4 边形，其余是 6 边形。

没有任何一个绘图函数参与判定，画图失败不会影响采样结果。

---

## 四、它和 `run` 的关系

| 阶段 | 本示例 | `viewpoint_sim run` |
|---|---|---|
| XY / Z 采样 | ✅ | ✅ |
| Look-at 朝向 | ✅ | ✅ |
| 六道成像判据 | ✅ | ✅ |
| 光轴去重 | ✅ | ✅ |
| IK / 关节限位 | ❌ | ✅ |
| 碰撞 / 路径规划 | ❌ | ✅ |
| 位置伺服执行 | ❌ | ✅ |
| RGB-D 渲染与质检 | ❌ | ✅ |

也就是说：**本示例产出的是"几何上成立的候选"，不是"机器人能拍到的视点"。**
后四步必须靠 MuJoCo，也正是本示例刻意排除的部分——把采样看清，不该被 IK 噪声干扰。

`--verify` 是**前缀比对**，不是全等比对，这是共用随机流的必然结果：

```bash
.venv/bin/python examples/xy_sampling/sample_xy.py --verify outputs/demo
# 5 leading attempt(s) identical out of 13 sampled / 68 attempted
#   diverges at h01_c0001: ...
```

原因：生产代码整轮共用一个 `sample_rng`，每次尝试抽 2 个数。只要某一层因为 IK/碰撞/
质检多试了几次，后面所有层的抽样序列就整体后移。本示例只跑几何门，接受计数一旦与生产
不同，后续自然对不上。

比对对象是 `candidates.jsonl`（每次尝试一行，含被拒的），因为只有它记录了全量尝试；
`views.json` 只有被接受的，无法逐次对齐。

一次运行里如果没有任何 IK/碰撞/质检拒绝，前缀会覆盖全部尝试——那就是全等。
另外，`--verify` 只在「配置与结果目录出自同一次运行」时有意义。

---

## 五、想改什么

| 想做的 | 改哪里 |
|---|---|
| 换采样区域 | `configs/default.json` → `sampling.x_B_m` / `y_B_m` |
| 换高度层 | 同上 `heights_above_tray_m`（同时要改 `tray_min_ratio_by_height` 长度） |
| 加 Z 扰动 | 同上 `z_jitter_m`（demo 会自动多抽一路随机数，并显示高度抖动） |
| 改每高度视点数 | 同上 `target_per_height` |
| 放宽/收紧成像判据 | `camera.image_margin_fraction`（边距）、`sampling.max_tilt_deg`（倾角）、`camera.depth_range_m`（量程） |
| 改 Food ROI | `roi.food_min_B` / `food_max_B`（同时要保证食材在里面，否则 `load_config` 直接报错） |
| 改图幅/配色 | `sample_xy.py` 顶部的颜色常量与 `draw_*` 函数 |

改完重跑即可，无需重新安装任何东西。

---

## 六、一个容易踩的坑

`sample_xy.py` 在文件顶部往 `sys.path` 塞了仓库根目录：

```python
_ROOT = Path(__file__).resolve().parents[2]
```

因为本目录在包外，`python examples/.../sample_xy.py` 默认找不到 `viewpoint_sim`。
这样处理后**从任意目录运行都可以**，也不要求 `pip install -e .`。
