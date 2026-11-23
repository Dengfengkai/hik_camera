# hik_camera

海康机器人（HIKROBOT）工业相机的 ROS 2 驱动，基于 MVS SDK 的 Python 封装，支持**硬件触发**与**时间基准自动对齐**。

面向 Fast-LIVO2 等激光-惯性-视觉紧耦合系统：相机的图像时间戳会主动对齐到雷达时间基准，而不是简单取本机墙钟。

## 特性

- 发布 `sensor_msgs/Image` 与 `sensor_msgs/CameraInfo`
- 支持自由运行与外部硬件触发两种模式
- **时间基准自动校准**：开机自动测量「墙钟 − 雷达时钟」差值，无需手工填偏移
- **绝不发出错误时间戳**：参考时钟未就绪时暂停发图，而不是用墙钟凑一个
- 设备 tick 异常（相机重上电、计数器归零）自动重建锚点
- 运行期持续抽查相机与参考时钟的偏差

## 环境要求

| 项目 | 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 相机 SDK | 海康 MVS SDK（默认安装到 `/opt/MVS`） |
| 相机型号 | 海康 USB3 / GigE 工业相机（已测 MV-CU013-A0UC） |

## 安装

### 1. 安装 MVS SDK

从海康机器人官网下载 MVS 安装包，安装到默认路径 `/opt/MVS`。

安装后确认这两个目录存在：

```bash
ls /opt/MVS/lib
ls /opt/MVS/Samples/64/Python/MvImport
```

驱动启动时会自动设置 `MVCAM_COMMON_RUNENV` 并把 `MvImport` 加入 `sys.path`，无需手动配置。

### 2. 编译

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/Dengfengkai/hik_camera.git
cd ~/ws

source /opt/ros/humble/setup.bash
colcon build --packages-select hik_camera
source install/setup.bash
```

> 仓库名为 `hik_camera`，ROS 包名同为 `hik_camera`。

## 运行

### 自由运行（无外触发）

```bash
ros2 launch hik_camera hik_camera.launch.py
```

或在启动前把 `config/hik_camera.yaml` 中的 `trigger_mode` 改为 `0`。

### 外部硬件触发（与雷达同步）

保持 `config/hik_camera.yaml` 中：

```yaml
trigger_mode: 1
trigger_source: "Line0"
trigger_polarity: "RisingEdge"
```

此时相机帧率由外部触发脉冲决定。

```bash
ros2 launch hik_camera hik_camera.launch.py
```

### 单独调试相机（不依赖雷达）

时间基准自动对齐需要参考时钟（默认 `/livox/imu`）。若只想单独看相机图像：

```bash
ros2 run hik_camera hik_camera_node --ros-args \
  -p time_offset_auto:=false \
  -p trigger_mode:=0
```

### 验证

```bash
ros2 topic hz /left_camera/image
ros2 run rqt_image_view rqt_image_view
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/left_camera/image` | `sensor_msgs/Image` | 图像，编码 `bgr8` |
| `/left_camera/camera_info` | `sensor_msgs/CameraInfo` | 内参与畸变，供 FAST-LIVO2 使用 |

话题名可通过 `image_topic` / `camera_info_topic` 参数修改。

## 时间同步原理

### 问题

同步器只给相机发**触发脉冲**，不发时间码。相机自身没有绝对时间，驱动只能用「首帧的墙钟 + 设备 tick」构造时间戳。

于是「相机时间戳」与「雷达时间戳」之间的差值，取决于同步器这次上电后累计了多久——**这个差每次开机都不同，不能写死。**

### 做法

1. 启动时订阅参考话题（默认 `/livox/imu`），采集 N 帧
2. 对每帧计算 `墙钟 − 消息时间戳`，取**中位数**作为基准差 `delta`
3. 之后所有图像时间戳 = `设备tick换算时间 − delta`

```
相机时间戳 = 设备tick × 10ns + 首帧锚点 − (墙钟 − 雷达时钟)
                                    ↑
                              自动测得的基准差
```

### 为什么不用均值

网络抖动、调度延迟会带来偶发的大偏差，均值会被拉偏。中位数对这种异常值不敏感。

### 参考时钟未就绪时

**驱动会暂停发图，而不是用墙钟凑一个时间戳。**

这是刻意的设计。早年用墙钟当初基准，会发出一批差几亿秒的时间戳，FAST-LIVO2 静默不建图，排查极为困难。现在宁可一帧不发，并在日志里持续报错。

超过 `sync_ref_timeout` 后，日志会每 10 秒提示一次原因。

## 参数说明

全部参数在 `config/hik_camera.yaml` 中，也可命令行用 `-p 参数名:=值` 覆盖。

### 相机基础

| 参数 | 默认 | 说明 |
|---|---|---|
| `serial` | `""` | 相机序列号。空 = 自动选第一台；多相机时必须填 |
| `exposure_us` | `10000.0` | 曝光时间（微秒） |
| `gain_db` | `0.0` | 增益（dB），`<=0` 不设置 |
| `fps` | `15.0` | 目标帧率，`<=0` 不设置。**外部触发模式下该参数不生效** |
| `width` / `height` | `0` / `0` | 分辨率。`0` = 相机原始尺寸。内参标定后不要改 |
| `frame_id` | `camera_optical_frame` | 图像坐标系名称 |

### 触发设置

| 参数 | 默认 | 说明 |
|---|---|---|
| `trigger_mode` | `0` | `0` = 连续采集（自由运行）；`1` = 外部触发 |
| `trigger_source` | `Line0` | 触发信号源，`Line0` / `Line1` |
| `trigger_polarity` | `RisingEdge` | 触发极性，`RisingEdge` / `FallingEdge` |

### 时间基准对齐

| 参数 | 默认 | 说明 |
|---|---|---|
| `time_offset_auto` | `true` | 开机自动测量基准差。**推荐保持 true** |
| `sync_ref_topic` | `/livox/imu` | 参考时钟话题，须与雷达同一硬件基准 |
| `sync_ref_type` | `auto` | 参考消息类型：`auto` / `imu` / `custom` / `pointcloud2` |
| `sync_ref_samples` | `30` | 采集多少帧求中位数 |
| `sync_ref_timeout` | `20.0` | 等待参考话题的秒数，超时后持续报错并暂停发图 |
| `sync_warn_s` | `0.2` | 运行中偏差超过此值告警（秒） |
| `time_offset` | `0.0` | 手动微调偏移（秒）。自动对齐后的额外 trim，一般保持 0 |

### 内参与发布

| 参数 | 默认 | 说明 |
|---|---|---|
| `fx` / `fy` | `0.0` | 焦距（像素），标定后填写 |
| `cx` / `cy` | `0.0` | 主点坐标 |
| `distortion` | `[0,0,0,0,0]` | 畸变系数，顺序 `k1 k2 p1 p2 k3` |
| `image_topic` | `/left_camera/image` | 图像话题名 |
| `camera_info_topic` | `/left_camera/camera_info` | 内参话题名 |
| `qos` | `sensor_data` | `sensor_data` = 尽力传输（标定/RViz）；`reliable` = 可靠传输（FAST-LIVO2 需要） |

### 建议配置

**标定相机内参时：**

```yaml
trigger_mode: 0
qos: sensor_data
time_offset_auto: false
exposure_us: 30000.0
```

**与雷达联合建图时：**

```yaml
trigger_mode: 1
qos: reliable
time_offset_auto: true
sync_ref_topic: /livox/imu
```

## 日志说明

正常启动会依次看到：

```
[INFO] [hik_camera]: 外部触发: source=Line0 polarity=RisingEdge ret=0x0/0x0/0x0
[INFO] [hik_camera]: 时间基准自校准：等待 /livox/imu 的 30 帧…
[INFO] [hik_camera]: 时间基准已校定: 墙钟-雷达时钟 = xxx.xxxxxxs (抖动 ±x.xxxms, n=30)
[INFO] [hik_camera]: 建立设备时间戳锚点 dev_ref=xxxxx
[INFO] [hik_camera]: 帧率 10.0 Hz | 取帧 96.4ms | 转换 1.0ms | 发布 2.6ms
```

运行中每 2 秒打印一次帧率与各阶段耗时。

## 常见问题

### 启动报「没有找到相机」

- 检查 USB3 线是否插好
- 检查 MVS 客户端是否占用相机（同时只能有一个程序访问）
- `lsusb | grep -i hik` 确认设备被识别

### 报「已等 20s 仍没收到 /livox/imu，暂停发布图像帧」

参考时钟话题没有数据。原因通常是雷达驱动没启动，或话题名不匹配。

**处理：**

```bash
# 确认雷达在发数据
ros2 topic hz /livox/imu

# 若雷达话题名不同，改参数
ros2 run hik_camera hik_camera_node --ros-args -p sync_ref_topic:=/你的话题
```

若只是单独测相机，加 `-p time_offset_auto:=false`。

### 报「相机与 /livox/imu 时间基准相差 +xxxms」

运行期抽查发现偏差超过 `sync_warn_s`。

**两种可能：**

1. **真实偏差** —— 同步器接线或配置有问题，检查 PPS / GPRMC / 触发线
2. **停机误报** —— 雷达已停、相机还在出图时，参考时间戳不再更新，差值会一直增大。这是停机阶段的正常现象，可以忽略

### 有图像但 FAST-LIVO2 不建图

按顺序排查：

1. `qos` 是否为 `reliable`（FAST-LIVO2 订阅端要求可靠传输）
2. 图像时间戳是否与雷达同量级 —— 对比：
   ```bash
   ros2 topic echo /left_camera/image --field header.stamp --once
   ros2 topic echo /livox/imu --field header.stamp --once
   ```
3. 内参 `fx/fy/cx/cy` 是否填写

### 外部触发模式下没有图像

- 确认触发信号已接入 `Line0`
- 用示波器或万用表确认有脉冲
- 临时改为 `-p trigger_mode:=0` 排除相机本身问题

## 已知限制

- `fps` 参数在外触发模式下无效（帧率由触发信号决定），日志仍会打印设置结果，属于正常现象
- 首帧时间戳使用「收到该帧的墙钟」，而非曝光时刻，存在约半个帧周期的偏差；后续帧通过设备 tick 累加，不再有该偏差
- 依赖 MVS SDK 的 Python 封装，路径固定在 `/opt/MVS`，自定义安装位置需修改代码中的 `_MVS_PY`

## 许可证

MIT，见 [LICENSE](LICENSE)。
