# HelloBalls 项目指引

本仓库 [SHKinsem/HelloBalls-Workspace](https://github.com/SHKinsem/HelloBalls-Workspace) 是 HelloBalls（基于 ESP32 的网球机器人）的总仓库，上位机和下位机作为 git 子模块挂在下面。

## 目录结构

| 目录 | 角色 | GitHub 仓库 | 跟踪分支 | 技术栈 |
|------|------|-------------|----------|--------|
| `host/` | 上位机 | [SHKinsem/HelloBalls-Host](https://github.com/SHKinsem/HelloBalls-Host) | `rewrite/v2` | Python（网球识别、控制）+ ROS 2 C++（定位） |
| `device/` | 下位机 | [SHKinsem/HelloBalls-2.0](https://github.com/SHKinsem/HelloBalls-2.0) | `imu_serial` | ESP-IDF（C/C++、CMake），UI 用 SquareLine Studio 做（LVGL） |

> 旧版 [SHKinsem/HelloBalls](https://github.com/SHKinsem/HelloBalls)（PlatformIO，默认分支 `shooter`）是 1.0 版本，不属于本仓库。

## 拉取代码

```bash
git clone https://github.com/SHKinsem/HelloBalls-Workspace.git
cd HelloBalls-Workspace
git submodule update --init host device     # 不要加 --recursive，见下
git -C host switch rewrite/v2               # 子模块默认停在某个提交上，改代码前先切到分支
git -C device switch imu_serial
```

- **不要用 `--recursive`**：`host` 的 `localization` 历史里引用了 VINS-Fusion 子模块（体积大且已不是主方案）；`tennis_weight` 是一个没有 URL 的坏子模块引用，待清理。
- 拉取子仓库最新代码：`git submodule update --remote host device`。

## 子模块工作流

1. 在子仓库里切到跟踪分支、提交、推送；
2. 回到总仓库 `git add host device` 并提交，更新记录的子模块提交号。**忘了这一步，云端会话拉到的还是旧代码。**

issue 仍然开在两个子仓库里（见"待办清单"）。

## 注意事项

- `host/` 和 `device/` 是独立的 git 仓库，提交、分支、推送要分别在各自目录里操作。
- 本文件夹位于 OneDrive 同步目录下，注意避免把构建产物（如 ESP-IDF 的 `build/`）、依赖目录、模型权重同步上云。
- 本机的 `gh` 在 `C:\Program Files\GitHub CLI\gh.exe`，Git Bash 的 PATH 里没有它。

## 上位机规划（重写中）

- 最终部署在 **RDK X5**（BPU 架构 `bayes-e`），开发阶段要能在本地 Windows 电脑上跑（无 NVIDIA 显卡，用 onnxruntime CPU）。
- 两大模块：① 网球识别与抓取 ② 球场视觉 + 惯性导航定位。先做 ①，且先在本地跑通。
- 架构：输入 / 预处理 / 推理 / 后处理分层，只有推理层按平台切换（PC: onnxruntime；X5: BPU `.bin`）。

### RDK X5 BPU 适配约束（训练和导出模型时就要遵守）

参考 D-Robotics RDK Model Zoo 的 YOLO11 方案（旧仓库 `yolo11_demo/README_cn.md`）：

- **Detect 头改成 6 个输出**：3 个尺度 × (bbox 的 DFL 原始值 64 通道, cls logits)，DFL 解码和 sigmoid 放到 CPU 后处理。PC 端也用同样的 ONNX 输出格式，让后处理两端共用一份。
- **Attention 等价改写**（`ultralytics/nn/modules/block.py`）：不改的话 Softmax 会落在 CPU 上，把模型切成两个 BPU 子图；改写后不用重新训练。
- 导出用 `opset=11`、`simplify=False`；输入在板端是 NV12，归一化由 BPU 完成。
- 编译前先跑 `hb_mapper checker --march bayes-e`，确认所有算子都在 BPU 上。
- int8 PTQ 量化：网球属于小目标，校准集要用真实球场图片，并对比 PC 上 fp32 和板端量化后的精度。

### 网球定位（相机坐标系下的距离、角度）

- 模型只输出 bbox，距离和角度由几何层算出，不让模型直接回归距离。
- 相机目前**未标定**，需要先做内参标定（OpenCV 棋盘格，按实际部署分辨率标定，每个相机单独存一份参数）。
- 角度：去畸变后由像素坐标和内参算方位角/俯仰角，只需要内参。
- 距离两种方法互补：
  - 已知尺寸：网球直径约 6.7 cm，`Z ≈ fx · D / d_px`，近处准，远处像素少误差大。
  - 地平面约束：球在地上，用相机安装高度和俯仰角（外参）做射线与地面求交，远处更稳。

### 上位机代码（`host/`，分支 `rewrite/v2`）

- `helloballs/vision/camera.py`：USB 相机（Windows 用 DirectShow，Linux/X5 用 V4L2，默认 MJPG）、视频、图片输入。
- `helloballs/vision/detector.py`：letterbox + YOLO 输出解码（类别无关 NMS）+ `OnnxDetector`。BPU 后端以后加在同一层。
- `helloballs/vision/geometry.py`：`CameraModel`（内参 yaml）、`Mount`（安装高度/俯仰）、`locate_ball()` 输出相机系下的位置、距离、方位角。
- `helloballs/vision/calibration.py`：棋盘格标定逻辑（CLI 和网页共用）；`overlay.py`：画框和 HUD。
- `helloballs/ui/`：网页控制台（纯标准库 HTTP 服务 + MJPEG 推流 + `static/index.html`），双击 `start_console.bat` 启动；在 X5 上用 `--host 0.0.0.0`。拍照存 `captures/`，标定图存 `calib_imgs/`，内参存 `configs/cameras/`。
- `tools/calibrate_camera.py`、`tools/detect_demo.py`：命令行版本，功能和网页一样。
- 旧代码（`main.py`、`scripts/`、`yolo11_demo/`）暂时保留做对照；串口协议 `scripts/HelloBalls_Serial.py` 之后迁进新结构。
- 相机：USB 相机（本地和 X5 都是）。"抓取"指机器人开过去把球捡起来。
- **车上的 USB 相机是双摄模组（广角 + 长焦），靠切换分辨率来切换摄像头**（`cv_work` 里切分辨率就是为此）。因此：
  - 每个分辨率对应一个物理镜头，内参必须**按分辨率分别标定**，不能从一个分辨率缩放到另一个。`CameraModel.scaled_to()` 和控制台"按宽高比自动缩放内参"的逻辑对这台相机是错的，待改（见上位机 issue #9）。
  - 两个镜头的外参（安装位置、朝向）也可能不同。
  - 实测（2026-10-24，Windows 上设备名 `USB Camera`，VID 15AA PID 1555；`tools/probe_camera_modes.py` 可复现，图在 `captures/camera_modes/`）：
    - **长焦**：1280×720、800×600、640×480、640×360、320×240（放大约 4～5 倍；对着 2～3 m 外的墙拍很模糊，焦点待查）
    - **广角**：1280×712、800×592、640×472、640×352、320×232（比长焦那组高度少 8 像素）。队友用的 800×592 和那份 1280×712 标定**都是广角**
    - **双路并排**：2560×720、1600×600、1280×480、1280×360、640×240，**左半长焦、右半广角，同一帧同时输出**，2560×720 实测约 30 fps
    - 所有分辨率都支持 MJPG 和 YUY2

## 本地开发环境（Windows）

- Python 虚拟环境在 `C:\Users\xusen\.venvs\helloballs-host`（Python 3.10，与 RDK X5 的 Ubuntu 22.04 一致；放在 OneDrive 外面避免同步）。
- 跑测试：在 `host/` 下执行 `C:\Users\xusen\.venvs\helloballs-host\Scripts\python.exe -m pytest -q`。
- **pip 需要显式代理**：系统代理是 `127.0.0.1:7897`，Python 3.10 会把它当成 `https://` 代理导致 SSL 错误，安装前先设置 `$env:HTTPS_PROXY='http://127.0.0.1:7897'`（`HTTP_PROXY` 同理）。
- Git Bash 里没有 `git` 和 `gh`，git 操作用 PowerShell。

## 队友已有工作：`HelloBalls-Host` 的 `localization` 分支（BelkanMag，截至 2026-08-01）

完整交接文档：[`docs/development_handover.md`](docs/development_handover.md)（串口协议、ROS 话题、定位算法细节、启动流程）。下面是要点。

- **ROS 2 Humble** 定位栈（`ros2_ws/src/helloballs_bringup`，C++）：`camera_publisher_node`（独占 USB 相机，发布 `/camera/image_raw`、`/camera/image_mono`）、`court_line_localizer_node`（白线检测 + 球场地图匹配 + 地面光流，约 2000 行）、`court_vio_fusion_node`（融合），最终输出 `/localization/odometry`。
- **结论：VINS-Fusion 在真实球场效果差，不作主方案**；主方案是"白线 + IMU 航向 + 差速轮速"平面定位。启动时车要放在球场角点、车头沿底线（`start_side`）。
- 机械外参：`base_link`（x 前 y 左 z 上）到 `camera_link` 平移 `(0.23668, 0, 0.14214)` m，旋转暂为单位阵（待核实）。相机高度约 0.142 m、俯仰 0。
- 相机内参：`helloballs_camera_pinhole.yaml` 有一份 1280×712 的标定（fx/fy 差 2.5%、主点偏离中心较多，可信度存疑）；`sensors.yaml` 实际采集用 800×592 @15fps，对应的 `camera_info.example.yaml` 全是 0，即**实际运行分辨率下没有有效内参**。
- 用户自己 2025 年的 `cv_work` 分支有自动追球 / PID / 手势和人体居中等实验代码。

## 上下位机串口协议（以交接文档为准）

- UART：X5 上 `/dev/ttyS1`，115200，8 位数据，**奇校验**，1 停止位。
- MCU → 上位机：`AA 55 | type | len | payload | checksum`（checksum = type 到 payload 末字节求和 & 0xFF），大端序。
  - **IMU v2（当前要求）**：type `0x02`，payload 34 B，整帧 39 B；含左右电机轴 **RPM**（字段名仍叫 `wheel1_distance`/`wheel2_distance`）、加速度 8192 LSB/g、角速度 64 LSB/(°/s)、`sample_sequence`、`sample_time_us`（MCU 采样时刻）。
  - v1（type `0x01`，27 B，无时间戳）只用于诊断。`main` 分支的 `scripts/HelloBalls_Serial.py` 只支持 v1。
  - IMU 轴映射到车体：`(x, y, z)_base = (raw_x, -raw_z, raw_y)`；加速度另乘 `0.907942` 的比例修正。
- 上位机 → MCU：ASCII，`state,motor_speed_1,motor_speed_2,tilt_angle,friction_wheel_speed\n`（兼容三字段 `state,w1,w2\n`），50 Hz 重发；`state=4` 为手动/扫描，其他状态以固件为准。
- 减速比默认 M3508 的 `3591/187`；轮半径、轮距、正负号需实车测量。

## 下位机固件要点（`device/`，以 `imu_serial` 分支为准；`main` 停在 2025-11，没有 IMU v2）

- ESP32-S3 + FreeRTOS，电机走 CAN：M3508 ×2 摩擦轮（ID 1/2）、M3508 ×2 驱动轮（ID 3/4，2 号反装，固件里已处理符号）、M2006 装填（ID 5），另有两个舵机（倾角）和一个步进电机。
- **上位机发的轮速单位是电机轴 RPM（int16）**，直接作为 M3508 速度环目标，固件做速度闭环（串级 PID 的内环）。
- `host_state`（指令第一个字段）：`0 IDLE`、`1 SEARCHING_BALL`、`2 BALL_REACHED`、`3 SHOOTING`、`4 SCANNING`、`5 ERROR`。
  - `1`：摩擦轮以 -800 反转吸球，舵机下压；轮速跟随上位机。
  - `2`：固件执行约 5 s 的阻塞式收球动作（摩擦轮、舵机抬到 30°、装填电机），完成后 `mcu_state = 2 (MCU_BALL_RETRIEVED)`；期间轮速仍跟随上位机，应发 0。
  - `3`：发射，用 `tilt_angle` 和 `friction_wheel_speed`。`4`：扫描，舵机角限幅 -5～35°，轮速跟随上位机。
  - 状态动作**只在状态变化时触发一次**。
- `mcu_state`（v2 上报帧第一个字节）：`0 IDLE`、`1 SEARCHING_BALL`、`2 BALL_RETRIEVED`、`3 SHOOTING`、`4 SHOOTING_COMPLETE`、`5 ERROR`。
- 串口看门狗：500 ms 收不到指令就关掉所有电机，上位机必须持续发送（当前 50 Hz）。

## 待办清单

待办以 GitHub issue 管理（2026-10-24 建立），新想法也记成 issue：
- 上位机：https://github.com/SHKinsem/HelloBalls-Host/issues （label：`vision`、`control`、`rdk-x5`、`serial`）
- 下位机：https://github.com/SHKinsem/HelloBalls-2.0/issues （label：`pickup`、`serial`）

队友（BelkanMag / Dong Tianyang）已不再维护代码，`localization` 和 `imu_serial` 两个分支现在由我们接手，可以直接修改。

## 待补充

- 下位机的构建和烧录方式
