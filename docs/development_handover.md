# HelloBalls 开发交接文档

## 1. 当前状态

完成了从“上位机读取 MCU 串口”到“ROS 2 传感器链路和球场定位”的基本闭环：

1. 建立了 MCU 与上位机之间的双向串口通信，接收 IMU、状态和两轮转速，发送运动控制指令。
2. 将相机、IMU、轮速发布为 ROS 2 topic，并完成相机内参、IMU 噪声、机械外参和录包工具的基础设施。
3. 移植并接入了 ROS 2 Humble 版本的 VINS-Fusion，能够输出 `/vio/odometry`。
4. 开发了基于标准网球场线地图的白线定位，并逐步加入 IMU 航向约束和差速轮速预测。
5. 增加了 `/localization/odometry`、轨迹、小车模型和球场地图的 RViz 可视化。

需要特别说明：**VINS 在实际网球场测试中表现欠佳，初始化要求高，不作为当前推荐的主定位方案。**
当前默认方案是平面球场定位：

```text
相机图像 --------------------> 地面光流/白线帧间配准 ----+
                                                       |
IMU z 轴角速度 -> 零偏校准 -> 图像帧间航向增量 ----------+-> /court/visual_odometry
                                                       |             |
左右轮 RPM -> 差速轮速里程计 -> 帧间运动先验/视觉回退 -----+             |
                                                                     v
相机图像 -> 白线检测 -> 地面投影 -> 标准球场地图匹配 -> /court/pose_measurement
                                                                     |
                                                                     v
                                                      court_vio_fusion_node
                                                                     |
                                                                     v
                                                        /localization/odometry
```

`court_vio_fusion_node` 名称沿用了最初的 VINS 融合设计，但当前它默认接收的是
`/court/visual_odometry`，不要求 VINS 运行。

## 2. Serial 串口协议

### 2.1 UART 参数

当前 `SerialReceiver` 使用：

| 参数     | 值             |
| -------- | -------------- |
| 默认设备 | `/dev/ttyS1` |
| 波特率   | 115200         |
| 数据位   | 8              |
| 校验     | 奇校验（Odd）  |
| 停止位   | 1              |
| 读取超时 | 0.05 s         |
| 单次读取 | 64 byte        |

串口是双向复用的：

- MCU → 上位机：二进制状态/IMU/轮速帧；
- 上位机 → MCU：以换行结束的 ASCII 控制指令。

### 2.2 MCU → 上位机：公共帧格式

所有多字节整数使用大端序，有符号整数使用二进制补码。

```text
AA 55 | message_type | payload_length | payload | checksum
```

校验方式：

```text
checksum = sum(message_type 到 payload 最后一个字节) & 0xFF
```

帧头和校验字节本身不参与求和。流式解析器会：

- 在任意分包、粘包中搜索 `AA 55`；
- 丢弃不支持的消息类型或错误长度；
- 校验失败时向后滑动一个字节重新同步；
- 同时兼容 v1 和 v2。

### 2.3 IMU v1（仅兼容诊断）

`message_type=0x01`，payload 22 byte，整帧 27 byte：

| payload 偏移 | 类型      | 字段                |
| -----------: | --------- | ------------------- |
|            0 | `uint8` | `mcu_state`       |
|            1 | `uint8` | `host_state`      |
|            2 | `int32` | `wheel1_distance` |
|            6 | `int32` | `wheel2_distance` |
|           10 | `int16` | `acc_x`           |
|           12 | `int16` | `acc_y`           |
|           14 | `int16` | `acc_z`           |
|           16 | `int16` | `gyr_x`           |
|           18 | `int16` | `gyr_y`           |
|           20 | `int16` | `gyr_z`           |

v1 没有采样时间。上位机只能使用 UART 收包时间；当一次串口读取拿到多帧时，会出现多帧时间戳
几乎相同、随后长时间空档的问题。该时间序列不适合 IMU 预积分，因此 ROS 发布器默认丢弃 v1。
`--allow-legacy-imu-timestamps` 只应用于临时联调。

### 2.4 IMU v2（当前要求）

`message_type=0x02`，payload 34 byte，整帧 39 byte：

| 整帧偏移 | 长度 | 类型         | 字段            | 说明                                       |
| -------: | ---: | ------------ | --------------- | ------------------------------------------ |
|        0 |    2 | `uint8[2]` | header          | 固定`AA 55`                              |
|        2 |    1 | `uint8`    | message_type    | 固定`02`                                 |
|        3 |    1 | `uint8`    | payload_length  | 固定`34`，即 `0x22`                    |
|        4 |    1 | `uint8`    | mcu_state       | MCU 状态，具体枚举需与固件同步             |
|        5 |    1 | `uint8`    | host_state      | 上位机状态，具体枚举需与固件同步           |
|        6 |    4 | `int32`    | wheel1_distance | 当前实际含义为左轮电机轴有符号 RPM         |
|       10 |    4 | `int32`    | wheel2_distance | 当前实际含义为右轮电机轴有符号 RPM         |
|       14 |    2 | `int16`    | acc_x           | 原始加速度，8192 LSB/g                     |
|       16 |    2 | `int16`    | acc_y           | 原始加速度，8192 LSB/g                     |
|       18 |    2 | `int16`    | acc_z           | 原始加速度，8192 LSB/g                     |
|       20 |    2 | `int16`    | gyr_x           | 原始角速度，64 LSB/(°/s)                  |
|       22 |    2 | `int16`    | gyr_y           | 原始角速度，64 LSB/(°/s)                  |
|       24 |    2 | `int16`    | gyr_z           | 原始角速度，64 LSB/(°/s)                  |
|       26 |    4 | `uint32`   | sample_sequence | 每次 IMU 采样递增，允许自然回绕            |
|       30 |    8 | `uint64`   | sample_time_us  | MCU 启动后的单调微秒时间，必须取自采样时刻 |
|       38 |    1 | `uint8`    | checksum        | 偏移 2～37 求和后取低 8 位                 |

上位机以第一帧的 `(sample_time_us, received_at)` 建立 MCU 单调时间到主机 Unix
时间的锚点，后续严格按 MCU 采样间隔恢复 ROS 时间戳。重复、乱序帧会被丢弃；检测到 MCU
重启时重新建立锚点，并确保 ROS 时间不倒退。

MCU 端应保证：

- 数据、序号、采样时间来自同一次原子快照；
- `sample_time_us` 取 IMU 实际采样时刻，而不是 UART 发送时刻；
- 即使某帧未发出，采样序号仍继续递增；
- 推荐固定 100 Hz，最低建议 50 Hz；
- MCU 重启后序号和单调时间清零是允许的。

### 2.5 数值换算和坐标轴

原始量换算：

```text
acc_g      = acc_raw / 8192
gyro_deg_s = gyro_raw / 64
```

发布 `/imu/data_raw` 前，原始 IMU 坐标会映射到车体坐标：

```text
(x, y, z)_base = (raw_x, -raw_z, raw_y)
```

随后：

```text
linear_acceleration = acc_g × 9.80665 × imu_accel_scale
angular_velocity    = gyro_deg_s × π / 180
```

当前默认 `imu_accel_scale=0.907942`，用于把实测静止模长约 `10.80097 m/s²`
缩放到标准重力。（疑似为imu精度不高导致的误差）它只是单一比例修正，不能替代六面加速度计标定。

### 2.6 轮速和 `/wheel/odom`

旧字段名 `wheel1_distance`、`wheel2_distance` 为兼容保留，当前实际传输左右电机轴 RPM。
只有同时设置 `--wheel-radius-m` 和 `--wheel-track-m` 后才发布 `/wheel/odom`。

```text
gear_ratio = motor_rpm / wheel_rpm
v_left  = sign_left  × rpm_left  / gear_ratio × 2πr / 60
v_right = sign_right × rpm_right / gear_ratio × 2πr / 60
v       = (v_left + v_right) / 2
w       = (v_right - v_left) / wheel_track
```

默认减速比为M3508电机标准`3591/187 ≈ 19.2032`。积分采用帧间中点航向；采样间隔超过 0.25 s
时不跨空档积分。轮半径、轮距和正负号必须以实车测量为准，文档中的数值不能直接当作标定值。

### 2.7 上位机 → MCU：控制指令

完整控制指令为：

```text
state,motor_speed_1,motor_speed_2,tilt_angle,friction_wheel_speed\n
```

例如键盘前进：

```text
4,100,100,0,0\n
```

另保留三字段兼容接口：

```text
state,wheel1_speed,wheel2_speed\n
```

当前上位机代码只明确了 `state=4` 为手动/扫描控制状态，其他状态枚举应以 MCU 固件为准。
键盘控制映射为：

| 按键  |     左电机 |     右电机 |
| ----- | ---------: | ---------: |
| `w` | `+speed` | `+speed` |
| `s` | `-speed` | `-speed` |
| `a` | `-speed` | `+speed` |
| `d` | `+speed` | `-speed` |
| 空格  |          0 |          0 |

命令默认以 50 Hz 重发；运动键超过 0.15 s 未续按会自动发零速，程序退出时也会尝试发送完整五字段停止命令。

## 3. VINS-Fusion 接入与结论

### 3.1 已完成的工程工作

VINS-Fusion 通过子模块接入：

```text
ros2_ws/src/third_party/vins_fusion_ros2
https://github.com/BelkanMag/HelloBalls-VINS-Fusion.git
branch: ros2-humble
```

当前 ROS 2 构建目标包含 `camera_models` 和 `vins`；回环和全局融合部分仍是 ROS 1/catkin，
不在当前第一阶段部署范围内。

围绕 VINS 完成了：

- C++ 相机发布器，替代实时性不足的 Python 相机节点；
- 后端 `mono8` 转换节点，避免相机直接灰度采集的性能问题；
- `/camera/camera_info`、相机内参、IMU 噪声和相机—IMU 外参配置；
- 静态 TF：`base_link`、`camera_link`、`camera_optical_frame`、`imu_link`；
- IMU v2 硬件采样时间戳；
- ROS 2 launch、进程拉起/退出、异常自动重启；
- rosbag2 采集和 VIO RViz 调试视图。

主要输入输出：

```text
/camera/image_mono 或 /camera/image_raw -> VINS
/imu/data_raw                           -> VINS
VINS -> /vio/odometry
        /vio/path
        /vio/image_track
        /vio/point_cloud
        ...
```

配置文件为：

- `ros2_ws/src/helloballs_bringup/config/vins_mono_imu.yaml`
- `ros2_ws/src/helloballs_bringup/config/helloballs_camera_pinhole.yaml`
- `ros2_ws/src/helloballs_bringup/config/tf_static.example.yaml`

### 3.2 构建和运行

```bash
git submodule update --init --recursive ros2_ws/src/third_party/vins_fusion_ros2

cd ros2_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --symlink-install --packages-select camera_models vins helloballs_bringup
source install/setup.bash
```

一体化启动：

```bash
cd <仓库根目录>
python3 main.py --ros-localization
```

或单独启动：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch helloballs_bringup vins.launch.py use_rviz:=true
```

### 3.3 实测评价

VINS 链路在工程上已经跑通，但在实际网球场环境中效果欠佳：

- 网球场地面纹理少且重复，除了白线外稳定特征有限；
- 快速运动、抖动和图像模糊会降低特征跟踪质量；
- 单目+IMU 初始化要求较高，需要足够的视觉视差和合适的激励；
- 起步方式、静止时间、运动幅度不合适时，初始化慢或不稳定；
- 相机内参、相机—IMU 外参、时间同步和 IMU 噪声任何一项偏差都会明显影响结果；
- 即使初始化成功，VIO 仍是相对里程计，无法单独解决球场内重复白线造成的全局位置歧义。

因此交接建议是：

- 保留 VINS 作为实验、离线对比和将来更换传感器后的备选方案；
- 当前实车定位优先维护“白线 + IMU + 轮速”平面方案；
- 不要让 VINS 的启动或初始化成为整车可运行的前置条件。

## 4. 当前白线 + IMU + 轮速定位

### 4.1 节点职责

`court_line_localizer_node` 同时承担两类任务：

1. 局部连续运动估计，发布 `/court/visual_odometry`；
2. 白线与标准球场地图匹配，发布 `/court/pose_measurement`。

`court_vio_fusion_node`：

1. 缓存并按时间插值局部里程计；
2. 将白线绝对位姿转换为 `court -> world` 校正候选；
3. 做协方差、时间同步、一致性和创新量门控；
4. 平滑更新校正，输出 `/localization/odometry` 和 TF。

### 4.2 输入输出

| Topic                             | 类型                          | 用途                     |
| --------------------------------- | ----------------------------- | ------------------------ |
| `/camera/image_mono`            | `sensor_msgs/Image`         | 默认图像输入             |
| `/camera/camera_info`           | `sensor_msgs/CameraInfo`    | 地面投影内参             |
| `/imu/data_raw`                 | `sensor_msgs/Imu`           | z 轴角速度、帧间航向     |
| `/wheel/odom`                   | `nav_msgs/Odometry`         | 差速轮运动先验           |
| `/court/visual_odometry`        | `nav_msgs/Odometry`         | 局部连续平面里程计       |
| `/court/pose_measurement`       | `PoseWithCovarianceStamped` | `court` 坐标系绝对位姿 |
| `/localization/odometry`        | `nav_msgs/Odometry`         | 最终定位输出             |
| `/court/debug_image`            | `sensor_msgs/Image`         | 白线检测调试图           |
| `/court/map_lines`              | `visualization_msgs/Marker` | 标准球场线               |
| `/localization/path`            | `nav_msgs/Path`             | 轨迹                     |
| `/localization/vehicle_markers` | `MarkerArray`               | 小车位置和朝向           |

坐标系约定：

```text
court -> world -> base_link -> camera_link -> camera_optical_frame
                         \-> imu_link
```

`court` 为标准网球场全局坐标；`world` 为本次启动的局部里程计坐标。融合节点启用时，
它是 `court -> world` 的唯一发布者，不要再启动另一个静态同名 TF。

### 4.3 白线绝对定位

当前为传统视觉识别，未来可以考虑训练语义识别场地白线。

处理流程：

1. 图像转灰度，在下半部分 ROI 内处理；
2. 高斯模糊、自适应阈值、开闭运算；
3. Canny + `HoughLinesP` 提取候选白线；
4. 使用内参、相机高度和俯仰把线段端点投影到地面；
5. 与标准球场线段地图做方向一致的完整线段匹配；
6. 在候选位姿附近分级搜索 `x/y/yaw`，按线到地图线段距离评分；
7. 弱匹配、越界、方向不一致或歧义过大的结果不发布。

标准尺寸默认：

```text
球场长度 23.77 m
双打宽度 10.97 m
单打宽度 8.23 m
发球线距球网 6.40 m
```

启动时左右边线存在对称歧义。当前规定车辆启动时必须放置在网球场的四个角点，且车头沿着底线摆放。故由中心对称只存在边线在车左侧或右侧两种可能性。推荐已知起始位置时显式设置：

```text
start_side:=sideline_left
start_side:=sideline_right
```

若使用 `unknown`，节点会根据图像下部横向白线在左右半幅的长度进行多帧投票，锁定前不会发布
绝对白线位姿。单条直线不能约束沿线方向的位置，重复平行线也仍可能产生局部歧义。

### 4.4 IMU 融合

IMU 默认启用。启动后前 2 s 收集 z 轴角速度并以中位数估计零偏；然后针对每一对相邻图像的
精确时间区间，用梯形积分得到 `dyaw`。

IMU 在当前算法中的作用：

- 固定地面光流估计中的旋转量，使视觉只需稳健求解平移；
- 白线/光流短暂失败时仍连续更新航向；
- 图像帧间隔较大时，只要 IMU 覆盖足够，可跨图像空档维持航向；
- 作为异常门控中心，拒绝不合理的视觉旋转。

默认要求 IMU 相邻样本间隔不超过 0.05 s，图像区间覆盖率至少 70%。若车辆启动后立即运动，
不要自动估零偏，应使用先前静止测得的值：

```bash
ros2 launch helloballs_bringup court_localization.launch.py \
  imu_auto_bias:=false \
  imu_yaw_bias_rad_s:=<实测值>
```

### 4.5 轮速融合

**实际测试中若只有来自相机+imu的数据，则几乎不可能完成里程计功能，会出现估计里程小于实际位移的问题，因此需要引入较高置信度的轮速约束。**`/wheel/odom` 的相邻位姿会转换成车体局部增量，并在两帧图像之间累计。轮速在当前算法中的作用：

- 给图像光流和白线帧间配准提供平移中心；
- 当视觉平移不可观或短时失败时，与 IMU 航向一起作为回退增量；
- 限制视觉搜索范围，减少重复白线下跳到错误解的概率。

最终航向优先采用零偏修正后的 IMU 积分；轮速主要提供平移。视觉可用时仍会用地面光流/白线配准
校正运动量。算法还施加差速车非完整约束，把平移投影到与航向变化一致的前向圆弧，抑制不合理横移。

### 4.6 局部运动估计

局部运动优先使用下部地面 ROI 的稀疏 LK 光流：

- 前后向光流一致性检查；
- 排除地平线附近和最大地面距离以外的特征；
- 有 IMU 时固定旋转，使用中位数和内点二次估计平移；
- 无 IMU 时使用 RANSAC 平面相似变换；
- 光流失败后退到已投影白线之间的迭代配准；
- 仍失败时使用 IMU 航向，以及可用的轮速平移。

相机不在车体旋转中心，代码会使用默认平面杆臂 `(0.23668, 0.0) m`
把相机运动换算回 `base_link`。

### 4.7 绝对/相对融合

融合节点按白线观测时间插值局部里程计，默认：

- 最大时间对齐误差 0.15 s；
- 需要连续 3 个一致观测才初始化或更新；
- 拒绝超过 1.0 m 或 0.35 rad 的校正跳变；
- 以 0.2 增益平滑更新；
- 单次最大平移校正 0.15 m、航向校正 0.05 rad；
- 拒绝过大的白线观测协方差。

## 5. 推荐启动流程

### 5.1 编译

当前白线主方案不要求编译 VINS 子模块：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --symlink-install --packages-select helloballs_bringup
source install/setup.bash
```

### 5.2 启动传感器和轮速

在仓库根目录启动串口、相机和灰度转换。下面的轮半径/轮距必须替换成实测值：

```bash
python3 main.py \
  --ros-sensors \
  --camera-mono-converter \
  --wheel-radius-m <驱动轮半径_m> \
  --wheel-track-m <左右驱动轮接地点间距_m> \
  --wheel-gear-ratio 19.20320856
```

直行时检查两个 RPM 的正负号；若某侧为负，增加 `--wheel1-sign -1` 或 `--wheel2-sign -1`。

### 5.3 启动球场定位

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch helloballs_bringup court_localization.launch.py \
  start_side:=sideline_right \
  use_rviz:=true
```

若起始侧确实未知，再使用默认 `start_side:=unknown`。定位 launch 默认：

```text
use_vio_prediction=false
use_wheel_prediction=true
use_imu_yaw_prediction=true
use_fusion=true
```

### 5.4 rosbag

默认脚本会录制相机、内参、IMU 和 TF。定位联调建议附加轮速和定位 topic：

```bash
EXTRA_TOPICS="/wheel/odom /court/visual_odometry /court/pose_measurement /localization/odometry" \
  ./scripts/record_sensor_bag.sh
```

回放时所有终端统一使用 Fast DDS：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 bag play <bag目录>
```

时间戳回退超过 1 s 时，图像、IMU、融合和轨迹状态会自动重置，因此同一 launch 可以从头重复回放。
