# HelloBalls Workspace

HelloBalls 网球机器人的总仓库，上位机和下位机以 git 子模块的形式挂在下面。

| 目录 | 内容 | 仓库 |
|------|------|------|
| `host/` | 上位机（RDK X5）：网球识别、捡球控制、ROS 2 球场定位 | [HelloBalls-Host](https://github.com/SHKinsem/HelloBalls-Host) |
| `device/` | 下位机（ESP32-S3）：电机、IMU、收球 / 发射机构 | [HelloBalls-2.0](https://github.com/SHKinsem/HelloBalls-2.0) |
| `docs/` | 跨上下位机的文档（交接文档、串口协议等） | |

```bash
git clone https://github.com/SHKinsem/HelloBalls-Workspace.git
cd HelloBalls-Workspace
git submodule update --init host device
```

项目说明、开发约定和待办见 [`CLAUDE.md`](CLAUDE.md)。
