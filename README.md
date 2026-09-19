# FR3 视觉抓取仿真

基于 **ROS 2 Humble + MoveIt 2 + MuJoCo** 的 Franka FR3 抓取演示：RGB-D 相机 → OpenCV 颜色检测 → 三维定位 → 运动规划 → 物理接触抓取。

目前支持固定顶视相机下的红、绿、蓝色 3 cm 方块；仅用于仿真。

## 运行

环境：Ubuntu 22.04，已安装 ROS 2 Humble。以下命令在本项目根目录执行。

```bash
bash scripts/setup_ubuntu.sh
bash scripts/run_demo.sh
```

另开终端，进入同一目录：

```bash
source scripts/env.sh
ros2 service call /pick/start std_srvs/srv/Trigger '{}'
ros2 topic echo /pick/status --qos-durability transient_local
```

再次抓取前调用 `/pick/reset`；切换颜色用 `bash scripts/run_demo.sh color:=green`（或 `blue`）。修改代码后运行 `bash scripts/build.sh`。

详细操作、接口和验证方法见 [使用说明](docs/usage.md)。

## 许可与来源

本项目代码采用 [Apache-2.0](LICENSE)。机器人描述来自 [Franka Robotics/franka_description](https://github.com/frankarobotics/franka_description)，MuJoCo 模型基于 [MuJoCo Menagerie/franka_fr3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_fr3)。上游许可证、版权声明及附加条款均保留，具体范围与修改见 [第三方说明](THIRD_PARTY_NOTICES.md)。本项目与 Franka Robotics 无官方隶属关系。
