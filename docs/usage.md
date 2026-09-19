# FR3 在 MuJoCo 中进行视觉抓取

入口是本目录的 `scripts/run_demo.sh`。当前实现：**真实仿真 RGB-D → 红绿蓝色块检测与标注 → TF2 三维定位 → MoveIt2 → MuJoCo 关节执行 → 夹爪接触抓取 → 抬升验证**。

桌面程序同时显示 MuJoCo、RViz 和 ROS 相机画面。抓取由命令触发，成功后保持物体；复位后可以再次抓取。第一版针对桌上已知尺寸的 3 cm 方块，检测器采用 OpenCV HSV 分割。没有假设通用 YOLO 权重能够识别这些自定义色块。

## 1. 复制到 Ubuntu 22.04

复制**包含 `README.md`、`scripts/`、`mujoco_simulations_fr3/` 和 `franka_ros2_ws/` 的项目根目录**，例如放到 `~/franka_project_ros2`。保留整个场景的 `assets/` 和 `franka_description/meshes/`。

需要已安装 ROS2 Humble。安装脚本补装 MoveIt、图像/TF 依赖，并创建使用系统 ROS Python 包的虚拟环境，固定 MuJoCo **3.13.0**、NumPy `<2`。不要在这个环境安装另一个 pip OpenCV，它可能覆盖 ROS 使用的系统 OpenCV。

```bash
cd ~/franka_project_ros2
bash scripts/setup_ubuntu.sh
```

它只构建 `franka_description`、`fr3_vision_grasp` 两个包，输出到 `franka_ros2_ws/{build_vision,install_vision,log_vision}`。旧构建产物已清理，安装脚本会重新生成所需输出。后续修改 Python、launch 或 XML 后执行：

```bash
bash scripts/build.sh
```

## 2. 启动并抓取

桌面终端 1：

```bash
cd ~/franka_project_ros2
bash scripts/run_demo.sh
```

会打开三个窗口：

- **MuJoCo**：实际物理仿真，机械臂和物体会运动。
- **FR3 | ROS2 camera + detection**：左侧原始相机，右侧识别框、颜色和机器人坐标系中的米制坐标。
- **RViz**：机器人、MoveIt 规划场景和检测位置。规划障碍物在启动抓取时加入。

终端 2：

```bash
cd ~/franka_project_ros2
source scripts/env.sh
ros2 service call /pick/start std_srvs/srv/Trigger '{}'
ros2 topic echo /pick/status --qos-durability transient_local
```

`/pick/start` 返回的是“任务已接受”，最终结果看 `/pick/status`：

```text
DETECTING → OPENING → APPROACHING → DESCENDING → CLOSING → LIFTING → SUCCEEDED
```

`SUCCEEDED` 要求真实 MuJoCo 物体被双指夹住，中心高于初始中心至少 5 cm，并持续保持。程序不修改物体位姿来模拟抓取。默认抬升目标为 10 cm，实际接触下物体抬升会略小。

再次尝试：

```bash
ros2 service call /pick/reset std_srvs/srv/Trigger '{}'
# 等 /pick/status 回到 READY，再启动
ros2 service call /pick/start std_srvs/srv/Trigger '{}'
```

取消：

```bash
ros2 service call /pick/cancel std_srvs/srv/Trigger '{}'
```

等待当前任务退出，再调用 `/pick/reset`。任务执行过程中拒绝重置，以免状态和轨迹不一致。已经抓住物体时再次启动也会被拒绝，需要先复位。

换颜色时关闭原 launch，再启动：

```bash
bash scripts/run_demo.sh color:=green
# 或 color:=blue
```

每个终端都要 `source scripts/env.sh`；默认 `ROS_DOMAIN_ID=77`，所有终端保持一致。关闭整套系统用启动终端的 Ctrl+C。

## 3. 无桌面测试与性能

无桌面运行：

```bash
MUJOCO_GL=egl bash scripts/run_demo.sh viewer:=false use_rviz:=false camera_view:=false
```

桌面运行使用默认 GLFW，不要沿用无桌面的 `MUJOCO_GL=egl`；必要时先 `unset MUJOCO_GL`。

相机默认 640×480、目标 15 Hz。实际帧率受 OpenGL、GPU 和 CPU 影响，**15 Hz 是配置值，不是性能保证**。RGB-D 渲染在独立线程，不用等待一帧渲染完才执行轨迹。软件 OpenGL 下关闭渲染阴影和反射以降低开销，物理接触不受影响。

```bash
bash scripts/run_demo.sh camera_rate:=8.0
ros2 topic hz /camera/color/image_raw
ros2 topic hz /perception/annotated_image
```

需要独立显示图像时：

```bash
ros2 launch fr3_vision_grasp camera_view.launch.py
# 或使用 rqt_image_view，选择 /perception/annotated_image
ros2 run rqt_image_view rqt_image_view
```

## 4. 相机在哪里定义

当前运行场景：`mujoco_simulations_fr3/scene_grasp.xml`。

```xml
<camera name="top_camera" pos=".50 0 1.25"
        mode="fixed" euler="0 0 0" fovy="50" />
```

- 这是固定顶视相机，位置相对于 MuJoCo 世界坐标。
- MuJoCo 相机向局部 `-Z` 看；ROS optical frame 的 `+Z` 向前。因此 bridge 使用 `diag(1,-1,-1)` 转换轴约定。
- `simulation.py` 从模型读取实际相机位姿、FOV，再生成内参和静态 TF。
- `fx = fy = H / (2 tan(fovy/2))`；像素中心取 `((W-1)/2,(H-1)/2)`。
- `Renderer` 输出 RGB 和单位为米的几何深度；两者取自同一次场景快照，使用相同时间戳。
- 深度反投影得到 optical frame 中的表面点，TF2 再变换到 `fr3_link0`。
- 当前已知方块高 3 cm，从可见顶面减去半高度得到方块中心，供抓取使用。

当前桌面 z=0.05 m，色块顶面 z≈0.08 m，因此桌面深度约 **1.20 m**，方块顶面深度约 **1.17 m**。相机的 GUI 自由视角不会改变 `top_camera` 的输出。

原始 `scene.xml` 保留用于对照；演示场景把原来的大方块改为可夹取的 3 cm 方块，并调整位置。共享的 `fr3.xml` 调整了 TCP、指尖碰撞垫和夹爪力限制，所以原场景加载时也会使用这些机器人修正。


## 5. 模块与接口

新包：`franka_ros2_ws/src/fr3_vision_grasp`。

| 文件 | 作用 |
|---|---|
| `simulation.py` | 模型加载、物理状态、RGB-D 快照、接触查询 |
| `bridge.py` | Viewer、图像、CameraInfo、关节反馈、时钟、TF、机械臂/夹爪 action |
| `core.py` | 颜色分割、反投影、轨迹检查和插值 |
| `perception.py` | 同步 RGB-D、过滤桌面色块、标注和 TF2 定位 |
| `pick.py` | 目标稳定性、PlanningScene、MoveIt 接近/直线下降/抬升、夹爪与结果验证 |
| `camera_view.py` | ROS 实时原图与识别图显示 |
| `launch/fr3_sim.launch.py` | 启动整条链路，生成与 MuJoCo 关节范围一致的 URDF |

| 话题 / action / service | 类型和约定 |
|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image`，RGB8 |
| `/camera/depth/image_raw` | `sensor_msgs/Image`，32FC1，米，与 RGB 对齐 |
| `/camera/{color,depth}/camera_info` | `sensor_msgs/CameraInfo` |
| `/perception/annotated_image` | `sensor_msgs/Image`，BGR8 |
| `/perception/detections` | `vision_msgs/Detection2DArray`，颜色类别和框 |
| `/perception/{red,green,blue}/pose` | `geometry_msgs/PoseStamped`，`fr3_link0` |
| `/perception/markers` | RViz MarkerArray |
| `/joint_states`、`/clock` | 实际关节状态、仿真时间 |
| `/fr3_arm_controller/follow_joint_trajectory` | 七关节 `FollowJointTrajectory` action |
| `/franka_gripper/gripper_action` | `GripperCommand`，position 是**单指行程** 0～0.04 m，总开口为其两倍 |
| `/pick/start`、`/pick/cancel`、`/pick/reset` | `std_srvs/Trigger` |
| `/pick/status` | JSON 字符串，包含 state、message、color |
| `/simulation/grasp_metrics` | 仅用于仿真评估：物体真值和接触，检测器不读取它 |

MoveIt 通过标准 action 控制 MuJoCo 的 position 执行器，没有运行 fake hardware 或真机 driver。桌面与容器根据 XML 加入 PlanningScene；方块位置来自视觉。抓住后增加 attached collision object 仅供规划避障，实际物体仍由 MuJoCo 的接触力和摩擦驱动。

## 6. 复现验收

先退出正在运行的 demo。自动脚本会启动独立演示、测量三色定位误差、执行抓取、检查双指持续接触和实际抬升，再复位重复：

```bash
source scripts/env.sh
MUJOCO_GL=egl python scripts/validate_demo.py --launch --trials 2 --output validation_results/red
MUJOCO_GL=egl python scripts/validate_demo.py --launch --color green --trials 1 --output validation_results/green
# 有桌面时，可一边显示三个窗口一边验收
python scripts/validate_demo.py --launch --gui --color blue --trials 1 --output validation_results/blue
```

输出包括 `report.json`、`launch.log` 和抓取前后的原图/识别图。定位误差阈值为 5 mm，抬升阈值为 5 cm，成功后继续检查至少 2 秒仿真时间的双指接触。

模型与算法回归测试：

```bash
python -m pytest franka_ros2_ws/src/fr3_vision_grasp/test -q
```

在**刚启动、尚未抓取的 demo** 上测试 action 校验、取消、夹爪开合与忙时拒绝复位：

```bash
python scripts/check_controls.py
```

清理前保留的历史验证记录见 [validation/README.md](validation/README.md)。验证使用 Ubuntu 22.04 容器、ROS2 Humble、MuJoCo 3.13.0；图形窗口通过虚拟 X 显示运行，仍需在你的独立 Ubuntu 电脑上确认实际 GPU、屏幕和帧率。

## 7. 当前边界与后续扩展

当前支持固定顶视相机、已知尺寸的轴对齐红绿蓝方块、每种颜色一个候选、桌面静止目标，抓起后保持。感知限定桌面高度和工作区域，以排除同色容器；被手臂遮挡或已抬离桌面的方块不再作为新的桌面目标。它不是未知物体姿态估计或动态追踪抓取系统。

接 YOLO 时替换 `perception.py` 的区域检测部分，保留 RGB-D、CameraInfo、采集时间戳、TF2 和目标 PoseStamped 接口。接 RealSense 时替换图像源并使用对齐深度和标定外参；运动腕部相机需要动态 TF。ACT、强化学习、VLA 可在基础链路经过更多场景验证后再接入。

常见排查：

- 无图像：检查 MuJoCo OpenGL 日志和 `/camera/color/image_raw`；没有 GPU 的服务器使用 EGL 无窗口模式。
- 无检测：检查图像是否看见方块、`/tf_static`、工作区/桌面高度；修改场景后需要重新构建。
- MoveIt 执行不动：确认只有本 bridge 发布 `/joint_states`，两个 action server 均存在。
- Python/NumPy/OpenCV 错误：重新使用本项目 `.venv-fr3`，不要 source 原来的 install，不要混用 Conda。
- Ctrl+C 时部分 Humble MoveIt 2.5.10 环境会在其退出析构阶段打印 segmentation fault；本次容器观察到这一上游退出问题，抓取执行和测试报告在关闭前完成。不要把该退出日志当作抓取成功的证据。
