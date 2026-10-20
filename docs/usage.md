# FR3 在 MuJoCo 中进行视觉抓取

入口是本目录的 `scripts/run_demo.sh`。当前实现：**真实仿真 RGB-D → 红绿蓝色块检测与标注 → TF2 三维定位 → MoveIt2 → MuJoCo 关节执行 → 夹爪接触抓取 → 搬运至同色容器 → 松爪放置验证**。

桌面程序同时显示 MuJoCo、RViz 和 ROS 相机画面。一次 `/pick/start` 自动分拣桌面上的所有色块：每放好一个就回到启动时的初始姿态，再识别并抓取下一个。确认桌面为空后，机械臂保持初始姿态不再运动。当前针对桌上已知尺寸的 3 cm 方块，检测器采用 OpenCV HSV 分割。

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
RETURNING_HOME → AT_HOME → DETECTING
  有色块：OPENING → APPROACHING → DESCENDING → CLOSING → LIFTING
          → TRANSPORTING → PLACING → RELEASING → RETREATING → VERIFYING
          → OBJECT_PLACED → RETURNING_HOME → AT_HOME → DETECTING（循环）
  桌面为空：SUCCEEDED（保持初始姿态）
```

`OBJECT_PLACED` 表示一个色块已在同色容器底部稳定停留至少 2 秒仿真时间；此时任务会继续回位并抓取下一个。只有确认桌面为空、机械臂已经回到初始姿态后才报告 `SUCCEEDED`。默认依次处理红、绿、蓝，缺少的颜色会跳过。

每次回位后重新收集视觉数据。结束前要求至少 5 帧新的、连续的空桌面识别结果，覆盖至少 2 秒仿真时间；相机断流、TF 不可用或目标不稳定会报告失败，不会被当作正常抓完。回位使用 MoveIt 规划到 `core.py` 中的 `HOME` 关节姿态，不调用仿真复位，已放好的色块会留在容器中。

抬升后先验证双指接触保持 2 秒，搬运途中持续监测接触；若物体滑落则取消正在执行的轨迹并报告 `FAILED`。程序不修改物体位姿来模拟抓取。

红色放到 `left_red_box`，绿色放到 `right_green_box`，蓝色放到 `right_blue_box`。容器位置、底面和边缘高度来自场景 XML，色块抓取位置仍来自视觉。抬升高度会根据容器边缘自动增加，确保横向搬运时有足够间隙。

夹爪掌部比容器开口宽，因此在色块下半部进入容器、掌部仍高于箱沿时松爪，让色块自由落到底部；不会让掌部强行下降到箱底。

三个色块之间不需要重复调用服务。整桌分拣完成后，如需恢复场景再试一轮：

```bash
ros2 service call /pick/reset std_srvs/srv/Trigger '{}'
# 等 /pick/status 回到 READY，再启动
ros2 service call /pick/start std_srvs/srv/Trigger '{}'
```

取消：

```bash
ros2 service call /pick/cancel std_srvs/srv/Trigger '{}'
```

等待当前任务退出，再调用 `/pick/reset`。整个连续分拣过程中都会拒绝重复启动和复位。整轮完成后再次启动也会被拒绝，需要先复位。复位会同时清理三种颜色的规划场景状态和进度。

若要优先抓取某种颜色，关闭原 launch，再启动（仍会自动处理剩余颜色）：

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
| `pick.py` | 连续分拣、PlanningScene、MoveIt 抓取/放置/回位、空桌面与结果验证 |
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
| `/pick/status` | JSON 字符串，包含 state、message、当前 color 和已放置的 placed_colors |
| `/simulation/grasp_metrics` | 仅用于仿真评估：物体真值和接触，检测器不读取它 |

MoveIt 通过标准 action 控制 MuJoCo 的 position 执行器，没有运行 fake hardware 或真机 driver。桌面与容器根据 XML 加入 PlanningScene；方块位置来自视觉。抓住后增加 attached collision object 仅供规划避障，实际物体仍由 MuJoCo 的接触力和摩擦驱动。

`fr3.xml` 开启 `noslip_iterations="5"`，消除软接触求解导致的持续切向漂移。原参数下即使夹爪持续施力，色块也会缓慢滑出指尖；仅检查刚抬起时的接触不能发现这个问题。

## 6. 复现验收

先退出正在运行的 demo。自动脚本会启动独立演示、测量三色定位误差，一次启动完成三个色块的抓取放置，检查每次回位和最终静止，再复位重复整轮：

```bash
source scripts/env.sh
MUJOCO_GL=egl python scripts/validate_demo.py --launch --trials 2 --output validation_results/red
MUJOCO_GL=egl python scripts/validate_demo.py --launch --color green --trials 1 --output validation_results/green
# 有桌面时，可一边显示三个窗口一边验收
python scripts/validate_demo.py --launch --gui --color blue --trials 1 --output validation_results/blue
```

输出包括 `report.json`、`launch.log` 和整轮前后的原图/识别图。定位误差阈值为 5 mm，搬运时抬升阈值为 5 cm。检查每次扫描前的关节角与初始姿态误差小于 0.02 rad；成功后继续观察至少 2 秒仿真时间，要求三个色块都在对应容器底部静止、机械臂保持初始姿态且不再启动动作。`--trials 2` 表示完整分拣两轮，共放置六次。

模型与算法回归测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest franka_ros2_ws/src/fr3_vision_grasp/test -q
```

在**刚启动、尚未抓取的 demo** 上测试 action 校验、取消、夹爪开合与忙时拒绝复位：

```bash
python scripts/check_controls.py
```

清理前保留的历史验证记录见 [validation/README.md](validation/README.md)。验证使用 Ubuntu 22.04 容器、ROS2 Humble、MuJoCo 3.13.0；图形窗口通过虚拟 X 显示运行，仍需在你的独立 Ubuntu 电脑上确认实际 GPU、屏幕和帧率。

## 7. 当前边界与后续扩展

当前支持固定顶视相机、已知尺寸的轴对齐红绿蓝方块、每种颜色一个候选、桌面静止目标，以及场景中固定的同色放置容器。感知限定桌面高度和工作区域，以排除同色容器；被手臂遮挡或已抬离桌面的方块不再作为新的桌面目标。它不是未知物体姿态估计或动态追踪抓取系统。

接 YOLO 时替换 `perception.py` 的区域检测部分，保留 RGB-D、CameraInfo、采集时间戳、TF2 和目标 PoseStamped 接口。接 RealSense 时替换图像源并使用对齐深度和标定外参；运动腕部相机需要动态 TF。ACT、强化学习、VLA 可在基础链路经过更多场景验证后再接入。

常见排查：

- 无图像：检查 MuJoCo OpenGL 日志和 `/camera/color/image_raw`；没有 GPU 的服务器使用 EGL 无窗口模式。
- 无检测：检查图像是否看见方块、`/tf_static`、工作区/桌面高度；修改场景后需要重新构建。
- MoveIt 执行不动：确认只有本 bridge 发布 `/joint_states`，两个 action server 均存在。
- Python/NumPy/OpenCV 错误：重新使用本项目 `.venv-fr3`，不要 source 原来的 install，不要混用 Conda。
- Ctrl+C 时部分 Humble MoveIt 2.5.10 环境会在其退出析构阶段打印 segmentation fault；本次容器观察到这一上游退出问题，抓取执行和测试报告在关闭前完成。不要把该退出日志当作抓取成功的证据。
