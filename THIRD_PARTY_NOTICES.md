# 第三方来源与许可

本项目自身代码沿用 `fr3_vision_grasp/package.xml` 已声明的 Apache-2.0，全文见根目录 [LICENSE](LICENSE)。第三方代码和模型保留各自版权及许可；根目录许可不替代上游附加条款。

## Franka 机器人描述

- 路径：`franka_ros2_ws/src/franka_description/`。
- 来源：<https://github.com/frankarobotics/franka_description>。
- 版本：2.8.1；本地原始 Git HEAD 为 `02afaae282d4a8e10d7d2f781b23b3515c303ce5`。
- 版权：Franka Robotics GmbH；各文件年份以原始声明为准。
- 许可：Apache-2.0；原始 [LICENSE](franka_ros2_ws/src/franka_description/LICENSE) 还包含 BSD 条款，完整保留。原始 [NOTICE](franka_ros2_ws/src/franka_description/NOTICE) 同样保留。
- 本地修改：裁剪为 FR3 与白色 Franka Hand 所需资源；移除其他机器人、附件、夹爪、上游 CI/Docker 配置、依赖 Docker 的辅助脚本和嵌套 Git 历史；调整 CMake 安装目录、可视化参数说明和测试型号范围，安装时附带许可文件，README 增加裁剪说明。保留的 URDF、SRDF、网格及公共宏未改写。

## MuJoCo FR3 模型

- 路径：`mujoco_simulations_fr3/`，包含 `assets/` 网格。
- 来源：<https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_fr3>；模型由 Franka 官方 URDF 派生，转换过程见该目录保留的 [README](mujoco_simulations_fr3/README.md)。本地副本没有保存精确上游提交号。
- 许可：Apache-2.0；原始 [LICENSE](mujoco_simulations_fr3/LICENSE) 中的 BSD 附加条款亦完整保留。派生自 Franka 的版权告知另随该目录 `NOTICE` 分发。
- 本地修改：`fr3.xml` 调整执行器、TCP、指尖碰撞及夹爪参数；`scene.xml` 扩展桌面、色块、容器与相机；`scene_grasp.xml` 在此基础上配置 3 cm 色块与抓取场景。修改告知已写入这些文件。移除了场景未引用的六个整块 OBJ 网格，保留实际使用的分块网格。

## 外部依赖

ROS 2、MoveIt 2、MuJoCo、OpenCV、NumPy 等由安装脚本安装，不随本仓库捆绑分发；其许可证以各自发行包为准。当前检测器使用 OpenCV HSV 分割，不包含 YOLO 代码或权重，也不包含 JetArm 镜像提取源码。

## 再发布时保留什么

README 的出处链接不能替代许可证。分发本项目或派生版本时，应保留对应的 `LICENSE`、`NOTICE`、源文件版权/归属告知，并在修改过的上游文件中注明修改。Apache-2.0 允许按其条款使用、修改及商用，不授予 Franka 等名称或商标的推广授权；不要暗示官方背书。
