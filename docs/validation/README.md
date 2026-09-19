# 历史验证记录

以下为清理前项目文档中记录的结果，原始报告未包含在本仓库中，不代表每次环境安装后的实测结果；请按使用说明重新运行验收脚本。

验证环境：Ubuntu 22.04 容器、ROS2 Humble、MoveIt2、MuJoCo 3.13.0、Python 3.10。验证脚本为 `scripts/validate_demo.py`，它只把 `/simulation/grasp_metrics` 作为验收真值，不提供检测目标。

| 项目 | 结果 |
|---|---|
| 红色目标，两次连续抓取 | 2/2 成功 |
| 蓝色目标，一次抓取 | 1/1 成功 |
| RGB-D 与识别图 | 运行中持续发布 |
| 视觉到 MuJoCo 真值误差 | 红 0.02–1.12 mm，绿 2.08 mm，蓝 1.57 mm |
| 物体实际抬升 | 约 86.8–90.9 mm |
| 抬升后双指接触保持 | 通过，连续检查 2 秒仿真时间 |
| MuJoCo / ROS 时间推进 | 通过 |
| malformed trajectory / 并发 action / 忙时 reset | 均按预期拒绝 |
| 取消轨迹 | 通过，保持实际关节位置 |
| 夹爪 0 / 0.04 m 单指行程 | 开合与到位反馈通过 |
| 内参、反投影、轨迹、URDF/MJCF TCP 回归测试 | 3 passed |

测试脚本会生成 `report.json`、`launch.log` 和抓取前后的图像。软件渲染时实际相机频率取决于机器；本次 3.13.0 EGL 验证约 5.7 Hz，配置仍为 15 Hz。机器人关节状态约 68–88 Hz，仿真速度约 0.68–0.88 倍实时。独立 Ubuntu 电脑若有 GPU，通常会更快。

示例识别画面：[camera_detection.png](../camera_detection.png)。图中标签是 TF2 转换后的 `fr3_link0` 坐标，不是摄像头坐标。

关闭 launch 时，容器中的 MoveIt Humble 2.5.10 曾在析构阶段打印上游 segmentation fault；它发生在所有任务和报告已完成之后，不能代表执行失败。正常启动、规划、action 和物理抓取均在退出前通过。
