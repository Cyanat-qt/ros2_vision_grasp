# FR3 visual grasp configuration

The default launch uses `scene_grasp.xml`, a small-block benchmark derived from the project scene. It keeps the original `scene.xml` unchanged and moves the three movable blocks to the reachable tabletop region.

Important parameters:

```bash
ros2 launch fr3_vision_grasp fr3_sim.launch.py viewer:=true use_rviz:=true
ros2 run fr3_vision_grasp camera_view
ros2 topic pub --once /perception/pick_request std_msgs/msg/String '{data: pick}'
```

Use `viewer:=false` on headless Ubuntu. Set `MUJOCO_GL=egl` for headless rendering.
