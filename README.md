# FR3 Vision Grasping Simulation

A Franka FR3 grasping demo based on **ROS 2 Humble + MoveIt 2 + MuJoCo**:

**RGB-D Camera → OpenCV Color Detection → 3D Localization → Motion Planning → Physical Contact Grasping → Placement**

Currently supports red, green, and blue **3 cm blocks** using a fixed overhead camera. One start request sorts all visible tabletop blocks into matching bins. The arm returns to its initial posture after each placement, checks for remaining blocks, and stays at home when the table is empty. This project is for simulation only.

## Run

**Environment:** Ubuntu 22.04 with ROS 2 Humble installed.

Run the following commands from the project root:

```bash
bash scripts/setup_ubuntu.sh
bash scripts/run_demo.sh
```

In another terminal, from the same directory:

```bash
source scripts/env.sh
ros2 service call /pick/start std_srvs/srv/Trigger '{}'
ros2 topic echo /pick/status --qos-durability transient_local
```

Call `/pick/start` once for the entire table. Use `/pick/reset` only to restore the scene for another complete trial.

The default order is red, green, blue. To choose the first color (the others are still sorted automatically):

```bash
bash scripts/run_demo.sh color:=green
```

or:

```bash
bash scripts/run_demo.sh color:=blue
```

After modifying the source code:

```bash
bash scripts/build.sh
```

For detailed usage, interfaces, and validation procedures, see [Usage Guide](docs/usage.md).

## License & Sources

This project is licensed under [Apache-2.0](LICENSE).

The robot description is based on [Franka Robotics/franka_description](https://github.com/frankarobotics/franka_description), and the MuJoCo model is based on [MuJoCo Menagerie/franka_fr3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_fr3).

Upstream licenses, copyright notices, and additional terms are preserved. See [Third-Party Notices](THIRD_PARTY_NOTICES.md) for details on the applicable components and modifications.

This project is not affiliated with or officially endorsed by Franka Robotics.
