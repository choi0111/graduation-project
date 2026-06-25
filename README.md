# Graduation Project

STM code and Jetson Nano ROS autonomous driving code.

## Jetson Nano

The active Jetson workspace is:

```bash
~/graduation-project/jetson/catkin_ws
```

One-time setup on the Jetson:

```bash
cd ~/graduation-project
bash jetson/setup_jetson_shell.sh
source ~/.bashrc
```

Run:

```bash
autodrive
```

Send a destination from another Jetson SSH terminal:

```bash
rosrun magni_nav navi.py 544호
```

See [docs/jetson-workspace.md](docs/jetson-workspace.md) for the manual commands.
