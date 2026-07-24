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

Run autonomous driving, voice navigation, and the LLM process together:

```bash
robot_start
```

In voice-command mode, the 20-second receipt timer starts after the destination
arrival TTS finishes. An `SCENARIO_8` receipt command advances immediately;
otherwise the delivery is marked unattended and the robot advances after the
timeout. After the final destination, the robot backs away and returns to the
fixed AMCL initial pose.
`SCENARIO_21` pauses without discarding the active destination, and
`SCENARIO_22` replans and resumes that same mission.

Send a destination from another Jetson SSH terminal:

```bash
rosrun magni_nav navi.py 544호
```

The direct `rosrun` form is a navigation test mode. It does not run the
voice-receipt waiting and automatic-return workflow.

See [docs/jetson-workspace.md](docs/jetson-workspace.md) for the manual commands.
