# Jetson Workspace

This repository is the active ROS workspace source for the Jetson Nano.

Expected path on the Jetson:

```bash
~/graduation-project
```

One-time shell setup on the Jetson:

```bash
cd ~/graduation-project
bash jetson/setup_jetson_shell.sh
source ~/.bashrc
```

The setup keeps the old `~/catkin_ws` directory but removes its active
`source` line from `~/.bashrc`. It also removes duplicate manual sourcing of
the graduation-project workspace, comments out previous fixed ROS IP exports,
and configures this Jetson-only runtime:

```bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP
unset ROS_HOSTNAME
```

This local mode is independent of the phone hotspot address. Reconfigure ROS
networking before using RViz from a separate MSI laptop.

Build and verify without hardware:

```bash
cd ~/graduation-project
git pull
cd ~/graduation-project/jetson/catkin_ws
catkin_make
source devel/setup.bash
rospack find magni_nav
```

Run autonomous driving:

```bash
autodrive
```

Run autonomous driving, `navi.py`, and `~/llm/main.py` together in one
terminal:

```bash
robot_start
```

`robot_start` waits for `/move_base/status`, `/scan`, and `/odom` before
starting voice navigation. Press `Ctrl+C` once to stop all three processes.
It uses `~/llm/venv`, then `~/llm/.venv`, and finally the system `python3`.

The LLM directory must contain these five Python files:

```text
main.py
llm_module2.py
config.py
realtime_stt2.py
tts_module2.py
```

`main_node.py` is not used. Runtime assets such as `silero_vad.onnx`,
`sounds/`, and `.env` remain in the same LLM directory.

Store the API key only in the Jetson's local `~/llm/.env` file:

```bash
cd ~/llm
nano .env
chmod 600 .env
```

The file contains one line:

```text
OPENAI_API_KEY=replace_with_a_new_key
```

Do not commit `.env`. `robot_start` directly uses `~/llm/venv/bin/python3`
when it exists, so manually running `source venv/bin/activate` is unnecessary.

Send a named navigation goal from another Jetson SSH terminal:

```bash
rosrun magni_nav navi.py 544호
```

Multiple CLI destinations are visited in order. The robot faces each stored
destination orientation and waits three seconds before continuing. Before the
next destination, it reverses 0.50 m using `/odom`, rotates in place toward the
next destination's corridor-center pose, and only then starts `move_base`.
After the last CLI or LLM destination, it remains stopped there. When a room
has a stored `_중앙` pose, navigation uses it to align with the room before
the final lidar-controlled approach:

```bash
rosrun magni_nav navi.py "544호" "540호" "542호"
```

The original Magni destination database remains unchanged. The large platform
navigates to the stored corridor-center pose, rotates toward the stored room
coordinate, and approaches at low speed until the front laser is 0.41 m from
the door. This represents a 0.30 m gap from the robot's front edge. If `/scan`
or `/odom` stops updating, the approach aborts and publishes a stop command.

Test room approaches separately:

```bash
rosrun magni_nav navi.py 542호
rosrun magni_nav navi.py 544호
```

The separate `542호_대형` test destination is still 0.30 m behind the original
542 room pose and shares its center waypoint. It continues to use `move_base`
for its final pose. Run it with:

```bash
rosrun magni_nav navi.py 542호_대형
```

A center-assisted final approach temporarily tightens the DWA position
tolerance from 1.00 m to 0.15 m, then restores it.

One-time setup on the MSI Ubuntu laptop:

```bash
cd ~/graduation-project
git pull --ff-only
bash jetson/setup_msi_shell.sh
source ~/.bashrc
```

After `autodrive` is running on the Jetson, run RViz from the MSI terminal:

```bash
rviz_nav
```

`rviz_nav` fixes the ROS master and MSI addresses for this project and checks
the navigation nodes and topics before opening the managed RViz configuration.

Equivalent manual command:

```bash
cd ~/graduation-project
git pull
cd ~/graduation-project/jetson/catkin_ws
catkin_make
source devel/setup.bash
roslaunch magni_nav jetson_autodrive.launch
```

## Encoder calibration

The encoder-motor configuration uses the measured wheel-output counts below:

```text
left:  203190 ticks/rev
right: 202795 ticks/rev
```

These values must remain identical in `stm/main.cpp` and
`jetson/catkin_ws/src/magni_nav/src/odom_publisher.py`. After changing the STM
firmware, rebuild and flash it in STM32CubeIDE. After changing the Jetson code,
pull the repository and rebuild the catkin workspace before launching
`autodrive`.
