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

Send a named navigation goal from another Jetson SSH terminal:

```bash
rosrun magni_nav navi.py 544호
```

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
