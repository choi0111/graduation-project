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

Multiple CLI destinations are visited in order. The robot faces each stored
destination orientation and waits three seconds before continuing. After the
last destination, it remains stopped there. When a room has a stored `_중앙`
pose, navigation uses it to align with the room before the short final approach:

```bash
rosrun magni_nav navi.py "544호" "540호" "542호"
```

The original Magni destination database remains unchanged. For `542호` and
`544호`, the large platform now navigates to the stored corridor-center pose,
rotates toward the stored room coordinate, performs a fine angular-only
alignment to within 0.035 rad, and then drives forward 0.30 m at 0.05 m/s.
The final distance is measured from `/odom`; it is not a time-based movement.
If `/odom` stops updating, the approach aborts and publishes a stop command.

Test the two fixed-distance approaches separately:

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
