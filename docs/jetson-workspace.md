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

Equivalent manual command:

```bash
cd ~/graduation-project
git pull
cd ~/graduation-project/jetson/catkin_ws
catkin_make
source devel/setup.bash
roslaunch magni_nav jetson_autodrive.launch
```
