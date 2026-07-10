# Encoderless DC Motor Demo Mode

This mode is only for temporary testing before the encoder motors are available.
The normal encoder-motor flow is still `autodrive` with `stm/main.cpp`.

## Files

- Jetson launch: `dc_autodrive`
- Jetson planner params: `jetson/catkin_ws/src/magni_nav/param/base_local_planner_dc_param.yaml`
- Jetson fake odom: `jetson/catkin_ws/src/magni_nav/scripts/encoderless_fake_odom.py`
- STM temporary code: `stm/main_dc_encoderless.cpp`

## Jetson commands

```bash
cd ~/graduation-project
git pull --ff-only
bash jetson/setup_jetson_shell.sh
source ~/.bashrc
dc_autodrive
```

Use the same navigation and voice terminals as before:

```bash
rosrun magni_nav navi.py
cd ~/llm
python3 main.py
```

On the MSI laptop:

```bash
rviz_nav
```

## STM build

For encoderless DC motor testing, flash `stm/main_dc_encoderless.cpp`.
When the encoder motors are installed again, go back to `stm/main.cpp`.

Both Jetson and STM DC-mode limits are set to:

- `max linear x`: `0.04 m/s`
- `max angular z`: `0.05 rad/s`

## Field tuning

Start with a short goal, not a long classroom delivery.

If the robot moves but RViz scan shifts forward too fast or too slow, tune these in
`dc_autodrive.launch`:

- `linear_scale`: fake odom forward distance
- `angular_scale`: fake odom rotation angle

If the motors do not start at low command speed, tune these in
`stm/main_dc_encoderless.cpp`:

- `PWM_MIN_MOVE`
- `FORWARD_PWM_LIMIT`
- `BACKWARD_PWM_LIMIT`

The current DC test values are intentionally higher than the first conservative
setting because the test motors did not reliably overcome static friction at the
lower PWM range:

- `PWM_MIN_MOVE`: `32`
- `FORWARD_PWM_LIMIT`: `56`
- `BACKWARD_PWM_LIMIT`: `44`

Encoderless mode can drift. It is acceptable only for a slow demo in a simple,
obstacle-free corridor. Final autonomous driving should use encoder motors.
