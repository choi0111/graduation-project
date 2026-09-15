# Home return baseline restoration

Restored navigation and its matching configuration to `fcb2d9f` on 2026-09-15.
This is the revision immediately before `b9251d8` introduced final home-axis
position adjustment. It is a history-based rollback candidate, not a verified
record of the exact revision used in a successful physical test.

The restored files are:

- `jetson/catkin_ws/src/magni_nav/scripts/navi.py`
- `jetson/catkin_ws/src/magni_nav/scripts/corridor_centering.py`
- `jetson/catkin_ws/src/magni_nav/launch/move_base.launch`
- `docs/jetson-workspace.md`

Return behavior: retreat, turn homeward, reset direction-dependent wall state,
return with corridor centering at a cruise command of 0.10 m/s, stop within
1.90 m of the stored home coordinates, and align with the initial heading.
Final position verification allows 2.00 m and heading verification allows
4 degrees. There is no subsequent longitudinal positioning maneuver.

This also rolls back the later shared rotation-clearance waiting changes,
mission replacement changes, and staging-tolerance equalization. Existing
clearance and stale-sensor checks in the baseline remain. Firmware, encoder
calibration, voice code, and outbound cruise configuration were not edited.

The pre-restoration working state, including the unfinished longitudinal-only
change, is preserved locally on `backup/return-before-restore-20260915` at
`ab54c17a79c29b2483aaa9ae75dd32c847f17fa5`.

Validation: restored files match the baseline, Python source compilation and
launch XML parsing pass, and `git diff --check` passes. ROS and physical robot
tests were not run. Do not treat the rollback as proof of collision-free return.
