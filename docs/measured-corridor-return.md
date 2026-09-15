# Measured-wall home return

Home return uses `/cmd_vel_return`; outbound DWA and door maneuvers keep their
existing command paths. The map pose is used for proximity/progress checks,
initial homeward turn, and final initial-heading alignment. It no longer
forces a mid-corridor rotation when the map heading differs by 22 degrees.

`return_corridor.py` fits side-wall lines and learns an odometry-frame corridor
axis and center from eight normal-width observations. Line-fit residuals,
not lateral-distance spread, determine whether a slanted wall is planar.
Opposite travel uses the same physical reference, with the heading reversed.
At an opening, a visible line must match a previously observed wall; a recessed
door surface is not treated as a new corridor boundary. The measured width is
2.36 m. The 0.08 m width tolerance and 0.10 m single-wall match tolerance are
engineering settings, tested synthetically, not calibrated from a rosbag.

The controller combines measured heading and a center-directed heading offset
capped at 8 degrees, with angular output capped at 0.06 rad/s. Cruise is at
most 0.10 m/s and correction speed is at most 0.04 m/s. Missing wall observations
permit at most 0.50 m displacement from the last trusted wall observation.
Larger gaps, stale scan/odom, excessive heading mismatch, or insufficient
clearance hold zero velocity. Odometry discontinuities invalidate the reference.
There is no blind fallback to a fixed map heading. A 0.5 s command watchdog
stops output if return commands cease. The existing mission timeout still applies.

Within 1.90 m Euclidean map distance of home, navigation stops and aligns with
the saved initial heading. No x/y or longitudinal positioning maneuver follows.
Final verification retains the 2.00 m proximity and 4-degree heading tolerances.

The supplied first log contains a recognized pause and replacement delivery,
followed by outbound DWA failure. That DWA failure is not fixed by this return
change. Cancellation logging and inactive-goal cancellation were corrected.
The second log shows repeated map-heading realignment followed by wall loss
and rotation refusal; this change addresses that return control sequence.

Validation: `python -B -m unittest discover -s tests -p test_return_corridor.py`.
Tests use synthetic walls, a differential-drive simulation, and mocked ROS
callbacks. They do not establish real-world collision clearance, lidar/odom
calibration, or successful physical return. ROS/catkin and hardware validation
remain required on the Jetson. `robot_start` pulls and runs `catkin_make`.
