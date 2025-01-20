# visual_virtual_tether
the visual version of virtual tether package, in cooperate with apriltag_detection, bluerov_sim and Mallard_sim package, input camera detection and output velociety to bluerov2 and mallard.

This is an attempt to clean up and further develop based on the original work by Kanzhong.

# What's new

- Replaced P control with PD control for virtual tether.
- Added local dead reckoning
- Added gain parameters for mixing joystick and virtual tether velocity inputs
- Added dynamic reconfiguration

# Parameters
#### ~virtual_tether/gain_kp
Default to 1. P gain for the PD controller for virtual tether.

#### ~virtual_tether/gain_kd
Default to 0. D gain for the PD controller for virtual tether. This value must be adjusted in small increment (0.0001) to avoid agressive manoeuvre.

#### ~virtual_tether/tag_yaw_offset
Default to 0. Yaw offset for tag detection. In case the tag is not mounted with the correct orientation on Bluerov, this parameter can be used to set an offset.

#### ~virtual_tether/local_dead_reckoning_timeout
Default to 5. This value determines how old the last known tag detection can be used for dead reckoning. Since this is a local dead reckoning in the camera's coordinate system, it is not recommended to set the timeout to a large value.

#### ~virtual_tether/local_dead_reckoning_fixed_velocity
Default to 0.1. This value is used to restrict the velocity of dead reckoning manoeuvre. Set to 0 to disable fixed velocity. However, it is recommended to use a fixed velocity for stability.

#### ~virtual_tether_mallard_mixer/gain_x
Overall gain for liner velocity of robot in x-axis.

#### ~virtual_tether_mallard_mixer/gain_y
Overall gain for liner velocity of robot in y-axis.

#### ~virtual_tether_mallard_mixer/gain_yaw
Overall gain for yaw velocity of robot in rad/s.

#### ~virtual_tether_mallard_mixer/joy_gain_x
Joy gain for liner velocity of robot in x-axis.

#### ~virtual_tether_mallard_mixer/joy_gain_y
Joy gain for liner velocity of robot in y-axis.

#### ~virtual_tether_mallard_mixer/joy_gain_yaw
Joy gain for yaw velocity of robot in rad/s.

#### ~virtual_tether_mallard_mixer/vt_gain_x
Virtual tether gain for liner velocity of robot in x-axis.

#### ~virtual_tether_mallard_mixer/vt_gain_y
Virtual tether gain for liner velocity of robot in x-axis.

#### ~virtual_tether_mallard_mixer/vt_gain_yaw
Virtual tether gain for yaw velocity of robot in rad/s.