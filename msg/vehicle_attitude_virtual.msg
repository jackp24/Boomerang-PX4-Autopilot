# Virtual (despun) vehicle attitude
# Published by boomerang_estimator.
# Identical data layout to vehicle_attitude_s but on a separate topic
# so that Boomerang's heading override never stomps EKF2's publication.
#
# mc_att_control, mc_rate_control, and heading_led should subscribe to
# this topic instead of vehicle_attitude when running on a Boomerang airframe.

uint64 timestamp          # hrt_absolute_time() in microseconds
uint64 timestamp_sample   # timestamp of the raw EKF2 sample this was derived from

float32[4] q              # Quaternion [w, x, y, z], yaw replaced with virtual heading
float32[3] delta_q_reset  # unused — kept for layout compatibility with vehicle_attitude_s
uint8 quat_reset_counter  # unused — kept for layout compatibility
