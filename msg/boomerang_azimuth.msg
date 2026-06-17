# Boomerang rotor azimuth state
# Published by boomerang_estimator at 200 Hz.
# Consumed by ActuatorEffectivenessBoomerang inside ControlAllocator.

uint64 timestamp          # hrt_absolute_time() in microseconds

float32[4] theta          # Blade azimuths in NED frame (rad), blade 0..3
                          # theta[0] = EKF2 yaw directly
                          # theta[1..3] = theta[0] + 90/180/270 deg, wrapped to [-pi,pi]

float32 omega_rad_s       # Rotor spin rate (rad/s), positive = CCW from above
float32 rpm               # Derived from omega_rad_s for convenience

bool valid                # False until first update from EKF2
