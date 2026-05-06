############################################################################
# boomerang — PX4 module CMakeLists.txt
#
# INSTALLATION
# ────────────
# 1. Copy this entire directory to:
#      PX4-Autopilot/src/modules/boomerang/
#
# 2. Enable in the Cube Orange board config:
#      boards/cubepilot/cubeorange/default.px4board
#    Add:
#      CONFIG_MODULES_BOOMERANG=y
#
# 3. Module enable/disable requirements:
#
#    DISABLE (must not fight our actuator outputs):
#      CONFIG_MODULES_CONTROL_ALLOCATOR=n
#
#    ENABLE (required — this module depends on all of these):
#      CONFIG_MODULES_MC_RATE_CONTROL=y     ← NOW ENABLED (was disabled)
#      CONFIG_MODULES_MC_ATT_CONTROL=y
#      CONFIG_MODULES_MC_POS_CONTROL=y
#      CONFIG_MODULES_EKF2=y
#
#    mc_rate_control reads our overridden vehicle_angular_velocity (despun
#    platform rates) and writes vehicle_torque_setpoint.  BoomerangController
#    subscribes to vehicle_torque_setpoint and maps roll/pitch torque into
#    CyclicMixer.  mc_rate_control never reaches ControlAllocator because
#    that module is disabled.
#
# 4. Build:
#      make cubepilot_cubeorange_default
#
# STARTUP
# ───────
# Add to boards/cubepilot/cubeorange/init.d/rcS or the appropriate
# vehicle setup script.  Start order matters — boomerang must start LAST
# so its vehicle_attitude and vehicle_angular_velocity overrides take
# effect after EKF2 has already started publishing:
#
#   ekf2 start
#   mc_pos_control start
#   mc_att_control start
#   mc_rate_control start
#   boomerang start
#
# PX4 PARAMETER RECOMMENDATIONS
# ──────────────────────────────
# Set these in QGroundControl before first flight:
#
#   MC_YAW_WEIGHT    = 0.0   (disable yaw control in mc_att_control)
#   MC_ROLLRATE_MAX  = 360   (deg/s — set to a sensible platform rate limit)
#   MC_PITCHRATE_MAX = 360   (deg/s — same)
#   MPC_THR_HOVER    = 0.5   (initial hover throttle — tune after first hover)
#
#   BC_TORQUE_ROLL_GAIN  = 1.0  (start here, tune up if response is weak)
#   BC_TORQUE_PITCH_GAIN = 1.0
#
# DEBUGGING
# ─────────
# After boot, run on the NSH console:
#   boomerang status
#
# This prints: state, virtual heading, rotor RPM, despun rates (p_virt/q_virt),
# cyclic demands, torque_sp validity, and per-blade flap commands.
#
# To confirm vehicle_attitude and vehicle_angular_velocity have only ONE
# instance each (not two), run:
#   uorb top vehicle_attitude
#   uorb top vehicle_angular_velocity
#
############################################################################

if(CONFIG_MODULES_BOOMERANG)

px4_add_module(
    MODULE modules__boomerang
    MAIN boomerang
    COMPILE_FLAGS
        -Wno-sign-compare
    SRCS
        BoomerangController.cpp
        BoomerangController.hpp
        AzimuthTracker.cpp
        AzimuthTracker.hpp
        CyclicMixer.cpp
        CyclicMixer.hpp
        OutputStage.cpp
        OutputStage.hpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
)

endif()
