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
#      CONFIG_MODULES_boomerang=y
#
# 3. Disable mc_rate_control (it must not fight our actuator outputs):
#    In the same .px4board file, ensure these lines are ABSENT or set to n:
#        CONFIG_MODULES_MC_RATE_CONTROL=n
#        CONFIG_MODULES_CONTROL_ALLOCATOR=n
#
#        # ADD this line:
#        CONFIG_MODULES_BOOMERANG_CONTROLLER=y
#
# 4. Keep these modules ENABLED (we depend on them):
#      CONFIG_MODULES_MC_ATT_CONTROL=y
#      CONFIG_MODULES_MC_POS_CONTROL=y
#      CONFIG_MODULES_EKF2=y
#
# 5. Build:
#      make cubepilot_cubeorange_default
#
# STARTUP
# ───────
# Add to boards/cubepilot/cubeorange/init.d/rcS or the appropriate
# vehicle setup script:
#
#   boomerang start
#
# This should be started AFTER ekf2, mc_att_control, and mc_pos_control.
#
# PX4 PARAMETER RECOMMENDATIONS
# ──────────────────────────────
# Set these in QGroundControl before first flight:
#
#   MC_YAW_WEIGHT    = 0.0   (disable yaw control in mc_att_control)
#   MC_ROLLRATE_MAX  = 0.0   (no body rate limits — we don't use them)
#   MC_PITCHRATE_MAX = 0.0
#   MPC_THR_HOVER    = 0.5   (initial hover throttle — tune after first hover)
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
