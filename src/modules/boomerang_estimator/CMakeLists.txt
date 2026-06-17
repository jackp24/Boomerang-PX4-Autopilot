if(CONFIG_MODULES_BOOMERANG_ESTIMATOR)

px4_add_module(
    MODULE modules__boomerang_estimator
    MAIN boomerang_estimator
    SRCS
        BoomerangEstimator.cpp
        BoomerangEstimator.hpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
)

endif()
