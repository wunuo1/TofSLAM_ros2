
add_library(${PROJECT_NAME}.liw
        poseParameterization.cpp
        lidarFactor.cpp
        lio_utils.cpp
        lio/lidarodom.cpp
        )

target_link_libraries(${PROJECT_NAME}.liw
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.algo
        ${third_party_libs}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
        ${G2O_TYPES_SLAM3D}
        )
