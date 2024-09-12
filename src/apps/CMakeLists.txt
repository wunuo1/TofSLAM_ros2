add_executable(${PROJECT_NAME}_eskf main_eskf.cpp  
                               common_utility.cpp)

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME}_eskf 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.liw
                         ${PROJECT_NAME}.algo
                         ${PROJECT_NAME}.loop
                         ${third_party_libs}
                         ${CMAKE_THREAD_LIBS_INIT}
                        ${OpenCV_LIBS}
                        yaml-cpp
                        ceres
                        ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/lib/libonnxruntime.so.1.16.0
                        )

# add_executable(${PROJECT_NAME}_eskf_rosbag main_eskf_rosbag.cpp  
#                         common_utility.cpp)
# target_link_libraries(${PROJECT_NAME}_eskf_rosbag 
#                   ${PROJECT_NAME}.common
#                   ${PROJECT_NAME}.pre
#                   ${PROJECT_NAME}.liw
#                   ${PROJECT_NAME}.algo
#                   ${PROJECT_NAME}.tools
#                   ${PROJECT_NAME}.loop
#                   ${third_party_libs}
#                  )


link_directories(${G2O_LIBRARY_DIRS})

#add_executable(test_analytic_factor test_analytic_factor.cpp )
#target_link_libraries(test_analytic_factor
#            ${PROJECT_NAME}.common
#            ${PROJECT_NAME}.pre
#            ${PROJECT_NAME}.liw
#            ${PROJECT_NAME}.algo
#            ${third_party_libs}
#        )
ament_target_dependencies(
    ${PROJECT_NAME}_eskf
    rclcpp
    cv_bridge
    nav_msgs
)

install(TARGETS ${PROJECT_NAME}_eskf
  DESTINATION lib/${PROJECT_NAME})

