cmake_minimum_required(VERSION 3.5)
project(ct_lio)

MESSAGE(STATUS "[Release Mode]")
set(CMAKE_BUILD_TYPE "Release")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#set(DNN_PATH "/usr/")
#set(APPSDK_PATH "/usr/")

#set(DNN_LIB_PATH ${DNN_PATH}/lib)
#set(APPSDK_LIB_PATH ${APPSDK_PATH}/lib/hbbpu)
#set(BPU_libs dnn cnn_intf hbrt_bernoulli_aarch64)

#include_directories(${DNN_PATH}/include
#                    ${APPSDK_PATH}/include)
#link_directories(${DNN_LIB_PATH}
#                ${APPSDK_PATH}/lib/hbbpu
#                ${APPSDK_PATH}/lib)

set(CMAKE_MODULE_PATH "/root/g2o/g2o/cmake_modules" ${CMAKE_MODULE_PATH})

find_package(G2O REQUIRED)
#find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp  REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
#find_package(eigen_conversions REQUIRED)
#find_package(message_generation REQUIRED)
find_package(Boost REQUIRED)
#find_package(glog REQUIRED)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/wunuo/g2o/g2o/cmake_modules)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#target_link_libraries(main_eskf ${OpenCV_LIBS} ${Boost_LIBRARIES} glog gflags Eigen3::Eigen3 )


ament_export_dependencies(${OpenCV_LIBS} ${Boost_LIBRARIES} glog gflags Eigen3::Eigen3 rclcpp nav_msgs cv_bridge g2o)

include_directories(
  include
  /opt/ros/foxy/include
  /usr/include/eigen3
  /usr/include/pcl-1.10
  /usr/include/vtk-7.1
)

set(DNN_PATH "/usr/")
set(APPSDK_PATH "/usr/")

set(DNN_LIB_PATH ${DNN_PATH}/lib)
set(APPSDK_LIB_PATH ${APPSDK_PATH}/lib/hbbpu)
set(BPU_libs dnn cnn_intf hbrt_bernoulli_aarch64)

include_directories(${DNN_PATH}/include
                    ${APPSDK_PATH}/include)
link_directories(${DNN_LIB_PATH}
                ${APPSDK_PATH}/lib/hbbpu
                ${APPSDK_PATH}/lib)


include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tessil-src/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty ${Boost_INCLUDE_DIRS})


include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS} /usr/include/opencv4)

link_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/lib)
link_directories(${G2O_LIBRARIES})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})



find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
link_directories(${BOOST_SERIALIZATION})

add_subdirectory(src)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()