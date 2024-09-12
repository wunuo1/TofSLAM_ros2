
set(DNN_PATH "/usr/")
set(APPSDK_PATH "/usr/")

set(DNN_LIB_PATH ${DNN_PATH}/lib)


include_directories(${DNN_PATH}/include
                    ${APPSDK_PATH}/include)
link_directories(${DNN_LIB_PATH}
                ${APPSDK_PATH}/lib/hbbpu
                ${APPSDK_PATH}/lib)
file(GLOB LIBS "${APPSDK_PATH}/lib/hbbpu/*.so")

message(${APPSDK_PATH}/lib "--------------------------------")

add_library(${PROJECT_NAME}.common
        timer/timer.cc
        utility.cpp
        utils_trans.cpp
        utils.cpp
        global_flags.cc
        read_onnx.cpp
        )

target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        ${LIBS}
        )