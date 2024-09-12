# Tofslam

  注意：
  1. 本代码同时支持PC端与X3端，在read_onnx.h里有宏定义#define X3控制是否使用x3 bpu编译
  2. read_onnx.h里有宏定义#define ONNX1_9_0控制PC端使用onnx1.9.0还是onnx1.16.0；更改onnx版本时同时更改cmake/packages.cmake中的环境路径：set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-gpu-1.9.0")
  3. 所有文件输入/输出路径以及模型路径均在config/mapping.yaml里修改
  4. 使用建图算法或者定位算法可在config/mapping.yaml里修改Localization_mode变量
  5. 若使用rviz可视化：使用建图算法时，关闭LocalizationMap显示；使用定位算法时，关闭mappingMap显示。方便可视化结果更清晰。
