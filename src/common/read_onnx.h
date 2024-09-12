#ifndef __READ_ONNX_H__
#define __READ_ONNX_H__
#define X3
//#define ONNX1_9_0

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/CXX11/Tensor>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <onnxruntime_cxx_api.h>

#if defined X3
#include "dnn/hb_dnn.h"
#endif

//#define HB_CHECK_SUCCESS(value, errmsg)                              \
//  do {                                                               \
//    /*value can be call of function*/       
//   /* Eigen::VectorXf ret_code = Eigen::VectorXf::Zero(1);	*/  \
//    auto ret_code = value;                                           \
//    if (ret_code = 0) {                                             \
//      std::cout << errmsg << ", error code:" << ret_code << std::endl; \
//    /*  return ret_code; */                                              \
//    }                                                                \
//  } while (0);
#if defined X3
int prepare_tensor(hbDNNTensor *input_tensor,
                   hbDNNTensor *output_tensor,
                   hbDNNHandle_t dnn_handle);

int32_t read_image_2_tensor_as_nv12(cv::Mat &image_file,
                                    hbDNNTensor *input_tensor);

Eigen::VectorXf inference_x3(cv::Mat img, std::string model_path);
#endif

// Ort::Session& get_session() {
//     Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
//     Ort::SessionOptions session_options;
//     session_options.SetIntraOpNumThreads(1);
//     // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
//     session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
//     const char* model_path = "../model/hfnet.onnx";
//     Ort::Session session(env, model_path, session_options);
//     return session;
// }

Eigen::VectorXf inference_img(cv::Mat img, std::string model_path);
Eigen::VectorXf inference_img(cv::Mat img, std::shared_ptr<Ort::Session>& session);
std::pair<std::string, Eigen::VectorXf> inference_img(std::string imgPath);

double global_match(Eigen::VectorXf global_desc1, Eigen::VectorXf global_desc2);

bool cmp(const std::pair<std::string, double>& a, const std::pair<std::string, double>& b);

// std::vector<std::string> NNMatch(std::vector<std::pair<std::string, Eigen::VectorXf>> db, std::string img_query, Ort::Session& session);

#endif
