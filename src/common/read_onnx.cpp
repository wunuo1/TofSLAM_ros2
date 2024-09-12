#include "read_onnx.h"

#if defined X3
int prepare_tensor(hbDNNTensor *input_tensor,
                   hbDNNTensor *output_tensor,
                   hbDNNHandle_t dnn_handle) {
  int input_count = 0;
  int output_count = 0;
  hbDNNGetInputCount(&input_count, dnn_handle);
  hbDNNGetOutputCount(&output_count, dnn_handle);

  /** Tips:
   * For input memory size:
   * *   input_memSize = input[i].properties.alignedByteSize
   * For output memory size:
   * *   output_memSize = output[i].properties.alignedByteSize
   */
  hbDNNTensor *input = input_tensor;
  for (int i = 0; i < input_count; i++) {

    hbDNNGetInputTensorProperties(&input[i].properties, dnn_handle, i);
    int input_memSize = input[i].properties.alignedByteSize;
    hbSysAllocCachedMem(&input[i].sysMem[0], input_memSize);
    /** Tips:
     * For input tensor, aligned shape should always be equal to the real
     * shape of the user's data. If you are going to set your input data with
     * padding, this step is not necessary.
     * */
    input[i].properties.alignedShape = input[i].properties.validShape;

    // Show how to get input name
    const char *input_name;
    hbDNNGetInputName(&input_name, dnn_handle, i);
    std::cout << "input[" << i << "] name is " << input_name << std::endl;
  }

  hbDNNTensor *output = output_tensor;
  for (int i = 0; i < output_count; i++) {
    hbDNNGetOutputTensorProperties(&output[i].properties, dnn_handle, i);
    int output_memSize = output[i].properties.alignedByteSize;
    hbSysAllocCachedMem(&output[i].sysMem[0], output_memSize);

    // Show how to get output name
    const char *output_name;
    hbDNNGetOutputName(&output_name, dnn_handle, i);
    std::cout << "output[" << i << "] name is " << output_name << std::endl;
  }
  return 0;
}

/** You can define read_image_2_tensor_as_other_type to prepare your data **/
int32_t read_image_2_tensor_as_nv12(cv::Mat &image_file,
                                    hbDNNTensor *input_tensor) {
  hbDNNTensor *input = input_tensor;
  hbDNNTensorProperties Properties = input->properties;
  int tensor_id = 0;
  // NCHW , the struct of mobilenetv1_224x224 shape is NCHW
  int input_h = Properties.validShape.dimensionSize[2];
  int input_w = Properties.validShape.dimensionSize[3];

  cv::Mat bgr_mat = image_file;
  if (bgr_mat.empty()) {
    std::cout << "image file not exist!" << std::endl;;
    return -1;
  }
  // resize
  cv::Mat mat;
  mat.create(input_h, input_w, bgr_mat.type());
  cv::resize(bgr_mat, mat, mat.size(), 0, 0);
  // convert to YUV420
  if (input_h % 2 || input_w % 2) {
    std::cout << "input img height and width must aligned by 2!" << std::endl;
    return -1;
  }

  // copy y data
  auto data = input->sysMem[0].virAddr;
  int32_t y_size = input_h * input_w;
  // cv::imwrite("2.png", bgr_mat);
  memcpy(data, bgr_mat.data, y_size);
 // hbSysFlushMem(data, HB_SYS_MEM_CACHE_CLEAN);

  return 0;
}

Eigen::VectorXf inference_x3(cv::Mat img, std::string model_path) {
  Eigen::VectorXf results = Eigen::VectorXf::Zero(4096);

  hbPackedDNNHandle_t packed_dnn_handle;
  hbDNNHandle_t dnn_handle;
  const char **model_name_list;
  // std::string img_name = "/root/project/test_bpu/build/3.png";
  std::string model_name = model_path; //"/root/catkin_ws/src/ct-lio/model/hfnet_x3_2.bin";
  auto modelFileName = model_name.c_str();
  int model_count = 0;
  // Step1: get model handle
  {
    hbDNNInitializeFromFiles(&packed_dnn_handle, &modelFileName, 1);
    hbDNNGetModelNameList(
                         &model_name_list, &model_count, packed_dnn_handle);
    hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]);
  }
  // Show how to get dnn version
  std::cout << "DNN runtime version: " << hbDNNGetVersion() << std::endl;

  std::vector<hbDNNTensor> input_tensors;
  std::vector<hbDNNTensor> output_tensors;
  int input_count = 0;
  int output_count = 0;
  // Step2: prepare input and output tensor
  {
    hbDNNGetInputCount(&input_count, dnn_handle);
    hbDNNGetOutputCount(&output_count, dnn_handle);
    input_tensors.resize(input_count);
    output_tensors.resize(output_count);
    prepare_tensor(input_tensors.data(), output_tensors.data(), dnn_handle);
  }

  // Step3: set input data to input tensor
  {
    // read a single picture for input_tensor[0], for multi_input model, you
    // should set other input data according to model input properties.
    read_image_2_tensor_as_nv12(img, input_tensors.data());
    std::cout << "read image to tensor as nv12 success" << std::endl;
  }

  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *output = output_tensors.data();
  // Step4: run inference
  {
    // make sure memory data is flushed to DDR before inference
    for (int i = 0; i < input_count; i++) {
      hbSysFlushMem(&input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle,
                &output,
                input_tensors.data(),
                dnn_handle,
                &infer_ctrl_param);
    // wait task done
    hbDNNWaitTaskDone(task_handle, 0);
  
  //  for (int i = 0; i < output_count; i++) {
  //    hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
  //  }
  //  hbDNNTensor *input = input_tensors.data(); 
  //  int *shape = input->properties.validShape.dimensionSize;
  //  int tensor_len = shape[0] * shape[1] * shape[2] * shape[3];

   // auto data = reinterpret_cast<float *>(input->sysMem[0].virAddr);
   // for(int i = 0; i < tensor_len; i++) {
   //   std::cout << data[i] << " ";
   // }

 //   std::cout << "shape[0]: " << shape[0] << " shape[1]: " << shape[1] << std::endl;
  }

  // Step5: do postprocess with output data
  // std::vector<Classification> top_k_cls;
  {
    // make sure CPU read data from DDR before using output tensor data
    for (int i = 0; i < output_count; i++) {
      hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    }
   
    int *shape = output->properties.validShape.dimensionSize;
    int tensor_len = shape[0] * shape[1] * shape[2] * shape[3];

    auto data = reinterpret_cast<float *>(output->sysMem[0].virAddr);
    for(int i = 0; i < tensor_len; i++) {
      // std::cout << data[i] << " ";
      results[i] = data[i];
    }
    // std::cout << std::endl;
    // for (int i = 0; i < 4096; i++) {
    //   std::cout << results(i) << " ";
    // }

    std::cout << "shape[0]: " << shape[0] << " shape[1]: " << shape[1] << std::endl;

  //   get_topk_result(output, top_k_cls, FLAGS_top_k);
  //   for (int i = 0; i < FLAGS_top_k; i++) {
  //     VLOG(EXAMPLE_REPORT) << "TOP " << i << " result id: " << top_k_cls[i].id;
  //   }
  }

  // Step6: release resources
  {
    // release task handle
    hbDNNReleaseTask(task_handle);
    // free input mem
    for (int i = 0; i < input_count; i++) {
      hbSysFreeMem(&(input_tensors[i].sysMem[0]));
    }
    // free output mem
    for (int i = 0; i < output_count; i++) {
      hbSysFreeMem(&(output_tensors[i].sysMem[0]));
    }
    // release model
    hbDNNRelease(packed_dnn_handle);
  }

  return results;
}
#endif

// std::pair<std::string, Eigen::VectorXf> inference_img(cv::Mat img) {
Eigen::VectorXf inference_img(cv::Mat img,  std::string model) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    const char* model_path = model.data(); //"/root/catkin_ws/src/ct-lio/model/hfnet.onnx";
    Ort::Session session(env, model_path, session_options);

    std::cout << "============================test line================================" << std::endl;
    clock_t startTime, endTime;
    Ort::AllocatorWithDefaultOptions allocator;
    #if defined ONNX1_9_0
    const char* input_name = session.GetInputName(0, allocator);
    #else
    auto input_name_ptr = session.GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_ptr.get(); 
    #endif                
    std::cout << "input_name:" << input_name << std::endl;
    //获取输出name
    #if defined ONNX1_9_0
    const char* output_name = session.GetOutputName(0, allocator);
    #else
    auto output_name_ptr = session.GetOutputNameAllocated(0, allocator);
    const char* output_name = output_name_ptr.get();
    #endif
    static constexpr const int width_ = 640; //模型input width
    static constexpr const int height_ = 480; //模型input height
    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 3> input_shape_{height_, width_, 3}; //NCHW, 1x3xHxW

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1, 4096}; //模型output shape，此处假设是二维的(1,3)

    std::array<float, width_ * height_ * 3> input_image_{}; //输入图片，HWC
    std::array<float, 4096> results_{}; //模型输出，注意和output_shape_对应

    // std::string imgPath = "../data/1700817635.288808.png";
    // cv::Mat img = cv::imread(imgPath);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());
    output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());
    const char* input_names[] = {input_name}; //输入节点名
    const char* output_names[] = {output_name}; //输出节点名

    //预处理
    cv::Mat img_f32;
    img.convertTo(img_f32, CV_32FC3);//转float
    // std::cout << "----------------------img_f32----------------------------" << std::endl;
    // std::cout << img_f32 << std::endl;

    //BGR2RGB,
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            input_image_[i * img.cols + j + 0] = img_f32.at<cv::Vec3f>(i, j)[2];
            input_image_[i * img.cols + j + 1 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[1];
            input_image_[i * img.cols + j + 2 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[0];
        }
    }
    startTime = clock();
    session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
    endTime = clock();
    std::cout << "The run time is:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    // for(int i = 0; i < 4096; i++) {
    //     std::cout << results_[i] << " ";
    // }
    // std::cout << std::endl;
    // std::vector<float> vec(results_.begin(), results_.end());
    Eigen::VectorXf pf = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(results_.data(), results_.size());
    // std::string img_name = imgPath.substr(imgPath.rfind('/') + 1, 17);
    // std::string img_name = "tmp_name";
    // std::pair<std::string, Eigen::VectorXf> p = std::make_pair(img_name, pf);
    // std::cout << imgPath.substr(imgPath.rfind('/') + 1, 17) << std::endl;
    return pf;
}

Eigen::VectorXf inference_img(cv::Mat img, std::shared_ptr<Ort::Session>& session) {
    std::cout << "============================test line================================" << std::endl;
    clock_t startTime, endTime;
    Ort::AllocatorWithDefaultOptions allocator;
    #if defined ONNX1_9_0
    const char* input_name = session->GetInputName(0, allocator);    
    #else
    auto input_name_ptr = session->GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_ptr.get();       
    #endif
    std::cout << "input_name:" << input_name << std::endl;
    //获取输出name
    #if defined ONNX1_9_0
    const char* output_name = session->GetOutputName(0, allocator);
    #else
    auto output_name_ptr = session->GetOutputNameAllocated(0, allocator);
    const char* output_name = output_name_ptr.get();
    #endif
    static constexpr const int width_ = 640; //模型input width
    static constexpr const int height_ = 480; //模型input height
    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 3> input_shape_{height_, width_, 3}; //NCHW, 1x3xHxW

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1, 4096}; //模型output shape，此处假设是二维的(1,3)

    std::array<float, width_ * height_ * 3> input_image_{}; //输入图片，HWC
    std::array<float, 4096> results_{}; //模型输出，注意和output_shape_对应

    // std::string imgPath = "../data/1700817635.288808.png";
    // cv::Mat img = cv::imread(imgPath);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());
    output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());
    const char* input_names[] = {input_name}; //输入节点名
    const char* output_names[] = {output_name}; //输出节点名

    //预处理
    cv::Mat img_f32;
    img.convertTo(img_f32, CV_32FC3);//转float
    // std::cout << "----------------------img_f32----------------------------" << std::endl;
    // std::cout << img_f32 << std::endl;

    //BGR2RGB,
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            input_image_[i * img.cols + j + 0] = img_f32.at<cv::Vec3f>(i, j)[2];
            input_image_[i * img.cols + j + 1 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[1];
            input_image_[i * img.cols + j + 2 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[0];
        }
    }
    startTime = clock();
    session->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
    endTime = clock();
    std::cout << "The run time is:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    Eigen::VectorXf pf = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(results_.data(), results_.size());
    return pf;
}


std::pair<std::string, Eigen::VectorXf> inference_img(std::string imgPath) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    const char* model_path = "/root/catkin_ws/src/ct-lio/model/hfnet.onnx";
    Ort::Session session(env, model_path, session_options);

    std::cout << "============================test line================================" << std::endl;
    std::cout << "inside session2_: " << session << std::endl;
    clock_t startTime, endTime;
    Ort::AllocatorWithDefaultOptions allocator;
    #if defined ONNX1_9_0
    const char* input_name = session.GetInputName(0, allocator);  
    #else   
    auto input_name_ptr = session.GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_ptr.get();         
    #endif    
    std::cout << "input_name:" << input_name << std::endl;
    //获取输出name
    #if defined ONNX1_9_0
    const char* output_name = session.GetOutputName(0, allocator);
    #else
    auto output_name_ptr = session.GetOutputNameAllocated(0, allocator);
    const char* output_name = output_name_ptr.get();
    #endif
    static constexpr const int width_ = 640; //模型input width
    static constexpr const int height_ = 480; //模型input height
    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 3> input_shape_{height_, width_, 3}; //NCHW, 1x3xHxW

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 2> output_shape_{1, 4096}; //模型output shape，此处假设是二维的(1,3)

    std::array<float, width_ * height_ * 3> input_image_{}; //输入图片，HWC
    std::array<float, 4096> results_{}; //模型输出，注意和output_shape_对应

    // std::string imgPath = "../data/1700817635.288808.png";
    cv::Mat img = cv::imread(imgPath);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_image_.data(), input_image_.size(), input_shape_.data(), input_shape_.size());
    output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());
    const char* input_names[] = {input_name}; //输入节点名
    const char* output_names[] = {output_name}; //输出节点名

    //预处理
    cv::Mat img_f32;
    img.convertTo(img_f32, CV_32FC3);//转float
    // std::cout << "----------------------img_f32----------------------------" << std::endl;
    // std::cout << img_f32 << std::endl;

    //BGR2RGB,
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            input_image_[i * img.cols + j + 0] = img_f32.at<cv::Vec3f>(i, j)[2];
            input_image_[i * img.cols + j + 1 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[1];
            input_image_[i * img.cols + j + 2 * img.cols * img.rows] = img_f32.at<cv::Vec3f>(i, j)[0];
        }
    }
    startTime = clock();
    session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
    endTime = clock();
    std::cout << "The run time is:" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    // for(int i = 0; i < 4096; i++) {
    //     std::cout << results_[i] << " ";
    // }
    // std::cout << std::endl;
    // std::vector<float> vec(results_.begin(), results_.end());
    Eigen::VectorXf pf = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(results_.data(), results_.size());
    std::string img_name = imgPath.substr(imgPath.rfind('/') + 1, 17);
    // std::string img_name = "tmp_name";
    std::pair<std::string, Eigen::VectorXf> p = std::make_pair(img_name, pf);
    // std::cout << imgPath.substr(imgPath.rfind('/') + 1, 17) << std::endl;
    return p;
}

double global_match(Eigen::VectorXf global_desc1, Eigen::VectorXf global_desc2) {
    Eigen::MatrixXd m;
    double result = 2.0 * (1.0 - (global_desc1.transpose() * global_desc2));
    return result;
}

bool cmp(const std::pair<std::string, double>& a, const std::pair<std::string, double>& b) {
    return a.second <= b.second;
}

// std::vector<std::string> NNMatch(std::vector<std::pair<std::string, Eigen::VectorXf>> db, std::string img_query, Ort::Session& session) {
//     std::pair<std::string, Eigen::VectorXf> query = inference_img(img_query, session);
//     std::vector<std::pair<std::string, double>> v;
//     for(int i = 0; i < db.size(); i++) {
//         double result = global_match(query.second, db[i].second);
//         std::pair<std::string, double> p = std::make_pair(db[i].first, result);
//         v.push_back(p);
//     }
//     std::sort(v.begin(), v.end(), cmp);
//     std::vector<std::string> result;
//     for (int i = 0; i < 10; i++) {
//         result.push_back(v[i].first);
//     }
//     return result;
// }
