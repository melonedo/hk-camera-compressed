#pragma once
#include <thread>

#include "JPEGEncoder.hpp"
#include "MvCameraControl.h"
#include "hk-utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// Depends on nRet and camName
#define CHECK_MV_OR(call, action)                                              \
  {                                                                            \
    nRet = (call);                                                             \
    if (nRet != MV_OK) {                                                       \
      printf("Cam[%s]: " #call " failed[%x]\n", camName, nRet);                \
      action;                                                                  \
    }                                                                          \
  }

#define CHECK_MV(call) CHECK_MV_OR(call, )
#define CHECK_MV_RETURN(call) CHECK_MV_OR(call, return nRet)

class TimerEvaluator {
  uint64_t device_time_offset = 0;
  uint64_t start_time = 0;
  unsigned int start_frame;
  double last_frame_time;
  double last_host_time = std::numeric_limits<double>::quiet_NaN();
  double pid_delta = 0;
  double pid_integral = 0;
  unsigned last_near_frame;

public:
  void run(const MV_FRAME_OUT_INFO_EX &stImageInfo, const char *camName);
};

class CameraMaster;

class FrameWorker {
  JPEGEncoder encoder;
  CameraMaster *master;
  unsigned int width;
  unsigned int height;
  unsigned int format;
  const char *camName;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
  unsigned int last_frame_num = 0;
  unsigned int lost_frame_count = 0;
  unsigned int total_frame_count = 0;
  static inline std::mutex creation_mutex;
  TimerEvaluator timer_evaluator;
  FrameWorker(cudaStream_t s, void *handle, CameraMaster *master_,
              const char *camName_,
              const rclcpp::Publisher<CompressedImage>::SharedPtr &pub)
      : encoder(s), master(master_), camName(camName_), publisher(pub) {
    int nRet;
    MVCC_INTVALUE_EX stIntValue;
    CHECK_MV(MV_CC_GetIntValueEx(handle, "PayloadSize", &stIntValue));
    unsigned dataSize = stIntValue.nCurValue;
    CHECK_MV(MV_CC_GetIntValueEx(handle, "Width", &stIntValue));
    width = stIntValue.nCurValue;
    CHECK_MV(MV_CC_GetIntValueEx(handle, "Height", &stIntValue));
    height = stIntValue.nCurValue;
    assert(dataSize == width * height);

    MVCC_ENUMVALUE stEnumValue;
    CHECK_MV(MV_CC_GetEnumValue(handle, "PixelFormat", &stEnumValue));
    format = stEnumValue.nCurValue;
    assert(format == PixelType_Gvsp_BayerRG8);

    // Warmup
    cv::Mat image(height, width, CV_8UC1);
    uint8_t *out;
    size_t len;
    nRet = encoder.encode_bayer(image, &out, &len);
    if (nRet)
      printf("Cam[%s]: encode failed[%d]\n", camName, nRet);
  }

public:
  static FrameWorker create(void *handle, CameraMaster *master,
                            const char *camName, int buffer_num,
                            rclcpp::Node *node) {
    cudaStream_t s;
    CHECK_CUDA(cudaStreamCreate(&s));
    using namespace std::literals::string_literals;
    // Other wise FastDDS will complain:
    // [PARTICIPANT Error] Type with the same name already exists:
    // sensor_msgs::msg::dds_::CompressedImage_ -> Function registerType
    std::lock_guard l(creation_mutex);
    auto pub = node->create_publisher<CompressedImage>("~/image/"s + camName,
                                                       buffer_num);
    return {s, handle, master, camName, pub};
  }

  void run(uint8_t *image_data, const MV_FRAME_OUT_INFO_EX &stImageInfo);
};

class CameraMaster {
  std::atomic_bool running;
  std::vector<void *> handles;
  std::vector<std::thread> threads;
  // 缓冲区大小为帧率*buffer_time+1
  const float buffer_time = 0.01;
  rclcpp::Node::SharedPtr node;

public:
  // Should not need this
  // ~CameraMaster() { stop(); }

  int init();
  // 开启抓取线程
  int start_grabbing();
  int stop();
  void WorkThread(void *handle, int buffer_num);
  void config_camera(void *handle, const char *camName);
};