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

class TriggerManager;

class FrameWorker {
  JPEGEncoder encoder;
  TriggerManager &trigger_manager;
  unsigned int width;
  unsigned int height;
  unsigned int pixel_format;
  unsigned int trigger_step;
  double frame_rate;
  const char *camName;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
  unsigned int last_frame_num = UINT_MAX;
  unsigned int lost_frame_count = 0;
  unsigned int total_frame_count = 0;
  static inline std::mutex creation_mutex;
  TimerEvaluator timer_evaluator;
  FrameWorker(cudaStream_t s, void *handle, TriggerManager &tm,
              const char *camName_,
              const rclcpp::Publisher<CompressedImage>::SharedPtr &pub);

public:
  static FrameWorker create(void *handle, TriggerManager &tm,
                            const char *camName, int buffer_num,
                            rclcpp::Node *node) {
    assert(strlen(camName));
    cudaStream_t s;
    CHECK_CUDA(cudaStreamCreate(&s));
    using namespace std::literals::string_literals;
    // Other wise FastDDS will complain:
    // [PARTICIPANT Error] Type with the same name already exists:
    // sensor_msgs::msg::dds_::CompressedImage_ -> Function registerType
    std::lock_guard l(creation_mutex);
    auto pub = node->create_publisher<CompressedImage>("~/image/"s + camName,
                                                       buffer_num);
    return {s, handle, tm, camName, pub};
  }

  void run(uint8_t *image_data, const MV_FRAME_OUT_INFO_EX &stImageInfo);
};

class TriggerManager {
  float trigger_rate;
  std::mutex lock;
  std::condition_variable cond;
  std::deque<uint64_t> fifo;
  unsigned first_frame;
  unsigned buffer_size;

public:
  void init(float trigger_rate_, float buffer_time) {
    trigger_rate = trigger_rate_;
    buffer_size = 1 + trigger_rate * buffer_time;
    first_frame = 0;
  }

  unsigned get_buffer_size() const { return buffer_size; }

  // Get the step to be used in `get_timestamp`
  unsigned get_frame_rate(const char *name, float frame_rate) const {
    unsigned trigger_step = trigger_rate / frame_rate;
    if (trigger_step * frame_rate != trigger_rate) {
      printf("Cam[%s]: Frame rate %f does not divide trigger rate %f\n", name,
             frame_rate, trigger_rate);
    }
    return trigger_step;
  }

  // Thread-safe. 0 if no external timestamp.
  template <typename _Rep, typename _Period>
  uint64_t get_timestamp(const char *name, unsigned frame_num,
                         unsigned trigger_step,
                         const std::chrono::duration<_Rep, _Period> &duration) {
    if (!trigger_step)
      return 0;

    std::unique_lock l(lock);
    if (frame_num < first_frame) {
      printf("Cam[%s]: Timestamp for frame %u has already been dropped.\n",
             name, frame_num);
      return 0;
    } else if (cond.wait_for(l, duration, [this, frame_num] {
                 return frame_num < first_frame + fifo.size();
               })) {
      return fifo[frame_num - first_frame];
    } else {
      printf("Cam[%s]: Timestamp for frame %u has not yet arrived.\n", name,
             frame_num);
      return 0;
    }
  }

  // Thread-safe.
  bool add_timestamp(unsigned frame_num, uint64_t timestamp) {
    std::lock_guard l(lock);
    int skip_num = frame_num - first_frame - fifo.size();
    if (skip_num < 0) {
      printf("Timestamp for frame %u arrived after frame %lu\n", frame_num,
             first_frame + fifo.size() - 1);
      return false;
    }
    if ((unsigned)skip_num > buffer_size) {
      fifo.resize(0);
      fifo.push_back(timestamp);
      first_frame = frame_num;
    } else {
      while (skip_num--) {
        printf("Timestamp for frame %lu did not arrive\n",
               first_frame + fifo.size());
        fifo.push_back(0);
      }
      while (fifo.size() > buffer_size) {
        first_frame++;
        fifo.pop_front();
      }
      fifo.push_back(timestamp);
    }

    assert(frame_num == first_frame + fifo.size() - 1);
    cond.notify_all();
    return true;
  }
};

class CameraMaster {
  std::atomic_bool running;
  std::vector<void *> handles;
  std::vector<std::thread> threads;
  // 缓冲区大小为帧率*buffer_time+1
  const float buffer_time = 0.01;
  rclcpp::Node::SharedPtr node;
  TriggerManager trigger_manager;

public:
  // Should not need this
  // ~CameraMaster() { stop(); }

  int init(float trigger_rate);
  // 开启抓取线程
  int start_grabbing();
  int stop();
  void WorkThread(void *handle, int buffer_num);
  void config_camera(void *handle, const char *camName);
};