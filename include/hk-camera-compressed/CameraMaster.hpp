#include <deque>
#include <fstream>
#include <thread>

#include "JPEGEncoder.hpp"
#include "MvCameraControl.h"
#include "hk-utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// Depends on nRet and camName
#define CHECK_MV_OR(call, action)                               \
  {                                                             \
    nRet = (call);                                              \
    if (nRet != MV_OK) {                                        \
      printf("Cam[%s]: " #call " failed[%x]\n", camName, nRet); \
      action;                                                   \
    }                                                           \
  }

#define CHECK_MV(call) CHECK_MV_OR(call, )
#define CHECK_MV_RETURN(call) CHECK_MV_OR(call, return nRet)

struct MVFrameGPU {
  void *buf;
  MV_FRAME_OUT_INFO_EX info;
  bool used;
};

class DeviceMemoryQueue {
  std::vector<MVFrameGPU> frames;
  std::deque<unsigned> queue;
  std::mutex lock;
  std::condition_variable cond;
  size_t size;
  unsigned received;
  unsigned dropped;
  unsigned last_num = 0;

 public:
  DeviceMemoryQueue(unsigned len, size_t size_)
      : frames(len), size(size_), received(0), dropped(0) {
    for (unsigned i = 0; i < len; i++) {
      CHECK_CUDA(cudaMalloc(&frames[i].buf, size));
      frames[i].used = false;
    }
  }

  ~DeviceMemoryQueue() {
    for (unsigned i = 0; i < frames.size(); i++) {
      CHECK_CUDA(cudaFree(frames[i].buf));
    }
  }

  DeviceMemoryQueue(const DeviceMemoryQueue &) = delete;
  DeviceMemoryQueue &operator=(const DeviceMemoryQueue &) = delete;

  bool enqueue(const MV_FRAME_OUT &frame) {
    if (last_num && last_num != frame.stFrameInfo.nFrameNum - 1) {
      std::cout << frame.stFrameInfo.nFrameNum - 1 - last_num
                << " Frames not processed\n";
    }
    last_num = frame.stFrameInfo.nFrameNum;

    assert(frame.stFrameInfo.nFrameLen == size);
    // Always prefer the lowest slot
    unsigned i;
    bool drop;
    received++;
    std::lock_guard l(lock);
    auto it = std::find_if(frames.begin(), frames.end(),
                           [](const MVFrameGPU &f) { return !f.used; });
    drop = (it == frames.end());
    if (drop) {
      dropped++;
      std::cout << "Dropped one frame " << drop << "/" << received << std::endl;
      assert(queue.size() <= frames.size());
      drop = true;
      i = queue.back();
      queue.pop_back();
    } else {
      i = std::distance(frames.begin(), it);
    }
    queue.push_front(i);
    cond.notify_one();
    CHECK_CUDA(cudaMemcpy(frames[i].buf, frame.pBufAddr, size,
                          cudaMemcpyHostToDevice));
    frames[i].info = frame.stFrameInfo;
    frames[i].used = true;
    return drop;
  }

  MVFrameGPU *dequeue(unsigned miliseconds) {
    std::unique_lock l(lock);
    bool res = cond.wait_for(l, std::chrono::milliseconds(miliseconds),
                             [this]() { return !queue.empty(); });
    if (res) {
      unsigned i = queue.back();
      queue.pop_back();
      return &frames[i];
    } else {
      return nullptr;
    }
  }

  void finish(MVFrameGPU *frame) {
    std::lock_guard l(lock);
    frame->used = false;
  }
};

class FrameWorker {
  JPEGEncoder encoder;
  unsigned int nDataSize;
  std::string camName;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
  std::unique_ptr<DeviceMemoryQueue> queue;
  unsigned int last_frame_num = 0;
  unsigned int lost_frame_count = 0;
  unsigned int total_frame_count = 0;
  static inline std::mutex creation_mutex;
  FrameWorker(cudaStream_t s, unsigned nDataSize_, const char *camName_,
              const rclcpp::Publisher<CompressedImage>::SharedPtr &pub,
              unsigned buffer_num)
      : encoder(s), nDataSize(nDataSize_), camName(camName_), publisher(pub) {
    queue = std::make_unique<DeviceMemoryQueue>(buffer_num, nDataSize_);
  }

 public:
  static FrameWorker create(void *handle, const char *camName,
                            unsigned buffer_num, rclcpp::Node *node) {
    cudaStream_t s;
    CHECK_CUDA(cudaStreamCreate(&s));

    // ch:获取数据包大小 | en:Get payload size
    int nRet = MV_OK;
    MVCC_INTVALUE nDataSize = {};
    CHECK_MV(MV_CC_GetIntValue(handle, "PayloadSize", &nDataSize));
    if (MV_OK == nRet) {
      printf("Cam[%s]: Payload size[%u]\n", camName, nDataSize.nCurValue);
    }

    // Other wise FastDDS will complain:
    // [PARTICIPANT Error] Type with the same name already exists:
    // sensor_msgs::msg::dds_::CompressedImage_ -> Function registerType
    std::lock_guard l(creation_mutex);
    using namespace std::literals::string_literals;
    auto pub = node->create_publisher<CompressedImage>("~/image/"s + camName,
                                                       buffer_num);
    return {s, nDataSize.nCurValue, camName, pub, buffer_num};
  }

  void receive(const MV_FRAME_OUT &frame) { queue->enqueue(frame); }
  MVFrameGPU *poll(unsigned miliseconds) { return queue->dequeue(miliseconds); }

  void run(MVFrameGPU *frame) {
    MV_FRAME_OUT_INFO_EX &stImageInfo = frame->info;
    const char *camName = this->camName.c_str();
    // uint64_t timestamp = (uint64_t)stImageInfo.nDevTimeStampHigh << 32 |
    //                      stImageInfo.nDevTimeStampLow;
    // printf("Cam [%s]: GetOneFrame, Width[%d], Height[%d], "
    //        "nFrameNum[%d], timestamp[%ld], HostTimeStamp[%ld]\n",
    //        camName, stImageInfo.nWidth, stImageInfo.nHeight,
    //        stImageInfo.nFrameNum, timestamp, stImageInfo.nHostTimeStamp);

    cv::cuda::GpuMat image(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1,
                           frame->buf);
    uint8_t *out;
    size_t len;
    int nRet = MV_OK;
    nRet = encoder.encode_bayer(image, &out, &len);
    if (nRet) {
      printf("Cam[%s]: encode failed[%d]\n", camName, nRet);
      return;
    } else {
      // printf("Cam[%s]: compressed size[%ld], ratio [%f]\n", camName, len,
      //        nDataSize / (double)len);
    }
    // if (total_frame_count % 10 == 0) {
    //   cv::imshow(camName, image);
    //   cv::waitKey(1);
    // }
    queue->finish(frame);

    CompressedImage image_msg;
    image_msg.format = "jpeg";
    image_msg.data = {out, out + len};
    image_msg.header.frame_id = stImageInfo.nFrameNum;
    // FIXME: use correct timestamp
    image_msg.header.stamp.sec = stImageInfo.nHostTimeStamp / 1000;
    image_msg.header.stamp.nanosec =
        stImageInfo.nHostTimeStamp % 1000 * 1000000;
    publisher->publish(image_msg);

    if (last_frame_num == 0) {
      last_frame_num = stImageInfo.nFrameNum;
    } else {
      assert(last_frame_num < stImageInfo.nFrameNum);
      lost_frame_count += stImageInfo.nFrameNum - 1 - last_frame_num;
      total_frame_count += stImageInfo.nFrameNum - last_frame_num;
      if (stImageInfo.nFrameNum != last_frame_num + 1) {
        printf("Cam[%s]: Lost frames %u/%u at frame[%u]\n", camName,
               lost_frame_count, total_frame_count, stImageInfo.nFrameNum);
      }
    }
    last_frame_num = stImageInfo.nFrameNum;
  }
};

class CameraMaster {
  std::atomic_bool running;
  std::vector<void *> handles;
  std::vector<std::thread> work_threads;
  std::vector<std::thread> receive_threads;
  std::vector<FrameWorker> frame_workers;
  // 缓冲区大小为帧率*buffer_time+1
  const float buffer_time = 0.05;
  rclcpp::Node::SharedPtr node;

 public:
  // Should not need this
  // ~CameraMaster() { stop(); }

  int init() {
    node = rclcpp::Node::make_shared("hk_cameras");

    int nRet = MV_OK;
    // ch:初始化SDK | en:Initialize SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
      printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
      return -1;
    }
    MV_CC_DEVICE_INFO_LIST stDeviceList = {};
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
      return -1;
    }
    if (stDeviceList.nDeviceNum > 0) {
      for (unsigned i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo) {
          break;
        }
        PrintDeviceInfo(pDeviceInfo);
      }
    } else {
      printf("Find No Devices!\n");
      return -1;
    }

    // 提示为多相机测试
    // Tips for multicamera testing
    printf("Start %d camera Grabbing Image test\n", stDeviceList.nDeviceNum);
    handles.clear();
    for (unsigned i = 0; i < stDeviceList.nDeviceNum; i++) {
      const char *camName = GetCameraName(stDeviceList.pDeviceInfo[i]);
      void *handle;
      CHECK_MV_RETURN(MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]));
      // 打开设备
      // open device
      // FIXME: the handle is leaked
      CHECK_MV_OR(MV_CC_OpenDevice(handle), {
        nRet = MV_OK;
        continue;
      });

      // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
      // package size(It only works for the GigE camera)
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0) {
          CHECK_MV(MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize));
        } else {
          printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
        }
      }

      // 设置触发模式为off
      // set trigger mode as off
      CHECK_MV(MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF));
      config_camera(handle, camName);
      handles.push_back(handle);
    }
    return nRet;
  }

  // 开启抓取线程
  int start_grabbing() {
    int nRet;
    running.store(true);
    receive_threads.clear();
    work_threads.clear();
    frame_workers.clear();
    frame_workers.reserve(handles.size());
    for (unsigned i = 0; i < handles.size(); i++) {
      MVCC_STRINGVALUE camName_;
      const char *camName = camName_.chCurValue;
      nRet = MV_CC_GetStringValue(handles[i], "DeviceUserID", &camName_);
      if (MV_OK != nRet) {
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
        nRet =
            snprintf(camName_.chCurValue, sizeof(camName_.chCurValue), "%d", i);
        assert(nRet > 0 && nRet < (int)sizeof(camName_.chCurValue) - 1);
      }

      // 计算需要的缓存帧数
      MVCC_FLOATVALUE frame_rate;
      CHECK_MV_RETURN(
          MV_CC_GetFloatValue(handles[i], "ResultingFrameRate", &frame_rate));
      printf("Cam[%s]: Resulting Frame Rate: %f\n", camName,
             frame_rate.fCurValue);

      int buffer_num = 1 + frame_rate.fCurValue * buffer_time;
      CHECK_MV_RETURN(MV_CC_SetImageNodeNum(handles[i], 1));
      // 开始取流
      // start grab image
      CHECK_MV_RETURN(MV_CC_StartGrabbing(handles[i]));

      frame_workers.emplace_back(
          FrameWorker::create(handles[i], camName, buffer_num, node.get()));
      receive_threads.emplace_back(
          [=]() { ReceiveThread(handles[i], frame_workers[i], camName); });
      work_threads.emplace_back([=]() {
        while (running.load(std::memory_order_relaxed)) {
          if (MVFrameGPU *frame = frame_workers[i].poll(100)) {
            frame_workers[i].run(frame);
          }
        }
      });
    }
    return nRet;
  }

  int stop() {
    int nRet = MV_OK;
    running.store(false, std::memory_order_relaxed);
    for (auto &t : receive_threads) {
      t.join();
    }
    for (auto &t : work_threads) {
      t.join();
    }
    for (void *&handle : handles) {
      if (!handle) continue;
      // 停止取流
      // end grab image
      nRet = MV_CC_StopGrabbing(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        continue;
      }
      // 关闭设备
      // close device
      nRet = MV_CC_CloseDevice(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        continue;
      }
      // 销毁句柄
      // destroy handle
      nRet = MV_CC_DestroyHandle(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        continue;
      }
      handle = NULL;
    }
    printf("All cameras stopped\n");
    return nRet;
  }

  void ReceiveThread(void *handle, FrameWorker &frame_worker,
                     const char *camName) {
    int nRet = MV_OK;
    while (running.load(std::memory_order_relaxed)) {
      MV_FRAME_OUT frame;
      CHECK_MV_OR(MV_CC_GetImageBuffer(handle, &frame, 100), continue);

      frame_worker.receive(frame);

      CHECK_MV(MV_CC_FreeImageBuffer(handle, &frame));

      // unsigned valid_num;
      // nRet = MV_CC_GetValidImageNum(handle, &valid_num);
      // if (nRet != MV_OK) {
      //   printf("Cam[%s]: MV_CC_GetValidImageNum failed![%x]\n", camName,
      //   nRet); continue;
      // }
      // if (valid_num > 1) {
      //   printf("Cam[%s]: %u valid images at frame[%u]\n", camName, valid_num,
      //          stImageInfo.nFrameNum);
      // }
    }
  }

  void config_camera(void *handle, const char *camName) {
    using namespace std::literals::string_literals;
    int nRet = MV_OK;
    std::string feature_file = "configs/"s + camName + ".ini";
    CHECK_MV(MV_CC_FeatureSave(handle, feature_file.c_str()));

    std::vector<uint8_t> xml_buf;
    unsigned xml_len;
    nRet = MV_XML_GetGenICamXML(handle, NULL, 0, &xml_len);
    assert((unsigned)nRet == MV_E_PARAMETER);
    xml_buf.resize(xml_len);
    CHECK_MV(
        MV_XML_GetGenICamXML(handle, xml_buf.data(), xml_buf.size(), &xml_len));
    if (nRet == MV_OK) {
      std::ofstream xml_file("configs/"s + camName + ".xml", std::ios::binary);
      xml_file.write(reinterpret_cast<const char *>(xml_buf.data()), xml_len);
    }

    if ("Fast"s == camName) {
      CHECK_MV(MV_CC_SetEnumValueByString(handle, "ADCBitDepth", "Bits_8"));
      MVCC_ENUMVALUE ADCBitDepth;
      CHECK_MV(MV_CC_GetEnumValue(handle, "ADCBitDepth", &ADCBitDepth));
      if (nRet == MV_OK)
        printf("Cam[%s]: ADCBitDepth: %u\n", camName, ADCBitDepth.nCurValue);
    } else if ("Color"s == camName) {
    } else {
      printf("Warning: unrecognized camera\n");
    }
  }
};