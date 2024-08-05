#include <fstream>
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
  void run(const MV_FRAME_OUT_INFO_EX &stImageInfo, const char *camName) {
    uint64_t dev_timestamp = (uint64_t)stImageInfo.nDevTimeStampHigh << 32 |
                             stImageInfo.nDevTimeStampLow;
    auto duration =
        std::chrono::high_resolution_clock::now().time_since_epoch();
    uint64_t cur_timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    if (!start_time) {
      start_time = cur_timestamp;
    }
    if (!device_time_offset && cur_timestamp > start_time + 500000000) {
      // cur_timestamp = cur_timestamp / 1000000 * 1000000;
      device_time_offset = dev_timestamp * 10 - cur_timestamp;
      start_time = cur_timestamp;
      start_frame = stImageInfo.nFrameNum;
    }
    double dev_time =
        (dev_timestamp * 10 - device_time_offset - start_time) / 1000000.;
    double host_time =
        (stImageInfo.nHostTimeStamp * 1000000 - start_time) / 1000000.;
    double cur_time = (cur_timestamp - start_time) / 1000000.;
    double interval = (dev_time - last_frame_time) /
                      (stImageInfo.nFrameNum - last_near_frame);
    if (device_time_offset && !isnan(last_host_time) &&
        cur_time - host_time < 10000) {
      double diff = dev_time - cur_time - pid_delta;
      // friction to limit i term
      pid_integral += (diff - pid_integral / 100) * (cur_time - last_host_time);
      pid_delta += diff / 1000 + pid_integral / 300000;
    }
    last_frame_time = dev_time;
    last_host_time = cur_time;
    double ppm = pid_delta / cur_time * 1000000.;
    printf("Cam[%s]: Device[%13.5f](%+4.1f, %+.5f = "
           "%+.3f%+.3f@%+05.1f, "
           "%+05.1fppm, %.5f), "
           "Host[%7.0f](%+.1f), "
           "Current[%14.6f], [%u][%u]\n",
           camName, dev_time, dev_time - host_time, dev_time - cur_time,
           pid_delta, dev_time - cur_time - pid_delta, pid_integral, ppm,
           interval, host_time, host_time - cur_time, cur_time,
           stImageInfo.nFrameNum, stImageInfo.nFrameNum - last_near_frame);
    last_near_frame = stImageInfo.nFrameNum;
  }
};

class FrameWorker {
  JPEGEncoder encoder;
  unsigned int nDataSize;
  const char *camName;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  rclcpp::Publisher<CompressedImage>::SharedPtr publisher;
  unsigned int last_frame_num = 0;
  unsigned int lost_frame_count = 0;
  unsigned int total_frame_count = 0;
  static inline std::mutex creation_mutex;
  TimerEvaluator timer_evaluator;
  FrameWorker(cudaStream_t s, unsigned int nDataSize_, const char *camName_,
              const rclcpp::Publisher<CompressedImage>::SharedPtr &pub)
      : encoder(s), nDataSize(nDataSize_), camName(camName_), publisher(pub) {}

public:
  static FrameWorker create(unsigned int nDataSize, const char *camName,
                            int buffer_num, rclcpp::Node *node) {
    cudaStream_t s;
    CHECK_CUDA(cudaStreamCreate(&s));
    using namespace std::literals::string_literals;
    // Other wise FastDDS will complain:
    // [PARTICIPANT Error] Type with the same name already exists:
    // sensor_msgs::msg::dds_::CompressedImage_ -> Function registerType
    std::lock_guard l(creation_mutex);
    auto pub = node->create_publisher<CompressedImage>("~/image/"s + camName,
                                                       buffer_num);
    return {s, nDataSize, camName, pub};
  }

  void run(uint8_t *image_data, const MV_FRAME_OUT_INFO_EX &stImageInfo) {
    // printf("Cam[%s]: GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
    //        camName, stImageInfo.nWidth, stImageInfo.nHeight,
    //        stImageInfo.nFrameNum);
    if (!strcmp(camName, "Color"))
      timer_evaluator.run(stImageInfo, camName);

    cv::Mat image(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, image_data);
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

    CompressedImage image_msg;
    image_msg.format = "jpeg";
    image_msg.data = {out, out + len};
    image_msg.header.frame_id = stImageInfo.nFrameNum;
    // FIXME: use correct timestamp
    image_msg.header.stamp.sec = stImageInfo.nHostTimeStamp / 1000000;
    image_msg.header.stamp.nanosec =
        stImageInfo.nHostTimeStamp % 1000 * 1000000;
    publisher->publish(image_msg);

    if (last_frame_num == 0) {
      last_frame_num = stImageInfo.nFrameNum;
    } else {
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
  std::vector<std::thread> threads;
  // 缓冲区大小为帧率*buffer_time+1
  const float buffer_time = 0.01;
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
    for (unsigned i = 0; i < stDeviceList.nDeviceNum; i++) {
      const char *camName = GetCameraName(stDeviceList.pDeviceInfo[i]);
      // if (strcmp(camName, "Color")) continue;
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
    threads.resize(handles.size());
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
      CHECK_MV_RETURN(MV_CC_SetImageNodeNum(handles[i], buffer_num));
      // 开始取流
      // start grab image
      CHECK_MV_RETURN(MV_CC_StartGrabbing(handles[i]));
      threads[i] = std::thread(
          [this, i, buffer_num]() { WorkThread(handles[i], buffer_num); });
    }
    return nRet;
  }

  int stop() {
    int nRet = MV_OK;
    running.store(false, std::memory_order_relaxed);
    for (auto &t : threads) {
      t.join();
    }
    threads.clear();
    for (void *&handle : handles) {
      if (!handle)
        continue;
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

  void WorkThread(void *handle, int buffer_num) {
    int nRet = MV_OK;
    char camName[256];
    {
      MVCC_STRINGVALUE stStringValue = {};
      CHECK_MV(MV_CC_GetStringValue(handle, "DeviceUserID", &stStringValue));
      if (MV_OK == nRet) {
        memcpy(camName, stStringValue.chCurValue,
               sizeof(stStringValue.chCurValue));
      }
    }
    // ch:获取数据包大小 | en:Get payload size
    unsigned int nDataSize;
    {
      MVCC_INTVALUE stParam = {};
      CHECK_MV(MV_CC_GetIntValue(handle, "PayloadSize", &stParam));
      if (MV_OK != nRet) {
        return;
      } else {
        printf("Cam[%s]: Payload size[%u]\n", camName, stParam.nCurValue);
      }
      nDataSize = stParam.nCurValue;
    }
    // std::vector<uint8_t> buf(nDataSize);
    // MV_FRAME_OUT_INFO_EX stImageInfo = {};

    // cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    FrameWorker frame_worker =
        FrameWorker::create(nDataSize, camName, buffer_num, node.get());
    while (running.load(std::memory_order_relaxed)) {
      // nRet = MV_CC_GetOneFrameTimeout(handle, buf.data(), nDataSize,
      //                                 &stImageInfo, 100);
      MV_FRAME_OUT frame;
      MV_FRAME_OUT_INFO_EX &stImageInfo = frame.stFrameInfo;
      CHECK_MV_OR(MV_CC_GetImageBuffer(handle, &frame, 100), continue);
      // unsigned valid_num;
      // CHECK_MV(MV_CC_GetValidImageNum(handle, &valid_num));

      frame_worker.run(frame.pBufAddr, stImageInfo);

      CHECK_MV(MV_CC_FreeImageBuffer(handle, &frame));

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