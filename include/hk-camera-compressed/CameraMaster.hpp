#include "JPEGEncoder.hpp"
#include "MvCameraControl.h"
#include "hk-utils.hpp"
#include <fstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

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
    // uint64_t timestamp = (uint64_t)stImageInfo.nDevTimeStampHigh << 32 |
    //                      stImageInfo.nDevTimeStampLow;
    // printf("Cam Serial Number[%s]: GetOneFrame, Width[%d], Height[%d], "
    //        "nFrameNum[%d], timestamp[%ld], HostTimeStamp[%ld]\n",
    //        camName, stImageInfo.nWidth, stImageInfo.nHeight,
    //        stImageInfo.nFrameNum, timestamp, stImageInfo.nHostTimeStamp);

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
    handles.resize(stDeviceList.nDeviceNum);
    for (unsigned i = 0; i < stDeviceList.nDeviceNum; i++) {
      const char *camName = GetCameraName(stDeviceList.pDeviceInfo[i]);
      void *handle;
      nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]);
      if (MV_OK != nRet) {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return nRet;
      }
      // 打开设备
      // open device
      nRet = MV_CC_OpenDevice(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return nRet;
      }

      // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
      // package size(It only works for the GigE camera)
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0) {
          nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
          if (nRet != MV_OK) {
            printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
          }
        } else {
          printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
        }
      }

      // 设置触发模式为off
      // set trigger mode as off
      nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
      if (MV_OK != nRet) {
        printf("Cam[%s]: MV_CC_SetTriggerMode fail! nRet [%x]\n", camName,
               nRet);
      }
      config_camera(handle, camName);
      handles[i] = handle;
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
      nRet = MV_CC_GetFloatValue(handles[i], "ResultingFrameRate", &frame_rate);
      printf("Cam[%s]: Resulting Frame Rate: %f\n", camName,
             frame_rate.fCurValue);
      if (MV_OK != nRet) {
        printf("Cam[%s]: Get ResultingFrameRate fail! nRet [%x]\n", camName,
               nRet);
        return nRet;
      }
      int buffer_num = 1 + frame_rate.fCurValue * buffer_time;
      nRet = MV_CC_SetImageNodeNum(handles[i], buffer_num);
      if (MV_OK != nRet) {
        printf("Cam[%s]: MV_CC_SetImageNodeNum fail! nRet [%x]\n", camName,
               nRet);
        return nRet;
      }
      // 开始取流
      // start grab image
      nRet = MV_CC_StartGrabbing(handles[i]);
      if (MV_OK != nRet) {
        printf("Cam[%s]: MV_CC_StartGrabbing fail! nRet [%x]\n", camName, nRet);
        return nRet;
      }
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
      nRet = MV_CC_GetStringValue(handle, "DeviceUserID", &stStringValue);
      if (MV_OK == nRet) {
        memcpy(camName, stStringValue.chCurValue,
               sizeof(stStringValue.chCurValue));
      } else {
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
      }
    }
    // ch:获取数据包大小 | en:Get payload size
    unsigned int nDataSize;
    {
      MVCC_INTVALUE stParam = {};
      nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
      if (MV_OK != nRet) {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
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
      nRet = MV_CC_GetImageBuffer(handle, &frame, 100);
      if (nRet != MV_OK) {
        printf("Cam[%s]: Get One Frame failed![%x]\n", camName, nRet);
        continue;
      }

      frame_worker.run(frame.pBufAddr, stImageInfo);

      MV_CC_FreeImageBuffer(handle, &frame);

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
    nRet = MV_CC_FeatureSave(handle, feature_file.c_str());
    if (nRet != MV_OK)
      printf("Cam[%s]: MV_CC_FeatureSave Failed: %x\n", camName, nRet);

    std::vector<uint8_t> xml_buf;
    unsigned xml_len;
    MV_XML_GetGenICamXML(handle, NULL, 0, &xml_len);
    xml_buf.resize(xml_len);
    nRet =
        MV_XML_GetGenICamXML(handle, xml_buf.data(), xml_buf.size(), &xml_len);
    if (nRet != MV_OK)
      printf("Cam[%s]: MV_XML_GetGenICamXML Failed: %x\n", camName, nRet);
    else {
      std::ofstream xml_file("configs/"s + camName + ".xml", std::ios::binary);
      xml_file.write(reinterpret_cast<const char *>(xml_buf.data()), xml_len);
    }

    if ("Fast"s == camName) {
      nRet = MV_CC_SetEnumValueByString(handle, "ADCBitDepth", "Bits_8");
      if (nRet != MV_OK) {
        printf("Cam[%s]: Set ADCBitDepth Failed: %x\n", camName, nRet);
      }
      MVCC_ENUMVALUE ADCBitDepth;
      nRet = MV_CC_GetEnumValue(handle, "ADCBitDepth", &ADCBitDepth);
      if (nRet != MV_OK) {
        printf("Cam[%s]: Get ADCBitDepth Failed: %x\n", camName, nRet);
      } else {
        printf("Cam[%s]: ADCBitDepth: %u\n", camName, ADCBitDepth.nCurValue);
      }
    } else if ("Color"s == camName) {

    } else {
      printf("Warning: unrecognized camera\n");
    }
  }
};