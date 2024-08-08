#include "CameraMaster.hpp"
#include "SerialTrigger.hpp"

#include <fstream>

void TimerEvaluator::run(const MV_FRAME_OUT_INFO_EX &stImageInfo,
                         const char *camName) {
  uint64_t dev_timestamp = (uint64_t)stImageInfo.nDevTimeStampHigh << 32 |
                           stImageInfo.nDevTimeStampLow;
  auto duration = std::chrono::high_resolution_clock::now().time_since_epoch();
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
  double interval =
      (dev_time - last_frame_time) / (stImageInfo.nFrameNum - last_near_frame);
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

FrameWorker::FrameWorker(
    cudaStream_t s, void *handle, TriggerManager &tm, const char *camName_,
    const rclcpp::Publisher<CompressedImage>::SharedPtr &pub)
    : encoder(s), trigger_manager(tm), camName(camName_), publisher(pub) {
  int nRet;

  MVCC_INTVALUE_EX stIntValue;
  CHECK_MV(MV_CC_GetIntValueEx(handle, "PayloadSize", &stIntValue));
  unsigned dataSize = stIntValue.nCurValue;
  CHECK_MV(MV_CC_GetIntValueEx(handle, "Width", &stIntValue));
  width = stIntValue.nCurValue;
  CHECK_MV(MV_CC_GetIntValueEx(handle, "Height", &stIntValue));
  height = stIntValue.nCurValue;
  // Expect Bayer or Mono format instead of RGB
  assert(dataSize == width * height);

  MVCC_ENUMVALUE stEnumValue;
  CHECK_MV(MV_CC_GetEnumValue(handle, "PixelFormat", &stEnumValue));
  pixel_format = stEnumValue.nCurValue;
  assert(pixel_format == PixelType_Gvsp_BayerRG8);

  MVCC_FLOATVALUE stFloatValue;
  CHECK_MV(MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &stFloatValue));
  frame_rate = stFloatValue.fCurValue;
  trigger_step = trigger_manager.get_frame_rate(camName, frame_rate);

  // Warmup
  cv::Mat image(height, width, CV_8UC1);
  uint8_t *out;
  size_t len;
  nRet = encoder.encode_bayer(image, &out, &len);
  if (nRet)
    printf("Cam[%s]: encode failed[%d]\n", camName, nRet);
}

void FrameWorker::run(uint8_t *image_data,
                      const MV_FRAME_OUT_INFO_EX &stImageInfo) {
  printf("Cam[%s]: GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
         camName, stImageInfo.nWidth, stImageInfo.nHeight,
         stImageInfo.nFrameNum);
  //   if (!strcmp(camName, "Color"))
  //     timer_evaluator.run(stImageInfo, camName);

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
  if (last_frame_num == UINT_MAX) {
    last_frame_num = stImageInfo.nFrameNum;
    printf("Cam[%s]: First received frame is #%d\n", camName, last_frame_num);
  } else {
    lost_frame_count += stImageInfo.nFrameNum - 1 - last_frame_num;
    total_frame_count += stImageInfo.nFrameNum - last_frame_num;
    if (stImageInfo.nFrameNum != last_frame_num + 1) {
      printf("Cam[%s]: Lost frames %u/%u at frame[%u]\n", camName,
             lost_frame_count, total_frame_count, stImageInfo.nFrameNum);
    }
  }

  CompressedImage image_msg;
  image_msg.format = "jpeg";
  image_msg.data = {out, out + len};
  // Should be the name of a reference frame
  // image_msg.header.frame_id = stImageInfo.nFrameNum;
  using namespace std::chrono_literals;
  if (uint64_t ts = trigger_manager.get_timestamp(
          camName, stImageInfo.nFrameNum, trigger_step, 100ms)) {
    image_msg.header.stamp.sec = ts / 1000000;
    image_msg.header.stamp.nanosec =
        stImageInfo.nHostTimeStamp % 1000000 * 1000;
  } else {
    // Fallback to host timestamp
    image_msg.header.stamp.sec = stImageInfo.nHostTimeStamp / 1000000;
    image_msg.header.stamp.nanosec =
        stImageInfo.nHostTimeStamp % 1000 * 1000000;
  }
  publisher->publish(image_msg);

  last_frame_num = stImageInfo.nFrameNum;
}

int CameraMaster::init(float trigger_rate) {
  node = rclcpp::Node::make_shared("hk_cameras");
  trigger_manager.init(trigger_rate, buffer_time);

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

int CameraMaster::start_grabbing() {
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
        [this, i, buffer_num] { WorkThread(handles[i], buffer_num); });
  }

  threads.emplace_back([this] {
    SerialTrigger trigger;
    trigger.init(&trigger_manager, trigger_manager.get_buffer_size());
    while (running.load(std::memory_order_relaxed)) {
      trigger.spin();
    }
    trigger.stop();
  });

  return nRet;
}

int CameraMaster::stop() {
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

void CameraMaster::WorkThread(void *handle, int buffer_num) {
  unsigned nRet = MV_OK;
  char camName[256];
  {
    MVCC_STRINGVALUE stStringValue = {};
    CHECK_MV(MV_CC_GetStringValue(handle, "DeviceUserID", &stStringValue));
    if (MV_OK == nRet) {
      memcpy(camName, stStringValue.chCurValue,
             sizeof(stStringValue.chCurValue));
    }
  }

  FrameWorker frame_worker = FrameWorker::create(
      handle, trigger_manager, camName, buffer_num, node.get());
  while (running.load(std::memory_order_relaxed)) {
    MV_FRAME_OUT frame;
    MV_FRAME_OUT_INFO_EX &stImageInfo = frame.stFrameInfo;

    nRet = MV_CC_GetImageBuffer(handle, &frame, 1000);

    if (nRet != MV_OK && nRet != MV_E_NODATA)
      printf("Cam[%s]: MV_CC_GetImageBuffer failed[%x]\n", camName, nRet);
    if (nRet != MV_OK)
      continue;

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

void CameraMaster::config_camera(void *handle, const char *camName) {
  using namespace std::literals::string_literals;
  int nRet = MV_OK;
  std::string feature_file = "configs/"s + camName + ".ini";
  CHECK_MV(MV_CC_FeatureSave(handle, feature_file.c_str()));

  if ("Fast"s == camName) {
    CHECK_MV(MV_CC_SetEnumValueByString(handle, "ADCBitDepth", "Bits_8"));
    CHECK_MV(MV_CC_SetFloatValue(handle, "ExposureTime", 2200));
    CHECK_MV(MV_CC_SetEnumValueByString(handle, "PixelFormat", "BayerRG8"));
    ;
    CHECK_MV(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 400));
  } else if ("Color"s == camName) {
  } else {
    printf("Warning: unrecognized camera\n");
  }
  CHECK_MV(MV_CC_SetEnumValueByString(handle, "GainAuto", "Once"));
  CHECK_MV(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false));
  CHECK_MV(MV_CC_SetEnumValueByString(handle, "TriggerMode", "On"));
  CHECK_MV(MV_CC_SetEnumValueByString(handle, "TriggerSource", "Line0"));
  // If without external trigger
  // CHECK_MV(MV_CC_SetEnumValueByString(handle, "TriggerMode", "Off"));

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
}
