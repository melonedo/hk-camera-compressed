#pragma once
#include "MvCameraControl.h"

bool PrintDeviceInfo(const MV_CC_DEVICE_INFO *pstMVDevInfo);
const char *GetCameraName(const MV_CC_DEVICE_INFO *pstMVDevInfo);
enum {
  MV_ADC_BIT_DEPTH_8 = 0,
  MV_ADC_BIT_DEPTH_12 = 3,
};