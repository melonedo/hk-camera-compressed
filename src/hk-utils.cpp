#include "hk-utils.hpp"
#include "MvCameraControl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool PrintDeviceInfo(const MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined
    // name
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    printf("Serial Number: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    printf("Serial Number: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    printf("Not support.\n");
  }
  return true;
}

const char *GetCameraName(const MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else {
    return nullptr;
  }
}