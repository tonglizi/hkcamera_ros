#include "MvCameraControl.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hkcamera_ros/mymsg.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>

//#include <opencv2/core.hpp>

bool g_bExit = false;

struct Mypara {
  void *pUser;
  ros::NodeHandle n;
};

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  g_bExit = true;
  sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
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
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else {
    printf("Not support.\n");
  }

  return true;
}

// void uchar2mat(int img_width, int img_height, unsigned char
// *pDataForSaveImage,cv::Mat *frame){
//    cv::Mat img(cv::Size(img_width, img_height),CV_8UC3);
//    for (int i = 0; i < img_width * img_height * 3; i++)
//    {
//        img.at<cv::Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i %
//        3] = pDataForSaveImage[i];
//    }
//    *frame=img;
//    return;
//}

static void *WorkThread(void *pt) {
  Mypara *p;
  p = (struct Mypara *)pt;

  // 定义subscriber
  ros::Publisher pub =
      (p->n).advertise<hkcamera_ros::mymsg>("hkcamera/image_data", 1);
  ros::Rate loop_rate(50);
  hkcamera_ros::mymsg msg;
  memset(&msg, 0, sizeof(hkcamera_ros::mymsg));

  //取流准备
  int nRet = MV_OK;

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(p->pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  unsigned char *pData =
      (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData) {
    return NULL;
  }
  unsigned int nDataSize = stParam.nCurValue;

  while (1) {
    if (g_bExit) {
      break;
    }

    nRet = MV_CC_GetOneFrameTimeout(p->pUser, pData, nDataSize, &stImageInfo,
                                    1000);

    // 处理图像
    unsigned char *pDataForSaveImage = (unsigned char *)malloc(
        stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
    if (NULL == pDataForSaveImage) {
      printf("pDataForSaveImage is error!");
      return NULL;
    }
    // 填充存图参数
    // fill in the parameters of save image
    MV_SAVE_IMAGE_PARAM_EX stSaveParam;
    memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
    // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
    // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
    // Top to bottom are：
    stSaveParam.enImageType = MV_Image_Bmp;
    stSaveParam.enPixelType = stImageInfo.enPixelType;
    stSaveParam.nBufferSize =
        stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
    stSaveParam.nWidth = stImageInfo.nWidth;
    stSaveParam.nHeight = stImageInfo.nHeight;
    stSaveParam.pData = pData;
    stSaveParam.nDataLen = stImageInfo.nFrameLen;
    stSaveParam.pImageBuffer = pDataForSaveImage;
    stSaveParam.nJpgQuality = 80;

    nRet = MV_CC_SaveImageEx2(p->pUser, &stSaveParam);

    if (nRet == MV_OK) {
      // width and height
      msg.width = stImageInfo.nWidth;
      msg.height = stImageInfo.nHeight;
      msg.imagelen = stSaveParam.nImageLen;
      std_msgs::Header header;
      header.frame_id = "image";
      header.seq = 1;
      header.stamp = ros::Time::now();
      msg.header = header;
      //数组复制
      size_t size = stSaveParam.nImageLen;
      msg.data.resize(size);
      memcpy((unsigned char *)&msg.data[0], pDataForSaveImage, size);
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();

    } else {
      printf("No data[%x]\n", nRet);
    }

    free(pDataForSaveImage);
  }

  free(pData);

  return 0;
}

int main(int argc, char **argv) {
  // ROS发布节点初始化
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle n;
  int nRet = MV_OK;

  void *handle = NULL;

  do {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
      break;
    }

    if (stDeviceList.nDeviceNum > 0) {
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo) {
          break;
        }
        PrintDeviceInfo(pDeviceInfo);
      }
    } else {
      printf("Find No Devices!\n");
      break;
    }

    printf("Please Intput camera index: ");
    unsigned int nIndex = 0;
    scanf("%d", &nIndex);

    if (nIndex >= stDeviceList.nDeviceNum) {
      printf("Intput error!\n");
      break;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
      printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
      break;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
    // package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
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
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    pthread_t nThreadID;
    Mypara para;
    para.pUser = handle;
    para.n = n;
    nRet = pthread_create(&nThreadID, NULL, WorkThread, &para);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }
    printf("Publishing image data...");

    PressEnterToExit();

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }
  } while (0);

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  printf("exit\n");
  return 0;
}
