#include "hkcamera_ros/mymsg.h"

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>                 //pcl点云格式头文件
#include <pcl_conversions/pcl_conversions.h> //转换

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

//数据编号，0～9999
int i = 0;
//先定义采集数据存放的路径，保证路径是真实存在的，否则需要先建立文件夹路径
std::string output_path("/media/xin/Elements/mydataset/");
std::string image_stamp = output_path + "timestamp_image.txt";
std::string ptcloud_stamp = output_path + "timestamp_ptcloud.txt";
// std::fstream fo;
// fo.open(image_stamp.c_str(),ios::out);
// fo.close();
// fo.open(ptcloud_stamp.c_str(),ios::out);
// fo.close();

//采集时间间隔
int secs=1;

std::string filename_creation(int i) {
  //数据最多采集10000帧
  if (i > 9999) {
    return string("");
  }
  std::string s = std::to_string(i);
  while (s.size() < 4) {
    s.insert(0, "0");
  }
  return s;
}

void callback(const hkcamera_ros::mymsgConstPtr &image_data,
              const PointCloud2ConstPtr &ptcloud) {
  /*
  采集时间戳，写在txt文件中
  */
  double image_timestamp =
      image_data->header.stamp.sec + 1e-9 * image_data->header.stamp.nsec;
  double ptcloud_timestamp =
      ptcloud->header.stamp.sec + 1e-9 * ptcloud->header.stamp.nsec;
  // ios::app表示文件内容不清除的情况下继续写入
  std::ofstream f;
  f.open(image_stamp.c_str(), ios::app);
  f << std::to_string(image_timestamp) << std::endl;
  f.close();
  f.open(ptcloud_stamp.c_str(), ios::app);
  f << std::to_string(ptcloud_timestamp) << std::endl;
  f.close();

  //文件名
  std::string filename = filename_creation(i);
  i++;
  /*采集图像写入文件:先利用opencv压缩成png格式
  */
  //先转化为Mat格式
  int size = image_data->data.size();
  int header_len = 54;
  unsigned char *pData = NULL;
  pData = (unsigned char *)malloc(size - header_len);

  int row, col, idx;
  for (int i = 0; i < image_data->height*image_data->width; i++) {
    row = i / image_data->width;
    col = i % image_data->width;
    idx = (image_data->height - row - 1)*image_data->width + col;
    for (int j = 0; j < 3; j++) {
      *(pData + 3 * i + j) =image_data->data[3 * idx + j + header_len];
    }
  }
  cv::Mat frame =
      cv::Mat(image_data->height, image_data->width, CV_8UC3, pData);
  //保存图像
  if (!filename.empty()) {
    std::string image_name = filename + ".png";
    std::string path = output_path + "images/" + image_name;
    cv::imwrite(path, frame);
  }
  free(pData);

  /*采集点云写入文件,借助pcl库对sensor_msgs::PointCloud2进行解析
  */
  pcl::PointCloud<pcl::PointXYZI> cloud_pcl_xyzi;
  pcl::fromROSMsg(*ptcloud, cloud_pcl_xyzi);
  int32_t num = cloud_pcl_xyzi.points.size();
  float *data = (float *)malloc(4 * num * sizeof(float));
  // pointers
  float *px = data + 0;
  float *py = data + 1;
  float *pz = data + 2;
  float *pi = data + 3;
  for (int32_t j = 0; j < num; j++) {
    *px = cloud_pcl_xyzi.points[j].x;
    *py = cloud_pcl_xyzi.points[j].y;
    *pz = cloud_pcl_xyzi.points[j].z;
    *pi = cloud_pcl_xyzi.points[j].intensity;
    px += 4;
    py += 4;
    pz += 4;
    pi += 4;
  }
  //写入文件
  if (!filename.empty()) {
    std::string ptcloud_name = filename + ".bin";
    std::string path = output_path + "velodyne/" + ptcloud_name;
    FILE *stream = fopen(path.c_str(), "wb");
    fwrite(data, sizeof(float), 4 * num, stream);
    fclose(stream);
  }
  free(data);
  //写完一次沉睡1s
  sleep(secs);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "data_acq_sync_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<hkcamera_ros::mymsg> image_sub(
      nh, "hkcamera/image_data", 1);
  message_filters::Subscriber<PointCloud2> ptcloud_sub(nh, "/velodyne_points",
                                                       1);
  typedef sync_policies::ApproximateTime<hkcamera_ros::mymsg, PointCloud2>
      MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, ptcloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
