#include<ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include "hkcamera_ros/mymsg.h"

 
using namespace sensor_msgs;
using namespace message_filters;


void callback(const hkcamera_ros::mymsgConstPtr& image_data, const PointCloud2ConstPtr& ptcloud)
{
double image_timestamp=image_data->header.stamp.sec+1e-9*image_data->header.stamp.nsec;
double ptcloud_timestamp=ptcloud->header.stamp.sec+1e-9*ptcloud->header.stamp.nsec;   
printf("Image timestamp:[%f] s\n",image_timestamp);
printf("Ptcloud timestamp:[%f] s\n",ptcloud_timestamp);
// TODO 进行volo算法处理（在线）

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "timesync_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<hkcamera_ros::mymsg> image_sub(nh, "hkcamera/image_data", 1);
  message_filters::Subscriber<PointCloud2> ptcloud_sub(nh, "/velodyne_points", 1);
  typedef sync_policies::ApproximateTime<hkcamera_ros::mymsg, PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),image_sub, ptcloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
