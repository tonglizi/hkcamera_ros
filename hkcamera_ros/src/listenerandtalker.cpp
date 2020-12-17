#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>
#include<image_transport/image_transport.h>

#include "hkcamera_ros/mymsg.h"

#include<opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>

 
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    image_transport::ImageTransport it(n_);
    pub_ = it.advertise("/hkcamera/image", 1);
    sub_ = n_.subscribe("/hkcamera/image_data", 100, &SubscribeAndPublish::callback, this);
  }
 
//TODO 复制处理占用cpu巨高，达到60%,需要进行优化；
  void callback(const hkcamera_ros::mymsg &msg)
  {
	
//printf("width:%d,%d,%d\n",msg.width,msg.height,msg.imagelen);
	//注意bmp24位真彩色图文件头占54字节，要减去前54字节文件头
	//位图数据阵列中的第一个字节表示位图左下角的象素，而最后一个字节表示位图右上角的象素
	//这和mat的定位不同，需调整，msg.data就是原始位图的全部信息：文件头+数据阵列
/*
1 2 3
4 5 6
7 8 9
mat: 1 2 3 4 5 6 7 8 9
bmp: 7 8 9 4 5 6 1 2 3
*/
	int size=msg.data.size();
	int header_len=54;	
	unsigned char* pData=NULL;
	pData=(unsigned char*)malloc(size-header_len);
	
	int row, col, idx;
	for(int i=0;i<msg.height*msg.width;i++){		
		row=i/msg.width;
		col=i%msg.width;
		idx=(msg.height-row-1)*msg.width+col;
		for(int j=0;j<3;j++){
			*(pData+3*i+j)=msg.data[3*idx+j+header_len];
}
	}
cv::Mat frame=cv::Mat(msg.height,msg.width,CV_8UC3,pData);
image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
pub_.publish(image);
free(pData);
  }
 
private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  image_transport::Publisher pub_;
  sensor_msgs::ImagePtr image;
};
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
