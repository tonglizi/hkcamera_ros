#include<ros/ros.h>
#include<hkcamera_ros/mymsg.h>
//#include<opencv2/highgui.hpp>
//#include<opencv2/opencv.hpp>

//std::Queue<> q;

void callback(const hkcamera_ros::mymsg &msg){
	//conver to image
	
	int size=msg.data.size();
printf("width:%d,%d,%d,%d\n",msg.width,msg.height,size,msg.imagelen);
	unsigned char* pData=NULL;
	pData=(unsigned char*)malloc(size);

	for(int i=0;i<size;i++){
		*(pData+i)=msg.data[i];
		//printf("loc_[%d]",*(pData+i));
	}
	
	FILE* fp = fopen("image1.bmp", "wb");
        if (NULL == fp)
           {
               printf("fopen failed\n");
           }
        fwrite(pData, 1, msg.imagelen, fp);
        fclose(fp);
        printf("save image succeed\n");

	free(pData);
    //cv::Mat mat=cv::Mat(msg.height,msg.width,CV_8UC3,pData);

}

void consumer(){
 //VOLO something...

}

int main(int argc, char** argv){
	ros::init(argc,argv,"image_listener");
	ros::NodeHandle n;
	
	ros::Subscriber sub=n.subscribe("hkcamera/image_data",100,callback);
	ros::spin();
	return 0;
}
