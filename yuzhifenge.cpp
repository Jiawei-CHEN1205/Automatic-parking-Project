//DIP大作业CPP--实现H标志的识别与停车
//陈家苇--何邦亮
#include <iostream>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <math.h>

#define LINEAR_X 0

using namespace cv;
using namespace std;

//////////////主函数//////////////
int main(int argc,char** argv)
{
    ROS_WARN("*****START");
    ros::init(argc,argv,"TargetTrack");//初始化ROS节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);//定义速度发布器
		

    VideoCapture capture;
    capture.open(0);//0为打开zed相机
    waitKey(1000);
    if(!capture.isOpened())
    {
        printf("Cannot open the camera!");
        return 0;
    }

    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
    int lowh = 25; //阈值分割的参数范围
    int lows = 65;
    int lowv = 196;
    int highh = 45;
    int highs = 255;
    int highv = 255;
    
	static int b = 0;//定义静态变量b

    //namedWindow("test",CV_WINDOW_AUTOSIZE);//需要根据实际测试
    //cvCreateTrackbar("lowh","test",&lowh,25);
    //cvCreateTrackbar("lows","test",&lows,65);
    //cvCreateTrackbar("lowv","test",&lowv,196);
    //cvCreateTrackbar("highh","test",&highh,45);
    //cvCreateTrackbar("highs","test",&highs,255);
    //cvCreateTrackbar("highv","test",&highv,255);
 
    while(ros::ok())
    {
        capture.read(frame);
        if(frame.empty())
        {
            break;
        }

    imshow("origin",frame);	//原图
    Mat dstHSV;
    vector<Mat> hsvsplit;
    cvtColor(frame,dstHSV,COLOR_BGR2HSV);//颜色转换
    split(dstHSV,hsvsplit); //通道分离
    equalizeHist(hsvsplit[2],hsvsplit[2]);//直方图均衡化
    merge(hsvsplit,dstHSV); //通道合并
    Mat img;
    inRange(dstHSV,Scalar(lowh,lows,lowv),Scalar(highh,highs,highv),img); //阈值分割
    int a=0;

    int rows = img.rows, cols = img.cols;
    float a_x=0,a_y=0;
    for(int i=0;i<rows;i++)//遍历像素点，求和，进一步求中心坐标
    {
        for (int j=0;j<cols;j++)
        {
            if(img.at<uchar >(i,j)>200)
            {
             a++;
             a_x=a_x+i;
             a_y=a_y+j;
            }
        }
    }
    imshow("img",img);//阈值分割后的图像
    a += 60;   //防止出现a_y=nan
    a_x=a_x/a; //求中心x
    a_y=a_y/a; //求中心y
   
    
    //spid car
	#ifndef READIMAGE_ONLY
    
    geometry_msgs::Twist cmd_red; //vel设置
    cmd_red.linear.x = 0.2;//车在一开始距离比较远看不见H的时候，设置转态为前进以寻找目标
    cmd_red.linear.y =0;
    cmd_red.linear.z = 0;
    cmd_red.angular.x=0;
    cmd_red.angular.y=0;
    cmd_red.angular.z=0;
    
    //if(a_y>=672&&(0 !=cmd_red.linear.x && 0 != cmd_red.angular.z))
    if(a_y>=672&&(cmd_red.linear.x != 0)) //角度微调
	{
	    cmd_red.angular.z=-0.25;
	}
	    if(a_y>0 && a_y<672)
	{
	    cmd_red.angular.z=0.25;
	}
	
    //a_y=a_y/(float)a;
    //cmd_red.linear.x =-(a-110000)*0.000003;
    //cmd_red.angular.z = -(a_y-336)*0.005;
    if(b > a && (a < 8000 && a> 5500)) //设置小车的停车条件
        {
            cmd_red.linear.x =0;
            cmd_red.angular.z = 0;
	    exit(0);
        }
	cout<<"a="<< a<<"   y="<<a_y<<"   v="<<cmd_red.linear.x<<"   angle="<<cmd_red.angular.z<<"   b="<<b<<endl;
	b = a;

     pub.publish(cmd_red);  
     
	#endif
        ros::spinOnce();
        waitKey(5);
    }
    
     
    return 0;

}


