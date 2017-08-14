#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "droneTest/Num.h"
#include "droneTest/Num_srv.h"
#define X 50
#define Y 50
using namespace std;
using namespace cv;
float coordinate_x[X], coordinate_y[Y];
float angle;
droneTest::Num msg;

ros::Publisher chatter_pub;
ros::ServiceServer chatter_svr;

    int iLowH = 101;
    int iHighH = 179;
 
    int iLowS = 43;
    int iHighS = 255;
 
    int iLowV = 46;
    int iHighV = 255;
long long count_flag;//输入的数据总组数即坐标的总个数
void deal();//根据输入的坐标点计算出拟合曲线
using namespace cv;
using namespace std;

void deal()//采用克莱默法则求解方程
{
    int i_deal;

    float a0, a1, a2, temp, temp0, temp1, temp2;
    float sy = 0, sx = 0, sxx = 0, syy = 0, sxy = 0, sxxy = 0, sxxx = 0, sxxxx = 0;//定义相关变量
    for (i_deal = 0; i_deal<count_flag; i_deal++)
    {
        sx += coordinate_x[i_deal];//计算xi的和
        sy += coordinate_y[i_deal];//计算yi的和
        sxx += coordinate_x[i_deal] * coordinate_x[i_deal];//计算xi的平方的和
        sxxx += pow(coordinate_x[i_deal], 3);//计算xi的立方的和
        sxxxx += pow(coordinate_x[i_deal], 4);//计算xi的4次方的和
        sxy += coordinate_x[i_deal] * coordinate_y[i_deal];//计算xi乘yi的的和
        sxxy += coordinate_x[i_deal] * coordinate_x[i_deal] * coordinate_y[i_deal];//计算xi平方乘yi的和
    }
    temp = count_flag*sxx - sx*sx;//方程的系数行列式
    temp0 = sy*sxx - sx*sxy;
    temp1 = count_flag*sxy - sy*sx;
    a0 = temp0 / temp;
    a1 = temp1 / temp;
    msg.distance=a0;
    angle = atan2(temp1, temp);
    angle = angle;
    //printf("经最小二乘法拟合得到的一元线性方程为:\n");
    //printf("f(x)=%3.3fx+%3.3f\n", a1, a0);
    msg.theta=angle;
    printf("line_angle_is %lf\n", angle);
    msg.flag = 0;
    chatter_pub.publish(msg);
}
void process(const sensor_msgs::ImageConstPtr& cam_image){
cv_bridge::CvImagePtr cv_ptr;
try
{
  cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
}

catch (cv_bridge::Exception& e)
{
  ROS_ERROR("cv_bridge exception:%s",e.what());
  return;
}

Mat img_rgb = cv_ptr->image;
////////////////////////////////////////////////////////////
    Mat imgHSV;
 
    Mat imgThresholded;
 
    Mat bwImg;
 
    std::vector<vector<cv::Point> > contours;
 
    vector<Mat> hsvSplit;


    // Open the video file 
    // Get the frame rate 
    double rate = 30;
    bool stop(false);
    cv::Mat imgOriginal= img_rgb; // current video frame 
    cv::Mat boximage;
    cv::namedWindow("Original_Frame");
    // Delay between each frame in ms 
    // corresponds to video frame rate 
    int delay = 1000 / rate;
    // for all frames in video 
        // read next frame if any 
        cv::imshow("Original_Frame", imgOriginal);
        boxFilter(imgOriginal, imgOriginal, -1, Size(3, 3), Point(-1, -1), false, BORDER_DEFAULT);
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        split(imgHSV, hsvSplit);//通道分离
        equalizeHist(hsvSplit[2], hsvSplit[2]);//直方均衡化
        merge(hsvSplit, imgHSV);//通道混合
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image       
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        //开操作
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
 
        //闭操作 (连接一些连通域)
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);                                                                           //开操作 (去除一些噪点)
        threshold(imgThresholded, bwImg, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);

        cv::findContours(bwImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        cv::imshow("threshold_region", bwImg);
        cv::Mat result1, result2;
 
        bwImg.copyTo(result1);
        bwImg.copyTo(result2);
        count_flag = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
 
            cv::Rect r = cv::boundingRect(contours[i]);
            printf("mianji: %d",(r.height*r.width));
            if ((r.height*r.width) >= 100)
            {
                cv::rectangle(result1, r, cv::Scalar(255));
                //printf("第%ld个x=%d，y=%d\n", count_flag, r.x, r.y);
                coordinate_x[count_flag] = r.x;
                coordinate_y[count_flag] = r.y;
                count_flag++;
 
            }
        }
//	printf(">>>>>>>>>>>>>>>%d\n",count_flag);
        if (count_flag >1)
        {
            deal();
	    //chatter_pub.publish();
        }
        else printf("识别的点太少\n");
        cv::imshow("all regions", result1);
        cv::findContours(bwImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
 
        // introduce a delay 
        // or press key to stop 
 
        if (cv::waitKey(delay) >= 0)
            stop = true;
    // Close the video file. 
    // Not required since called by destructor 
    waitKey(1);
//////////////////////////////////////////////////////////////
}

bool startSvr(droneTest::Num_srv::Request &req, droneTest::Num_srv::Response &res)
{
    if(msg.theta < 10){
        res.distance = msg.distance;
        res.theta = msg.theta;
        res.flag = msg.flag;
        return true;
    }else{
        return false;
    }
    msg.theta = 10;
}

int main(int argc, char **argv){

    //定义了一个窗口，窗口中有6个滑动条，通过滑动滑动条，可以改变阈值的大小，如果要分割其他颜色，就可以使用滑动条
 
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                                                //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
 
    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
 
    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
 
 
ros::init(argc,argv,"droneTest");
ros::NodeHandle n;
image_transport::ImageTransport it(n);
//ros::init(argc,argv,"lineFind") ;
chatter_pub = n.advertise<droneTest::Num>("chatter", 100);
image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);
//ros::NodeHandle node ;

//cv::namedWindow(WINDOW);
//cv::namedWindow(WINDOW2);
ros::spin();
return 0;
}

