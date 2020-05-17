#include "ros/ros.h"

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <gst/gst.h>
#include <cv.h>
#include <highgui.h>
#include <cv.hpp>
//#include <opencv2/co>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;
//#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\core\core.hpp>
//#include <opencv\cv.hpp>

#define PORT 4000
#define IPADDR "163.152.125.102"

int16_t encoder_state[3] = { 0 };
float encoder_count_per_rotation = 6.0*2.0*2.0;
float digit_8bit = 256.0;
float wheel = 3.14 * 0.32; //meter
float encoder_to_velocity = (wheel * 3.6) / (encoder_count_per_rotation * digit_8bit);
int16_t pwm_unit = 10;
int16_t deg_per_pwm = -10 / pwm_unit;
//string me;
std::string me;
std::string me2;
std::string me3;
cv::Mat im1, im2, im3, im4, im5, im6;
int n, len;
float car;

 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
    sensor_msgs::Image rgb_msg;
    rgb_msg = *msg;
    rgb_msg.encoding="BGR8";
    std::vector<unsigned char> m;
    m=rgb_msg.data;
//    ROS_INFO("%d", rgb_msg.data.size());
    int w = rgb_msg.width;
    int h = rgb_msg.height;
    int s = rgb_msg.step;
    int total = h*s;

    std::string ms(m.begin(),m.end());
    //ROS_INFO("%d", ms.length());
    me=ms;

     cv_bridge::CvImagePtr cv_ptr;
    try
    {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
    catch (cv_bridge::Exception& e)
    {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
    }
    im1 = cv_ptr->image;

          
}
 void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
   {

       cv_bridge::CvImagePtr cv_ptr;
       try
       {
           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
           ROS_ERROR("cv_bridge exception: %s", e.what());
           return;
       }
       im2 = cv_ptr->image;
}

 void imageCallback3(const sensor_msgs::ImageConstPtr& msg)
   {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
           ROS_ERROR("cv_bridge exception: %s", e.what());
           return;
       }
       im3 = cv_ptr->image;
          
}
void imageCallback4(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    im4 = cv_ptr->image;

}
void imageCallback5(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    im5 = cv_ptr->image;

}
void imageCallback6(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    im6 = cv_ptr->image;

}
void chatterCallback(const std_msgs::Float32 msg)
 {
   car=msg.data;
 }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp");
    ros::NodeHandle nh;

    ros::Subscriber subs = nh.subscribe("/car_select", 1, chatterCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/racer/camera1/image_raw", 1, imageCallback);
    image_transport::Subscriber sub2 = it.subscribe("/racer/camera2/image_raw", 1, imageCallback2);
    image_transport::Subscriber sub3 = it.subscribe("/racer/camera3/image_raw", 1, imageCallback3);
    image_transport::Subscriber sub4 = it.subscribe("/racer2/camera1/image_raw", 1, imageCallback4);
    image_transport::Subscriber sub5 = it.subscribe("/racer2/camera2/image_raw", 1, imageCallback5);
    image_transport::Subscriber sub6 = it.subscribe("/racer2/camera3/image_raw", 1, imageCallback6);
    ros::Rate loop_rate(30);


    cv::VideoWriter writer;
    cv::VideoWriter writer2;
    cv::VideoWriter writer3;
//    cv::VideoWriter writer4;
//    cv::VideoWriter writer5;
//    cv::VideoWriter writer6;

    writer.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:5001 sync=false",
                0, (double)30, cv::Size(800, 450), true);
    writer2.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:5002 sync=false",
                0, (double)30, cv::Size(800, 450), true);
    writer3.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:5003 sync=false",
                0, (double)30, cv::Size(800, 450), true);
//
//    writer4.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:7001 sync=false",
//                0, (double)30, cv::Size(800, 450), true);
//    writer5.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:7002 sync=false",
//                 0, (double)30, cv::Size(800, 450), true);
//    writer6.open("appsrc ! videoconvert ! omxh264enc ! video/x-h264,stream-format=byte-stream ! h264parse ! rtph264pay mtu=60000 ! udpsink clients=163.152.125.102:7003 sync=false",
//                 0, (double)30, cv::Size(800, 450), true);

    if (!writer.isOpened()) {
        printf("=ERR= can't create writer\n");
        return -1;
    }
    int key;
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = inet_addr(IPADDR);
    servaddr.sin_port = htons(PORT);
    double bf[2]={1, 2};
    while (ros::ok())
    {
        ros::spinOnce();
        if (car==0) {
            writer.write(im1);
            writer2.write(im2);
            writer3.write(im3);
        }
        else{
            writer.write(im4);
            writer2.write(im5);
            writer3.write(im6);
        }

        n = sendto(sockfd, bf, sizeof bf, 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
//        ROS_INFO("asdf");
//           d
//        std::cout<<"sdfsd\n";
//        writer << im1;
//        key = cv::waitKey( 30 );
        loop_rate.sleep();

    }
return 0;
}
