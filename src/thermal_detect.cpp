#include <iostream>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "thermal_detect.h"

using namespace cv;
//using namespace std;
//     running_sub = rospy.Subscriber("/cv_detection/set_running_state", Bool, running_callback)
// self.cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

//Mat red(Mat frame);
//bool suficient_red(Mat frame);

void cam_callback(const sensor_msgs::ImageConstPtr& img);
void running_callback (bool data);
int main (){

    int contador = 0;
    Mat frame;
    Mat imgred;
    Mat imggray; 
   

    ros::init(argc, argv, "dummie_detection");

    ros::NodeHandle n;
  
    ros::Subscriber cam_sub = n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, cam_callback);
    ros:: Subscriber running_state_sub = n.subscribe("/cv_detection/set_running_state", 10, running_callback);
    ros::spin(); // trava o programa para rodar somente o callback
    
    bool running = NodeHandle::subscriber(running_state_sub); 
    
    while (running == true) {
        // Show each frame
        frame = NodeHandle::subscriber(cam_sub);
        imshow("Camera", frame);

        cvtColor(frame, imgHSV, COLOR_RGB2HSV);
       
        imgred = red(imgHSV);
        
        cvtColor(imgred, imggray, COLOR_RGB2GRAY);
    
        if (suficient_red(imggray)){
            std::cout << "DUMMIE FOUND" << std::endl;
            contador++;
            //break;
        }
        if (contador == 1){
            std::cout << "ALL DUMMIES HAVE BEEN FOUND" << std::endl;
            break;
        }
        running = NodeHandle::subscriber(running_state_sub); 
    }
    if (running == true){
        // end of webcam
        namedWindow("RED", CV_WINDOW_AUTOSIZE);
        namedWindow("GRAY", CV_WINDOW_AUTOSIZE);
        imshow("RED", imgred);
        imshow("GRAY", imggray);
        waitKey(0);

    }
    // end of red and gray image

    return 0;
};
    
void running_callback(bool data)
{
   bool running = data.data;

}

void cam_callback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cam_frame;    
    try{
        cam_frame = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


//void HDetector::image_cb(const sensor_msgs::ImageConstPtr& img){
//    if(this->runnin){
//        cv_bridge::CvImagePtr cv_ptr;