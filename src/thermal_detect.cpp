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
//running_sub = rospy.Subscriber("/cv_detection/set_running_state", Bool, running_callback)
//self.cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

//Mat red(Mat frame);
//bool suficient_red(Mat frame);


void cam_callback(const sensor_msgs::ImageConstPtr& img);
void running_callback (bool data);
int main (int argc, char**argv){

    int contador = 0;
    Mat frame;
    Mat imgred;
    Mat imggray; 
   

    ros::init(argc, argv, "dummie_detection");

    ros::NodeHandle n;
    Run* run = new Run();
    Cam* cam = new Cam();

    
    
    ros::spin(); // trava o programa para rodar somente o callback
    
    bool running = run.running; 
    
    while (running == true) {
        // Show each frame
        frame = cam.cam_frame;
        imshow("Camera", frame);

        cvtColor(frame, imggray, COLOR_RGB2HSV);
       
        imgred = red(imggray);
        
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
        running = run.running; 
    }
    if (running == true){
        // end of webcam
        namedWindow("RED", CV_MINOR_VERSION);
        namedWindow("GRAY", CV_MINOR_VERSION);
        imshow("RED", imgred);
        imshow("GRAY", imggray);
        waitKey(0);

    }
    // end of red and gray image

    return 0;
};
    
void Run::running_callback(bool data)
{
   bool running = data.data;

}

void Cam::cam_callback(const sensor_msgs::ImageConstPtr& img)
{ 
    try{
        this->cam_frame = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


//void HDetector::image_cb(const sensor_msgs::ImageConstPtr& img){
//    if(this->runnin){
//        cv_bridge::CvImagePtr cv_ptr;