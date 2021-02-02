#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
using namespace cv;

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include "std_msgs/Bool.h"
#include <cmath>
#include "dummie_recognition/Running.h"

#define MAX_DUMMIES 1
#define MIN_DIST 1
#define MIN_RED 4000

//CLASS DEFINITION
class DumDetect
{   
    private:
        ros::NodeHandle n;
        
        ros::Subscriber cam_sub;
        void cam_callback(const sensor_msgs::ImageConstPtr& img);

        ros::Subscriber pose_sub;
        void pose_callback(geometry_msgs::PoseStamped pose);
        geometry_msgs::PoseStamped actual_pose;

        ros::Publisher run_pub;
        ros::Subscriber run_sub;
        void run_callback(dummie_recognition::Running state);
        dummie_recognition::Running run_state;

    public:
        DumDetect();
        ~DumDetect();

        Mat red(Mat frame);
        bool suficient_red(Mat frame);
        bool found(Mat frame);
};

//CLASS IMPLEMENTATION
DumDetect::DumDetect()
{

    this->cam_sub = this->n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &DumDetect::cam_callback, this);
    this->pose_sub = this->n.subscribe("mavros/local_position_pose", 10, &DumDetect::pose_callback, this);
    this->run_sub = this->n.subscribe("/dummie_recognition/set_running_state", 10, &DumDetect::run_callback, this);
    this->run_pub = this->n.advertise<dummie_recognition::Running>("/dummie_recognition/set_running_state", 10);

}

DumDetect::~DumDetect()
{
    std::cout << "DumDetect() destroyed" << std::endl;
}

//CALLBACKS
void DumDetect::cam_callback(const sensor_msgs::ImageConstPtr& img)
{
    if(this->run_state.search)
    {
       // ROS_WARN("IF CAM_CALLBACK - cpp");
        cv_bridge::CvImagePtr ros_img;
        try
        {
            ros_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
            //this->cam_frame = ros_img->image;
            //this->cam_frame = cv_bridge::toCvCopy(const sensor_msgs::Image& this->cam_frame);
        }
        
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        dummie_recognition::Running dummie;
        if(this->found(ros_img->image) == true)
        {
            ROS_WARN("DUMMIE FOUND, CAM_CALLBACK - cpp");
            dummie.pose_x = this->actual_pose.pose.position.x;
            dummie.pose_y = this->actual_pose.pose.position.y;
            dummie.detected = true;
            dummie.id = dummie.id + 1;
            for (int i = 0; i < 10; i++)
            {
                this->run_pub.publish(dummie);
            }
        }
        else if (this->found(ros_img->image) == false)
        {
            ROS_WARN("DUMMIE NOT FOUND, CAM_CALLBACK - cpp");
        } 
    }
}

void DumDetect::pose_callback(geometry_msgs::PoseStamped pose)
{
    this->actual_pose = pose;
}

void DumDetect::run_callback(dummie_recognition::Running state)
{
    this->run_state = state;
}

//FUNCTIONS IMPLEMENTATIONS
Mat DumDetect::red(Mat frame)
{
    Mat mask1, mask2, res1;
    //std::cout<< (frame) << std::endl;
    // creating mask
    inRange(frame, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    inRange(frame, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);
    
    mask1 += mask2;

    Mat kernel = Mat::ones(3,3, CV_32F);
    morphologyEx(mask1,mask1,cv::MORPH_OPEN,kernel);
    morphologyEx(mask1,mask1,cv::MORPH_DILATE,kernel);
    // inverted mask
    bitwise_not(mask1, mask2);
    // red part
    bitwise_and(frame, frame, res1, mask2);

    return mask2;
}

bool DumDetect::suficient_red(Mat frame)
{
    int soma = 0; //number of red pixels
    int M = MIN_RED;

    for(int i = 1; i < frame.rows; i++){
        for(int j = 1; j < frame.cols; j++){
            if (frame.at<uchar>(i, j) >= 200){
                soma++;
            }
        }
    }
    //std::cout << soma << std::endl;
    ROS_WARN("SOMA = %d", soma);
    if (soma > M){
        imshow("frame", frame);
        waitKey(1000);
        return true;
    }
    else{
        return false;
    }
}

bool DumDetect::found(Mat frame){

    for(int i = 0; i < 30; i++){
        cvtColor(frame, frame, COLOR_RGB2HSV);
        frame = this->red(frame);
        cvtColor(frame, frame, COLOR_RGB2GRAY);
    }

    if(this->suficient_red(frame))
    {
        ROS_WARN("SUFICIENT RED TRUE - cpp");
        return true;
    }
    else
    {
        ROS_WARN("SUFICIENT RED FALSE - cpp");
        return false;
    }
}

int main(int argc, char** arvg){
    ROS_WARN("RUNNING DUMMIE DETECTION");
    ros::init(argc, arvg, "dummie_detection");
    DumDetect* detect = new DumDetect();
    ros::spin();
};

//https://answers.ros.org/question/344690/opencv-error-assertion-failed-scn-3-scn-4-in-cvtcolor/
//http://techawarey.com/programming/install-opencv-c-c-in-ubuntu-18-04-lts-step-by-step-guide/