#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
//#include "thermal_detect.h"
#include "std_msgs/Bool.h"
#include <cmath>
//#include "cv_detection/H_info.h"

#define MAX_DUMMIES 3
#define MIN_DIST 1

using namespace cv;

//CLASS DEFINITION
class DumDetect{
    private:
        ros::NodeHandle n;
        ros::Subscriber cam_sub;
        void cam_callback(const sensor_msgs::ImageConstPtr& img);
        ros::Subscriber run_sub;
        void running_callback(std_msgs::Bool data);
        ros::Subscriber pose_sub;
        void pose_callback(geometry_msgs::PoseStamped pose);
    
    public:
        DumDetect();
        ~DumDetect();
        Mat cam_frame;
        bool running_state; 
        bool change_position = true;
        geometry_msgs::PoseStamped actual_pose;
        ros::Publisher run_pub;

        Mat red(Mat frame);
        bool suficient_red(Mat frame);
        bool allfound(Mat frame);
};

//CLASS IMPLEMENTATION
DumDetect::DumDetect()
{
    this->cam_sub = this->n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &DumDetect::cam_callback, this);
    this->run_sub = this->n.subscribe("/cv_detection/set_running_state", 10, &DumDetect::running_callback, this);
    this->run_pub = this->n.advertise<std_msgs::Bool>("/cv_detection/set_running_state", 1);
    this->pose_sub = this->n.subscribe("/mavros/local_position/pose", 1, &DumDetect::pose_callback, this);
}

DumDetect::~DumDetect(){
    std::cout << "DumDetect() destroyed" << std::endl;
}

//CALLBACKS 
void DumDetect::running_callback(std_msgs::Bool data)
{
   this->running_state = data.data;
}

void DumDetect::cam_callback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr ros_img;
    try{
         
        ros_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
        
        this->cam_frame = ros_img->image;
        //this->cam_frame = cv_bridge::toCvCopy(const sensor_msgs::Image& this->cam_frame);

    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void DumDetect::pose_callback(geometry_msgs::PoseStamped pose)
{
    this->actual_pose = pose;
}
    
//FUNCTIONS IMPLEMENTATIONS
Mat DumDetect::red(Mat frame){

    Mat mask1, mask2, res1;
    
  // identify red images from frame.
 //https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/

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

    return (res1);
}

bool DumDetect::suficient_red(Mat frame)
{
    int soma = 0; //number of red pixels
    int MIN_RED = 10000;

    for(int i = 1; i < frame.rows; i++){
        for(int j = 1; j < frame.cols; j++){
            if (frame.at<uchar>(i, j) >= 200){
                soma++;
            }
        }
    }
    std::cout << soma << std::endl;    
    if (soma > MIN_RED){
        return true;
    }
    else
        return false;

}

 
bool DumDetect::allfound(Mat frame){
    int n = MAX_DUMMIES, contador = 0;
    geometry_msgs::PoseStamped drone_pose_initial;
    geometry_msgs::PoseStamped drone_pose;
    double dist_x, dist_y, dist, m = MIN_DIST;

    while (this->running_state == true) {
        // Save each frame
        imshow("Camera", frame);
        Mat imggray;
        Mat imgred;
                
        cvtColor(frame, frame, COLOR_RGB2HSV);
       
        imgred = this->red(frame);
        
        cvtColor(imgred, imggray, COLOR_RGB2GRAY);
        
    
        if (change_position && this->suficient_red(imggray)){
            //std::cout << "DUMMIE FOUND" << std::endl;
            ROS_WARN( "DUMMIE FOUND ");
            
            drone_pose_initial = this->actual_pose;
            contador++;
            change_position = false;
            //break;
        }
        if (contador == n){
            //std::cout << "ALL DUMMIES HAVE BEEN FOUND" << std::endl;
            ROS_WARN("ALL DUMMIES HAVE BEEN FOUND");
            return true;
        }
        // MIN DISTANCE
        drone_pose = this->actual_pose; 
        dist_x = (drone_pose.pose.position.x - drone_pose_initial.pose.position.x);
        dist_y = (drone_pose.pose.position.y - drone_pose_initial.pose.position.y);
        
        dist = dist_x * dist_x + dist_y * dist_y;
        dist = sqrt(dist);
        
        if (dist > m) // definir a conditional
        {
            this->change_position = true;
        }
        ros::spin();
    }
    return false;
}

//MAIN 
int main (int argc, char**argv){

    bool teste = false;
    while(ros::ok()){
        
        ros::init(argc, argv, "dummie_detection");
        DumDetect* find = new DumDetect();
        if(find->running_state)
        {
            Mat img = find->cam_frame;
            teste = find->allfound(img);
        }
        if (teste)
        {
            std_msgs::Bool teste_pub;
            teste_pub.data = true;
            find->run_pub.publish(teste_pub);
            ROS_WARN("MISSION COMPLETED");
        }
        else
        {
            ROS_WARN("MISSION INCOMPLETE");
        }
        ros::spin(); // trava o programa para rodar somente o callback    
    
    }
    delete find;
    return 0;
};



