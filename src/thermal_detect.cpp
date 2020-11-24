#include <iostream>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
//#include "thermal_detect.h"
#include "std_msgs/Bool.h"
//#include "cv_detection/H_info.h"
#define MAX_DUMMIES 1

using namespace cv;

//CLASS DEFINITION
class DumDetect{
    private:
        ros::NodeHandle n;
        ros::Subscriber cam_sub;
        void cam_callback(const sensor_msgs::ImageConstPtr& img);
        ros::Subscriber run_sub;
        void running_callback(std_msgs::Bool data);
    
    public:
        DumDetect();
        ~DumDetect();
        sensor_msgs::ImageConstPtr& cam_frame;
        bool running_state; 
        ros::Publisher run_pub;

        Mat red(Mat frame);
        Mat suficient_red(Mat frame);
        bool allfound(Mat frame);
};

//CLASS IMPLEMENTATION
DumDetect::DumDetect()
{
    this->cam_sub = this->n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &DumDetect::cam_callback, &cam_frame);
    this->run_sub = this->n.subscribe("/cv_detection/set_running_state", 10, &DumDetect::running_callback, &running_state);
    this->run_pub = this->n.advertise<std_msgs::Bool>("/cv_detection/set_running_state", 0);
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
    try{
        this->cam_frame = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

        this->cam_frame = cv_bridge::toCvCopy(const sensor_msgs::Image& this->cam_frame);

    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
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
    int n = MAX_DUMMIES;
    int contador = 0;
    while (this->running_state == true) {
        // Save each frame
        imshow("Camera", frame);
        Mat imggray;
        Mat imgred;
        
        cvtColor(frame, frame, COLOR_RGB2HSV);
       
        imgred = this->red(frame);
        
        cvtColor(imgred, imggray, COLOR_RGB2GRAY);
    
        if (this->suficient_red(imggray)){
            std::cout << "DUMMIE FOUND" << std::endl;
            contador++;
            //break;
        }
        if (contador == n){
            std::cout << "ALL DUMMIES HAVE BEEN FOUND" << std::endl;
            return true;
        }
        ros::spin();
    }
    return false;
}

//MAIN 
int main (int argc, char**argv){

    bool teste = false;
    ros::init(argc, argv, "dummie_detection");
    DumDetect* find = new DumDetect();
    ros::spin(); // trava o programa para rodar somente o callback    
    if(this->running_state)
    {
        teste = find->allfound(find->cam_frame);
    }
    if (teste)
    {
        find->run_pub.publish(teste);
        std::cout << "MISSION COMPLETED" << std::endl;
    }
    else
    {
        std::cout<< "MISSION INCOMPLETE" << std::endl;
    }
    return 0;
};


// http://library.isr.ist.utl.pt/docs/roswiki/cv_bridge(2f)Tutorials(2f)ConvertingBetweenROSImagesAndOpenCVImagesPython.html
/* 
    detect(Mat frame)
    detect(cam_frame->image)
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                      const std::string& encoding = std::string());
    CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                        const std::string& encoding = std::string());
//void HDetector::image_cb(const sensor_msgs::ImageConstPtr& img){ 
    */

//    if(this->running){
//        cv_bridge::CvImagePtr cv_ptr;

