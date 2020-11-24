#include <iostream>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "thermal_detect.h"
#include "std_msgs/Bool.h"
//#include "cv_detection/H_info.h"

using namespace cv;

//CLASS DEFINITIONS
class Cam{
    private:
        ros::Subscriber cam_sub;
        void cam_callback(const sensor_msgs::ImageConstPtr& img);
        
    public:
        Cam(ros::NodeHandle n);
        ~Cam();
        sensor_msgs::ImageConstPtr& cam_frame;
};

class Run{
    private:
        ros::Subscriber run_sub;
        void running_callback(std_msgs::Bool data);
        
    public:
        Run(ros::NodeHandle n);
        ~Run();
        bool running_state; 
};

//CLASS IMPLEMENTATIONS
Cam::Cam(ros::NodeHandle n)
{
    this->cam_sub = n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 5, &Cam::cam_callback, &cam_frame);
}
Cam::~Cam(){
    std::cout << "Cam() destroyed" << std::endl;
}

Run::Run(ros::NodeHandle n)
{
    this->run_sub = n.subscribe("/cv_detection/set_running_state", 10, &Run::running_callback, &running);
    
}
Run::~Run(){
    std::cout << "Run() destroyed" << std::endl;
}


//MAIN 
int main (int argc, char**argv){

    int contador = 0;
    Mat frame;
    Mat imgred;
    Mat imggray; 
    

    ros::init(argc, argv, "dummie_detection");

    ros::NodeHandle n;
    Run* run = new Run(n);
    Cam* cam = new Cam(n);

    ros::spin(); // trava o programa para rodar somente o callback
    
    bool running = run->running_state; 
    
    while (running == true) {
        // Show each frame
        frame = cam->cam_frame;
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
        running = run->running_state; 
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
    
//FUNCTIONS IMPLEMENTATIONS

void Run::running_callback(std_msgs::Bool data)
{
   this->running_state = (data->data);
}

void Cam::cam_callback(const sensor_msgs::ImageConstPtr& img)
{
    try{
        this->cam_frame = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

        this->cam_frame = cv_bridge::toCvCopy(const sensor_msgs::Image& this->cam_frame);

    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


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

