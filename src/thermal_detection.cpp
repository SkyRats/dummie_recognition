#include <iostream>
#include <opencv2/opencv.hpp>
#define MIN_RED 1000

using namespace cv;
using namespace std;

Mat red(Mat frame);
bool suficient_red(Mat frame);

int main (){
/*
    Mat imgHSV;
    VideoCapture camera(0);
    Mat frame;

    if (!camera.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }
*/

    // test with image
    cv::Mat imgHSV;
    cv::Mat img;
    cv::Mat frame;

    img = cv::imread("love.jpg", CV_LOAD_IMAGE_COLOR);
    cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::imshow("Original", img); 
    //camera >> frame; 
    
    cv::cvtColor(frame, imgHSV, COLOR_RGB2HSV);

    cv::Mat imgred;
    imgred = red(imgHSV);
    cv::namedWindow("RED", CV_WINDOW_AUTOSIZE);
    cv::imshow("RED", imgred);

    cv::Mat imggray;
    cv::cvtColor(imgred, imggray, COLOR_RGB2GRAY);
    cv::namedWindow("GRAY", CV_WINDOW_AUTOSIZE);
    cv::imshow("GRAY", imggray);
    
    if (suficient_red(imggray)){
        std::cout << "YAY!!" << std::endl;
    }
    
    /*int sum = 0;

    for(int i = 1; i < frame.rows; i++){
    for(int j = 1; j < frame.cols; j++){
    frame.at<uchar>(i, j);
    sum += frame.at<uchar>(i, j);
    */    
    
    return 0;
};

Mat red(Mat frame) // identify red images from frame.
//https://www.learnopencv.com/invisibility-cloak-using-color-detection-and-segmentation-with-opencv/

{
    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat res1;

    // creating mask
    cv::inRange(frame, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    cv::inRange(frame, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);

    mask1 += mask2;

    // inverted mask
    cv::bitwise_not(mask1, mask2);

    // red part
    cv::bitwise_and(frame, frame, res1, mask2);

    return res1;
}

bool suficient_red(Mat frame)
{
    int sum = 0; //number of red pixels

    for(int i = 1; i < frame.rows; i++){
        for(int j = 1; j < frame.cols; j++){
            if (frame.at<uchar>(i, j) != 0){
                sum += 1;
            }
        }
    }

    std::cout << sum << std::endl;    
    if (sum > MIN_RED){
        return true;
    }
    else
        return false;
}
    

// g++ webcam_capture.cpp -o webcam_capture `pkg-config --cflags --libs opencv`
