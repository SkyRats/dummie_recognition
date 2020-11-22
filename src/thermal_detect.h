#ifndef REDDETECTION_H
#define REDDETECTION_H
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;

Mat red(Mat frame);
bool suficient_red(Mat frame);

// Insert Functions

Mat red(Mat frame){

    Mat mask1, mask2, res1;
    //Mat mask2;
   // Mat res1;
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

bool suficient_red(Mat frame)
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


#endif 