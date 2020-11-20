#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
//using namespace std;


Mat red(Mat frame);
bool suficient_red(Mat frame);

int main (){
/*
    // test with image
    Mat imgHSV;
    Mat img;
    Mat frame;

    img = imread("mario.jpeg", CV_LOAD_IMAGE_COLOR);
    namedWindow("Original", CV_WINDOW_AUTOSIZE);
    imshow("Original", img);
    // end of test with image
*/
    // test with webcam
    Mat imgHSV;
    VideoCapture camera(0);
    Mat frame;

    if (!camera.isOpened()) {
        std::cout << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    //namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    
    Mat imgred;
    Mat imggray;
    
    while (1) {
        // Show each frame
        camera.read(frame);
        imshow("Webcam", frame);

        cvtColor(frame, imgHSV, COLOR_RGB2HSV);

       
        imgred = red(imgHSV);
        //namedWindow("RED", CV_WINDOW_AUTOSIZE);

        
        cvtColor(imgred, imggray, COLOR_RGB2GRAY);
        //namedWindow("GRAY", CV_WINDOW_AUTOSIZE); CV_WINDOW_AUTOSIZE
    
        if (suficient_red(imggray)){
            std::cout << "YAY!!" << std::endl;
            break;
        }
    }

    // end of webcam
    
    imshow("RED", imgred);
    imshow("GRAY", imggray);

    waitKey(0);
    // end of red and gray image

    return 0;
};
    

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
    int MIN_RED = 100000;

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


// g++ thermal_detection.cpp -o thermal_detection `pkg-config --cflags --libs opencv`
// ./thermal_detection
