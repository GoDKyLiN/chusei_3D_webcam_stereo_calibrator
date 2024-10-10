#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

using namespace cv;
using namespace std;

int main()
{
    string command;
    if (access("left", 0) == -1){//如果文件夹不存在
        //则创建
        command = "mkdir -p left";
        system(command.c_str());
    }
    if (access("right", 0) == -1){//如果文件夹不存在
        //则创建
        command = "mkdir -p right";
        system(command.c_str());
    }
    cv::VideoCapture Camera(0);// /dev/video#
    if (!Camera.isOpened())
    {
        cout << "Could not open the Camera " << endl;
        return -1;
    }
    cv::Mat frame;
    Camera >> frame;
    imshow("【双目视图】", frame);
    //cv::waitKey(30);
    system("/home/runlin/Desktop/Camera/camera_stereo_setting.sh");  //绝对路径
    //cv::waitKey(30);
    
    int count = 0;
    
    while (true)
    {
        Camera >> frame;
        if (frame.empty()) break;
        Mat doubleImage;
        resize(frame, doubleImage, Size(640, 240), 0, 0, INTER_AREA);
        imshow("【双目视图】", doubleImage);
        Mat LeftImage = doubleImage(Rect(0, 0, 320, 240));
        Mat RightImage = doubleImage(Rect(320, 0, 320, 240));
        Mat grayImageL;
        cvtColor(LeftImage, grayImageL, CV_BGR2GRAY);
        //imshow("【左视图】", LeftImage);
        //imshow("【右视图】", RightImage);
        char key = waitKey(30);
        if (key == 27) // Esc键退出
        {
            break;
        }
        if (key == 32)
        {
            bool leftSaved = imwrite("left/left" + std::to_string(count) + ".jpg", LeftImage);
            bool rightSaved = imwrite("right/right" + std::to_string(count) + ".jpg", RightImage);
            //imwrite("left/left" + std::to_string(count) + "_gray.jpg", grayImageL);
            if (leftSaved && rightSaved) {
                cout << "Take Photo " << endl;
            } else {
                cout << "Failed to save photos" << endl;
            }
            count++;
            imshow("图片left", LeftImage);
            imshow("图片right",RightImage);
        }
    }
    return 0;
}
