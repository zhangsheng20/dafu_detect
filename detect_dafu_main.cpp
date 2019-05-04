#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/opengl.hpp"


#include "detect.h"


#include <iostream>
using namespace std;
using namespace cv;


void CoutFps();
Mat mythreshold(Mat &InputImage, int threshold);
int OpenVideoStream(int camWay);
void GetCameraPra();



String video_file_name1= "/home/sheng/桌面/大符视频/2019-5-2_22-48-37.avi"; //blue-mark18 ,,infan2
String video_file_name = video_file_name1;


int camWay = 2; // 0: MVcamera, 1: usb cam  2: vedio
Mat frame_read, gray;
VideoCapture capture;
//#define SERIAL_SEND //是否开启串口并发送数据
#define SHOW_FRAMES //是否显示图像
//#define VIDEO_SAVE

#ifdef SERIAL_SEND
    #include "serial.h"
#endif

int main() //6
{
    #ifdef SERIAL_SEND
        {//打开串口
            if(sel.setPara(115200,8,1,'n'))
             cout<<"config success"<<endl;
            else
             cout<<"config failed"<<endl;
        }
    #endif

    Mat frame_read;
    Mat threshold_frame;             //二值化图像存贮
    OpenVideoStream(camWay);
    GetCameraPra();
    while (capture.read(frame_read))
    {
        threshold_frame = mythreshold(frame_read, 70);

        //连接连通域
        //static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1));
        //morphologyEx(threshold_frame, threshold_frame, MORPH_CLOSE, kernel_close);

        //去除噪点
        static Mat kernel_open = getStructuringElement(MORPH_RECT, Size(2, 2), Point(-1, -1));
        morphologyEx(threshold_frame, threshold_frame, MORPH_OPEN, kernel_open);

        DetectDafuArmor(threshold_frame, frame_read);


        CoutFps();


        #ifdef SHOW_FRAMES
            imshow("threshold_frame", threshold_frame);
            imshow("frame_read", frame_read);
            char c = waitKey(1);
        #endif

        #ifdef SERIAL_SEND
            SendDataToInfantry();
        #endif

    }

}


int OpenVideoStream(int camWay)
{


    if (camWay == 2) {
        capture.open(video_file_name); return true;
    }
    else if (camWay == 1) {
        capture.open(0); return true;
    }
    else if (camWay == 0) {
        return true;
    }

    if (!capture.isOpened())
    {
        printf("can not open camera or video file\n");
        return -1;
    }



}

//InputImage为三通道图像，将第三通道threshold
Mat mythreshold(Mat &InputImage, int threshold)
{
    Mat OutImage(400, 640, CV_8UC1);

    int channels = InputImage.channels();  //通道数
    int nRows = InputImage.rows;            //行数
    int nCols = InputImage.cols* channels;  //列数

    //if (InputImage.isContinuous())
    //{
    //	nCols *= nRows;
    //	nRows = 1;
    //}
    int i, j;
    uchar* pInputImage;
    uchar* pOutImage;
    for (i = 0; i < nRows; ++i)     //行数
    {
        pInputImage = InputImage.ptr<uchar>(i);
        pOutImage = OutImage.ptr<uchar>(i);
        for (j = 0; j < InputImage.cols; ++j)
        {
            if (pInputImage[j * 3 + 2] > threshold)
                pOutImage[j] = 255;
            else
                pOutImage[j] = 0;
        }
    }
    return OutImage;


}







void CoutFps()
{
    double  fps = 0;
    double t = 0;

    static double time1 = 0.001;
    static double time2 = 0.002;
    time2 = time1;
    time1 = (double)cv::getTickCount();

    t = (time1 - time2) / cv::getTickFrequency();
    fps = 1.0 / t;

    //std::cout << t << "\n";
    cout << "fps" << fps << "\n";
    //std::cout << cv::getTickFrequency() << "\n\n";

}
