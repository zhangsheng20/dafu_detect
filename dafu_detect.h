#ifndef DAFU_DETECT_H
#define DAFU_DETECT_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"


#include "opencv2/core/core.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace std;
using namespace cv;


//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y);
double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm);
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm);
void DrawEnclosingRexts(Mat &grayImage, Mat &dstImage);
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y); //通过像素坐标就算云台需要转过的角度
float GetPixelLength(Point PixelPointO, Point PixelPointA);
Point2f  predcit(float angle_degree,Mat frame); //calculate  predcit
Mat GetROI(RotatedRect rotate_recte_rect, Mat &grayImage);
void GetCameraPra();

void DetectDafuArmor(Mat &grayImage, Mat &dstImage);

class FilterOutStep
{
public:
    Point2f Filter(Point2f InputPixel,float InterframeError,int FilterLength );
    //FilterOutStep();
private:

    Point2f PixelRecord=Point2f(320,200);
    Point2f LastInputPixel=Point2f(320,200);
    Point2f CurrentInputPixel=Point2f(320,200);
    int jump_cnt=0; //


};
extern FilterOutStep FilterShootArmourCenter;

#endif
