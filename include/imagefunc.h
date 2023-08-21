#pragma once

#include "dataread.h"
#include "globalpara.h"

// using namespace cv;
//class mycomparison;

void dynamicBrightness(vector<vector<uchar>>& dataMatrix_step2,double index);
void calDeltaXY();
void ringScanForEdge(const cv::Mat& image, cv::Mat& edge, int threshold);
void ringScanForEdge_Sector(const cv::Mat& image, cv::Mat& edge, int threshold);
void linearTrans(cv::Mat& src, cv::Mat& dst, double a, double b);
void powerTrans(cv::Mat& src, cv::Mat& dst, double para, double exp);