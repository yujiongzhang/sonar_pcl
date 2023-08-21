#ifndef GLOBALPARA_H
#define GLOBALPARA_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>

using namespace std;
//using namespace cv;
#define PI 3.1415926

//extern只能声明，不能定义、赋值
//全局变量
extern int readBuf[500];
extern int dataBuf[300];
extern int lastDataBuf[300];
extern int midDataBuf[300];

extern int nLastAngle;
extern int nAngle;
extern int nMaxAngle;
extern double angle;

extern double scan_step;
extern int m_nScanLineMax;
extern int m_nScanPerAngle;
extern uint m_dwDeltaX[4800];
extern uint m_dwDeltaY[4800];
extern int m_nOrgX;
extern int m_nOrgY;
extern int m_width;
extern int m_height;

extern uchar colorTable[768]; // 256*RGB

extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

extern vector<vector<uchar>> dataMatrix;
//extern vector<vector<uchar>> dataMatrix_step2;
extern cv::Mat srcPingImage;
extern cv::Mat srcPingImage_step2;
extern cv::Mat imageNNIA;
extern cv::Mat imageINNIA;
extern cv::Mat imageRTheta_step4;



#endif