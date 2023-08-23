#pragma once

#include "globalpara.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

//实时成像相关函数
void CalDelta();
void initColortable();
void Scan(double angle, int *buf, int len, double step);
void readSonarImage();



void readSonarMatrix();//读取像素值矩阵
void readSonarMatrix_step2(string filePath, vector<vector<uchar>>& dataMatrix_step2);//读取像素值矩阵

void readMatrixFromImage_step2(string filePath, vector<vector<uchar>>& dataMatrix_step2, cv::Mat& src);
void readMatrixFromImage_step2_540(string filePath, vector<vector<uchar>>& dataMatrix_step2, cv::Mat& src);


void drawSrcPingImage();
void drawSrcPingImage_step2(vector<vector<uchar>>& matrix, cv::Mat& pingimage);
//插值成像
void sonarImage_NNIA();
void sonarImage_INNIA();

void sonarImage_RTheta();
void sonarImage_RTheta_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRTheta);
void sonarImage_RTheta_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRTheta, 
                            int pingBegin, int pingEnd);
void sonarImage_Cubic_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd);
void sonarImage_Impro_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd);
void sonarImage_Impro_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd);
void sonarImage_Impro_gray_step2_zyj(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd);
void sonarImage_Impro_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd);

void sonarImage_Cubic_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd);
void sonarImage_Cubic_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd);

void sonarImage_RTheta_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRThetaSector, 
                            int pingBegin, int pingEnd);
void sonarImage_RTheta_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageRThetaSector, 
                            int pingBegin, int pingEnd);

void pingImage_to_Matrix_Step2(const cv::Mat& pingimage, vector<vector<uchar>>& matrix);

double culPSNR(cv::Mat& srcImage,cv::Mat& dstImage);
double ssim(cv::Mat &i1, cv::Mat & i2);
double epi(cv::Mat & srcImage, cv::Mat & dstImage);
double psnr(cv::Mat &I1, cv::Mat &I2);