#include "pclfunc.h"

//将二维边缘图片转为三维点云，向点云中添加点
void singlePointCloud(cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, int def_x){
    for(int r=0; r<image.rows; r++) {
        for(int c=0; c<image.cols; c++) {
            if(image.at<uchar>(r,c)>0) {
                pcl::PointXYZ p;
                p.x=def_x;    //前进方向为X轴  
                p.y=c-300;    //面向ROV右侧为Y轴
                p.z=300-r;    //垂直向上为Z轴
                pointCloud->push_back(p);
            }
        }
    }
}