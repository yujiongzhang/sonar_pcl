#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <boost/timer.hpp>

#include "globalpara.h"
#include "dataread.h"
#include "imagefunc.h"
#include "pclfunc.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZ PointT;

//全局函数
void pcl_test();
void beforetrans_test();
void aftertrans_test(string filepath, cv::Mat &imageOut);
void coordinateTrans_test();
void multiangle_test();
void dataToPc_test();
void enhance_test();
void filt_test();
void morph_test();
void edge_test();
void handledata_test();
void interpolation_test();
void interlolation_moni();
void interlolation_moni_720();
void drawHistImage(const Mat &image, string windowName);
void data_to_point(char* _filepath);
void str_to_int(string str, vector<int> &nums);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> cylinderRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud);

void data_to_point_zyj(char* _filepath);
void data_to_image_zyj(char* _filepath);

static void on_BilateralFilter(int, void *); //双边滤波轨迹条回调函数

// Mat srcImage(700, 800, CV_8UC3, Scalar(18, 0, 0)); //行数，列数

int g_nBilateralFilterValue = 10; //双边滤波参数值
// Mat g_dstBilateral;

int main(int argc, char **argv)
{
    // beforetrans_test();
    // aftertrans_test();
    // coordinateTrans_test();
    // multiangle_test();
    // dataToPc_test();
    //  handledata_test();
    //  handleimage_test();

    // interlolation_moni();
    // interlolation_moni_720();
    // interpolation_test();

    // enhance_test();
    // filt_test();
    // morph_test();

    // edge_test();

    // data_to_point(argv[1]);
    data_to_image_zyj(argv[1]);

    // data_to_point_zyj(argv[1]);

    waitKey(0);
    return 0;
}


void data_to_image_zyj(char* _filepath)
{
    ifstream ifs;
    ifs.open(_filepath, ios::in);
    if(!ifs.is_open())
    {
        cout << "fail to open" << _filepath << endl;
        return;
    }
    
    string str_line;
    int lastpingNum = -1;
    int zhencount = 0;

    int nAngle = 0; //声纳上传的原始角度(0~588)
    int pingNum = 0; //声纳单个ping的索引

    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵

    cout << "start data to image " << endl;
    while (getline(ifs, str_line))
    {
        vector<int> tempArray;
        str_to_int(str_line, tempArray);


        for (int i = 0; i < 300; i++)
        {
            // if (i<=7)
            // {
            //     dataMatrix_step2[pingNum][i] = 4; //消除发射圈高亮
            // }
            // else{
                dataMatrix_step2[pingNum][i] = tempArray[i+1];
            // }
        }

        pingNum++;
        lastpingNum = pingNum -1;

        if (pingNum == 294) // 一圈结束
        {
            cout << "show image " << endl;
            pingNum = 0; //先清零
            lastpingNum = -1;
            //------------------------扇形成像---------------------------------
            drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像,坐标变化前
            imshow("sonar_origin_image", srcPingImage_step2);

            Mat edge;
            blur(srcPingImage_step2, edge, Size(3, 3)); //模糊图像
            // imshow("after_bur", edge);

            Canny(edge, edge, 30, 60, 3);
            imshow("after_canny", edge);
            
            vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
            pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵
            cv::Mat srcImageImproSector;                                        //插值成像图片(1维)
            sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 0, 294);
            imshow("sectorimage_impro", srcImageImproSector);
            imwrite(strcat(_filepath,"sectorimage_impro.jpg"), srcImageImproSector);

            cv::Mat srcImageImproSector3; //插值成像图片(3维)
            sonarImage_Impro_step2(dataMatrix_step2, srcImageImproSector3, edgeMatrix_step2, 0, 294);  //20,140
            imshow("sectorimage_impro3", srcImageImproSector3);
            imwrite(strcat(_filepath,"sectorimage_impro3.jpg"), srcImageImproSector3);

            // // waitKey(0);
            
            //--------------------有效边缘-----------------------------
            //图像增强
            Mat pow_res15(600, 600, CV_8UC1, Scalar(0));
            powerTrans(srcImageImproSector, pow_res15, 0.2, 2);
            imshow("pow_res15", pow_res15);
            //图像降噪
            cv::Mat bila_res;
            bilateralFilter(pow_res15, bila_res, 24, 24 * 2, 24 / 2);
            imshow("bila_res", bila_res);
            //形态学处理
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(6, 6));
            cv::Mat close1;
            morphologyEx(bila_res, close1, MORPH_CLOSE, element);
            imshow("close-1", close1); //掩模大小
            imwrite(strcat(_filepath,"close1.jpg"), close1);
            // //边缘检测
            // // canny容易出现断点
            // //环扫提取有效轮廓
            // Mat edge_effe(300, 600, CV_8UC1, Scalar(0));
            // ringScanForEdge_Sector(close1, edge_effe, 130);
            // imshow("edge_effe", edge_effe);
            // zhencount++;
            // waitKey(0);

        }
        
    }

    cout << pingNum<<endl;
    cout << zhencount << endl;

}

void data_to_point_zyj(char* _filepath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    ifstream ifs;
    ifs.open(_filepath, ios::in);
    if(!ifs.is_open())
    {
        cout << "fail to open" << _filepath << endl;
        return;
    }
    
    string str_line;
    int lastpingNum = -1;
    int zhencount = 0;

    int nAngle = 0; //声纳上传的原始角度(0~588)
    int pingNum = 0; //声纳单个ping的索引

    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵

    cout << "start data to image " << endl;
    while (getline(ifs, str_line))
    {
        vector<int> tempArray;
        str_to_int(str_line, tempArray);

        for (int i = 0; i < 300; i++)
        {
            if (i<=30)
            {
                dataMatrix_step2[pingNum][i] = 7; //消除发射圈高亮
            }
            else{
                dataMatrix_step2[pingNum][i] = tempArray[i+1];
            }
        }

        pingNum++;
        lastpingNum = pingNum -1;

        if (pingNum == 294) // 一圈结束
        {
            cout << "show image " << endl;
            pingNum = 0; //先清零
            lastpingNum = -1;
            //------------------------扇形成像---------------------------------
            drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像,坐标变化前
            imshow("sonar_origin_image", srcPingImage_step2);

            Mat edge;
            blur(srcPingImage_step2, edge, Size(3, 3)); //模糊图像
            // imshow("after_bur", edge);

            Canny(edge, edge, 30, 60, 3);
            imshow("after_canny", edge);
            
            vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
            pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵
            cv::Mat srcImageImproSector;                                        //插值成像图片(1维)
            sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 1, 292);
            imshow("sectorimage_impro", srcImageImproSector);

            cv::Mat srcImageImproSector3; //插值成像图片(3维)
            sonarImage_Impro_step2(dataMatrix_step2, srcImageImproSector3, edgeMatrix_step2, 1, 291);  //20,140
            imshow("sectorimage_impro3", srcImageImproSector3);
            
            //--------------------有效边缘-----------------------------
            //图像增强
            Mat pow_res15(600, 600, CV_8UC1, Scalar(0));
            powerTrans(srcImageImproSector, pow_res15, 0.2, 2);
            imshow("pow_res15", pow_res15);
            //图像降噪
            cv::Mat bila_res;
            bilateralFilter(pow_res15, bila_res, 24, 24 * 2, 24 / 2);
            imshow("bila_res", bila_res);
            //形态学处理
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(6, 6));
            cv::Mat close1;
            morphologyEx(bila_res, close1, MORPH_CLOSE, element);
            imshow("close-1", close1); //掩模大小
            imwrite("bridge1.jpg",close1);
            // //边缘检测
            // // canny容易出现断点
            // //环扫提取有效轮廓
            // Mat edge_effe(300, 600, CV_8UC1, Scalar(0));
            // ringScanForEdge_Sector(close1, edge_effe, 130);
            // imshow("edge_effe", edge_effe);
            // zhencount++;
            // waitKey(0);

            for (int r = 0; r < close1.rows; r++)
                {
                    for (int c = 0; c < close1.cols; c++)
                    {
                        if (close1.at<uchar>(r, c) > 100)
                        {
                            pcl::PointXYZ p;            //unit: mm
                            p.x = (r - 300) * 3.333;
                            p.y = (300 - c) * 3.333; //面向ROV右侧为Y轴  小水池3.333
                            p.z = 0;
                            pointCloud->push_back(p);
                        }
                    }
                }

        }
        
    }

    cout << pingNum<<endl;
    cout << zhencount << endl;

    // Build a passthrough filter to remove spurious NaNs
    // pcl::PassThrough<PointT> pass; //直通滤波对象
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pass.setInputCloud(pointCloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-1000, 1000);
    // pass.filter(*cloud_filtered); //直通滤波保留（0，1.5）范围内的点
    // pcl::io::savePCDFile("pcdpool-2.pcd", *cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered = pointCloud;
    pcl::io::savePCDFile("bridge1.pcd", *cloud_filtered);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pointcloud"));
    pcl::visualization::PCLVisualizer viewer("display");
    //原始点云绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_filtered, 100, 100, 255);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_filtered, blue, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); //设置点云大小
    viewer.spin();

}

void data_to_point(char* _filepath)
{

    ifstream ifs;
    // ifs.open("pool-1.txt", ios::in);
    ifs.open(_filepath,ios::in);
    if (!ifs.is_open())
    {
        cout << "fail to open"<< _filepath << endl;
        return;
    }
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵

    string str_line;

    int pingcount = 0;
    int lastpingNum = -1;
    int zhencount = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "start data to point " << endl;
    while (getline(ifs, str_line))
    {
        vector<int> tempArray;
        str_to_int(str_line, tempArray);
        if (tempArray[3] == 254 && tempArray[4] == 128 && tempArray[5] == 0 && tempArray[6] == 44 && tempArray[10] == 253)
        {
            int nAngle = (((tempArray[9] & 0x7f) << 7) | (tempArray[8] & 0x7f));
            int pingNum = nAngle / 2;
            if (lastpingNum == -1)
            {
                lastpingNum = pingNum;
            }
            else if (pingNum != (lastpingNum + 1) && abs(pingNum - lastpingNum) > 200)
            {
                continue;
            }

            for (int i = 0; i < 300; i++)
            {
                if (i <= 5)                           //量程1：21
                    dataMatrix_step2[pingNum][i] = 7; //消除发射圈高亮
                else
                    dataMatrix_step2[pingNum][i] = tempArray[i + 11];
                // dataMatrix_step2[pingNum][i] = tempArray[i + 11]*2;
            }

            pingcount++;
            if (pingcount == 294) // 一圈结束
            {
                pingcount = 0; //先清零
                lastpingNum = -1;
                //------------------------扇形成像---------------------------------
                drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像,坐标变化前
                Mat edge;
                blur(srcPingImage_step2, edge, Size(3, 3)); //模糊图像
                Canny(edge, edge, 30, 60, 3);
                vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
                pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵
                cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(255));        //插值成像图片
                sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 1, 159);
                // imshow("sectorimage_impro", srcImageImproSector);

                cv::Mat srcImageImproSector3(300, 600, CV_8UC3, Scalar(255, 255, 255)); //插值成像图片
                sonarImage_Impro_step2(dataMatrix_step2, srcImageImproSector3, edgeMatrix_step2, 30, 130);  //20,140
                // imshow("sectorimage_impro3", srcImageImproSector3);

                // waitKey(0);
                //--------------------有效边缘-----------------------------
                //图像增强
                Mat pow_res15(300, 600, CV_8UC1, Scalar(0));
                powerTrans(srcImageImproSector, pow_res15, 0.2, 2);
                // imshow("pow_res15", pow_res15);
                //图像降噪
                cv::Mat bila_res;
                bilateralFilter(pow_res15, bila_res, 24, 24 * 2, 24 / 2);
                // imshow("bila_res", bila_res);
                //形态学处理
                Mat element = getStructuringElement(MORPH_ELLIPSE, Size(6, 6));
                cv::Mat close1;
                morphologyEx(bila_res, close1, MORPH_CLOSE, element);
                // imshow("close-1", close1); //掩模大小
                //边缘检测
                // canny容易出现断点
                //环扫提取有效轮廓
                Mat edge_effe(300, 600, CV_8UC1, Scalar(0));
                ringScanForEdge_Sector(close1, edge_effe, 130);
                // imshow("edge_effe", edge_effe);
                zhencount++;
                // waitKey(0);

                for (int r = 0; r < edge_effe.rows; r++)
                {
                    for (int c = 0; c < edge_effe.cols; c++)
                    {
                        if (edge_effe.at<uchar>(r, c) > 0)
                        {
                            pcl::PointXYZ p;
                            p.x = zhencount * 34;        //前进方向为X轴   小水池5
                            p.y = (c - 300) * 3.333 * 3; //面向ROV右侧为Y轴  小水池3.333
                            p.z = (300 - r) * 3.333 * 3; //垂直向上为Z轴   小水池3.333
                            pointCloud->push_back(p);
                        }
                    }
                }
                if (zhencount == 68)
                    break;
            }
        }
    }
    cout << zhencount << endl;

    // Build a passthrough filter to remove spurious NaNs
    pcl::PassThrough<PointT> pass; //直通滤波对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(pointCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1000, 1000);
    pass.filter(*cloud_filtered); //直通滤波保留（0，1.5）范围内的点
    pcl::io::savePCDFile("pcdpool-2.pcd", *cloud_filtered);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pointcloud"));
    pcl::visualization::PCLVisualizer viewer("display");
    //原始点云绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_filtered, 100, 100, 255);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_filtered, blue, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); //设置点云大小
    viewer.spin();
}

void str_to_int(string str, vector<int> &nums)
{
    int len_s = str.size();
    int i = 0, j = 0;
    while (i < len_s)
    {
        if (str[i] >= '0' && str[i] <= '9')
        {
            j = i;
            int len = 1;
            while (str[i] >= '0' && str[i] <= '9')
            {
                i++;
                len++;
            }
            string s_num = str.substr(j, len); //获取子串
            int num = 0;                       //数字字符串转换为整型数字
            stringstream ss(s_num);
            ss >> num;
            nums.push_back(num);
        }
        else
        {
            i++;
        }
    }
}

void data_to_point_test()
{

    ifstream ifs;
    ifs.open("bury-1.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "fail" << endl;
        return;
    }
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    for (int i = 0; i < 294; i++)
    {
        vector<int> tempArray(500, 0);
        for (int count = 0; count < 500; count++)
        {
            ifs >> tempArray[count];
        }
        int nAngle = (((tempArray[9] & 0x7f) << 7) | (tempArray[8] & 0x7f));
        int pingNum = nAngle / 2;

        for (int i = 0; i < 300; i++)
        {
            if (i <= 21)
                dataMatrix_step2[pingNum][i] = 7; //消除发射圈高亮
            else
                dataMatrix_step2[pingNum][i] = tempArray[i + 11];
            // dataMatrix_step2[pingNum][i] = tempArray[i + 11]*2;
        }
    }
    //------------------------扇形成像---------------------------------
    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    Mat edge;
    blur(srcPingImage_step2, edge, Size(3, 3));
    Canny(edge, edge, 30, 60, 3);
    vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵
    cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(255));        //插值成像图片
    sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 20, 140);

    imshow("sectorimage_impro", srcImageImproSector);

    //--------------------有效边缘-----------------------------
    //图像增强
    Mat pow_res15(300, 600, CV_8UC1, Scalar(0));
    powerTrans(srcImageImproSector, pow_res15, 0.2, 2);
    // imshow("pow_res15", pow_res15);
    //图像降噪
    cv::Mat bila_res;
    bilateralFilter(pow_res15, bila_res, 24, 24 * 2, 24 / 2);
    // imshow("bila_res", bila_res);
    //形态学处理
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(22, 22));
    cv::Mat close1;
    morphologyEx(bila_res, close1, MORPH_CLOSE, element);
    // imshow("close-1", close1);  //掩模大小
    //边缘检测
    // canny容易出现断点
    //环扫提取有效轮廓
    Mat edge_effe(300, 600, CV_8UC1, Scalar(0));
    ringScanForEdge_Sector(close1, edge_effe, 50);
    imshow("edge_effe", edge_effe);
}

void edge_test()
{

    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    readSonarMatrix_step2("onlytank.txt", dataMatrix_step2);

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);
    Mat edge;
    blur(srcPingImage_step2, edge, Size(3, 3));
    imshow("blur", edge);
    Canny(edge, edge, 30, 60, 3);
    imshow("edge", edge);

    vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵

    cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(0)); //插值成像图片
    sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 88, 190);
    imshow("sectorimage_impro", srcImageImproSector);
    imwrite("edge_src.jpg", srcImageImproSector);

    //图像增强
    Mat pow_res15(300, 600, CV_8UC1, Scalar(0));
    powerTrans(srcImageImproSector, pow_res15, 0.2, 2);
    imshow("pow_res15", pow_res15);
    // imwrite("pow_res15.jpg", pow_res15);

    //图像降噪
    cv::Mat bila_res;
    bilateralFilter(pow_res15, bila_res, 24, 24 * 2, 24 / 2);
    imshow("bila_res", bila_res);
    // imwrite("bila_res.jpg", bila_res);

    //形态学处理
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(22, 22));
    cv::Mat close1;
    morphologyEx(bila_res, close1, MORPH_CLOSE, element);
    imshow("close-1", close1); //掩模大小
    imwrite("edge_close.jpg", close1);

    //边缘检测
    // canny
    Mat edge_canny;
    Canny(close1, edge_canny, 30, 60, 3);
    imshow("edge_canny", edge_canny);
    imwrite("edge_canny.jpg", edge_canny);

    // Mat edge_test;
    // Canny(bila_res, edge_test, 10, 30, 3);
    // imshow("edge_test", edge_test);

    // sobel

    //环扫提取有效轮廓
    Mat edge_effe(300, 600, CV_8UC1, Scalar(0));
    ringScanForEdge_Sector(edge_canny, edge_effe, 50);
    imshow("edge_effe", edge_effe);
    imwrite("edge_res.jpg", edge_effe);

    //中值滤波
    // Mat med_res;
    // medianBlur(edge_effe, med_res, 1);
    // imshow("med_res", med_res);
    // imwrite("med_res.jpg", med_res);
}

void morph_test()
{

    Mat src = imread("bila_res.jpg", 0);
    // cout<<src.channels()<<endl;
    imshow("src", src);
    imwrite("morph_src.jpg", src);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(24, 24));
    cv::Mat close1;
    morphologyEx(src, close1, MORPH_CLOSE, element);
    imshow("close-1", close1); //掩模大小
    imwrite("morph_close.jpg", close1);

    Mat e = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    cv::Mat dila1;
    dilate(src, dila1, e);
    imshow("close-2", dila1); //掩模大小
    imwrite("morph_dilate.jpg", dila1);
}

void filt_test()
{
    Mat src = imread("pow_res15.jpg", 0);
    // cout<<src.channels()<<endl;
    imshow("src", src);

    //均值滤波
    Mat mean_res;
    blur(src, mean_res, Size(5, 5));
    imshow("mean_res", mean_res);
    imwrite("mean_res.jpg", mean_res);

    //高斯滤波
    Mat gauss_res;
    GaussianBlur(src, gauss_res, Size(5, 5), 0, 0);
    imshow("gauss_res", gauss_res);
    imwrite("gauss_res.jpg", gauss_res);

    //中值滤波
    Mat med_res;
    medianBlur(src, med_res, 5);
    imshow("med_res", med_res);
    imwrite("med_res.jpg", med_res);

    //双边滤波
    cv::Mat bila_res;
    bilateralFilter(src, bila_res, 16, 16 * 2, 16 / 2);
    imshow("bila_res", bila_res);
    imwrite("bila_res.jpg", bila_res);
}

void enhance_test()
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    readSonarMatrix_step2("onlytank.txt", dataMatrix_step2);

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);
    Mat edge;
    blur(srcPingImage_step2, edge, Size(3, 3));
    imshow("blur", edge);
    Canny(edge, edge, 30, 60, 3);
    imshow("edge", edge);

    vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵

    cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 60, 210);
    imshow("sectorimage_impro", srcImageImproSector);

    // imwrite("enhanceimage.jpg", srcImageImproSector);
    // Mat src = imread("image_forenhance.jpg");
    // cvtColor(src, srcImageImproSector, COLOR_BGR2GRAY);

    //截取目标区域
    Mat srcImage;
    srcImage = srcImageImproSector(cv::Rect(307, 5, 200, 200));
    imshow("srcImage", srcImage);
    imwrite("src.jpg", srcImage);

    Mat equ_res;
    //直方图均衡化
    equalizeHist(srcImage, equ_res);
    imshow("equ_res", equ_res);
    imwrite("equ.jpg", equ_res);

    //线性变换
    Mat lin_res(200, 200, CV_8UC1, Scalar(255));
    linearTrans(srcImage, lin_res, 2, 0);
    imshow("lin_res", lin_res);
    imwrite("lin.jpg", lin_res);

    //幂次变换
    Mat pow_res1(200, 200, CV_8UC1, Scalar(255));
    powerTrans(srcImage, pow_res1, 0.2, 1);
    imshow("pow_res1", pow_res1);
    imwrite("pow_res1.jpg", pow_res1);

    Mat pow_res15(200, 200, CV_8UC1, Scalar(255));
    powerTrans(srcImage, pow_res15, 0.2, 1.5);
    imshow("pow_res15", pow_res15);
    imwrite("pow_res15.jpg", pow_res15);

    Mat pow_res2(200, 200, CV_8UC1, Scalar(255));
    powerTrans(srcImage, pow_res2, 0.2, 2);
    imshow("pow_res2", pow_res2);
    imwrite("pow_res2.jpg", pow_res2);

    // Mat element = getStructuringElement(MORPH_RECT, Size(5,5));

    // cv::Mat close(200,200, CV_8UC1);
    // morphologyEx(pow_res, close, MORPH_CLOSE, element);
    // imshow("close-3", close);  //掩模大小

    // Mat element = getStructuringElement(MORPH_RECT, Size(4,4));

    // cv::Mat close(300,294, CV_8UC1);
    // morphologyEx(srcImageRTheta, close, MORPH_CLOSE, element);
    // imshow("close-3", close);  //掩模大小

    // cv::Mat filt(300,294, CV_8UC1);
    // bilateralFilter(close, filt, 20, 20*2, 20/2);
    // imshow("srcfilter-30", filt);

    // cv::Mat imageRTheta(650,650, CV_8UC1);//插值成像图片
    // sonarImage_RTheta_step2(dataMatrix_step2,imageRTheta);
    // imshow("sectorimage", imageRTheta);

    //双边滤波
    // cv::Mat imageout(650,650, CV_8UC1);
    // bilateralFilter(imageRTheta, imageout, 20, 20*2, 20/2);
    // imshow("after filter-10", imageout);

    // Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    // cv::Mat close(300,294, CV_8UC1);
    // morphologyEx(imageRTheta, close, MORPH_CLOSE, element);
    // imshow("close-3", close);  //掩模大小

    // cv::Mat imageRTheta(650,650, CV_8UC1);//插值成像图片
    // sonarImage_RTheta_step2(dataMatrix_step2,imageRTheta);
    // imshow("sectorimage", imageRTheta);

    // cv::Mat edgeImage(650,650,CV_8UC1);
    // ringScanForEdge(filt, edgeImage,23);
    // imshow("after edgescan(thres=23)", edgeImage);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // singlePointCloud(edgeImage, pointCloud, 0);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = simpleVis(pointCloud);
    // viewer->spin();
}

void interlolation_moni_720()
{
    vector<vector<uchar>> dataMatrix_step2(540, vector<uchar>(300, 0)); //数据矩阵
    cv::Mat src(300, 600, CV_8UC1, Scalar(255));                        //插值成像图片
    readMatrixFromImage_step2_540("moni_for3.png", dataMatrix_step2, src);

    // cout<<"000"<<endl;

    cv::Mat srcImageRThetaSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t1;
    sonarImage_RTheta_gray_step2_540(dataMatrix_step2, srcImageRThetaSector, 180, 360);
    cout << t1.elapsed() << endl;
    imshow("sectorimage_rtheta", srcImageRThetaSector);
    imwrite("rtheta540.jpg", srcImageRThetaSector);

    double mse_rtheta = culPSNR(srcImageRThetaSector, src);
    cout << mse_rtheta << endl;

    double s1 = ssim(srcImageRThetaSector, src);
    cout << s1 << endl;

    double e1 = epi(src, srcImageRThetaSector);
    cout << e1 << endl;

    cout << endl;

    cv::Mat srcImageCubicSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t2;
    sonarImage_Cubic_gray_step2_540(dataMatrix_step2, srcImageCubicSector, 180, 360);
    cout << t2.elapsed() << endl;
    imshow("sectorimage_cubic", srcImageCubicSector);
    imwrite("cubic540.jpg", srcImageCubicSector);

    double mse_cubic = culPSNR(srcImageCubicSector, src);
    cout << mse_cubic << endl;

    double s2 = ssim(srcImageCubicSector, src);
    cout << s2 << endl;
    double e2 = epi(src, srcImageCubicSector);
    cout << e2 << endl;

    cout << endl;

    cv::Mat srcPingImage_step2_540(300, 540, CV_8UC1);
    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2_540); //绘制Ping图像  不需要修改
    imshow("srcPingImage", srcPingImage_step2_540);
    imwrite("pingimage540.jpg", srcPingImage_step2_540);
    Mat edge;
    blur(srcPingImage_step2_540, edge, Size(3, 3));
    imshow("blur", edge);
    Canny(edge, edge, 30, 60, 3);
    imshow("edge", edge);

    vector<vector<uchar>> edgeMatrix_step2(540, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵

    cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t3;
    sonarImage_Impro_gray_step2_540(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 180, 360);
    cout << t3.elapsed() << endl;
    imshow("sectorimage_impro", srcImageImproSector);
    // imwrite("cubic.jpg", srcImageCubicSector);
    imwrite("impro540.jpg", srcImageImproSector);

    double mse_impro = culPSNR(srcImageImproSector, src);
    cout << mse_impro << endl;

    double s3 = ssim(srcImageImproSector, src);
    cout << s3 << endl;

    double e3 = epi(src, srcImageImproSector);
    cout << e3 << endl;

    cout << endl;
}

void interlolation_moni()
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    cv::Mat src(300, 600, CV_8UC1, Scalar(255));                        //插值成像图片
    readMatrixFromImage_step2("moni_for3.png", dataMatrix_step2, src);

    // cout<<"000"<<endl;

    cv::Mat srcImageRThetaSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t1;
    sonarImage_RTheta_gray_step2(dataMatrix_step2, srcImageRThetaSector, 98, 196);
    cout << t1.elapsed() << endl;
    imshow("sectorimage_rtheta", srcImageRThetaSector);
    // imwrite("rtheta.jpg", srcImageRThetaSector);

    double mse_rtheta = culPSNR(srcImageRThetaSector, src);
    cout << mse_rtheta << endl;

    double s1 = ssim(srcImageRThetaSector, src);
    cout << s1 << endl
         << endl;

    cv::Mat srcImageCubicSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t2;
    sonarImage_Cubic_gray_step2(dataMatrix_step2, srcImageCubicSector, 98, 196);
    cout << t2.elapsed() << endl;
    imshow("sectorimage_cubic", srcImageCubicSector);
    // imwrite("cubic.jpg", srcImageCubicSector);

    double mse_cubic = culPSNR(srcImageCubicSector, src);
    cout << mse_cubic << endl;

    double s2 = ssim(srcImageCubicSector, src);
    cout << s2 << endl
         << endl;

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);
    Mat edge;
    blur(srcPingImage_step2, edge, Size(3, 3));
    imshow("blur", edge);
    Canny(edge, edge, 30, 60, 3);
    imshow("edge", edge);

    vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵

    cv::Mat srcImageImproSector(300, 600, CV_8UC1, Scalar(255)); //插值成像图片
    boost::timer t3;
    sonarImage_Impro_gray_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 98, 196);
    cout << t3.elapsed() << endl;
    imshow("sectorimage_impro", srcImageImproSector);
    // imwrite("cubic.jpg", srcImageCubicSector);

    double mse_impro = culPSNR(srcImageImproSector, src);
    cout << mse_impro << endl;

    double s3 = ssim(srcImageImproSector, src);
    cout << s3 << endl;
}

void interpolation_test()
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    readSonarMatrix_step2("onlytank.txt", dataMatrix_step2);
    // readSonarMatrix_step2("2-1.txt", dataMatrix_step2);

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);

    // cv::Mat filt(300,294, CV_8UC1);
    // bilateralFilter(srcPingImage_step2, filt, 16, 16*2, 16/2);
    // imshow("srcfilter-30", filt);

    //闭操作
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    cv::Mat close(300, 294, CV_8UC1);
    morphologyEx(srcPingImage_step2, close, MORPH_CLOSE, element);
    imshow("close-3", close); //掩模大小

    pingImage_to_Matrix_Step2(close, dataMatrix_step2); //转为数据矩阵

    cv::Mat srcImageRThetaSector(300, 600, CV_8UC3, Scalar(255, 255, 255)); //插值成像图片
    boost::timer t1;
    sonarImage_RTheta_step2(dataMatrix_step2, srcImageRThetaSector, 90, 190);
    cout << t1.elapsed() << endl;
    imshow("sectorimage_rtheta", srcImageRThetaSector);
    imwrite("rtheta.jpg", srcImageRThetaSector);

    cv::Mat srcImageCubicSector(300, 600, CV_8UC3, Scalar(255, 255, 255)); //插值成像图片
    boost::timer t2;
    sonarImage_Cubic_step2(dataMatrix_step2, srcImageCubicSector, 90, 190);
    cout << t2.elapsed() << endl;
    imshow("sectorimage_cubic", srcImageCubicSector);
    imwrite("cubic.jpg", srcImageCubicSector);

    Mat edge;
    blur(close, edge, Size(3, 3));
    imshow("blur", edge);
    Canny(edge, edge, 30, 60, 3);
    imshow("edge", edge);

    vector<vector<uchar>> edgeMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    pingImage_to_Matrix_Step2(edge, edgeMatrix_step2);                  //转为数据矩阵

    cv::Mat srcImageImproSector(300, 600, CV_8UC3, Scalar(255, 255, 255)); //插值成像图片
    boost::timer t3;
    sonarImage_Impro_step2(dataMatrix_step2, srcImageImproSector, edgeMatrix_step2, 90, 190);
    cout << t3.elapsed() << endl;
    imshow("sectorimage_impro", srcImageImproSector);
    imwrite("impro.jpg", srcImageImproSector);
}

void handledata_test()
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    readSonarMatrix_step2("onlytank.txt", dataMatrix_step2);

    cv::Mat srcImageRTheta(650, 650, CV_8UC1); //插值成像图片
    sonarImage_RTheta_step2(dataMatrix_step2, srcImageRTheta);
    imshow("srcimage", srcImageRTheta);

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);

    dynamicBrightness(dataMatrix_step2, 1.2);

    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("bright-1.2", srcPingImage_step2);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));

    // cv::Mat open(300,294, CV_8UC1);
    // morphologyEx(srcPingImage_step2, open, MORPH_OPEN, element);
    // imshow("open", open);

    cv::Mat close(300, 294, CV_8UC1);
    morphologyEx(srcPingImage_step2, close, MORPH_CLOSE, element);
    imshow("close-3", close); //掩模大小

    cv::Mat filt(300, 294, CV_8UC1);
    bilateralFilter(close, filt, 20, 20 * 2, 20 / 2);
    imshow("srcfilter-30", filt);

    pingImage_to_Matrix_Step2(filt, dataMatrix_step2);

    cv::Mat imageRTheta(650, 650, CV_8UC1); //插值成像图片
    sonarImage_RTheta_step2(dataMatrix_step2, imageRTheta);
    imshow("sectorimage", imageRTheta);

    //双边滤波
    // cv::Mat imageout(650,650, CV_8UC1);
    // bilateralFilter(imageRTheta, imageout, 20, 20*2, 20/2);
    // imshow("after filter-10", imageout);

    // Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    // cv::Mat close(300,294, CV_8UC1);
    // morphologyEx(imageRTheta, close, MORPH_CLOSE, element);
    // imshow("close-3", close);  //掩模大小

    // Mat element = getStructuringElement(MORPH_ELLIPSE, Size(4,4));
    // dilate(imageRTheta,imageRTheta,element);
    // imshow("after dilate", imageRTheta);

    // erode(imageRTheta,imageRTheta,element);
    // imshow("after erode", imageRTheta);

    cv::Mat edgeImage(650, 650, CV_8UC1);
    ringScanForEdge(imageRTheta, edgeImage, 23);
    imshow("after edgescan(thres=23)", edgeImage);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(edgeImage, pointCloud, 0);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(pointCloud);
    viewer->spin();
}

//分割实验数据
void dataToPc_test()
{
    cv::Mat image(650, 650, CV_8UC1);
    aftertrans_test("10-7.txt", image);
    imshow("after edgescan(thres=23)", image);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(image, pointCloud, 0);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(pointCloud);
    viewer->spin();
}

void multiangle_test()
{
    cv::Mat imageOut1(650, 650, CV_8UC1);
    aftertrans_test("1-1.txt", imageOut1);
    imshow("after edgescan(thres=23)1", imageOut1);
    cv::Mat imageOut2(650, 650, CV_8UC1);
    aftertrans_test("2-1.txt", imageOut2);
    imshow("after edgescan(thres=23)2", imageOut2);

    //生成点云
    //第一帧
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(imageOut1, pointCloud1, 0);

    //第二帧
    // singlePointCloud(imageAfterTranslate, pointCloud1,10); //叠加式，不做坐标转换之前可以用这个
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(imageOut2, pointCloud2, 0);

    // 方式2：Affine3f
    // 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    float theta = M_PI * 28 / 180; // 旋转弧度
    // 定义在各轴上的平移
    transform_2.translation() << 0, -36, 2; // 三个数分别对应X轴、Y轴、Z轴方向上的平移
    // 定义旋转矩阵，绕z轴
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴.
                                                                            // 打印平移、旋转矩阵
                                                                            // std::cout << "\n方式2: 使用Affine3f\n";
    // std::cout << transform_2.matrix() << std::endl;	//注意：不是transform_2

    /// 执行点云转换
    // transform_1 或者 transform_2 都可以实现相同的转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointCloud2, *transformed_cloud, transform_2); //注意：不是transform_2.matrix()

    //点云显示
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(pointCloud1);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "pointcloud2");
    viewer->spin();
}

//测试坐标转换模型
void coordinateTrans_test()
{
    cv::Mat imageOut(650, 650, CV_8UC1);
    aftertrans_test("2-4.txt", imageOut);
    imshow("after edgescan(thres=23)", imageOut);

    //进行图片平移
    cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1); //平移矩阵
    t_mat.at<float>(0, 0) = 1;
    t_mat.at<float>(0, 2) = 150; //水平平移量  右移150
    t_mat.at<float>(1, 1) = 1;
    t_mat.at<float>(1, 2) = 250; //竖直平移量  下移250
    cv::Size dst_sz = imageOut.size();
    cv::Mat imageAfterTranslate(650, 650, CV_8UC1); //平移后的图片
    warpAffine(imageOut, imageAfterTranslate, t_mat, dst_sz);
    imshow("after translate", imageAfterTranslate);

    //生成点云
    //第一帧
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(imageOut, pointCloud1, 0);

    //第二帧
    // singlePointCloud(imageAfterTranslate, pointCloud1,10); //叠加式，不做坐标转换之前可以用这个
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    singlePointCloud(imageAfterTranslate, pointCloud2, 50);

    // 方式2：Affine3f
    // 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // 定义在各轴上的平移
    transform_2.translation() << 0, -150, 250; // 三个数分别对应X轴、Y轴、Z轴方向上的平移
                                               // 定义旋转矩阵，绕z轴
                                               // transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));	//同理，UnitX(),绕X轴；UnitY(),绕Y轴.
    // 打印平移、旋转矩阵
    // std::cout << "\n方式2: 使用Affine3f\n";
    // std::cout << transform_2.matrix() << std::endl;	//注意：不是transform_2

    /// 执行点云转换
    // transform_1 或者 transform_2 都可以实现相同的转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointCloud2, *transformed_cloud, transform_2); //注意：不是transform_2.matrix()

    //点云显示
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(pointCloud1);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "pointcloud2");
    viewer->spin();
}

void aftertrans_test(string filepath, cv::Mat &imageOut)
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0)); //数据矩阵
    readSonarMatrix_step2(filepath, dataMatrix_step2);
    // sonarImage_RTheta_step2(dataMatrix_step2);
    // imshow("src", imageRTheta);
    dynamicBrightness(dataMatrix_step2, 1);

    cv::Mat imageRTheta(650, 650, CV_8UC1); //插值成像图片
    sonarImage_RTheta_step2(dataMatrix_step2, imageRTheta);
    imshow("index=1", imageRTheta);

    //双边滤波
    cv::Mat imageout(650, 650, CV_8UC1);
    bilateralFilter(imageRTheta, imageout, 6, 6 * 2, 6 / 2);
    imshow("after filter", imageout);

    cv::Mat edgeImage(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage, 19);
    // imshow("after edgescan(thres=19)", edgeImage);

    cv::Mat edgeImage1(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage1, 20);
    // imshow("after edgescan(thres=20)", edgeImage1);

    cv::Mat edgeImage2(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage2, 21);
    // imshow("after edgescan(thres=21)", edgeImage2);

    cv::Mat edgeImage3(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage3, 22);
    // imshow("after edgescan(thres=22)", edgeImage3);

    cv::Mat edgeImage4(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage4, 23);
    // imshow("after edgescan(thres=23)", edgeImage4);

    cv::Mat edgeImage5(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage5, 24);
    // imshow("after edgescan(thres=24)", edgeImage5);

    cv::Mat edgeImage6(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage6, 25);
    // imshow("after edgescan(thres=25)", edgeImage6);

    cv::Mat edgeImage7(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage7, 26);
    // imshow("after edgescan(thres=26)", edgeImage7);

    cv::Mat edgeImage8(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage8, 30);
    // imshow("after edgescan(thres=30)", edgeImage8);

    cv::Mat edgeImage9(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage9, 40);
    // imshow("after edgescan(thres=40)", edgeImage9);

    cv::Mat edgeImage10(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage10, 60);
    // imshow("after edgescan(thres=60)", edgeImage10);

    cv::Mat edgeImage11(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage11, 80);
    // imshow("after edgescan(thres=80)", edgeImage11);

    cv::Mat edgeImage12(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage12, 100);
    // imshow("after edgescan(thres=100)", edgeImage12);

    cv::Mat edgeImage13(650, 650, CV_8UC1);
    ringScanForEdge(imageout, edgeImage13, 200);
    // imshow("after edgescan(thres=200)", edgeImage13);

    imageOut = edgeImage6;

    //边缘提取
    // Canny(imageout,imageout,30,90,3);
    // imshow("after canny", imageout);

    //生成单帧点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // singlePointCloud(imageOut, pointCloud);
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = simpleVis(pointCloud);
    // viewer->spin();
}

//利用PING图像边缘检测，出现多层和断裂
// TXT数据--矩阵--动态亮度调节--绘制Ping图像--双边滤波--canny边缘检测--边缘点矩阵--插值成像
void beforetrans_test()
{
    vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300, 0));
    cv::Mat imageRTheta(650, 650, CV_8UC1);
    readSonarMatrix_step2("step2.txt", dataMatrix_step2);         //将TXT文件数据读入矩阵
    dynamicBrightness(dataMatrix_step2, 1.15);                    //动态亮度调节
    drawSrcPingImage_step2(dataMatrix_step2, srcPingImage_step2); //绘制Ping图像
    imshow("srcPingImage", srcPingImage_step2);
    // drawHistImage(srcPingImage_step2, "beforeEqulizeHist");

    sonarImage_RTheta_step2(dataMatrix_step2, imageRTheta); //插值成像
    imshow("src", imageRTheta);

    //中值滤波
    // Mat medianImage;
    // medianBlur(srcPingImage_step2, srcPingImage_step2, 3);
    // imshow("aftermedianblur", srcPingImage_step2);

    Mat filtPingImage_step2;
    bilateralFilter(srcPingImage_step2, filtPingImage_step2, 20, 20 * 2, 20 / 2); //双边滤波
    imshow("after filter", filtPingImage_step2);

    // blur(output, output, Size(3,3));
    // imshow("afterblur", output);

    //边缘检测
    Mat edgePingImage_step2; // 300*294
    Canny(filtPingImage_step2, edgePingImage_step2, 50, 100, 3);
    imshow("aftercanny", edgePingImage_step2);

    //边缘检测后将图像转换为矩阵
    vector<vector<uchar>> edgePingMatrix_step2(294, vector<uchar>(300, 0)); // 294*300
    pingImage_to_Matrix_Step2(edgePingImage_step2, edgePingMatrix_step2);
    //第一峰值位置
    vector<int> firstEdgePoint(294, 0);
    for (int i = 0; i < 294; i++)
    {
        for (int j = 0; j < 300; j++)
        {
            if (edgePingMatrix_step2[i][j] == 255)
            {
                firstEdgePoint[i] = j;
                break;
            }
        }
    }
    // for(int i=1; i<293; i++) {
    //     if(firstEdgePoint[i]==0) {
    //         firstEdgePoint[i]=(firstEdgePoint[i-1]+firstEdgePoint[i+1])/2;
    //     }
    // }
    //根据第一峰值位置生成边缘图像矩阵
    vector<vector<uchar>> pointPingMatrix_step2(294, vector<uchar>(300, 0));
    for (int i = 0; i < 294; i++)
    {
        if (firstEdgePoint[i] != 0)
            pointPingMatrix_step2[i][firstEdgePoint[i]] = 255;
        // if(firstEdgePoint[i]<299) pointPingMatrix_step2[i][firstEdgePoint[i]+1]=255;
        // if(firstEdgePoint[i]>0) pointPingMatrix_step2[i][firstEdgePoint[i]-1]=255;
    }
    //生成边缘PING图像
    cv::Mat pointPingImage_step2(300, 294, CV_8UC1);
    drawSrcPingImage_step2(pointPingMatrix_step2, pointPingImage_step2);
    imshow("srcPingImage1", pointPingImage_step2);
    //根据边缘图像矩阵----坐标转化插值成像
    sonarImage_RTheta_step2(pointPingMatrix_step2, imageRTheta); //插值成像
    imshow("dst", imageRTheta);

    //生成单帧点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // singlePointCloud(imageRTheta, pointCloud);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = simpleVis(pointCloud);
    // viewer->spin();

    // blur(output, output, Size(3,3));
    // imshow("afterblur", output);

    // //寻找轮廓
    // vector<vector<Point>> g_vContours;
    // vector<Vec4i> g_vHierarchy;
    // findContours(output, g_vContours,g_vHierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));

    // //绘制轮廓
    // Mat contoursImage1 = Mat::zeros(output.size(), CV_8UC1);
    // Mat contoursImage2 = Mat::zeros(output.size(), CV_8UC1);
    // for(int i=0; i<g_vContours.size(); i++)
    // {
    //     drawContours(contoursImage1, g_vContours, i, Scalar(255), 1, 8, g_vHierarchy, 0, Point());
    //     if(contourArea(g_vContours[i])>20)
    //     drawContours(contoursImage2, g_vContours, i, Scalar(255), 1, 8, g_vHierarchy, 0, Point());
    // }
    // imshow("contours1", contoursImage1);
    // imshow("contours2", contoursImage2);

    // //直方图均衡
    // cv::Mat dstPingImage_step2(300,294, CV_8UC1);
    // equalizeHist(srcPingImage_step2, dstPingImage_step2);
    // imshow("dstPingImage", dstPingImage_step2);
    // drawHistImage(dstPingImage_step2, "afterEqulizeHist");
}

void drawHistImage(const Mat &image, string windowName)
{
    Mat hist;                    //用于存放直方图计算结果
    const int channels[1] = {0}; //通道索引
    float inRanges[2] = {0, 255};
    const float *ranges[1] = {inRanges};                         //像素灰度值范围
    const int bins[1] = {256};                                   //直方图的维度，其实就是像素灰度值的最大值
    calcHist(&image, 1, channels, Mat(), hist, 1, bins, ranges); //计算图像直方图
    //准备绘制直方图
    int hist_w = 512;
    int hist_h = 400;
    int width = 2;
    Mat histImage = Mat::zeros(hist_h, hist_w, CV_8UC3);
    for (int i = 1; i <= hist.rows; ++i)
    {
        rectangle(histImage, Point(width * (i - 1), hist_h - 1),
                  Point(width * i - 1, hist_h - cvRound(hist.at<float>(i - 1) / 20)),
                  Scalar(255, 255, 255), -1);
    }
    imshow(windowName, histImage);
}

void pcl_test()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);

    initColortable();
    CalDelta();
    readSonarImage();

    cloud_ptr->width = (int)cloud_ptr->points.size();
    cloud_ptr->height = 1;

    //点云直通滤波
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(325, 350);
    pass.filter(*cloud_filtered_x);
    pass.setInputCloud(cloud_filtered_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(250, 300);
    pass.filter(*cloud_filtered_y);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered_y->points.size() << " data points." << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = simpleVis(cloud_filtered_y);
    viewer = cylinderRansac(cloud_filtered_y);
    viewer->spin();
}

// static void on_BilateralFilter(int,void*)
// {
//     bilateralFilter(srcImage, g_dstBilateral, g_nBilateralFilterValue, g_nBilateralFilterValue*2,g_nBilateralFilterValue/2);
//     imshow("双边滤波", g_dstBilateral);
// }

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    //创建3D窗口并添加点云
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> cylinderRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud)
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data
    cloud = srcCloud;
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    //   pass.setInputCloud(cloud);
    //   pass.setFilterFieldName("z");
    //   pass.setFilterLimits(0, 1.5);
    //   pass.filter(*cloud_filtered);
    //   std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setNormalDistanceWeight(0.1);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.03);
    // seg.setInputCloud(cloud_filtered);
    // seg.setInputNormals(cloud_normals);
    // // Obtain the plane inliers and coefficients
    // seg.segment(*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl; //分割平面

    // Extract the planar inliers from the input cloud
    //   extract.setInputCloud(cloud_filtered);
    //   extract.setIndices(inliers_plane);
    //   extract.setNegative(false);

    //   // Write the planar inliers to disk
    //   pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    //   extract.filter(*cloud_plane);
    //   std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
    //   writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false); //存储平面点集

    // Remove the planar inliers, extract the rest
    //   extract.setNegative(true);
    //   extract.filter(*cloud_filtered2);
    //   extract_normals.setNegative(true);
    //   extract_normals.setInputCloud(cloud_normals);
    //   extract_normals.setIndices(inliers_plane);
    //   extract_normals.filter(*cloud_normals2); //去除平面点集

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(5);
    seg.setRadiusLimits(5, 10);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
        // writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    //创建3D窗口并添加显示点云其包括法线
    //创建视窗对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    //创建视口
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Radius: 0.05", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud1", v1);
    //创建视口
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "sample cloud2", v2);
    //配置所有视口属性
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(100, 100, 0, 0, 0, 1, 0, -1, 0);
    //给视口添加法线
    // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
    // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

    return viewer;
}
