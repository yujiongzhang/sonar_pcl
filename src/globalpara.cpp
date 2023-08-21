#include <globalpara.h>


//全局变量
int readBuf[500];
int dataBuf[300];
int lastDataBuf[300] = {0};
int midDataBuf[300];

int nLastAngle = -1;
int nAngle = 0;
int nMaxAngle = 588;
double angle;

double scan_step = 360. / nMaxAngle * 2;
int m_nScanLineMax;
int m_nScanPerAngle;
uint m_dwDeltaX[4800];
uint m_dwDeltaY[4800];
int m_nOrgX = 300;
int m_nOrgY = 300;
int m_width = 600;
int m_height = 600;

uchar colorTable[768]; // 256*RGB

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

vector<vector<uchar>> dataMatrix(147, vector<uchar>(300,0));
//vector<vector<uchar>> dataMatrix_step2(294, vector<uchar>(300,0));


cv::Mat srcPingImage(300,147, CV_8UC1);
cv::Mat srcPingImage_step2(300,294, CV_8UC1);

cv::Mat imageNNIA(600,600, CV_8UC1);
cv::Mat imageINNIA(650,650, CV_8UC1);
cv::Mat imageRTheta_step4(650,650, CV_8UC1);
