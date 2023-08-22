#include "dataread.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace cv;

//计算线填充时4800条线的方向上前进一个单位长度，X、Y方向上的增量
void CalDelta()
{
    double Q = -90;
    double dQ;

    m_nScanLineMax = 4800; //4800条线填充
    m_nScanPerAngle = 14;  //每一度中有14条线

    dQ = 360. / (double)m_nScanLineMax;
    for (uint i = 0; i < m_nScanLineMax; i++)
    {
        double dx = cos(Q * M_PI / 180.);
        double dy = sin(Q * M_PI / 180.);

        m_dwDeltaX[i] = (uint32_t)(dx * 65536 + 0.5);
        m_dwDeltaY[i] = (uint32_t)(dy * 65536 + 0.5);
        Q += dQ;
    }
}

//初始化颜色表
void initColortable()
{
    std::ifstream infile("ctable.bin", std::ifstream::binary);
    if (!infile.is_open())
    {
        cout << "fail ctable.bin" << endl;
        return;
    }
    infile.read((char *)colorTable, 768);
    infile.close();
}

void Scan(double angle, int *buf, int len, double step)
{
    int index;

    // if( !hDIB ) return;
    while (angle >= 360.0)
        angle -= 360.0;

    int scans = (int)(m_nScanPerAngle * step) + 1;  //每个步距中的填充线数

    int idx = (int)(angle * m_nScanLineMax / 360.0 - scans / 2);  //起始填充线下标

    if (idx < 0)
        idx += m_nScanLineMax;

    for (int i = 0; i < scans; i++)
    {
        uint32_t x1 = m_nOrgX * 65536;
        uint32_t y1 = m_nOrgY * 65536;

        for (uint j = 0; j < 300; j++)
        {
            x1 += m_dwDeltaX[idx];
            y1 += m_dwDeltaY[idx];

            if (j <= 17)
                continue;

            if (buf[j] >= 50)
            {
                uint x = (uint)0x0000ffff & (x1 >> 16);
                uint y = (uint)0x0000ffff & (y1 >> 16);

                if (x >= m_width || y >= m_height)
                    continue;
                if (x < 0 || y < 0)
                    continue;

                pcl::PointXYZ basic_point;
                basic_point.x = x;
                basic_point.y = y;
                for (int z = 0; z < 100; z++)
                {
                    basic_point.z = z;
                    cloud_ptr->points.push_back(basic_point);
                }
                // basic_point.z = 0;
                // cloud_ptr->points.push_back(basic_point);
                // cout << "-----" << endl;
            }

            // uchar *data = srcImage.ptr<uchar>(y);
            // data[3 * x] = colorTable[buf[j] * 6 + 2];     // B
            // data[3 * x + 1] = colorTable[buf[j] * 6 + 1]; // G
            // data[3 * x + 2] = colorTable[buf[j] * 6];     // R
        }
        idx++;
        if ((uint)idx >= m_nScanLineMax)
            idx = 0;
    }
}
//实时成像方法，线填充
void readSonarImage()
{
    ifstream ifs;
    ifs.open("hmsdata.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "fail hmsdata.txt" << endl;
        return;
    }

    for (int i = 0; i < 147; i++)
    {
        int count = 0;
        while (count < 500)
        {
            ifs >> readBuf[count];
            // if(i==35) cout << readBuf[count] << " ";
            count++;
        }
        // cout<<endl;
        nAngle = (((readBuf[9] & 0x7f) << 7) | (readBuf[8] & 0x7f));
        memcpy(dataBuf, readBuf + 11, 300 * 4);

        // cout<<dataBuf[0]<<endl;
        // cout<<nAngle<<endl;
        if (nLastAngle != -1)
        {
            int j = 0;
            for (j = 0; j < 300; j++)
            {
                midDataBuf[j] = (dataBuf[j] + lastDataBuf[j]) / 2;
            }
            if (abs(nAngle - nLastAngle) > 100)
            {
                j = (nAngle + nLastAngle + nMaxAngle) / 2;
                while (j > nMaxAngle)
                    j -= nMaxAngle;
            }
            else
            {
                j = (nAngle + nLastAngle) / 2;
            }
            angle = j * 360.0 / nMaxAngle; //将此角度作为中心角度，两边分别画0.612°
            Scan(angle, midDataBuf, 300, scan_step);
        }
        angle = nAngle * 360.0 / nMaxAngle;
        Scan(angle, dataBuf, 300, scan_step);
        nLastAngle = nAngle;
        memcpy(lastDataBuf, dataBuf, 300 * 4);
    }

    ifs.close();
}

//将txt文件数据读入数据矩阵中，步距为4
void readSonarMatrix()
{
    ifstream ifs;
    ifs.open("hmsdata.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "fail hmsdata.txt" << endl;
        return;
    }
    for (int i = 0; i < 147; i++)
    {
        vector<int> tempArray(500, 0);
        for (int count = 0; count < 500; count++)
        {
            ifs >> tempArray[count];
        }
        // for(int count=0; count<500; count++)
        // {
        //     cout<<tempArray[count]<<" ";
        // }
        // cout<<endl;
        int nAngle = (((tempArray[9] & 0x7f) << 7) | (tempArray[8] & 0x7f));
        int pingNum = nAngle / 4;
        for (int i = 0; i < 300; i++)
        {
            dataMatrix[pingNum][i] = tempArray[i + 11]*2;
        }
    }
    /*
    //PPI，只保留120度
    for (int ping = 0; ping < 3; ping++)
    {
        for (int num = 0; num < 300; num++)
        {
            dataMatrix[ping][num] = 8;
        }
    }
    for (int ping = 52; ping < 147; ping++)
    {
        for (int num = 0; num < 300; num++)
        {
            dataMatrix[ping][num] = 8;
        }
    }
    for (int ping = 3; ping < 52; ping++)
    {
        for (int num = 0; num < 20; num++)
        {
            dataMatrix[ping][num] = 8;
        }
    }
    */
}

//将txt文件数据读入数据矩阵中，步距为2
void readSonarMatrix_step2(string filePath, vector<vector<uchar>>& dataMatrix_step2)
{
    ifstream ifs;
    ifs.open(filePath, ios::in);
    if (!ifs.is_open())
    {
        cout << "fail" << filePath << endl;
        return;
    }
    for (int i = 0; i < 294; i++)
    {
        vector<int> tempArray(500, 0);
        for (int count = 0; count < 500; count++)
        {
            ifs >> tempArray[count];
        }
        // for(int count=0; count<500; count++)
        // {
        //     cout<<tempArray[count]<<" ";
        // }
        // cout<<endl;
        int nAngle = (((tempArray[9] & 0x7f) << 7) | (tempArray[8] & 0x7f));
        int pingNum = nAngle / 2;
        for (int i = 0; i < 300; i++)
        {
            if(i<=21) dataMatrix_step2[pingNum][i]=7;  //消除发射圈高亮
            else dataMatrix_step2[pingNum][i] = tempArray[i + 11];
            // dataMatrix_step2[pingNum][i] = tempArray[i + 11]*2;
        }
        //cout<<"read ping success " << pingNum <<endl;
    }
    //cout<<"read success"<<endl;
}

void CalDelta_moni()
{
    double Q = -90;
    double dQ;

    m_nScanLineMax = 4800; //4800条线填充
    m_nScanPerAngle = 14;  //每一度中有14条线

    dQ = 360. / (double)m_nScanLineMax;
    for (uint i = 0; i < m_nScanLineMax; i++)
    {
        double dx = cos(Q * M_PI / 180.);
        double dy = sin(Q * M_PI / 180.);

        m_dwDeltaX[i] = (uint32_t)(dx * 65536 + 0.5);
        m_dwDeltaY[i] = (uint32_t)(dy * 65536 + 0.5);
        Q += dQ;
    }
}

void readMatrixFromImage_step2(string filePath, vector<vector<uchar>>& dataMatrix_step2,cv::Mat& src)
{
    Mat image;
    image=imread(filePath);
    //cout<<image.channels()<<endl;  //3
    cvtColor(image, src, COLOR_BGR2GRAY);
    // cout<<grayimage.channels()<<endl;  //1
    imshow("yuantu", src);
    double dangle=360./294.;
    for (int i = 98; i <= 196; i++)
    {
        // cout<<i<<endl;
        // cout<<dangle<<endl;

        double Q = i*dangle;
        // double Q = 30;
        // cout<<Q<<endl;
        double dx = sin(Q/180.0 * M_PI);
        double dy = cos(Q/180.0 * M_PI);

        // cout<<"--"<<endl;
        
        double orow=0.0;
        double ocol=300.0;
        
        for(int j=0; j<300; j++) {
            int row=orow-j*dy+0.5;
            int col=ocol+j*dx+0.5;
            // cout<< row <<" "<<col<<endl;
            dataMatrix_step2[i][j]=src.at<uchar>(row,col);
        }

    }
    // cout<<"read success"<<endl;
}

void readMatrixFromImage_step2_540(string filePath, vector<vector<uchar>>& dataMatrix_step2,cv::Mat& src)
{
    Mat image;
    image=imread(filePath);
    //cout<<image.channels()<<endl;  //3
    cvtColor(image, src, COLOR_BGR2GRAY);
    // cout<<grayimage.channels()<<endl;  //1
    imshow("yuantu", src);
    imwrite("gray.jpg",src);
    double dangle=360./540.;
    for (int i = 180; i <= 360; i++)
    {
        // cout<<i<<endl;
        // cout<<dangle<<endl;

        double Q = i*dangle;
        // double Q = 30;
        // cout<<Q<<endl;
        double dx = sin(Q/180.0 * M_PI);
        double dy = cos(Q/180.0 * M_PI);

        // cout<<"--"<<endl;
        
        double orow=0.0;
        double ocol=300.0;
        
        for(int j=0; j<300; j++) {
            int row=orow-j*dy+0.5;
            int col=ocol+j*dx+0.5;
            // cout<< row <<" "<<col<<endl;
            dataMatrix_step2[i][j]=src.at<uchar>(row,col);
        }

    }
    // cout<<"read success"<<endl;
}

//绘制Ping图像，坐标转换之前的
void drawSrcPingImage()
{
    // readSonarMatrix();
    int rowNumber = srcPingImage.rows;
    int colNumber = srcPingImage.cols;
    // cout<< rowNumber <<" "<<colNumber<<endl;
    for (int i = 0; i < rowNumber; i++)
    {
        // cout<<"----"<<endl;
        uchar *data = srcPingImage.ptr<uchar>(i);
        // cout<<"----"<<endl;
        for (int j = 0; j < colNumber; j++)
        {
            // cout<<sizeof(data[j])<<endl;
            // cout<<j<<" ";
            data[j] = dataMatrix[j][299 - i];
            // cout<<(int)data[j]<<endl;
        }
        // cout<<endl;
    }
}

//绘制Ping图像，坐标转换之前的
void drawSrcPingImage_step2(vector<vector<uchar>>& matrix, cv::Mat& pingimage)
{
    // readSonarMatrix();
    int rowNumber = pingimage.rows;  //300
    int colNumber = pingimage.cols;
    // cout<< rowNumber <<" "<<colNumber<<endl;
    for (int i = 0; i < rowNumber; i++) //行
    {
        // cout<<"----"<<endl;
        uchar *data = pingimage.ptr<uchar>(i);// data 指向 pingimage的第 i 行
        // cout<<"----"<<endl;
        for (int j = 0; j < colNumber; j++)
        {
            // cout<<sizeof(data[j])<<endl;
            // cout<<j<<" ";
            data[j] = matrix[j][299 - i];
            // cout<<(int)data[j]<<endl;
        }
        // cout<<endl;
    }
}

// //绘制Ping图像，坐标转换之前的
// void drawSrcPingImage_step2_540(vector<vector<uchar>>& matrix, cv::Mat& pingimage)
// {
//     // readSonarMatrix();
//     int rowNumber = pingimage.rows;  //300
//     int colNumber = pingimage.cols;
//     // cout<< rowNumber <<" "<<colNumber<<endl;
//     for (int i = 0; i < rowNumber; i++)
//     {
//         // cout<<"----"<<endl;
//         uchar *data = pingimage.ptr<uchar>(i);
//         // cout<<"----"<<endl;
//         for (int j = 0; j < colNumber; j++)
//         {
//             // cout<<sizeof(data[j])<<endl;
//             // cout<<j<<" ";
//             data[j] = matrix[j][299 - i];
//             // cout<<(int)data[j]<<endl;
//         }
//         // cout<<endl;
//     }
// }

//PING图像转化为294*300矩阵
void pingImage_to_Matrix_Step2(const cv::Mat& pingimage, vector<vector<uchar>>& matrix) 
{
    int rowNumber = pingimage.rows;  //300
    int colNumber = pingimage.cols;  //294

    for (int i = 0; i < rowNumber; i++)
    {
        // cout<<"----"<<endl;
        const uchar *data = pingimage.ptr<uchar>(i);
        // cout<<"----"<<endl;
        for (int j = 0; j < colNumber; j++)
        {
            // cout<<sizeof(data[j])<<endl;
            // cout<<j<<" ";
            matrix[j][299 - i] = data[j];
            // cout<<(int)data[j]<<endl;
        }
        // cout<<endl;
    }

}

//最邻近插值算法
void sonarImage_NNIA()
{
    // readSonarMatrix();
    for (int i = 0; i < 147; i++)
    {
        for (int j = 0; j < 300; j++)
        {
            int x = j * sin((i / 147.0) * 2 * PI) + 300.5;
            int y = 300.5 - j * cos((i / 147.0) * 2 * PI);

            imageNNIA.at<uchar>(y, x) = dataMatrix[i][j];
        }
    }
}

//改进的最邻近插值算法
void sonarImage_INNIA()
{
    double dAngle = 2 * PI / 147.0; // rad
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            int r = sqrt(x * x + y * y) + 0.5;
            if (r >= 300)
                continue;
            double angle; // rad
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            int pingNum = angle / dAngle + 0.5;
            if (pingNum == 147)
                pingNum = 0;
            int row = 300 - y;
            int col = x + 300;
            imageINNIA.at<uchar>(row, col) = dataMatrix[pingNum][r];
        }
    }
}

//双线性插值算法
void sonarImage_RTheta()
{
    double dAngle = 2 * PI / 147.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y);
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;
            int prePingNum = angle / dAngle;
            int afterPingNum = prePingNum + 1;
            if (afterPingNum == 147)
                afterPingNum = 0;
            int prer = sqrt(x * x + y * y);
            int afterr = prer + 1;
            //cout << prePingNum<< " " << afterPingNum << " ";
            //cout << prer << " " << afterr << " " <<endl;
            int UA = dataMatrix[prePingNum][afterr];
            int UB = dataMatrix[afterPingNum][afterr];
            int UC = dataMatrix[prePingNum][prer];
            int UD = dataMatrix[afterPingNum][prer];
            double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
            double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
            int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE;

            int row = 300 - y;
            int col = x + 300;
            imageRTheta_step4.at<uchar>(row, col) = UT;
        }
    }
}

//双线性插值算法 步距为2
void sonarImage_RTheta_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRTheta)
{
    double dAngle = 2 * PI / 294.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y);
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;
            int prePingNum = angle / dAngle;
            int afterPingNum = prePingNum + 1;
            if (afterPingNum == 294)
                afterPingNum = 0;
            int prer = sqrt(x * x + y * y);
            int afterr = prer + 1;
            // cout << prePingNum<< " " << afterPingNum << " ";
            // cout << prer << " " << afterr << " " <<endl;
            int UA = matrix[prePingNum][afterr];
            int UB = matrix[afterPingNum][afterr];
            int UC = matrix[prePingNum][prer];
            int UD = matrix[afterPingNum][prer];
            double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
            double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
            int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE;

            int row = 300 - y;
            int col = x + 300;
            imageRTheta.at<uchar>(row, col) = UT;
        }
    }
}

//双线性插值算法 步距为2 120度扇形成像
void sonarImage_RTheta_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRThetaSector, 
                            int pingBegin, int pingEnd)
{
    initColortable();

    cv::Mat imageRTheta(600,600, CV_8UC3, cv::Scalar(255,255,255));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y);
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                int prePingNum = angle / dAngle;
                int afterPingNum = prePingNum + 1;
                if (afterPingNum == 294)
                    afterPingNum = 0;
                int prer = sqrt(x * x + y * y);
                int afterr = prer + 1;
                // cout << prePingNum<< " " << afterPingNum << " ";
                // cout << prer << " " << afterr << " " <<endl;
                int UA = matrix[prePingNum][afterr];
                int UB = matrix[afterPingNum][afterr];
                int UC = matrix[prePingNum][prer];
                int UD = matrix[afterPingNum][prer];
                double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE;

                //坐标旋转
                // double rotateAngle = (pingBegin*360.0/294.0+60)/180*PI;
                // int newx =x*cos(rotateAngle)-y*sin(rotateAngle);
                // int newy =x*sin(rotateAngle)+y*cos(rotateAngle);


                // int row = 300 - newy;
                // int col = newx + 300;
                int row = 300 - y;
                int col = x + 300;

                if(UT>127) UT=127;
                if(UT<0) UT=0;

                //imageRTheta.at<uchar>(row, col) = UT;
                uchar *data = imageRTheta.ptr<uchar>(row);
                data[3 * col] = colorTable[UT * 6 + 2];     // B
                data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageRTheta);
    //图片旋转裁剪

    // cout<<" ------ "<<endl;

    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
    // cout<<rotateAngle<<endl;
	int w = imageRTheta.cols;
	int h = imageRTheta.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
    
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
    // cout<<nw<<endl;

	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageRTheta, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255,255,255));
 
	// imshow("旋转演示", dst);

    imageRThetaSector=dst(cv::Rect(nw/2-300,nh/2,600,300));

    // for(int r=0; r<300; r++) {
    //     for(int c=0; c<600; c++) {
    //         int h=imageRTheta.at<uchar>(r,c);
    //         // uchar *data = imageRThetaSector.ptr<uchar>(r);
    //         //     data[3 * c] = colorTable[h * 3 + 2];     // B
    //         //     data[3 * c + 1] = colorTable[h * 3 + 1]; // G
    //         //     data[3 * c + 2] = colorTable[h * 3];     // R 

    //     }
    // }
    

}



double func_Cubic(double x) {
    double absx=abs(x);
    double y;
    if(absx>=2) y=0;
    else if(absx>=1) {
        y=2-4*absx+2.5*absx*absx-0.5*absx*absx*absx;
        // y=4-8*absx+5*absx*absx-absx*absx*absx;
        // y=6-12*absx+7.5*absx*absx-1.5*absx*absx*absx;
        // y=4*0.67-8*0.67*absx+5*0.67*absx*absx-0.67*absx*absx*absx;
    }
    else {
        y=1-2.5*absx*absx+1.5*absx*absx*absx;
        // y=1-2*absx*absx+1*absx*absx*absx;
        // y=1-1.5*absx*absx+0.5*absx*absx*absx;
        // y=1-(3-0.67)*absx*absx+(2-0.67)*absx*absx*absx;
    }
    return y;
}

//双立方插值 步距2 扇形成像
void sonarImage_Cubic_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd) 
{
    initColortable();

    cv::Mat imageCubic(600,600, CV_8UC3, cv::Scalar(255,255,255));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;  //精确波束号
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if(r>1 && r<298) {
                    int prePingNum = angle / dAngle;
                    int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)E(0,0);

                }
                else {
                    int prePingNum = angle / dAngle;
                    int afterPingNum = prePingNum + 1;
                    if (afterPingNum == 294) afterPingNum = 0;
                    int prer = sqrt(x * x + y * y);
                    int afterr = prer + 1;
                    // cout << prePingNum<< " " << afterPingNum << " ";
                    // cout << prer << " " << afterr << " " <<endl;
                    int UA = matrix[prePingNum][afterr];
                    int UB = matrix[afterPingNum][afterr];
                    int UC = matrix[prePingNum][prer];
                    int UD = matrix[afterPingNum][prer];
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE;
                }
                
                int row = 300 - y;
                int col = x + 300;

                if(UT>127) UT=127;
                if(UT<0) UT=0;

                //imageRTheta.at<uchar>(row, col) = UT;
                uchar *data = imageCubic.ptr<uchar>(row);
                data[3 * col] = colorTable[UT * 6 + 2];     // B
                data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageCubic);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
	int w = imageCubic.cols;
	int h = imageCubic.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageCubic, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255,255,255));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
}



void sonarImage_Impro_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd)
{
    initColortable();

    cv::Mat imageImpro(600,600, CV_8UC3, cv::Scalar(0,0,0));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角

    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;  //精确波束号
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;//强度值

            int prePingNum = angle / dAngle;
            int afterPingNum = prePingNum + 1;
            if (afterPingNum == 294) afterPingNum = 0;
            int prer = sqrt(x * x + y * y);
            int afterr = prer + 1;

            int UA = matrix[prePingNum][afterr];
            int UB = matrix[afterPingNum][afterr];
            int UC = matrix[prePingNum][prer];
            int UD = matrix[afterPingNum][prer];

            int EA = edgeMatrix[prePingNum][afterr];
            int EB = edgeMatrix[afterPingNum][afterr];
            int EC = edgeMatrix[prePingNum][prer];
            int ED = edgeMatrix[afterPingNum][prer];

            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if((EA==0 && EB==0 && EC==0 && ED==0) || (r<=4) || (r>=296)) {
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE + 0.5;
                }
                else {
                    // int prePingNum = angle / dAngle;
                    // int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)(E(0,0)+0.5);

                }
                
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=127;
                if(UT<0) UT=0;

                // imageImpro.at<uchar>(row, col) = UT;
                uchar *data = imageImpro.ptr<uchar>(row);
                data[3 * col] = colorTable[UT * 6 + 2];     // B
                data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageCubic);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

     // int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
    int rotateAngle = (pingBegin+pingEnd)/2.*360.0/294.0-180;

    // int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
	int w = imageImpro.cols;
	int h = imageImpro.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageImpro, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(0,0,0));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,302));
}

//得到均方误差MSE
double culPSNR(cv::Mat & srcImage, cv::Mat & dstImage)
{
	Mat src = dstImage;
	Mat dst = srcImage;
	int channels = dstImage.channels();
	int rowsNumber = src.rows;
	int colsNumber = src.cols*channels;
	double sigma=0.0;
	double mse=0.0;
	for (int i = 0; i < rowsNumber; i++)
	{
		for (int j = 0; j < colsNumber; j++)
		{
            if(sqrt(i*i+(j-300)*(j-300))<297)
            if(j==300 || (i*1.0/abs(j-300))>0.578)
			mse += (src.ptr<uchar>(i)[j] - dst.ptr<uchar>(i)[j])*(src.ptr<uchar>(i)[j] - dst.ptr<uchar>(i)[j]);
		}
	}
	mse=mse/(rowsNumber*colsNumber);

    double psnr=10*log10(255*255/mse);
	return psnr;
}

//epi
double epi(cv::Mat & srcImage, cv::Mat & dstImage) {
    Mat src = dstImage;
	Mat dst = srcImage;
	int channels = dstImage.channels();
	int rowsNumber = src.rows;
	int colsNumber = src.cols*channels;

    int sum1=0;
    int sum2=0;

    for(int i=0; i<rowsNumber; i++) {
        for(int j=0; j<colsNumber-1; j++) {
            sum1+=abs(src.ptr<uchar>(i)[j]-src.ptr<uchar>(i)[j+1]);
        }
    }
    for(int i=0; i<rowsNumber-1; i++) {
        for(int j=0; j<colsNumber; j++) {
            sum1+=abs(src.ptr<uchar>(i)[j]-src.ptr<uchar>(i+1)[j]);
        }
    }

    for(int i=0; i<rowsNumber; i++) {
        for(int j=0; j<colsNumber-1; j++) {
            sum2+=abs(dst.ptr<uchar>(i)[j]-dst.ptr<uchar>(i)[j+1]);
        }
    }
    for(int i=0; i<rowsNumber-1; i++) {
        for(int j=0; j<colsNumber; j++) {
            sum2+=abs(dst.ptr<uchar>(i)[j]-dst.ptr<uchar>(i+1)[j]);
        }
    }

    // return sum2*1.0/sum1;
    return sum1*1.0/sum2;
}

double psnr(cv::Mat &I1, cv::Mat &I2)
{ //注意，当两幅图像一样时这个函数计算出来的psnr为0 
    Mat s1;  
    absdiff(I1, I2, s1);  
    s1.convertTo(s1, CV_32F);//转换为32位的float类型，8位不能计算平方  
    s1 = s1.mul(s1);  
    Scalar s = sum(s1);  //计算每个通道的和  
    double sse = s.val[0] + s.val[1] + s.val[2];  
    if( sse <= 1e-10) // for small values return zero  
        return 0;  
    else  
    {  
        double mse = sse / (double)(I1.channels() * I1.total()); //  sse/(w*h*3)  
        double psnr = 10.0 * log10((255*255)/mse);  
        return psnr;  
    }   
}  

double ssim(cv::Mat &s1, cv::Mat & s2){  
    Mat i1, i2;
    i1=s1(cv::Rect(100,115,400,100));
    i2=s2(cv::Rect(100,115,400,100));
    imshow("i1", i1);

    const double C1 = 6.5025, C2 = 58.5225;  
    int d = CV_32F;  
    Mat I1, I2;  
    i1.convertTo(I1, d);  
    i2.convertTo(I2, d);  
    Mat I1_2 = I1.mul(I1);  
    Mat I2_2 = I2.mul(I2);  
    Mat I1_I2 = I1.mul(I2);  
    Mat mu1, mu2;  
    GaussianBlur(I1, mu1, Size(11,11), 1.5);  
    GaussianBlur(I2, mu2, Size(11,11), 1.5);  
    Mat mu1_2 = mu1.mul(mu1);  
    Mat mu2_2 = mu2.mul(mu2);  
    Mat mu1_mu2 = mu1.mul(mu2);  
    Mat sigma1_2, sigam2_2, sigam12;  
    GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);  
    sigma1_2 -= mu1_2;  

    GaussianBlur(I2_2, sigam2_2, Size(11, 11), 1.5);  
    sigam2_2 -= mu2_2;  

    GaussianBlur(I1_I2, sigam12, Size(11, 11), 1.5);  
    sigam12 -= mu1_mu2;  
    Mat t1, t2, t3;  
    t1 = 2 * mu1_mu2 + C1;  
    t2 = 2 * sigam12 + C2;  
    t3 = t1.mul(t2);  

    t1 = mu1_2 + mu2_2 + C1;  
    t2 = sigma1_2 + sigam2_2 + C2;  
    t1 = t1.mul(t2);  

    Mat ssim_map;  
    divide(t3, t1, ssim_map);  
    Scalar mssim = mean(ssim_map);  

    // double ssim = (mssim.val[0] + mssim.val[1] + mssim.val[2]) /3;  
    double ssim = mssim.val[0];  
    return ssim;  

    // double C1 = 6.5025, C2 = 58.5225;
    // cv::Mat image_ref = i1;
    // cv::Mat image_obj = i2;
    // int width = image_ref.cols;
    // int height = image_ref.rows;
    // int width2 = image_obj.cols;
    // int height2 = image_obj.rows;
    // double mean_x = 0;
    // double mean_y = 0;
    // double sigma_x = 0;
    // double sigma_y = 0;
    // double sigma_xy = 0;
    // for (int v = 0; v < height; v++)
    // {
    //     for (int u = 0; u < width; u++)
    //     {
    //         mean_x += image_ref.at<uchar>(v, u);
    //         mean_y += image_obj.at<uchar>(v, u);

    //     }
    // }
    // mean_x = mean_x / width / height;
    // mean_y = mean_y / width / height;
    // for (int v = 0; v < height; v++)
    // {
    //     for (int u = 0; u < width; u++)
    //     {
    //         sigma_x += (image_ref.at<uchar>(v, u) - mean_x)* (image_ref.at<uchar>(v, u) - mean_x);
    //         sigma_y += (image_obj.at<uchar>(v, u) - mean_y)* (image_obj.at<uchar>(v, u) - mean_y);
    //         sigma_xy += abs((image_ref.at<uchar>(v, u) - mean_x)* (image_obj.at<uchar>(v, u) - mean_y));
    //     }
    // }
    // sigma_x = sigma_x / (width*height - 1);
    // sigma_y = sigma_y / (width*height - 1);
    // sigma_xy = sigma_xy / (width*height - 1);
    // double fenzi = (2 * mean_x*mean_y + C1) * (2 * sigma_xy + C2);
    // double fenmu = (mean_x*mean_x + mean_y * mean_y + C1) * (sigma_x + sigma_y + C2);
    // double ssim = fenzi / fenmu;
    // return ssim;
    
}  

//双线性插值算法 步距为2 120度扇形成像
void sonarImage_RTheta_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageRThetaSector, 
                            int pingBegin, int pingEnd)
{
    initColortable();

    cv::Mat imageRTheta(600,600, CV_8UC1, cv::Scalar(255));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y);
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                int prePingNum = angle / dAngle;
                int afterPingNum = prePingNum + 1;
                if (afterPingNum == 294)
                    afterPingNum = 0;
                int prer = sqrt(x * x + y * y);
                int afterr = prer + 1;
                // cout << prePingNum<< " " << afterPingNum << " ";
                // cout << prer << " " << afterr << " " <<endl;
                int UA = matrix[prePingNum][afterr];
                int UB = matrix[afterPingNum][afterr];
                int UC = matrix[prePingNum][prer];
                int UD = matrix[afterPingNum][prer];
                double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE+0.5;

                //坐标旋转
                // double rotateAngle = (pingBegin*360.0/294.0+60)/180*PI;
                // int newx =x*cos(rotateAngle)-y*sin(rotateAngle);
                // int newy =x*sin(rotateAngle)+y*cos(rotateAngle);


                // int row = 300 - newy;
                // int col = newx + 300;
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=87;
                // if(UT<0) UT=0;

                // if(UT>127) UT=115;
                // if(UT<0) UT=0;

                imageRTheta.at<uchar>(row, col) = UT;
                // uchar *data = imageRTheta.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageRTheta);
    //图片旋转裁剪

    // cout<<" ------ "<<endl;

    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
    // cout<<rotateAngle<<endl;
	int w = imageRTheta.cols;
	int h = imageRTheta.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
    
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
    // cout<<nw<<endl;

	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageRTheta, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255));
 
	// imshow("旋转演示", dst);

    imageRThetaSector=dst(cv::Rect(nw/2-300,nh/2,600,300));

    // for(int r=0; r<300; r++) {
    //     for(int c=0; c<600; c++) {
    //         int h=imageRTheta.at<uchar>(r,c);
    //         // uchar *data = imageRThetaSector.ptr<uchar>(r);
    //         //     data[3 * c] = colorTable[h * 3 + 2];     // B
    //         //     data[3 * c + 1] = colorTable[h * 3 + 1]; // G
    //         //     data[3 * c + 2] = colorTable[h * 3];     // R 

    //     }
    // }
    

}

//双线性插值算法 步距为2 120度扇形成像
void sonarImage_RTheta_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageRThetaSector, 
                            int pingBegin, int pingEnd)
{
    initColortable();

    cv::Mat imageRTheta(600,600, CV_8UC1, cv::Scalar(255));//插值成像图片
    double dAngle = 2 * PI / 540.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y);
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                int prePingNum = angle / dAngle;
                int afterPingNum = prePingNum + 1;
                if (afterPingNum == 540)
                    afterPingNum = 0;
                int prer = sqrt(x * x + y * y);
                int afterr = prer + 1;
                // cout << prePingNum<< " " << afterPingNum << " ";
                // cout << prer << " " << afterr << " " <<endl;
                int UA = matrix[prePingNum][afterr];
                int UB = matrix[afterPingNum][afterr];
                int UC = matrix[prePingNum][prer];
                int UD = matrix[afterPingNum][prer];
                double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE+0.5;

                //坐标旋转
                // double rotateAngle = (pingBegin*360.0/294.0+60)/180*PI;
                // int newx =x*cos(rotateAngle)-y*sin(rotateAngle);
                // int newy =x*sin(rotateAngle)+y*cos(rotateAngle);


                // int row = 300 - newy;
                // int col = newx + 300;
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=87;
                // if(UT<0) UT=0;

                // if(UT>127) UT=115;
                // if(UT<0) UT=0;

                imageRTheta.at<uchar>(row, col) = UT;
                // uchar *data = imageRTheta.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageRTheta);
    //图片旋转裁剪

    // cout<<" ------ "<<endl;

    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/540.0-120; //旋转角度
    // cout<<rotateAngle<<endl;
	int w = imageRTheta.cols;
	int h = imageRTheta.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
    
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
    // cout<<nw<<endl;

	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageRTheta, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255));
 
	// imshow("旋转演示", dst);

    imageRThetaSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
    

}


//双立方插值 步距2 扇形成像
void sonarImage_Cubic_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd) 
{
    // initColortable();

    cv::Mat imageCubic(600,600, CV_8UC1, cv::Scalar(255));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;  //精确波束号
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if(r>4 && r<296) {
                    int prePingNum = angle / dAngle;
                    int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)(E(0,0)+0.5);

                }
                else {
                    int prePingNum = angle / dAngle;
                    int afterPingNum = prePingNum + 1;
                    if (afterPingNum == 294) afterPingNum = 0;
                    int prer = sqrt(x * x + y * y);
                    int afterr = prer + 1;
                    // cout << prePingNum<< " " << afterPingNum << " ";
                    // cout << prer << " " << afterr << " " <<endl;
                    int UA = matrix[prePingNum][afterr];
                    int UB = matrix[afterPingNum][afterr];
                    int UC = matrix[prePingNum][prer];
                    int UD = matrix[afterPingNum][prer];
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE+0.5;
                }
                
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=117;
                if(UT<0) UT=0;

                imageCubic.at<uchar>(row, col) = UT;
                // uchar *data = imageCubic.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageCubic);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
	int w = imageCubic.cols;
	int h = imageCubic.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageCubic, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
}

//双立方插值 步距2 扇形成像
void sonarImage_Cubic_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageSector, 
                            int pingBegin, int pingEnd) 
{
    // initColortable();

    cv::Mat imageCubic(600,600, CV_8UC1, cv::Scalar(255));//插值成像图片
    double dAngle = 2 * PI / 540.0; // rad  步距角
    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;  //精确波束号
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;
            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if(r>=4 && r<=290) {
                    int prePingNum = angle / dAngle;
                    int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)(E(0,0)+0.5);

                }
                else {
                    int prePingNum = angle / dAngle;
                    int afterPingNum = prePingNum + 1;
                    if (afterPingNum == 540) afterPingNum = 0;
                    int prer = sqrt(x * x + y * y);
                    int afterr = prer + 1;
                    // cout << prePingNum<< " " << afterPingNum << " ";
                    // cout << prer << " " << afterr << " " <<endl;
                    int UA = matrix[prePingNum][afterr];
                    int UB = matrix[afterPingNum][afterr];
                    int UC = matrix[prePingNum][prer];
                    int UD = matrix[afterPingNum][prer];
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    int UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE;
                }
                
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=115;
                if(UT<0) UT=0;

                imageCubic.at<uchar>(row, col) = UT;
                // uchar *data = imageCubic.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageCubic);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/540.0-120; //旋转角度
	int w = imageCubic.cols;
	int h = imageCubic.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageCubic, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
}

void sonarImage_Impro_gray_step2(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd)
{
    // initColortable();

    cv::Mat imageImpro(600,600, CV_8UC1, cv::Scalar(0));//插值成像图片
    double dAngle = 2 * PI / 294.0; // rad  步距角

    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;    //精确波束号,double
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;//强度值

            int prePingNum = angle / dAngle;    //向下取整
            int afterPingNum = prePingNum + 1;  //向上取整
            if (afterPingNum == 294) afterPingNum = 0;
            int prer = sqrt(x * x + y * y);
            int afterr = prer + 1;

            int UA = matrix[prePingNum][afterr];
            int UB = matrix[afterPingNum][afterr];
            int UC = matrix[prePingNum][prer];
            int UD = matrix[afterPingNum][prer];

            int EA = edgeMatrix[prePingNum][afterr];
            int EB = edgeMatrix[afterPingNum][afterr];
            int EC = edgeMatrix[prePingNum][prer];
            int ED = edgeMatrix[afterPingNum][prer];

            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if((EA==0 && EB==0 && EC==0 && ED==0) || (r<=4) || (r>=296))  //不为边缘像素点：用周向和径向线性插值
                {
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE+0.5;
                }
                else {  //位于边缘像素点：用复杂方法插值
                
                    // int prePingNum = angle / dAngle;
                    // int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)(E(0,0)+0.5);

                }
                
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=127;
                if(UT<0) UT=0;

                imageImpro.at<uchar>(row, col) = UT*2;
                // uchar *data = imageImpro.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    // imshow("image", imageImpro);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

    // int rotateAngle = pingBegin*360.0/294.0-120; //旋转角度
    int rotateAngle = (pingBegin+pingEnd)/2.*360.0/294.0-180;
	int w = imageImpro.cols;
	int h = imageImpro.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageImpro, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(0));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
}

void sonarImage_Impro_gray_step2_540(vector<vector<uchar>>& matrix, cv::Mat& imageSector, vector<vector<uchar>>& edgeMatrix, 
                            int pingBegin, int pingEnd)
{
    // initColortable();

    cv::Mat imageImpro(600,600, CV_8UC1, cv::Scalar(255));//插值成像图片
    double dAngle = 2 * PI / 540.0; // rad  步距角

    for (int y = 300; y > -300; y--)
    {
        for (int x = -300; x < 300; x++)
        {
            double r = sqrt(x * x + y * y); //精确行号
            if (r >= 299)
                continue;
            double angle; // rad 与y轴正方向的夹角
            if (y < 0)
            {
                angle = atan(x * 1.0 / y) + PI;
            }
            else if (y == 0)
            {
                if (x < 0)
                    angle = 1.5 * PI;
                else
                    angle = 0.5 * PI;
            }
            else
            {
                if (x < 0)
                    angle = atan(x * 1.0 / y) + 2 * PI;
                else
                    angle = atan(x * 1.0 / y);
            }
            double pingNum = angle / dAngle;  //精确波束号
            //double r=sqrt(x * x + y * y);     //精确行号
            int UT=0;//强度值

            int prePingNum = angle / dAngle;
            int afterPingNum = prePingNum + 1;
            if (afterPingNum == 540) afterPingNum = 0;
            int prer = sqrt(x * x + y * y);
            int afterr = prer + 1;

            int UA = matrix[prePingNum][afterr];
            int UB = matrix[afterPingNum][afterr];
            int UC = matrix[prePingNum][prer];
            int UD = matrix[afterPingNum][prer];

            int EA = edgeMatrix[prePingNum][afterr];
            int EB = edgeMatrix[afterPingNum][afterr];
            int EC = edgeMatrix[prePingNum][prer];
            int ED = edgeMatrix[afterPingNum][prer];

            if(pingNum>=pingBegin && pingNum<=pingEnd){
                if((EA==0 && EB==0 && EC==0 && ED==0) || (r<=4) || (r>=296)) {
                    double UE = (afterr - r) * UC + (1 - (afterr - r)) * UA;
                    double UF = (afterr - r) * UD + (1 - (afterr - r)) * UB;
                    UT = (pingNum - prePingNum) * UF + (1 - (pingNum - prePingNum)) * UE+0.5;
                }
                else {
                    // int prePingNum = angle / dAngle;
                    // int prer = sqrt(x * x + y * y);  //左下

                    double weight_angle=pingNum-prePingNum;
                    double weight_distance=prer+1-r;

                    Eigen::Matrix<float, 1, 4> A;
                    //MatrixXf A(1,4);
                    A<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                       func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);
                    // A<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                    //    func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);

                    Matrix4f B;
                    B<<matrix[prePingNum-1][prer+2], matrix[prePingNum][prer+2], matrix[prePingNum+1][prer+2], matrix[prePingNum+2][prer+2],
                    matrix[prePingNum-1][prer+1], matrix[prePingNum][prer+1], matrix[prePingNum+1][prer+1], matrix[prePingNum+2][prer+1],
                    matrix[prePingNum-1][prer], matrix[prePingNum][prer], matrix[prePingNum+1][prer], matrix[prePingNum+2][prer],
                    matrix[prePingNum-1][prer-1], matrix[prePingNum][prer-1], matrix[prePingNum+1][prer-1], matrix[prePingNum+2][prer-1];
                    
                    Eigen::Matrix<float, 4, 1> C;
                    //MatrixXf C(4,1);
                    C<<func_Cubic(1+weight_angle), func_Cubic(weight_angle),
                       func_Cubic(weight_angle-1), func_Cubic(weight_angle-2);
                    // C<<func_Cubic(1+weight_distance), func_Cubic(weight_distance),
                    //     func_Cubic(weight_distance-1), func_Cubic(weight_distance-2);

                    MatrixXf D(1,4);
                    D=A*B;

                    Eigen::Matrix<float, 1, 1> E;
                    E=D*C;

                    UT=(int)(E(0,0)+0.5);

                }
                
                int row = 300 - y;
                int col = x + 300;

                // if(UT>127) UT=127;
                if(UT<0) UT=0;

                imageImpro.at<uchar>(row, col) = UT;
                // uchar *data = imageImpro.ptr<uchar>(row);
                // data[3 * col] = colorTable[UT * 6 + 2];     // B
                // data[3 * col + 1] = colorTable[UT * 6 + 1]; // G
                // data[3 * col + 2] = colorTable[UT * 6];     // R
                
            }
        }
    }
    //imshow("image", imageCubic);
    //图片旋转裁剪
    Mat dst, M;//M为变换矩阵

    int rotateAngle = pingBegin*360.0/540.0-120; //旋转角度
	int w = imageImpro.cols;
	int h = imageImpro.rows;
	M = getRotationMatrix2D(Point2f(w / 2, h / 2), rotateAngle, 1.0);
 
	//C++的abs则可以自然支持对整数和浮点数两个版本（实际上还能够支持复数）
	double cos = abs(M.at<double>(0, 0));
	double sin = abs(M.at<double>(0, 1));
 
	int nw = w * cos + h * sin;
	int nh = w * sin + h * cos;
 
	//新图像的旋转中心
	M.at<double>(0, 2) += (nw / 2 - w / 2);
	M.at<double>(1, 2) += (nh / 2 - h / 2);
 
	warpAffine(imageImpro, dst, M, Size(nw,nh),INTER_CUBIC,0,Scalar(255));
 
	//imshow("旋转演示", dst);

    imageSector=dst(cv::Rect(nw/2-300,nh/2,600,300));
}