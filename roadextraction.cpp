#pragma comment (linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#include <iostream>

//#include "gdal.h"//GDALAllRegister()
#include "gdal_priv.h"
//#include "cpl_conv.h"

#include <windows.h>

#include "opencv2\highgui\highgui.hpp" 
#include "opencv\cv.h"

#include <string>  
#include <list>  
#include <vector>  
#include <map>  
#include <stack>  

#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2\core\operations.hpp"

#include "atltypes.h"
//#include <afxdlgs.h>

#include <stdio.h>
#include <io.h>
using namespace std;



void fillHole(const cv::Mat srcImg, cv::Mat &dstImg);//孔洞填充
int SeedFill(cv::Mat& _binImg, cv::Mat& _lableImg);//种子填充，返回区域数
void RegionGrow(cv::Mat &Img, cv::Mat &img_bilateral);//区域增长
cv::Mat thinning(cv::Mat image);//骨架提取
float PtDis(std::pair<int,int> pt1, std::pair<int,int> pt2);//两点距离


double find_distance(int no, CPoint point);
double find_curvature(int no, CPoint point);
double find_continuity(int no, CPoint point, double avg_distance);
CPoint find_min_energy(int no, CPoint point, double avg_distance);
void Snake_algorithm();

const double Pi = 3.14159265;


int roadextraction(const char* filePath,string outptsPath,float samdens)//返回影像
{
	/*char* filePath= "D:\\道路提取影像\\定边\\DOM\\dom-area56.TIF";
    string outptsPath= "E:\mysource\roadextraction\roadregion\testdata\2901.5-476.5.tif";
	float samdens=10.000000;*/

struct ImgParam //去掉extern
	{
		//  /*! Unknown or unspecified type */ 		    GDT_Unknown = 0,
		//	/*! Eight bit unsigned integer */           GDT_Byte = 1,
		//	/*! Sixteen bit unsigned integer */         GDT_UInt16 = 2,
		//	/*! Sixteen bit signed integer */           GDT_Int16 = 3,
		//	/*! Thirty two bit unsigned integer */      GDT_UInt32 = 4,
		//	/*! Thirty two bit signed integer */        GDT_Int32 = 5,
		//	/*! Thirty two bit floating point */        GDT_Float32 = 6,
		//	/*! Sixty four bit floating point */        GDT_Float64 = 7,
		//	/*! Complex Int16 */                        GDT_CInt16 = 8,
		//	/*! Complex Int32 */                        GDT_CInt32 = 9,
		//	/*! Complex Float32 */                      GDT_CFloat32 = 10,
		//	/*! Complex Float64 */                      GDT_CFloat64 = 11,
		//	/* maximum type # + 1 */ GDT_TypeCount = 12         
		int dataType; //文件类型
		double dataMax;//数据中最大值 最小值
		double dataMin;
		int width;//宽度
		int height;//高度 
		double utmX;//左上角的UTM 坐标
		double utmY;
		int zone;
		double pGeoTrans[6];//反射变换6参数 
		char sProjRef[1024];//地理信息描述
		double xRes;//X轴地面分辨率
		double yRes;//Y轴地面分辨率 
		char filename[250];
	}; //指向ImgParam


#pragma region  图像输入
	   //std::string filePath1 = "C:/Users/hj/Desktop/2901.5-476.5.tif";
	ImgParam param;
//	const char* filePath = (LPSTR)(LPCTSTR)filePath1;

	//std::string str1(filePath);

	/*std::string filePath2(filePath1.GetBuffer());
	const char* filePath=filePath2.c_str();*/

	/*std::string path=CStringToString(filePath1);
	const char* filePath=path.c_str();*/

#pragma endregion

#pragma region 数据读取
	//注册GDAL驱动
	GDALAllRegister();
	//中文路径支持
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	//读取文件DataSet
	GDALDataset* poDataset = (GDALDataset *)GDALOpen(filePath, GA_ReadOnly);//GA_Update改为GA_ReadOnly
	if (poDataset == NULL)
		return -1;

	int rasterCount = poDataset->GetRasterCount();//通道数量
	param.width = poDataset->GetRasterXSize();//图像大小
	param.height = poDataset->GetRasterYSize();
	GDALRasterBand *poBand;
	poBand = poDataset->GetRasterBand(1);//读取指向第1波段的指针
	param.dataType = poBand->GetRasterDataType();//获取数据类型

												 //数据抽样的最大最小值 
	double argout[2];
	if (param.dataType == GDT_Byte) {
		param.dataMin = 0;
		param.dataMax = 255;
	}
	else
	{
		poBand->ComputeRasterMinMax(0, argout);
		param.dataMin = argout[0];
		param.dataMax = argout[1];
	}

	double adfGeoTransform[6];//影像六参数（省略[2][4]-旋转参数）
	if (poDataset->GetGeoTransform(adfGeoTransform) == CE_None)
	{
		std::cout << "提取影像六参数" << std::endl;
	}

	////经纬度信息
	//strcpy(param.sProjRef, poDataset->GetProjectionRef());
	////转换参数
	//poDataset->GetGeoTransform(param.pGeoTrans);
	////取得地面分辨率
	//param.xRes = param.pGeoTrans[1];
	//param.yRes = param.pGeoTrans[5];
	////取得左上角UTM坐标
	//param.utmX = param.pGeoTrans[0];
	//param.utmY = param.pGeoTrans[3];
	//double centerLon = GetParaValue(param.sProjRef, "central_meridian");
	//param.zone = (int)((centerLon + 180.0) / 6.0 + 1.0);
	//可以输出这些信息看看

#pragma endregion  

#pragma region 开辟内存
	int lineLength = param.width * sizeof(char);//lineLenth:每行所占字节长度 
	int readBufferLength = 0;//readBufferLenth:每行所占字节长度
	char* dataBuffer = NULL;//dataBuffer:指向每行开头的指针
	unsigned char* dataChangeBuffer = NULL;//dataChangeBuffer:指向每行开头的指针
	if (param.dataType == GDT_Byte)
		readBufferLength = param.width * sizeof(char);
	else if (param.dataType == GDT_UInt16)
		readBufferLength = param.width * sizeof(short);
	dataBuffer = new char[readBufferLength];
	dataChangeBuffer = new unsigned char[param.width];
#pragma endregion

#pragma region 获得物理内存参数
	MEMORYSTATUS MemoryStatus;//MEMORYSTATUS结构反应内存信息
	GlobalMemoryStatus(&MemoryStatus);//函数的返回信息被存储在MEMORYSTATUS结构中
	DWORD  dataLenth = lineLength*param.height;//数据占字节数
	char* imgBuffer = NULL;//imgBuffer:指向整个图像的指针
						   //bool hasUnmapViewOfFile = false;
						   //HANDLE fileHandle, fileMemMapping;

						   //如果图像小于2G 并且物理可用内存大于图像内存
	if (dataLenth < 1024 * 1024 * 1024 * 1.5 && MemoryStatus.dwAvailPhys > dataLenth)
		imgBuffer = new char[dataLenth];//imgBuffer：图像数据块

										//else//如果图像大于2G，并且可用物理内存大于图像内存
										//{
										//	char tempPath[MAX_PATH] = {0},memFilePath[MAX_PATH]= {0},path[MAX_PATH] = {0};
										//	//GetTempPath(MAX_PATH,tempPath); 
										//	GetTempFileName(path, // 临时文件目录
										//		"NEW",                    // 临时文件文的前缀
										//		0,                        // 创建唯一的名字
										//		memFilePath);              // 保存名字的缓冲
										//	fileHandle = CreateFile(TEXT(memFilePath),
										//		GENERIC_READ | GENERIC_WRITE,// 文件访问权限,写
										//		0,//共享模式,这里设置0防止其他进程打开文件或设备
										//		NULL,//SECURITY_ATTRIBUTES结构，安全描述，这里NULL代表默认安全级别
										//		CREATE_ALWAYS,//对于存在或不存在的设置执行的操作，这里是始终创建
										//		FILE_ATTRIBUTE_NORMAL|FILE_FLAG_DELETE_ON_CLOSE,//设置文件的属性，里面有高速缓存的选项
										//		NULL);//这里输入需要复制的文件 src 
										//	if(fileHandle == NULL) 
										//	{ 
										//		printf( "临时文件新建失败\n"); // 显示信息 
										//		return -2; 
										//	}
										//	fileMemMapping = CreateFileMapping(fileHandle, 
										//		NULL, 
										//		PAGE_READWRITE, 
										//		0,
										//		(DWORD)dataLenth,
										//		0);
										//	if (fileMemMapping == NULL){ 
										//		printf("错误发生：%d",GetLastError());
										//		return -2;
										//	}
										//	imgBuffer = (char* )MapViewOfFile( fileMemMapping, 
										//		FILE_MAP_ALL_ACCESS,0,0,0);

										//	hasUnmapViewOfFile = true;
										//}
#pragma endregion


#pragma region 取数据给imgBuffer
	double ratio = 255.0 / (param.dataMax - param.dataMin);
	uint64 allCnt = 0;
	for (int j = 0; j < param.height; j++) {
		CPLErr err = poBand->RasterIO(GF_Read,
			0,
			j,
			param.width,
			1,
			dataBuffer,
			param.width,
			1,
			(GDALDataType)param.dataType, 0, 0);

		if (param.dataType == GDT_Byte) {
			memcpy(imgBuffer + allCnt, dataBuffer, param.width);
		}
		else
		{
			unsigned short * dataShortBuffer = (unsigned short *)dataBuffer;
			for (int i = 0; i < param.width; i++) {
				dataChangeBuffer[i] = (unsigned char)std::min<int>((dataShortBuffer[i] - param.dataMin) * ratio, 255);
			}
			memcpy(imgBuffer + allCnt, dataChangeBuffer, param.width);
		}
		allCnt += param.width;
	}
	delete[] dataBuffer; //释放多余内存
	delete[] dataChangeBuffer;
	//关闭数据
	poDataset->FlushCache();//刷新缓存，为后面数据留出cache空间
	GDALClose(poDataset);
	poDataset = NULL;
#pragma endregion


#pragma region 预处理
	cv::Mat imageAll = cv::Mat(param.height, param.width, CV_8U, imgBuffer);//构造函数初始化imageALL

	std::cout << "start processing..." << std::endl;
	//重采样
	// cv::Mat_<uchar> img_resample;
	// cv::resize(imageAll, img_resample, cv::Size(imageAll.rows / 1, imageAll.cols / 1), 0, 0, 1);
	//高斯滤波 
	cv::Mat img_gauss;
	cv::Size size(5, 5);
	cv::GaussianBlur(imageAll, img_gauss, size, 1.0, 1.0);//去噪
    //cv::imwrite("C:/Users/hj/Desktop/gauss.tif", img_gauss);
	//双边滤波
	delete[] imgBuffer;
	imgBuffer = NULL;
	cv::Mat img_bilateral;
	cv::bilateralFilter(img_gauss, img_bilateral, 5, 20.0, 20.0);//保边去噪
	//cv::imwrite("C:/Users/hj/Desktop/bilateral.tif", img_bilateral);
#pragma endregion 

#pragma region 算法处理

#pragma region 二值化
	const float threshold = 25;
	cv::Mat_<uchar> erzhi(img_bilateral.rows, img_bilateral.cols);
	erzhi = 0;
	//5-邻域内灰度一致
	const uchar MINGRAY_ROAD = 150;
	for (int i = 5; i < img_bilateral.rows - 5; i++)
		for (int j = 5; j < img_bilateral.cols - 5; j++)
		{
			int gray_near[5] = { false,false,false,false,false };
			for (int k = 1; k < 6; k++)
			{
				if (img_bilateral.at<uchar>(i, j) > MINGRAY_ROAD &&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i - k, j - k)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i - k, j)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i - k, j + k)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i, j - k)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i, j + k)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i + k, j - k)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i + k, j)) < threshold&&
					abs(img_bilateral.at<uchar>(i, j) - img_bilateral.at<uchar>(i + k, j + k)) < threshold
					)
					gray_near[k - 1] = true;
			}
			if (gray_near[0] && gray_near[1] && gray_near[2] && gray_near[3] && gray_near[4])
				erzhi.at<uchar>(i, j) = 255;
		}
#pragma endregion

	//降采样
   // cv::imwrite("C:/Users/hj/Desktop/erzhi.tif", erzhi);
	/*uchar* data = erzhi.ptr<uchar>(1);
	data[2] = 200;*/
	/*cv::Mat_<uchar> erzhi_dsam;
	cv::resize(erzhi, erzhi_dsam, cv::Size(erzhi.rows / 1, erzhi.cols / 1), 0, 0, 1);
	for (int i = 0; i < erzhi_dsam.rows; i++)
		for (int j = 0; j < erzhi_dsam.cols; j++)
		{
			if (erzhi_dsam.at<uchar>(i, j) > 0)
				erzhi_dsam.at<uchar>(i, j) = 255;
		}*/

#pragma region 小区域剔除
	cv::Mat seed;
	int regions_n;//区域数
	regions_n = SeedFill(erzhi, seed);
	//std::vector<Region> rgs(regions_n);
	std::vector<std::vector<std::pair<int, int>>> rgs(regions_n);//各区域点集
	//int* perregion_n = new int[regions_n];//各区域点数
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n[i] = 0;

	for (int i = 1; i < seed.rows - 1; i++)
		for (int j = 1; j < seed.cols - 1; j++)
		{
			if (seed.at<int>(i, j) > 255)
			{
				rgs[seed.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//点集合点位
				//perregion_n[seed.at<int>(i, j) - 256]++;
			}
		}
	cv::Mat_<int> BigR;//作后续处理
	BigR = seed.clone();
	for (int i = 0; i < regions_n; i++)
	{
		if (rgs[i].size() > 250)
		{
			for (int k = 0; k < rgs[i].size(); k++)
				BigR.at<int>(rgs[i][k].first, rgs[i][k].second) = 255;

			//	 rgs_big.push_back(rgs[i]);
		}
		else
			for (int k = 0; k < rgs[i].size(); k++)
				BigR.at<int>(rgs[i][k].first, rgs[i][k].second) = 0;
	}

	//delete[] perregion_n;
#pragma  endregion


#pragma region  形态学处理
	cv::Mat Kernel;
	Kernel = cv::getStructuringElement(0, cv::Size(5, 5));//矩形结构

   /* cv::Mat_<uchar> erd(erzhi_dsam.rows, erzhi_dsam.cols);
	cv::erode(erzhi_dsam, erd, Kernel, cv::Point(-1, -1), 3);*/
	//	 cv::imwrite("C:/Users/hj/Desktop/erode.tif", erd);

	cv::Mat_<uchar> erd(BigR.rows, BigR.cols);
	cv::Mat_<uchar> dlt(BigR.rows, BigR.cols);
	erd = BigR.clone();
	for (int i = 0; i < 20; i++)
	{
		cv::dilate(erd, dlt, Kernel, cv::Point(-1, -1), 1);
		cv::erode(dlt, erd, Kernel, cv::Point(-1, -1), 1);
	}


	/*  cv::Mat_<uchar> erd1(BigR.rows, BigR.cols);
	 cv::erode(erd, erd1, Kernel1, cv::Point(-1, -1), 1);*/
#pragma endregion
//
#pragma region 过大过小区域剔除
	cv::Mat seed1;
	regions_n = SeedFill(erd, seed1);
	std::vector<std::vector<std::pair<int,int>>> remain(regions_n);

	//3.区域初始化（可考虑在种子填充时初始化，效率会高些）
	//int* perregion_n1 = new int[regions_n];//各区域点数
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n1[i] = 0;

	for (int i = 1; i < seed1.rows - 1; i++)
		for (int j = 1; j < seed1.cols - 1; j++)
		{
			if (seed1.at<int>(i, j) > 255)
			{
				remain[seed1.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//点集合点位
				//perregion_n1[seed1.at<int>(i, j) - 256]++;
			}
		}

	//delete[] perregion_n1;

	cv::Mat_<int> BigR1;
	BigR1 = seed1.clone();
	const int THREPTS_MIN = 1500;//区域点数最小阈值
	const int THREPTS_MAX = 250000;//区域点数最大阈值
	for (int i = 0; i < regions_n; i++)
	{
		if (remain[i].size() > THREPTS_MIN && remain[i].size() < THREPTS_MAX)
		{
			for (int k = 0; k < remain[i].size(); k++)
				BigR1.at<int>(remain[i][k].first, remain[i][k].second) = 255;
		}
		else
			for (int k = 0; k < remain[i].size(); k++)
				BigR1.at<int>(remain[i][k].first, remain[i][k].second) = 0;
	}
	//	imwrite("C:/Users/hj/Desktop/bigregion.tif", BigR);
	

	cv::Mat BigR_uc1;
	BigR1.convertTo(BigR_uc1, CV_8UC1);//int to uchar

#pragma endregion

//腐蚀、膨胀

			/*cv::Mat_<uchar> dlt1(BigR_uc.rows, BigR_uc.cols);
			cv::dilate(BigR_uc, dlt1, Kernel, cv::Point(-1, -1), 1);*/
			//	cv::imwrite("C:/Users/hj/Desktop/dilate1.tif", dlt1);

				/*cv::Mat_<uchar> ero1(BigR_uc.rows, BigR_uc.cols);
				cv::erode(BigR_uc, ero1, Kernel, cv::Point(-1, -1), 3);*/

				//cvThreshold(dlt1, dlt3, 120, 255, CV_THRESH_BINARY);
			// Canny(dlt1, dlt1, 60, 200);

#pragma region 区域形状特征计算检测

	struct Region_Feature{
		int label;//标记值

		double Area;//区域面积
		double Perimeter;//区域周长
		double AP;//复杂度 Perimeter*Perimeter / Area(本质为道路长宽比)
		double PA;//面积比周长

		//cv::RotatedRect MinRec;//最小外接矩形

		double Lmer;//外接矩形长
		double Wmer;//外接矩形宽
		double LW;//长宽比 Lmer / Wmer

		double Smer;//最小外接矩形面积 Lmer * Wmer

		double Fdegree;//充满度 Smer/Area
	};
	
	cv::Mat_<uchar> newimg;
	newimg = BigR_uc1.clone();

	std::vector<cv::Mat> contours(1000);
	// std::vector<cv::Vec4i> hierarchy;
	cv::findContours(newimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//char *g_result = "result.txt";
	//FILE *pf = fopen(g_result, "wt");
	//for (int i = 0; i < contours.size(); i++)
	//{
	//	for (int j = 0; j < contours[i].size(); j++)
	//	{
	//		fprintf(pf, "%d %d\n", contours[i][j].x, dlt2.rows - contours[i][j].y);
	//	}
	//}
	//fclose(pf);

	cv::Mat Contoursimg;
	Contoursimg = newimg.clone();
	for(int i=0;i<Contoursimg.rows;i++)
		for (int j = 0; j < Contoursimg.cols; j++)
		{
			if (Contoursimg.at<uchar>(i, j) > 1)
				Contoursimg.at<uchar>(i, j) = 255;
		}

	const cv::Scalar color(255);
	//画轮廓
	/*for (int idx = 0; idx < contours.size(); idx++)
	{
		cv::drawContours(Contoursimg, contours, idx, color, 1, 8);
	}*/


	//std::vector<Region> rgs1;
	//rgs1.resize(contours.size());
	Region_Feature *rgs1;
	rgs1 = new Region_Feature[contours.size()];
	//外接矩形长宽，长宽比，面积，区域面积，周长，复杂度，充满度
	std::vector<cv::RotatedRect> Rects;


	for (int i = 0; i < contours.size(); i++)
	{
		Rects.push_back(cv::minAreaRect(contours[i]));
		rgs1[i].Wmer = std::min(Rects[i].size.width, Rects[i].size.height);
		rgs1[i].Lmer = std::max(Rects[i].size.width, Rects[i].size.height);
		rgs1[i].LW = rgs1[i].Lmer / rgs1[i].Wmer;//外接矩形长宽比

		rgs1[i].Area = cv::contourArea(contours[i], false);
		rgs1[i].Perimeter = cv::arcLength(contours[i], true);
		rgs1[i].AP = rgs1[i].Perimeter*rgs1[i].Perimeter / rgs1[i].Area;//复杂度
		rgs1[i].PA = rgs1[i].Area / rgs1[i].Perimeter;//面积比周长

		rgs1[i].Fdegree = rgs1[i].Wmer * rgs1[i].Lmer / rgs1[i].Area;//充满度
	}
	//delete[] rgs1;
	//6.区域筛选
	const double threshold_AP = 75.0;//复杂度
	const double threshold_LW = 3.0;//长宽比
	const double threshold_Fdegree = 3.0;//充满度

	//（1）曲线道路检测
	std::vector<cv::Mat> rgs_cur;
	//rgs_cur.reserve(rgs1.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (rgs1[i].AP > threshold_AP && rgs1[i].Fdegree > threshold_Fdegree)
		{
			rgs_cur.push_back(contours[i]);
		}
	}
	cv::Mat_<uchar> regions_cur(newimg.rows, newimg.cols);
	cv::drawContours(regions_cur, rgs_cur, -1, color, -1, 8);

	/*
	 for(int i=0;i<rgs_cur.size();i++)
	 {
		 std::cout << "S/L:" << cv::contourArea(rgs_cur[i], false) / cv::arcLength(rgs_cur[i], true) << std::endl;
	 }*/

	 //(2)直线道路检测
	 //&&rgs[i].LW > threshold_LW&&rgs[i].Fdegree > threshold_Fdegree
	std::vector<cv::Mat> rgs_line;
	//rgs_line.reserve(rgs1.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (rgs1[i].AP > threshold_AP && rgs1[i].LW > threshold_LW)
		{
			rgs_line.push_back(contours[i]);
		}
	}
	cv::Mat_<uchar> regions_line(newimg.rows, newimg.cols);
	cv::drawContours(regions_line, rgs_line, -1, color, -1, 8);

	/*for (int i = 0; i < rgs_line.size(); i++)
	{
		std::cout << "S/L:" << cv::contourArea(rgs_line[i], false) / cv::arcLength(rgs_line[i], true) << std::endl;;
	}*/

	//(3)两线相合
	cv::Mat_<uchar> regions_merge(newimg.rows, newimg.cols);
	for (int i = 0; i < regions_merge.rows; i++)
		for (int j = 0; j < regions_merge.cols; j++)
		{
			if (regions_line.at<uchar>(i, j) == 255 || regions_cur.at<uchar>(i, j) == 255)
				regions_merge.at<uchar>(i, j) = 255;
			else
				regions_merge.at<uchar>(i, j) = 0;
		}
	delete[] rgs1;
	
#pragma endregion

#pragma region 噪声剔除

#pragma endregion

#pragma region 道路区域增长(过增长问题)
	cv::Mat regiongrow_p;
	regiongrow_p = regions_merge.clone();
	RegionGrow(regiongrow_p, img_bilateral);
	for (int i = 0; i < regiongrow_p.rows; i++)
		for (int j = 0; j < regiongrow_p.cols; j++)
		{
			if (regiongrow_p.at<uchar>(i, j) == 1)
				regiongrow_p.at<uchar>(i, j) = 255;
		}
	//弥补空洞
	cv::Mat regiongrow;
	fillHole(regiongrow_p, regiongrow);
#pragma endregion 

#pragma region 区域特征再检测
	cv::Mat_<uchar> newimg1;
	newimg1 = regiongrow.clone();
	std::vector<cv::Mat> contours1(100);
	cv::findContours(newimg1, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	cv::Mat Contoursimg1;
	Contoursimg1 = newimg.clone();
	for (int i = 0; i < Contoursimg1.rows; i++)
		for (int j = 0; j < Contoursimg1.cols; j++)
		{
			if (Contoursimg1.at<uchar>(i, j) > 1)
				Contoursimg1.at<uchar>(i, j) = 255;
		}

	//std::vector<Region> rgs3;
	//rgs3.resize(contours1.size());
	Region_Feature *rgs3;
	rgs3 = new Region_Feature[contours1.size()];

	//外接矩形长宽，长宽比，面积，区域面积，周长，复杂度，充满度
	std::vector<cv::RotatedRect> Rects1;
	for (int i = 0; i < contours1.size(); i++)
	{
		Rects1.push_back(cv::minAreaRect(contours1[i]));
		rgs3[i].Wmer = std::min(Rects1[i].size.width, Rects1[i].size.height);
		rgs3[i].Lmer = std::max(Rects1[i].size.width, Rects1[i].size.height);
		rgs3[i].LW = rgs3[i].Lmer / rgs3[i].Wmer;//外接矩形长宽比

		rgs3[i].Area = cv::contourArea(contours1[i], false);
		rgs3[i].Perimeter = cv::arcLength(contours1[i], true);
		rgs3[i].AP = rgs3[i].Perimeter*rgs3[i].Perimeter / rgs3[i].Area;//复杂度
		rgs3[i].PA = rgs3[i].Area / rgs3[i].Perimeter;//面积比周长

		rgs3[i].Fdegree = rgs3[i].Wmer * rgs3[i].Lmer / rgs3[i].Area;//充满度
	}


	//(1) 曲线道路检测
	std::vector<cv::Mat> rgs_cur1;
	//rgs_cur1.reserve(rgs3.size());
	for (int i = 0; i < contours1.size(); i++)
	{
		if (rgs3[i].AP > threshold_AP && rgs3[i].Fdegree > threshold_Fdegree)
		{
			rgs_cur1.push_back(contours1[i]);
		}
	}
	cv::Mat_<uchar> regions_cur1(newimg1.rows, newimg1.cols);
	cv::drawContours(regions_cur1, rgs_cur1, -1, color, -1, 8);
	//(2) 直线道路检测
	std::vector<cv::Mat> rgs_line1;
	//rgs_line1.reserve(rgs3.size());
	for (int i = 0; i < contours1.size(); i++)
	{
		if (rgs3[i].AP > threshold_AP && rgs3[i].LW > threshold_LW+2)
		{
			rgs_line1.push_back(contours1[i]);
		}
	}
	cv::Mat_<uchar> regions_line1(newimg1.rows, newimg1.cols);
	cv::drawContours(regions_line1, rgs_line1, -1, color, -1, 8);

	cv::Mat_<uchar> regions_merge1(newimg1.rows, newimg1.cols);
	for (int i = 0; i < regions_merge1.rows; i++)
		for (int j = 0; j < regions_merge1.cols; j++)
		{
			if (regions_line1.at<uchar>(i, j) == 255 || regions_cur1.at<uchar>(i, j) == 255)
				regions_merge1.at<uchar>(i, j) = 255;
			else
				regions_merge1.at<uchar>(i, j) = 0;
		}
	delete[] rgs3;
#pragma endregion

	//#pragma region 路宽检测
	//	cv::Mat regions_AP_seed;
	//	regions_n = SeedFill(regiongrow, regions_AP_seed);
	//
	//	int* perregion_n2 = new int[regions_n];//各区域点数
	//	for (int i = 0; i < regions_n; i++)
	//		perregion_n2[i] = 0;
	//
	//	for (int i = 1; i < regions_AP_seed.rows - 1; i++)
	//		for (int j = 1; j < regions_AP_seed.cols - 1; j++)
	//		{
	//			if (regions_AP_seed.at<int>(i, j) > 255)
	//			{
	//				rgs1[regions_AP_seed.at<int>(i, j) - 256].pts_p.push_back(std::pair<int, int>(i, j));//点集合点位
	//				perregion_n2[regions_AP_seed.at<int>(i, j) - 256]++;
	//			}
	//		}
	//
	//	for (int i = 0; i < regions_n; i++)//各区域点数
	//		rgs1[i].points_n = perregion_n2[i];
	//
	//	delete[] perregion_n2;
	//
	//	std::vector<Region> temp;
	//	
	//	const int direction_n = 12;//方向数
	//	//float *road_width =new float[regions_n];
	//	std::vector<int> road_width;
	//	for (int i = 0; i < regions_n; i++)//每条线
	//	{
	//		//std::cout << std::endl;
	//		std::pair<int, int> sample_p[3];//起始样本点(取三个)
	//		float maxdis[3] = { 10.0 ,10.0,10.0 };//方向线段最长值
	//		float mindis[3] = { 100.0 ,100.0,100.0 };//方向线段最短值
	//		//bool is_appro_samplept = false;
	//		for (int j = 0; j < 3; j++)
	//		{//判断是否为边缘点
	//			sample_p[j].first = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1)].first;
	//			sample_p[j].second = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1)].second;
	//			int p = 0;
	//			while (!(regiongrow.at<uchar>(sample_p[j].first + 5, sample_p[j].second + 5) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first, sample_p[j].second + 5) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first + 5, sample_p[j].second) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first - 5, sample_p[j].second - 5) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first, sample_p[j].second - 5) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first - 5, sample_p[j].second) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first + 5, sample_p[j].second - 5) != 0 &&
	//				regiongrow.at<uchar>(sample_p[j].first - 5, sample_p[j].second + 5) != 0) && p < rgs1[i].points_n / 4)
	//			{
	//				p++;
	//				sample_p[j].first = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1) + p].first;//保证其变成非边缘附近点
	//				sample_p[j].second = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1) + p].second;
	//			//	is_appro_samplept = true;
	//			}
	//			//else
	//			//{
	//			//	sample_p[j].first = rgs1[i].pts_p[rgs1[i].points_n / 6 * (2 * j + 1) + 5].first;//+5可保证其变成非边缘点
	//			//	sample_p[j].second = rgs1[i].pts_p[rgs1[i].points_n / 6 * (2 * j + 1) + 5].second;
	//			//	is_appro_samplept = true;
	//			//}
	//		/*	if (is_appro_samplept)
	//			 {*/
	//				std::pair<double, double> sample_p_next1;//正方向端点
	//				std::pair<double, double> sample_p_next2;//反方向端点
	//				float *dis_p = new float[direction_n];//每个方向道路宽
	//				for (int n = 0; n < direction_n; n++)//每个方向
	//				{
	//					int count1 = 0;
	//					int count2 = 0;
	//					bool n1 = true;
	//					bool n2 = true;
	//					while (n1)//正方向
	//					{
	//						count1++;
	//						sample_p_next1.first = (double)sample_p[j].first + count1*cos(Pi / direction_n*n);
	//						sample_p_next1.second = (double)sample_p[j].second + count1*sin(Pi / direction_n*n);
	//						if (regiongrow.at<uchar>((int)sample_p_next1.first, (int)sample_p_next1.second) != 255)
	//							n1 = false;
	//					}
	//					while (n2)//反方向
	//					{
	//						count2++;
	//						sample_p_next2.first = (double)sample_p[j].first - count2*cos(Pi / direction_n*n);
	//						sample_p_next2.second = (double)sample_p[j].second - count2*sin(Pi / direction_n*n);
	//						if (regiongrow.at<uchar>((int)sample_p_next2.first, (int)sample_p_next2.second) != 255)
	//							n2 = false;
	//					}
	//					dis_p[n] = PtDis(sample_p_next1, sample_p_next2);
	//					std::cout <<"第"<< n << "方向距离:" << dis_p[n] << std::endl;
	//				}
	//				for (int n = 0; n < direction_n; n++)//取各方向中最大的距离过小的剔除
	//				{
	//					if (dis_p[n] > maxdis[j])
	//						maxdis[j] = dis_p[n];
	//				}
	//				for (int n = 0; n < direction_n; n++)//取各方向中最小的距离为路宽(特殊情况(取到了边缘点及附近点)距离小于20忽略),剔除最小距离过大的
	//				{
	//					if (dis_p[n] < mindis[j] && dis_p[n]>20)
	//						mindis[j] = dis_p[n];
	//				}
	//				delete[] dis_p;
	//		//	}	
	//		}
	//		if (maxdis[0] > 80 && maxdis[1] > 80 && maxdis[2] > 80 && mindis[0]<50 && mindis[1]<50 && mindis[2]<50)
	//		{
	//			temp.push_back(rgs1[i]);
	//			road_width.push_back((mindis[0] + mindis[1] + mindis[2]) / 3);//筛选后道路存储结果
	//		}
	//	}
	//	for (int i = 0; i < rgs1.size(); i++)
	//	{
	//		rgs1[i].~Region();
	//	}
	//	/*for (int i = 0; i < road_width.size(); i++)
	//	{
	//		std::cout << "第" << i << "条:" << road_width[i] << std::endl;
	//	}*/
	//
	//	//干扰剔除
	//	cv::Mat_<uchar> widthde(regiongrow.rows, regiongrow.cols);
	//	widthde = 0;
	//	for (int i = 0; i < temp.size();i++)
	//		for (int j = 0; j < temp[i].pts_p.size(); j++)
	//		{
	//			widthde.at<uchar>(temp[i].pts_p[j].first, temp[i].pts_p[j].second) = 255;
	//		}
	//	for(int i=0;i<widthde.rows;i++)
	//		for (int j = 0; j < widthde.cols; j++)
	//		{
	//			if (widthde.at<uchar>(i, j) != 255)
	//				widthde.at<uchar>(i, j) = 0;
	//		}
	//	//widthde.convertTo(regions_merge,CV_8UC1);
	//
	//	//cv::Mat regions_merge_dlt;
	//	//regions_merge_dlt = regions_merge.clone();
	//	//cv::dilate(regions_merge, regions_merge, Kernel, cv::Point(-1, -1), 5);
	//#pragma endregion


#pragma region 骨架提取，剔除噪声
	cv::Mat regions_thin;
	regions_thin = thinning(regions_merge1);
	cv::Mat regions_thin1;

	regions_n = SeedFill(regions_thin, regions_thin1);
	std::vector<std::vector<std::pair<int, int>>> rgs2(regions_n);//各区域点集
	//std::vector<Region> rgs2(regions_n);
	//int* perregion_n3 = new int[regions_n];//各区域点数
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n3[i] = 0;

	for (int i = 1; i < regions_thin1.rows - 1; i++)
		for (int j = 1; j < regions_thin1.cols - 1; j++)
		{
			if (regions_thin1.at<int>(i, j) > 255)
			{
				rgs2[regions_thin1.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//点集合点位
				//perregion_n3[regions_thin1.at<int>(i, j) - 256]++;
			}
		}

	//delete[] perregion_n3;

	//噪声滤除
	std::vector<std::vector<std::pair<int, int>>> rgs_big1(regions_n);
	cv::Mat_<int> BigR2;
	BigR2 = regions_thin1.clone();

	for (int i = 0; i < regions_n; i++)
	{
		if (rgs2[i].size() > 300)
		{
			for (int k = 0; k < rgs2[i].size(); k++)
				BigR2.at<int>(rgs2[i][k].first, rgs2[i][k].second) = 255;

			rgs_big1.push_back(rgs2[i]);
		}
		else
			for (int k = 0; k < rgs2[i].size(); k++)
				BigR2.at<int>(rgs2[i][k].first, rgs2[i][k].second) = 0;
	}

	//delete[] perregion_n3;

	cv::Mat BigR_uc2;
	BigR2.convertTo(BigR_uc2, CV_8UC1);//int to uchar


#pragma region 去噪（：TODO）
	//1.各区域采样
	//2.曲线拟合
	//3.离散性计算（方差）

#pragma endregion
	//在骨架的基础上再细化（每行像素均不连续）
	cv::Mat seed2;
	regions_n = SeedFill(BigR_uc2, seed2);
	for (int i = 0; i < seed2.rows; i++)
	{
		std::vector<std::vector<std::pair<int, int>>> temp;
		temp.resize(seed2.cols);
		std::vector<int> label;//标记值
		label.resize(seed2.cols);
		int new_line = -1;//新增线
		int pre = 10000;//标记每行线上的前一亮点的列位置(j)
		bool has_pts = false;//每行线上是否有亮点
		for (int j = 0; j < seed2.cols; j++)
		{
			if (seed2.at<int>(i, j) > 255)
			{
				has_pts = true;
				if (j - pre == 1)
				{
					pre = j;
					temp[new_line].push_back(std::pair<int, int>(i, j));//点集分别存入temp
					label[new_line] = seed2.at<int>(i, j);
					seed2.at<int>(i, j) = 0;
				}
				else
				{
					pre = j;
					new_line++;
					temp[new_line].push_back(std::pair<int, int>(i, j));//点集分别存入temp
					label[new_line] = seed2.at<int>(i, j);
					seed2.at<int>(i, j) = 0;
				}
			}
		}
		if (has_pts)
		{
			for (int k = 0; k < temp.size(); k++)
			{
				if (temp[k].size() > 1)
				{
					{
						int sumx = 0;
						int sumy = 0;
						for (int p = 0; p < temp[k].size(); p++)
						{
							sumx += temp[k][p].first;
							sumy += temp[k][p].second;
						}
						seed2.at<int>(sumx / temp[k].size(), sumy / temp[k].size()) = label[k];
					}
				}
				if (temp[k].size() == 1)
				{
					seed2.at<int>(temp[k][0].first, temp[k][0].second) = label[k];
				}
			}
		}
	}

	//合并近距离点
	for (int i = 0; i < seed2.rows; i = i + 5)
	{
		std::vector<std::pair<int, int>> pts_temp;
		pts_temp.reserve(regions_n * 5);
		int pts_n = 0;
		for (int j = 0; j < seed2.cols; j++)//亮点存储
		{
			if (seed2.at<int>(i, j) > 255)
			{
				pts_n++;
				pts_temp.push_back(std::pair<int, int>(i, j));
				seed2.at<int>(i, j) = seed2.at<int>(i, j) + regions_n;
			}
		}
		if (!pts_temp.empty())
		{
			std::vector<std::pair<int, int>> pts_erase;//要合并的点
			pts_erase.reserve(regions_n * 5);
			for (int k = 0; k < pts_n - 1; k++)
				for (int p = k + 1; p < pts_n; p++)
				{
					if (abs(pts_temp[k].second - pts_temp[p].second) < 20)
					{
						pts_temp.push_back(std::pair<int, int>((pts_temp[k].first + pts_temp[p].first) / 2,
							(pts_temp[k].second + pts_temp[p].second) / 2));//取中点合并
						pts_erase.push_back(pts_temp[k]);
						pts_erase.push_back(pts_temp[p]);
					}
				}
				//点筛选
				if (!pts_erase.empty())
				{
					//for (int k = 0; k < pts_temp.size(); k++)
					//{
					//	seed2.at<int>(pts_temp[k].first, pts_temp[k].second)
					//		= seed2.at<int>(pts_temp[0].first, pts_temp[0].second);//
					//}
					for (int k = 0; k < pts_erase.size(); k++)
					{
						seed2.at<int>(pts_erase[k].first, pts_erase[k].second) = 0;
					}
				}
		}
	}
//seed2.convertTo(OutputImg,CV_8UC1);
//cv::imwrite("C:/Users/hj/Desktop/bilateral.tif",OutputImg);
#pragma region 待改进
	//每隔5行取点,控制距离（5-10）
	std::vector<std::vector<std::pair<int, int>>> region_pts;//存放筛选的点
	region_pts.resize(regions_n);

	std::vector<std::vector<std::vector<std::pair<int, int>>>> perline_pts;//*每行每个区域存储的点集
	perline_pts.resize((seed2.rows-1)/5+1);
	for (int i = 0; i < (seed2.rows - 1) / 5 + 1; i++)//每行区域数设置
		perline_pts[i].resize(regions_n);
	
	for (int i = 0; i < seed2.rows; i++)
		for (int j = 0; j < seed2.cols; j++)
		{
			if (seed2.at<int>(i, j) > 255 + regions_n)
			{
				perline_pts[i / 5][seed2.at<int>(i, j) - 256 - regions_n].push_back(std::pair<int, int>(i, j));
			}
			else
				seed2.at<int>(i, j) = 0;
		}

	const float Ptdensity=10;
	for (int r = 0; r < regions_n; r++)
	{
		for (int k = 0; k < (seed2.rows - 1) / 5; k++)
		{
			if (!perline_pts[k+1][r].empty() && !perline_pts[k][r].empty())
			{
				for (int p = 0; p < perline_pts[k + 1][r].size(); p++)
				{
					bool is_appr_pt=false;
					std::vector<float> dis;
					float mindis = 10000.0;
					for (int q = 0; q < perline_pts[k][r].size(); q++)
					{
						dis.push_back(PtDis(perline_pts[k+1][r][p], perline_pts[k][r][q]));
						if( *(dis.end()-1) > 5 && *(dis.end()-1) < Ptdensity && * (dis.end() - 1)== Ptdensity)
						is_appr_pt = true;
					}
					if (is_appr_pt)
					{
						region_pts[r].push_back(perline_pts[k + 1][r][p]);
					}
					else//找距离此点最近的点，然后均匀取点
					{
						float mindis = 10000.0;
						int label;//标记最小距离点的索引
						for (int a = 0; a < dis.size(); a++)
						{
							if (dis[a] < mindis)
							{
								mindis = dis[a];
								label = a;
							}
						}
						int segment_num = int(dis[label]) / Ptdensity;//内插点数
						if (segment_num < 3)
						{
							//内插
							std::pair<int, int>	minpt = perline_pts[k][r][label%perline_pts[k][r].size()];//距离最小点
							for (int b = 0; b < segment_num + 1; b++)
							{
								std::pair<int, int> temp_pt;
								temp_pt.first = perline_pts[k + 1][r][p].first + b * (float)(minpt.first - perline_pts[k + 1][r][p].first) / (float)(segment_num + 1);
								temp_pt.second = perline_pts[k + 1][r][p].second + b * (float)(minpt.second - perline_pts[k + 1][r][p].second) / (float)(segment_num + 1);
								region_pts[r].push_back(temp_pt);
							}
						}
					}
				}
			}
		}
	}

	cv::Mat_<int> seize(seed2.rows,seed2.cols);
	for (int i = 0; i < seize.rows; i++)
		for (int j = 0; j < seize.cols; j++)
			seize.at<int>(i,j) = 0;

	for (int i = 0; i < region_pts.size(); i++)
		for (int j = 0; j < region_pts[i].size(); j++)
			seize.at<int>(region_pts[i][j].first, region_pts[i][j].second) 
		= 255;
	//std::vector<std::vector<std::pair<int, int>>> perregion_lastpts;//上一行每个区域的点
	//for (int i = 0; i < seed2.rows; i=i+5)
	//{
	//	for (int j = 0; j < seed2.cols; j++)
	//	{
	//		if (seed2.at<int>(i, j) > 255)
	//		{
	//			if (!perregion_lastpts[seed2.at<int>(i, j) - 256].empty())
	//			{
	//				/*perregion_lastpts[seed2.at<int>(i, j) - 256].begin();*/
	//				std::vector<float> dis;
	//				bool dis_appro=false;
	//				/*std::vector<bool> dis_appro;
	//				dis_appro.resize(perregion_lastpts[seed2.at<int>(i, j) - 256].size(),false);*///判断每个点和之前行所有点的距离是否满足条件
	//				for (int k = 0; k < perregion_lastpts[seed2.at<int>(i, j) - 256].size(); k++)
	//				{
	//					dis.push_back(PtDis(perregion_lastpts[seed2.at<int>(i, j) - 256][k],std::pair<int,int>(i,j)));
	//				}
	//				for (int k = 0; k < perregion_lastpts[seed2.at<int>(i, j) - 256].size(); k++)
	//				{
	//					if (dis[k] < 10.0 && dis[k] > 5)
	//					{
	//						dis_appro = true;
	//					}
	//				}
	//				if (dis_appro)
	//				{
	//					preregion_pts[seed2.at<int>(i, j) - 256].push_back(std::pair<int, int>(i,j));
	//					perregion_lastpts[seed2.at<int>(i, j) - 256].clear();
	//					perregion_lastpts[seed2.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));
	//				}
	//			}
	//		}
	//	}
	//}
	std::vector<std::vector<std::pair<int, int>>> preregion_pts;//存放筛选的点
	preregion_pts.resize(regions_n);
	for (int i = 0; i < seed2.rows; i++)
		for (int j = 0; j < seed2.cols; j++)
		{
			if (seed2.at<int>(i, j) < 256+regions_n)
				seed2.at<int>(i, j) = 0;
		}
	//	//将不同线上点分别存入容器
	//	std::vector<std::vector<std::pair<int, int>>> pts_temp;
	//	pts_temp.resize(regions_n);
	//	for(int i=0;i<seed2.rows;i=i+5)
	//		for (int j = 0; j < seed2.cols; j++)
	//		{
	//			for (int k = 0; k < regions_n; k++)
	//			{
	//				if (seed2.at<int>(i, j) == 256 + regions_n + k)
	//				{
	//					pts_temp[k].push_back(std::pair<int,int> (i,j));
	//				}
	//			}
	//		}
	//	//每条线上点的排序
	//	std::vector<std::vector<std::pair<int, int>>> pts_obj;
	//	pts_obj.resize(regions_n);
	//	std::vector<std::pair<int, int>> pt_side(regions_n);//每条线上的端点容器
	//	//先找每条线上端点（利用最近邻两个点与此点连线的夹角判断即可）
	//	for (int k = 0; k < regions_n; k++)
	//	{
	//		for (int p = 0; p < pts_temp[k].size(); p++)
	//		{
	//			std::pair<int, int> minst1;
	//			std::pair<int, int> minst2;
	//			float *dis_cluster = new float[pts_temp[k].size()];
	//
	//			for (int h = 0; h < pts_temp[k].size(); h++)
	//				dis_cluster[h]=PtDis(pts_temp[k][p], pts_temp[k][h]);//距离簇
	//
	//			 dis_cluster[p]=10000;//让与自身的距离不最小
	//			 float min_dis = dis_cluster[p];
	//			 int index=p;
	//			for (int h = 0; h < pts_temp[k].size(); h++)//找最小距离
	//			{
	//				if (dis_cluster[h] < min_dis)
	//				{
	//					min_dis = dis_cluster[h];
	//					index = h;
	//				}
	//			}
	//			minst1 = pts_temp[k][index];//第一近邻点
	//			dis_cluster[index] = 10000;
	//			min_dis = dis_cluster[index];
	//			for (int h = 0; h < pts_temp[k].size(); h++)//找第二小距离
	//			{
	//				if (dis_cluster[h] < min_dis)
	//				{
	//					min_dis = dis_cluster[h];
	//					index = h;
	//				}
	//			}
	//			delete[]dis_cluster;
	//			minst2 = pts_temp[k][index];//第二近邻点
	//			double pt_pt1;
	//			double pt_pt2;
	//			pt_pt1=atan2(minst1.first, pts_temp[k][p].second);
	//			pt_pt2=atan2(minst2.first, pts_temp[k][p].second);
	//			if (abs(pt_pt1 - pt_pt2) > Pi / 3)
	//			{
	//				pt_side[k] = pts_temp[k][p];
	//			}
	//			break;//只找一个端点
	//		}
	//	}
	//	for (int k = 0; k < regions_n; k++)
	//	{
	//		seed2.at<int>(pt_side[k].first, pt_side[k].second)=255;
	//	}
	//
	//	/*for (int k = 0; k < regions_n; k++)
	//	{
	//		min_x[k].first = pts_temp[k][0].first;
	//		min_x[k].second = pts_temp[k][0].second;
	//		for (int p = 1; p < pts_temp[k].size(); p++)
	//		{
	//			if (pts_temp[k][p].first < min_x[k].first)
	//			{
	//				min_x[k].first = pts_temp[k][p].first;
	//				min_x[k].second = pts_temp[k][p].second;
	//			}
	//		}
	//	}*/
	//	//for (int k = 0; k < regions_n; k++)
	//	//{
	//	//	std::pair<int,int> mindis_pt;
	//	//	mindis_pt
	//	//	float min_dis = PtDis(min_x[k], pts_temp[k][0]);
	//	//	for (int p = 1; p < pts_temp[k].size(); p++)
	//	//	{
	//	//		//pts_obj[k].push_back(min_x[k]);
	//	//		if (PtDis(min_x[k], pts_temp[k][p]) < min_dis)
	//	//		{
	//
	//	//		}
	//	//	}
	//	//}
	#pragma endregion

#pragma region 后处理（道路地理坐标提取）
//cv::Mat BigR_uc1_resample;
////cv::resize(BigR_uc1, BigR_uc1_resample, cv::Size(BigR_uc1.rows / 4, BigR_uc1.cols / 4), 0, 0, 0);
std::vector < std::vector<std::pair < double, double>>> gp(regions_n);
for (int i = 0; i < seed2.rows; i=i+5)
for (int j = 0; j < seed2.cols; j++)
{
	if (seed2.at<int>(i, j) > 255 + regions_n)
	{
		std::pair<double, double> pair;
		pair.first = adfGeoTransform[0] + j*adfGeoTransform[1] + i*adfGeoTransform[2];
		pair.second = adfGeoTransform[3] + j*adfGeoTransform[4] + i*adfGeoTransform[5];
		gp[seed2.at<int>(i, j)-256-regions_n].push_back(pair);
	}
}
//write ptsfile
const char *g_result = "rst.txt";
FILE *pf = fopen(g_result, "wt");
for (int i = 0; i < gp.size(); i++)
{
	//fprintf(pf,"%s %d %s \n","第",i,"路段点：");
	for (int j = 0; j < gp[i].size(); j++)
	{
		fprintf(pf, "%f %f\n", gp[i][j].first, gp[i][j].second);
	}
	//fprintf(pf, "\n");
}
fclose(pf);
#pragma endregion

#pragma endregion

	//std::cout << "here" << std::endl;
	//std::cin.get();
	return 0;
}

//种子填充
void fillHole(const cv::Mat srcImg, cv::Mat &dstImg)
{
	cv::Size m_Size = srcImg.size();
	cv::Mat Temp = cv::Mat::zeros(m_Size.height + 2, m_Size.width + 2, srcImg.type());//延展图像
	srcImg.copyTo(Temp(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)));

	cv::floodFill(Temp, cv::Point(0, 0), cv::Scalar(255));

	cv::Mat cutImg;//裁剪延展的图像
	Temp(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)).copyTo(cutImg);

	dstImg = srcImg | (~cutImg);
}
int SeedFill(cv::Mat& _binImg, cv::Mat& _lableImg)//返回区域个数
{
	/*if (_binImg.empty() ||
	_binImg.type() != CV_8UC1)
	{
	return;
	}*/
	if (_binImg.empty() ||
		_binImg.type() != CV_8UC1)
	{
		return -1;
	}

	_lableImg.release();
	_binImg.convertTo(_lableImg, CV_32SC1);

	int label = 255;  // start by 256 
	int region_num = 0;//count number of regions

	int rows = _binImg.rows - 1;
	int cols = _binImg.cols - 1;
	for (int i = 1; i < rows; i++)
	{
		//std::cout << "第" << i << "行" << std::endl;
		int * data = _lableImg.ptr<int>(i);
		for (int j = 1; j < cols; j++)
		{
			if (data[j] == 255)
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // pixel position: <i,j>  
				region_num++;
				++label;  // begin with a new label  
				while (!neighborPixels.empty())
				{
					//k++;
					// get the top pixel on the stack and label it with the same label  
					std::pair<int, int> curPixel = neighborPixels.top();
					int curX = curPixel.first;
					int curY = curPixel.second;
					_lableImg.at<int>(curX, curY) = label;

					// pop the top pixel  
					neighborPixels.pop();

					if (curX>0 && curX<cols  && curY>0 && curY<cols)
					{
						// push the 4-neighbors (foreground pixels)  
						if (_lableImg.at<int>(curX, curY - 1) == 255)
						{// left pixel  
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (_lableImg.at<int>(curX, curY + 1) == 255)
						{// right pixel  
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (_lableImg.at<int>(curX - 1, curY) == 255)
						{// up pixel  
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (_lableImg.at<int>(curX + 1, curY) == 255)
						{// down pixel  
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
					}
				}
			}
		}
	}
	return region_num;
}
//道路区域增长
void RegionGrow(cv::Mat &Img, cv::Mat &img_bilateral)
{
	int rows = Img.rows - 1;
	int cols = Img.cols - 1;
	for (int i = 1; i < Img.rows; i++)
	{
		//std::cout << "第" << i << "行" << std::endl;
		uchar * data = Img.ptr<uchar>(i);
		for (int j = 1; j < Img.cols; j++)
		{
			if (data[j] == 255)
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // pixel position: <i,j>  
				while (!neighborPixels.empty())
				{
					//k++;
					// get the top pixel on the stack and label it with the same label  
					std::pair<int, int> curPixel = neighborPixels.top();
					int curX = curPixel.first;
					int curY = curPixel.second;
					Img.at<uchar>(curX, curY) = 1;
					// pop the top pixel  
					neighborPixels.pop();

					if (curX > 1 && curX < cols-1  && curY > 1 && curY < cols-1)
					{
						// push the 4-neighbors (foreground pixels)  
						if (abs(img_bilateral.at<uchar>(curX, curY - 1) - img_bilateral.at<uchar>(curX, curY)) <5
							&&abs(img_bilateral.at<uchar>(curX-1, curY - 1) - img_bilateral.at<uchar>(curX, curY)) <5
							&&abs(img_bilateral.at<uchar>(curX+1, curY - 1) - img_bilateral.at<uchar>(curX, curY)) <5
							&&abs(img_bilateral.at<uchar>(curX, curY - 2) - img_bilateral.at<uchar>(curX, curY)) <5
							&& img_bilateral.at<uchar>(curX, curY - 1)>125 && Img.at<uchar>(curX, curY - 1)!=1)
						{// left pixel  
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (abs(img_bilateral.at<uchar>(curX, curY + 1) - img_bilateral.at<uchar>(curX, curY)) <5
							&& abs(img_bilateral.at<uchar>(curX - 1, curY + 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX + 1, curY + 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX, curY + 2) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& img_bilateral.at<uchar>(curX, curY + 1)>125 && Img.at<uchar>(curX, curY + 1) != 1)
						{// right pixel  
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (abs(img_bilateral.at<uchar>(curX - 1, curY) - img_bilateral.at<uchar>(curX, curY)) <5
							&& abs(img_bilateral.at<uchar>(curX - 2, curY) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX - 1, curY - 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX - 1, curY + 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& img_bilateral.at<uchar>(curX - 1, curY)>125 && Img.at<uchar>(curX - 1, curY) != 1)
						{// up pixel  
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (abs(img_bilateral.at<uchar>(curX + 1, curY) - img_bilateral.at<uchar>(curX, curY)) <5
							&& abs(img_bilateral.at<uchar>(curX + 1, curY - 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX + 2, curY) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& abs(img_bilateral.at<uchar>(curX + 1, curY + 1) - img_bilateral.at<uchar>(curX, curY)) < 5
							&& img_bilateral.at<uchar>(curX + 1, curY)>125 && Img.at<uchar>(curX + 1, curY) != 1)
						{// down pixel  
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
					}
				}
			}
		}
	}

	/*cv::Mat _lableImg;
	int regions_n;
	regions_n=SeedFill(Img, _lableImg);
	for (int i = 256; i < 256 + regions_n; i++)
	{

	}*/
}
//骨架提取
cv::Mat thinning(cv:: Mat image)
{
	using namespace cv;
	IplImage *src;
	IplImage *distsrc;
	IplImage *out;
	IplImage *S00;
	IplImage *S45;
	IplImage *S90;
	IplImage *S135;
	CvMat kern00, kern45, kern90, kern135;
	float Smax = 0;

	float L0[] = {
		-1,-1,-1,-1,-1,
		0, 0, 0, 0, 0,
		2, 2, 2, 2, 2,
		0, 0, 0, 0, 0,
		-1,-1,-1,-1,-1
	};
	float L45[] = {
		0,-1,-1, 0, 2,
		-1,-1, 0, 2, 0,
		-1, 0, 2, 0,-1,
		0, 2, 0,-1,-1,
		2, 0,-1,-1, 0
	};
	float L90[] = {
		-1, 0, 2, 0,-1,
		-1, 0, 2, 0,-1,
		-1, 0, 2, 0,-1,
		-1, 0, 2, 0,-1,
		-1, 0, 2, 0,-1
	};
	float L135[] = {
		2, 0,-1,-1, 0,
		0, 2, 0,-1,-1,
		-1, 0, 2, 0,-1,
		-1,-1, 0, 2, 0,
		0,-1,-1, 0, 2
	};
	src = &IplImage(image);
	distsrc = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	S00 = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	S45 = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	S90 = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	S135 = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	out = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	//kernel建立矩阵卷积核
	cvInitMatHeader(&kern00, 5, 5, CV_32FC1, L0);
	cvInitMatHeader(&kern45, 5, 5, CV_32FC1, L45);
	cvInitMatHeader(&kern90, 5, 5, CV_32FC1, L90);
	cvInitMatHeader(&kern135, 5, 5, CV_32FC1, L135);
	//距离变换
	cvDistTransform(src, distsrc, CV_DIST_L2, 5);
	//过滤，其实就是卷积
	cvFilter2D(distsrc, S00, &kern00);
	cvFilter2D(distsrc, S45, &kern45);
	cvFilter2D(distsrc, S90, &kern90);
	cvFilter2D(distsrc, S135, &kern135);
	//
	//Smax = MAX(S00,S45,S90,S135)
	//     / Smax, Smax >= 0
	// g = |
	//     \ 0 , others
	//
	for (int y = 0; y < out->height; y++) {
		for (int x = 0; x < out->width; x++) {
			Smax = MAX(
				MAX(((float*)(S00->imageData + y* S00->widthStep))[x], ((float*)(S45->imageData + y* S45->widthStep))[x]),
				MAX(((float*)(S90->imageData + y* S90->widthStep))[x], ((float*)(S135->imageData + y* S135->widthStep))[x]));
			((float*)(out->imageData + y* out->widthStep))[x] = Smax > 0 ? Smax : 0.0;
		}
	}
	cvThreshold(out, out, 7, 1, CV_THRESH_BINARY);
	/*cvNamedWindow("S00", 1);
	cvNamedWindow("S45", 1);
	cvNamedWindow("S90", 1);
	cvNamedWindow("S135", 1);
	cvNamedWindow("out", 1);
	cvShowImage("S00", S00);
	cvShowImage("S45", S45);
	cvShowImage("S90", S90);
	cvShowImage("S135", S135);
	cvShowImage("out", out);
	cvWaitKey(0);*/

	cvCvtScaleAbs(S00, src, 32, 0);
	cvSaveImage("S00.png", src);
	cvCvtScaleAbs(S45, src, 32, 0);
	cvSaveImage("S45.png", src);
	cvCvtScaleAbs(S90, src, 32, 0);
	cvSaveImage("S90.png", src);
	cvCvtScaleAbs(S135, src, 32, 0);
	cvSaveImage("S135.png", src);
	cvCvtScaleAbs(out, src, 255, 0);
	cvSaveImage("out.png", src);

	return Mat(src, 0);
}
//两点距离
float PtDis(std::pair<int,int> pt1, std::pair<int,int> pt2)
{
	return sqrt(pow(float(pt1.first - pt2.first), 2) + pow(float(pt1.second - pt2.second), 2));
}
float PtDis(std::pair<float, float> pt1, std::pair<float, float> pt2)
{
	return sqrt(pow(float(pt1.first - pt2.first), 2) + pow(float(pt1.second - pt2.second), 2));
}
float PtDis(std::pair<double, double> pt1, std::pair<double, double> pt2)
{
	return sqrt(pow(float(pt1.first - pt2.first), 2) + pow(float(pt1.second - pt2.second), 2));
}
////snake算法
//bool termination_point;
//bool neighbor3by3; /* true means 3x3 neighborhood, false means 5x5 neighborhood */
//int grad_mag[500][500];	/* Magnitude of Gradient */
//int m_Cols; /* Number of Columns */
//int m_Rows; /* Number of Rows */
//int no_of_snake_points;
//CPoint Snake_points[200]; /* Snake Points */
//double *alpha, *beta, *gamma; /* Weights for energies */
//double threshold_curvature;
//int threshold_movement;
//const int Max_Iterations=10;
//
//void Snake_algorithm()
//{
//	bool flag_change;
//	int movement, i, j, iteration = 0;
//	double avg_distance = 0.0, max_curvature;
//	CPoint temp;
//	double *curvature;
//	alpha = new double[no_of_snake_points];
//	beta = new double[no_of_snake_points];
//	gamma = new double[no_of_snake_points];
//	curvature = new double[no_of_snake_points];
//	termination_point = false;
//
//	for (i = 0; i < no_of_snake_points; i++)
//	{
//		*(alpha + i) = 1.0;
//		*(beta + i) = 1.0;
//		*(gamma + i) = 1.2;
//		avg_distance += find_distance(i, Snake_points[i]);
//	}
//	j = no_of_snake_points;
//	
//	//pdc =GetDC();
//	while (!termination_point)
//	{
//		movement = 0;
//		avg_distance = avg_distance / (double)no_of_snake_points;
//		max_curvature = -1000000000.0;
//		for (i = 0; i < no_of_snake_points; i++)
//		{
//			temp = find_min_energy(i, Snake_points[i], avg_distance);
//			flag_change = false;
//			if (temp != Snake_points[i] && temp != Snake_points[(i - 1 + j) % j] && temp != Snake_points[(i + 1) % j])
//			{
//				Snake_points[i] = temp;
//				movement++;
//			}
//			curvature[i] = find_curvature(i, Snake_points[i]);
//			if (max_curvature < curvature[i])
//				max_curvature = curvature[i];
//		}
//		avg_distance = 0.0;
//		for (i = 0; i < no_of_snake_points; i++)
//			curvature[i] = curvature[i] / max_curvature;
//		for (i = 0; i < no_of_snake_points; i++)
//		{
//			avg_distance += find_distance(i, Snake_points[i]);
//			if (curvature[i] > threshold_curvature&&curvature[i] > curvature[(i + 1) % no_of_snake_points] && curvature[i] > curvature[(i - 1 + no_of_snake_points) % no_of_snake_points])
//				*(beta + i) = 0;
//		}
//		if (movement < threshold_movement)
//			termination_point = true;
//		iteration++;
//		if (iteration > Max_Iterations)
//			termination_point = true;
//	}
//	delete alpha;
//	delete beta;
//	delete gamma;
//	delete curvature;
//}
//
//double find_distance(int no, CPoint point)
//{
//	int x = no_of_snake_points;
//	point -= Snake_points[(no - 1 + x) % x];
//	return(sqrt(point.x*point.x + point.y*point.y));
//}
//
//double find_curvature(int no, CPoint point)
//{
//	int x = no_of_snake_points;
//	point = Snake_points[(no - 1 + x) % x] - point - point + Snake_points[(no + 1) % x];
//	return(point.x*point.x + point.y*point.y);
//}
//
//double find_continuity(int no, CPoint point, double avg_distance)
//{
//	return(pow(avg_distance - find_distance(no, point), 2));
//}
//
//CPoint find_min_energy(int no, CPoint point, double avg_distance)
//{
//	CPoint p, min_point;
//	double max_curvature, max_continuity, max_internal, min_internal, min_energy, energy;
//	double curvatures[5][5];
//	double continuities[5][5];
//	double internal_energies[5][5];
//	int i, j, limit = 1;
//	max_curvature = max_continuity = max_internal = -1000000000000.0;
//	min_internal = 1000000000000.0;
//	if (!neighbor3by3)
//		limit++;
//	for (i = -limit; i <= limit; i++)
//	{
//		p.y = point.y + i;
//		if (p.y < 0)
//			p.y = 0;
//		if (p.y >= m_Rows)
//			p.y = m_Rows - 1;
//		for (j = -limit; j <= limit; j++)
//		{
//			p.x = point.x + j;
//			if (p.x < 0)
//				p.x = 0;
//			if (p.x >= m_Cols)
//				p.x = m_Cols - 1;
//			curvatures[i + limit][j + limit] = find_curvature(no, p); //This code can cause problem near
//			continuities[i + limit][j + limit] = find_continuity(no, p, avg_distance);	//border of image
//			internal_energies[i + limit][j + limit] = (double)grad_mag[p.y][p.x];
//			if (curvatures[i + limit][j + limit] > max_curvature)
//				max_curvature = curvatures[i + limit][j + limit];
//			if (continuities[i + limit][j + limit] > max_continuity)
//				max_continuity = continuities[i + limit][j + limit];
//			if (internal_energies[i + limit][j + limit] > max_internal)
//				max_internal = internal_energies[i + limit][j + limit];
//			if (internal_energies[i + limit][j + limit] < min_internal)
//				min_internal = internal_energies[i + limit][j + limit];
//		}
//	}
//	for (i = 0; i <= 2 * limit; i++)
//	{
//		for (j = 0; j <= 2 * limit; j++)
//		{
//			curvatures[i][j] = curvatures[i][j] / max_curvature;
//			continuities[i][j] = continuities[i][j] / max_continuity;
//			internal_energies[i][j] = (internal_energies[i][j] - min_internal) / (max_internal - min_internal);
//		}
//	}
//	min_point.x = -limit;
//	min_point.y = -limit;
//	min_energy = 1000000000000.0;
//	for (i = -limit; i <= limit; i++)
//	{
//		for (j = -limit; j <= limit; j++)
//		{
//			energy = *(alpha + no)*continuities[i + limit][j + limit] + *(beta + no)*curvatures[i + limit][j + limit] - *(gamma + no)*internal_energies[i + limit][j + limit];
//			if (energy < min_energy || (energy == min_energy&&i == 0 && j == 0))
//			{
//				min_energy = energy;
//				min_point.x = j;
//				min_point.y = i;
//			}
//		}
//	}
//	min_point.x = min_point.x + point.x;
//	min_point.y = min_point.y + point.y;
//	return(min_point);
//}


int main(int argc,char *argv[])
{
	for(int i=0;i<argc;i++)
	{
		std::cout<<argv[i]<<endl;
	}
	//std::cout<<"0"<<std::endl;
	//std::cin.get();
	//string s_samdens(argv[3]);
	//std::cout<<"1"<<std::endl;
 //   std::cin.get();
	////float samdens=(float)(argv[3]);
	//const char* ch_samdens=s_samdens.c_str();
	//std::cout<<"2"<<std::endl;
	//std::cin.get();
	double samdens=atof(argv[2]);
	//std::cout<<"3"<<std::endl;
	//std::cin.get();
	roadextraction(argv[0],argv[1],samdens);
	//std::cout<<"processing done!"<<std::endl;
	return 0;
}