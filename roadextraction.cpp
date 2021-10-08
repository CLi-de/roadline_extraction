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



void fillHole(const cv::Mat srcImg, cv::Mat &dstImg);//�׶����
int SeedFill(cv::Mat& _binImg, cv::Mat& _lableImg);//������䣬����������
void RegionGrow(cv::Mat &Img, cv::Mat &img_bilateral);//��������
cv::Mat thinning(cv::Mat image);//�Ǽ���ȡ
float PtDis(std::pair<int,int> pt1, std::pair<int,int> pt2);//�������


double find_distance(int no, CPoint point);
double find_curvature(int no, CPoint point);
double find_continuity(int no, CPoint point, double avg_distance);
CPoint find_min_energy(int no, CPoint point, double avg_distance);
void Snake_algorithm();

const double Pi = 3.14159265;


int roadextraction(const char* filePath,string outptsPath,float samdens)//����Ӱ��
{
	/*char* filePath= "D:\\��·��ȡӰ��\\����\\DOM\\dom-area56.TIF";
    string outptsPath= "E:\mysource\roadextraction\roadregion\testdata\2901.5-476.5.tif";
	float samdens=10.000000;*/

struct ImgParam //ȥ��extern
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
		int dataType; //�ļ�����
		double dataMax;//���������ֵ ��Сֵ
		double dataMin;
		int width;//���
		int height;//�߶� 
		double utmX;//���Ͻǵ�UTM ����
		double utmY;
		int zone;
		double pGeoTrans[6];//����任6���� 
		char sProjRef[1024];//������Ϣ����
		double xRes;//X�����ֱ���
		double yRes;//Y�����ֱ��� 
		char filename[250];
	}; //ָ��ImgParam


#pragma region  ͼ������
	   //std::string filePath1 = "C:/Users/hj/Desktop/2901.5-476.5.tif";
	ImgParam param;
//	const char* filePath = (LPSTR)(LPCTSTR)filePath1;

	//std::string str1(filePath);

	/*std::string filePath2(filePath1.GetBuffer());
	const char* filePath=filePath2.c_str();*/

	/*std::string path=CStringToString(filePath1);
	const char* filePath=path.c_str();*/

#pragma endregion

#pragma region ���ݶ�ȡ
	//ע��GDAL����
	GDALAllRegister();
	//����·��֧��
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	//��ȡ�ļ�DataSet
	GDALDataset* poDataset = (GDALDataset *)GDALOpen(filePath, GA_ReadOnly);//GA_Update��ΪGA_ReadOnly
	if (poDataset == NULL)
		return -1;

	int rasterCount = poDataset->GetRasterCount();//ͨ������
	param.width = poDataset->GetRasterXSize();//ͼ���С
	param.height = poDataset->GetRasterYSize();
	GDALRasterBand *poBand;
	poBand = poDataset->GetRasterBand(1);//��ȡָ���1���ε�ָ��
	param.dataType = poBand->GetRasterDataType();//��ȡ��������

												 //���ݳ����������Сֵ 
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

	double adfGeoTransform[6];//Ӱ����������ʡ��[2][4]-��ת������
	if (poDataset->GetGeoTransform(adfGeoTransform) == CE_None)
	{
		std::cout << "��ȡӰ��������" << std::endl;
	}

	////��γ����Ϣ
	//strcpy(param.sProjRef, poDataset->GetProjectionRef());
	////ת������
	//poDataset->GetGeoTransform(param.pGeoTrans);
	////ȡ�õ���ֱ���
	//param.xRes = param.pGeoTrans[1];
	//param.yRes = param.pGeoTrans[5];
	////ȡ�����Ͻ�UTM����
	//param.utmX = param.pGeoTrans[0];
	//param.utmY = param.pGeoTrans[3];
	//double centerLon = GetParaValue(param.sProjRef, "central_meridian");
	//param.zone = (int)((centerLon + 180.0) / 6.0 + 1.0);
	//���������Щ��Ϣ����

#pragma endregion  

#pragma region �����ڴ�
	int lineLength = param.width * sizeof(char);//lineLenth:ÿ����ռ�ֽڳ��� 
	int readBufferLength = 0;//readBufferLenth:ÿ����ռ�ֽڳ���
	char* dataBuffer = NULL;//dataBuffer:ָ��ÿ�п�ͷ��ָ��
	unsigned char* dataChangeBuffer = NULL;//dataChangeBuffer:ָ��ÿ�п�ͷ��ָ��
	if (param.dataType == GDT_Byte)
		readBufferLength = param.width * sizeof(char);
	else if (param.dataType == GDT_UInt16)
		readBufferLength = param.width * sizeof(short);
	dataBuffer = new char[readBufferLength];
	dataChangeBuffer = new unsigned char[param.width];
#pragma endregion

#pragma region ��������ڴ����
	MEMORYSTATUS MemoryStatus;//MEMORYSTATUS�ṹ��Ӧ�ڴ���Ϣ
	GlobalMemoryStatus(&MemoryStatus);//�����ķ�����Ϣ���洢��MEMORYSTATUS�ṹ��
	DWORD  dataLenth = lineLength*param.height;//����ռ�ֽ���
	char* imgBuffer = NULL;//imgBuffer:ָ������ͼ���ָ��
						   //bool hasUnmapViewOfFile = false;
						   //HANDLE fileHandle, fileMemMapping;

						   //���ͼ��С��2G ������������ڴ����ͼ���ڴ�
	if (dataLenth < 1024 * 1024 * 1024 * 1.5 && MemoryStatus.dwAvailPhys > dataLenth)
		imgBuffer = new char[dataLenth];//imgBuffer��ͼ�����ݿ�

										//else//���ͼ�����2G�����ҿ��������ڴ����ͼ���ڴ�
										//{
										//	char tempPath[MAX_PATH] = {0},memFilePath[MAX_PATH]= {0},path[MAX_PATH] = {0};
										//	//GetTempPath(MAX_PATH,tempPath); 
										//	GetTempFileName(path, // ��ʱ�ļ�Ŀ¼
										//		"NEW",                    // ��ʱ�ļ��ĵ�ǰ׺
										//		0,                        // ����Ψһ������
										//		memFilePath);              // �������ֵĻ���
										//	fileHandle = CreateFile(TEXT(memFilePath),
										//		GENERIC_READ | GENERIC_WRITE,// �ļ�����Ȩ��,д
										//		0,//����ģʽ,��������0��ֹ�������̴��ļ����豸
										//		NULL,//SECURITY_ATTRIBUTES�ṹ����ȫ����������NULL����Ĭ�ϰ�ȫ����
										//		CREATE_ALWAYS,//���ڴ��ڻ򲻴��ڵ�����ִ�еĲ�����������ʼ�մ���
										//		FILE_ATTRIBUTE_NORMAL|FILE_FLAG_DELETE_ON_CLOSE,//�����ļ������ԣ������и��ٻ����ѡ��
										//		NULL);//����������Ҫ���Ƶ��ļ� src 
										//	if(fileHandle == NULL) 
										//	{ 
										//		printf( "��ʱ�ļ��½�ʧ��\n"); // ��ʾ��Ϣ 
										//		return -2; 
										//	}
										//	fileMemMapping = CreateFileMapping(fileHandle, 
										//		NULL, 
										//		PAGE_READWRITE, 
										//		0,
										//		(DWORD)dataLenth,
										//		0);
										//	if (fileMemMapping == NULL){ 
										//		printf("��������%d",GetLastError());
										//		return -2;
										//	}
										//	imgBuffer = (char* )MapViewOfFile( fileMemMapping, 
										//		FILE_MAP_ALL_ACCESS,0,0,0);

										//	hasUnmapViewOfFile = true;
										//}
#pragma endregion


#pragma region ȡ���ݸ�imgBuffer
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
	delete[] dataBuffer; //�ͷŶ����ڴ�
	delete[] dataChangeBuffer;
	//�ر�����
	poDataset->FlushCache();//ˢ�»��棬Ϊ������������cache�ռ�
	GDALClose(poDataset);
	poDataset = NULL;
#pragma endregion


#pragma region Ԥ����
	cv::Mat imageAll = cv::Mat(param.height, param.width, CV_8U, imgBuffer);//���캯����ʼ��imageALL

	std::cout << "start processing..." << std::endl;
	//�ز���
	// cv::Mat_<uchar> img_resample;
	// cv::resize(imageAll, img_resample, cv::Size(imageAll.rows / 1, imageAll.cols / 1), 0, 0, 1);
	//��˹�˲� 
	cv::Mat img_gauss;
	cv::Size size(5, 5);
	cv::GaussianBlur(imageAll, img_gauss, size, 1.0, 1.0);//ȥ��
    //cv::imwrite("C:/Users/hj/Desktop/gauss.tif", img_gauss);
	//˫���˲�
	delete[] imgBuffer;
	imgBuffer = NULL;
	cv::Mat img_bilateral;
	cv::bilateralFilter(img_gauss, img_bilateral, 5, 20.0, 20.0);//����ȥ��
	//cv::imwrite("C:/Users/hj/Desktop/bilateral.tif", img_bilateral);
#pragma endregion 

#pragma region �㷨����

#pragma region ��ֵ��
	const float threshold = 25;
	cv::Mat_<uchar> erzhi(img_bilateral.rows, img_bilateral.cols);
	erzhi = 0;
	//5-�����ڻҶ�һ��
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

	//������
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

#pragma region С�����޳�
	cv::Mat seed;
	int regions_n;//������
	regions_n = SeedFill(erzhi, seed);
	//std::vector<Region> rgs(regions_n);
	std::vector<std::vector<std::pair<int, int>>> rgs(regions_n);//������㼯
	//int* perregion_n = new int[regions_n];//���������
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n[i] = 0;

	for (int i = 1; i < seed.rows - 1; i++)
		for (int j = 1; j < seed.cols - 1; j++)
		{
			if (seed.at<int>(i, j) > 255)
			{
				rgs[seed.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//�㼯�ϵ�λ
				//perregion_n[seed.at<int>(i, j) - 256]++;
			}
		}
	cv::Mat_<int> BigR;//����������
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


#pragma region  ��̬ѧ����
	cv::Mat Kernel;
	Kernel = cv::getStructuringElement(0, cv::Size(5, 5));//���νṹ

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
#pragma region �����С�����޳�
	cv::Mat seed1;
	regions_n = SeedFill(erd, seed1);
	std::vector<std::vector<std::pair<int,int>>> remain(regions_n);

	//3.�����ʼ�����ɿ������������ʱ��ʼ����Ч�ʻ��Щ��
	//int* perregion_n1 = new int[regions_n];//���������
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n1[i] = 0;

	for (int i = 1; i < seed1.rows - 1; i++)
		for (int j = 1; j < seed1.cols - 1; j++)
		{
			if (seed1.at<int>(i, j) > 255)
			{
				remain[seed1.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//�㼯�ϵ�λ
				//perregion_n1[seed1.at<int>(i, j) - 256]++;
			}
		}

	//delete[] perregion_n1;

	cv::Mat_<int> BigR1;
	BigR1 = seed1.clone();
	const int THREPTS_MIN = 1500;//���������С��ֵ
	const int THREPTS_MAX = 250000;//������������ֵ
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

//��ʴ������

			/*cv::Mat_<uchar> dlt1(BigR_uc.rows, BigR_uc.cols);
			cv::dilate(BigR_uc, dlt1, Kernel, cv::Point(-1, -1), 1);*/
			//	cv::imwrite("C:/Users/hj/Desktop/dilate1.tif", dlt1);

				/*cv::Mat_<uchar> ero1(BigR_uc.rows, BigR_uc.cols);
				cv::erode(BigR_uc, ero1, Kernel, cv::Point(-1, -1), 3);*/

				//cvThreshold(dlt1, dlt3, 120, 255, CV_THRESH_BINARY);
			// Canny(dlt1, dlt1, 60, 200);

#pragma region ������״����������

	struct Region_Feature{
		int label;//���ֵ

		double Area;//�������
		double Perimeter;//�����ܳ�
		double AP;//���Ӷ� Perimeter*Perimeter / Area(����Ϊ��·�����)
		double PA;//������ܳ�

		//cv::RotatedRect MinRec;//��С��Ӿ���

		double Lmer;//��Ӿ��γ�
		double Wmer;//��Ӿ��ο�
		double LW;//����� Lmer / Wmer

		double Smer;//��С��Ӿ������ Lmer * Wmer

		double Fdegree;//������ Smer/Area
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
	//������
	/*for (int idx = 0; idx < contours.size(); idx++)
	{
		cv::drawContours(Contoursimg, contours, idx, color, 1, 8);
	}*/


	//std::vector<Region> rgs1;
	//rgs1.resize(contours.size());
	Region_Feature *rgs1;
	rgs1 = new Region_Feature[contours.size()];
	//��Ӿ��γ�������ȣ����������������ܳ������Ӷȣ�������
	std::vector<cv::RotatedRect> Rects;


	for (int i = 0; i < contours.size(); i++)
	{
		Rects.push_back(cv::minAreaRect(contours[i]));
		rgs1[i].Wmer = std::min(Rects[i].size.width, Rects[i].size.height);
		rgs1[i].Lmer = std::max(Rects[i].size.width, Rects[i].size.height);
		rgs1[i].LW = rgs1[i].Lmer / rgs1[i].Wmer;//��Ӿ��γ����

		rgs1[i].Area = cv::contourArea(contours[i], false);
		rgs1[i].Perimeter = cv::arcLength(contours[i], true);
		rgs1[i].AP = rgs1[i].Perimeter*rgs1[i].Perimeter / rgs1[i].Area;//���Ӷ�
		rgs1[i].PA = rgs1[i].Area / rgs1[i].Perimeter;//������ܳ�

		rgs1[i].Fdegree = rgs1[i].Wmer * rgs1[i].Lmer / rgs1[i].Area;//������
	}
	//delete[] rgs1;
	//6.����ɸѡ
	const double threshold_AP = 75.0;//���Ӷ�
	const double threshold_LW = 3.0;//�����
	const double threshold_Fdegree = 3.0;//������

	//��1�����ߵ�·���
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

	 //(2)ֱ�ߵ�·���
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

	//(3)�������
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

#pragma region �����޳�

#pragma endregion

#pragma region ��·��������(����������)
	cv::Mat regiongrow_p;
	regiongrow_p = regions_merge.clone();
	RegionGrow(regiongrow_p, img_bilateral);
	for (int i = 0; i < regiongrow_p.rows; i++)
		for (int j = 0; j < regiongrow_p.cols; j++)
		{
			if (regiongrow_p.at<uchar>(i, j) == 1)
				regiongrow_p.at<uchar>(i, j) = 255;
		}
	//�ֲ��ն�
	cv::Mat regiongrow;
	fillHole(regiongrow_p, regiongrow);
#pragma endregion 

#pragma region ���������ټ��
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

	//��Ӿ��γ�������ȣ����������������ܳ������Ӷȣ�������
	std::vector<cv::RotatedRect> Rects1;
	for (int i = 0; i < contours1.size(); i++)
	{
		Rects1.push_back(cv::minAreaRect(contours1[i]));
		rgs3[i].Wmer = std::min(Rects1[i].size.width, Rects1[i].size.height);
		rgs3[i].Lmer = std::max(Rects1[i].size.width, Rects1[i].size.height);
		rgs3[i].LW = rgs3[i].Lmer / rgs3[i].Wmer;//��Ӿ��γ����

		rgs3[i].Area = cv::contourArea(contours1[i], false);
		rgs3[i].Perimeter = cv::arcLength(contours1[i], true);
		rgs3[i].AP = rgs3[i].Perimeter*rgs3[i].Perimeter / rgs3[i].Area;//���Ӷ�
		rgs3[i].PA = rgs3[i].Area / rgs3[i].Perimeter;//������ܳ�

		rgs3[i].Fdegree = rgs3[i].Wmer * rgs3[i].Lmer / rgs3[i].Area;//������
	}


	//(1) ���ߵ�·���
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
	//(2) ֱ�ߵ�·���
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

	//#pragma region ·����
	//	cv::Mat regions_AP_seed;
	//	regions_n = SeedFill(regiongrow, regions_AP_seed);
	//
	//	int* perregion_n2 = new int[regions_n];//���������
	//	for (int i = 0; i < regions_n; i++)
	//		perregion_n2[i] = 0;
	//
	//	for (int i = 1; i < regions_AP_seed.rows - 1; i++)
	//		for (int j = 1; j < regions_AP_seed.cols - 1; j++)
	//		{
	//			if (regions_AP_seed.at<int>(i, j) > 255)
	//			{
	//				rgs1[regions_AP_seed.at<int>(i, j) - 256].pts_p.push_back(std::pair<int, int>(i, j));//�㼯�ϵ�λ
	//				perregion_n2[regions_AP_seed.at<int>(i, j) - 256]++;
	//			}
	//		}
	//
	//	for (int i = 0; i < regions_n; i++)//���������
	//		rgs1[i].points_n = perregion_n2[i];
	//
	//	delete[] perregion_n2;
	//
	//	std::vector<Region> temp;
	//	
	//	const int direction_n = 12;//������
	//	//float *road_width =new float[regions_n];
	//	std::vector<int> road_width;
	//	for (int i = 0; i < regions_n; i++)//ÿ����
	//	{
	//		//std::cout << std::endl;
	//		std::pair<int, int> sample_p[3];//��ʼ������(ȡ����)
	//		float maxdis[3] = { 10.0 ,10.0,10.0 };//�����߶��ֵ
	//		float mindis[3] = { 100.0 ,100.0,100.0 };//�����߶����ֵ
	//		//bool is_appro_samplept = false;
	//		for (int j = 0; j < 3; j++)
	//		{//�ж��Ƿ�Ϊ��Ե��
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
	//				sample_p[j].first = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1) + p].first;//��֤���ɷǱ�Ե������
	//				sample_p[j].second = rgs1[i].pts_p[rgs1[i].points_n / 4 * (j + 1) + p].second;
	//			//	is_appro_samplept = true;
	//			}
	//			//else
	//			//{
	//			//	sample_p[j].first = rgs1[i].pts_p[rgs1[i].points_n / 6 * (2 * j + 1) + 5].first;//+5�ɱ�֤���ɷǱ�Ե��
	//			//	sample_p[j].second = rgs1[i].pts_p[rgs1[i].points_n / 6 * (2 * j + 1) + 5].second;
	//			//	is_appro_samplept = true;
	//			//}
	//		/*	if (is_appro_samplept)
	//			 {*/
	//				std::pair<double, double> sample_p_next1;//������˵�
	//				std::pair<double, double> sample_p_next2;//������˵�
	//				float *dis_p = new float[direction_n];//ÿ�������·��
	//				for (int n = 0; n < direction_n; n++)//ÿ������
	//				{
	//					int count1 = 0;
	//					int count2 = 0;
	//					bool n1 = true;
	//					bool n2 = true;
	//					while (n1)//������
	//					{
	//						count1++;
	//						sample_p_next1.first = (double)sample_p[j].first + count1*cos(Pi / direction_n*n);
	//						sample_p_next1.second = (double)sample_p[j].second + count1*sin(Pi / direction_n*n);
	//						if (regiongrow.at<uchar>((int)sample_p_next1.first, (int)sample_p_next1.second) != 255)
	//							n1 = false;
	//					}
	//					while (n2)//������
	//					{
	//						count2++;
	//						sample_p_next2.first = (double)sample_p[j].first - count2*cos(Pi / direction_n*n);
	//						sample_p_next2.second = (double)sample_p[j].second - count2*sin(Pi / direction_n*n);
	//						if (regiongrow.at<uchar>((int)sample_p_next2.first, (int)sample_p_next2.second) != 255)
	//							n2 = false;
	//					}
	//					dis_p[n] = PtDis(sample_p_next1, sample_p_next2);
	//					std::cout <<"��"<< n << "�������:" << dis_p[n] << std::endl;
	//				}
	//				for (int n = 0; n < direction_n; n++)//ȡ�����������ľ����С���޳�
	//				{
	//					if (dis_p[n] > maxdis[j])
	//						maxdis[j] = dis_p[n];
	//				}
	//				for (int n = 0; n < direction_n; n++)//ȡ����������С�ľ���Ϊ·��(�������(ȡ���˱�Ե�㼰������)����С��20����),�޳���С��������
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
	//			road_width.push_back((mindis[0] + mindis[1] + mindis[2]) / 3);//ɸѡ���·�洢���
	//		}
	//	}
	//	for (int i = 0; i < rgs1.size(); i++)
	//	{
	//		rgs1[i].~Region();
	//	}
	//	/*for (int i = 0; i < road_width.size(); i++)
	//	{
	//		std::cout << "��" << i << "��:" << road_width[i] << std::endl;
	//	}*/
	//
	//	//�����޳�
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


#pragma region �Ǽ���ȡ���޳�����
	cv::Mat regions_thin;
	regions_thin = thinning(regions_merge1);
	cv::Mat regions_thin1;

	regions_n = SeedFill(regions_thin, regions_thin1);
	std::vector<std::vector<std::pair<int, int>>> rgs2(regions_n);//������㼯
	//std::vector<Region> rgs2(regions_n);
	//int* perregion_n3 = new int[regions_n];//���������
	//for (int i = 0; i < regions_n; i++)
	//	perregion_n3[i] = 0;

	for (int i = 1; i < regions_thin1.rows - 1; i++)
		for (int j = 1; j < regions_thin1.cols - 1; j++)
		{
			if (regions_thin1.at<int>(i, j) > 255)
			{
				rgs2[regions_thin1.at<int>(i, j) - 256].push_back(std::pair<int, int>(i, j));//�㼯�ϵ�λ
				//perregion_n3[regions_thin1.at<int>(i, j) - 256]++;
			}
		}

	//delete[] perregion_n3;

	//�����˳�
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


#pragma region ȥ�루��TODO��
	//1.���������
	//2.�������
	//3.��ɢ�Լ��㣨���

#pragma endregion
	//�ڹǼܵĻ�������ϸ����ÿ�����ؾ���������
	cv::Mat seed2;
	regions_n = SeedFill(BigR_uc2, seed2);
	for (int i = 0; i < seed2.rows; i++)
	{
		std::vector<std::vector<std::pair<int, int>>> temp;
		temp.resize(seed2.cols);
		std::vector<int> label;//���ֵ
		label.resize(seed2.cols);
		int new_line = -1;//������
		int pre = 10000;//���ÿ�����ϵ�ǰһ�������λ��(j)
		bool has_pts = false;//ÿ�������Ƿ�������
		for (int j = 0; j < seed2.cols; j++)
		{
			if (seed2.at<int>(i, j) > 255)
			{
				has_pts = true;
				if (j - pre == 1)
				{
					pre = j;
					temp[new_line].push_back(std::pair<int, int>(i, j));//�㼯�ֱ����temp
					label[new_line] = seed2.at<int>(i, j);
					seed2.at<int>(i, j) = 0;
				}
				else
				{
					pre = j;
					new_line++;
					temp[new_line].push_back(std::pair<int, int>(i, j));//�㼯�ֱ����temp
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

	//�ϲ��������
	for (int i = 0; i < seed2.rows; i = i + 5)
	{
		std::vector<std::pair<int, int>> pts_temp;
		pts_temp.reserve(regions_n * 5);
		int pts_n = 0;
		for (int j = 0; j < seed2.cols; j++)//����洢
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
			std::vector<std::pair<int, int>> pts_erase;//Ҫ�ϲ��ĵ�
			pts_erase.reserve(regions_n * 5);
			for (int k = 0; k < pts_n - 1; k++)
				for (int p = k + 1; p < pts_n; p++)
				{
					if (abs(pts_temp[k].second - pts_temp[p].second) < 20)
					{
						pts_temp.push_back(std::pair<int, int>((pts_temp[k].first + pts_temp[p].first) / 2,
							(pts_temp[k].second + pts_temp[p].second) / 2));//ȡ�е�ϲ�
						pts_erase.push_back(pts_temp[k]);
						pts_erase.push_back(pts_temp[p]);
					}
				}
				//��ɸѡ
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
#pragma region ���Ľ�
	//ÿ��5��ȡ��,���ƾ��루5-10��
	std::vector<std::vector<std::pair<int, int>>> region_pts;//���ɸѡ�ĵ�
	region_pts.resize(regions_n);

	std::vector<std::vector<std::vector<std::pair<int, int>>>> perline_pts;//*ÿ��ÿ������洢�ĵ㼯
	perline_pts.resize((seed2.rows-1)/5+1);
	for (int i = 0; i < (seed2.rows - 1) / 5 + 1; i++)//ÿ������������
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
					else//�Ҿ���˵�����ĵ㣬Ȼ�����ȡ��
					{
						float mindis = 10000.0;
						int label;//�����С����������
						for (int a = 0; a < dis.size(); a++)
						{
							if (dis[a] < mindis)
							{
								mindis = dis[a];
								label = a;
							}
						}
						int segment_num = int(dis[label]) / Ptdensity;//�ڲ����
						if (segment_num < 3)
						{
							//�ڲ�
							std::pair<int, int>	minpt = perline_pts[k][r][label%perline_pts[k][r].size()];//������С��
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
	//std::vector<std::vector<std::pair<int, int>>> perregion_lastpts;//��һ��ÿ������ĵ�
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
	//				dis_appro.resize(perregion_lastpts[seed2.at<int>(i, j) - 256].size(),false);*///�ж�ÿ�����֮ǰ�����е�ľ����Ƿ���������
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
	std::vector<std::vector<std::pair<int, int>>> preregion_pts;//���ɸѡ�ĵ�
	preregion_pts.resize(regions_n);
	for (int i = 0; i < seed2.rows; i++)
		for (int j = 0; j < seed2.cols; j++)
		{
			if (seed2.at<int>(i, j) < 256+regions_n)
				seed2.at<int>(i, j) = 0;
		}
	//	//����ͬ���ϵ�ֱ��������
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
	//	//ÿ�����ϵ������
	//	std::vector<std::vector<std::pair<int, int>>> pts_obj;
	//	pts_obj.resize(regions_n);
	//	std::vector<std::pair<int, int>> pt_side(regions_n);//ÿ�����ϵĶ˵�����
	//	//����ÿ�����϶˵㣨�����������������˵����ߵļн��жϼ��ɣ�
	//	for (int k = 0; k < regions_n; k++)
	//	{
	//		for (int p = 0; p < pts_temp[k].size(); p++)
	//		{
	//			std::pair<int, int> minst1;
	//			std::pair<int, int> minst2;
	//			float *dis_cluster = new float[pts_temp[k].size()];
	//
	//			for (int h = 0; h < pts_temp[k].size(); h++)
	//				dis_cluster[h]=PtDis(pts_temp[k][p], pts_temp[k][h]);//�����
	//
	//			 dis_cluster[p]=10000;//��������ľ��벻��С
	//			 float min_dis = dis_cluster[p];
	//			 int index=p;
	//			for (int h = 0; h < pts_temp[k].size(); h++)//����С����
	//			{
	//				if (dis_cluster[h] < min_dis)
	//				{
	//					min_dis = dis_cluster[h];
	//					index = h;
	//				}
	//			}
	//			minst1 = pts_temp[k][index];//��һ���ڵ�
	//			dis_cluster[index] = 10000;
	//			min_dis = dis_cluster[index];
	//			for (int h = 0; h < pts_temp[k].size(); h++)//�ҵڶ�С����
	//			{
	//				if (dis_cluster[h] < min_dis)
	//				{
	//					min_dis = dis_cluster[h];
	//					index = h;
	//				}
	//			}
	//			delete[]dis_cluster;
	//			minst2 = pts_temp[k][index];//�ڶ����ڵ�
	//			double pt_pt1;
	//			double pt_pt2;
	//			pt_pt1=atan2(minst1.first, pts_temp[k][p].second);
	//			pt_pt2=atan2(minst2.first, pts_temp[k][p].second);
	//			if (abs(pt_pt1 - pt_pt2) > Pi / 3)
	//			{
	//				pt_side[k] = pts_temp[k][p];
	//			}
	//			break;//ֻ��һ���˵�
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

#pragma region ������·����������ȡ��
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
	//fprintf(pf,"%s %d %s \n","��",i,"·�ε㣺");
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

//�������
void fillHole(const cv::Mat srcImg, cv::Mat &dstImg)
{
	cv::Size m_Size = srcImg.size();
	cv::Mat Temp = cv::Mat::zeros(m_Size.height + 2, m_Size.width + 2, srcImg.type());//��չͼ��
	srcImg.copyTo(Temp(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)));

	cv::floodFill(Temp, cv::Point(0, 0), cv::Scalar(255));

	cv::Mat cutImg;//�ü���չ��ͼ��
	Temp(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)).copyTo(cutImg);

	dstImg = srcImg | (~cutImg);
}
int SeedFill(cv::Mat& _binImg, cv::Mat& _lableImg)//�����������
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
		//std::cout << "��" << i << "��" << std::endl;
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
//��·��������
void RegionGrow(cv::Mat &Img, cv::Mat &img_bilateral)
{
	int rows = Img.rows - 1;
	int cols = Img.cols - 1;
	for (int i = 1; i < Img.rows; i++)
	{
		//std::cout << "��" << i << "��" << std::endl;
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
//�Ǽ���ȡ
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
	//kernel������������
	cvInitMatHeader(&kern00, 5, 5, CV_32FC1, L0);
	cvInitMatHeader(&kern45, 5, 5, CV_32FC1, L45);
	cvInitMatHeader(&kern90, 5, 5, CV_32FC1, L90);
	cvInitMatHeader(&kern135, 5, 5, CV_32FC1, L135);
	//����任
	cvDistTransform(src, distsrc, CV_DIST_L2, 5);
	//���ˣ���ʵ���Ǿ��
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
//�������
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
////snake�㷨
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