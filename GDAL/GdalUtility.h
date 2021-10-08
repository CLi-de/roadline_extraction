#ifndef __GdalUtility_h
#define __GdalUtility_h

#include "stdafx.h"  
#include "gdal.h"
#include "gdal_version.h"
#include "gdal_priv.h"
#include "gdal_frmts.h"
#include "ogr_spatialref.h" 
// Windows 头文件:
#include <windows.h> 
#include <cstring>
#include "opencv2\core\core.hpp"
using namespace cv; 


typedef unsigned char byte;
//点信息
struct PointD{
public:
	double X;
	double Y;
	PointD (double x = 0, double y = 0):
	X(x),
		Y(y)
	{}
};

//点信息
struct RectD{
public:
	double   left;
	double   top;
	double   right;
	double   bottom;
};

/// <summary>
/// 图像信息
/// </summary>
struct GeoTiffInfo
{
public:
	bool hasGeo; //是否包含地理信息
	double pGeoTrans[6];//反射变换6参数 
	char sProjRef[1024];//地理信息描述
	double cLon;//中心经度
	double clat;//中心纬度
	RectD rect;
	double xRes;//X轴地面分辨率
	double yRes;//Y轴地面分辨率
	int imageXSize;//地图宽度
	int imageYSize;//地图高度
	int zone;//中心点所在区域 1—60
	/// <summary>
	///图像类型
	///GDT_Unknown = 0,
	///GDT_Byte = 1,
	///GDT_UInt16 = 2,
	///GDT_Int16 = 3,
	///GDT_UInt32 = 4,
	///GDT_Int32 = 5,
	///GDT_Float32 = 6,
	///GDT_Float64 = 7,
	///GDT_CInt16 = 8,
	///GDT_CInt32 = 9,
	///GDT_CFloat32 = 10,
	///GDT_CFloat64 = 11,
	///GDT_TypeCount = 12,
	/// </summary>
	GDALDataType type;//图像类型
	int rasterCount;//图层数量 
	//double minNum;//图像数据中的最大值（采样）
	//double maxNum;//图像数据中的最小值（采样）
	double centerLon ;
	double falseNorth ;
	double falseEast ;
	double scaleFact ;
};



class CGdalUtility
{
public:
	inline bool IsReadySuc() const{ return (m_poDataset != NULL);}
	inline GeoTiffInfo& GetGeoTiffInfo() {return m_tiffInfo;}

	//构造函数
	CGdalUtility::CGdalUtility()
	{
		//注册GDAL驱动
		GDALAllRegister(); 
		//中文路径支持
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
		//初始化
		m_poDataset = NULL;
		m_poLL2TMTransform = NULL;
		m_poTM2LLTransform= NULL;
	}

	//析构函数
	~CGdalUtility()
	{
		Dispose();
	}

	//打开图像 
	bool Open(const char* lpszFilePathName)
	{  
		//销毁旧数据
		Dispose();
		//获取数据集
		m_poDataset = (GDALDataset *) GDALOpen( lpszFilePathName, GA_ReadOnly );

		if (m_poDataset == NULL) 
			return false;

		//通道数量
		m_tiffInfo.rasterCount = m_poDataset->GetRasterCount();

		//图像大小
		m_tiffInfo.imageXSize = m_poDataset->GetRasterXSize();
		m_tiffInfo.imageYSize = m_poDataset->GetRasterYSize();

		//获取数据类型
		GDALRasterBand *poBand;
		//读取数据类型
		poBand = m_poDataset->GetRasterBand(1);
		m_tiffInfo.type =  poBand->GetRasterDataType(); 

		////数据抽样的最大最小值
		//double argout[2];
		//if (m_tiffInfo.type == 1){
		//	m_tiffInfo.minNum = 0;
		//	m_tiffInfo.maxNum = 255;
		//}
		//else
		//{ 
		//	poBand->ComputeRasterMinMax(0,argout);
		//	m_tiffInfo.minNum = argout[0];
		//	m_tiffInfo.maxNum = argout[1];
		//}

		//经纬度信息
		const char* projRef = m_poDataset->GetProjectionRef(); 
		strcpy_s(m_tiffInfo.sProjRef ,projRef);	

		//转换参数
		m_poDataset->GetGeoTransform(m_tiffInfo.pGeoTrans); 
		//取得地面分辨率
		m_tiffInfo.xRes = m_tiffInfo.pGeoTrans[1];
		m_tiffInfo.yRes = m_tiffInfo.pGeoTrans[5];
		//取得经度 纬度
		double tlLon = m_tiffInfo.pGeoTrans[0];
		double tlLat = m_tiffInfo.pGeoTrans[3];


		if (strlen(m_tiffInfo.sProjRef) == 0 ||
			abs(m_tiffInfo.xRes) < 0.01 ||
			abs(m_tiffInfo.xRes) > 500000 ||
			abs(m_tiffInfo.yRes) < 0.01 ||
			abs(m_tiffInfo.yRes) > 500000)
		{
			m_tiffInfo.hasGeo = false;
			return true;
		}

		m_tiffInfo.centerLon = GetParaValue(m_tiffInfo.sProjRef, "central_meridian");
		m_tiffInfo.falseNorth = GetParaValue(m_tiffInfo.sProjRef, "false_northing");
		m_tiffInfo.falseEast = GetParaValue(m_tiffInfo.sProjRef, "false_easting");
		m_tiffInfo.scaleFact = GetParaValue(m_tiffInfo.sProjRef, "scale_factor");	

		char * tiffPjRef = m_tiffInfo.sProjRef;
		OGRSpatialReference oSRS;
		oSRS.importFromWkt(&tiffPjRef);

		if (!oSRS.IsProjected())//{//有投影信息
			return true;
		//else if(oSRS.IsGeographic())//判断是否有地理坐标信息 

		OGRSpatialReference* geographicSRS;
		geographicSRS = oSRS.CloneGeogCS();

		if(!geographicSRS->IsGeographic())
			return true;

		m_poTM2LLTransform= OGRCreateCoordinateTransformation(  &oSRS, geographicSRS );
		m_poLL2TMTransform= OGRCreateCoordinateTransformation( geographicSRS,&oSRS );

		//区域
		m_tiffInfo.zone = (int)((m_tiffInfo.centerLon + 180.0) / 6.0 + 1.0); 


		//左上角
		POINT imagePos;
		PointD llPos;
		imagePos.x = 0;
		imagePos.y = 0;
		//图像坐标到UTM坐标
		IPos2LL(imagePos,llPos);
		m_tiffInfo.rect.left = llPos.X;
		m_tiffInfo.rect.top = llPos.Y;

		//右下角 
		imagePos.x = m_tiffInfo.imageXSize;
		imagePos.y = m_tiffInfo.imageYSize;
		//图像坐标到UTM坐标
		IPos2LL(imagePos,llPos);
		m_tiffInfo.rect.right = llPos.X;
		m_tiffInfo.rect.bottom = llPos.Y;

		//中心点
		imagePos.x = m_tiffInfo.imageXSize/2;
		imagePos.y = m_tiffInfo.imageYSize/2;
		//得到经纬度
		IPos2LL(imagePos,llPos);
		m_tiffInfo.cLon = llPos.X;
		m_tiffInfo.clat = llPos.Y;

		m_tiffInfo.hasGeo = true;

		return true;
	};

	/// <summary>
	/// 图像坐标转经纬度坐标
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool IPos2LL(POINT imgPos,PointD& llPos){
		if (!IsReadySuc())
			return false;
		PointD utmPos;
		//图像坐标到UTM坐标
		IPos2Utm(imgPos ,utmPos);

		double lon = utmPos.X;
		double lat = utmPos.Y;
		//得到经纬度
		int re = m_poTM2LLTransform->Transform(1,&lon,&lat);

		if (!re) return false;

		llPos = PointD(lon,lat);
		return true;
	}


	/// <summary>
	/// 经纬度坐标转图像坐标
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool LL2IPos(PointD llPos,POINT& imgPos){
		if (!IsReadySuc())
			return false;


		double utmx=llPos.X;
		double utmy=llPos.Y;
		//得到经纬度
		int re = m_poLL2TMTransform->Transform(1,&utmx,&utmy);

		if (!re) return false;

		PointD utmPos = PointD(utmx,utmy);

		//图像坐标到UTM坐标
		Utm2IPos(utmPos,imgPos);
		return true;
	}

	/// <summary>
	/// 图像坐标转UTM坐标
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool IPos2Utm(POINT imgPos,PointD& utmPos)
	{
		if (!IsReadySuc())
			return false;

		double * geoTransform = m_tiffInfo.pGeoTrans;
		utmPos.X = geoTransform[0] + imgPos.x * geoTransform[1] + imgPos.y * geoTransform[2];
		utmPos.Y = geoTransform[3] + imgPos.x * geoTransform[4] + imgPos.y * geoTransform[5];
		return true;
	}


	/// <summary>
	/// UTM坐标转图像坐标
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool Utm2IPos(PointD utmPos,POINT& imgPos)
	{
		if (!IsReadySuc())
			return false;

		double * geotransMat = m_tiffInfo.pGeoTrans;      
		imgPos.x = 
			(int)((utmPos.X - geotransMat[0] - geotransMat[2] / geotransMat[5] * utmPos.Y + geotransMat[2] * geotransMat[3] / geotransMat[5]) / (geotransMat[1] - geotransMat[2] * geotransMat[4] / geotransMat[5]));
		imgPos.y = 
			(int)((utmPos.Y - geotransMat[3] - geotransMat[4] / geotransMat[1] * utmPos.X + geotransMat[0] * geotransMat[4] / geotransMat[1]) / (geotransMat[5] - geotransMat[2] * geotransMat[4] / geotransMat[1]));
		return true;
	}

	//读取Rect范围内指定Band的图像到Mat *注意：BandNo的序号从1开始
	bool read(RECT rect,int bandNo, Mat& mImg){
		if (!IsReadySuc())
			return false;
		//范围只能在图像中
		assert(rect.left >= 0 &&
			rect.top >= 0 &&
			rect.right <= m_tiffInfo.imageXSize &&
			rect.bottom <= m_tiffInfo.imageYSize);

		//取得通道数量
		int rasterCnt = GetRasterCount();
		assert(bandNo > 0 &&
			bandNo <= rasterCnt);

		//取得高度、宽度
		int rWidth = GetRectWidth(rect);
		int rHeight =GetRectHeight(rect);

		switch (GetType())
		{
		case 1:
			{
				//取得类型
				int matType = CV_8UC1; 
				//图像缓存
				/*		void* pixels;*/
				int bandDataLen= rWidth * rHeight;
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);

				byte *  pImageData = new byte[rWidth * rHeight];
				///循环通道取出数据 		
				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){			
					//清除缓存
					delete[] pImageData;
					pImageData = NULL;

					return false;
				} 
				int index = 0;
				if (mImg.isContinuous()){
					byte * bPixels = (byte *)mImg.data; 
					//拷贝数据
					for (int j = 0;j< bandDataLen;j++){
						bPixels[j] = pImageData[j];
					}	
				}
				else{
					//拷贝数据
					for (int h = 0;h< rHeight;h++){
						byte * bPixels = (byte *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							bPixels[r] = pImageData[index];
						}
					}	
				}

				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 2:
			{ 
				//取得类型
				int matType = CV_16UC1; 
				//数据定义
				int bandDataLen= rWidth * rHeight;
				//图像缓存
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);
				/*memset(pixels,NULL,sizeof(USHORT) * bandDataLen * rasterCnt);*/

				USHORT *  pImageData = new USHORT[rWidth * rHeight];

				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){

					//清除缓存
					delete[] pImageData;
					pImageData = NULL;
					return false;
				}

				int index = 0;
				//拷贝数据
				if (mImg.isContinuous()){
					USHORT * usPixels = (USHORT *)mImg.data;
					//拷贝数据
					for (int j = 0;j< bandDataLen;j++){
						usPixels[j] = pImageData[j];
					}	
				}
				else{
					//拷贝数据
					for (int h = 0;h< rHeight;h++){
						USHORT * usPixels = (USHORT *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							usPixels[r] = pImageData[index];
						}
					}	
				}


				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 3:
			{ 
				//取得类型
				int matType = CV_16SC1; 
				//数据定义
				int bandDataLen= rWidth * rHeight;
				//图像缓存
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);

				SHORT * pImageData = new SHORT[rWidth * rHeight];
				///循环通道取出数据 
				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){

					//清除缓存
					delete[] pImageData;
					pImageData = NULL;
					return false;
				}

				int index = 0;
				//拷贝数据
				if (mImg.isContinuous()){
					SHORT * sPixels = (SHORT *)mImg.data;
					//拷贝数据
					for (int j = 0;j< bandDataLen;j++){
						sPixels[j] = pImageData[j];
					}	
				}
				else{
					//拷贝数据
					for (int h = 0;h< rHeight;h++){
						SHORT * sPixels = (SHORT *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							sPixels[r] = pImageData[index];
						}
					}	
				} 
				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		} 
		return true;
	}

	//读取Rect范围内图像到Mat *注意：通道数量只能为1-4 如果通道数量超过的话请勿调用此方法
	bool read(RECT rect,Mat& mImg){
		if (!IsReadySuc())
			return false;
		//范围只能在图像中
		assert(rect.left >= 0 &&
			rect.top >= 0 &&
			rect.right <= m_tiffInfo.imageXSize &&
			rect.bottom <= m_tiffInfo.imageYSize);

		//取得高度、宽度
		int rWidth = GetRectWidth(rect);
		int rHeight =GetRectHeight(rect);

		//取得通道数量
		int rasterCnt = GetRasterCount();
		//通道数量只能为1-4
		assert(rasterCnt >= 1 &&
			rasterCnt <= 4);

		//取得类型
		int matType = GetMatType();
		assert(matType > -1 );
		//图像缓存
		/*		void* pixels;*/
		int bandDataLen= rWidth * rHeight;
		mImg = cv::Mat::zeros(rHeight,rWidth,matType);
		switch (GetType())
		{
		case 1:
			{
				byte *  pImageData = new byte[rWidth * rHeight];
				///循环通道取出数据
				for (int i = 1;i<=rasterCnt;i++){		
					GDALColorInterp colorInterp;
					//读取通道失败
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){			
						//清除缓存
						delete[] pImageData;
						pImageData = NULL;

						/*				delete[] pixels;*/
						return false;
					}

					int rasterIndex = i -1;
					if (rasterCnt > 1){
						if (colorInterp == GCI_BlueBand) 
							rasterIndex = 0;
						else if (colorInterp == GCI_GreenBand) 
							rasterIndex = 1;
						else if (colorInterp == GCI_RedBand) 
							rasterIndex = 2;                                         
					}

					int index = 0;
					if (mImg.isContinuous()){
						byte * bPixels = (byte *)mImg.data; 
						//拷贝数据
						for (int j = 0;j< bandDataLen;j++){
							bPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//拷贝数据
						for (int h = 0;h< rHeight;h++){
							byte * bPixels = (byte *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								bPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}

				}
				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 2:
			{
				//数据定义

				/*memset(pixels,NULL,sizeof(USHORT) * bandDataLen * rasterCnt);*/

				USHORT *  pImageData = new USHORT[rWidth * rHeight];
				///循环通道取出数据
				for (int i = 1;i<=rasterCnt;i++){
					GDALColorInterp colorInterp;
					//读取通道失败
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){
						//delete[] pixels; 

						//清除缓存
						delete[] pImageData;
						pImageData = NULL;
						return false;
					}
					int rasterIndex = i -1;
					if (rasterCnt > 1){
						if (colorInterp == GCI_BlueBand) 
							rasterIndex = 0;
						else if (colorInterp == GCI_GreenBand) 
							rasterIndex = 1;
						else if (colorInterp == GCI_RedBand) 
							rasterIndex = 2;                                         
					}
					int index = 0;
					//拷贝数据
					if (mImg.isContinuous()){
						USHORT * usPixels = (USHORT *)mImg.data;
						//拷贝数据
						for (int j = 0;j< bandDataLen;j++){
							usPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//拷贝数据
						for (int h = 0;h< rHeight;h++){
							USHORT * usPixels = (USHORT *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								usPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}
				}

				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 3:
			{
				//数据定义
				/*pixels = new SHORT[bandDataLen * rasterCnt];
				memset(pixels,NULL,sizeof(SHORT) * bandDataLen * rasterCnt);*/ 

				SHORT * pImageData = new SHORT[rWidth * rHeight];
				///循环通道取出数据
				for (int i = 1;i<=rasterCnt;i++){
					GDALColorInterp colorInterp;
					//读取通道失败
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){
						//delete[] pixels;

						//清除缓存
						delete[] pImageData;
						pImageData = NULL;
						return false;
					}
					int rasterIndex = i -1;
					if (rasterCnt > 1){
						if (colorInterp == GCI_BlueBand) 
							rasterIndex = 0;
						else if (colorInterp == GCI_GreenBand) 
							rasterIndex = 1;
						else if (colorInterp == GCI_RedBand) 
							rasterIndex = 2;                                         
					}
					int index = 0;
					//拷贝数据
					if (mImg.isContinuous()){
						SHORT * sPixels = (SHORT *)mImg.data;
						//拷贝数据
						for (int j = 0;j< bandDataLen;j++){
							sPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//拷贝数据
						for (int h = 0;h< rHeight;h++){
							SHORT * sPixels = (SHORT *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								sPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}
				}
				//清除缓存
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		}
		//赋值
		//mImg = cv::Mat(rHeight,rWidth,matType,pixels);
		return true;
	} 

	//读取Rect范围内指定Band的图像到Mat *注意：BandNo的序号从1开始
	bool read(POINT pos,int bandNo, double& value){
		if (!IsReadySuc())
			return false;
		//范围只能在图像中
		assert(pos.x >= 0 &&
			pos.y >= 0 &&
			pos.x <= m_tiffInfo.imageXSize &&
			pos.y <= m_tiffInfo.imageYSize);

		//取得通道数量
		int rasterCnt = GetRasterCount();
		assert(bandNo > 0 &&
			bandNo <= rasterCnt);

		//取得高度、宽度 
		switch (GetType())
		{
		case 1:
			{  
				byte * pImageData= new byte [1]; 
				///循环通道取出数据 		
				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(pos,bandNo,colorInterp,(void **)&pImageData)){			
 					return false;
				} 
			    value = pImageData[0];
				delete pImageData;
			}
			break;
		case 2:
			{ 
				unsigned short *  pImageData= new unsigned short [1]; 
				///循环通道取出数据 		
				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(pos,bandNo,colorInterp,(void **)&pImageData)){			
					return false;
				} 
				value = pImageData[0];
				delete pImageData;
			}
			break;
		case 3:
			{ 
				short*  pImageData = new short[1]; 
				///循环通道取出数据 		
				GDALColorInterp colorInterp;
				//读取通道失败
				if (!ReadBandData(pos,bandNo,colorInterp,(void **)&pImageData)){			
					return false;
				} 
				value = pImageData[0];
				delete pImageData;
			}
			break;
		} 
		return true;
	}
	/************************************************************************
	* 读取Band最大，最小值         
	* int     bandNo 波段号
	* double* argout double型2维数组 
	* argout[0] =min argout[1] =max 
	************************************************************************/
	bool GetBandMaxMin(int bandNo,double* argout)
		{
			//读取通道 
			GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
			CPLErr err =	pBand->ComputeRasterMinMax(0,argout);
			//判断是否出错
			if( err == CE_Failure )
			{
				return false;
			}
			return true;
		}
	//取得Mat类型
	int GetMatType()
	{
		//类型
		int type = -1;
		switch (GetType())
		{
		case 1:
			switch (GetRasterCount()){
			case 1:
				type = CV_8UC1;
				break;
			case 2:
				type = CV_8UC2;
				break;
			case 3:
				type = CV_8UC3;
				break;
			case 4:
				type = CV_8UC4;
				break;
			}
			break;
		case 2:
			switch (GetRasterCount()){
			case 1:
				type = CV_16UC1;
				break;
			case 2:
				type = CV_16UC2;
				break;
			case 3:
				type = CV_16UC3;
				break;
			case 4:
				type = CV_16UC4;
				break;
			}

			break;
		case 3:
			switch (GetRasterCount()){
			case 1:
				type = CV_16SC1;
				break;
			case 2:
				type = CV_16SC2;
				break;
			case 3:
				type = CV_16SC3;
				break;
			case 4:
				type = CV_16SC4;
				break;
			}
			break;
		}
		return type;
	}

	/// <summary>
	/// 读取指定通道数据
	/// </summary> 
	bool ReadBandData(POINT pos, int bandNo,GDALColorInterp& colorInterp, void**  posValue)
	{ 		 
		//读取通道 
		GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
		colorInterp = pBand->GetColorInterpretation();

		//读取图片中的数据
		CPLErr err;
        void * pImageData = *posValue;
		switch (GetType())
		{
		case 1:
			{
				//取数据
				err = pBand->RasterIO(GF_Read,
					pos.x,
					pos.y,
					1,
					1,
					pImageData,
					1,
					1,
					GDT_Byte,0,0);


			}
			break;
		case 2:
			//取数据
			err = pBand->RasterIO(GF_Read,
				pos.x,
				pos.y,
				1,
				1,
				pImageData,
				1,
				1,
				GDT_UInt16,
				0,
				0);
			break; 
		case 3:
			//取数据
			err = pBand->RasterIO(GF_Read,
				pos.x,
				pos.y,
				1,
				1,
				pImageData,
				1,
				1,
				GDT_Int16,
				0,
				0);
			break; 
		}
		//判断是否出错
		if( err == CE_Failure )
		{
			return false;
		}
		return true;
	}

	/// <summary>
	/// 读取指定通道数据
	/// </summary> 
	bool ReadBandData(RECT rect,int readW,int readH, int bandNo,GDALColorInterp& colorInterp, void ** hImageHandle)
	{

		if (!IsReadySuc())
			return false;;

		if (bandNo >  GetRasterCount() || bandNo < 1)
			return false;


		//读取通道 
		GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
		colorInterp = pBand->GetColorInterpretation();

		//读取图片中的数据
		CPLErr err;

		//取得高度、宽度
		int rWidth = GetRectWidth(rect);
		int rHeight = GetRectHeight(rect);

		void * pImageData = *hImageHandle;
		switch (GetType())
		{
		case 1:
			{
				//取数据
				err = pBand->RasterIO(GF_Read,
					rect.left,
					rect.top,
					rWidth,
					rHeight,
					pImageData,
					readW,
					readH,
					GDT_Byte,0,0);


			}
			break;
		case 2:
			//取数据
			err = pBand->RasterIO(GF_Read,
				rect.left,
				rect.top,
				rWidth,
				rHeight,
				pImageData,
				readW,
				readH,
				GDT_UInt16,
				0,
				0);
			break; 
		case 3:
			//取数据
			err = pBand->RasterIO(GF_Read,
				rect.left,
				rect.top,
				rWidth,
				rHeight,
				pImageData,
				readW,
				readH,
				GDT_Int16,
				0,
				0);
			break; 
		}
		//判断是否出错
		if( err == CE_Failure )
		{
			return false;
		}
		return true;
	}

	//图像类型
	inline GDALDataType GetType() {return  m_tiffInfo.type;}
	inline int GetRasterCount() {return m_tiffInfo.rasterCount;}
	//取得一个Rect的宽度
	inline static int  GetRectWidth(RECT& rect){return rect.right-rect.left + 1;}
	//取得一个Rect的高度
	inline static int  GetRectHeight(RECT& rect){return rect.bottom-rect.top + 1;}
	/*增加地理信息一个Rect的高度
	*filePath 路径
	*picRect 图像在原图的位置
	*gu 原图GDAL信息
	*/
	static bool AddGeoInfo(const char *  filePath,RECT& picRect,CGdalUtility& gu){
		//注册GDAL驱动
		GDALAllRegister(); 
		//中文路径支持
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
		//读取文件DataSet
		GDALDataset* poDataset = (GDALDataset *) GDALOpen( filePath, GA_Update );
		if (poDataset == NULL)
			return false;

		POINT leftPos;
		leftPos.x = picRect.left;
		leftPos.y = picRect.top;

		//确认中心经纬度
		PointD leftutm;
		gu.IPos2Utm(leftPos,leftutm);

		GeoTiffInfo tiffInfo = gu.GetGeoTiffInfo();
		//坐标变换矩阵
		double * geoTransform = tiffInfo.pGeoTrans;
		geoTransform[0] = leftutm.X;
		geoTransform[3] = leftutm.Y;

		poDataset->SetGeoTransform(geoTransform);
		poDataset->SetProjection(tiffInfo.sProjRef);

		//写信息
		poDataset->FlushCache();
		GDALClose(poDataset);
		poDataset = NULL;
		return true;
	}  
private:
	GDALDataset* m_poDataset; 	//GDAL数据集
	GeoTiffInfo m_tiffInfo; 
	OGRCoordinateTransformation *m_poTM2LLTransform;
	OGRCoordinateTransformation *m_poLL2TMTransform;
	//销毁句柄
	void Dispose(){
		if (IsReadySuc())
			GDALClose(m_poDataset);
		m_poDataset = NULL; 
		//初始化
		memset(&m_tiffInfo,NULL,sizeof(GeoTiffInfo));

		if (m_poTM2LLTransform != NULL)
		{
			OGRCoordinateTransformation::DestroyCT(m_poTM2LLTransform);
			m_poTM2LLTransform= NULL;
		}

		if (m_poLL2TMTransform != NULL)
		{
			OGRCoordinateTransformation::DestroyCT(m_poLL2TMTransform);
			m_poLL2TMTransform= NULL;
		}

	}

	//解析参数
	double GetParaValue(string projRef, string ParaName)
	{
		int ParaNameLen = (int)ParaName.length();
		int ParaNameSite= (int)projRef.find(ParaName);

		if(ParaNameSite == -1)
			return 0;

		int ParaSite = ParaNameSite+ParaNameLen+2;
		string valueS;
		char tempS = projRef.at(ParaSite);
		char sign = ']';
		while (tempS!=sign)
		{
			valueS+=tempS;
			ParaSite++;
			tempS = projRef.at(ParaSite);
		}
		return (double)atof(valueS.c_str());
	}

};



#endif