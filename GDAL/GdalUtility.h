#ifndef __GdalUtility_h
#define __GdalUtility_h

#include "stdafx.h"  
#include "gdal.h"
#include "gdal_version.h"
#include "gdal_priv.h"
#include "gdal_frmts.h"
#include "ogr_spatialref.h" 
// Windows ͷ�ļ�:
#include <windows.h> 
#include <cstring>
#include "opencv2\core\core.hpp"
using namespace cv; 


typedef unsigned char byte;
//����Ϣ
struct PointD{
public:
	double X;
	double Y;
	PointD (double x = 0, double y = 0):
	X(x),
		Y(y)
	{}
};

//����Ϣ
struct RectD{
public:
	double   left;
	double   top;
	double   right;
	double   bottom;
};

/// <summary>
/// ͼ����Ϣ
/// </summary>
struct GeoTiffInfo
{
public:
	bool hasGeo; //�Ƿ����������Ϣ
	double pGeoTrans[6];//����任6���� 
	char sProjRef[1024];//������Ϣ����
	double cLon;//���ľ���
	double clat;//����γ��
	RectD rect;
	double xRes;//X�����ֱ���
	double yRes;//Y�����ֱ���
	int imageXSize;//��ͼ���
	int imageYSize;//��ͼ�߶�
	int zone;//���ĵ��������� 1��60
	/// <summary>
	///ͼ������
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
	GDALDataType type;//ͼ������
	int rasterCount;//ͼ������ 
	//double minNum;//ͼ�������е����ֵ��������
	//double maxNum;//ͼ�������е���Сֵ��������
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

	//���캯��
	CGdalUtility::CGdalUtility()
	{
		//ע��GDAL����
		GDALAllRegister(); 
		//����·��֧��
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
		//��ʼ��
		m_poDataset = NULL;
		m_poLL2TMTransform = NULL;
		m_poTM2LLTransform= NULL;
	}

	//��������
	~CGdalUtility()
	{
		Dispose();
	}

	//��ͼ�� 
	bool Open(const char* lpszFilePathName)
	{  
		//���پ�����
		Dispose();
		//��ȡ���ݼ�
		m_poDataset = (GDALDataset *) GDALOpen( lpszFilePathName, GA_ReadOnly );

		if (m_poDataset == NULL) 
			return false;

		//ͨ������
		m_tiffInfo.rasterCount = m_poDataset->GetRasterCount();

		//ͼ���С
		m_tiffInfo.imageXSize = m_poDataset->GetRasterXSize();
		m_tiffInfo.imageYSize = m_poDataset->GetRasterYSize();

		//��ȡ��������
		GDALRasterBand *poBand;
		//��ȡ��������
		poBand = m_poDataset->GetRasterBand(1);
		m_tiffInfo.type =  poBand->GetRasterDataType(); 

		////���ݳ����������Сֵ
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

		//��γ����Ϣ
		const char* projRef = m_poDataset->GetProjectionRef(); 
		strcpy_s(m_tiffInfo.sProjRef ,projRef);	

		//ת������
		m_poDataset->GetGeoTransform(m_tiffInfo.pGeoTrans); 
		//ȡ�õ���ֱ���
		m_tiffInfo.xRes = m_tiffInfo.pGeoTrans[1];
		m_tiffInfo.yRes = m_tiffInfo.pGeoTrans[5];
		//ȡ�þ��� γ��
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

		if (!oSRS.IsProjected())//{//��ͶӰ��Ϣ
			return true;
		//else if(oSRS.IsGeographic())//�ж��Ƿ��е���������Ϣ 

		OGRSpatialReference* geographicSRS;
		geographicSRS = oSRS.CloneGeogCS();

		if(!geographicSRS->IsGeographic())
			return true;

		m_poTM2LLTransform= OGRCreateCoordinateTransformation(  &oSRS, geographicSRS );
		m_poLL2TMTransform= OGRCreateCoordinateTransformation( geographicSRS,&oSRS );

		//����
		m_tiffInfo.zone = (int)((m_tiffInfo.centerLon + 180.0) / 6.0 + 1.0); 


		//���Ͻ�
		POINT imagePos;
		PointD llPos;
		imagePos.x = 0;
		imagePos.y = 0;
		//ͼ�����굽UTM����
		IPos2LL(imagePos,llPos);
		m_tiffInfo.rect.left = llPos.X;
		m_tiffInfo.rect.top = llPos.Y;

		//���½� 
		imagePos.x = m_tiffInfo.imageXSize;
		imagePos.y = m_tiffInfo.imageYSize;
		//ͼ�����굽UTM����
		IPos2LL(imagePos,llPos);
		m_tiffInfo.rect.right = llPos.X;
		m_tiffInfo.rect.bottom = llPos.Y;

		//���ĵ�
		imagePos.x = m_tiffInfo.imageXSize/2;
		imagePos.y = m_tiffInfo.imageYSize/2;
		//�õ���γ��
		IPos2LL(imagePos,llPos);
		m_tiffInfo.cLon = llPos.X;
		m_tiffInfo.clat = llPos.Y;

		m_tiffInfo.hasGeo = true;

		return true;
	};

	/// <summary>
	/// ͼ������ת��γ������
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool IPos2LL(POINT imgPos,PointD& llPos){
		if (!IsReadySuc())
			return false;
		PointD utmPos;
		//ͼ�����굽UTM����
		IPos2Utm(imgPos ,utmPos);

		double lon = utmPos.X;
		double lat = utmPos.Y;
		//�õ���γ��
		int re = m_poTM2LLTransform->Transform(1,&lon,&lat);

		if (!re) return false;

		llPos = PointD(lon,lat);
		return true;
	}


	/// <summary>
	/// ��γ������תͼ������
	/// </summary>
	/// <param name="imgPos"></param>
	/// <param name="geoTransform"></param>
	/// <param name="utmPos"></param>
	bool LL2IPos(PointD llPos,POINT& imgPos){
		if (!IsReadySuc())
			return false;


		double utmx=llPos.X;
		double utmy=llPos.Y;
		//�õ���γ��
		int re = m_poLL2TMTransform->Transform(1,&utmx,&utmy);

		if (!re) return false;

		PointD utmPos = PointD(utmx,utmy);

		//ͼ�����굽UTM����
		Utm2IPos(utmPos,imgPos);
		return true;
	}

	/// <summary>
	/// ͼ������תUTM����
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
	/// UTM����תͼ������
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

	//��ȡRect��Χ��ָ��Band��ͼ��Mat *ע�⣺BandNo����Ŵ�1��ʼ
	bool read(RECT rect,int bandNo, Mat& mImg){
		if (!IsReadySuc())
			return false;
		//��Χֻ����ͼ����
		assert(rect.left >= 0 &&
			rect.top >= 0 &&
			rect.right <= m_tiffInfo.imageXSize &&
			rect.bottom <= m_tiffInfo.imageYSize);

		//ȡ��ͨ������
		int rasterCnt = GetRasterCount();
		assert(bandNo > 0 &&
			bandNo <= rasterCnt);

		//ȡ�ø߶ȡ����
		int rWidth = GetRectWidth(rect);
		int rHeight =GetRectHeight(rect);

		switch (GetType())
		{
		case 1:
			{
				//ȡ������
				int matType = CV_8UC1; 
				//ͼ�񻺴�
				/*		void* pixels;*/
				int bandDataLen= rWidth * rHeight;
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);

				byte *  pImageData = new byte[rWidth * rHeight];
				///ѭ��ͨ��ȡ������ 		
				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){			
					//�������
					delete[] pImageData;
					pImageData = NULL;

					return false;
				} 
				int index = 0;
				if (mImg.isContinuous()){
					byte * bPixels = (byte *)mImg.data; 
					//��������
					for (int j = 0;j< bandDataLen;j++){
						bPixels[j] = pImageData[j];
					}	
				}
				else{
					//��������
					for (int h = 0;h< rHeight;h++){
						byte * bPixels = (byte *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							bPixels[r] = pImageData[index];
						}
					}	
				}

				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 2:
			{ 
				//ȡ������
				int matType = CV_16UC1; 
				//���ݶ���
				int bandDataLen= rWidth * rHeight;
				//ͼ�񻺴�
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);
				/*memset(pixels,NULL,sizeof(USHORT) * bandDataLen * rasterCnt);*/

				USHORT *  pImageData = new USHORT[rWidth * rHeight];

				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){

					//�������
					delete[] pImageData;
					pImageData = NULL;
					return false;
				}

				int index = 0;
				//��������
				if (mImg.isContinuous()){
					USHORT * usPixels = (USHORT *)mImg.data;
					//��������
					for (int j = 0;j< bandDataLen;j++){
						usPixels[j] = pImageData[j];
					}	
				}
				else{
					//��������
					for (int h = 0;h< rHeight;h++){
						USHORT * usPixels = (USHORT *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							usPixels[r] = pImageData[index];
						}
					}	
				}


				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 3:
			{ 
				//ȡ������
				int matType = CV_16SC1; 
				//���ݶ���
				int bandDataLen= rWidth * rHeight;
				//ͼ�񻺴�
				mImg = cv::Mat::zeros(rHeight,rWidth,matType);

				SHORT * pImageData = new SHORT[rWidth * rHeight];
				///ѭ��ͨ��ȡ������ 
				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
				if (!ReadBandData(rect,rWidth,rHeight,bandNo,colorInterp,(void **)&pImageData)){

					//�������
					delete[] pImageData;
					pImageData = NULL;
					return false;
				}

				int index = 0;
				//��������
				if (mImg.isContinuous()){
					SHORT * sPixels = (SHORT *)mImg.data;
					//��������
					for (int j = 0;j< bandDataLen;j++){
						sPixels[j] = pImageData[j];
					}	
				}
				else{
					//��������
					for (int h = 0;h< rHeight;h++){
						SHORT * sPixels = (SHORT *)mImg.ptr<uchar>(h);
						for (int r = 0;r <rWidth;r++,index++){
							sPixels[r] = pImageData[index];
						}
					}	
				} 
				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		} 
		return true;
	}

	//��ȡRect��Χ��ͼ��Mat *ע�⣺ͨ������ֻ��Ϊ1-4 ���ͨ�����������Ļ�������ô˷���
	bool read(RECT rect,Mat& mImg){
		if (!IsReadySuc())
			return false;
		//��Χֻ����ͼ����
		assert(rect.left >= 0 &&
			rect.top >= 0 &&
			rect.right <= m_tiffInfo.imageXSize &&
			rect.bottom <= m_tiffInfo.imageYSize);

		//ȡ�ø߶ȡ����
		int rWidth = GetRectWidth(rect);
		int rHeight =GetRectHeight(rect);

		//ȡ��ͨ������
		int rasterCnt = GetRasterCount();
		//ͨ������ֻ��Ϊ1-4
		assert(rasterCnt >= 1 &&
			rasterCnt <= 4);

		//ȡ������
		int matType = GetMatType();
		assert(matType > -1 );
		//ͼ�񻺴�
		/*		void* pixels;*/
		int bandDataLen= rWidth * rHeight;
		mImg = cv::Mat::zeros(rHeight,rWidth,matType);
		switch (GetType())
		{
		case 1:
			{
				byte *  pImageData = new byte[rWidth * rHeight];
				///ѭ��ͨ��ȡ������
				for (int i = 1;i<=rasterCnt;i++){		
					GDALColorInterp colorInterp;
					//��ȡͨ��ʧ��
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){			
						//�������
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
						//��������
						for (int j = 0;j< bandDataLen;j++){
							bPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//��������
						for (int h = 0;h< rHeight;h++){
							byte * bPixels = (byte *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								bPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}

				}
				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 2:
			{
				//���ݶ���

				/*memset(pixels,NULL,sizeof(USHORT) * bandDataLen * rasterCnt);*/

				USHORT *  pImageData = new USHORT[rWidth * rHeight];
				///ѭ��ͨ��ȡ������
				for (int i = 1;i<=rasterCnt;i++){
					GDALColorInterp colorInterp;
					//��ȡͨ��ʧ��
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){
						//delete[] pixels; 

						//�������
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
					//��������
					if (mImg.isContinuous()){
						USHORT * usPixels = (USHORT *)mImg.data;
						//��������
						for (int j = 0;j< bandDataLen;j++){
							usPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//��������
						for (int h = 0;h< rHeight;h++){
							USHORT * usPixels = (USHORT *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								usPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}
				}

				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		case 3:
			{
				//���ݶ���
				/*pixels = new SHORT[bandDataLen * rasterCnt];
				memset(pixels,NULL,sizeof(SHORT) * bandDataLen * rasterCnt);*/ 

				SHORT * pImageData = new SHORT[rWidth * rHeight];
				///ѭ��ͨ��ȡ������
				for (int i = 1;i<=rasterCnt;i++){
					GDALColorInterp colorInterp;
					//��ȡͨ��ʧ��
					if (!ReadBandData(rect,rWidth,rHeight,i,colorInterp,(void **)&pImageData)){
						//delete[] pixels;

						//�������
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
					//��������
					if (mImg.isContinuous()){
						SHORT * sPixels = (SHORT *)mImg.data;
						//��������
						for (int j = 0;j< bandDataLen;j++){
							sPixels[j * rasterCnt + rasterIndex] = pImageData[j];
						}	
					}
					else{
						//��������
						for (int h = 0;h< rHeight;h++){
							SHORT * sPixels = (SHORT *)mImg.ptr<uchar>(h);
							for (int r = 0;r <rWidth;r++,index++){
								sPixels[r * rasterCnt + rasterIndex] = pImageData[index];
							}
						}	
					}
				}
				//�������
				delete[] pImageData;
				pImageData = NULL;
			}
			break;
		}
		//��ֵ
		//mImg = cv::Mat(rHeight,rWidth,matType,pixels);
		return true;
	} 

	//��ȡRect��Χ��ָ��Band��ͼ��Mat *ע�⣺BandNo����Ŵ�1��ʼ
	bool read(POINT pos,int bandNo, double& value){
		if (!IsReadySuc())
			return false;
		//��Χֻ����ͼ����
		assert(pos.x >= 0 &&
			pos.y >= 0 &&
			pos.x <= m_tiffInfo.imageXSize &&
			pos.y <= m_tiffInfo.imageYSize);

		//ȡ��ͨ������
		int rasterCnt = GetRasterCount();
		assert(bandNo > 0 &&
			bandNo <= rasterCnt);

		//ȡ�ø߶ȡ���� 
		switch (GetType())
		{
		case 1:
			{  
				byte * pImageData= new byte [1]; 
				///ѭ��ͨ��ȡ������ 		
				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
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
				///ѭ��ͨ��ȡ������ 		
				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
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
				///ѭ��ͨ��ȡ������ 		
				GDALColorInterp colorInterp;
				//��ȡͨ��ʧ��
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
	* ��ȡBand�����Сֵ         
	* int     bandNo ���κ�
	* double* argout double��2ά���� 
	* argout[0] =min argout[1] =max 
	************************************************************************/
	bool GetBandMaxMin(int bandNo,double* argout)
		{
			//��ȡͨ�� 
			GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
			CPLErr err =	pBand->ComputeRasterMinMax(0,argout);
			//�ж��Ƿ����
			if( err == CE_Failure )
			{
				return false;
			}
			return true;
		}
	//ȡ��Mat����
	int GetMatType()
	{
		//����
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
	/// ��ȡָ��ͨ������
	/// </summary> 
	bool ReadBandData(POINT pos, int bandNo,GDALColorInterp& colorInterp, void**  posValue)
	{ 		 
		//��ȡͨ�� 
		GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
		colorInterp = pBand->GetColorInterpretation();

		//��ȡͼƬ�е�����
		CPLErr err;
        void * pImageData = *posValue;
		switch (GetType())
		{
		case 1:
			{
				//ȡ����
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
			//ȡ����
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
			//ȡ����
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
		//�ж��Ƿ����
		if( err == CE_Failure )
		{
			return false;
		}
		return true;
	}

	/// <summary>
	/// ��ȡָ��ͨ������
	/// </summary> 
	bool ReadBandData(RECT rect,int readW,int readH, int bandNo,GDALColorInterp& colorInterp, void ** hImageHandle)
	{

		if (!IsReadySuc())
			return false;;

		if (bandNo >  GetRasterCount() || bandNo < 1)
			return false;


		//��ȡͨ�� 
		GDALRasterBand* pBand = m_poDataset->GetRasterBand(bandNo);
		colorInterp = pBand->GetColorInterpretation();

		//��ȡͼƬ�е�����
		CPLErr err;

		//ȡ�ø߶ȡ����
		int rWidth = GetRectWidth(rect);
		int rHeight = GetRectHeight(rect);

		void * pImageData = *hImageHandle;
		switch (GetType())
		{
		case 1:
			{
				//ȡ����
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
			//ȡ����
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
			//ȡ����
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
		//�ж��Ƿ����
		if( err == CE_Failure )
		{
			return false;
		}
		return true;
	}

	//ͼ������
	inline GDALDataType GetType() {return  m_tiffInfo.type;}
	inline int GetRasterCount() {return m_tiffInfo.rasterCount;}
	//ȡ��һ��Rect�Ŀ��
	inline static int  GetRectWidth(RECT& rect){return rect.right-rect.left + 1;}
	//ȡ��һ��Rect�ĸ߶�
	inline static int  GetRectHeight(RECT& rect){return rect.bottom-rect.top + 1;}
	/*���ӵ�����Ϣһ��Rect�ĸ߶�
	*filePath ·��
	*picRect ͼ����ԭͼ��λ��
	*gu ԭͼGDAL��Ϣ
	*/
	static bool AddGeoInfo(const char *  filePath,RECT& picRect,CGdalUtility& gu){
		//ע��GDAL����
		GDALAllRegister(); 
		//����·��֧��
		CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
		//��ȡ�ļ�DataSet
		GDALDataset* poDataset = (GDALDataset *) GDALOpen( filePath, GA_Update );
		if (poDataset == NULL)
			return false;

		POINT leftPos;
		leftPos.x = picRect.left;
		leftPos.y = picRect.top;

		//ȷ�����ľ�γ��
		PointD leftutm;
		gu.IPos2Utm(leftPos,leftutm);

		GeoTiffInfo tiffInfo = gu.GetGeoTiffInfo();
		//����任����
		double * geoTransform = tiffInfo.pGeoTrans;
		geoTransform[0] = leftutm.X;
		geoTransform[3] = leftutm.Y;

		poDataset->SetGeoTransform(geoTransform);
		poDataset->SetProjection(tiffInfo.sProjRef);

		//д��Ϣ
		poDataset->FlushCache();
		GDALClose(poDataset);
		poDataset = NULL;
		return true;
	}  
private:
	GDALDataset* m_poDataset; 	//GDAL���ݼ�
	GeoTiffInfo m_tiffInfo; 
	OGRCoordinateTransformation *m_poTM2LLTransform;
	OGRCoordinateTransformation *m_poLL2TMTransform;
	//���پ��
	void Dispose(){
		if (IsReadySuc())
			GDALClose(m_poDataset);
		m_poDataset = NULL; 
		//��ʼ��
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

	//��������
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