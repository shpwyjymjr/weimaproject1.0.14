#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <unistd.h>
#endif // _WIN32

#include <math.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <vector>
#include <cv.h>
#include <cxcore.h>
#include <cvaux.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "geefish_tof.h"

#define VID    0x04f2
#define PID    0xb008
#define WIDTH  640
#define HEIGHT 240


typedef struct tof_calib {
  uint16_t valid;
  uint16_t ver;
  uint16_t temperature;
  uint16_t width, height;
  float *data;
} tof_calib_t;

enum tof_state {
  TOF_IDLE = 0x01,
  TOF_CALIBRATE,
  TOF_CALIBRATING,
  TOF_DONE,
};

tof_state state = TOF_IDLE;
frame_cb tof_handler;
tof_cloud_t cloud;

float amplitude[320 * 240] = { 0 };
float depth[320*240] = { 0 };
float calibration[320*240] = { 0 };                         //���ڴ洢�������󣬴洢���Ǳ�׼�¶ȶ�Ӧ�ľ���֡�Ͳο�����Ĳ�ֵ��ÿ�ε���calibration������ʱ����Ҫ����
float grayscal[320 * 240] = { 0 };
tof_point_t point_3D[320 * 240];
extern float fStandardDistance = 0;
extern int nFlag;
char *calibfilename;
int nFlag_RaworCorrect = 0;                                 //���ڱ�ʾ��ԭʼ���ݻ��ǽ���֮������ݣ�ԭʼ���ݱ�־Ϊ0������֮������ݱ�־Ϊ1.һ��ս��������Ϊ�궨��״̬�£�����Ϊ0���궨�����б���Ϊ0���궨��������Ϊ1
std::vector<float> vFrameCenterDepth;                       //���ڼ�����Ưϵ����С���˽�ľ��룬v1.0.12�汾�洢��������֡�����ĵ㣬v1.0.13�汾�洢��������֡�ľ�ֵ
std::vector<int> vTemperatureSeries;                        //���ڼ�����Ưϵ����С���˽���¶�
int nFrameCount = 0;                                        //������С���˷�����֡��������,ͬʱ�����������̿��Ƶı�־,�궨������Ҫ����
int nStandardTemperature = 0;                               //���ڴ洢ƽ���ı�׼�¶ȣ�ÿ�ε���calibration������ʱ����Ҫ����
float standardCalib[320 * 240] = { 0 };                     //���ڴ洢ƽ���ı�׼����֡��ÿ�ε���calibration������ʱ����Ҫ����
int nCalibCount = 0;                                        //���ڼ�¼����nStandardTemperature��standardCalib[320 * 240]���������궨������Ҫ����
float delta1 = 0;                                           //���ڼ�¼����õ����������ϵ������Ϊб�ʣ�����궨���̿���ѡ������
float delta2 = 0;                                           //���ڼ�¼����õ����������ϵ������Ϊ�ؾ࣬����궨���̿���ѡ������

int nFirstCount = 0;                                        //����tof����ڸտ�����ʱ�����ݲ��ȣ�������Ҫ�����տ�ʼ�Ĳ������ݣ��������������ʼ�׶���Ҫ��������֡�ļ�����
float fAveragePixelDistance = 0.0;

#define fp_fract 9
#define coeff_1 101
#define coeff_2 503
#define coeff_3 402

unsigned int active_int_time = 0;


std::mutex sLock;
int nTemp = 0;

/*static*/ inline int32_t __fp_mul(int32_t a, int32_t b)
{
  int32_t r = (int32_t)((int32_t)((int32_t)a * b) >> fp_fract);

  return r;
}

/*static*/ inline int32_t __fp_div(int32_t a, int32_t b)
{
  int32_t r = (int32_t)((((int32_t)a << fp_fract)) / b);

  return r;
}

tof_calib_t calib = {0, 0, 0, 0, 0};

inline int DCS2INT1(const uint16_t &data)
{
  return data & 0xFFFF;
}

inline int32_t fp_atan2(int16_t y, int16_t x)
{
  int32_t fp_y = ((int32_t)y << fp_fract);
  int32_t fp_x = ((int32_t)x << fp_fract);

  int32_t mask = (fp_y >> 31);
  int32_t fp_abs_y = (mask + fp_y) ^ mask;

  mask = (fp_x >> 31);
  int32_t fp_abs_x = (mask + fp_x) ^ mask;

  int32_t r, angle;

  if ((fp_abs_y < 100) && (fp_abs_x < 100))
    return 0;


  if (x >= 0) { 
    r = __fp_div((fp_x - fp_abs_y), (fp_x + fp_abs_y));

    angle =
      __fp_mul(coeff_1, __fp_mul(r, __fp_mul(r, r))) -
      __fp_mul(coeff_2, r) + coeff_3;
  } else {
    r = __fp_div((fp_x + fp_abs_y), (fp_abs_y - fp_x));
    angle =
      __fp_mul(coeff_1, __fp_mul(r, __fp_mul(r, r))) -
      __fp_mul(coeff_2, r) + 3 * coeff_3;
  }

  if (y < 0)
    return (-angle);    // negate if in quad 3 or 4
  else
    return (angle);
}

void raw_cb(uint16_t *frame) {
	//sLock.lock();
	const uint16_t *data = (const uint16_t *)frame;
	// uint32_t sum = 0;
	// for(int i=0;i<frame->width*frame->height;i++){
	//   sum += data[i];
	// }
	// return;

	int width = 320;
	int height = 120;
	const int hh = height / 2;
	const int wh = width / 2;

	const int dcs_size = width*height;
	const uint16_t *DCS0 = data;
	const uint16_t *DCS1 = DCS0 + dcs_size;
	const uint16_t *DCS2 = DCS1 + dcs_size;
	const uint16_t *DCS3 = DCS2 + dcs_size;
	//printf("DCS0's address is %04x\n", DCS0);
	//printf("DCS1's address is %04x\n", DCS1);
	//printf("DCS2's address is %04x\n", DCS2);
	//printf("DCS3's address is %04x\n", DCS3);

	const double t1 = 3.1416 * (23.87 / 24.0);
	const double t2 = (23.87 / 24.0 / 512.0);
	const double t3 = 3.2f * (wh / 160.0f) / 1000.0f / (float)(wh) / 0.012f;
	const double t4 = 2.4f * (hh / 120.0f) / 1000.0f / (float)(hh) / 0.012f;

	int temperature = 0;

	uint8_t *pp = (uint8_t *)frame;
	int CurrentTemperature = (int)(pp[8*dcs_size - 1] * 256) + (int)(pp[8*dcs_size - 2]);

	//printf("%s: %d\n", __FILE__, __LINE__);
	int d0 = 0, d1 = 0, d2 = 0, d3 = 0;
	int pos = width * height + width;
	int kk = 0;
	short tt;
	for (int ii = height; ii > 0; ii -= 2)
	{
		pos -= 3 * width;
		for (int j = 0; j < width; j++)
		{
			d0 = DCS2INT1(DCS0[pos]);
			d1 = DCS2INT1(DCS1[pos]);
			d2 = DCS2INT1(DCS2[pos]);
			d3 = DCS2INT1(DCS3[pos]);
			pos++;

#ifdef DUMP_DCS
			tt = d0; fwrite(&tt, 1, sizeof(int16_t), fout0);
			tt = d1; fwrite(&tt, 1, sizeof(int16_t), fout1);
			tt = d2; fwrite(&tt, 1, sizeof(int16_t), fout2);
			tt = d3; fwrite(&tt, 1, sizeof(int16_t), fout3);
#endif
			grayscal[kk] = d0;

			int arg1 = (d1 - d3);
			int arg2 = (d0 - d2);
			double distance = static_cast<int>((t1 + t2 * fp_atan2(arg1, arg2)) * 1000);
			depth[kk] = static_cast<float>(distance);

			double x = t3 * distance * (float)(j - wh);
			double y = -t4 * distance * (float)(ii / 2);

			float amplitude_tmp = sqrt(arg1 * arg1 + arg2 * arg2) / 2;
			amplitude[kk] = amplitude_tmp;

			point_3D[kk].x = static_cast<float>(x);
			point_3D[kk].y = static_cast<float>(y);
			point_3D[kk].z = static_cast<float>(distance);
			kk++;
		}
	}

	for (int ii = 1; ii < height; ii += 2)
	{
		for (int j = 0; j < width; j++)
		{
			d0 = DCS2INT1(DCS0[pos]);
			d1 = DCS2INT1(DCS1[pos]);
			d2 = DCS2INT1(DCS2[pos]);
			d3 = DCS2INT1(DCS3[pos]);
			pos++;

#ifdef DUMP_DCS
			tt = d0; fwrite(&tt, 1, sizeof(int16_t), fout0);
			tt = d1; fwrite(&tt, 1, sizeof(int16_t), fout1);
			tt = d2; fwrite(&tt, 1, sizeof(int16_t), fout2);
			tt = d3; fwrite(&tt, 1, sizeof(int16_t), fout3);
#endif
			grayscal[kk] = d0;

			int arg1 = (d1 - d3);
			int arg2 = (d0 - d2);
			double distance = static_cast<int>((t1 + t2 * fp_atan2(arg1, arg2)) * 1000);
			depth[kk] = static_cast<float>(distance);

			double x = t3 * distance * (float)(j - wh);
			double y = t4 * distance * (float)(ii / 2);

			float amplitude_tmp = sqrt(arg1 * arg1 + arg2 * arg2) / 2;
			amplitude[kk] = amplitude_tmp;
			point_3D[kk].x = static_cast<float>(x);
			point_3D[kk].y = static_cast<float>(y);
			point_3D[kk].z = static_cast<float>(distance);

			kk++;

		}
		pos += width;
	}

	if (CurrentTemperature == 65535)
	{
		point_3D[kk - 1].x = 12345.0f;
		point_3D[kk - 1].y = 12345.0f;
		point_3D[kk - 1].z = 12345.0f;
	}

	//�¶��쳣��ֱ�ӷ���
	if (CurrentTemperature == 65535)
		return;

	//�����¶ȣ��õ�0�µ��¶�
	int nNegative = CurrentTemperature;
	short nChange = (short)nNegative;
	CurrentTemperature = (int)nChange;

	cloud.width = width;
	cloud.height = height;
	int nFrameWidth = width;
	int nFrameHeight = height;
	cloud.points = point_3D;
	cloud.distance = depth;
	cloud.amplitude = amplitude;
	cloud.temperature = CurrentTemperature;
	cloud.gray = grayscal;

	//if (calib.valid == 0) 
	//{
		if (state == TOF_CALIBRATING)
		{
			/*if (nFrameCount == 0)
			{
				memset(calibration, 0, 320 * 240 * sizeof(float));
				memset(standardCalib, 0, 320 * 240 * sizeof(float));
				nStandardTemperature = 0;
				nFlag_RaworCorrect = 0;
				delta1 = 0;
				delta2 = 0;
				nFirstCount = 0;
			}*/

			//������ʼ֡�׶�
			int nFirstThresh = 50;                                  //���ڿ��ƶ���֡������
			if (nFirstCount < nFirstThresh)
			{
				nFirstCount++;
				std::cout << "Discarding... " << nFirstCount << std::endl;
				return;
			}

			//����ȡ��ֵ��Ӧ������
			int nFrameThresh = 15000;                               //���ڿ��Ʋ���֡������
			if (nFrameCount < nFrameThresh)
			{
				int nTemptFrameThresh = 3000;                       //���ڿ����¶�ɸѡ����ʼ֡��λ��
				if (vTemperatureSeries.size() > nTemptFrameThresh)
				{
					float fAbsTempt = (CurrentTemperature - vTemperatureSeries[vTemperatureSeries.size() - 1]);
					if (fAbsTempt > 6)
						return;
				}

				if (CurrentTemperature < 650)                      //��65�����µ����ݽ���Ԥ���������65�����ֵΪ�ݶ�
				{
					for (int n = 0; n < width*height; n++)
					{
						depth[n] -= (650 - CurrentTemperature) * 0.28f;                                       //�¶�Խ�ͣ�����Խ�࣬�ó�ʼ�׶����Զȸ���
					}
				}
				else
				{
					for (int n = 0; n < width*height; n++)
					{
						depth[n] -= (CurrentTemperature - 650) * 0.28f;                                       //�¶�Խ�ͣ�����Խ�࣬�ó�ʼ�׶����Զȸ���
					}
				}

				float fTotal = 0;
				for (size_t i = 0; i < nFrameHeight * nFrameWidth; i++)
				{
					fTotal += depth[i];
				}
				float fAverage = fTotal / (nFrameHeight * nFrameWidth);
				vFrameCenterDepth.push_back(fAverage);
				vTemperatureSeries.push_back(CurrentTemperature);
				nFrameCount++;		

				std::cout << "Sampling... " << nFrameCount << std::endl;
			}

			//���ݵ�֡��ȡ��������������С���˷���
			float fA = 0, fB = 0;
			if (nFrameCount == nFrameThresh)
			{
				/*
				����������ߣ�D = aT + b��D��T�ֱ��Ӧ���������¶Ⱦ�������
				    [ 
					  D1
					  D2
				D =  ....
				      D_FT
				    ]
 
                    
					[
					  T1     1
					  T2     1
			    T =   ....
				      T_FT   1
					]

			    ���Ľ����ϵ������
                    [
					  a
				X =   b
				    ]

				*/
				
				int nSize = vFrameCenterDepth.size();
				float* pT = new float[2 * nSize];                
				float* pD = new float[nSize];
				for (size_t i = 0; i < nSize; i++)
				{
					//��֯�¶Ⱦ���
					pT[2 * i] = vTemperatureSeries[i];
					pT[2 * i + 1] = 1;
					//��֯�������
					pD[i] = vFrameCenterDepth[i];
				}

				CvMat* T = cvCreateMat(nSize, 2, CV_32FC1);
				CvMat* X = cvCreateMat(2, 1, CV_32FC1);
				CvMat* D = cvCreateMat(nSize, 1, CV_32FC1);

				cvSetData(T, pT, CV_AUTOSTEP);
				cvSetData(D, pD, CV_AUTOSTEP);
				cvSolve(T, D, X, cv::DECOMP_SVD);

				const float* ptr = (const float*)(X->data.ptr);
				fA = *ptr;
				ptr = (const float*)(X->data.ptr + X->step);
				fB = *ptr;

				//�����ڴ˴��洢������ϵ������Ϊ������ɾ�ֵ���׼֡�ǻ������ص������ģ��ֲ������ͷš�
				delta1 = fA;
				delta2 = fB;

				delete[] pT;
				delete[] pD;

				nFrameCount++;                           //+1, ��������
			}

			//ȡnNumCountThresh֡���ݣ����Ծ�����¶����ֵ
			if (nFrameCount == nFrameThresh + 1)
			{
				int nNumCountThresh = 10;

				if (nCalibCount < nNumCountThresh)
				{
					for (size_t i = 0; i < nFrameWidth * nFrameHeight; i++)
					{
						standardCalib[i] += depth[i];											
					}
					nStandardTemperature += CurrentTemperature;
					nCalibCount++;
				}
				else
				{
					for (size_t i = 0; i < nFrameWidth * nFrameHeight; i++)
					{
						fAveragePixelDistance += standardCalib[i];
						standardCalib[i] /= (float)nNumCountThresh;
					}
					nStandardTemperature /= nNumCountThresh;
					fAveragePixelDistance /= (float)(nNumCountThresh * nFrameWidth * nFrameHeight);

					nFrameCount++;                   //+2, ��������
				}

			}

			//����������׼�¶�nStandardTemperature������¶��µı�׼֡standardCalib[320 * 240]��������
			if (nFrameCount == nFrameThresh + 2)
			{
				for (size_t i = 0; i < nFrameWidth * nFrameHeight; i++)
				{
					calibration[i] = standardCalib[i] - fStandardDistance;
				}
				nFrameCount++;                      //+3, ��������
			}

			//д���ļ�
			if (nFrameCount == nFrameThresh + 3)
			{
				uint16_t valid = 1;
				uint16_t version = 0;
				FILE *fileout = fopen(calibfilename, "wb");
				fwrite(&valid, sizeof(uint16_t), 1, fileout);
				fwrite(&version, sizeof(uint16_t), 1, fileout);
				fwrite(&nStandardTemperature, sizeof(uint16_t), 1, fileout);
				fwrite(&nFrameWidth, sizeof(uint16_t), 1, fileout);
				fwrite(&nFrameHeight, sizeof(uint16_t), 1, fileout);
				fwrite(&fAveragePixelDistance, sizeof(float), 1, fileout);
				fwrite(&delta1, sizeof(float), 1, fileout);
				fwrite(&delta2, sizeof(float), 1, fileout);
				fwrite(calibration, sizeof(float), nFrameWidth * nFrameHeight, fileout);
				fclose(fileout);				

				nFrameCount = 0;
				nCalibCount = 0;

				state = TOF_DONE;                              //״̬����־��Ϊ�궨����
				nFlag_RaworCorrect = 1;                        //�궨��ɣ���־λ��1
			}
			
		}

	//}
	else 
	{
		if (nFlag_RaworCorrect == 0)                           //û����ɱ궨��û�б궨����ʾԭʼ���ݡ�
		{
			if (nFlag == 1)
			{
				//depth[n] -= calibration[n];
				std::ifstream fin;
				fin.open("calibration.dat", std::ios::in | std::ios::binary);
				if (fin.is_open())
				{
					//opened
					union UintChange {
						char suint16[2];
						unsigned short nInt;
					};
					UintChange a;
					union FloatChange {
						char sfloat[4];
						float fFloat;
					};
					FloatChange b;

					int nW = 0, nH = 0;
					float fAverageCalib = 0.0;
					fin.read(a.suint16, 2);
					//std::cout << (unsigned short)a.nInt << std::endl;
					if (a.nInt == 1)
					{
						fin.read(a.suint16, 2);
						//std::cout << (unsigned short)a.nInt << std::endl;
						fin.read(a.suint16, 2);
						nStandardTemperature = (unsigned short)a.nInt;
						//std::cout << (unsigned short)a.nInt << std::endl;
						//std::cout << nStandardTemperature << std::endl;
						fin.read(a.suint16, 2);
						nW = (unsigned short)a.nInt;
						//std::cout << (unsigned short)a.nInt << std::endl;
						//std::cout << nW << std::endl;
						fin.read(a.suint16, 2);
						nH = (unsigned short)a.nInt;
						//std::cout << (unsigned short)a.nInt << std::endl;
						//std::cout << nH << std::endl;

						fin.read(b.sfloat, 4);
						fAverageCalib = (float)b.fFloat;
						fin.read(b.sfloat, 4);
						delta1 = (float)b.fFloat;
						//std::cout << (float)b.fFloat << std::endl;
						//std::cout << delta1 << std::endl;
						fin.read(b.sfloat, 4);
						delta2 = (float)b.fFloat;
						//std::cout << (float)b.fFloat << std::endl;
						//std::cout << delta2 << std::endl;

						float afCalib[320 * 120] = { 0 };
						for (size_t i = 0; i < nW * nH; i++)
						{
							fin.read(b.sfloat, 4);
							float fMiners = (float)b.fFloat;
							//std::cout << fMiners << std::endl;
							afCalib[i] = fMiners;
						}

						float fAverageNow = 0.0;
						for (size_t i = 0; i < nW * nH; i++)
						{
							fAverageNow += depth[i];
						}
						fAverageNow /= (float)(nW * nH);

						float fWenpiaoCorrect = fAverageNow - fAverageCalib;
						for (size_t i = 0; i < nW * nH; i++)
						{
							depth[i] -= fWenpiaoCorrect;
							depth[i] -= afCalib[i];
						}

						fin.close();
					}
					else
					{
						//float fTotal = 0;
						//for (int n = 0; n < width*height; n++)
						//{
						//	fTotal += depth[n];                                       //�¶�Խ�ߣ�����Խ�࣬�ó�ʼ�׶����Զȸ���
						//}
						//std::cout << fTotal / (float)(width*height) << std::endl;

						fin.close();
					}
				}
				else
				{
					//not opened
					//float fTotal = 0;
					//for (int n = 0; n < width*height; n++)
					//{
					//	fTotal += depth[n];                                       //�¶�Խ�ߣ�����Խ�࣬�ó�ʼ�׶����Զȸ���
					//}
					//std::cout << fTotal / (float)(width*height) << std::endl;
				}
			}
			
		}
		else                                                   //��ɱ궨����ʾ�������ݡ�
		{
			float fAverageNow = 0.0;
			for (size_t i = 0; i < width*height; i++)
			{
				fAverageNow += depth[i];
			}
			fAverageNow /= (float)(width*height);

			float fWenpiaoCorrect = fAverageNow - fAveragePixelDistance;
			for (size_t i = 0; i < width*height; i++)
			{
				depth[i] -= fWenpiaoCorrect;
				depth[i] -= calibration[i];
			}
		}
		

	}



	/*printf("%s: %d\n", __FILE__, __LINE__);
	if (tof_handler) tof_handler(&cloud);
	printf("%s: %d\n", __FILE__, __LINE__);*/

	if (tof_handler) tof_handler(&cloud);

	//sLock.unlock();

	return;
}

//void ThreadDisplay()
//{
//	while (1)
//	{
//		if (tof_handler) tof_handler(&cloud);
//	}
//}

#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <Strmif.h>
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")

template <class T> void SafeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

typedef void(*frame_cb)(const tof_cloud_t *);

static frame_cb raw_handler = NULL;

HRESULT EnumerateCaptureFormats(IMFMediaSource *pSource)
{
	IMFPresentationDescriptor *pPD = NULL;
	IMFStreamDescriptor *pSD = NULL;
	IMFMediaTypeHandler *pHandler = NULL;
	IMFMediaType *pType = NULL;

	HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
	if (FAILED(hr))
	{
		goto done;
	}

	BOOL fSelected;
	hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
	if (FAILED(hr))
	{
		goto done;
	}

	hr = pSD->GetMediaTypeHandler(&pHandler);
	if (FAILED(hr))
	{
		goto done;
	}

	DWORD cTypes = 0;
	hr = pHandler->GetMediaTypeCount(&cTypes);
	if (FAILED(hr))
	{
		goto done;
	}

	for (DWORD i = 0; i < cTypes; i++)
	{
		hr = pHandler->GetMediaTypeByIndex(i, &pType);
		if (FAILED(hr))
		{
			goto done;
		}

		OutputDebugString(L"\n");

		SafeRelease(&pType);
	}

done:
	SafeRelease(&pPD);
	SafeRelease(&pSD);
	SafeRelease(&pHandler);
	SafeRelease(&pType);
	return hr;
}
HRESULT SetDeviceFormat(IMFMediaSource *pSource, DWORD dwFormatIndex)
{
	IMFPresentationDescriptor *pPD = NULL;
	IMFStreamDescriptor *pSD = NULL;
	IMFMediaTypeHandler *pHandler = NULL;
	IMFMediaType *pType = NULL;

	HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
	if (FAILED(hr))
	{
		goto done;
	}

	BOOL fSelected;
	hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
	if (FAILED(hr))
	{
		goto done;
	}

	hr = pSD->GetMediaTypeHandler(&pHandler);
	if (FAILED(hr))
	{
		goto done;
	}

	hr = pHandler->GetMediaTypeByIndex(dwFormatIndex, &pType);
	if (FAILED(hr))
	{
		goto done;
	}

	hr = pHandler->SetCurrentMediaType(pType);

done:
	SafeRelease(&pPD);
	SafeRelease(&pSD);
	SafeRelease(&pHandler);
	SafeRelease(&pType);
	return hr;
}
IMFMediaSource *g_pSource = NULL;

DWORD WINAPI MyThreadFunction(LPVOID lpParam);
// Sample custom data structure for threads to use.
// This is passed by void pointer so it can be any data type
// that can be passed using a single void pointer (LPVOID).
typedef struct MyData {
	IMFSourceReader* val1;
} MYDATA, *PMYDATA;

MYDATA DataArray;
DWORD   dwThreadIdArray;
HANDLE  hThreadArray;

bool running = TRUE;
DWORD WINAPI MyThreadFunction(LPVOID lpParam)
{
	PMYDATA that = (PMYDATA)lpParam;
	IMFSourceReader *source_reader = (IMFSourceReader *)that->val1;

	IMFSample *pSample = NULL;
	HRESULT hr;
	while (running) {
		DWORD streamIndex, flags;
		LONGLONG llTimeStamp;
		hr = source_reader->ReadSample(
			MF_SOURCE_READER_ANY_STREAM,    // Stream index.
			0,                              // Flags.
			&streamIndex,                   // Receives the actual stream index. 
			&flags,                         // Receives status flags.
			&llTimeStamp,                   // Receives the time stamp.
			&pSample                        // Receives the sample or NULL.
		);
		if (flags & MF_SOURCE_READERF_ENDOFSTREAM) {
			fprintf(stdout, "\tEnd of stream\n");
			break;
		}
		if (pSample) {
			DWORD max, current;
			BYTE *data;
			IMFMediaBuffer* buffer;
			pSample->GetBufferByIndex(0, &buffer);
			buffer->Lock(&data, &max, &current);
			if (current == 307200) { // check valid frame
				uint16_t *frame = (uint16_t *)data;
				raw_cb(frame);
			}

			buffer->Unlock();
			SafeRelease(&buffer);
		}

		SafeRelease(&pSample);
	}
	SafeRelease(&pSample);
	return 0;
}
static tof_calib_t tof_get_calib() 
{
  return calib;
}

IMFSourceReader* source_reader = 0;

int tof_open() 
{
	HRESULT hr;
	hr = ::CoInitialize(NULL);
	if (FAILED(hr))
	{
		//abort();
		return -1;
	}
	hr = ::MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
	if (FAILED(hr))
	{
		//abort();
		return -1;
	}
	IMFMediaSource*  media_source = 0;
	
	IMFAttributes* pAttributes = 0;
	hr = MFCreateAttributes(&pAttributes, 1);
	if (FAILED(hr))
	{
		//abort();
		return -1;
	}
	if (SUCCEEDED(hr))
	{
		hr = pAttributes->SetGUID(
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
		);
	}

	UINT32 count;
	IMFActivate **ppDevices = NULL;

	IMFActivate *pActiveDevice = NULL;
	hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
	if (FAILED(hr))
	{
		//abort();
		return -1;
	}


	if (count == 0)
	{
		//abort();
		return -1;
	}
	for (int i = 0; i < count; i++) {
		WCHAR *szFriendlyName = NULL;

		hr = ppDevices[i]->GetAllocatedString(
			MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
			&szFriendlyName,
			NULL
		);

		if (FAILED(hr))
		{
			break;
		}
		if (true) {//wcscmp(szFriendlyName, L"geefish") == 0) {
			pActiveDevice = ppDevices[i];
			break;
		}
	}
	// Create the media source object.
	IMFMediaSource *pSource = NULL;
	hr = pActiveDevice->ActivateObject(IID_PPV_ARGS(&pSource));
	if (FAILED(hr))
	{
		//abort();
		return -1;
	}
	pSource->AddRef();
	g_pSource = pSource;
	EnumerateCaptureFormats(pSource);
	SetDeviceFormat(pSource, 0);
	IAMVideoProcAmp *pProcAmp = NULL;
	hr = pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
	if (FAILED(hr))
	{
		return -1;
	}
	if (SUCCEEDED(hr))
	{
		long Min, Max, Step, Default, Flags, Val = 0;
		hr = pProcAmp->Set(VideoProcAmp_BacklightCompensation, 1000, VideoProcAmp_Flags_Auto);
		hr = pProcAmp->Get(VideoProcAmp_BacklightCompensation, &Val, &Flags);
		Val = 0;

		//����***************************************************
		//hr = pProcAmp->Set(VideoProcAmp_Brightness, 100, VideoProcAmp_Flags_Auto);
		//if (!SUCCEEDED(hr))
		//{
		//	printf("This is a test");
		//}
		//hr = pProcAmp->Get(VideoProcAmp_Brightness, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Contrast, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Contrast, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Hue, 0, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Hue, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Saturation, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Saturation, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Sharpness, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Sharpness, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Gamma, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Gamma, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_ColorEnable, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_ColorEnable, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_WhiteBalance, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_WhiteBalance, &Val, &Flags);
		//Val = 0;

		//hr = pProcAmp->Set(VideoProcAmp_Gain, 1000, VideoProcAmp_Flags_Auto);
		//hr = pProcAmp->Get(VideoProcAmp_Gain, &Val, &Flags);
		//Val = 0;

		//*******************************************************
	}
	hr = MFCreateSourceReaderFromMediaSource(pSource, NULL, &source_reader);
	//IMFSample *pSample = NULL;
	DataArray.val1 = source_reader;
	running = TRUE;
	hThreadArray = CreateThread(
		NULL,                   // default security attributes
		0,                      // use default stack size  
		MyThreadFunction,       // thread function name
		&DataArray,          // argument to thread function 
		0,                      // use default creation flags 
		&dwThreadIdArray);   // returns the thread identifier 

  //state = TOF_IDLE;
  //uvc_stream_ctrl_t ctrl;

  //uvc_error_t res = uvc_init(&ctx, NULL);

  //if (res < 0) {
  //  //uvc_perror(res, "uvc_init");
  //  printf("not initialized\n");
  //  return res;
  //}

  ////puts("UVC initialized");
  //printf("UVC initialized\n");

  //res = uvc_find_device(ctx, &dev, VID, PID, NULL);

  //if (res < 0) {
  //  //uvc_perror(res, "uvc_find_device");
  //  printf("not found\n");
  //  return res;
  //} else {
  //  //puts("Device found");
  //  printf("Device found\n");

  //  res = uvc_open(dev, &devh);
  //}

  //res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_YUYV, WIDTH, HEIGHT, 0);
  //if (res < 0)
  //{
  //  printf("uvc_get_stream_ctrl_format_size error\n");
  //  return res;
  //}

  //    res = uvc_get_stream_ctrl_format_size(
  //        devh, &ctrl, /* result stored in ctrl */
  //        UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
  //        640, 240, 0 /* width, height, fps */
  //        //640, 360, 0 /* width, height, fps */
  //    );
  //tof_get_calib();

  ////TODO:call uvc_get_stream_ctrl_format_size to get ctrl
 
  //res = uvc_start_streaming(devh, &ctrl, raw_cb, (void *)0, 0);

  //if (res < 0) {
  //  //uvc_perror(res, "start_streaming");
  //  printf("start_streaming error\n");
  //} else {
  //  //puts("Streaming...");
  //  printf("Streaming\n");
  //}

  //// uvc_frame_t * a = 0;
  //// int b = 0;
  //// raw_cb(a, &b);

	/*std::thread displaythread(ThreadDisplay);
	displaythread.detach();*/

  return 0;
}

int tof_close() 
{
	if (running == FALSE)
	{
		return 0;
	}

	running = FALSE;
	Sleep(2);
	g_pSource->Release();
	source_reader->Release();
	source_reader = 0;

	MFShutdown();
	::CoUninitialize();
	return 0;
  //printf("tof_close in\n");
  //uvc_stop_streaming(devh);
  //uvc_close(devh);
  ////puts("Device closed");
  //printf("Device closed\n");

  //uvc_unref_device(dev);

  //uvc_exit(ctx);
  //printf("Device closed end\n");
  //return 0;
}

int tof_set_cb(frame_cb handler) {
  tof_handler = handler;
  return 0;
}

int tof_set_expose(uint16_t value) {
	long Val = value;
	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr;
	hr = g_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
	if (SUCCEEDED(hr))
	{
		hr = pProcAmp->Set(VideoProcAmp_BacklightCompensation, Val, VideoProcAmp_Flags_Auto);
	}   
  //uvc_error_t res = uvc_set_backlight_compensation(devh, value);
  return 0;
}

int tof_get_expose(uint16_t &value) 
{
	IAMVideoProcAmp *pProcAmp = NULL;
	long Val = value;
	HRESULT hr;
	hr = g_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
	if (SUCCEEDED(hr))
	{
		long Min, Max, Step, Default, Flags, Val = 0;
		hr = pProcAmp->Get(VideoProcAmp_BacklightCompensation, &Val, &Flags);
		value = Val;
	}
	//uvc_error_t res = uvc_set_backlight_compensation(devh, value);
   //uvc_error_t res = uvc_get_backlight_compensation(devh, &value, UVC_GET_CUR);
  return 0;
}

int tof_save_calib(const char *name) {

  return 0;
}

int tof_calibrate(float ref, const char *name) {
  fStandardDistance = ref;
  calibfilename = (char*)name;

  //��ʼ�궨����ʼ����ر������ڴ�����
  memset(calibration, 0, 320 * 240 * sizeof(float));
  memset(standardCalib, 0, 320 * 240 * sizeof(float));
  nStandardTemperature = 0;
  nFlag_RaworCorrect = 0;
  delta1 = 0;
  delta2 = 0;
  nFirstCount = 0;

  nFlag_RaworCorrect = 0;                  //�ڱ궨��ʱ��ѡ����ʾԭʼ����

  calib.valid = 0;
  state = TOF_CALIBRATING;
  while (state != TOF_DONE) {
    Sleep(1);
  }
  return 0;
}

const char *tof_version() {
  return "not implemented\n";
}

int tof_video_to_mass_storage()
{
	long Val = 1324;
	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr;
	long Flags;
	hr = g_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
	if (SUCCEEDED(hr))
	{
		hr = pProcAmp->Set(VideoProcAmp_BacklightCompensation, 0x1FFF, VideoProcAmp_Flags_Auto);
// 		if (!SUCCEEDED(hr))
// 		{
// 			return -1;
// 		}
	}

	return 0;
}

