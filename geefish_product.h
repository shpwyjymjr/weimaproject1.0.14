#pragma once

#ifdef _WIN32
#define GF_PRODUCT_FUNC __stdcall
#else
#define GF_PRODUCT_FUNC
#endif // _WIN32

typedef enum geefish_error {
  /** Success (no error) */
  GEEFISH_SUCCESS = 0,
  /** Input/output error */
  GEEFISH_ERROR_IO = -1,
  /** Invalid parameter */
  GEEFISH_ERROR_INVALID_PARAM = -2,
  /** Access denied */
  GEEFISH_ERROR_ACCESS = -3,
  /** No such device */
  GEEFISH_ERROR_NO_DEVICE = -4,
  /** Undefined error */
  GEEFISH_ERROR_OTHER = -99
} geefish_error_t;

typedef struct point_
{
	int x;
	int y; 
} point;

//֡���ݷ���
typedef struct geefish_frame_data_{
	int   width;
	int   height;
	float *image_buff = 0;	    //ͼ�񻺳���
	int   buff_size;			//��������С

	float mean;					//��ֵ
	float std;					//��׼��
	float temperature;			//�¶�
} geefish_frame_data;

typedef enum camera_mode_ {
	DISTENCE	= 1,
	AMPLITUDE	= 2,
	GRAYSCALE	= 3,
	POINTCLOUD	= 4
} camera_mode;

/*
* ��������:	frame_cb
* ��������:	�ص�����
* ��    ��:
*			device_name	- �豸·��
* �� �� ֵ:
*/
typedef void (*geefish_frame_cb)(geefish_frame_data* frame_data);

#ifdef __cplusplus
extern "C" {
#endif

/*
* ��������:	GeeFishProduct_Start
* ��������:	���豸
* ��    ��:
*			device_name	- �豸·��
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetFrameCallBack(geefish_frame_cb, geefish_frame_data* pData);

/*
* ��������:	GeeFishProduct_SetMode
* ��������:	�������ģʽ
* ��    ��:
*			mode - ģʽ
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetMode(camera_mode mode);

/*
* ��������:	GeeFishProduct_Start
* ��������:	���豸
* ��    ��:
*			device_name	- �豸·��
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Start();

/*
* ��������:	GeeFishProduct_Stop
* ��������:	�ر��豸
* ��    ��:
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Stop();

/*
* ��������:	GeeFishProduct_Calibrate
* ��������:	�豸�궨
* ��    ��:
*			file_name   - �궨�����ɵ��ļ���
*			distence	- �豸�궨ʱ�ľ��룬��λm
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Calibrate(float distence = 667);

/*
* ��������:	GeeFishProduct_Switch
* ��������:	�����ģʽ�л���U��ģʽ�������ش����б�
* ��    ��:
*			port_buff   - ���ش����б�Ļ����������ΪNULL������Ҫ�Ĵ�С����size
*			size		- ����ʱΪ��������С������Ϊʵ�ʷ��صĴ�С�����buff��NULL����ֱ�ӷ��������С
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Switch(wchar_t* port_buff, unsigned int* size);

/*
* ��������:	GeeFishProduct_BurnFile
* ��������:	��д�궨�ļ�
* ��    ��:
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_BurnFile(wchar_t* port);

/*
* ��������:	GeeFishProduct_GetDistance
* ��������:	��ȡ�Ҷ�ͼ����
* ��    ��:
*			point_value   - ����x��y������
*			distance	  - ���ض�Ӧ�ľ���
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetDistance(point point_value, float* distance);

/*
* ��������:	GeeFishProduct_GetConfidence
* ��������:	��ȡ�������Ŷ�
* ��    ��:
*			point_value   - ����x��y������
*			confidence	  - �������Ŷ�
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetConfidence(point point_value, unsigned int* confidence);

/*
* ��������:	GeeFishProduct_SetExposureTime
* ��������:	�����ع�ʱ��
* ��    ��:
*			nExposuretime - ���������ع�ʱ���ֵ
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetExposureTime(int nExposuretime);

/*
* ��������:	GeeFishProduct_GetExposureTime
* ��������:	��ȡ�ع�ʱ��
* ��    ��:
*			nExposuretime - ���ڴ洢��ȡ�����ع�ʱ���ֵ
* �� �� ֵ:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetExposureTime(int& nExposuretime);


#ifdef __cplusplus
}
#endif