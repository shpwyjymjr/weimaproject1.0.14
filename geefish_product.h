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

//帧数据返回
typedef struct geefish_frame_data_{
	int   width;
	int   height;
	float *image_buff = 0;	    //图像缓冲区
	int   buff_size;			//缓冲区大小

	float mean;					//均值
	float std;					//标准差
	float temperature;			//温度
} geefish_frame_data;

typedef enum camera_mode_ {
	DISTENCE	= 1,
	AMPLITUDE	= 2,
	GRAYSCALE	= 3,
	POINTCLOUD	= 4
} camera_mode;

/*
* 函数名称:	frame_cb
* 函数功能:	回调函数
* 参    数:
*			device_name	- 设备路径
* 返 回 值:
*/
typedef void (*geefish_frame_cb)(geefish_frame_data* frame_data);

#ifdef __cplusplus
extern "C" {
#endif

/*
* 函数名称:	GeeFishProduct_Start
* 函数功能:	打开设备
* 参    数:
*			device_name	- 设备路径
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetFrameCallBack(geefish_frame_cb, geefish_frame_data* pData);

/*
* 函数名称:	GeeFishProduct_SetMode
* 函数功能:	设置相机模式
* 参    数:
*			mode - 模式
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetMode(camera_mode mode);

/*
* 函数名称:	GeeFishProduct_Start
* 函数功能:	打开设备
* 参    数:
*			device_name	- 设备路径
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Start();

/*
* 函数名称:	GeeFishProduct_Stop
* 函数功能:	关闭设备
* 参    数:
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Stop();

/*
* 函数名称:	GeeFishProduct_Calibrate
* 函数功能:	设备标定
* 参    数:
*			file_name   - 标定后生成的文件名
*			distence	- 设备标定时的距离，单位m
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Calibrate(float distence = 667);

/*
* 函数名称:	GeeFishProduct_Switch
* 函数功能:	从相机模式切换到U盘模式，并返回串口列表
* 参    数:
*			port_buff   - 返回串口列表的缓冲区，如果为NULL，则将需要的大小返给size
*			size		- 传入时为缓冲区大小，传出为实际返回的大小，如果buff是NULL，则直接返回所需大小
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Switch(wchar_t* port_buff, unsigned int* size);

/*
* 函数名称:	GeeFishProduct_BurnFile
* 函数功能:	烧写标定文件
* 参    数:
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_BurnFile(wchar_t* port);

/*
* 函数名称:	GeeFishProduct_GetDistance
* 函数功能:	获取灰度图距离
* 参    数:
*			point_value   - 像素x、y的坐标
*			distance	  - 像素对应的距离
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetDistance(point point_value, float* distance);

/*
* 函数名称:	GeeFishProduct_GetConfidence
* 函数功能:	获取像素置信度
* 参    数:
*			point_value   - 像素x、y的坐标
*			confidence	  - 像素置信度
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetConfidence(point point_value, unsigned int* confidence);

/*
* 函数名称:	GeeFishProduct_SetExposureTime
* 函数功能:	设置曝光时间
* 参    数:
*			nExposuretime - 用于设置曝光时间的值
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetExposureTime(int nExposuretime);

/*
* 函数名称:	GeeFishProduct_GetExposureTime
* 函数功能:	获取曝光时间
* 参    数:
*			nExposuretime - 用于存储获取到的曝光时间的值
* 返 回 值:
*/
geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetExposureTime(int& nExposuretime);


#ifdef __cplusplus
}
#endif