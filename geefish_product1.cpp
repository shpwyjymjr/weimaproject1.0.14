#include "geefish_product.h"
#include "geefish_tof.h"
#include <iostream>
#include <Windows.h>
#include "Serial.h"

#define CALIBRATION_FILE_NAME "calibration.dat"
#define BAUD_RATE 9600
#define CMD_LEN   8

extern BOOL g_bCopyFileIsRead;
extern float fStandardDistance;

geefish_frame_data* geefish_gData;
geefish_frame_cb geefish_gHanle;
const tof_cloud_t * pCloud_point_to_get;
float pPointToGet[320 * 240] = { 0 };
float pAmplitudeToGet[320 * 240] = { 0 };
int nFlag = 0;

int enum_usb_disk(char usb_paths[], int cnt)
{
	int usb_disk_cnt = 0;

	char disk_path[5] = { 0 };
	char device_path[10] = { 0 };
	DWORD all_disk = GetLogicalDrives();
	printf("System volume flag:0x%X\n", all_disk);

	int i = 0;
	DWORD bytes_returned = 0;
	while (all_disk && usb_disk_cnt < cnt)
	{
		if ((all_disk & 0x1) == 1)
		{
			sprintf_s(disk_path, "%c:", 'A' + i);
			sprintf_s(device_path, "\\\\.\\%s", disk_path);
			if (GetDriveTypeA(disk_path) == DRIVE_REMOVABLE)
			{
				usb_paths[usb_disk_cnt++] = 'A' + i;
			}
		}
		all_disk = all_disk >> 1;
		i++;
	}

	return usb_disk_cnt;
}

static void my_frame_cb(const tof_cloud_t * pCloud_pipe){
	pCloud_point_to_get = pCloud_pipe;

	//pPointToGet = (float*)malloc(pCloud_pipe->width * pCloud_pipe->height * sizeof(float));
	memcpy(pPointToGet, pCloud_pipe->distance, pCloud_pipe->width * pCloud_pipe->height * sizeof(float));

	//pAmplitudeToGet = (float*)malloc(pCloud_pipe->width * pCloud_pipe->height * sizeof(float));
	memcpy(pAmplitudeToGet, pCloud_pipe->amplitude, pCloud_pipe->width * pCloud_pipe->height * sizeof(float));

	
	//printf("%s: %d\n", __FILE__, __LINE__);
	//std::cout << "in algori:" << pCloud_pipe->width << std::endl;
	geefish_gData->width = pCloud_pipe->width;
	geefish_gData->height = pCloud_pipe->height;
	geefish_gData->temperature = pCloud_pipe->temperature;
	float distance_mean_value = 0;
	float distance_sum = 0;
	float deviation = 0;
	int image_size = pCloud_pipe->width * pCloud_pipe->height;
	
	/*if(pCloud_pipe->width * pCloud_pipe->height > geefish_gData->buff_size)
		return;*/
	
	if(nFlag == DISTENCE)
	{
		
		float *diff = (float*)malloc(pCloud_pipe->width * pCloud_pipe->height * sizeof(float));
		memset(diff, 0, pCloud_pipe->width * pCloud_pipe->height * sizeof(float));
		float diff_sum = 0;
		for (int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			distance_sum += pCloud_pipe->distance[i];
			float a = pCloud_pipe->distance[i] - fStandardDistance;
			*(diff + i) = a;
			diff_sum += pCloud_pipe->distance[i] - fStandardDistance;
			//deviation += (pCloud_pipe->distance[i] - fStandardDistance) * (pCloud_pipe->distance[i] - fStandardDistance);
		}

		float diff_mean = diff_sum / image_size;
		for (int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			deviation += (*(diff + i) - diff_mean) * (*(diff + i) - diff_mean);
		}

		float fDisMax = 6248;
		/*for(int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			if(pCloud_pipe->distance[i] > fDisMax)
				fDisMax = pCloud_pipe->distance[i];
		}*/
		
		for(int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			float fDistance = pCloud_pipe->distance[i] / fDisMax;
			if (fDistance > 1)
				fDistance = 1.0;
			fDistance = 1.0 - fDistance;
			pCloud_pipe->distance[i] = fDistance;
		}
		//std::cout << "最大距离为" << fDisMax << std::endl;
		
		geefish_gData->mean = distance_sum / image_size;
		deviation = sqrt(deviation / image_size);
		geefish_gData->std = deviation;
		geefish_gData->image_buff = pCloud_pipe->distance;

		free(diff);
	}
	else if(nFlag == AMPLITUDE)
	{
		
		float fAmpMax = 0/*2895.6*/;
		for(int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			if(pCloud_pipe->amplitude[i] > fAmpMax)
				fAmpMax = pCloud_pipe->amplitude[i];
		}
		
		/*for(int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			float fAmplitude = pCloud_pipe->amplitude[i] / fAmpMax;
			if (fAmplitude > 1)
				fAmplitude = 1.0;
			fAmplitude = 1.0 - fAmplitude;
			pCloud_pipe->amplitude[i] /= fAmplitude;
		}*/
		//std::cout << "最大置信度为" << fAmpMax << std::endl;
		
		geefish_gData->image_buff = pCloud_pipe->amplitude;
	}
	else if(nFlag == GRAYSCALE)
	{
		float fGrayMax = 4000;
		/*for (int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			if (pCloud_pipe->gray[i] > fGrayMax)
				fGrayMax = pCloud_pipe->gray[i];
		}*/

		for (int i = 0; i < pCloud_pipe->width * pCloud_pipe->height; i++)
		{
			float fGray = pCloud_pipe->gray[i] / fGrayMax;
			if (fGray > 1)
				fGray = 1.0;
			fGray = 1.0 - fGray;
			pCloud_pipe->gray[i] = fGray;
		}
		//std::cout << "最大灰度值为" << fGrayMax << std::endl;

		geefish_gData->image_buff = pCloud_pipe->gray;
	}
	else if(nFlag == POINTCLOUD)
	{
		
	}
	
	//std::cout << "in algorithm cb" << std::endl;
	//printf("%s: %d\n", __FILE__, __LINE__);
	geefish_gHanle(geefish_gData);
	//printf("%s: %d\n", __FILE__, __LINE__);
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Start()
{
	//std::cout << "GeeFishProduct_Star in." << std::endl;
	tof_set_cb(my_frame_cb);
	//std::cout << "just go out of the tof_set_cb" << std::endl;
	int nResult = tof_open();
	if (nResult == -1)
		return GEEFISH_ERROR_NO_DEVICE;
	else
		return GEEFISH_SUCCESS;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Stop()
{
	printf("GeeFishProduct_Stop in\n");
	tof_close();
	return GEEFISH_SUCCESS;
}


geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetFrameCallBack(geefish_frame_cb geefish_Handle, geefish_frame_data* pData)
{
	if(geefish_Handle == NULL || pData == NULL)
		return GEEFISH_ERROR_INVALID_PARAM;
	else
	{
		geefish_gHanle = geefish_Handle;
		geefish_gData = pData;
	    return GEEFISH_SUCCESS;
	}
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Calibrate(float distance)
{
	int ret = tof_calibrate(distance, CALIBRATION_FILE_NAME);
	return (geefish_error_t)ret;	
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_Switch(wchar_t* port_buff, unsigned int* size)
{
	int ret = tof_video_to_mass_storage();
	if (ret != GEEFISH_SUCCESS)
	{
		return (geefish_error_t)ret;
	}

	Sleep(2000);

	CSerial serial;
	UINT port_number = 0;
	BOOL result = serial.EnumSerialPort(port_buff, size, &port_number);
	if (result != TRUE)
	{
		return GEEFISH_ERROR_NO_DEVICE;
	}

	return GEEFISH_SUCCESS;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetMode(camera_mode mode)
{
	if (mode == DISTENCE || mode == AMPLITUDE || mode == GRAYSCALE || mode == POINTCLOUD)
	{
		nFlag = mode;
		return GEEFISH_SUCCESS;
	}
	else
		return GEEFISH_ERROR_INVALID_PARAM;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetDistance(point point_value, float* distance)
{
	if(point_value.x < 0 || point_value.x > pCloud_point_to_get->width || point_value.y < 0 || point_value.y > pCloud_point_to_get->height || distance == NULL)
		return GEEFISH_ERROR_INVALID_PARAM;
	else
	{
		int nWidth = pCloud_point_to_get->width;
		int nHeight = pCloud_point_to_get->height;
		int nIndex = point_value.y * nWidth + point_value.x;
		*distance = pPointToGet[nIndex];
		return GEEFISH_SUCCESS;
	}
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_SetExposureTime(int nExposuretime)
{
	tof_set_expose(nExposuretime);
	return GEEFISH_SUCCESS;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetExposureTime(int& nExposuretime)
{
	uint16_t nTime = 0;
	tof_get_expose(nTime);
	nExposuretime = (int)nTime;
	return GEEFISH_SUCCESS;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_BurnFile(wchar_t* port)
{
	//发送握手命令
	CSerial serial;
	serial.OpenSerialPort(port, BAUD_RATE, CMD_LEN, 1);
	char cmd_hand_shake[CMD_LEN] = { 0 };

	memcpy(cmd_hand_shake, "\xAA\xFE\x02\x00\x00\x00\x00\x55", CMD_LEN);

	//if (serial.SendData(cmd_hand_shake, CMD_LEN) == FALSE)
	//{
	//	return GEEFISH_ERROR_NO_DEVICE;
	//}

	while (true)
	{
		serial.SendData(cmd_hand_shake, CMD_LEN);
		Sleep(3000);
		if (g_bCopyFileIsRead)
		{
			g_bCopyFileIsRead = FALSE;
			break;
		}
	}

	//将文件复制到模组
	char usb_volume[8] = { 0 };
	char destination_path[32] = {0};
	int usb_cnt = enum_usb_disk(usb_volume, 8);
	if (usb_cnt == 0)
	{
		return GEEFISH_ERROR_NO_DEVICE;
	}
	sprintf(destination_path, "%c:\\%s", usb_volume[0], CALIBRATION_FILE_NAME);

	WCHAR w_destination_path[256];
	memset(w_destination_path, 0, sizeof(w_destination_path));
	MultiByteToWideChar(CP_ACP, 0, destination_path, strlen(destination_path) + 1, w_destination_path, sizeof(w_destination_path) / sizeof(w_destination_path[0]));
	//CopyFile(w_destination_path, L"test.txt", FALSE);
	//BOOL ret = CopyFile(L"calibration.dat", L"D:\\test.dat", FALSE);
	BOOL ret = CopyFile(L"calibration.dat", w_destination_path, FALSE);
	if (ret == FALSE)
	{
		return GEEFISH_ERROR_IO;
	}

	//将U盘模式切换为相机模式
	char cmd_set_mode[CMD_LEN] = { 0 };
	char cmd_reset[CMD_LEN] = { 0 };

	memcpy(cmd_set_mode, "\xAA\xFF\x02\x20\x00\x00\x00\x55", CMD_LEN);
	memcpy(cmd_reset, "\xAA\xFF\x04\x20\x00\x00\x00\x55", CMD_LEN);

	serial.SendData(cmd_set_mode, CMD_LEN);
	serial.SendData(cmd_set_mode, CMD_LEN);
	serial.SendData(cmd_reset, CMD_LEN);

	return GEEFISH_SUCCESS;
}

geefish_error_t GF_PRODUCT_FUNC GeeFishProduct_GetConfidence(point point_value, unsigned int* confidence)
{
	if (point_value.x < 0 || point_value.x > pCloud_point_to_get->width || point_value.y < 0 || point_value.y > pCloud_point_to_get->height || confidence == NULL)
		return GEEFISH_ERROR_INVALID_PARAM;
	else
	{
		int nWidth = pCloud_point_to_get->width;
		int nHeight = pCloud_point_to_get->height;
		int nIndex = point_value.y * nWidth + point_value.x;
		*confidence = pAmplitudeToGet[nIndex];
		return GEEFISH_SUCCESS;
	}
	return GEEFISH_SUCCESS;
}

