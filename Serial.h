#pragma once  

#include <windows.h>  

class CSerial
{
public:
	CSerial(void);
	~CSerial(void);

	//枚举串口
	BOOL EnumSerialPort(TCHAR* port_buff, UINT* buff_size, UINT* port_number);

	//打开串口  
	BOOL OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity = NOPARITY);

	//发送数据  
	BOOL SendData(char* data, int len);
public:
	HANDLE m_hComm;
	HANDLE m_hReadCommThread;
};
