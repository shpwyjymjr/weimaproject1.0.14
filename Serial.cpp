 #include"Serial.h"  
#include<process.h> 
#include <list>
#include <atlconv.h>
#include <tchar.h>

using namespace std;

typedef unsigned(__stdcall *PTHREEA_START) (void *);

extern BOOL g_bCopyFileIsRead = FALSE;

CSerial::CSerial(void)
{
	m_hComm = INVALID_HANDLE_VALUE;
}

CSerial::~CSerial(void)
{
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
		//_endthreadex(0);
	}
}

/*********************************************************************************************
* ����    ��    �������̻߳ص�����
* ����       �� �յ����ݺ󣬼򵥵���ʾ����
********************************************************************************************/
DWORD WINAPI CommProc(LPVOID lpParam) {

	CSerial* pSerial = (CSerial*)lpParam;

	//��մ���  
	PurgeComm(pSerial->m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	char buf[512];
	DWORD dwRead;
	while (pSerial->m_hComm != INVALID_HANDLE_VALUE) 
	{
		BOOL bReadOK = ReadFile(pSerial->m_hComm, buf, 512, &dwRead, NULL);
		if (bReadOK && (dwRead > 0)) 
		{
			buf[dwRead] = '\0';

			char cmd_hand_shake_request[8] = { 0 };

			memcpy(cmd_hand_shake_request, "\xAA\xFF\x01\x00\x00\x00\x00\x55", 8);
			//int cmpsize = strlen("test");
			if (memcmp(buf, cmd_hand_shake_request, 8) == 0)
			{
				g_bCopyFileIsRead = TRUE;
			}
			
		}
	}
	return 0;
}

/*******************************************************************************************
* ����			��  ö�ٴ���
* port_buff     :   ���ڻ�������ÿ����������֮����'\0'����
* buff_size		:   ��port_buffΪNULLʱ������buff�Ĵ�С��port_buff��ΪNULL������Ϊbuff��С������Ϊʵ�ʵ�buff��С
* port_number	:   ���ڸ���
********************************************************************************************/
BOOL CSerial::EnumSerialPort(TCHAR* port_buff, UINT* size, UINT* port_number)
{
	HKEY hKey;
	list<string> strlstPort;

	if (port_number == NULL)
	{
		return FALSE;
	}

	if (ERROR_SUCCESS == ::RegOpenKeyEx(HKEY_LOCAL_MACHINE, _T("Hardware\\DeviceMap\\SerialComm"), NULL, KEY_READ, &hKey))//�򿪴���ע����Ӧ�ļ�ֵ   
	{
		int count = 0;
		TCHAR RegKeyName[128],  SerialPortName[128];
		TCHAR AllSerialPortName[256] = {0};
		DWORD dwLong, dwSize;
		int offset = 0;

		while (TRUE)
		{	
			dwLong = dwSize = sizeof(RegKeyName);
			if (ERROR_NO_MORE_ITEMS == ::RegEnumValue(hKey, count, RegKeyName, &dwLong, NULL, NULL, (PUCHAR)SerialPortName, &dwSize))//ö�ٴ���   
			{
				break;
			}

			//�������⴮��
			if (!memcmp(RegKeyName, _T("\\Device\\"), 8))  
			{
				//const size_t cSize = strlen((char*)SerialPortName) + 1;
				//TCHAR* wSerialPortName = new wchar_t[cSize];
				//mbstowcs(wSerialPortName, (char*)SerialPortName, cSize);
				int len = wcslen(SerialPortName);
				int wcharsize = sizeof(wchar_t);
				memcpy(AllSerialPortName + offset, SerialPortName, wcslen(SerialPortName)*sizeof(wchar_t));

				offset += wcslen(SerialPortName) + 1;
				//��ȡ��������SerialPortName���Ǵ�������
				//string strSerialPortName = SerialPortName;
				//strlstPort.push_back(strSerialPortName);
			}
			count++;
		}

		if (port_buff == NULL)
		{
			*size = offset;
		}
		else
		{
			if (*size < offset)
			{
				return FALSE;
			}

			int cpsize = offset * sizeof(wchar_t);
			memcpy(port_buff, AllSerialPortName, cpsize);
			*port_number = count;
			*size = cpsize;
		}

		RegCloseKey(hKey);
	}
	else
	{
		RegCloseKey(hKey);
		return FALSE;
	}
	return TRUE;
}

/*******************************************************************************************
* ����     ��  �򿪴���
* port     :   ���ں�, ��_T("COM1:")
* baud_rate:   ������
* date_bits:  ����λ����Ч��Χ4~8��
* stop_bit :    ֹͣλ
* parity   : ��żУ�顣Ĭ��Ϊ��У�顣NOPARITY 0�� ODDPARITY 1��EVENPARITY 2��MARKPARITY 3��SPACEPARITY 4
********************************************************************************************/
BOOL CSerial::OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity)
{
	//�򿪴���  
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);//��ռ��ʽ�򿪴���  

	TCHAR err[512];

	if (m_hComm == INVALID_HANDLE_VALUE) {
		//_stprintf(err, _T("�򿪴���%s ʧ�ܣ���鿴�ô����Ƿ��ѱ�ռ��"), port);
		//MessageBox(NULL, err, _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//MessageBox(NULL,_T("�򿪳ɹ�"),_T("��ʾ"),MB_OK);  

	//��ȡ����Ĭ������  
	DCB dcb;
	if (!GetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("��ȡ���ڵ�ǰ���Բ���ʧ��"), _T("��ʾ"), MB_OK);
	}

	//���ô��ڲ���  
	dcb.BaudRate = baud_rate;  //������  
	dcb.fBinary = TRUE;            //������ģʽ������ΪTRUE  
	dcb.ByteSize = date_bits;  //����λ����Χ4-8  
	dcb.StopBits = ONESTOPBIT; //ֹͣλ  

	if (parity == NOPARITY) {
		dcb.fParity = FALSE;   //��żУ�顣����żУ��  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}
	else {
		dcb.fParity = TRUE;        //��żУ�顣  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}

	dcb.fOutxCtsFlow = FALSE;  //CTS���ϵ�Ӳ������  
	dcb.fOutxDsrFlow = FALSE;  //DST���ϵ�Ӳ������  
	dcb.fDtrControl = DTR_CONTROL_ENABLE;//DTR����  
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;//  
	dcb.fOutX = FALSE;         //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fInX = FALSE;          //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fErrorChar = FALSE;        //�Ƿ�ʹ�÷��ʹ���Э��  
	dcb.fNull = FALSE;         //ͣ��null stripping  
	dcb.fRtsControl = RTS_CONTROL_ENABLE;//  
	dcb.fAbortOnError = FALSE; //���ڷ��ʹ��󣬲�����ֹ���ڶ�д  

							   //���ô��ڲ���  
	if (!SetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("���ô��ڲ���ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//���ô����¼�  
	SetCommMask(m_hComm, EV_RXCHAR);//�ڻ��������ַ�ʱ�����¼�  
	SetupComm(m_hComm, 16384, 16384);

	//���ô��ڶ�дʱ��  
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts(m_hComm, &CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 10;
	CommTimeOuts.WriteTotalTimeoutConstant = 1000;

	if (!SetCommTimeouts(m_hComm, &CommTimeOuts)) {
		MessageBox(NULL, _T("���ô���ʱ��ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//�����̣߳���ȡ����  
	m_hReadCommThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREEA_START)CommProc, (LPVOID) this, 0, NULL);
	return TRUE;
}

/********************************************************************************************
* ����    ��    ͨ�����ڷ���һ������
********************************************************************************************/
BOOL CSerial::SendData(char* data, int len) {
	if (m_hComm == INVALID_HANDLE_VALUE) {
		MessageBox(NULL, _T("����δ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//д����  
	DWORD dwWrite = 0;
	DWORD dwRet = WriteFile(m_hComm, data, len, &dwWrite, NULL);

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	if (!dwRet) {
		//MessageBox(NULL, _T("��������ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}
	return TRUE;
}