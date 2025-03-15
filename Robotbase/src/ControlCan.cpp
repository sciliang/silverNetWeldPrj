#include <iostream>
#include <string>
#include <unistd.h>
#include "controlcan.h"
#include "IniParser.h"
#include "ControlCanFun.h"
#include "YinwMotion.h"
#include "IOdeviceControl.h"
#include <cmath>
#include <string.h>
#include <vector> // 添加vector头文件
#include <cstdint>
#include <chrono>
#include "HYYRobotInterface.h"
#include "device_interface.h"
#include <thread>
#include <array>
using namespace HYYRobotBase;

// Can初始化
bool init_can()
{
	VCI_BOARD_INFO pInfo, pInfo1[50];
	int count = 0, num = 0;
	printf(">>this is hello !\r\n");
	// 找到USBCAN设备
	num = VCI_FindUsbDevice2(pInfo1); // USBCAN的设备数量
	printf(">>USBCAN DEVICE NUM:");
	printf("%d PCS\n", num);
	int nDeviceType = 4;
	int nDeviceInd = 0;
	int nCANInd = 0;
	DWORD dwRel;
	VCI_INIT_CONFIG vic;
	// 打开USBCAN设备
	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
	if (dwRel != 1)
	{
		LOG(INFO) << "open-fail:" << dwRel << std::endl;
		return false;
	}
	else
	{
		LOG(INFO) << "open success:1";
	}
	vic.AccCode = 0x80000008;
	vic.AccMask = 0xFFFFFFFF;
	vic.Filter = 1;									   // 接收所有帧
	vic.Timing0 = 0x00; /*波特率125 Kbps  0x03  0x1C*/ // 用的时候是500kbs
	vic.Timing1 = 0x1C;								   // 1000kbs
	vic.Mode = 0;									   // 正常模式
	dwRel = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic);
	if (dwRel != 1)
	{
		LOG(INFO) << "init-fail:" << dwRel << std::endl;
		VCI_CloseDevice(nDeviceType, nDeviceInd);
		return false;
	}
	else
	{
		LOG(INFO) << "initsuccess:" << dwRel << std::endl;
	}
	// 启动CAN设备的0通道
	if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
	{
		LOG(INFO) << "start-fail:" << dwRel << std::endl;
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	else
	{
		LOG(INFO) << "startsuccess:1" << std::endl;
	}
	return 0;
}

// 将16进制数转成10进制
int32_t convertHexArrayToDecimal(const uint8_t hexArray[4])
{
	int32_t result = 0;
	for (int i = 0; i < 4; i++)
		result = (result << 8) | hexArray[i];
	if (result > 0x7FFFFFFF)
		result -= 0x100000000;
	return result;
}

/*这段代码将int类型的number按字节分解，并存储在res数组中，数组中的字节顺序为小端序*/
void toIntArray(int number, int *res, int size)
{
	unsigned int unsignedNumber = static_cast<unsigned int>(number);
	for (int i = 0; i < size; ++i)
	{
		res[i] = unsignedNumber & 0xFF;
		unsignedNumber >>= 8;
	}
}
