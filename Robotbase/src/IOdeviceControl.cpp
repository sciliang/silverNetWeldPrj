#include <array>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector> // 添加vector头文件
#include <cstdint>
#include "controlcan.h"
#include <glog/logging.h>
#include "IOdeviceControl.h"
#include <thread>
#include <cstdint>
#include <string.h>
/**
 * 发的是1,就是IO板子1要相应的反馈;发的是2,就是板子2要相应的反馈
 * */
short IoBoardCanCmdName = -1;

/**
 * 控制设备的IO的can通信
 * para1：
 * para2：
 */
void send_IO_CanCommand(uint16_t CAN_ID, uint8_t *cmdLib)
{
    VCI_CAN_OBJ send[1]; // 定义一个CAN消息对象send，用于存储和发送CAN消息
    LOG(INFO) << "Send_IO_CanCommand: ";
    for (size_t i = 0; i < 8; i++)
    {
        // LOG(INFO) << cmdLib[i];
        std::cout << cmdLib[i];
    }
    LOG(INFO) << std::endl;

    send[0].ID = CAN_ID;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0; // 数据帧
    send[0].ExternFlag = 0; // 扩展帧
    send[0].DataLen = 8;

    // 数据填充
    send[0].Data[0] = cmdLib[0];
    send[0].Data[1] = cmdLib[1];
    send[0].Data[2] = cmdLib[2];
    send[0].Data[3] = cmdLib[3];
    send[0].Data[4] = cmdLib[4];
    send[0].Data[5] = cmdLib[5];
    send[0].Data[6] = cmdLib[6];
    send[0].Data[7] = cmdLib[7];
    // 发送函数。返回值为发送成功的帧数
    if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
    {
        // for (int i = 0; i < send[0].DataLen; i++)
        // printf(" %02X", send[0].Data[i]);
    }
    else
    {
        LOG(INFO) << "ERROR, VCI_Transmit failed" << std::endl;
        // break;
    }
}

uint8_t *createCanCommand(uint8_t Cmd2_1, uint8_t Cmd4_3, uint8_t Cmd6_5, uint8_t Cmd8_7,
                          uint8_t Cmd10_9, uint8_t Cmd12_11, uint8_t Cmd14_13, uint8_t Cmd16_15)
{
    static uint8_t IOcanCMD[8];
    // 可以进行修改或其他操作
    IOcanCMD[0] = Cmd2_1;
    IOcanCMD[1] = Cmd4_3;
    IOcanCMD[2] = Cmd6_5;
    IOcanCMD[3] = Cmd8_7;
    IOcanCMD[4] = Cmd10_9;
    IOcanCMD[5] = Cmd12_11;
    IOcanCMD[6] = Cmd14_13;
    IOcanCMD[7] = Cmd16_15;
    return IOcanCMD; // 返回 std::array
}

/**IO上电控制说明
 * Y1:3台钛虎,银网展平-->Power
 * Y2:1台钛虎,银网压片移动-->Power
 * Y3:鸣志步进电机1,展平移动-->Power
 * Y4:鸣志步进电机2,极片库-->Power
 * Y5:鸣志步进电机3,成品库-->Power
 * Y6:鸣志步进电机4,废品库-->Power
 * Y7:电机抱闸,压机电机抱闸-->Power
 * Y8:3个迈信驱动器,三轴移动-->Power
 * Y9:信捷驱动器,压机-->Power
 * Y10:电机抱闸,三轴移动电机抱闸-->Power
 * Y11：鸣志步进电机*2,焊接台固定银网及极片-->Power
 * Y12：气泵,吸附极片-->Airpump
 * Y13：照明灯,设备照明-->Floodlight
 * Y14：三色灯报警绿色,设备正常工作-->Greenlight
 * Y15：三色灯报警红色,设备待机-->Redlight
 * Y16：三色灯报警橙色,设备报警(自带蜂鸣)-->Orangelight
 */
void IODeviceTeleControl(const char *CmdName)
{
    uint16_t IOcanID_Y = 0x0706;
    uint8_t *IOcanCMD;
    // 0:设备上电和下电 1:气泵打开和关闭 2:照明灯打开和关闭
    // 3:绿灯打开和关闭 4:红灯打开和关闭 5:橙色灯打开和关闭
    static bool IOChildDevice[6];
    if (strcmp(CmdName, "power") == 0) // 0-设备上电
    {
        LOG(INFO) << "Device power up" << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        IOChildDevice[0] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "poweroff") == 0) // 0-设备下电
    {
        IOChildDevice[0] = false;
        LOG(INFO) << "Device power off" << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
        memset(IOChildDevice, 0, sizeof(IOChildDevice));
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Airpumpon") == 0) // 1-气泵打开
    {
        LOG(INFO) << "Device Airpumpon.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00);
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x00);
            LOG(INFO) << "Airpump Open, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            LOG(INFO) << "Airpump Open, Red Open." << std::endl;
            // //--------------------- -21----43----65----87---10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x01);
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            LOG(INFO) << "Airpump Open, Orange Open." << std::endl;
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x10);
        }
        if (IOChildDevice[2]) // 照明
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00);
            LOG(INFO) << "Airpump Open, Light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00);
                LOG(INFO) << "Airpump Open, Light Open, Green Open." << std::endl;
            }
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01);
                LOG(INFO) << "Airpump Open, Light Open, Red Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10);
                LOG(INFO) << "Airpump Open, Light Open, Orange Open." << std::endl;
            }
        }
        IOChildDevice[1] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Airpumpoff") == 0) // 1-气泵关闭
    {
        LOG(INFO) << "Device Airpumpoff.." << std::endl;
        IOChildDevice[1] = false;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00);
            LOG(INFO) << "Airpump Close, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01);
            LOG(INFO) << "Airpump Close, Red Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10};
            LOG(INFO) << "Airpump Close, Orange Open." << std::endl;
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10);
        }
        if (IOChildDevice[2]) // 照明
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00);
            LOG(INFO) << "Airpump Close, Light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00);
                LOG(INFO) << "Airpump Close, Light Open, Green Open." << std::endl;
            }
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01);
                LOG(INFO) << "Airpump Close, Light Open, Red Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10);
                LOG(INFO) << "Airpump Close, Light Open, Orange Open." << std::endl;
            }
        }
        IOChildDevice[1] = false;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Floodlightup") == 0) // 2-照明灯打开
    {
        LOG(INFO) << "Device Floodlight Open.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00);
            LOG(INFO) << "Floodlight Open, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00);
            LOG(INFO) << "Floodlight Open, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01);
            LOG(INFO) << "Floodlight Open, Red Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10);
            LOG(INFO) << "Floodlight Open, Orange Open." << std::endl;
        }
        IOChildDevice[2] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Floodlightdown") == 0) // 2-照明灯关闭
    {
        LOG(INFO) << "Device Floodlight Close.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00);
            LOG(INFO) << "Floodlight Close, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00);
            LOG(INFO) << "Floodlight Close, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01);
            LOG(INFO) << "Floodlight Close, Red Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10);
            LOG(INFO) << "Floodlight Close, Orange Open." << std::endl;
        }
        IOChildDevice[2] = false;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Greenlightup") == 0) // 3-绿灯打开
    {
        LOG(INFO) << "Device Greenlight Open." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x00);
            LOG(INFO) << "Greenlight Open, Airpump Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x01);
            LOG(INFO) << "Greenlight Open, Red Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x10);
            LOG(INFO) << "Greenlight Open, Orange Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00);
            LOG(INFO) << "Greenlight Open, Light Open." << std::endl;
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x01);
                LOG(INFO) << "Greenlight Open, Light Open, Red Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x10);
                LOG(INFO) << "Greenlight Open, Light Open, Orange Open." << std::endl;
            }
        }
        IOChildDevice[3] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Greenlightdown") == 0) // 3-绿灯关闭
    {
        LOG(INFO) << "Device Greenlight.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        LOG(INFO) << "Greenlight Close." << std::endl;
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00);
            LOG(INFO) << "Greenlight Close, Airpump Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01);
            LOG(INFO) << "Greenlight Close, Red Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10);
            LOG(INFO) << "Greenlight Close, Orange Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00);
            LOG(INFO) << "Greenlight Close, Foodlight Open." << std::endl;
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01);
                LOG(INFO) << "Greenlight Close, Foodlight Open, Red Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10);
                LOG(INFO) << "Greenlight Close, Foodlight Open, Orange Open." << std::endl;
            }
        }
        IOChildDevice[3] = false;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Redlightup") == 0) // 4-红灯打开
    {
        LOG(INFO) << "Device Redlight Open.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x01);
            LOG(INFO) << "Greenlight Close, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x01);
            LOG(INFO) << "Greenlight Close, Green Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x11};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x11);
            LOG(INFO) << "Greenlight Close, Orange Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01);
            LOG(INFO) << "Greenlight Close, Light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x01);
                LOG(INFO) << "Greenlight Close, Light Open, Green Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x11};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x11);
                LOG(INFO) << "Greenlight Close, Light Open, Orange Open." << std::endl;
            }
        }
        IOChildDevice[4] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Redlightdown") == 0) // 4-红灯关闭
    {
        LOG(INFO) << "Device Redlight Close.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00);
            LOG(INFO) << "Redlight Close, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00);
            LOG(INFO) << "Redlight Close, Green Open." << std::endl;
        }
        if (IOChildDevice[5]) // 橙灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10);
            LOG(INFO) << "Redlight Close, Orange Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00);
            LOG(INFO) << "Redlight Close, Light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00);
                LOG(INFO) << "Redlight Close, Light Open,Green Open." << std::endl;
            }
            if (IOChildDevice[5]) // 橙灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10);
                LOG(INFO) << "Redlight Close, Light Open,Orange Open." << std::endl;
            }
        }
        IOChildDevice[4] = false;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Orangelightup") == 0) // 5-橙色灯打开
    {
        LOG(INFO) << "Device Orangelight Open.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x10);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x10);
            LOG(INFO) << "Orangelight Open, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11,0x11, 0x01, 0x10, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x10);
            LOG(INFO) << "Orangelight Open, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x11};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x11);
            LOG(INFO) << "Orangelight Open, Red Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x10);
            LOG(INFO) << "Orangelight Open, light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x10};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x10);
                LOG(INFO) << "Orangelight Open, Green Open." << std::endl;
            }
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x11};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x11);
                LOG(INFO) << "Orangelight Open, Red Open." << std::endl;
            }
        }
        IOChildDevice[5] = true;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else if (strcmp(CmdName, "Orangelightdown") == 0) // 5-橙色灯关闭
    {
        LOG(INFO) << "Device Orangelight Close.." << std::endl;
        // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
        // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00};
        IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x00);
        if (IOChildDevice[1]) // 气泵
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00);
            LOG(INFO) << "Orangelight Close, Airpump Open." << std::endl;
        }
        if (IOChildDevice[3]) // 绿灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x10, 0x00);
            LOG(INFO) << "Orangelight Close, Green Open." << std::endl;
        }
        if (IOChildDevice[4]) // 红灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x00, 0x01);
            LOG(INFO) << "Orangelight Close, Red Open." << std::endl;
        }
        if (IOChildDevice[2]) // 照明灯
        {
            // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
            // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00};
            IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x00);
            LOG(INFO) << "Orangelight Close, light Open." << std::endl;
            if (IOChildDevice[3]) // 绿灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x11, 0x00);
                LOG(INFO) << "Orangelight Close, Orange Open." << std::endl;
            }
            if (IOChildDevice[4]) // 红灯
            {
                // //--------------------- -21----43----65----87--10 9--12 11--14 13--16-15
                // uint8_t IOcanCMD[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01};
                IOcanCMD = createCanCommand(0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x01);
                LOG(INFO) << "Orangelight Close, Red Open." << std::endl;
            }
        }
        IOChildDevice[5] = false;
        send_IO_CanCommand(IOcanID_Y, IOcanCMD);
    }
    else
    {
        printf("CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
    }
}

/**
 * 接收
 *
 */
std::vector<uint8_t> recv_IO_CanCommand()
{
    VCI_CAN_OBJ rec[3000];             // 接收缓存，设为3000为佳
    int ind = 0, reclen = 0;           //
    std::vector<uint8_t> datafeedback; // 用于存储接收到的数据
    // datafeedback.clear();
    if ((reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100)) > 0)
    {
        // LOG(INFO) << "\n reclen = " << reclen << std::endl;
        for (int j = 0; j < reclen; j++)
        {
            if (((IoBoardCanCmdName == 1) && (rec[j].ID == 0x0106)) ||
                ((IoBoardCanCmdName == 2) && (rec[j].ID == 0x0107)))
            {
                LOG(INFO) << "Recv_IO_CanCommand:";
                for (int i = 0; i < rec[j].DataLen; i++)
                {
                    printf(" %02x ", rec[j].Data[i]); // 打印内存中的所有数据
                    datafeedback.push_back(rec[j].Data[i]);
                }
                LOG(INFO) << std::endl;
            }
        }
    }
    return datafeedback; // 返回接收到的所有数据
}

/**
 * 发送获取反馈信息的指令:
 * IO板子1: 名字--> IOfirst
 * IO板子2: 名字--> IOsecond
 */
void IO_X_PinsCmdSEND(const char *IO_name)
{
    uint16_t IOcanID_X;
    if (strcmp(IO_name, "IOfirst") == 0)
    {
        IOcanID_X = 0x0206;
        IoBoardCanCmdName = 1;
    }
    if (strcmp(IO_name, "IOsecond") == 0)
    {
        IOcanID_X = 0x0207;
        IoBoardCanCmdName = 2;
    }
    uint8_t IOcanCMD[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_IO_CanCommand(IOcanID_X, IOcanCMD);
}

/**针对IO板子1：设备反馈读取 这个是真正测试好用留下作为参考
 * 极片库：--> X1 --> IOstatus[0]
 * 合格品(上)：-->X2 --> IOstatus[1]
 * 合格品(下)：-->X3 --> IOstatus[2]
 * 废品(上)：-->X4 --> IOstatus[3]
 * 废品(下)：-->X5 --> IOstatus[4]
 * 气泵(吸片传感器)：-->X6 --> IOstatus[5]
 * 焊台(右)：-->X7 --> IOstatus[6]
 * 焊台(中)：-->X8 --> IOstatus[7]
 * 焊台(左)：-->X9 --> IOstatus[8]
 * 压机(上)：-->X13 --> IOstatus[12]
 * 压机(下)：-->X14 --> IOstatus[13]
 * 焊架(A-前下)：-->X15 --> IOstatus[14]
 * 焊架(B-中)：-->X16 --> IOstatus[15]
 **/
std::array<bool, 16> IO_X_PinsCmdGET()
{
    std::array<bool, 16> IOstatus = {false}; // 初始化为 false
    std::vector<uint8_t> datafeedback;       // 用于存储接收到的数据
    datafeedback = recv_IO_CanCommand();
    if (datafeedback.empty())
        LOG(INFO) << "No data received!!" << std::endl;
    else
    {
        // LOG(INFO) << "IODeviceFeedback, RecvData = [";
        for (size_t i = 0; i < datafeedback.size(); i++)
        {
            // printf("%02x ", datafeedback[i]);
            // 加入边界检查，确保不会越界访问 IOstatus
            if (2 * i + 1 >= IOstatus.size())
            {
                std::cerr << "Error: datafeedback size exceeds IOstatus capacity." << std::endl;
                break; // 超出范围，退出循环
            }
            // 根据反馈数据解析 IO 状态
            switch (datafeedback[i])
            {
            case 0x00:
            {
                IOstatus[2 * i] = false;
                IOstatus[2 * i + 1] = false;
                break;
            }
            case 0x01:
            {
                IOstatus[2 * i] = true;
                IOstatus[2 * i + 1] = false;
                break;
            }
            case 0x10:
            {
                IOstatus[2 * i] = false;
                IOstatus[2 * i + 1] = true;
                break;
            }
            case 0x11:
            {
                IOstatus[2 * i] = true;
                IOstatus[2 * i + 1] = true;
                break;
            }
            default:
            {
                std::cerr << "Warning: Unknown datafeedback value: "
                          << std::hex << datafeedback[i] << std::endl;
                datafeedback.clear();
                IOstatus = {false};
                break;
            }
            }
        }
        // LOG(INFO) << "]" << std::endl;
        LOG(INFO) << "IO_X_PinsCmdGET_IOstatus Data=[ ";
        for (size_t i = 0; i < 16; i++)
            printf("%d ", IOstatus[i]);
        LOG(INFO) << "]" << std::endl;
    }
    LOG(INFO) << std::endl;
    return IOstatus;
}
