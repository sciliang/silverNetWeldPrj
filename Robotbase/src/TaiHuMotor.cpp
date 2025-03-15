#include "YinwMotionGlobal.h"

// 发送简单的CAN指令
std::vector<int32_t> sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList)
{
    VCI_CAN_OBJ send[1];
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 1;

    std::vector<int32_t> results; // 用于存储所有的decimal值
    for (int i = 0; i < numOfActuator; i++)
    {
        // send[0].ID = i + 1;//测试用的
        send[0].ID = canIdList[i]; // 准备实际用的
        send[0].Data[0] = commandList[i];
        if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {
            int cnt = 5;
            int reclen = 0, ind = 0;
            VCI_CAN_OBJ rec[3000];

            while ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) <= 0 && cnt)
                cnt--;
            if (cnt == 0)
                LOG(INFO) << "ops! ID " << send[0].ID << " failed after try 5 times." << std::endl;
            else
            {
                for (int j = 0; j < reclen; j++)
                {
                    std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
                    std::int32_t decimal = convertHexArrayToDecimal(hexArray);
                    results.push_back(decimal); // 将值存储到vector
                    // LOG(INFO) << "ID: " << send[0].ID << "       data: " << decimal << std::endl;
                    if (rec[j].Data[0] == 0x04)
                    {
                        LOG(INFO) << "" << send[0].ID << "  current_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x03)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  mode_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x06)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  velocity_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x16)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  MostAcc_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x18)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  MostVel_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x08)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  position_data: " << decimal << std::endl;
                    }
                }
            }
        }
        else
            break;
    }
    return results; // 返回所有的decimal值
}

std::int32_t sendSimpleCanCommand2(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList)
{
    VCI_CAN_OBJ send[1];
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 1;
    std::int32_t decimal;
    for (int i = 0; i < numOfActuator; i++)
    {
        // send[0].ID = i + 1;//测试用的
        send[0].ID = canIdList[i]; // 准备实际用的
        send[0].Data[0] = commandList[i];
        if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {
            int cnt = 5;
            int reclen = 0, ind = 0;
            VCI_CAN_OBJ rec[3000];
            while ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) <= 0 && cnt)
                cnt--;
            if (cnt == 0)
            {
                // LOG(INFO) << "ops! ID " << send[0].ID << " failed after try 5 times." << std::endl;
            }
            else
            {
                for (int j = 0; j < reclen; j++)
                {
                    std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
                    decimal = convertHexArrayToDecimal(hexArray);
                    // LOG(INFO) << "ID: " << send[0].ID << "       data: " << decimal << std::endl;
                    if (rec[j].Data[0] == 0x04)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  current_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x03)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  mode_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x06)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  velocity_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x16)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  MostAcc_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x18)
                    {
                        LOG(INFO) << "ID: " << send[0].ID << "  MostVel_data: " << decimal << std::endl;
                    }
                    else if (rec[j].Data[0] == 0x08)
                    {
                        // LOG(INFO) << "ID: " << send[0].ID << "  position_data: " << decimal << std::endl;
                    }
                }
            }
        }
        else
            break;
    }
    return decimal; // 返回所有的decimal值
}

/**
 * 将CAN命令发送到指定数量的执行器
 * 1. 要发送命令的执行器数量
 * 2. 包含每个执行器的CAN ID的数组（未使用到）
 * 3. 包含每个执行器的命令的数组
 * 4. 包含每个执行器的参数的数组，每个参数为一个32位整数
 * */
void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
    VCI_CAN_OBJ send[1];    // 定义一个CAN消息对象send，用于存储和发送CAN消息
    send[0].SendType = 0;   // 设置发送类型
    send[0].RemoteFlag = 0; // 设置为数据帧(非远程帧)
    send[0].ExternFlag = 0; // 设置为标准帧
    send[0].DataLen = 5;    // 设置数据长度为5个字节
    // 遍历每个执行器，准备并发送CAN消息
    for (int i = 0; i < numOfActuator; i++)
    {
        // send[0].ID = i + 1;				  // 设置CAN消息的ID，后面应该是用canIdList
        send[0].ID = canIdList[i];        // 设置CAN消息的ID，后面应该是用canIdList
        send[0].Data[0] = commandList[i]; // 将当前执行器的命令写入数据的第一个字节
        int res[4];
        toIntArray(parameterList[i], res, 4); // 将32位参数转换为4个字节的小端形式，存在res数组中
        for (int j = 1; j < 5; j++)           // 将res数组中的字节一次填入数组的索引1到4中
            send[0].Data[j] = res[j - 1];

        if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {
            // 查看发送的数据是什么
            // for (int cnt = 0; cnt < send[0].DataLen; cnt++)
            // {
            // 	// printf("");
            // 	// printf(" %02X", send[0].Data[cnt]);
            // }
            // LOG(INFO) << std::endl;
        }
        else
            break;
    }
}

int TaiHuMotorControl()
{
    uint8_t canidlist[10] = {1}, cmd_pos[10] = {30}, cmd_get_pos[10] = {8};
    uint8_t cmd_get_current[10] = {4}, cmd_get_mode[10] = {3}, cmd_get_vel[10] = {6};
    uint8_t cmd_get_MostAcc[10] = {16}, cmd_get_MostVel[10] = {18};
    uint32_t pos[10] = {static_cast<uint32_t>(int(-5000000))};
    pos[0] = {static_cast<uint32_t>(int(5000000))};
    sendCanCommand(1, canidlist, cmd_pos, pos);
    LOG(INFO) << "\n\n sendCanCommand finish!" << std::endl;
    sendSimpleCanCommand(1, canidlist, cmd_get_pos);
    sendSimpleCanCommand(1, canidlist, cmd_get_current);
    sendSimpleCanCommand(1, canidlist, cmd_get_mode);
    sendSimpleCanCommand(1, canidlist, cmd_get_vel);
    sendSimpleCanCommand(1, canidlist, cmd_get_MostAcc);
    sendSimpleCanCommand(1, canidlist, cmd_get_MostVel);
    return 0;
}

// 获取底层位置反馈信息,将四个钛虎电机进行统一的控制
std::vector<int32_t> THgetActualPos()
{
    std::vector<int32_t> ActualPos; // 用于存储所有的decimal值
    uint8_t numOfActuator = 4;
    uint8_t canidlist[numOfActuator] = {1, 2, 3, 4}, cmd_get_pos[numOfActuator] = {8, 8, 8, 8};
    ActualPos = sendSimpleCanCommand(numOfActuator, canidlist, cmd_get_pos);
    LOG(INFO) << "ThaiHu 111 Motor actualPos : ";
    for (const auto &value : ActualPos)
        LOG(INFO) << value << " ";
    LOG(INFO) << std::endl; // 换行
    return ActualPos;
}

// 获取底层单轴的位置反馈信息
std::int32_t THgetActualAxisPos(uint8_t Axis)
{
    std::int32_t ActualPos; // 用于存储所有的decimal值
    uint8_t numOfActuator = 1;
    uint8_t canidlist[numOfActuator] = {Axis}, cmd_get_pos[numOfActuator] = {8};
    short cnt_flag = 5;
    // 为了保证拿到的数据是最新的，因此进行多重数据的下发
    while (cnt_flag-- > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ActualPos = sendSimpleCanCommand2(numOfActuator, canidlist, cmd_get_pos);
        // LOG(INFO) << "ThaiHu: Axis-ActualPos= " << ActualPos << std::endl;
    }
    return ActualPos;
}

// 获取底层速度反馈信息,将四个钛虎电机进行统一的控制
std::vector<int32_t> ThgetActualVel()
{
    std::vector<int32_t> ActualVel; // 用于存储所有的decimal值
    uint8_t numOfActuator = 4;
    uint8_t canidlist[numOfActuator] = {1, 2, 3, 4}, cmd_get_vel[numOfActuator] = {6};
    ActualVel = sendSimpleCanCommand(numOfActuator, canidlist, cmd_get_vel);
    for (const auto &value : ActualVel)
    {
        LOG(INFO) << value << " ";
    }
    LOG(INFO) << std::endl; // 换行
    return ActualVel;
}

// 获取底层电流反馈信息,将四个钛虎电机进行统一的控制
std::vector<int32_t> ThgetActualCur()
{
    std::vector<int32_t> ActualCur; // 用于存储所有的decimal值
    uint8_t numOfActuator = 4;
    uint8_t canidlist[numOfActuator] = {1, 2, 3, 4}, cmd_get_current[numOfActuator] = {4};
    ActualCur = sendSimpleCanCommand(numOfActuator, canidlist, cmd_get_current);
    for (const auto &value : ActualCur)
    {
        LOG(INFO) << value << " ";
    }
    LOG(INFO) << std::endl; // 换行
    return ActualCur;
}

// 进行电机的期望控制,下发的期望位置
int16_t ThSetTargetPos(uint8_t *canidlist, uint32_t *pos)
{
    uint8_t numOfActuator = 1;
    // uint8_t canidlist[numOfActuator] = {1};
    uint8_t cmd_pos[numOfActuator] = {30};
    // pos[0] = {static_cast<uint32_t>(int(5000000))};
    sendCanCommand(1, canidlist, cmd_pos, pos);
    return 0;
}

// 进行电机的速度大小控制 最小值设定 速度为负
int16_t ThSetTargetVel_min(uint8_t *canidlist, uint32_t *vel)
{
    uint8_t numOfActuator = 1;
    // uint8_t canidlist[numOfActuator] = {1};
    uint8_t cmd_vel[numOfActuator] = {37};
    // pos[0] = {static_cast<uint32_t>(int(5000000))};
    sendCanCommand(1, canidlist, cmd_vel, vel);
    return 0;
}

// 进行电机的速度大小控制 最大值设定 速度为正
int16_t ThSetTargetVel_max(uint8_t *canidlist, uint32_t *vel)
{
    uint8_t numOfActuator = 1;
    // uint8_t canidlist[numOfActuator] = {1};
    uint8_t cmd_vel[numOfActuator] = {36};
    // pos[0] = {static_cast<uint32_t>(int(5000000))};
    sendCanCommand(1, canidlist, cmd_vel, vel);
    return 0;
}

// 进行电机的期望控制,下发的期望速度
int16_t ThSetTargetVel(uint8_t *canidlist, uint32_t *vel)
{
    uint8_t numOfActuator = 1;
    // uint8_t canidlist[numOfActuator] = {3};
    uint8_t cmd_vel[numOfActuator] = {29};
    // vel[0] = {static_cast<uint32_t>(int(5000000))};
    sendCanCommand(1, canidlist, cmd_vel, vel);
    return 0;
}

/**对钛虎电机的最大速度和最小速度进行限位
 * 向右转是减小，向左转是增加
 *para1：电机的转速(目前的单位还不确定)
 **/
int16_t TaiHuMotorVelLimitSet(int Vel_ID1, int Vel_ID2, int Vel_ID3, int Vel_ID4, int Vel_ID5)
{
    // 钛虎电机速度限位,分别对应1、2、3、4号钛虎电机的速度限位值
    uint32_t firstTargetVel2_min[1], firstTargetVel2_max[1];
    uint32_t secondTargetVel2_min[1], secondTargetVel2_max[1];
    uint32_t thirdTargetVel2_min[1], thirdTargetVel2_max[1];
    uint32_t fourthTargetVel2_min[1], fourthTargetVel2_max[1];
    uint32_t fifthTargetVel2_min[1], fifthTargetVel2_max[1];

    firstTargetVel2_min[0] = static_cast<uint32_t>(-Vel_ID1);
    firstTargetVel2_max[0] = static_cast<uint32_t>(Vel_ID1);

    secondTargetVel2_min[0] = static_cast<uint32_t>(-Vel_ID2);
    secondTargetVel2_max[0] = static_cast<uint32_t>(Vel_ID2);

    thirdTargetVel2_min[0] = static_cast<uint32_t>(-Vel_ID3);
    thirdTargetVel2_max[0] = static_cast<uint32_t>(Vel_ID3);

    fourthTargetVel2_min[0] = static_cast<uint32_t>(-Vel_ID4);
    fourthTargetVel2_max[0] = static_cast<uint32_t>(Vel_ID4);

    fifthTargetVel2_min[0] = static_cast<uint32_t>(-Vel_ID5);
    fifthTargetVel2_max[0] = static_cast<uint32_t>(Vel_ID5);

    uint8_t _canidlist_1[1] = {1}, _canidlist_2[1] = {2}, _canidlist_3[1] = {3}, _canidlist_4[1] = {4}, _canidlist_5[1] = {4};
    for (size_t i = 0; i < 3; i++)
    {
        ThSetTargetVel_min(_canidlist_1, firstTargetVel2_min);
        usleep(10000);
        ThSetTargetVel_max(_canidlist_1, firstTargetVel2_max);

        usleep(10000);
        ThSetTargetVel_min(_canidlist_2, secondTargetVel2_min);
        usleep(10000);
        ThSetTargetVel_max(_canidlist_2, secondTargetVel2_max);

        usleep(10000);
        ThSetTargetVel_min(_canidlist_3, thirdTargetVel2_min);
        usleep(10000);
        ThSetTargetVel_max(_canidlist_3, thirdTargetVel2_max);

        usleep(10000);
        ThSetTargetVel_min(_canidlist_4, fourthTargetVel2_min);
        usleep(10000);
        ThSetTargetVel_max(_canidlist_4, fourthTargetVel2_max);

        usleep(10000);
        ThSetTargetVel_min(_canidlist_5, fifthTargetVel2_min);
        usleep(10000);
        ThSetTargetVel_max(_canidlist_5, fifthTargetVel2_max);
    }
    return 0;
}

/**对钛虎电机进行急停操作
 *para1：电机的转速
 **/
int16_t TaiHuMotorEmergencyStop(int Vel_ID)
{
    uint32_t EmergencyStopVel[1];
    EmergencyStopVel[0] = static_cast<uint32_t>(Vel_ID);
    uint8_t _canidlist_1[1] = {1}, _canidlist_2[1] = {2}, _canidlist_3[1] = {3}, _canidlist_4[1] = {4}, _canidlist_5[1] = {5};
    for (size_t i = 0; i < 3; i++)
    {
        ThSetTargetVel(_canidlist_1, EmergencyStopVel);
        usleep(10000);
        ThSetTargetVel(_canidlist_2, EmergencyStopVel);
        usleep(10000);
        ThSetTargetVel(_canidlist_3, EmergencyStopVel);
        usleep(10000);
        ThSetTargetVel(_canidlist_4, EmergencyStopVel);
        usleep(10000);
        ThSetTargetVel(_canidlist_5, EmergencyStopVel);
    }
    return 0;
}