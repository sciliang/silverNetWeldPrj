#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;

/**
 * fun：银网设备EtherCAT电机位置控制
 * 参数解释：
 * Para1. 机器人名字
 * Para2. 电机总数
 * Para3. 电机名字
 * Para4. 最大位置量
 * Para5. 零位位置量
 * Para6. 最小位置量
 * Para7. 目标位置量
 * Para8. 步进量
 * para9. 到达位置阈值
 */
int16_t YinwMotion::YinwPosMotorControl(const char *deviceName, short MotorNUM, short MotorName, int MaxMovePos, int ZeroPos,
                                        int MinMovePos, int AIMPOS, int dataMoveS, int Arrivethreshold)
{
    int MotorCurrentPos[MotorNUM], targetMovePos;
    short MotorCurrenTorque[MotorNUM], dataMoveS_mini = 10, YW_acceleration = 100;
    // static float YW_velocity;
    LOG(INFO) << " \nMaxMovePos=" << MaxMovePos
              << " \ndeviceName=" << deviceName
              << " \nMotorNUM=" << MotorNUM
              << " \nMotorName=" << MotorName
              << " \nMaxMovePos=" << MaxMovePos
              << " \nZeroPos=" << ZeroPos
              << " \nMinMovePos=" << MinMovePos
              << " \nAIMPOS=" << AIMPOS
              << " \ndataMoveS=" << dataMoveS
              << " \nArrivethreshold=" << Arrivethreshold << std::endl;
    // 最大值限定在2000
    if (dataMoveS >= 4000)
        dataMoveS = 4000;

    // 获取期望位置
    if (get_group_target_position(deviceName, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;

    // 打印期望值
    // for (size_t i = 0; i < MotorNUM; i++)
    //     LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;

    // 获取电流值
    if (get_group_torque(deviceName, MotorCurrenTorque) != 0)
        LOG(INFO) << "get_group_torque ERROR!" << endl;

    // 逼近运动
    if (abs(AIMPOS - MotorCurrentPos[MotorName - 1]) <= 1.2 * Arrivethreshold)
    {
        LOG(INFO) << "Yinw Motor Arrived!" << std::endl;
        return 1;
    }
    else
    {
        if ((abs(AIMPOS - MotorCurrentPos[MotorName - 1]) > Arrivethreshold) && (abs(AIMPOS - MotorCurrentPos[MotorName - 1]) <= dataMoveS))
        {

            // 慢速逼近
            if ((AIMPOS - MotorCurrentPos[MotorName - 1]) > 0)
            {
                targetMovePos = MotorCurrentPos[MotorName - 1] + dataMoveS_mini; // dataMoveS_mini
            }
            else
            {
                targetMovePos = MotorCurrentPos[MotorName - 1] - dataMoveS_mini; // dataMoveS_mini
            }
        }
        else
        {

            // 快速逼近
            if ((AIMPOS - MotorCurrentPos[MotorName - 1]) > 0)
            {
                targetMovePos = MotorCurrentPos[MotorName - 1] + dataMoveS; // dataMoveS
            }
            else
            {
                targetMovePos = MotorCurrentPos[MotorName - 1] - dataMoveS; // dataMoveS
            }
        }
    }

    // 运动限位
    if (targetMovePos >= MaxMovePos)
    {
        targetMovePos = MaxMovePos;
    }
    else if (targetMovePos <= MinMovePos)
    {
        targetMovePos = MinMovePos;
    }
    LOG(INFO) << "deviceName = " << deviceName
              << " targetMovePos = " << targetMovePos
              << " MotorName = " << MotorName << endl;
    // 运动下发
    if (set_axis_position(deviceName, targetMovePos, MotorName) != 0)
    {
        LOG(INFO) << "set_axis_position ERROR!" << endl;
    }
    return 0;
}

/**
 * fun：银网设备EtherCAT电机位置控制
 * 参数解释：
 * Para1. 机器人名字
 * Para2. 电机总数
 * Para3. 电机名字
 * Para4. 最大位置量
 * Para5. 零位位置量
 * Para6. 最小位置量
 * Para7. 目标位置量
 * Para8. 步进量
 * para9. 到达位置阈值
 */
int16_t YinwMotion::YinwPosMotorControl_New(const char *deviceName, short MotorNUM, short MotorName, int MaxMovePos, int ZeroPos,
                                            int MinMovePos, int AIMPOS, int dataMoveS, int Arrivethreshold)
{
    int MotorCurrentPos[MotorNUM], targetMovePos;
    short MotorCurrenTorque[MotorNUM];
    int YW_acceleration = 100, dataMoveS_mini = 10; // 初始化初值
    int maxSpeed = dataMoveS;                       // 相当于是最大速度
    int ACC_DCC_threshold = 10 * dataMoveS;
    // 加速度定向配置
    if ((MotorName == YinwForgingMotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == ThreeAxisYmotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 10;
        dataMoveS_mini = 50;
        ACC_DCC_threshold = 100 * dataMoveS;
    }
    else if ((MotorName == ThreeAxisXmotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 10;
        dataMoveS_mini = 50;
        ACC_DCC_threshold = 100 * dataMoveS;
    }
    else if ((MotorName == YinwFeedinANDFlatteningMOVEmotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == PolarPlateWarehouseMoveMotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == QualifiedProductWareHouseMoveMotor) && (std::string(deviceName) == "ec_device1/robot0"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == DiscardedProductWareHouseMoveMotor) && (std::string(deviceName) == "ec_device1/robot1"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == ThreeAxisZmotor) && (std::string(deviceName) == "ec_device1/robot1"))
    {
        YW_acceleration = 10;
        dataMoveS_mini = 50;
        ACC_DCC_threshold = 100 * dataMoveS;
    }
    else if ((MotorName == WeldingPosMoveMotor) && (std::string(deviceName) == "ec_device1/robot1"))
    {
        YW_acceleration = 4000;
        dataMoveS_mini = 400;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == WeldingShelfLeftRightMotor) && (std::string(deviceName) == "ec_device1/robot1"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }
    else if ((MotorName == WeldingShelfUpDownMotor) && (std::string(deviceName) == "ec_device1/robot1"))
    {
        YW_acceleration = 100;
        dataMoveS_mini = 10;
        ACC_DCC_threshold = 10 * dataMoveS;
    }

    // LOG(INFO) << "YW_acceleration = " << YW_acceleration;

    // // 最大值限定在4000
    // if (dataMoveS >= 4000)
    //     dataMoveS = 4000;
    // 获取期望位置
    if (get_group_target_position(deviceName, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;

    // 获取电流值
    if (get_group_torque(deviceName, MotorCurrenTorque) != 0)
        LOG(INFO) << "get_group_torque ERROR!" << endl;

    int positionDifference = AIMPOS - MotorCurrentPos[MotorName - 1];
    // 使用独立的 currentSpeed
    int &currentSpeed = (std::string(deviceName) == "ec_device1/robot0")
                            ? this->ET_MotorsCurSpeed[MotorName - 1]
                            : this->ET_MotorsCurSpeed[MotorName + 6 - 1];

    if (MotorName < 1 || MotorName > MotorNUM)
    {
        LOG(INFO) << "Invalid MotorName!" << std::endl;
        return -1; // 错误处理
    }

    if ((abs(positionDifference) < 1.2 * Arrivethreshold))
    {
        LOG(INFO) << "Yinw Motor Arrived!" << std::endl;
        resetET_MotorSpeed(deviceName, MotorName); // 速度重置
        return 1;
    }
    else
    {
        // 根据目标与当前位置的差距，调整速度，实施平滑处理
        if (abs(positionDifference) >= dataMoveS)
        {
            // LOG(INFO) << "positionDifference > dataMoveS" << std::endl;
            if (abs(positionDifference) < ACC_DCC_threshold) // 快要接近
            {
                // 接近目标时，减速
                currentSpeed = std::max(dataMoveS_mini, currentSpeed - YW_acceleration);
                // LOG(INFO) << "positionDifference < 10 * dataMoveS" << std::endl;
            }
            else // 离的还比较远
            {
                // 距离较远时，加速
                currentSpeed = std::min(dataMoveS, currentSpeed + YW_acceleration);
                // LOG(INFO) << "positionDifference >= 10 * dataMoveS" << std::endl;
            }
        }
        else
        {
            // 当接近目标位置时，使用最小速度进行精细逼近
            currentSpeed = dataMoveS_mini;
            // LOG(INFO) << "positionDifference < dataMoveS" << std::endl;
        }
    }
    // 限制最大速度
    if (currentSpeed > maxSpeed)
    {
        currentSpeed = maxSpeed;
        LOG(INFO) << "currentSpeed > maxSpeed" << std::endl;
    }
    // 限制速度为≥0
    currentSpeed = std::max(0, currentSpeed);
    // LOG(INFO) << "currentSpeed = " << currentSpeed;
    // 选择方向并设置目标位置
    targetMovePos = MotorCurrentPos[MotorName - 1] + (positionDifference > 0 ? currentSpeed : -currentSpeed);

    // 运动限位
    if (targetMovePos >= MaxMovePos)
        targetMovePos = MaxMovePos;
    else if (targetMovePos <= MinMovePos)
        targetMovePos = MinMovePos;

    // LOG(INFO) << "deviceName = " << deviceName << " targetMovePos = " << targetMovePos << " MotorName = " << MotorName << endl;
    // 运动下发
    if (set_axis_position(deviceName, targetMovePos, MotorName) != 0)
    {
        LOG(INFO) << "set_axis_position ERROR!" << endl;
        return -1; // 错误处理
    }
    return 0;
}

// 每次新的动作开始时清零对应电机的速度
void YinwMotion::resetET_MotorSpeed(const char *deviceName, short MotorName)
{
    if (std::string(deviceName) == "ec_device1/robot0")
    {
        ET_MotorsCurSpeed[MotorName - 1] = 0;
    }
    else if (std::string(deviceName) == "ec_device1/robot1")
    {
        ET_MotorsCurSpeed[MotorName + 6 - 1] = 0;
    }
}

/**
 * fun：银网设备EtherCAT电机速度控制
 * 参数解释：
 * Para1. 机器人名字
 * Para2. 电机名字
 * Para3. 期望速度
 * Para4. 最大速度
 * Para5. 最小速度
 */
int16_t YinwMotion::YinwVELMotorControl(const char *deviceName, short MotorName, int targetMoveVEL, int MaxMoveVEL, int MinMoveVEL)
{
    // 运动限位
    if (targetMoveVEL >= MaxMoveVEL)
    {
        targetMoveVEL = MaxMoveVEL;
    }
    else if (targetMoveVEL <= MinMoveVEL)
    {
        targetMoveVEL = MinMoveVEL;
    }

    // 运动下发
    if (set_axis_position(deviceName, targetMoveVEL, MotorName) != 0)
    {
        LOG(INFO) << "set_axis_position ERROR!" << endl;
    }
    return 0;
}