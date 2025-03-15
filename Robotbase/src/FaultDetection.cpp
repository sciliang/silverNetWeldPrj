#include <glog/logging.h>
#include "YinwMotionGlobal.h"
#include "YinwMotion.h"
using namespace HYYRobotBase;

/**
 * fun：将三个int合并成一个int
 * 参数解释：
 * Para1. 参数1
 * Para2. 参数2
 * Para3. 参数3
 */
int YinwMotion::CombinedNum2Seen(int a, int b, int c)
{
    // 将每个整数转换为字符串
    std::string strA = std::to_string(a);
    std::string strB = std::to_string(b);
    std::string strC = std::to_string(c);

    // 拼接字符串
    std::string concatenated = strA + strB + strC;
    // 将拼接后的字符串转换为整数并返回
    return std::stoi(concatenated);
}

/**
 * fun：银网设备EtherCAT电机的故障检测
 * 参数解释：
 * Para1. 设备相关参数
 * Para2. 机器人名字
 * Para3. 电机的名字
 * Para4. 循环周期(s)
 */
YinwMotion::ET_MotorFaultInfo YinwMotion::YinwET_MotorFaultDetct(const YinwDeviceParas &deviceParas, const char *deviceName, short SleepTime)
{
    ET_MotorFaultInfo motorFaultInfo = {}; // 初始化,清空所有成员
    int MotorNUM = get_group_dof(deviceName);
    short Minute2Second = 60, RateMotorCurrent = 1100; // 额定电流限制
    int MotorCurrentPos[MotorNUM], MotorTargetPos[MotorNUM], MotorTar_CurPosError[MotorNUM], MotorTar_CurPosErrorTolerance[MotorNUM];
    short MotorCurrenTorque[MotorNUM], ET_MotorErrorCode[MotorNUM]; // 电机的实际电流 电机的EtherCAT故障码
    memset(MotorTar_CurPosErrorTolerance, 0, sizeof(MotorTar_CurPosErrorTolerance));
    int RobotNameID;
    if (std::string(deviceName) == "ec_device1/robot0")
    {
        RobotNameID = 1;
        MotorTar_CurPosErrorTolerance[0] = deviceParas.Paras.ALLMotorsRatedVelocity_.YinwForgingMotorRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[1] = deviceParas.Paras.ALLMotorsRatedVelocity_.ThreeAxisYmotorRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[2] = deviceParas.Paras.ALLMotorsRatedVelocity_.ThreeAxisXmotorRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[3] = deviceParas.Paras.ALLMotorsRatedVelocity_.YinwFeedinANDFlatteningMOVERatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[4] = deviceParas.Paras.ALLMotorsRatedVelocity_.PolarPlateWarehouseMoveRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[5] = deviceParas.Paras.ALLMotorsRatedVelocity_.QualifiedProductWareHouseRatedVel / Minute2Second * SleepTime;
    }
    else if (std::string(deviceName) == "ec_device1/robot1")
    {
        RobotNameID = 2;
        MotorTar_CurPosErrorTolerance[0] = deviceParas.Paras.ALLMotorsRatedVelocity_.DiscardedProductWareHouseRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[1] = deviceParas.Paras.ALLMotorsRatedVelocity_.ThreeAxisZmotorRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[2] = deviceParas.Paras.ALLMotorsRatedVelocity_.WeldingPosMoveRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[3] = deviceParas.Paras.ALLMotorsRatedVelocity_.WeldingShelfLeftRightRatedVel / Minute2Second * SleepTime;
        MotorTar_CurPosErrorTolerance[4] = deviceParas.Paras.ALLMotorsRatedVelocity_.WeldingShelfUpDownRatedVel / Minute2Second * SleepTime;
    }

    // 获取期望位置
    if (get_group_target_position(deviceName, MotorTargetPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;

    // 获取实际位置
    if (get_group_position(deviceName, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_position ERROR!" << endl;

    // 位置误差计算
    for (int i = 0; i < MotorNUM; i++)
    {
        MotorTar_CurPosError[i] = MotorTargetPos[i] - MotorCurrentPos[i];
        // LOG(INFO) << "MotorTar_CurPosError[" << MotorTar_CurPosError[i] << "]" << endl;
        if (abs(MotorTar_CurPosError[i]) > abs(1.5 * MotorTar_CurPosErrorTolerance[i]))
        {
            int YW_ErrorCode = CombinedNum2Seen(RobotNameID, i, OVERSPEEDERROR);
            motorFaultInfo.motorSpeedFaults.push_back(YW_ErrorCode); // 记录超速故障
            // LOG(INFO) << "ERROR, Motor overspeed-1.0, MotorTargetPos[" << MotorTargetPos[i] << "] " << endl;
            // LOG(INFO) << "ERROR, Motor overspeed-1.1, MotorCurrentPos[" << MotorCurrentPos[i] << "] " << endl;
            // LOG(INFO) << "ERROR, Motor overspeed-1.2, MotorTar_CurPosError[" << MotorTar_CurPosError[i] << "] " << endl;
            // LOG(INFO) << "ERROR, Motor overspeed-1.2, MotorTar_CurPosErrorTolerance[" << MotorTar_CurPosErrorTolerance[i] << "] " << endl;
        }
        else
        {
            motorFaultInfo.motorSpeedFaults.push_back(0); // 记录超速故障
        }
    }

    // 获取实际电流
    if (get_group_torque(deviceName, MotorCurrenTorque) != 0)
        LOG(INFO) << "get_group_torque ERROR!" << endl;
    for (int i = 0; i < MotorNUM; i++)
    {
        if (abs(MotorCurrenTorque[i]) > RateMotorCurrent)
        {
            int YW_ErrorCode = CombinedNum2Seen(RobotNameID, i, OVERCURRENTERROR);
            motorFaultInfo.motorCurrentFaults.push_back(YW_ErrorCode); // 记录超流故障
            LOG(INFO) << "ERROR, Motor OverCurrent, MotorCurrenTorque[" << MotorCurrenTorque[i] << "] " << endl;
        }
        else
        {
            motorFaultInfo.motorCurrentFaults.push_back(0); // 记录超流故障
        }
    }

    // 获取
    if (get_group_ErrorCode(deviceName, ET_MotorErrorCode) != 0)
        LOG(INFO) << "get_group_ErrorCode ERROR!" << endl;
    for (int i = 0; i < MotorNUM; i++)
    {
        int YW_ErrorCode = CombinedNum2Seen(RobotNameID, i, ET_MotorErrorCode[i]);
        // LOG(INFO) << "ET_MotorErrorCode[" << ET_MotorErrorCode[i] << "]" << endl;
        motorFaultInfo.motorET_ErrorCode.push_back(YW_ErrorCode); // 记录故障码
    }
    return motorFaultInfo;
}

/**
 * fun：银网设备故障检测线程函数
 * 参数解释：
 */
void YinwMotion::YinwSystemFaultDetct()
{
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 9;
    RTimer_.cycle_times = 1;
    // 获取机器人的名字
    const char *_before_Robot = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 0);
    const char *_back_Robot = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 1);
#endif
    // 开门狗定时器
    int FaultDetctDog, FaultDetctStartProtect;
    // 故障码存储区
    ET_MotorFaultInfo MotorFaultDetctRet;
    // 三类错误标志位
    while (bRunning_Flag)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
        if (FaultDetctDog++ % 2 == 0)
        {
            // 奇数时获取第一个机器人的故障信息
            MotorFaultDetctRet = YinwET_MotorFaultDetct(deviceParas, _before_Robot, 10);
        }
        else
        {
            // 偶数时获取第二个机器人的故障信息
            MotorFaultDetctRet = YinwET_MotorFaultDetct(deviceParas, _back_Robot, 10);
        }
#endif
        // 打印 motorSpeedFaults
        for (size_t i = 0; i < MotorFaultDetctRet.motorSpeedFaults.size(); ++i)
        {
            if (MotorFaultDetctRet.motorSpeedFaults[i] != 0)
            {
                hasSpeedFault.store(true);
                LOG(INFO) << "SpeedFaults-Index[" << i << "] =" << MotorFaultDetctRet.motorSpeedFaults[i] << std::endl;
                LOG(INFO) << "System Has motorSpeedFaults !!" << endl;
            }
        }

        // 打印 motorCurrentFaults
        for (size_t i = 0; i < MotorFaultDetctRet.motorCurrentFaults.size(); ++i)
        {
            if (MotorFaultDetctRet.motorCurrentFaults[i] != 0)
            {
                hasPositionFault.store(true);
                LOG(INFO) << "CurrentFaults-Index[" << i << "] =" << MotorFaultDetctRet.motorCurrentFaults[i] << std::endl;
                LOG(INFO) << "System Has motorCurrentFaults !!" << endl;
            }
        }

        // 打印 motorET_ErrorCode
        // for (size_t i = 0; i < MotorFaultDetctRet.motorET_ErrorCode.size(); ++i)
        // {
        //     LOG(INFO) << "ErrorCode-Index[" << i << "] =" << MotorFaultDetctRet.motorET_ErrorCode[i] << std::endl;
        //     if (MotorFaultDetctRet.motorET_ErrorCode[i] != 0)
        //     {
        //         hasErrorCodeFault.store(true);
        //         LOG(INFO) << "System Has motorET_ErrorCode !!" << endl;
        //     }
        // }

        if ((hasSpeedFault || hasPositionFault || hasErrorCodeFault) && (FaultDetctStartProtect++ > 10000))
        {
            FaultDetctStartProtect = 11000;
            std::cout << "System has some faults. Waiting for manual reset..." << std::endl;
            // 阻塞等待直到 resumeFlag 被置为 true（外部通知错误已处理）
            std::unique_lock<std::mutex> lock(FaultWaitMtx);
            FaultWait.wait(lock, [this]()
                           { return ResumeFlag.load(); });
            // 恢复后重置状态
            ResumeFlag.store(false);
        }
        else
        {
            // std::cout << "System is running normally." << std::endl;
        }

        if (FaultDetctDog == 100)
        {
            FaultDetctDog = 0;
        }
    }
}
