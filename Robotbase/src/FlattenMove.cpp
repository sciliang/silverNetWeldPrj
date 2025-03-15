#include "YinwMotionGlobal.h"
#include "YinwDeviceAutoRunFlow.h"
#include <iomanip>
using namespace HYYRobotBase;

/**【这个函数用在正常的控制阶段,不是复位阶段!!!!】
 * fun：展平移动
 * 位置（1）：向上抬起停留位置:-->flattenRightPos
 * 位置（2）：向下进行压片位置:-->flattenLeftPos
 */
// 展平移动寻零 向左减小 向右增大 向左找零
bool YinwMotion::FlatteningMovement(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    LOG(INFO) << "FlatteningMovement" << endl;
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int dataLeft_RightPos = abs(deviceParas.Paras.FlattenMove_.FlattenMoveLeftPos - deviceParas.Paras.FlattenMove_.FlattenMoveRightPos);
    int FlattenAimPos;
    switch (YWzeroNameANDpos[1][0])
    {
    case 1: // 最左侧位置是零点
    {
        if (strcmp(CmdName, "flattenLeftPos") == 0)
        {
            FlattenAimPos = YWzeroNameANDpos[1][1];
        }
        if (strcmp(CmdName, "flattenRightPos") == 0)
        {
            FlattenAimPos = YWzeroNameANDpos[1][1] + dataLeft_RightPos;
        }
        LOG(INFO) << "FlatteningMovement ZeroPosPoint is Left! FlattenAimPos = " << FlattenAimPos << std::endl;
        break;
    }
    case 2: // 最右侧位置是零点
    {
        if (strcmp(CmdName, "flattenLeftPos") == 0)
        {
            FlattenAimPos = YWzeroNameANDpos[1][1] - dataLeft_RightPos;
        }
        if (strcmp(CmdName, "flattenRightPos") == 0)
        {
            FlattenAimPos = YWzeroNameANDpos[1][1];
        }
        LOG(INFO) << "FlatteningMovement ZeroPosPoint is UP! FlattenAimPos = " << FlattenAimPos << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "ERROR , FlatteningMovement, Unknown ZeroPosPoint state!" << std::endl;
        break;
    }
    }
    int16_t FlatteningMovementRet = YinwPosMotorControl(BeforeRobot, DofNUM, YinwFeedinANDFlatteningMOVEmotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                        deviceParas.Paras.Generalparams_.MinMovePos, FlattenAimPos,
                                                        deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB,
                                                        deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB / 2);
    if (FlatteningMovementRet == 1)
    {
        LOG(INFO) << "FlatteningMovement run over!" << std::endl;
        return true;
    }
    int MotorCurrentPos[DofNUM];
    // 获取实际位置
    if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    // 打印实际位置
    for (size_t i = 0; i < DofNUM; i++)
        LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    return false;
}

// 展平移动步进控制: 展平移动寻零 向左减小 向右增大 向左找零
bool YinwMotion::FlatteningStepMove(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int dataLeft_RightPos = abs(deviceParas.Paras.FlattenMove_.FlattenMoveLeftPos - deviceParas.Paras.FlattenMove_.FlattenMoveRightPos);
    LOG(INFO) << "dataLeft_RightPos = " << dataLeft_RightPos << std::endl;
    int LeftLimitPos = YWzeroNameANDpos[1][1];                      // 左侧限位开关的位置
    int RightLimitPos = YWzeroNameANDpos[1][1] + dataLeft_RightPos; // 右侧限位开关的位置
    LOG(INFO) << "LeftLimitPos = " << LeftLimitPos << "  RightLimitPos = " << RightLimitPos << std::endl;
    // 将展平移动阈值宽度计算码值和mm之间的关系 一圈对应的码值 导程,得到的是mm
    double FlatteningStepCnd2Len = static_cast<double>(dataLeft_RightPos) / 20000.0 * 4.0; // 对应的是100mm
    LOG(INFO) << "FlatteningStepCnd2Len = " << std::setprecision(3) << FlatteningStepCnd2Len << std::endl;
    // 码值和长度之间的映射关系
    double FlatteningStepMoveScale = 20000.0 / 4.0;
    LOG(INFO) << "FlatteningStepMoveScale = " << std::setprecision(3) << FlatteningStepMoveScale << std::endl;
    int MotorCurrentPos[DofNUM];
    static float FlattenAimPos;
    switch (FlatteningMOVEflow)
    {
    case FlatteningMOVEfirst:
    {
        // 获取实际位置
        if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
        float RemainLength = static_cast<float>(RightLimitPos - MotorCurrentPos[YinwFeedinANDFlatteningMOVEmotor - 1]);
        LOG(INFO) << "RemainLength = " << std::fixed << std::setprecision(3) << RemainLength << std::endl;

        float SilverNetOuterDiameter = deviceParas.Paras.SilverNet_.YinWangWaiJing; // mm
        LOG(INFO) << "SilverNetOuterDiameter = " << std::setprecision(3) << SilverNetOuterDiameter << std::endl;
        if (RemainLength > (SilverNetOuterDiameter + 5.0) * FlatteningStepMoveScale)
        {
            FlattenAimPos = static_cast<float>(MotorCurrentPos[YinwFeedinANDFlatteningMOVEmotor - 1]) + (SilverNetOuterDiameter + 5.0) * FlatteningStepMoveScale;
            LOG(INFO) << "FlattenAimPos = " << std::fixed << std::setprecision(3) << FlattenAimPos << std::endl;
            LOG(INFO) << "FlatteningStepMove StepRun to Right!" << std::endl;
        }
        else
        {
            FlattenAimPos = YWzeroNameANDpos[1][1];
            LOG(INFO) << "FlattenAimPos = " << std::setprecision(3) << FlattenAimPos << std::endl;
            LOG(INFO) << "FlatteningMovement StepRun to LeftZero!" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        FlatteningMOVEflow = FlatteningMOVEsecond;
        break;
    }
    case FlatteningMOVEsecond:
    {
        LOG(INFO) << "FlattenAimPos = " << std::setprecision(3) << FlattenAimPos << std::endl;
        int16_t FlatteningMovementRet = YinwPosMotorControl(BeforeRobot, DofNUM, YinwFeedinANDFlatteningMOVEmotor,
                                                            deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                            deviceParas.Paras.Generalparams_.MinMovePos, static_cast<int>(FlattenAimPos),
                                                            deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB,
                                                            deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB);
        if (FlatteningMovementRet == 1)
        {
            LOG(INFO) << "FlatteningMovement run over!" << std::endl;
            return true;
        }
        break;
    }
    default:
        break;
    }
    return false;
}

// 1号钛虎电机的步进控制
int16_t YinwMotion::FirstTaiHuMotorStepControl(const YinwDeviceParas &deviceParas)
{
    //******* 计算1号电机从重新换银网之后已经旋转的圈数,分析的都是末端,即减速机端*********//
    // 电机实际位置
    int32_t PosActualVal_1 = THgetActualAxisPos(1);
    switch (FirstTaiHuMotorStepControlflow)
    {
    case FirstTaiHuMotorStepControlfirst:
    {
        LOG(INFO) << "FirstTaiHuMotorStepControlfirst!!! " << std::endl;
        float firstHasRunCircleNum = fabs((PosActualVal_1 - deviceParas.Paras.Flatten_.firstMotorApexPos) / deviceParas.Paras.TaiHu_.encoder / deviceParas.Paras.TaiHu_.ReductionRatio);
        float fractional_part = firstHasRunCircleNum - floor(firstHasRunCircleNum);
        // 四舍五入取整
        if (fractional_part > 0.5)
            firstHasRunCircleNum = ceil(firstHasRunCircleNum);
        else
            firstHasRunCircleNum = floor(firstHasRunCircleNum);
        LOG(INFO) << "firstHasRunCircleNum = " << firstHasRunCircleNum << std::endl;
        // 这一拍需要转过的度数
        float firstTartPosDu = FirstStepCtrlValue(deviceParas.Paras.TaiHu_.ReductionRatio,
                                                  deviceParas.Paras.SilverNet_.YinWangWaiJing,
                                                  firstHasRunCircleNum,
                                                  deviceParas.Paras.SilverNet_.YinWangHouDu,
                                                  deviceParas.Paras.Flatten_.firstYWtAllThick);
        firstTartPosDu = 180;
        if (firstTartPosDu < 0)
        {
            LOG(INFO) << "Error, FirstStepCtrlValue failed" << std::endl;
            return -1;
        }
        else
        {
            // 这一拍需要转过的码值
            float FirstPosDu2Code = firstTartPosDu / 360.0 * deviceParas.Paras.TaiHu_.ReductionRatio * deviceParas.Paras.TaiHu_.encoder;
            int FirstTargetDataPos = static_cast<int>(std::round(FirstPosDu2Code));
            firstTargetFinalPos[0] = static_cast<uint32_t>(FirstTargetDataPos + PosActualVal_1);
            LOG(INFO) << "TaiH_1 hao: Target Pos = " << firstTargetFinalPos[0] << std::endl;
        }
        FirstTaiHuMotorStepControlflow = FirstTaiHuMotorStepControlsecond;
        break;
    }
    case FirstTaiHuMotorStepControlsecond:
    {
        // 已转过的总圈数
        LOG(INFO) << "FirstTaiHuMotorStepControlsecond!!! " << std::endl;
        uint8_t canidlist_1[1] = {1};
        LOG(INFO) << "TaiH_1 hao: Target Pos = " << firstTargetFinalPos[0] << std::endl;
        ThSetTargetPos(canidlist_1, firstTargetFinalPos); // 下发期望的位置
        PosActualVal_1 = THgetActualAxisPos(1);           // 获取回实际位置
        LOG(INFO) << " TaiHuMotor-1-Pos = " << PosActualVal_1 << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 跳出机制
        if (abs(PosActualVal_1 - static_cast<int32_t>(firstTargetFinalPos[0])) < 1000)
        {
            LOG(INFO) << "TaiH_1 hao_StepControl finish" << std::endl;
            return 1;
        }
        else
        {
            LOG(INFO) << "TaiH_1 hao_StepControl is running....." << std::endl;
            return -1;
        }
        break;
    }
    default:
        LOG(INFO) << "FirstTaiHuMotorStepControlflow ERROR!!!" << std::endl;
        break;
    }
    return 0;
}

// 2号钛虎电机的步进控制
int16_t YinwMotion::SecondTaiHuMotorStepControl(const YinwDeviceParas &deviceParas)
{
    int32_t PosActualVal_2;
    switch (SecondTaiHuMotorStepControlflow)
    {
    case SecondTaiHuMotorStepControlfirst:
    {
        LOG(INFO) << "SecondTaiHuMotorStepControlfirst!!! " << std::endl;
        PosActualVal_2 = THgetActualAxisPos(2);
        LOG(INFO) << "TaiHuMotor-2-Pos = " << PosActualVal_2 << std::endl;
        // 2号电机控制量计算
        float SecondTarPosDu = SecondStepCtrlValue(deviceParas.Paras.TaiHu_.ReductionRatio,
                                                   deviceParas.Paras.TaiHu_.MotorDiameter,
                                                   deviceParas.Paras.SilverNet_.YinWangWaiJing);
        SecondTarPosDu = 480.0; // 逆时针旋转
        if (SecondTarPosDu < 0)
        {
            LOG(INFO) << "Error, SecondStepCtrlValue failed" << std::endl;
            return -1;
        }
        else
        {
            // 2号电机控制量计算 转过的码值
            float SecondPosDu2Code = SecondTarPosDu / 360.0 * deviceParas.Paras.TaiHu_.ReductionRatio * deviceParas.Paras.TaiHu_.encoder;
            int SecondTargetDataPos = static_cast<int>(std::round(SecondPosDu2Code));
            SecondTargetFinalPos[0] = static_cast<uint32_t>(SecondTargetDataPos + PosActualVal_2); // 准备测试用// 左旋是增大，右旋是减小
            LOG(INFO) << "TaiH_2 hao: Target Pos = " << SecondTargetFinalPos[0] << std::endl;
        }
        SecondTaiHuMotorStepControlflow = SecondTaiHuMotorStepControlsecond;
        break;
    }
    case SecondTaiHuMotorStepControlsecond:
    {
        // 2号电机运动
        LOG(INFO) << "SecondTaiHuMotorStepControlsecond!!! " << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint8_t canidlist_2[1] = {2};
        LOG(INFO) << "TaiH_2 hao: Target Pos = " << SecondTargetFinalPos[0] << std::endl;
        ThSetTargetPos(canidlist_2, SecondTargetFinalPos);
        PosActualVal_2 = THgetActualAxisPos(2);
        LOG(INFO) << " PosActualVal_2 = " << PosActualVal_2 << std::endl;
        if (abs(PosActualVal_2 - static_cast<int32_t>(SecondTargetFinalPos[0])) < 1000)
        {
            LOG(INFO) << "TaiH_2_StepControl finish" << std::endl;
            return 1;
        }
        else
        {
            LOG(INFO) << "TaiH_2_StepControl is running....." << std::endl;
            return -1;
        }
        break;
    }
    default:
        LOG(INFO) << "SecondTaiHuMotorStepControlflow ERROR!!!" << std::endl;
        break;
    }
    return 0;
}

// 4号钛虎电机的步进控制
int16_t YinwMotion::FourthTaiHuMotorStepControl(const YinwDeviceParas &deviceParas)
{
    int32_t PosActualVal_4;
    switch (FourthTaiHuMotorStepControlflow)
    {
    case FourthTaiHuMotorStepControlfirst:
    { //***********计算四号电机从重新换银网之后已经旋转的圈数,分析的都是末端,即减速机端************/
        PosActualVal_4 = THgetActualAxisPos(4);
        LOG(INFO) << "\n\nTaiHuMotor4Pos = " << PosActualVal_4 << std::endl;
        // 用绝对位置计算电机已经转过的圈数
        float fourthHasRunCircleNum = fabs((PosActualVal_4 - deviceParas.Paras.Flatten_.fourthMotorApexPos) /
                                           deviceParas.Paras.TaiHu_.encoder / deviceParas.Paras.TaiHu_.ReductionRatio);
        // 取小数
        float fractional_part = fourthHasRunCircleNum - floor(fourthHasRunCircleNum);
        // 取整操作
        if (fractional_part > 0.5)
            fourthHasRunCircleNum = ceil(fourthHasRunCircleNum);
        else
            fourthHasRunCircleNum = floor(fourthHasRunCircleNum);
        LOG(INFO) << "TaiH4_HasRunCircleNum = " << fourthHasRunCircleNum << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 4号电机控制量计算 转过的角度
        float fourthTartPosDu = FourthStepCtrlValue(deviceParas.Paras.TaiHu_.ReductionRatio, deviceParas.Paras.TaiHu_.MotorDiameter,
                                                    deviceParas.Paras.SilverNet_.YinWangWaiJing, fourthHasRunCircleNum,
                                                    deviceParas.Paras.SilverNet_.YinWangHouDu, deviceParas.Paras.Flatten_.fourthYWAllThick);
        fourthTartPosDu = 60.0; // 逆时针旋转
        if (fourthTartPosDu < 0)
        {
            LOG(INFO) << "Error, FourthStepCtrlValue failed" << std::endl;
            return -1;
        }
        else
        {
            LOG(INFO) << "fourthTartPosDu = " << fourthTartPosDu << std::endl;
            // 4号电机控制量计算 转过的码值
            float fourthPosDu2Code = fourthTartPosDu / 360.0 * deviceParas.Paras.TaiHu_.ReductionRatio * deviceParas.Paras.TaiHu_.encoder;
            LOG(INFO) << "fourthPosDu2Code = " << fourthPosDu2Code << std::endl;
            int fourthTargetDataPos = static_cast<int>(std::round(fourthPosDu2Code));
            LOG(INFO) << "fourthTargetDataPos = " << fourthTargetDataPos << std::endl;
            fourthTargetFinalPos[0] = static_cast<uint32_t>(fourthTargetDataPos + PosActualVal_4);
            LOG(INFO) << "fourthTargetFinalPos[0] = " << fourthTargetFinalPos[0] << std::endl;
            // 4号电机运动
        }
        FourthTaiHuMotorStepControlflow = FourthTaiHuMotorStepControlsecond;
        LOG(INFO) << "FourthTaiHuMotorStepControlflow!!! =" << FourthTaiHuMotorStepControlflow << std::endl;
        break;
    }
    case FourthTaiHuMotorStepControlsecond:
    {
        LOG(INFO) << "FourthTaiHuMotorStepControlsecond!!! " << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint8_t canidlist_4[1] = {4};
        ThSetTargetPos(canidlist_4, fourthTargetFinalPos);
        PosActualVal_4 = THgetActualAxisPos(4);
        LOG(INFO) << " 2-----TaiHuMotor-4-Pos = " << PosActualVal_4 << std::endl;
        if (abs(PosActualVal_4 - static_cast<int32_t>(fourthTargetFinalPos[0])) < 1000)
        {
            // LOG(INFO) << "^^^^^^^^TaiH4StepControl finish" << std::endl;
            return 1;
        }
        else
        {
            LOG(INFO) << "$$$$$$$$$$$TaiH4StepControl is running....." << std::endl;
            return -1;
        }
        break;
    }
    default:
        LOG(INFO) << "FourthTaiHuMotorStepControlflow ERROR!!!" << std::endl;
        break;
    }
    return 0;
}

/**
 * fun：银网给网及展平作业
 */
int16_t YinwMotion::YinwFeedinANDFlattening(const YinwDeviceParas &deviceParas)
{
    // FlattnMoveStausStepflow = FourthTaiHuMotor;
    switch (FlattnMoveStausStepflow)
    {
    case FourthTaiHuMotor: // 4号电机步进
    {
        if (FourthTaiHuMotorStepControl(deviceParas) != 1)
        {
            LOG(INFO) << "FourthTaiHuMotorStepControl running....." << std::endl;
        }
        else
        {
            FlattnMoveStausStepflow = SecondTaiHuMotor;
            LOG(INFO) << "FourthTaiHuMotor finish !" << std::endl;
        }
        break;
    }
    case SecondTaiHuMotor: // 2号电机步进
    {
        if (SecondTaiHuMotorStepControl(deviceParas) != 1)
        {
            LOG(INFO) << "SecondTaiHuMotorStepControl running....." << std::endl;
        }
        else
        {
            LOG(INFO) << "SecondTaiHuMotorStepControl finish !" << std::endl;
            FlattnMoveStausStepflow = FirstTaiHuMotor;
        }
        break;
    }
    case FirstTaiHuMotor: // 1号电机步进
    {
        if (FirstTaiHuMotorStepControl(deviceParas) != 1)
        {
            LOG(INFO) << "FirstTaiHuMotorStepControl running....." << std::endl;
        }
        else
        {
            LOG(INFO) << "FirstTaiHuMotorStepControl finish !" << std::endl;
            return 1;
        }
        break;
    }
    default:
    {
        LOG(INFO) << "Error, FlattnMoveStausStepflow unkownStauts!!!!!!!!!" << std::endl;
        return -1;
        break;
    }
    }
    return 0;
}