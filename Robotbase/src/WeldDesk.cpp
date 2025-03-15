#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;

/**【这个函数用在正常的控制阶段,不是复位阶段!!!!】
 * fun：焊台控制：
 * 位置（1）：焊接准备位置A:-->WeldPreparePos
 * 位置（2）：焊接位置B:-->WeldingPos
 * 位置（3）：检测位置C:-->WeldDetectingPos
 * 位置（4）：产品取走位置D:-->PickWeldPos
 */
// 焊台寻零 向右侧走(负方向)  向左为正  向右为负 向左寻零
bool YinwMotion::WeldingDeskCtrl(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    const char *backRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(backRobot);
    int AB_length = abs(deviceParas.Paras.weldingfixture_.WeldPreparePos - deviceParas.Paras.weldingfixture_.WeldingPos);
    int BC_length = abs(deviceParas.Paras.weldingfixture_.WeldingPos - deviceParas.Paras.weldingfixture_.WeldDetectPos);
    int CD_length = abs(deviceParas.Paras.weldingfixture_.WeldDetectPos - deviceParas.Paras.weldingfixture_.WeldProductPos);
    int WeldingDeskAIMPOS;
    // LOG(INFO) << "@@@YWzeroNameANDpos[0][0] = " << YWzeroNameANDpos[0][0] << std::endl;
    switch (YWzeroNameANDpos[0][0])
    {
    case 1: // 原点在D 最右侧
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length + BC_length + AB_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length + BC_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is D!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        // LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is D, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 2: // 原点在B
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + AB_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - BC_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - BC_length - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is B!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        // LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is B, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 3: // 原点在A 最左侧
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length - BC_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length - BC_length - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is A!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        // LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is A, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 4: // 原点在C
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + AB_length + BC_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + BC_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is C!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        // LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is C, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "Error, WeldingDeskCtrl, Unknown ZeroPosPoint state!" << std::endl;
        return -1;
    }
    }

    // 运动控制 YinwPosMotorControl YinwPosMotorControl_New
    int16_t WeldingDeskCtrlRet = YinwPosMotorControl_New(backRobot, DofNUM, WeldingPosMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                         deviceParas.Paras.Generalparams_.MinMovePos, WeldingDeskAIMPOS,
                                                         deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD,
                                                         deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD);

    if (WeldingDeskCtrlRet == 1)
    {
        LOG(INFO) << "WeldingDeskCtrl arrive AimPos!" << endl;
        return true;
    }

    int MotorCurrentPos[DofNUM];
    // 获取实际位置
    if (get_group_position(backRobot, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    return false;
}

/**【这个函数用在焊台在取走产品后的复位阶段】
 * fun：焊台控制：
 * 位置（1）：焊接准备位置A:-->WeldPreparePos
 * 位置（2）：焊接位置B:-->WeldingPos
 * 位置（3）：检测位置C:-->WeldDetectingPos
 * 位置（4）：产品取走位置D:-->PickWeldPos
 */
// 焊台寻零 向右侧走(负方向)  向左为正  向右为负 向左寻零
bool YinwMotion::WeldingDeskCtrlReset(const YinwDeviceParas &deviceParas, const char *CmdName)
{
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 3;
    RTimer_.cycle_times = 1;
#endif
    const char *backRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(backRobot);
    int AB_length = abs(deviceParas.Paras.weldingfixture_.WeldPreparePos - deviceParas.Paras.weldingfixture_.WeldingPos);
    int BC_length = abs(deviceParas.Paras.weldingfixture_.WeldingPos - deviceParas.Paras.weldingfixture_.WeldDetectPos);
    int CD_length = abs(deviceParas.Paras.weldingfixture_.WeldDetectPos - deviceParas.Paras.weldingfixture_.WeldProductPos);
    int WeldingDeskAIMPOS;
    LOG(INFO) << "@@@YWzeroNameANDpos[0][0] = " << YWzeroNameANDpos[0][0] << std::endl;

    switch (YWzeroNameANDpos[0][0])
    {
    case 1: // 原点在D 最右侧
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length + BC_length + AB_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length + BC_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + CD_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is D!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is D, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 2: // 原点在B
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + AB_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - BC_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - BC_length - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is B!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is B, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 3: // 原点在A 最左侧
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length - BC_length;
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - AB_length - BC_length - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is A!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is A, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    case 4: // 原点在C
    {
        if (strcmp(CmdName, "WeldPreparePos") == 0) // A
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + AB_length + BC_length;
        }
        else if (strcmp(CmdName, "WeldingPos") == 0) // B
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] + BC_length;
        }
        else if (strcmp(CmdName, "WeldDetectingPos") == 0) // C
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1];
        }
        else if (strcmp(CmdName, "PickWeldPos") == 0) // D
        {
            WeldingDeskAIMPOS = YWzeroNameANDpos[0][1] - CD_length;
        }
        else
        {
            printf("WeldingDeskCtrl ZeroPos is C!!, CmdName Error: %s\n", CmdName); // 可选：处理未识别的命令
        }
        LOG(INFO) << "WeldingDeskCtrl ZeroPosPoint is C, WeldingDeskAIMPOS = " << WeldingDeskAIMPOS << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "Error, WeldingDeskCtrl, Unknown ZeroPosPoint state!" << std::endl;
        return -1;
    }
    }

    // 循环控制运动
    while (WeldingDeskResetThreadisRunning)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        LOG(INFO) << "#### ---WeldingDeskReset----ThreadisRunning is running!#####" << endl;
        // 运动控制
        int16_t WeldingDeskCtrlRet = YinwPosMotorControl_New(backRobot, DofNUM, WeldingPosMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                             deviceParas.Paras.Generalparams_.MinMovePos, WeldingDeskAIMPOS,
                                                             deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD,
                                                             deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD);
        if (WeldingDeskCtrlRet == 1)
        {
            LOG(INFO) << "WeldingDeskCtrl arrive AimPos!" << endl;
            for (size_t i = 0; i < 5; i++)
            {
                LOG(INFO) << "WeldingDeskResetThreadisRunning = " << WeldingDeskResetThreadisRunning << endl;
                LOG(INFO) << "!!!!!!!!!!WeldingDeskResetThreadisRunning EXIT succeed!!!!!!!!!!" << endl;
                WeldingDeskResetThreadisRunning.store(false);
            }

            break; // 退出循环，函数后续返回 true，线程结束
        }
        int MotorCurrentPos[DofNUM];
        // 获取实际位置
        if (get_group_position(backRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    return true;
}

/**【这个函数用在正常的控制阶段,不是复位阶段!!!!】
 * fun：焊架上下锁紧运动控制：
 * 位置（1）：向下运动锁紧位置:-->WeldDeskDownPos
 * 位置（2）：向上运动至零点位置(松开位置):-->WeldDeskUpPos
 */
// 焊架上下寻零 限位开关在最上侧 向上增加 向下减少
bool YinwMotion::WeldingDeskUpDownLock(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    const char *backRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(backRobot);
    int WeldingDeskLockAIMPOS;
    int UpDownLockPos = abs(deviceParas.Paras.weldingfixture_.WeldingframelockingPos);
    switch (YWzeroNameANDpos[6][0])
    {
    case 1: // 最高位置为0点位置
    {
        if (strcmp(CmdName, "WeldDeskDownPos") == 0)
        {
            WeldingDeskLockAIMPOS = YWzeroNameANDpos[6][1] - UpDownLockPos;
        }
        else if (strcmp(CmdName, "WeldDeskUpPos") == 0)
        {
            WeldingDeskLockAIMPOS = YWzeroNameANDpos[6][1];
        }
        LOG(INFO) << "WeldingDeskUpDownLock ZeroPosPoint is UP! WeldingDeskLockAIMPOS = " << WeldingDeskLockAIMPOS << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "WeldingDeskUpDownLock, Unknown ZeroPosPoint state!" << std::endl;
        break;
    }
    }

    int16_t WeldingDeskUpDownLockRet = YinwPosMotorControl(backRobot, DofNUM, WeldingShelfUpDownMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                           deviceParas.Paras.Generalparams_.MinMovePos, WeldingDeskLockAIMPOS,
                                                           deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelD,
                                                           deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelD);
    if (WeldingDeskUpDownLockRet == 1)
    {
        LOG(INFO) << "WeldingDeskUpDownLock finish!" << std::endl;
        return true;
    }

    int MotorCurrentPos[DofNUM];
    // 获取实际位置
    if (get_group_position(backRobot, MotorCurrentPos) != 0)
    {
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    // 打印实际位置
    // for (size_t i = 0; i < DofNUM; i++)
    //     LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;

    return false;
}

/**【这个函数用在正常的控制阶段,不是复位阶段!!!!】
 * fun：焊架锁紧机构左右运动控制：
 * 位置（1）：向右运动至锁紧位置:-->WeldRightLockPos
 * 位置（2）：向左运动至零点(放开)位置:-->WeldLeftOpenPos
 */
// 焊架左右运动寻零  向左减小 向右增加 限位开关在最左侧
bool YinwMotion::WeldingDeskLeftRightRun(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    const char *backRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(backRobot);
    int WeldingDeskLeftRightAIMPOS;
    int LeftRigtLockPos = abs(deviceParas.Paras.weldingfixture_.WeldingframetranslatePos);
    switch (YWzeroNameANDpos[7][0])
    {
    case 1: // 最左侧为零点位置
    {
        if (strcmp(CmdName, "WeldRightLockPos") == 0)
        {
            WeldingDeskLeftRightAIMPOS = YWzeroNameANDpos[7][1] + LeftRigtLockPos;
        }
        else if (strcmp(CmdName, "WeldLeftOpenPos") == 0)
        {
            WeldingDeskLeftRightAIMPOS = YWzeroNameANDpos[7][1];
        }
        LOG(INFO) << "WeldingDeskLeftRightRun ZeroPosPoint is UP! WeldingDeskLeftRightAIMPOS = " << WeldingDeskLeftRightAIMPOS << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "WeldingDeskLeftRightRun, Unknown ZeroPosPoint state!" << std::endl;
        break;
    }
    }

    int16_t WeldingDeskLeftRightRunRet = YinwPosMotorControl(backRobot, DofNUM, WeldingShelfLeftRightMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                             deviceParas.Paras.Generalparams_.MinMovePos, WeldingDeskLeftRightAIMPOS,
                                                             deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelD,
                                                             deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelD);
    if (WeldingDeskLeftRightRunRet == 1)
    {
        LOG(INFO) << "WeldingDeskLeftRightRunRet finish!" << std::endl;
        return true;
    }

    int MotorCurrentPos[DofNUM];
    // 获取实际位置
    if (get_group_position(backRobot, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    // 打印实际位置
    for (size_t i = 0; i < DofNUM; i++)
        LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    // 跳出判断
    if (abs(MotorCurrentPos[WeldingShelfLeftRightMotor - 1] - WeldingDeskLeftRightAIMPOS) <=
        deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA)
    {
        LOG(INFO) << "ThreeAxisZmotor Arrive!" << endl;
        return true;
    }
    return false;
}