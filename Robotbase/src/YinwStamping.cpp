#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;
/**【这个函数用在正常的控制阶段,不是复位阶段!!!!】
 * fun：银网冲制
 * 位置（1）：向上抬起停留位置:-->PunchUpWaitPos
 * 位置（2）：向下进行压片位置:-->PunchDownPressPos
 */
// 冲压向上寻零 向下为正 向上为负
bool YinwMotion::silverNetPunching(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int dataZero_UpWaitPos = abs(deviceParas.Paras.Stamping_.ChongYaUpWaitPos);
    int dataZero_DownPressPos = abs(deviceParas.Paras.Stamping_.ChongYaDownPressPos);
    int PuchAIMPOS;
    switch (YWzeroNameANDpos[2][0])
    {
    case 1: // 最高限位为零位
    {
        if (strcmp(CmdName, "PunchUpWaitPos") == 0)
        {
            PuchAIMPOS = YWzeroNameANDpos[2][1] + dataZero_UpWaitPos;
        }
        if (strcmp(CmdName, "PunchDownPressPos") == 0)
        {
            PuchAIMPOS = YWzeroNameANDpos[2][1] + dataZero_DownPressPos;
        }
        LOG(INFO) << "silverNetPunching ZeroPosPoint is UP! PuchAIMPOS = " << PuchAIMPOS << std::endl;
        break;
    }
    case 2: // 最低限位为零位
    {
        if (strcmp(CmdName, "PunchUpWaitPos") == 0)
        {
            PuchAIMPOS = YWzeroNameANDpos[2][1] - dataZero_UpWaitPos;
        }
        if (strcmp(CmdName, "PunchDownPressPos") == 0)
        {
            PuchAIMPOS = YWzeroNameANDpos[2][1] - dataZero_DownPressPos;
        }
        LOG(INFO) << "silverNetPunching ZeroPosPoint is DOWN! PuchAIMPOS = " << PuchAIMPOS << std::endl;
        break;
    }
    default:
    {
        LOG(INFO) << "silverNetPunching, Unknown ZeroPosPoint state!" << std::endl;
        break;
    }
    }
    // YinwPosMotorControl_New
    int16_t silverNetPunchingRet = YinwPosMotorControl_New(BeforeRobot, DofNUM, YinwForgingMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                                           deviceParas.Paras.Generalparams_.MinMovePos, PuchAIMPOS,
                                                           deviceParas.Paras.StampingErrorCompensation_.StampingVelD,
                                                           deviceParas.Paras.StampingErrorCompensation_.StampingVelD);
    if (silverNetPunchingRet == 1)
    {
        LOG(INFO) << "silverNetPunching finish!" << std::endl;
        return true;
    }

    int MotorCurrentPos[DofNUM];
    // 获取实际位置
    if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    // 打印实际位置
    // for (size_t i = 0; i < DofNUM; i++)
    //     LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    // 状态跳出
    if (abs(MotorCurrentPos[YinwForgingMotor - 1] - PuchAIMPOS) <=
        deviceParas.Paras.StampingErrorCompensation_.StampingVelA)
    {
        LOG(INFO) << "silverNetPunching Arrive!" << endl;
        return true;
    }
    return false;
}

bool YinwMotion::silverNetPunchGeneralFlow(const YinwDeviceParas &deviceParas)
{
    switch (YinWPunchGeneralFlow)
    {
    case FirstPunchStep:
    {
        if (silverNetPunching(deviceParas, "PunchDownPressPos"))
        {
            YinWPunchGeneralFlow = SecondPunchStep;
        }
        break;
    }
    case SecondPunchStep:
    {
        if (silverNetPunching(deviceParas, "PunchUpWaitPos"))
        {
            return true;
        }
        break;
    }
    default:
        break;
    }
    return false;
}