#include "YinwMotionGlobal.h"

/**
 * fun：4. 银网压制位运动
 * 位置(1):银网压制位 WeldingPressPos
 * 位置(2):银网运输位置 WeldingTransNetPos
 */
bool YinwMotion::WeldingPressMove(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    /* 控制3号电机运行 银网压片位复位 准备进行银网的冲制*/
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint8_t canidlist_3[1] = {3};
    int32_t PosActualVal;
    uint32_t ThirdTargetFinalPos[1];
    int WeldingPressMoveAimPos;
    if (strcmp(CmdName, "WeldingPressPos") == 0)
    {
        WeldingPressMoveAimPos = deviceParas.Paras.SilverNetPress_.SilverNetPressPos;
    }
    else if (strcmp(CmdName, "WeldingTransNetPos") == 0)
    {
        WeldingPressMoveAimPos = deviceParas.Paras.SilverNetPress_.SilverNetTransPos;
    }
    LOG(INFO) << "WeldingPressMoveAimPos = " << WeldingPressMoveAimPos << std::endl;
    ThirdTargetFinalPos[0] = static_cast<uint32_t>(WeldingPressMoveAimPos);
    ThSetTargetPos(canidlist_3, ThirdTargetFinalPos);
    PosActualVal = THgetActualAxisPos(3); // 一个字节指令的下发，如果两次下发之间间隔时间过短，可能会有问题
    LOG(INFO) << "PosActualVal = " << PosActualVal << std::endl;
    if (abs(PosActualVal - static_cast<int32_t>(ThirdTargetFinalPos[0])) < 1000)
    {
        return true;
    }
    return false;
}

/**
 * fun：4.4. 银网压制位复位运动
 * 位置(1):银网压制位 WeldingPressPos
 * 位置(2):银网运输位置 WeldingTransNetPos
 */
bool YinwMotion::WeldingPressMoveReset(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    short TargetSendNum = 0;
    while (WeldingPressResetThreadisRunning)
    {
        LOG(INFO) << "#### ---WeldingPressReset--  ThreadisRunning is running!!####" << endl;
        /* 控制3号电机运行 银网压片位复位 准备进行银网的冲制*/
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint8_t canidlist_3[1] = {3};
        int32_t PosActualVal;
        uint32_t ThirdTargetFinalPos[1];
        int WeldingPressMoveAimPos;
        if (strcmp(CmdName, "WeldingPressPos") == 0)
        {
            WeldingPressMoveAimPos = deviceParas.Paras.SilverNetPress_.SilverNetPressPos;
        }
        else if (strcmp(CmdName, "WeldingTransNetPos") == 0)
        {
            WeldingPressMoveAimPos = deviceParas.Paras.SilverNetPress_.SilverNetTransPos;
        }
        LOG(INFO) << "WeldingPressMoveAimPos = " << WeldingPressMoveAimPos << std::endl;
        ThirdTargetFinalPos[0] = static_cast<uint32_t>(WeldingPressMoveAimPos);
        if (TargetSendNum++ <= 3) // 只下发三次，后面的就不发了，放置造成干扰
        {
            ThSetTargetPos(canidlist_3, ThirdTargetFinalPos);
        }
        PosActualVal = THgetActualAxisPos(3); // 一个字节指令的下发，如果两次下发之间间隔时间过短，可能会有问题
        LOG(INFO) << "PosActualVal = " << PosActualVal << std::endl;
        if (abs(PosActualVal - static_cast<int32_t>(ThirdTargetFinalPos[0])) < 1000)
        {
            for (size_t i = 0; i < 2; i++)
            {
                TargetSendNum = 0;
                LOG(INFO) << "WeldingPressResetThreadisRunning = " << WeldingPressResetThreadisRunning << std::endl;
                LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!WeldingPressResetThreadisRunning EXIT succeed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                WeldingPressResetThreadisRunning.store(false); // 保证线程退出
            }
            break;
        }
        LOG(INFO) << "FifthStep Run succeed." << std::endl;
    }
    return true;
}