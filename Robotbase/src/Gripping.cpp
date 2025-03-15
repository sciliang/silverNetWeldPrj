#include "YinwMotionGlobal.h"
/**
 * fun：银网的抓取和放下
 * 位置（1）：夹爪打开 ClampOpenPos
 * 位置（2）：夹爪关闭 ClampClosePos
 */
bool YinwMotion::SliverNetClamp(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    static short caculateFlag;
    std::int32_t ClampUpActualPos; // 用于存储所有的decimal值
    uint8_t ThCANid = 5;
    uint32_t SliverNetClampUpPos[1];
    int PuchAIMPOS;
    if (strcmp(CmdName, "ClampOpenPos") == 0)
    {
        LOG(INFO) << "SliverNetClamp AIMPOS = " << deviceParas.Paras.ClampAimPos_.ClampOpenPos << std::endl;
        PuchAIMPOS = abs(deviceParas.Paras.ClampAimPos_.ClampOpenPos);
    }
    if (strcmp(CmdName, "ClampClosePos") == 0)
    {
        LOG(INFO) << "SliverNetClamp AIMPOS = " << deviceParas.Paras.ClampAimPos_.ClampClosePos << std::endl;
        PuchAIMPOS = abs(deviceParas.Paras.ClampAimPos_.ClampClosePos);
    }

    SliverNetClampUpPos[0] = static_cast<uint32_t>(PuchAIMPOS);
    uint8_t canidlist_2[1] = {ThCANid};
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 5号电机运动
    ThSetTargetPos(canidlist_2, SliverNetClampUpPos);
    ClampUpActualPos = THgetActualAxisPos(ThCANid);
    if (abs(PuchAIMPOS - ClampUpActualPos) < 1000)
    {
        LOG(INFO) << "SliverNetClamp Arrive the aim pos." << std::endl;
        return true;
    }
    else
    {
        LOG(INFO) << "SliverNetClampUp is running..." << std::endl;
        return false;
    }
}

/**
 * fun：银网的三坐标和夹爪抓取
 */
bool YinwMotion::SliverNetPickAndClampClose(const YinwDeviceParas &deviceParas)
{
    switch (NetPickAndClampCloseStatusflow)
    {
    case PickStepOfClampDown:
    {
        if (SliverNetPick(deviceParas))
        {
            return true;
            // NetPickAndClampCloseStatusflow = ClampCloseStep;//夹爪闭合目前取消了
            LOG(INFO) << "SliverNetPick is finish." << std::endl;
        }
        else
        {
            LOG(INFO) << "SliverNetPick is running....." << std::endl;
        }
        break;
    }
    case ClampCloseStep:
    {
        if (SliverNetClamp(deviceParas, "ClampClosePos"))
        {
            LOG(INFO) << "ClampClosePos finish!" << std::endl;
            return true;
        }
        else
        {
            LOG(INFO) << "ClampClosePos runing..." << std::endl;
        }
        break;
    }
    default:
    {
        LOG(INFO) << "PlatePickAndClampStatusflow unkonwn Status!" << std::endl;
        break;
    }
    }
    return false;
}

/**
 * fun：运输银网至焊台和夹爪放开
 */
bool YinwMotion::SliverNetPlaceAndClampOpen(const YinwDeviceParas &deviceParas)
{
    switch (NetPlaceAndClampOpenStatusflow)
    {
    case PlaceStepOfClampDown:
    {
        if (SliverNetPlace(deviceParas))
        {
            // NetPlaceAndClampOpenStatusflow = ClampOpenStep;//夹爪出了问题，目前把夹爪略过了
            NetPlaceAndClampOpenStatusflow = PlaceStepOfClampDownZaxisUp;
            LOG(INFO) << "SliverNetPlace is finish." << std::endl;
        }
        else
        {
            LOG(INFO) << "SliverNetPlace is running....." << std::endl;
        }
        break;
    }
    case ClampOpenStep:
    {
        if (SliverNetClamp(deviceParas, "ClampOpenPos"))
        {
            NetPlaceAndClampOpenStatusflow = PlaceStepOfClampDownZaxisUp;
            LOG(INFO) << "ClampOpenPos finish!" << std::endl;
        }
        else
        {
            LOG(INFO) << "ClampOpenPos runing..." << std::endl;
        }
        break;
    }
    case PlaceStepOfClampDownZaxisUp:
    {
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    }
    default:
    {
        LOG(INFO) << "ERROR, NetPlaceAndClampOpenStatusflow unkonwn Status!" << std::endl;
        break;
    }
    }
    return false;
}