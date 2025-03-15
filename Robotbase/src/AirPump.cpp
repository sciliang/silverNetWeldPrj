#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;
/**
 * fun：极片的吸取和放下
 */
bool YinwMotion::polePlateSuckControl(const YinwDeviceParas &deviceParas, const char *CmdName)
{
    LOG(INFO) << "polePlateSuck succeed!" << std::endl;
    IODeviceTeleControl(CmdName);
    return true;
}

/**
 * fun：三坐标和气泵吸取极片
 */
bool YinwMotion::polePlatePickAndSuckUp(const YinwDeviceParas &deviceParas)
{
    std::array<bool, 16> GET_IOstatus = {false}, GET_IOstatus_SAVE = {false};
    LOG(INFO) << "PlatePickAndSuckUpStatusflow = " << PlatePickAndSuckUpStatusflow << std::endl;
    switch (PlatePickAndSuckUpStatusflow)
    {
    case PickStepOfsuckUp:
    {
        if (polePlatePick(deviceParas))
        {
            LOG(INFO) << "polePlatePick succeed!!" << std::endl;
            PlatePickAndSuckUpStatusflow = PickSuckUpStep;
        }
        break;
    }
    case PickSuckUpStep:
    {
        polePlateSuckControl(deviceParas, "Airpumpon");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        PlatePickAndSuckUpStatusflow = PickSuckPlateRoom;
        break;
    }
    case PickSuckPlateRoom:
    {
        LOG(INFO) << "StepMotorsWatchDog = " << StepMotorsWatchDog << std::endl;
        StepMotorsWatchDog++;
        if (StepMotorsWatchDog == StepMotorsRecvDog)
        {
            IO_X_PinsCmdSEND("IOsecond");
            LOG(INFO) << "IO_X_PinsCmdSEND 11 !" << std::endl;
        }
        else if (StepMotorsWatchDog == StepMotorsDog)
        {
            StepMotorsWatchDog = 0;
            GET_IOstatus = IO_X_PinsCmdGET();
            LOG(INFO) << "22 GET_IOstatusRET= [";
            for (size_t i = 0; i < GET_IOstatus.size(); i++)
                LOG(INFO) << GET_IOstatus[i];
            LOG(INFO) << "]" << std::endl;
            GET_IOstatus[1] = 1;
            if (GET_IOstatus[1])
            {
                LOG(INFO) << "Airpumb limit switch arrive!" << std::endl;
                return true;
            }
            else
            {
                if (polePlateRoomStepping(deviceParas))
                {
                    ControlStepflow = ThirtiethStep;
                    LOG(INFO) << "polePlateRoom Step Run Over!!!" << std::endl;
                }
                else
                    LOG(INFO) << "polePlateRoom Running Now..." << std::endl;
            }
        }
        break;
    }
    default:
    {
        LOG(INFO) << "polePlatePickAndSuckUp Unknown status!" << std::endl;
        break;
    }
    }
    return false;
}

/**
 * fun：三坐标和气泵吸取极片
 */
bool YinwMotion::polePlatePickAndSuckUpThread(const YinwDeviceParas &deviceParas)
{
    std::array<bool, 16> GET_IOstatus = {false}, GET_IOstatus_SAVE = {false};
    LOG(INFO) << "PlatePickAndSuckUpStatusflow = " << PlatePickAndSuckUpStatusflow << std::endl;
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 8;
    RTimer_.cycle_times = 1;
#endif
    while (SuckUpThreadisRunning)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        switch (PlatePickAndSuckUpStatusflow)
        {
        case PickStepOfsuckUp:
        {
            if (polePlatePick(deviceParas))
            {
                LOG(INFO) << "polePlatePick succeed!!" << std::endl;
                PlatePickAndSuckUpStatusflow = PickSuckUpStep;
            }
            break;
        }
        case PickSuckUpStep:
        {
            polePlateSuckControl(deviceParas, "Airpumpon");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            PlatePickAndSuckUpStatusflow = PickSuckPlateRoom;
            break;
        }
        case PickSuckPlateRoom:
        {
            // LOG(INFO) << "StepMotorsWatchDog = " << StepMotorsWatchDog << std::endl;
            StepMotorsWatchDog++;
            if (StepMotorsWatchDog == StepMotorsRecvDog)
            {
                IO_X_PinsCmdSEND("IOsecond");
                // LOG(INFO) << "IO_X_PinsCmdSEND 11 !" << std::endl;
            }
            else if (StepMotorsWatchDog == StepMotorsDog)
            {
                StepMotorsWatchDog = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                LOG(INFO) << "22 GET_IOstatusRET= [";
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << "]" << std::endl;
                GET_IOstatus[1] = 1;
                if (GET_IOstatus[1])
                {
                    LOG(INFO) << "Airpumb limit switch arrive!" << std::endl;
                    PlatePickAndSuckUpStatusflow = PickTransPlate1;
                    // return true;
                }
                else
                {
                    if (polePlateRoomStepping(deviceParas))
                    {
                        // ControlStepflow = ThirtiethStep;
                        LOG(INFO) << "polePlateRoom Step Run Over!!!" << std::endl;
                    }
                    else
                        LOG(INFO) << "polePlateRoom Running Now..." << std::endl;
                }
            }
            break;
        }
        case PickTransPlate1:
        {
            if (polePlatePlace(deviceParas))
            {
                PlatePickAndSuckUpStatusflow = PickTransPlate2;
            }
            break;
        }
        case PickTransPlate2:
        {
            polePlateSuckControl(deviceParas, "Airpumpoff");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            for (size_t i = 0; i < 2; i++)
            {
                LOG(INFO) << "SuckUpThreadisRunning = " << SuckUpThreadisRunning << endl;
                LOG(INFO) << "!!!!!!!!!!!!!!!!!!SuckUpThreadisRunning EXIT succeed!!!!!!!!!!!!!!!!!!" << SuckUpThreadisRunning << endl;
                SuckUpThreadisRunning.store(false);
            }
            return true;
            break;
        }
        default:
        {
            LOG(INFO) << "polePlatePickAndSuckUp Unknown status!" << std::endl;
            break;
        }
        }
    }
    return false;
}

/**
 * fun：三坐标和气泵放置极片到焊台
 */
bool YinwMotion::polePlatePickAndSuckDown(const YinwDeviceParas &deviceParas)
{
    switch (PlatePickAndSuckDownStatusflow)
    {
    case PickStepOfsuckDown:
    {
        if (polePlatePlace(deviceParas))
        {
            PlatePickAndSuckDownStatusflow = SuckDownStep;
        }
        break;
    }
    case SuckDownStep:
    {
        polePlateSuckControl(deviceParas, "Airpumpoff");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        return true;
        break;
    }
    default:
    {
        LOG(INFO) << "polePlatePickAndSuckUp Unknown status!" << std::endl;
        break;
    }
    }
    return false;
}