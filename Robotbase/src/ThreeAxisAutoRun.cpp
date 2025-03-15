#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;
/**
 * fun：极片运输：放极片
 */
bool YinwMotion::polePlatePlace(const YinwDeviceParas &deviceParas)
{
    // LOG(INFO) << "TransSheet2WeldDeskX = " << deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskX << std::endl;
    // LOG(INFO) << "TransSheet2WeldDeskY = " << deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskY << std::endl;
    // LOG(INFO) << "TransSheet2WeldDeskZ = " << deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskZ << std::endl;
    // LOG(INFO) << "ZaxisHighest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHighest << std::endl;
    // LOG(INFO) << "ZaxisHiglowest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHiglowest << std::endl;
    switch (polePlatePlaceflow)
    {
    case polePlatePlacefirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            polePlatePlaceflow = polePlatePlacesecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is runing" << endl;
        break;
    case polePlatePlacesecond:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskX))
            polePlatePlaceflow = polePlatePlaceThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is runing" << endl;
        break;
    case polePlatePlaceThird:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskY))
            polePlatePlaceflow = polePlatePlaceFourth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is runing" << endl;
        break;
    case polePlatePlaceFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.TransSheet2WeldDeskZ))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is runing" << endl;
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：极片运输：取极片
 */
bool YinwMotion::polePlatePick(const YinwDeviceParas &deviceParas)
{
    switch (polePlatePickAndSuckUpflow)
    {
    case polePlatePickAndSuckUpfirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            polePlatePickAndSuckUpflow = polePlatePickAndSuckUpsecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl running" << endl;
        break;
    case polePlatePickAndSuckUpsecond:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpPolePlateY))
            polePlatePickAndSuckUpflow = polePlatePickAndSuckUpThird;
        else
            LOG(INFO) << "ThreeAxisY_ctrl running" << endl;
        break;
    case polePlatePickAndSuckUpThird:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpPolePlateX))
            polePlatePickAndSuckUpflow = polePlatePickAndSuckUpFourth;
        else
            LOG(INFO) << "ThreeAxisX_ctrl running" << endl;
        break;
    case polePlatePickAndSuckUpFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpPolePlateZ))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl running" << endl;
        break;
    default:
        LOG(INFO) << "ERROR!!!, polePlatePickAndSuckUpflow UNkonwn Status!" << endl;
        break;
    }
    return false;
}

/**
 * fun：拾取银网
 */
bool YinwMotion::SliverNetPick(const YinwDeviceParas &deviceParas)
{
    // LOG(INFO) << "PickUpSilverNetX = " << deviceParas.Paras.ThreeDimensional_.PickUpSilverNetX << std::endl;
    // LOG(INFO) << "PickUpSilverNetY = " << deviceParas.Paras.ThreeDimensional_.PickUpSilverNetY << std::endl;
    // LOG(INFO) << "PickUpSilverNetZ = " << deviceParas.Paras.ThreeDimensional_.PickUpSilverNetZ << std::endl;
    // LOG(INFO) << "ZaxisHighest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHighest << std::endl;
    // LOG(INFO) << "ZaxisHiglowest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHiglowest << std::endl;
    switch (SliverNetPickflow)
    {
    case SliverNetPickfirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            SliverNetPickflow = SliverNetPicksecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is runing!!!" << endl;
        break;
    case SliverNetPicksecond:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpSilverNetX))
            SliverNetPickflow = SliverNetPickThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is runing!!!" << endl;
        break;
    case SliverNetPickThird:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpSilverNetY))
            SliverNetPickflow = SliverNetPickFourth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is runing!!!" << endl;
        break;
    case SliverNetPickFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.PickUpSilverNetZ))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is runing!!!" << endl;
        break;
    default:
        LOG(INFO) << "ERROR!!! SliverNetPickflow unKnown status!!!" << endl;
        break;
    }
    return false;
}

/**
 * fun：安置银网
 */
bool YinwMotion::SliverNetPlace(const YinwDeviceParas &deviceParas)
{
    // LOG(INFO) << "TransSilverNet2WeldDeskX = " << deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskX << std::endl;
    // LOG(INFO) << "TransSilverNet2WeldDeskY = " << deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskY << std::endl;
    // LOG(INFO) << "TransSilverNet2WeldDeskZ = " << deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskZ << std::endl;
    // LOG(INFO) << "ZaxisHighest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHighest << std::endl;
    // LOG(INFO) << "ZaxisHiglowest = " << deviceParas.Paras.ThreeDimensional_.ZaxisHiglowest << std::endl;
    switch (SliverNetPlaceflow)
    {
    case SliverNetPlacefirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            SliverNetPlaceflow = SliverNetPlacesecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case SliverNetPlacesecond:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskX))
            SliverNetPlaceflow = SliverNetPlaceThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
        break;
    case SliverNetPlaceThird:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskY))
            SliverNetPlaceflow = SliverNetPlaceFourth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case SliverNetPlaceFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.TransSilverNet2WeldDeskZ))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    default:
        LOG(INFO) << "ERROR, SliverNetPlaceflow unkonwn status!" << endl;
        break;
    }
    return false;
}

/**
 * fun：取成品或者废品
 */
bool YinwMotion::ProductPickUp(const YinwDeviceParas &deviceParas)
{
    switch (ProductPickUpflow)
    {
    case ProductPickUpfirst: // Z轴升到最高
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            ProductPickUpflow = ProductPickUpsecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case ProductPickUpsecond: // 准备到放置废品的位置(废品库):
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductY))
            ProductPickUpflow = ProductPickUpThird;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case ProductPickUpThird:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductX))
            ProductPickUpflow = ProductPickUpFourth;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
        break;
    case ProductPickUpFourth:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PickupProductY))
            ProductPickUpflow = ProductPickUpFifth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case ProductPickUpFifth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.PickupProductZ))
            ProductPickUpflow = ProductPickUpSixth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case ProductPickUpSixth:
        polePlateSuckControl(deviceParas, "Airpumpon");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ProductPickUpflow = ProductPickUpSeventh;
        break;
    case ProductPickUpSeventh: // Z轴升到最高
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：开辟一个新的线程，进行三坐标至取产品位置等待
 */
bool YinwMotion::ProductPickUpWait(const YinwDeviceParas &deviceParas)
{
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 7;
    RTimer_.cycle_times = 1;
#endif
    while (ProductWaitThreadisRunning)
    {
        LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        LOG(INFO) << "#### ---ProductWaitThread---  isRunning is running!!####" << endl;
        switch (ProductPickUpflow)
        {
        case ProductPickUpfirst: // Z轴升到最高
            if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
                ProductPickUpflow = ProductPickUpsecond;
            else
                LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
            break;
        case ProductPickUpsecond: // 准备到放置废品的位置(废品库):
            if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductY))
                ProductPickUpflow = ProductPickUpThird;
            else
                LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
            break;
        case ProductPickUpThird:
            if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductX))
                ProductPickUpflow = ProductPickUpFourth;
            else
                LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
            break;
        case ProductPickUpFourth:
            if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PickupProductY))
            {
                for (size_t i = 0; i < 5; i++)
                {
                    LOG(INFO) << "!!!!!!!!!!!!!!!!!!ProductWaitThreadisRunning EXIT succeed!!!!!!!!!!!!!!!!!!!!" << endl;
                    ProductWaitThreadisRunning.store(false); // 启动时设置为 true
                }
                return true;
            }
            else
                LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
            break;
        default:
            break;
        }
    }
    return false;
}

/**
 * fun：废弃堆码
 */
bool YinwMotion::WasteStacking(const YinwDeviceParas &deviceParas)
{
    switch (WasteStackingflow)
    {
    case WasteStackingfirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            WasteStackingflow = WasteStackingsecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case WasteStackingsecond:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductY))
            WasteStackingflow = WasteStackingThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
        break;
    case WasteStackingThird:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductX))
            WasteStackingflow = WasteStackingFourth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case WasteStackingFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductZ))
            WasteStackingflow = WasteStackingFifth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case WasteStackingFifth:
        polePlateSuckControl(deviceParas, "Airpumpoff");
        WasteStackingflow = WasteStackingSixth;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        break;
    case WasteStackingSixth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：合格堆码
 */
bool YinwMotion::QualifiedStacking(const YinwDeviceParas &deviceParas)
{
    switch (QualifiedStackingflow)
    {
    case QualifiedStackingfirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            QualifiedStackingflow = QualifiedStackingsecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case QualifiedStackingsecond:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceGoodProductY))
            QualifiedStackingflow = QualifiedStackingThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
        break;
    case QualifiedStackingThird:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceGoodProductX))
            QualifiedStackingflow = QualifiedStackingFourth;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    case QualifiedStackingFourth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceGoodProductZ))
            QualifiedStackingflow = QualifiedStackingFifth;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case QualifiedStackingFifth:
        polePlateSuckControl(deviceParas, "Airpumpoff");
        QualifiedStackingflow = QualifiedStackingSixth;
        std::this_thread::sleep_for(std::chrono::seconds(3));
    case QualifiedStackingSixth:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            return true;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：三坐标复位控制,目前准备复位至极片库的正上方
 */
bool YinwMotion::ThreeAxisResetRun(const YinwDeviceParas &deviceParas)
{
    switch (ThreeAxisResetflow)
    {
    case ThreeAxisResetfirst:
        if (ThreeAxisZ_ctrl(deviceParas.Paras.ThreeDimensional_.ZaxisHighest))
            ThreeAxisResetflow = WasteStackingsecond;
        else
            LOG(INFO) << "ThreeAxisZ_ctrl is running!!" << endl;
        break;
    case ThreeAxisResetsecond:
        if (ThreeAxisY_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductY))
            ThreeAxisResetflow = WasteStackingThird;
        else
            LOG(INFO) << "ThreeAxisX_ctrl is running!!" << endl;
        break;
    case ThreeAxisResetThird:
        if (ThreeAxisX_ctrl(deviceParas.Paras.ThreeDimensional_.PlaceBadProductX))
            return true;
        else
            LOG(INFO) << "ThreeAxisY_ctrl is running!!" << endl;
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：三坐标X轴控制
 * */
bool YinwMotion::ThreeAxisX_ctrl(int X_AimPos)
{
    // LOG(INFO) << "YinW ThreeAxisXmotor" << endl;
    int ThreeAxisXAIMPOS = X_AimPos;
    short ThreeAxisXdataMoveS = 400;
    short ThreeAxisXArrivethreshold = 100;
    const char *beforerobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(beforerobot);
    int MotorCurrentPos[DofNUM];
    // YinwPosMotorControl_New
    int16_t ThreeAxisXret = YinwPosMotorControl_New(beforerobot, DofNUM, ThreeAxisXmotor,
                                                    deviceParas.Paras.ThreeDimensional_.XaxisHighest, 0,
                                                    deviceParas.Paras.ThreeDimensional_.XaxisHiglowest,
                                                    ThreeAxisXAIMPOS, ThreeAxisXdataMoveS, ThreeAxisXdataMoveS);
    if (ThreeAxisXret == 1)
    {
        LOG(INFO) << "ThreeAxisX_ctrl Arrive !!!" << std::endl;
        return true;
    }

    if (get_group_position(beforerobot, MotorCurrentPos) != 0)
    {
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    // for (size_t i = 0; i < DofNUM; i++)
    // {
    //     LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    // }
    // LOG(INFO) << "ThreeAxisXmotor POS =" << MotorCurrentPos[ThreeAxisXmotor - 1] << std::endl;
    // LOG(INFO) << "ThreeAxisXArrivethreshold =" << ThreeAxisXArrivethreshold << std::endl;
    return false;
}

/**
 * fun：三坐标Y轴控制
 */
bool YinwMotion::ThreeAxisY_ctrl(int Y_AimPos)
{
    int ThreeAxisYAIMPOS = Y_AimPos;
    short ThreeAxisYdataMoveS = 400;
    short ThreeAxisYArrivethreshold = 60;
    const char *beforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(beforeRobot);
    // YinwPosMotorControl_New
    int16_t ThreeAxisYret = YinwPosMotorControl_New(beforeRobot, DofNUM, ThreeAxisYmotor,
                                                    deviceParas.Paras.ThreeDimensional_.YaxisHighest, 0,
                                                    deviceParas.Paras.ThreeDimensional_.YaxisHiglowest,
                                                    ThreeAxisYAIMPOS, ThreeAxisYdataMoveS, ThreeAxisYdataMoveS);
    if (ThreeAxisYret == 1)
    {
        LOG(INFO) << "ThreeAxisY_ctrl Arrive !!!" << std::endl;
        return true;
    }
    int MotorCurrentPos[DofNUM];
    if (get_group_position(beforeRobot, MotorCurrentPos) != 0)
    {
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    return false;
}

/**
 * fun：三坐标Z轴控制
 */
bool YinwMotion::ThreeAxisZ_ctrl(int Z_AimPos)
{
    // LOG(INFO) << "YinW ThreeAxisZmotor" << endl;
    int ThreeAxisZAIMPOS = Z_AimPos;
    short ThreeAxisZdataMoveS = 500; // 100
    short ThreeAxisZArrivethreshold = 50;
    const char *backrobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(backrobot);
    // YinwPosMotorControl_New
    int16_t ThreeAxisZret = YinwPosMotorControl_New(backrobot, DofNUM, ThreeAxisZmotor,
                                                    deviceParas.Paras.ThreeDimensional_.ZaxisHighest, 0,
                                                    deviceParas.Paras.ThreeDimensional_.ZaxisHiglowest,
                                                    ThreeAxisZAIMPOS, ThreeAxisZdataMoveS, ThreeAxisZdataMoveS);
    if (ThreeAxisZret == 1)
    {
        LOG(INFO) << "ThreeAxisZ_ctrl Arrive !!!" << std::endl;
        return true;
    }

    int MotorCurrentPos[DofNUM];
    if (get_group_position(backrobot, MotorCurrentPos) != 0)
    {
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    return false;
}

/**
 * fun：对控制流的数据进行暂存
 */
void YinwMotion::ControlStepflowSAVE(short ControlStepValue)
{
    // 关机、控制器重启、设备急停、设备复位、设备开灯、设备关灯、设备暂停、设备继续
    // 设备上电、系统状态恢复、IO反馈读取、设备下电、控制器重启、参数存储
    if ((ControlStepValue != ThirtyFifthStep) && (ControlStepValue != FiftySeventhStep) &&
        (ControlStepValue != ThirtySixthStep) && (ControlStepValue != ThirtySeventhStep) &&
        (ControlStepValue != FortyFirstStep) && (ControlStepValue != FortySecondStep) &&
        (ControlStepValue != ThirtyEighthStep) && (ControlStepValue != ThirtyNineStep) &&
        (ControlStepValue != ThirtyThirdStep) && (ControlStepValue != ThirtySecondStep) &&
        (ControlStepValue != ThirtyFourthStep) && (ControlStepValue != FiftySeventhStep) &&
        (ControlStepValue != FiftySecondStep))
    {
        if (!ControlStepflowSave.empty())
        {
            int lastStepflowElement = ControlStepflowSave[ControlStepflowSave.size() - 1];
            LOG(INFO) << "Latest Element = " << lastStepflowElement << std::endl;
            if (lastStepflowElement != ControlStepValue)
            {
                LOG(INFO) << "ControlStepflowSave  NoEmpty." << std::endl;
                ControlStepflowSave.push_back(ControlStepValue); // 插入新的元素
            }
        }
        else
        {
            LOG(INFO) << "ControlStepflowSave  empty." << std::endl;
            ControlStepflowSave.push_back(ControlStepValue); // 插入新的元素
        }
    }
    // 删除最旧的元素
    if (ControlStepflowSave.size() > ControlStepflowMaxSize)
    {
        // LOG(INFO) << "ControlStepflowSaveSize  Overlimit !" << std::endl;
        ControlStepflowSave.erase(ControlStepflowSave.begin());
    }
    LOG(INFO) << "ControlStepflowSave Data:";
    for (short value : ControlStepflowSave)
    {
        LOG(INFO) << value << " ";
    }
    LOG(INFO) << std::endl;
}

/**
 * fun：对控制流的标志位进行重置
 */
void YinwMotion::ControlStepflowReset()
{
    LOG(INFO) << "ControlStepflow Reset!!" << std::endl;
    ControlStepflow = -1;
}
