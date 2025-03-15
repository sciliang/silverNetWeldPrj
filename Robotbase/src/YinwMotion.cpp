#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;

/**
 *1号驱动器：信捷  冲压
  2号驱动器：迈信  Y轴移动
  3号驱动器：迈信  X轴移动
  4号驱动器：鸣志  展平机构移动
  5号驱动器：鸣志  极片库
  6号驱动器：鸣志  成品库
  7号驱动器：鸣志  废品库
  8号驱动器：迈信  Z轴移动
  9号驱动器：松下  焊接移动
  10号驱动器：钛虎
  11号驱动器：钛虎
  12号驱动器：钛虎
  13号驱动器：钛虎
 *
*/

// 初始化8x2的二维向量，所有值为0
YinwMotion::YinwMotion() : YWzeroNameANDpos(8, std::vector<int>(2, 0)),
                           YWzeroNameANDpos_INIsave(8, std::vector<int>(2, 0)),
                           DeviceControlMode(StepControlMode),
                           ControlStepflow(-1), // 控制流！！
                           DeviceRunStepsCalculate(0),
                           FlattnMoveStausStepflow(FourthTaiHuMotor),
                           YinWPunchGeneralFlow(FirstPunchStep),
                           PlatePickAndSuckUpStatusflow(PickStepOfsuckUp),
                           PlatePickAndSuckDownStatusflow(PickStepOfsuckDown),
                           NetPickAndClampCloseStatusflow(PickStepOfClampDown),
                           NetPlaceAndClampOpenStatusflow(PlaceStepOfClampDown),
                           FourthTaiHuMotorStepControlflow(FourthTaiHuMotorStepControlfirst),
                           SecondTaiHuMotorStepControlflow(SecondTaiHuMotorStepControlfirst),
                           FirstTaiHuMotorStepControlflow(FirstTaiHuMotorStepControlfirst),
                           polePlatePickAndSuckUpflow(polePlatePickAndSuckUpfirst),
                           polePlatePlaceflow(polePlatePlacefirst),
                           SliverNetPickflow(SliverNetPickfirst),
                           SliverNetPlaceflow(SliverNetPlacefirst),
                           ProductPickUpflow(ProductPickUpfirst),
                           WasteStackingflow(WasteStackingfirst),
                           QualifiedStackingflow(QualifiedStackingfirst),
                           ThreeAxisResetflow(ThreeAxisResetfirst),
                           GeneralResetflow(GeneralResetfirst),
                           OnlineInspectionRecvRet(false),
                           StepMotorsWatchDog(0),
                           StepMotorsDog(30),
                           StepMotorsRecvDog(10),
                           ControlStepflowMaxSize(4),
                           WasteHouseflow(WasteHousefirst),
                           GoodHouseflow(GoodHousefirst),
                           FlatteningMOVEflow(FlatteningMOVEfirst),
                           PolePlateHouseflow(PolePlateHousefirst),
                           SuckUpThreadisRunning(true),
                           ProductWaitThreadisRunning(true),
                           WeldingPressResetThreadisRunning(true),
                           WeldingDeskResetThreadisRunning(true),
                           ET_MotorsDofNUM(11),
                           hasSpeedFault(false),
                           hasPositionFault(false),
                           hasErrorCodeFault(false),
                           ResumeFlag(false)
{
    LOG(INFO) << "YinwMotion has built!" << std::endl;
    ControlStepflowSave.resize(4, -1);
    ET_MotorsCurSpeed = new int[ET_MotorsDofNUM](); // 已经初始化为0了
    // memset(RH_PRODUCTIONINFO_, 0, sizeof(RH_PRODUCTIONINFO_));
}

YinwMotion::~YinwMotion()
{
    // delete[] ET_MotorsCurSpeed; // 释放内存
    LOG(INFO) << "YinwMotion has been break down!!!" << std::endl;
}

// EtherCAT进行初始化 将8个驱动器全部初始化成位置模式
int YinwMotion::InitRescueEherCAT(short TorqueLimit)
{
    int robot_dofNUM[2]; // 只有一个机器人，在这里总共包含了8个自由度
    const char *before_Robot = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 0);
    const char *back_Robot = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 1);
    cout << "<---------------robotName------------->\n"
         << "before_Robot= " << before_Robot
         << "  back_Robot=" << back_Robot << endl;

    // 获取两个机器人的自由度
    robot_dofNUM[0] = get_group_dof(before_Robot);
    robot_dofNUM[1] = get_group_dof(back_Robot);
    cout << "before_RobotDOF_NUM = " << robot_dofNUM[0] << endl;
    cout << "back_RobotDOF_NUM = " << robot_dofNUM[1] << endl;

    // 第一组机器人：before_Robot
    for (int i = 1; i <= robot_dofNUM[0]; i++)
    {
        printf("128 i=%d\n", i);
        if ((set_axis_control(before_Robot, (unsigned short)128, i)) != 0)
        {
            cout << "Path:" << __FILE__ << endl
                 << "Line:" << __LINE__ << " "
                 << "set_axis_control Error! "
                 << endl;
        }
        // 设置电流限制
        // if (set_axis_torqueMaxLimit(ChassisRob, TorqueLimit, i) != 0)
        // {
        //     cout << "[__FILE__:__LINE__] set_axis_torqueMaxLimit failed! "
        //          << __FILE__ << __LINE__ << endl;
        // };
    }

    // 第二组机器人：back_Robot
    for (int i = 1; i <= robot_dofNUM[1]; i++)
    {
        printf("128 i=%d\n", i);
        if ((set_axis_control(back_Robot, (unsigned short)128, i)) != 0)
        {
            cout << "Path:" << __FILE__ << endl
                 << "Line:" << __LINE__ << " "
                 << "set_axis_control Error! "
                 << endl;
        }
        // 设置电流限制
        // if (set_axis_torqueMaxLimit(ChassisRob, TorqueLimit, i) != 0)
        // {
        //     cout << "[__FILE__:__LINE__] set_axis_torqueMaxLimit failed! "
        //          << __FILE__ << __LINE__ << endl;
        // };
    }

    // 现在三坐标Z轴没有抱闸，下电的话就会掉下来，因此，把断电重启关掉了
    // before_Robot机器人下电
    // if (0 != group_power_off(before_Robot))
    // {
    //     printf("[drive wheel]before_Robot group_power_on ERROR!\n");
    //     return -1;
    // }
    // // back_Robot机器人下电
    // if (0 != group_power_off(back_Robot))
    // {
    //     printf("[drive wheel]back_Robot group_power_on ERROR!\n");
    //     return -1;
    // }

    // before_Robot模式设置
    signed char before_Robot_mode[robot_dofNUM[0]] =
        {
            PositionMode, //
            PositionMode, //
            PositionMode, //
            PositionMode, //
            PositionMode, //
            PositionMode, //
        };
    // back_Robot模式设置
    signed char back_Robot_mode[robot_dofNUM[1]] =
        {
            PositionMode, //
            PositionMode, //
            PositionMode, //
            PositionMode, //
            PositionMode, //
        };

    // 查看模式设置的是否对
    for (int i = 0; i < robot_dofNUM[0]; i++)
    {
        printf("before_Robot_mode[%d] =%d\n", i, before_Robot_mode[i]);
    }
    // 查看模式设置的是否对
    for (int i = 0; i < robot_dofNUM[1]; i++)
    {
        printf("back_Robot_mode[%d] =%d\n", i, back_Robot_mode[i]);
    }
    // before_Robot模式设置
    LOG_IF(ERROR, set_group_mode(before_Robot, before_Robot_mode) != 0)
        << "Set_group_mode ChassisRob_mode error!";
    // back_Robot模式设置
    LOG_IF(ERROR, set_group_mode(back_Robot, back_Robot_mode) != 0)
        << "Set_group_mode ChassisRob_mode error!";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // before_Robot上电
    if (0 != group_power_on(before_Robot))
    {
        printf("YWHJ_Robot group_power_on ERROR!\n");
        return -1;
    }
    // before_Robot上电
    if (0 != group_power_on(back_Robot))
    {
        printf("YWHJ_Robot group_power_on ERROR!\n");
        return -1;
    }
    return 0;
}

// 主控函数
int YinwMotion::YwMotionPlanRun(YinwDeviceParas &deviceParas)
{
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 1;
    RTimer_.cycle_times = 1;
#endif
#ifdef USBCAN
    init_can();
    TaiHuMotorVelLimitSet(400, 400, 5000, 400, 500); // CAN的钛虎电机速度初始化
#endif
#ifdef ETHERCAT
    YWDeviceSeekZeroAndReset(deviceParas);         // 设备寻零和复位
    ZeroPointPosINISave(YWzeroNameANDpos_INIsave); // 零点存储
#endif
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ZeroPointPosINIRead(YWzeroNameANDpos);
    LOG(INFO) << "zeroNameANDpos: relative-Motor Zero Pos Matrix= [" << std::endl; // 打印所有的零点名字和零点位置
    for (size_t i = 0; i < 8; i++)
    {
        for (size_t j = 0; j < 2; j++)
            LOG(INFO) << i << j << ": " << YWzeroNameANDpos[i][j] << "   ";
        LOG(INFO) << std::endl;
    }
    LOG(INFO) << "]\n"
              << std::endl;
    while (bRunning_Flag)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        YWDeviceRunFlow(deviceParas);
    }
    return 0;
}
