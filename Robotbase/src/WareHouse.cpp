#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;

/**
 * fun：极片库运动:向下为正,向上为负。
 * 是向下取零,逐步向上步进 步进
 */
bool YinwMotion::polePlateRoomStep(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int MotorCurrentPos[DofNUM];
    static int TargetPos;
    // 步进量mm和码值的对应关系: [最大(下方)位置(脉冲-码值)]/[一圈码值]*丝杆导程
    double PolePlateRoomScale = deviceParas.Paras.storeroom_.PolePlateDownPos / 20000.0 * 4.0;
    LOG(INFO) << "PolePlateRoomScale = " << PolePlateRoomScale << endl;
    // 步进量码值
    double PolarPlateWarehouseAIMPOS = static_cast<double>(deviceParas.Paras.PoleSheet_.JiPianHouDu) * PolePlateRoomScale;
    LOG(INFO) << "PolarPlateWarehouseAIMPOS = " << PolarPlateWarehouseAIMPOS << endl;
    switch (PolePlateHouseflow)
    {
    case PolePlateHousefirst:
    {
        if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
        TargetPos = static_cast<double>(MotorCurrentPos[PolarPlateWarehouseMoveMotor - 1]) - (int)PolarPlateWarehouseAIMPOS;
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        PolePlateHouseflow = PolePlateHousesecond;
        break;
    }
    case PolePlateHousesecond:
    {
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        // 期望位置码值
        int16_t polePlateRoomRet = YinwPosMotorControl(BeforeRobot, DofNUM, PolarPlateWarehouseMoveMotor, deviceParas.Paras.storeroom_.PolePlateDownPos, 0,
                                                       deviceParas.Paras.storeroom_.PolePlateUpPos, TargetPos,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (polePlateRoomRet == 1)
        {
            LOG(INFO) << "polePlateRoom Arrive !!!" << std::endl;
            return true;
        }
    }
    default:
        break;
    }
    return false;
}

/**
 * fun：极片库运动:向下为正,向上为负。
 * 是向下取零,逐步向上步进  一直走
 */
bool YinwMotion::polePlateRoomStepping(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int MotorCurrentPos[DofNUM];
    static int TargetPos;
    // 步进量mm和码值的对应关系: [最大(下方)位置(脉冲-码值)]/[一圈码值]*丝杆导程
    double PolePlateRoomScale = deviceParas.Paras.storeroom_.PolePlateDownPos / 20000.0 * 4.0;
    LOG(INFO) << "PolePlateRoomScale = " << PolePlateRoomScale << endl;
    // 步进量码值
    double PolarPlateWarehouseAIMPOS = static_cast<double>(deviceParas.Paras.PoleSheet_.JiPianHouDu) * PolePlateRoomScale;
    LOG(INFO) << "PolarPlateWarehouseAIMPOS = " << PolarPlateWarehouseAIMPOS << endl;
    if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    TargetPos = static_cast<double>(MotorCurrentPos[PolarPlateWarehouseMoveMotor - 1]) - (int)PolarPlateWarehouseAIMPOS;
    LOG(INFO) << "TargetPos = " << TargetPos << endl;
    PolePlateHouseflow = PolePlateHousesecond;
    LOG(INFO) << "TargetPos = " << TargetPos << endl;
    // 期望位置码值
    int16_t polePlateRoomRet = YinwPosMotorControl(BeforeRobot, DofNUM, PolarPlateWarehouseMoveMotor, deviceParas.Paras.storeroom_.PolePlateDownPos, 0,
                                                   deviceParas.Paras.storeroom_.PolePlateUpPos, TargetPos,
                                                   deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                                   deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
    if (polePlateRoomRet == 1)
    {
        LOG(INFO) << "polePlateRoom Arrive !!!" << std::endl;
        return true;
    }
    return false;
}

/**
 * fun：复位用的!!!!!
 * 极片库运动:向下为正,向上为负。
 * 是向下取零,逐步向上步进
 */
bool YinwMotion::polePlateRoomReset(const YinwDeviceParas &deviceParas)
{
    LOG(INFO) << "PolePlateRoomReset Step Run!!" << endl;
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 4;
    RTimer_.cycle_times = 1;
#endif
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int MotorCurrentPos[DofNUM];
    while (true)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        int16_t polePlateRoomRet = YinwPosMotorControl(BeforeRobot, DofNUM, PolarPlateWarehouseMoveMotor, deviceParas.Paras.storeroom_.PolePlateDownPos, 0,
                                                       deviceParas.Paras.storeroom_.PolePlateUpPos, deviceParas.Paras.storeroom_.PolePlateDownPos,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (polePlateRoomRet == 1)
        {
            LOG(INFO) << "polePlateRoom Arrive !!!" << std::endl;
            break;
        }

        if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
        {
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
        }
        // for (size_t i = 0; i < DofNUM; i++)
        //     LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    }
    return false;
}

/**
 * fun：成品库运动：向下为正,向上为负。
 * 是向上取零,逐步向下步进
 */
bool YinwMotion::QualifiedRoom(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int MotorCurrentPos[DofNUM];

    // 步进量mm和码值的对应关系: [最大(下方)位置(脉冲-码值)]/[一圈码值]*丝杆导程
    double QualifiedRoomScale = deviceParas.Paras.storeroom_.PolePlateDownPos / 20000.0 * 4.0;
    LOG(INFO) << "QualifiedRoomScale = " << QualifiedRoomScale << endl;
    // 步进量码值
    double QualifiedRoomAIMPOS = static_cast<double>(deviceParas.Paras.PoleSheet_.JiPianHouDu) * QualifiedRoomScale;
    LOG(INFO) << "QualifiedRoomAIMPOS = " << QualifiedRoomAIMPOS << endl;
    static int TargetPos;
    int16_t QualifiedRoomRet;
    switch (GoodHouseflow)
    {
    case GoodHousefirst:
        if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
        TargetPos = MotorCurrentPos[QualifiedProductWareHouseMoveMotor - 1] + (int)QualifiedRoomAIMPOS;
        // 期望位置码值
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        GoodHouseflow = GoodHousesecond;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        break;
    case GoodHousesecond:
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        QualifiedRoomRet = YinwPosMotorControl(BeforeRobot, DofNUM, QualifiedProductWareHouseMoveMotor,
                                               deviceParas.Paras.storeroom_.GoodProductDownPos, 0,
                                               deviceParas.Paras.storeroom_.GoodProductUpPos, TargetPos,
                                               deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                               deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (QualifiedRoomRet == 1)
        {
            LOG(INFO) << "QualifiedRoom Arrive !!!" << std::endl;
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

/**
 * fun：复位用
 * 成品库运动：向下为正,向上为负。
 * 是向上取零,逐步向下步进
 */
bool YinwMotion::QualifiedRoomReset(const YinwDeviceParas &deviceParas)
{
    LOG(INFO) << "QualifiedRoom Step Run!!" << endl;
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 5;
    RTimer_.cycle_times = 1;
#endif
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUM = get_group_dof(BeforeRobot);
    int MotorCurrentPos[DofNUM];
    int TargetAimPos = YWzeroNameANDpos[5][1];
    while (true)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        int16_t QualifiedRoomRet = YinwPosMotorControl(BeforeRobot, DofNUM, QualifiedProductWareHouseMoveMotor,
                                                       deviceParas.Paras.storeroom_.GoodProductDownPos, 0,
                                                       deviceParas.Paras.storeroom_.GoodProductUpPos,
                                                       deviceParas.Paras.storeroom_.GoodProductUpPos,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (QualifiedRoomRet == 1)
        {
            LOG(INFO) << "QualifiedRoom Arrive !!!" << std::endl;
            break;
        }

        if (get_group_position(BeforeRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    return false;
}

/**
 * fun：废品库运动：向下为正,向上为负
 */
bool YinwMotion::abandonedRoom(const YinwDeviceParas &deviceParas)
{
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(BackRobot);
    int _MotorCurrentPos[DofNUM];
    // 步进量mm和码值的对应关系: [最大(下方)位置(脉冲-码值)]/[一圈码值]*丝杆导程
    double abandonedRoomScale = deviceParas.Paras.storeroom_.PolePlateDownPos / 20000.0 * 4.0;
    LOG(INFO) << "abandonedRoomScale = " << abandonedRoomScale << endl;
    // 步进量码值
    double abandonedRoomAIMPOS = static_cast<double>(deviceParas.Paras.PoleSheet_.JiPianHouDu) * abandonedRoomScale;
    LOG(INFO) << "abandonedRoomAIMPOS = " << abandonedRoomAIMPOS << endl;
    static int TargetPos;
    int16_t QualifiedRoomRet;
    switch (WasteHouseflow)
    {
    case WasteHousefirst:
    {
        // 期望位置码值
        if (get_group_position(BackRobot, _MotorCurrentPos) != 0)
        {
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
        }
        TargetPos = _MotorCurrentPos[DiscardedProductWareHouseMoveMotor - 1] + (int)abandonedRoomAIMPOS;
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        WasteHouseflow = WasteHousesecond;
        break;
    }
    case WasteHousesecond:
    {
        LOG(INFO) << "TargetPos = " << TargetPos << endl;
        QualifiedRoomRet = YinwPosMotorControl(BackRobot, DofNUM, DiscardedProductWareHouseMoveMotor,
                                               deviceParas.Paras.storeroom_.BadProductDownPos, 0,
                                               deviceParas.Paras.storeroom_.BadProductUpPos, TargetPos,
                                               deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                               deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (QualifiedRoomRet == 1)
        {
            LOG(INFO) << "QualifiedRoom Arrive !!!" << std::endl;
            return true;
        }
        break;
    }
    default:
        LOG(INFO) << "abandonedRoom Unknown Status!" << std::endl;
        break;
    }
    return false;
}

/**
 * fun：废品库运动 复位控制 向下为正,向上为负
 */
bool YinwMotion::abandonedRoomReset(const YinwDeviceParas &deviceParas)
{
    LOG(INFO) << "abandonedRoom" << endl;
#ifdef ETHERCAT
    RTimer RTimer_;
    RTimer_.index = 6;
    RTimer_.cycle_times = 1;
#endif
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUM = get_group_dof(BackRobot);
    int MotorCurrentPos[DofNUM];
    int targetResetPos = YWzeroNameANDpos[4][1];
    while (true)
    {
#ifdef ETHERCAT
        userTimerE(&RTimer_);
#endif
        int16_t QualifiedRoomRet = YinwPosMotorControl(BackRobot, DofNUM, DiscardedProductWareHouseMoveMotor,
                                                       deviceParas.Paras.storeroom_.BadProductDownPos, 0,
                                                       deviceParas.Paras.storeroom_.BadProductUpPos, targetResetPos,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                                       deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA);
        if (QualifiedRoomRet == 1)
        {
            LOG(INFO) << "QualifiedRoom Arrive !!!" << std::endl;
            break;
        }

        if (get_group_position(BackRobot, MotorCurrentPos) != 0)
            LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    return false;
}

/**
 * fun：计算库的移动距离
 * para 1: 极片厚度 单位：um
 */
int16_t YinwMotion::AllRoomMoveLen(const YinwDeviceParas &deviceParas, float thickness)
{
    float EncoderValue = 131072.0;
    float Encode2Degree = EncoderValue / 360.0;
    // 丝杆导程4mm 转一圈前进4mm; 1mm=1000um
    float perCircle2Distance = 4000.0; // 4mm=4000um
    float ChangeCode = EncoderValue / perCircle2Distance * thickness;
    LOG(INFO) << "ChangeCode" << ChangeCode << std::endl;

    // 四舍五入并限制在 int16_t 范围内
    int roundedValue = static_cast<int>(std::round(ChangeCode));
    if (roundedValue > INT16_MAX)
    {
        roundedValue = INT16_MAX;
        LOG(INFO) << "1. roundedValue" << roundedValue << std::endl;
    }
    else if (roundedValue < INT16_MIN)
    {
        roundedValue = INT16_MIN;
        LOG(INFO) << "2. roundedValue" << roundedValue << std::endl;
    }
    return static_cast<int16_t>(roundedValue);
}