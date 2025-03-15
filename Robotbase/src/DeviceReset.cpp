#include "YinwDeviceParas.h"
#include "YinwMotion.h"
#include "HYYRobotInterface.h"
#include "device_interface.h"
#include "IOdeviceControl.h"
#include <glog/logging.h>
#include "ControlCanFun.h"
#include <array>
#include <chrono>
using namespace HYYRobotBase;

/*设备的寻零和复位*/
int16_t YinwMotion::YWDeviceSeekZeroAndReset(const YinwDeviceParas &deviceParas)
{
    RTimer RTimer_;
    RTimer_.index = 2;
    RTimer_.cycle_times = 1;
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUMbefore = get_group_dof(BeforeRobot);
    int DofNUMback = get_group_dof(BackRobot);
    short ResetRunFlow = WeldingDeskReset;
    LOG(INFO) << "DofNUMbefore = " << DofNUMbefore << " DofNUMback = " << DofNUMback << std::endl;
    // 需要保障数据安全 溢出的安全退出机制!!!!
    std::array<bool, 16> GET_IOstatus = {false}, GET_IOstatus_SAVE = {false};
    int BeforeRobCurrentPos[DofNUMbefore], BackRobCurrentPos[DofNUMback];
    bool ResetComplete = false; // 新增标志位，控制循环退出
    int WatchDogColock = 0, IO_SendDog = 10, IO_RecvDog = 40, MingZhiClock = 0;
    while (!ResetComplete)
    {
        userTimerE(&RTimer_);
        if (ResumeFlag)
        {
            LOG(INFO) << "Warning, This System Has faults" << std::endl;
            return -1;
        }
        switch (ResetRunFlow)
        {
        case WeldingDeskReset: // 焊台寻零 向右侧走(负方向)  向左为正  向右为负 向左寻零
        {
            // 程序开始时间
            auto starttime = std::chrono::high_resolution_clock::now();
            auto starttime_ms = std::chrono::time_point_cast<std::chrono::microseconds>(starttime);
            // LOG(INFO) << "starttime = " << starttime_ms.time_since_epoch().count() << " us" << std::endl;
            // 向左是增加的 向右是减小的 运动值最小
            WatchDogColock++;
            YinwPosMotorControl(BackRobot, DofNUMback, WeldingPosMoveMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MaxMovePos,
                                deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelC,
                                deviceParas.Paras.weldingfixtureErrorCompensation_.weldingfixtureVelC / 2);
            if (get_group_target_position(BackRobot, BackRobCurrentPos) != 0)
                LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!WeldingDeskReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                LOG(INFO) << "GET_IOstatusRET= [";
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << "]" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[6] || GET_IOstatus[7] || GET_IOstatus[8] || GET_IOstatus[11])
                {
                    LOG(INFO) << "WeldingDesk find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[6] = GET_IOstatus[6];
                    GET_IOstatus_SAVE[7] = GET_IOstatus[7];
                    GET_IOstatus_SAVE[8] = GET_IOstatus[8];
                    if (GET_IOstatus[6]) // 焊台右侧 D
                    {
                        YWzeroNameANDpos_INIsave[0][0] = 1;
                        YWzeroNameANDpos_INIsave[0][1] = BackRobCurrentPos[WeldingPosMoveMotor - 1];
                        LOG(INFO) << "WeldingDesk Sensor__D__Pos: GET_IOstatus[6] = " << GET_IOstatus[6] << std::endl;
                    }
                    else if (GET_IOstatus[7]) // 焊台中间 B
                    {
                        YWzeroNameANDpos_INIsave[0][0] = 2;
                        YWzeroNameANDpos_INIsave[0][1] = BackRobCurrentPos[WeldingPosMoveMotor - 1];
                        LOG(INFO) << "WeldingDesk Sensor__B__Pos: GET_IOstatus[7] = " << GET_IOstatus[7] << std::endl;
                    }
                    else if (GET_IOstatus[8]) // 焊台左侧 A
                    {
                        YWzeroNameANDpos_INIsave[0][0] = 3;
                        YWzeroNameANDpos_INIsave[0][1] = BackRobCurrentPos[WeldingPosMoveMotor - 1];
                        LOG(INFO) << "WeldingDesk Sensor__A__Pos: GET_IOstatus[8] = " << GET_IOstatus[8] << std::endl;
                    }
                    else if (GET_IOstatus[11]) // 焊台左侧 C
                    {
                        YWzeroNameANDpos_INIsave[0][0] = 4;
                        YWzeroNameANDpos_INIsave[0][1] = BackRobCurrentPos[WeldingPosMoveMotor - 1];
                        LOG(INFO) << "WeldingDesk Sensor__C__Pos: GET_IOstatus[11] = " << GET_IOstatus[11] << std::endl;
                    }
                    ResetRunFlow = WeldingDeskReset + 1;
                    LOG(INFO) << "!!WeldingDesk switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
        case FeedinANDFlattenReset: // 展平移动寻零 向左减小 向右增大 向左找零
        {
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                YinwPosMotorControl(BeforeRobot, DofNUMbefore, YinwFeedinANDFlatteningMOVEmotor,
                                    deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                    deviceParas.Paras.Generalparams_.MinMovePos,
                                    deviceParas.Paras.Generalparams_.MinMovePos,
                                    deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA,
                                    deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA / 2);
                LOG(INFO) << "!!!FeedinANDFlattenReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BeforeRobot, BeforeRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[9] || GET_IOstatus[10])
                {
                    LOG(INFO) << "FeedinANDFlattenReset right zero Pos!" << std::endl;
                    GET_IOstatus_SAVE[9] = GET_IOstatus[9]; // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[10] = GET_IOstatus[10];
                    if (GET_IOstatus[9]) // 展平移动左侧
                    {
                        YWzeroNameANDpos_INIsave[1][0] = 1;
                        YWzeroNameANDpos_INIsave[1][1] = BeforeRobCurrentPos[YinwFeedinANDFlatteningMOVEmotor - 1];
                        LOG(INFO) << "FeedinANDFlatten Left_SensorPos: GET_IOstatus[9] = " << GET_IOstatus[9] << std::endl;
                    }
                    else if (GET_IOstatus[10]) // 展平移动右侧
                    {
                        YWzeroNameANDpos_INIsave[1][0] = 2;
                        YWzeroNameANDpos_INIsave[1][1] = BeforeRobCurrentPos[YinwFeedinANDFlatteningMOVEmotor - 1];
                        LOG(INFO) << "FeedinANDFlatten Right_SensorPos: GET_IOstatus[10] = " << GET_IOstatus[10] << std::endl;
                    }
                    ResetRunFlow = FeedinANDFlattenReset + 1;
                    LOG(INFO) << "FeedinANDFlatten switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
                LOG(INFO) << "FeedinANDFlattenReset.." << std::endl;
            }
            break;
        }
        case ForgingReset: // 冲压向上寻零 向下为正 向上为负    限位开关位置：-1437903  悬停位置：596498  冲压位置：2280397
        {
            YinwPosMotorControl(BeforeRobot, DofNUMbefore, YinwForgingMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.StampingErrorCompensation_.StampingVelA,
                                deviceParas.Paras.StampingErrorCompensation_.StampingVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!ForgingReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BeforeRobot, BeforeRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[12] || GET_IOstatus[13])
                {
                    LOG(INFO) << "ForgingReset find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[12] = GET_IOstatus[12];
                    GET_IOstatus_SAVE[13] = GET_IOstatus[13];
                    if (GET_IOstatus[12]) // 压机上侧
                    {
                        YWzeroNameANDpos_INIsave[2][0] = 1;
                        YWzeroNameANDpos_INIsave[2][1] = BeforeRobCurrentPos[YinwForgingMotor - 1];
                        LOG(INFO) << "ForgingReset UP_SensorPos: GET_IOstatus[12] = " << GET_IOstatus[12] << std::endl;
                    }
                    else if (GET_IOstatus[13]) // 压机下侧
                    {
                        YWzeroNameANDpos_INIsave[2][0] = 2;
                        YWzeroNameANDpos_INIsave[2][1] = BeforeRobCurrentPos[YinwForgingMotor - 1];
                        LOG(INFO) << "ForgingReset Down_SensorPos: GET_IOstatus[13] = " << GET_IOstatus[13] << std::endl;
                    }
                    ResetRunFlow = ForgingReset + 1;
                    LOG(INFO) << "ForgingReset switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
        case PolePlateRoomReset: // 极片库寻零 向上为负 向下为正
        {
            YinwPosMotorControl(BeforeRobot, DofNUMbefore, PolarPlateWarehouseMoveMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MaxMovePos,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!PolePlateRoomReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BeforeRobot, BeforeRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[0] || GET_IOstatus[5])
                {
                    LOG(INFO) << "PolePlateRoomReset find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[0] = GET_IOstatus[0];
                    GET_IOstatus_SAVE[5] = GET_IOstatus[5];
                    if (GET_IOstatus[0]) // 极片库上侧
                    {
                        YWzeroNameANDpos_INIsave[3][0] = 1;
                        YWzeroNameANDpos_INIsave[3][1] = BeforeRobCurrentPos[PolarPlateWarehouseMoveMotor - 1];
                        LOG(INFO) << "PolePlateRoomReset UP_SensorPos: GET_IOstatus[12] = " << GET_IOstatus[12] << std::endl;
                    }
                    else if (GET_IOstatus[5]) // 极片库下侧
                    {
                        YWzeroNameANDpos_INIsave[3][0] = 2;
                        YWzeroNameANDpos_INIsave[3][1] = BeforeRobCurrentPos[PolarPlateWarehouseMoveMotor - 1];
                        LOG(INFO) << "PolePlateRoomReset UP_SensorPos: GET_IOstatus[12] = " << GET_IOstatus[12] << std::endl;
                    }
                    ResetRunFlow = PolePlateRoomReset + 1;
                    LOG(INFO) << "PolePlateRoomReset switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
        case DiscardedProductReset: // 废品库寻零 向上为负 向下为正
        {
            YinwPosMotorControl(BackRobot, DofNUMback, DiscardedProductWareHouseMoveMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                WatchDogColock = 0;
                LOG(INFO) << "!!!DiscardedProductReset.." << std::endl;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BackRobot, BackRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[3] || GET_IOstatus[4])
                {
                    LOG(INFO) << "DiscardedProductReset find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[3] = GET_IOstatus[3];
                    GET_IOstatus_SAVE[4] = GET_IOstatus[4];
                    if (GET_IOstatus[3]) // 废品库上侧
                    {
                        YWzeroNameANDpos_INIsave[4][0] = 1;
                        YWzeroNameANDpos_INIsave[4][1] = BackRobCurrentPos[DiscardedProductWareHouseMoveMotor - 1];
                        LOG(INFO) << "DiscardedProductReset LeftSensorPos: GET_IOstatus[3] = " << GET_IOstatus[3] << std::endl;
                    }
                    else if (GET_IOstatus[4]) // 废品库下侧
                    {
                        YWzeroNameANDpos_INIsave[4][0] = 2;
                        YWzeroNameANDpos_INIsave[4][1] = BackRobCurrentPos[DiscardedProductWareHouseMoveMotor - 1];
                        LOG(INFO) << "DiscardedProductReset Left_SensorPos: GET_IOstatus[4] = " << GET_IOstatus[4] << std::endl;
                    }
                    ResetRunFlow = DiscardedProductReset + 1;
                    LOG(INFO) << "DeviceReset switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
        case QualifiedProductReset: // 合格品库寻零 向上为负 向下为正
        {
            YinwPosMotorControl(BeforeRobot, DofNUMbefore, QualifiedProductWareHouseMoveMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!QualifiedProductReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BeforeRobot, BeforeRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[1] || GET_IOstatus[2])
                {
                    LOG(INFO) << "QualifiedProductReset find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[1] = GET_IOstatus[1];
                    GET_IOstatus_SAVE[2] = GET_IOstatus[2];
                    if (GET_IOstatus[1]) // 合格品库上侧
                    {
                        YWzeroNameANDpos_INIsave[5][0] = 1;
                        YWzeroNameANDpos_INIsave[5][1] = BeforeRobCurrentPos[QualifiedProductWareHouseMoveMotor - 1];
                        LOG(INFO) << "QualifiedProductReset UP_SensorPos: GET_IOstatus[3] = " << GET_IOstatus[3] << std::endl;
                    }
                    else if (GET_IOstatus[2]) // 合格品库下侧
                    {
                        YWzeroNameANDpos_INIsave[5][0] = 2;
                        YWzeroNameANDpos_INIsave[5][1] = BeforeRobCurrentPos[QualifiedProductWareHouseMoveMotor - 1];
                        LOG(INFO) << "QualifiedProductReset Down_SensorPos: GET_IOstatus[4] = " << GET_IOstatus[4] << std::endl;
                    }
                    ResetComplete = true;
                    // ResetRunFlow = QualifiedProductReset + 1;
                    LOG(INFO) << "DeviceReset switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
#ifdef Hanjia // 已经取消了
        // 这部分向下就没有执行,因为将焊架上面的左右和上下两个机构取消了
        case WeldingShelfUpDownReset: // 焊架上下寻零 限位开关在最上侧 向上增加 向下减少
        {
            YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfUpDownMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MaxMovePos,
                                deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA,
                                deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!WeldingShelfUpDownReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BackRobot, BackRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[14]) // 上下运动限位开关
                {
                    LOG(INFO) << "WeldingShelfUpDownMotor find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[14] = GET_IOstatus[14];
                    YWzeroNameANDpos_INIsave[6][0] = 1;
                    YWzeroNameANDpos_INIsave[6][1] = BackRobCurrentPos[WeldingShelfUpDownMotor - 1];
                    LOG(INFO) << "WeldingShelfUpDownMotor UP_SensorPos: GET_IOstatus[14] = " << GET_IOstatus[14] << std::endl;
                    ResetRunFlow = WeldingShelfUpDownReset + 1;
                    LOG(INFO) << "WeldingShelfUpDownMotor switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                }
            }
            break;
        }
        case WeldingShelfLeftRightReset: // 焊架左右运动寻零  向左减小 向右增加 限位开关在最左侧
        {
            YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfLeftRightMotor,
                                deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.Generalparams_.MinMovePos,
                                deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA,
                                deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA / 2);
            WatchDogColock++;
            if (WatchDogColock == IO_SendDog)
            {
                IO_X_PinsCmdSEND("IOfirst");
            }
            else if (WatchDogColock == IO_RecvDog)
            {
                LOG(INFO) << "!!!WeldingShelfLeftRightReset.." << std::endl;
                WatchDogColock = 0;
                GET_IOstatus = IO_X_PinsCmdGET();
                for (size_t i = 0; i < GET_IOstatus.size(); i++)
                    LOG(INFO) << GET_IOstatus[i];
                LOG(INFO) << std::endl;
                if (get_group_target_position(BackRobot, BackRobCurrentPos) != 0)
                    LOG(INFO) << "get_group_target_position ERROR!" << std::endl;
                // 判断IO状态
                if (GET_IOstatus[15]) // 左右运动限位开关
                {
                    LOG(INFO) << "WeldingShelfLeftRightReset find itself succeed!!" << std::endl;
                    // 转存一下,防止损坏状态值
                    GET_IOstatus_SAVE[15] = GET_IOstatus[15];
                    YWzeroNameANDpos_INIsave[7][0] = 1;
                    YWzeroNameANDpos_INIsave[7][1] = BackRobCurrentPos[WeldingShelfLeftRightMotor - 1];
                    LOG(INFO) << "WeldingShelfLeftRightReset Left_SensorPos: GET_IOstatus[15] = " << GET_IOstatus[15] << std::endl;
                    LOG(INFO) << "WeldingShelfLeftRightReset switch succeed! ResetRunFlow = " << ResetRunFlow << std::endl;
                    ResetComplete = true;
                }
            }
            break;
        }
#endif
        default:
        {
            LOG(INFO) << "Unknown ResetRunFlow state!" << std::endl;
            ResetComplete = true; // 防止死循环
            break;
        }
        }
    }
    return 0;
}

/*
 * fun:冲压设备复位
 */
int16_t YinwMotion::YW_ForgingReset(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUMbefore = get_group_dof(BeforeRobot);
    // 机器人1-冲压机构 1
    int16_t ResetRet = YinwPosMotorControl(BeforeRobot, DofNUMbefore, YinwForgingMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[2][1],
                                           deviceParas.Paras.StampingErrorCompensation_.StampingVelA,
                                           deviceParas.Paras.StampingErrorCompensation_.StampingVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:极片库复位
 */
int16_t YinwMotion::YW_PolePlateRoomReset(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUMbefore = get_group_dof(BeforeRobot);
    int16_t ResetRet = YinwPosMotorControl(BeforeRobot, DofNUMbefore, PolarPlateWarehouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[3][1],
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:废品库复位
 */
int16_t YinwMotion::YW_DiscardedProductReset(const YinwDeviceParas &deviceParas)
{
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUMback = get_group_dof(BackRobot);
    int16_t ResetRet = YinwPosMotorControl(BackRobot, DofNUMback, DiscardedProductWareHouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[4][1],
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:成品库复位
 */
int16_t YinwMotion::YW_QualifiedProductReset(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    int DofNUMbefore = get_group_dof(BeforeRobot);
    int16_t ResetRet = YinwPosMotorControl(BeforeRobot, DofNUMbefore, QualifiedProductWareHouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[5][1],
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                           deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:焊架上下运动复位
 */
int16_t YinwMotion::YW_WeldingShelfUpDownReset(const YinwDeviceParas &deviceParas)
{
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUMback = get_group_dof(BackRobot);
    int16_t ResetRet = YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfUpDownMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[6][1],
                                           deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA,
                                           deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:焊架左右运动复位
 */
int16_t YinwMotion::YW_WeldingShelfLeftRightReset(const YinwDeviceParas &deviceParas)
{
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUMback = get_group_dof(BackRobot);
    int16_t ResetRet = YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfLeftRightMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                           deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[7][1],
                                           deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA,
                                           deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA / 2);
    if (ResetRet)
    {
        return 1;
    }
    return 0;
}

/*
 * fun:EtherCAT设备复位
 * 这个函数在用的时候,设备已经寻零完毕
 * 在控制设备主动复位的时候用
 */
int16_t YinwMotion::YW_EtherCAT_DeviceReset(const YinwDeviceParas &deviceParas)
{
    const char *BeforeRobot = get_name_robot_device(get_deviceName(0, NULL), 0);
    const char *BackRobot = get_name_robot_device(get_deviceName(0, NULL), 1);
    int DofNUMbefore = get_group_dof(BeforeRobot);
    int DofNUMback = get_group_dof(BackRobot);
    int16_t ResetRet[9];
    //-----机器人1------//
    // 机器人1-冲压机构 1
    ResetRet[0] = YinwPosMotorControl(BeforeRobot, DofNUMbefore, YinwForgingMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[2][1],
                                      deviceParas.Paras.StampingErrorCompensation_.StampingVelA,
                                      deviceParas.Paras.StampingErrorCompensation_.StampingVelA / 2);

    // 机器人1-展平移动 4
    ResetRet[1] = YinwPosMotorControl(BeforeRobot, DofNUMbefore, YinwFeedinANDFlatteningMOVEmotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[1][1],
                                      deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA,
                                      deviceParas.Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA / 2);

    // 机器人1-极片库运动 5
    ResetRet[2] = YinwPosMotorControl(BeforeRobot, DofNUMbefore, PolarPlateWarehouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[3][1],
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);

    // 机器人1-成品库移动 6
    ResetRet[3] = YinwPosMotorControl(BeforeRobot, DofNUMbefore, QualifiedProductWareHouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[5][1],
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);

    //-----机器人2------//
    // 机器人2-废品库移动 1
    ResetRet[4] = YinwPosMotorControl(BackRobot, DofNUMback, DiscardedProductWareHouseMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[4][1],
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA,
                                      deviceParas.Paras.storeroomErrorCompensation_.storeroomVelA / 2);

    // 机器人2-焊台移动 3
    ResetRet[5] = YinwPosMotorControl(BackRobot, DofNUMback, WeldingPosMoveMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[0][1],
                                      deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA,
                                      deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA / 2);

    // 机器人2-焊架左右移动 4
    ResetRet[6] = YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfLeftRightMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[7][1],
                                      deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA,
                                      deviceParas.Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA / 2);

    // 机器人2-焊架上下移动 5
    ResetRet[7] = YinwPosMotorControl(BackRobot, DofNUMback, WeldingShelfUpDownMotor, deviceParas.Paras.Generalparams_.MaxMovePos, 0,
                                      deviceParas.Paras.Generalparams_.MinMovePos, YWzeroNameANDpos[6][1],
                                      deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA,
                                      deviceParas.Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA / 2);
    ResetRet[8] = ThreeAxisResetRun(deviceParas);
    for (int i = 0; i < 9; i++)
    {
        // 如果当前元素不等于 1，则返回 false
        if (ResetRet[i] == 1)
        {
            return 1;
        }
    }
    return 0;
}

/*
 * fun:设备总体复位：EtherCAT+CAN
 * 这个函数在用的时候,设备已经寻零完毕
 * 在控制设备主动复位的时候用
 */
int16_t YinwMotion::YWDeviceReset(const YinwDeviceParas &deviceParas)
{
    bool TaihuCanMotorReset[2];
    switch (GeneralResetflow)
    {
    case GeneralResetfirst:
        if (YW_EtherCAT_DeviceReset(deviceParas) == 1) // EtherCAT电机复位
        {
            GeneralResetflow = GeneralResetsecond;
            LOG(INFO) << "YW_EtherCAT_DeviceReset finish!" << std::endl;
        }
        else
            LOG(INFO) << "YW_EtherCAT_DeviceReset running..." << std::endl;
        break;
    case GeneralResetsecond:
        // Can钛虎电机复位时间上有所耽搁,但由于是绝对的编码器是很准的,所以问题不大
        TaihuCanMotorReset[0] = WeldingPressMove(deviceParas, "WeldingPressPos"); // 焊台复位
        TaihuCanMotorReset[1] = WeldingPressMove(deviceParas, "ClampClosePos");   // 夹爪复位
        if ((TaihuCanMotorReset[0] == 1) && (TaihuCanMotorReset[1] == 1))
        {
            LOG(INFO) << "TaihuCanMotorReset finish!" << std::endl;
            return 1;
        }
        else
            LOG(INFO) << "TaihuCanMotorReset running..." << std::endl;
        break;
    default:
        break;
    }
    return 0;
}

/*
 * fun:系统的标志位复位, ALL Flags
 */
int16_t YinwMotion::YW_SystemFlowsReset()
{
    // DeviceControlMode = StepControlMode;                                // 控制模式切换
    ControlStepflow = FifthStep; // 自动控制流参数
    // DeviceRunStepsCalculate = 0;                                        // 单次设备上电运行的总步数
    FlattnMoveStausStepflow = FourthTaiHuMotor;                         // 展平控制切换
    YinWPunchGeneralFlow = FirstPunchStep;                              // 银网冲压控制位
    PlatePickAndSuckUpStatusflow = PickStepOfsuckUp;                    // 吸取极片控制位
    PlatePickAndSuckDownStatusflow = PickStepOfsuckDown;                // 放置极片控制位
    NetPickAndClampCloseStatusflow = PickStepOfClampDown;               // 三坐标抓取银网控制位
    NetPlaceAndClampOpenStatusflow = PlaceStepOfClampDown;              // 三坐标放置银网控制位
    FourthTaiHuMotorStepControlflow = FourthTaiHuMotorStepControlfirst; // 4号钛虎电机步进控制位
    SecondTaiHuMotorStepControlflow = SecondTaiHuMotorStepControlfirst; // 2号钛虎电机步进控制位
    FirstTaiHuMotorStepControlflow = FirstTaiHuMotorStepControlfirst;   // 1 号钛虎电机步进控制位
    polePlatePickAndSuckUpflow = polePlatePickAndSuckUpfirst;           // 取极片控制位
    polePlatePlaceflow = polePlatePlacefirst;                           // 极片放置控制位
    SliverNetPickflow = SliverNetPickfirst;                             // 银网夹取控制位
    SliverNetPlaceflow = SliverNetPlacefirst;                           // 银网放置控制位
    ProductPickUpflow = ProductPickUpfirst;                             // 产品吸取控制位
    WasteStackingflow = WasteStackingfirst;                             // 废品堆栈控制位
    QualifiedStackingflow = QualifiedStackingfirst;                     // 成品堆栈控制位
    ThreeAxisResetflow = ThreeAxisResetfirst;                           // 三坐标复位控制位
    GeneralResetflow = GeneralResetfirst;                               // 系统整体复位控制位
    OnlineInspectionRecvRet = false;                                    // 成品和废品的控制位
    WasteHouseflow = WasteHousefirst;                                   // 废品库控制位
    GoodHouseflow = GoodHousefirst;                                     // 成品库控制位
    FlatteningMOVEflow = FlatteningMOVEfirst;                           // 展平移动控制位
    PolePlateHouseflow = PolePlateHousefirst;                           // 极片库控制位
    return 0;
}

/*
 * fun:系统的各个子机构运动模块标志位复位, Partly Flags
 */
int16_t YinwMotion::YW_PartlySystemFlowsReset()
{
    FlattnMoveStausStepflow = FourthTaiHuMotor;                         // 展平控制切换
    YinWPunchGeneralFlow = FirstPunchStep;                              // 银网冲压控制位
    PlatePickAndSuckUpStatusflow = PickStepOfsuckUp;                    // 吸取极片控制位
    PlatePickAndSuckDownStatusflow = PickStepOfsuckDown;                // 放置极片控制位
    NetPickAndClampCloseStatusflow = PickStepOfClampDown;               // 三坐标抓取银网控制位
    NetPlaceAndClampOpenStatusflow = PlaceStepOfClampDown;              // 三坐标放置银网控制位
    FourthTaiHuMotorStepControlflow = FourthTaiHuMotorStepControlfirst; // 4号钛虎电机步进控制位
    SecondTaiHuMotorStepControlflow = SecondTaiHuMotorStepControlfirst; // 2号钛虎电机步进控制位
    FirstTaiHuMotorStepControlflow = FirstTaiHuMotorStepControlfirst;   // 1 号钛虎电机步进控制位
    polePlatePickAndSuckUpflow = polePlatePickAndSuckUpfirst;           // 取极片控制位
    polePlatePlaceflow = polePlatePlacefirst;                           // 极片放置控制位
    SliverNetPickflow = SliverNetPickfirst;                             // 银网夹取控制位
    SliverNetPlaceflow = SliverNetPlacefirst;                           // 银网放置控制位
    ProductPickUpflow = ProductPickUpfirst;                             // 产品吸取控制位
    WasteStackingflow = WasteStackingfirst;                             // 废品堆栈控制位
    QualifiedStackingflow = QualifiedStackingfirst;                     // 成品堆栈控制位
    ThreeAxisResetflow = ThreeAxisResetfirst;                           // 三坐标复位控制位
    GeneralResetflow = GeneralResetfirst;                               // 系统整体复位控制位
    WasteHouseflow = WasteHousefirst;                                   // 废品库控制位
    GoodHouseflow = GoodHousefirst;                                     // 成品库控制位
    FlatteningMOVEflow = FlatteningMOVEfirst;                           // 展平移动控制位
    PolePlateHouseflow = PolePlateHousefirst;                           // 极片库控制位
    return 0;
}