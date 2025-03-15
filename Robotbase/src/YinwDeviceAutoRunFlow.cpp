#include <iostream>
#include <string>
#include <unistd.h>
#include "controlcan.h"
#include "IniParser.h"
#include "ControlCanFun.h"
#include "YinwMotion.h"
#include "IOdeviceControl.h"
#include <cmath>
#include <string.h>
#include <vector> // 添加vector头文件
#include <cstdint>
#include <chrono>
#include <glog/logging.h>
#include "HYYRobotInterface.h"
#include "device_interface.h"
#include "TaiHuCAN.h"
#include "YinwDeviceAutoRunFlow.h"
#include "TCP_Protocol.h"
#include "TcpTeleGlobalStruct.h"
#include "defineConfig.h"
#include <thread>
#include <array>
#include <future>
#include <atomic>

/** ControlStepflow:控制流参数
 * 1：银网上料（人工）
 * 2：银片上料（人工）
 * 3：设置作业要求（人工）
 * 4：启动作业（人工）
 * 5：给网及展平作业
 * 6：焊台运动至初始位置(A)
 * 7：焊架上下锁紧机构升起
 * 8：焊架左右运输机构移动至焊台左侧
 * 9：银网压制位运动至初始位置
 * 10：压机冲制银网
 * 11：银网压制位运动至送单片银网位置
 * 12：三坐标自极片库取极片，气泵吸极片
 * 13：三坐标运输极片至焊台，气泵放极片
 * 14：三坐标至银网压制位取银网，夹爪闭合
 * 15：三坐标夹取小的单片银网运送至焊台，控制夹爪打开
 * 16：焊架银网压紧机构向右移至压紧银网极片
 * 17：焊架银网压紧机构向下运动压紧银网极片
 * 18：焊台由焊接准备位置运动至焊接位置(B)
 * 19：开始焊接
 * 20：焊架上下锁紧机构升起
 * 21：焊架左右移动机构移动至最左侧
 * 22：焊接结束后，焊台继续运动至检测为止(C)
 * 23：接收检测设备的信号
 * 24：焊台继续运动至产品出口为止(D)
 * 25：三坐标运动至产品拾取位置，气泵吸取产品
 * 26：根据检测信号，是废品，三坐标吸取废品运动至废品库，放开产品
 * 27：根据检测信号，是成品，三坐标吸取成品运动至成品库，放开产品
 * 28：展平机构移动
 * 29：极片库运动
 * 30：合格品库运动
 * 31：废品库运动
 * */
int16_t YinwMotion::YWDeviceRunFlow(YinwDeviceParas &deviceParas)
{
    std::array<bool, 16> GET_IOstatus;
    // 限制msgID扰动控制流
    if (ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID != 0)
    {
        LOG(INFO) << "msgID = " << ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID << std::endl;
        ControlStepflow = static_cast<short>(ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID);
        // 急停, 将最后一个状态更新进vector
        if (ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID == ThirtySixthStep)
            ControlStepflowSAVE(-1);
        // 继续, 执行内存中的最后一个状态
        if (ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID == ThirtyNineStep)
        {
            ControlStepflow = ControlStepflowSave[3]; // 最后一个数
            // 启动故障检测线程
            std::lock_guard<std::mutex> lock(FaultWaitMtx);
            ResumeFlag.store(true);
            FaultWait.notify_one();
        }
        // 帧ID清空
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = 0;
        // 存储控制流
        ControlStepflowSAVE(ControlStepflow);
        // 控制设备的运作模式
        DeviceControlMode = ShareMsgHuman2RobCMD_.DeviceControlMode;
        // 如果是步进模式,需要对各个机构的控制位进行复位
        if (DeviceControlMode == StepControlMode)
            YW_PartlySystemFlowsReset();
    }
    // 故障检测
    if (ResumeFlag)
    {
        LOG(INFO) << "Warning, This System Has faults, System is set to Waiting!" << std::endl;
        ControlStepflow = -1;
    }
    // 主控制流
    switch (ControlStepflow)
    {
    case FifthStep: // 5：给网及展平作业
    {
        LOG(INFO) << "################ -5- FifthStep: YinwFeedinANDFlattening  #########" << std::endl;
#ifdef ETHERCAT
        if (YinwFeedinANDFlattening(deviceParas) == 1)
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = SixthStep;
                LOG(INFO) << "YinwFeedinANDFlattening  finish, switch!." << std::endl;
            }
            else
            {
                ControlStepflowReset();
                LOG(INFO) << "YinwFeedinANDFlattening  finish, No Switch!." << std::endl;
            }
        }
        else
            LOG(INFO) << "YinwFeedinANDFlattening running now..." << std::endl;
#endif
        break;
    }
    case SixthStep: // 6:焊台运动至初始位置(A)
    {
        LOG(INFO) << "################  -6- SixthStep: WeldPreparePos A ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskCtrl(deviceParas, "WeldPreparePos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = NinthStep;
                LOG(INFO) << "WeldPreparePos WeldingPrepare arrive over!." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldPreparePos is going ...." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldPreparePos is running...." << std::endl;
#endif
        break;
    }
#ifdef Hanjia
    case SeventhStep: // 7: 焊架上下锁紧机构升起
    {
        LOG(INFO) << "################ -7- SeventhStep: WeldDeskUpPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskUpDownLock(deviceParas, "WeldDeskUpPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = EighthStep;
                LOG(INFO) << "WeldDeskUpPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldDeskUpPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldDeskUpPos is running ...." << std::endl;
#endif
        break;
    }
    case EighthStep: // 8:焊架左右运输机构移动至焊台左侧
    {
        LOG(INFO) << " ################ -8- EighthStep: WeldLeftOpenPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskLeftRightRun(deviceParas, "WeldLeftOpenPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = NinthStep;
                LOG(INFO) << "WeldLeftOpenPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldLeftOpenPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldLeftOpenPos is running ...." << std::endl;
#endif
        break;
    }
#endif
    case NinthStep: // 9：银网压制位运动至初始位置
    {
        LOG(INFO) << "################ -9- NinthStep: WeldingPressPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingPressMove(deviceParas, "WeldingPressPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TenthStep;
                LOG(INFO) << "WeldingPressPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldingPressPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldingPressPos is running ...." << std::endl;
#endif
        break;
    }
    case TenthStep: // 10：压机冲制银网
    {
        LOG(INFO) << " ################ -10- TenthStep: silverNetPunchGeneralFlow ################" << std::endl;
        auto starttime = std::chrono::high_resolution_clock::now();
        auto starttime_ms = std::chrono::time_point_cast<std::chrono::microseconds>(starttime);
        LOG(INFO) << "starttime = " << starttime_ms.time_since_epoch().count() << " us" << std::endl;
#ifdef ETHERCAT
        if (silverNetPunchGeneralFlow(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = EleventhStep;
                LOG(INFO) << "silverNetPunchGeneralFlow finish, Switch." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // 吸取极片和运输极片
                SuckUpThreadisRunning.store(true);
                std::thread polePlatePickAndSuckUp_thread([&]()
                                                          { polePlatePickAndSuckUpThread(deviceParas); });
                polePlatePickAndSuckUp_thread.detach();
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "silverNetPunchGeneralFlow finish, No Switch." << std::endl;
            }
            LOG(INFO) << "TenthStep finish!" << std::endl;
        }
        else
            LOG(INFO) << "TenthStep is running..." << std::endl;
#endif
        break;
    }
    case EleventhStep: // 11：银网压制位运动至送单片银网位置
    {
        LOG(INFO) << " ################ -11- EleventhStep: WeldingTransNetPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingPressMove(deviceParas, "WeldingTransNetPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                if (!SuckUpThreadisRunning.load())
                {
                    ControlStepflow = FourteenthStep;
                    LOG(INFO) << "polePlatePickAndSuckUpThread finish, WeldingTransNetPos finish, Switch!" << std::endl;
                }
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldingTransNetPos finish, No Switch!" << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldingTransNetPos is running ...." << std::endl;
#endif
        break;
    }
    case TwelfthStep: // 12：三坐标自极片库取极片，气泵吸极片 <已经被合并到10的线程中了>
    {
        LOG(INFO) << " ################ -12- TwelfthStep: polePlatePickAndSuckUp ################" << std::endl;
#ifdef ETHERCAT
        if (polePlatePickAndSuckUp(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = ThirteenthStep;
                LOG(INFO) << "polePlatePickAndSuckUp finish, Switch!" << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "polePlatePickAndSuckUp finish, No Switch" << std::endl;
            }
        }
        else
            LOG(INFO) << "polePlatePickAndSuckUp is running ...." << std::endl;
#endif
        break;
    }
    case ThirteenthStep: // 13：三坐标运输极片至焊台，气泵放极片 <已经被合并到10的线程中了>
    {
        LOG(INFO) << "################ -13- ThirteenthStep: polePlatePickAndSuckDown ################" << std::endl;
#ifdef ETHERCAT
        if (polePlatePickAndSuckDown(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = FourteenthStep;
                LOG(INFO) << "polePlatePickAndSuckDown finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "polePlatePickAndSuckDown finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "polePlatePickAndSuckDown is running ...." << std::endl;
#endif
        break;
    }
    case FourteenthStep: // 14：三坐标至银网压制位取银网，夹爪闭合
    {
        LOG(INFO) << "################ -14- FourteenthStep: SliverNetPickAndClampClose ################" << std::endl;
#ifdef ETHERCAT
        if (SliverNetPickAndClampClose(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = FifteenthStep;
                LOG(INFO) << "SliverNetPickAndClampClose finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "SliverNetPickAndClampClose finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "SliverNetPickAndClampClose is running ...." << std::endl;
#endif
        break;
    }
    case FifteenthStep: // 15：三坐标夹取小的单片银网运送至焊台，控制夹爪打开
    {
        LOG(INFO) << "################ -15- FifteenthStep: SliverNetPlaceAndClampOpen ################" << std::endl;
#ifdef ETHERCAT
        if (SliverNetPlaceAndClampOpen(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = EighteenthStep;
                LOG(INFO) << "SliverNetPlaceAndClampOpen finish, Switch." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // 线程1: 银网压制位复位
                WeldingPressResetThreadisRunning.store(true);
                std::thread WeldingPress([&]()
                                         { WeldingPressMoveReset(deviceParas, "WeldingPressPos"); });
                WeldingPress.detach();
                ProductWaitThreadisRunning.store(true);
                // 线程2: 三坐标到取产品位置
                std::thread PickProductWait([&]()
                                            { ProductPickUpWait(deviceParas); });
                PickProductWait.detach();
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "SliverNetPlaceAndClampOpen finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "SliverNetPlaceAndClampOpen is running ...." << std::endl;
#endif
        break;
    }
#ifdef Hanjia
    case SixteenthStep: // 16：焊架银网压紧机构向右移至压紧银网极片
    {
        LOG(INFO) << "################ -16- SixteenthStep: WeldRightLockPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskLeftRightRun(deviceParas, "WeldRightLockPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = SeventeenthStep;
                LOG(INFO) << "WeldRightLockPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldRightLockPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldRightLockPos is running ...." << std::endl;
#endif
        break;
    }
    case SeventeenthStep: // 17：焊架银网压紧机构向下运动压紧银网极片 和 18步融合到一起
    {
        LOG(INFO) << "################ -17- -18- SeventeenthStep: WeldDeskDownPos ################" << std::endl;
#ifdef ETHERCAT
        if ((WeldingDeskUpDownLock(deviceParas, "WeldDeskDownPos") && (WeldingDeskCtrl(deviceParas, "WeldingPos"))))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = NineteenthStep;
                LOG(INFO) << "WeldDeskDownPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldDeskDownPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldDeskDownPos is running ...." << std::endl;
#endif
        break;
    }
#endif
    case EighteenthStep: // 18：焊台由焊接准备位置运动至焊接位置(B) [暂时可能不用，融合在了17步中][原先不用,但是取消了焊架,这个又要重新启用了]
    {
        LOG(INFO) << "################  -18- EighteenthStep: WeldingPos B ################ " << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskCtrl(deviceParas, "WeldingPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = NineteenthStep;
                LOG(INFO) << "WeldingPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldingPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldingPos is going to WeldingPrepare...." << std::endl;
#endif
        break;
    }
    case NineteenthStep: // 19：开始焊接
    {
        ControlStepflowSAVE(ControlStepflow); // 存在状态
        ControlStepflow = TwentysecondStep;
        LOG(INFO) << "################ -19-  NineteenthStep: Welding is starting ################" << std::endl;
        break;
    }
#ifdef Hanjia
    case TwentiethStep: // 20：焊架上下锁紧机构升起
    {
        LOG(INFO) << "################  -20-  TwentiethStep: WeldDeskUpPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskUpDownLock(deviceParas, "WeldDeskUpPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentyfirstStep;
                LOG(INFO) << "WeldDeskUpPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldDeskUpPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldDeskUpPos is running ...." << std::endl;
#endif
        break;
    }
    case TwentyfirstStep: // 21：焊架左右移动机构移动至最左侧
    {
        LOG(INFO) << "################  -21- TwentyfirstStep: WeldLeftOpenPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskLeftRightRun(deviceParas, "WeldLeftOpenPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentysecondStep;
                LOG(INFO) << "WeldLeftOpenPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldLeftOpenPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldLeftOpenPos is running ...." << std::endl;
#endif
        break;
    }
#endif
    case TwentysecondStep: // 22：焊接结束后，焊台继续运动至检测位置(C)
    {
        LOG(INFO) << " ################  -22- TwentysecondStep: WeldDetectingPos ################" << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskCtrl(deviceParas, "WeldDetectingPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentythirdStep;
                LOG(INFO) << "WeldDetectingPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldDetectingPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldDetectingPos is going to WeldingPrepare...." << std::endl;
#endif
        break;
    }
    case TwentythirdStep: // 23：接收检测设备的信号
    {
        LOG(INFO) << "################  -23- TwentythirdStep: OnlineInspectionRecv ################" << std::endl;
#ifdef ETHERCAT
        OnlineInspectionRecvRet = OnlineInspectionRecv(1);
        ControlStepflowSAVE(ControlStepflow); // 存在状态
        if (!ProductWaitThreadisRunning)
        {
            LOG(INFO) << "ProductWaitThreadisRunning and OnlineInspectionRecv OVER!!!";
            ControlStepflow = TwentyfourthStep;
        }
#endif
        break;
    }
    case TwentyfourthStep: // 24：焊台继续运动至产品出口为止(D)--产品检测已结束
    {
        LOG(INFO) << "################   -24- TwentyfourthStep: PickWeldPos ################ " << std::endl;
#ifdef ETHERCAT
        if (WeldingDeskCtrl(deviceParas, "PickWeldPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentyfifthStep;
                LOG(INFO) << "PickWeldPos WeldingPrepare finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "PickWeldPos WeldingPrepare finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "PickWeldPos is going to WeldingPrepare...." << std::endl;
#endif
        break;
    }
    case TwentyfifthStep: // 25：三坐标运动至产品拾取位置，气泵吸取产品
    {
        LOG(INFO) << "################  -25- TwentyfifthStep: ProductPickUp ################" << std::endl;
#ifdef ETHERCAT
        if (ProductPickUp(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                std::this_thread::sleep_for(std::chrono::seconds(1));
                WeldingDeskResetThreadisRunning.store(true);
                // Lambda 表达式创建线程,控制焊台复位
                std::thread WeldDeskReset([&]()
                                          { WeldingDeskCtrlReset(deviceParas, "WeldPreparePos"); });
                WeldDeskReset.detach();
                // 成品和废品进行分类跳转
                if (OnlineInspectionRecvRet)
                {
                    ControlStepflow = ThirtiethStep; // 成品
                    LOG(INFO) << "ProductPickUp GOOD, Switch." << std::endl;
                }
                else
                {
                    ControlStepflow = ThirtyfirtStep; // 废品
                    LOG(INFO) << "ProductPickUp BAD, Switch." << std::endl;
                }
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "ProductPickUp finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "ProductPickUp is running..." << std::endl;
#endif
        break;
    }
    case ThirtiethStep: // 30：合格品库运动 由于要进行合格品库先动 所以将这步提前
    {
        LOG(INFO) << "################  -30- ThirtiethStep: QualifiedRoom ################" << std::endl;
        if (StepMotorsWatchDog++ == StepMotorsDog)
        {
#ifdef ETHERCAT
            // LOG(INFO) << "StepMotorsWatchDog = " << StepMotorsWatchDog << std::endl;
            StepMotorsWatchDog = 0;
            if (QualifiedRoom(deviceParas))
            {
                LOG(INFO) << "DeviceControlMode = " << DeviceControlMode << std::endl;
                if (DeviceControlMode == AutoControlMode)
                {
                    ControlStepflowSAVE(ControlStepflow); // 存在状态
                    ControlStepflow = TwentyseventhStep;
                    LOG(INFO) << "QualifiedRoom Step Run Over, Auto Switch!!!" << std::endl;
                }
                else
                {
                    ControlStepflowReset();
                    LOG(INFO) << "QualifiedRoom Step Run Over, No Switch!!!" << std::endl;
                }
            }
            else
                LOG(INFO) << "QualifiedRoom is Running ..." << std::endl;
#endif
        }
        break;
    }
    case TwentyseventhStep: // 27：根据检测信号，是成品，三坐标吸取成品运动至成品库，放开产品
    {
        LOG(INFO) << "################  -27- TwentyseventhStep: QualifiedStacking ################" << std::endl;
#ifdef ETHERCAT
        if (QualifiedStacking(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentyeighthStep;
                LOG(INFO) << "QualifiedStacking finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "QualifiedStacking finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "QualifiedStacking is running..." << std::endl;
#endif
        break;
    }
    case ThirtyfirtStep: // 31：废品库运动
    {
        LOG(INFO) << "################  -31- ThirtyfirtStep: abandonedRoom ################" << std::endl;
        if (StepMotorsWatchDog++ == StepMotorsDog)
        {
            StepMotorsWatchDog = 0;
#ifdef ETHERCAT
            if (abandonedRoom(deviceParas))
            {
                if (DeviceControlMode == AutoControlMode)
                {
                    ControlStepflowSAVE(ControlStepflow); // 存在状态
                    ControlStepflow = TwentysixthStep;
                    LOG(INFO) << "abandonedRoom Step Run Over, Auto Switch!" << std::endl;
                }
                else
                {
                    ControlStepflowReset();
                    LOG(INFO) << "abandonedRoom Step Run Over, No Swithch!" << std::endl;
                }
            }
            else
                LOG(INFO) << "abandonedRoom Running Now..." << std::endl;
#endif
        }
        break;
    }
    case TwentysixthStep: // 26：根据检测信号，是废品，三坐标吸取废品运动至废品库，放开产品
    {
        LOG(INFO) << "################  -26- TwentysixthStep: WasteStacking ################" << std::endl;
#ifdef ETHERCAT
        if (WasteStacking(deviceParas))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = TwentyeighthStep;
                LOG(INFO) << "WasteStacking finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WasteStacking finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WasteStacking is running..." << std::endl;
#endif
        break;
    }
    case TwentyeighthStep: // 28：展平机构移动
    {
        LOG(INFO) << "################  -28- TwentyeighthStep: FlatteningStepMove ################" << std::endl;
        if (StepMotorsWatchDog++ == StepMotorsDog)
        {
            StepMotorsWatchDog = 0;
#ifdef ETHERCAT
            if (FlatteningStepMove(deviceParas))
                if (DeviceControlMode == AutoControlMode)
                {
                    ControlStepflowSAVE(ControlStepflow); // 存在状态
                    ControlStepflow = ThirtySecondStep;
                    LOG(INFO) << "FlatteningStepMove finish, Switch." << std::endl;
                }
                else
                {
                    ControlStepflow = -1;
                    LOG(INFO) << "FlatteningStepMove finish, No Switch." << std::endl;
                }
            else
                LOG(INFO) << "FlatteningStepMove is running..." << std::endl;
#endif
        }
        break;
    }
    case TwentyninethStep: // 29：极片库运动,目前已经将其融入到步骤12中了
    {
        LOG(INFO) << "################ -29- TwentyninethStep: polePlateRoom ################" << std::endl;
        if (StepMotorsWatchDog++ == StepMotorsDog)
        {
            StepMotorsWatchDog = 0;
#ifdef ETHERCAT
            if (polePlateRoomStepping(deviceParas))
            {
                ControlStepflowSAVE(ControlStepflow); // 存在状态
                ControlStepflow = ThirtiethStep;
                LOG(INFO) << "polePlateRoom Step Run Over!!!" << std::endl;
            }
            else
                LOG(INFO) << "polePlateRoom Running Now..." << std::endl;
#endif
        }
        break;
    }
    case ThirtySecondStep: // 32：状态恢复 整个系统满血复活
    {
        LOG(INFO) << "################  -32- ThirtySecondStep, status Reset ################" << std::endl;
        ControlStepflowSAVE(ControlStepflow); // 存在状态
        YW_SystemFlowsReset();
        break;
    }
    case ThirtyThirdStep: // 33:General:上电
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        LOG(INFO) << "################  -33- ThirtyThirdStep, Start Power!!! ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("power");
#endif
        ControlStepflowReset();
        break;
    }
    case ThirtyFourthStep: // 34:对IO设备进行反馈读取
    {
        uint16_t IOcanID = 0x0207;
        IoBoardCanCmdName = 2;
        uint8_t IOcanCMD[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#ifdef USBCAN
        send_IO_CanCommand(IOcanID, IOcanCMD);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        recv_IO_CanCommand();
#endif
        ControlStepflowReset();
        break;
    }
    case ThirtyFifthStep: // 35：General:控制器下电关机
    {
        LOG(INFO) << "################  -35- YWHJ System Is Power Off... ################" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
#ifdef USBCAN
        IODeviceTeleControl("poweroff");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // system("sudo shutdown -h now"); // 使用sudo执行关机命令
#endif
        ControlStepflowReset();
        break;
    }
    case ThirtySixthStep: // 36：设备急停
    {
        // 切换到这,EtherCAT电机会自动停下,仅需控制CAN电机停下就可以
        LOG(INFO) << " ################  -36- ThirtySixthStep: TaiHuMotorEmergencyStop ################" << std::endl;
#ifdef USBCAN
        TaiHuMotorEmergencyStop(0);
#endif
        ControlStepflowReset();
        break;
    }
    case ThirtySeventhStep: // 37：设备复位
    {
        LOG(INFO) << "################  -37- ThirtySeventhStep: YWDeviceReset ################" << std::endl;
#ifdef ETHERCAT
        if (YWDeviceReset(deviceParas) == 1)
        {
            ControlStepflow = -1;
            YW_SystemFlowsReset();
            LOG(INFO) << "YWDeviceReset finish!" << std::endl;
        }
        else
            LOG(INFO) << "YWDeviceReset running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case ThirtyEighthStep: // 38：设备暂停
    {
        LOG(INFO) << " ################  -38- ThirtyEighthStep: SystemPause ################" << std::endl;
#ifdef USBCAN
        TaiHuMotorEmergencyStop(0); // 切换到这,EtherCAT电机会自动停下,仅需控制CAN电机停下就可以
#endif
        ControlStepflowReset();
        break;
    }
    // case ThirtyNineStep: // 39：设备继续
    // {
    //     LOG(INFO) << "################  -39- YWDeviceReset go on running !! ################" << std::endl;
    //     ControlStepflow = ControlStepflowSave[3];
    //     break;
    // }
    case FortiethStep: // 40：设备自动模式 一键启动
    {
        LOG(INFO) << "################  -40- FortiethStep, OneKey start!! ################" << std::endl;
        ControlStepflow = FifthStep;
        DeviceControlMode == AutoControlMode;
        break;
    }
    case FortyFirstStep: // 41：设备开灯
    {
        LOG(INFO) << "################  -41- FortyFirstStep, Floodlightup! ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Floodlightup");
#endif
        ControlStepflowReset();
        break;
    }
    case FortySecondStep: // 42：设备关灯
    {
        LOG(INFO) << "################  -42- FortySecondStep, Floodlightdown!################ " << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Floodlightdown");
#endif
        ControlStepflowReset();
        break;
    }
    case FortyThirdStep: // 43：给网及展平作业复位
    {
        LOG(INFO) << "################  -43- FortyThirdStep:FeedinANDFlattenReset!!!################" << std::endl;
        ControlStepflowReset();
        break;
    }
    case FortyFourthStep: // 44：冲压复位
    {
        LOG(INFO) << "################  -44- FortyFourthStep: ForgingReset ################" << std::endl;
#ifdef ETHERCAT
        if (YW_ForgingReset(deviceParas))
        {
            LOG(INFO) << "ForgingReset finish" << std::endl;
            ControlStepflowReset();
        }
        else
            LOG(INFO) << "ForgingReset running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case FortyFifthStep: // 45：焊架竖直机构向下运动锁死
    {
        LOG(INFO) << "################  -45- FortyFifthStep:!!! ################" << std::endl;
#ifdef ETHERCAT
        if ((WeldingDeskUpDownLock(deviceParas, "WeldDeskDownPos")))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                LOG(INFO) << "WeldDeskDownPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "WeldDeskDownPos finish, No Switch." << std::endl;
            }
        }
        else
            LOG(INFO) << "WeldDeskDownPos is running ...." << std::endl;
#endif
        break;
    }
    case FortySixthStep: // 46：压机等待位置
    {
        LOG(INFO) << "################ -46-  FortySixthStep:PunchUpWaitPos!!!################" << std::endl;
#ifdef ETHERCAT
        if (silverNetPunching(deviceParas, "PunchUpWaitPos"))
        {
            if (DeviceControlMode == AutoControlMode)
            {
                LOG(INFO) << "PunchUpWaitPos finish, Switch." << std::endl;
            }
            else
            {
                ControlStepflow = -1;
                LOG(INFO) << "PunchUpWaitPos finish, No Switch." << std::endl;
            }
        }
#endif
        break;
    }
    case FortySeventhStep: // 47：极片库复位
    {
        LOG(INFO) << "################  -47- FortySeventhStep:PolePlateRoomReset!!!################ " << std::endl;
#ifdef ETHERCAT
        if (YW_PolePlateRoomReset(deviceParas))
        {
            LOG(INFO) << "PolePlateRoomReset finish" << std::endl;
            ControlStepflowReset();
        }
        else
            LOG(INFO) << "PolePlateRoomReset running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case FortyEighthStep: // 48：合格品库复位
    {
        LOG(INFO) << "################ -48- FortyEighthStep:QualifiedProductReset!!! ################" << std::endl;
#ifdef ETHERCAT
        if (YW_QualifiedProductReset(deviceParas))
        {
            LOG(INFO) << "QualifiedProductReset finish" << std::endl;
            ControlStepflowReset();
        }
        else
            LOG(INFO) << "QualifiedProductReset running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case FortyNinethStep: // 49：废品库复位
    {
        LOG(INFO) << "################ -49- FortyNinethStep:DiscardedProductRese!!!################" << std::endl;
#ifdef ETHERCAT
        if (YW_DiscardedProductReset(deviceParas))
        {
            LOG(INFO) << "DiscardedProductRese finish" << std::endl;
            ControlStepflowReset();
        }
        else
            LOG(INFO) << "DiscardedProductRese running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case FiftiethStep: // 50：焊架上下移动的复位
    {
        LOG(INFO) << "################ -50- FiftiethStep:WeldingShelfUpDownReset!!! ################" << std::endl;
#ifdef ETHERCAT
        if (YW_WeldingShelfUpDownReset(deviceParas))
        {
            LOG(INFO) << "WeldingShelfUpDownReset finish" << std::endl;
            ControlStepflow = -1;
        }
        else
            LOG(INFO) << "WeldingShelfUpDownReset running..." << std::endl;
#endif
        ControlStepflowReset();
        break;
    }
    case FiftyFistStep: // 51：焊架左右移动的复位
    {
        LOG(INFO) << "################ -51- FiftyFistStep:WeldingShelfLeftRightReset!!! ################" << std::endl;
#ifdef ETHERCAT
        if (YW_WeldingShelfLeftRightReset(deviceParas))
        {
            LOG(INFO) << "WeldingShelfLeftRightReset finish" << std::endl;
            ControlStepflowReset();
        }
        else
            LOG(INFO) << "WeldingShelfLeftRightReset running..." << std::endl;
#endif
#ifdef SIMULATION
        ControlStepflowReset();
#endif
        break;
    }
    case FiftySecondStep: // 52:参数存储
    {
        LOG(INFO) << "################ -52- FiftySecondStep:SystemParasSettingSave!################" << std::endl;
        SystemParasSettingSave(ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame);
        deviceParas.loadConfig();
        ControlStepflowReset();
        break;
    }
    case FiftyThirdStep: // 53:夹爪开
    {
        LOG(INFO) << "################ -53- FiftyThirdStep:ClampOpenPos! ################" << std::endl;
#ifdef USBCAN
        SliverNetClamp(deviceParas, "ClampOpenPos");
#endif
        ControlStepflowReset();
        break;
    }
    case FiftyFourthStep: // 54:夹爪关
    {
        LOG(INFO) << "################ -54- FiftyFourthStep:ClampClosePos!################" << std::endl;
#ifdef USBCAN
        SliverNetClamp(deviceParas, "ClampClosePos");
#endif
        ControlStepflowReset();
        break;
    }
    case FiftyFifthStep: // 55:气泵开
    {
        LOG(INFO) << "################ -55- FiftyFifthStep:Airpumpon!################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Airpumpon");
#endif
        ControlStepflowReset();
        break;
    }
    case FiftySixthStep: // 56:气泵关
    {
        LOG(INFO) << "################ -56- FiftySixthStep:Airpumpoff! ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Airpumpoff");
#endif
        ControlStepflowReset();
        break;
    }
    case FiftySeventhStep: // 57:控制器重启
    {
        LOG(INFO) << "################ -57- Computer Reboot. ################" << std::endl;
#ifdef USBCAN
        std::this_thread::sleep_for(std::chrono::seconds(1));
        system("sudo shutdown -r now"); // 使用sudo执行重启指令
#endif
        ControlStepflowReset();
        break;
    }
    case FiftyEighthStep: // 58:红灯开
    {
        LOG(INFO) << "################ -58- Red Light Open. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Redlightup");
#endif
        ControlStepflowReset();
        break;
    }
    case FiftyNinethStep: // 59:红灯关
    {
        LOG(INFO) << "################ -59- Red Light Close. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Redlightdown");
#endif
        ControlStepflowReset();
        break;
    }
    case SixtiehStep: // 60:绿灯开
    {
        LOG(INFO) << "################ -60- Green Light Open. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Greenlightup");
#endif
        ControlStepflowReset();
        break;
    }
    case SixtyFirstStep: // 61:绿灯关
    {
        LOG(INFO) << "################ -61- Green Light Close. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Greenlightdown");
#endif
        ControlStepflowReset();
        break;
    }
    case SixtySecondStep: // 62:橙灯开
    {
        LOG(INFO) << "################ -62- Orange Light Open. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Orangelightup");
#endif
        ControlStepflowReset();
        break;
    }
    case SixtyThirdStep: // 63:橙灯关
    {
        LOG(INFO) << "################ -63- Orange Light Close. ################" << std::endl;
#ifdef USBCAN
        IODeviceTeleControl("Orangelightdown");
#endif
        ControlStepflowReset();
        break;
    }
    default:
        std::this_thread::sleep_for(std::chrono::seconds(2));
        YW_SystemFlowsReset(); // 在待机状态对系统的系列标志位进行复位
        LOG(INFO) << "YinW System Standby Mode!" << std::endl;
        break;
    }
    return 0;
}