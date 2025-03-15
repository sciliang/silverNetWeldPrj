#ifndef YINWMOTION
#define YINWMOTION

#include <vector>
#include "YinwDeviceParas.h"
#include "TCP_Protocol.h"
#include <glog/logging.h>
#include <future>
#include <atomic>
#include <iostream>
class YinwMotion
{
private:
    YinwDeviceParas deviceParas; // 成员变量：YinwDeviceParas 类对象
    int DeviceRunStepsCalculate; // 单次上电设备运行的总步数
    short ControlStepflow;       // 自动控制流参数
    typedef enum
    {
        PositionMode = 8,
        VelocityMode,
        TorqueMode
    } Driver_mode;

    typedef enum
    {
        WeldingDeskReset = 1,       // 焊台复位
        FeedinANDFlattenReset,      // 展平移动复位
        ForgingReset,               // 冲压复位
        PolePlateRoomReset,         // 极片库复位
        DiscardedProductReset,      // 废品库复位
        QualifiedProductReset,      // 成品库复位
        WeldingShelfUpDownReset,    // 焊架上下复位
        WeldingShelfLeftRightReset, // 焊架左右复位
    } DeviceReset;

    typedef enum
    {
        StepControlMode = 1, // 步进控制
        AutoControlMode,     // 自主控制
    } deviceCtrlMode;        //
    short DeviceControlMode; // 控制模式

    typedef enum
    {
        FirstTaiHuMotor = 1,       // 1号钛虎展平控制
        SecondTaiHuMotor,          // 2号钛虎展平控制
        FourthTaiHuMotor,          // 3号钛虎展平控制
    } FlattnMoveStaus;             //
    short FlattnMoveStausStepflow; // 展平

    typedef enum
    {
        FirstPunchStep = 1,     // 冲压银网的第一步
        SecondPunchStep,        // 冲压银网的第二步
    } PunchGeneralStaus;        //
    short YinWPunchGeneralFlow; // 冲压银网

    typedef enum
    {
        PickStepOfsuckUp = 1,           // 第一步三坐标到位置
        PickSuckUpStep,                 // 第二步气泵吸取极片
        PickSuckPlateRoom,              // 第三步极片库运动
        PickTransPlate1,                // 第三步极片库运动
        PickTransPlate2,                // 第三步极片库运动
    } PlatePickAndSuckUpStatus;         //
    short PlatePickAndSuckUpStatusflow; // 吸极片

    typedef enum
    {
        PickStepOfsuckDown = 1,           // 第一步三坐标运动到位置
        SuckDownStep,                     // 第二步气泵放开极片
    } PlatePickAndSuckDownStatus;         //
    short PlatePickAndSuckDownStatusflow; // 放极片

    typedef enum
    {
        PickStepOfClampDown = 1,          // 第一步三坐标运动抓银网位置
        ClampCloseStep,                   // 第二步夹爪开始抓取
    } NetPickAndClampCloseStatus;         //
    short NetPickAndClampCloseStatusflow; // 抓银网

    typedef enum
    {
        PlaceStepOfClampDown = 1,         // 第一步运输银网至焊台
        ClampOpenStep,                    // 第二步夹爪放开银网
        PlaceStepOfClampDownZaxisUp,      // 第三步Z轴升起
    } NetPlaceAndClampOpenStatus;         //
    short NetPlaceAndClampOpenStatusflow; // 放银网

    typedef enum
    {
        FourthTaiHuMotorStepControlfirst = 1, // 4号钛虎电机步进控制第一步
        FourthTaiHuMotorStepControlsecond,    // 4号钛虎电机步进控制第二步
    } FourthTaiHuMotorStepControlStatus;      //
    short FourthTaiHuMotorStepControlflow;    // 4号钛虎电机步进控制

    typedef enum
    {
        SecondTaiHuMotorStepControlfirst = 1, // 2号钛虎电机步进控制第一步
        SecondTaiHuMotorStepControlsecond,    // 2号钛虎电机步进控制第二步
    } SecondTaiHuMotorStepControlStatus;      //
    short SecondTaiHuMotorStepControlflow;    // 2号钛虎电机步进控制

    typedef enum
    {
        FirstTaiHuMotorStepControlfirst = 1, // 1号钛虎电机步进控制第一步
        FirstTaiHuMotorStepControlsecond,    // 1号钛虎电机步进控制第二步
    } FirstTaiHuMotorStepControlStatus;      //
    short FirstTaiHuMotorStepControlflow;    // 1号钛虎电机步进控制

    typedef enum
    {
        polePlatePickAndSuckUpfirst = 1, // 取极片第一步
        polePlatePickAndSuckUpsecond,    // 取极片第二步
        polePlatePickAndSuckUpThird,     // 取极片第三步
        polePlatePickAndSuckUpFourth,    // 取极片第四步
    } polePlatePickAndSuckUpStatus;      //
    short polePlatePickAndSuckUpflow;    // 三坐标取极片

    typedef enum
    {
        polePlatePlacefirst = 1, // 放极片第一步
        polePlatePlacesecond,    // 放极片第二步
        polePlatePlaceThird,     // 放极片第三步
        polePlatePlaceFourth,    // 放极片第四步
    } polePlatePlaceStatus;      //
    short polePlatePlaceflow;    // 三坐标放极片

    typedef enum
    {
        SliverNetPickfirst = 1, // 夹爪取银网第一步
        SliverNetPicksecond,    // 夹爪取银网第二步
        SliverNetPickThird,     // 夹爪取银网第三步
        SliverNetPickFourth,    // 夹爪取银网第四步
    } SliverNetPickStatus;      //
    short SliverNetPickflow;    // 夹取银网

    typedef enum
    {
        SliverNetPlacefirst = 1, // 放置银网第一步
        SliverNetPlacesecond,    // 放置银网第二步
        SliverNetPlaceThird,     // 放置银网第三步
        SliverNetPlaceFourth,    // 放置银网第四步
    } SliverNetPlaceStatus;      //
    short SliverNetPlaceflow;    // 放置银网

    typedef enum
    {
        ProductPickUpfirst = 1, // 吸取产品第一步
        ProductPickUpsecond,    // 吸取产品第二步
        ProductPickUpThird,     // 吸取产品第三步
        ProductPickUpFourth,    // 吸取产品第四步
        ProductPickUpFifth,     // 吸取产品第五步
        ProductPickUpSixth,     // 吸取产品第六步
        ProductPickUpSeventh,   // 吸取产品第七步
    } ProductPickUpStatus;      //
    short ProductPickUpflow;    // 吸取产品

    typedef enum
    {
        WasteStackingfirst = 1, // 废品堆栈第一步
        WasteStackingsecond,    // 废品堆栈第二步
        WasteStackingThird,     // 废品堆栈第三步
        WasteStackingFourth,    // 废品堆栈第四步
        WasteStackingFifth,     // 废品堆栈第五步
        WasteStackingSixth,     // 废品堆栈第六步
    } WasteStackingStatus;      //
    short WasteStackingflow;    // 废品堆栈

    typedef enum
    {
        WasteHousefirst = 1, // 废品库运动第一步
        WasteHousesecond,    // 废品库运动第二步
        WasteHouseThird,     // 废品库运动第三步
    } WasteHouseStatus;      //
    short WasteHouseflow;    // 废品库运动

    typedef enum
    {
        QualifiedStackingfirst = 1, // 成品堆栈第一步
        QualifiedStackingsecond,    // 成品堆栈第二步
        QualifiedStackingThird,     // 成品堆栈第三步
        QualifiedStackingFourth,    // 成品堆栈第四步
        QualifiedStackingFifth,     // 成品堆栈第五步
        QualifiedStackingSixth,     // 成品堆栈第六步
    } QualifiedStackingStatus;      //
    short QualifiedStackingflow;    // 成品堆栈

    typedef enum
    {
        GoodHousefirst = 1, // 成品库运动第一步
        GoodHousesecond,    // 成品库运动第二步
        GoodHouseThird,     // 成品库运动第三步
    } GoodHouseStatus;      //
    short GoodHouseflow;    // 成品库运动

    typedef enum
    {
        PolePlateHousefirst = 1, // 极片库运动第一步
        PolePlateHousesecond,    // 极片库运动第二步
        PolePlateHouseThird,     // 极片库运动第三步
    } PolePlateHouseStatus;      //
    short PolePlateHouseflow;    // 极片库运动

    typedef enum
    {
        FlatteningMOVEfirst = 1, // 展平移动运动第一步
        FlatteningMOVEsecond,    // 展平移动运动第二步
        FlatteningMOVEThird,     // 展平移动运动第三步
    } FlatteningMOVEStatus;      //
    short FlatteningMOVEflow;    // 展平移动

    typedef enum
    {
        ThreeAxisResetfirst = 1, // 三坐标复位第一步
        ThreeAxisResetsecond,    // 三坐标复位第二步
        ThreeAxisResetThird,     // 三坐标复位第三步
        ThreeAxisResetFourth,    // 三坐标复位第四步
        ThreeAxisResetFifth,     // 三坐标复位第五步
    } ThreeAxisResetStatus;      //
    short ThreeAxisResetflow;    // 三坐标复位

    typedef enum
    {
        GeneralResetfirst = 1, // 设备整体复位第一步
        GeneralResetsecond,    // 设备整体复位第二步
        GeneralResetThird,     // 设备整体复位第三步
    } GeneralResetStatus;      //
    short GeneralResetflow;    // 设备整体复位

    bool OnlineInspectionRecvRet; // 1:成品 2:废品
    /**二维向量，构造函数中完成初始化
     * 8X2 向量
     * 0 : 焊台零位-----> 1 右侧、2 中间、3 左侧；
     * 1 : 展平移动零位->
     * 2 : 冲压零位-----> 1 上侧、2 下侧
     * 3 : 极片库零位---> 1 上侧、2 下侧
     * 4 : 废品库零位---> 1 上侧、2 下侧
     * 5 : 成品库零位---> 1 上侧、2 下侧
     * 6 : 焊架高度零位->
     * 7 : 焊架宽度零位->
     * */
    std::vector<std::vector<int>> YWzeroNameANDpos, YWzeroNameANDpos_INIsave;
    // 1 2 4号钛虎电机计算出来的步进控制位置
    uint32_t fourthTargetFinalPos[1], SecondTargetFinalPos[1], firstTargetFinalPos[1];
    int StepMotorsWatchDog, StepMotorsDog, StepMotorsRecvDog;

    std::vector<short> ControlStepflowSave;             // 使用vector存储控制流
    short ControlStepflowMaxSize;                       // 最大容量
    std::atomic<bool> SuckUpThreadisRunning;            // 吸取极片线程退出控制
    std::atomic<bool> ProductWaitThreadisRunning;       // 吸取产品线程退出控制
    std::atomic<bool> WeldingPressResetThreadisRunning; // 银网压制位复位线程退出控制
    std::atomic<bool> WeldingDeskResetThreadisRunning;  // 焊台复位线程退出控制

    RH_PRODUCTIONINFO RH_PRODUCTIONINFO_; // 生产的所有产品的统计信息
    int *ET_MotorsCurSpeed;
    short ET_MotorsDofNUM;

    //===================故障检测用====================//
    typedef enum
    {
        OVERSPEEDERROR = 1,    // 设备速度超限制
        OVERCURRENTERROR,      // 设备电流超限制
    } ET_MotorFaultInfoNumber; //
    // EtherCAT电机故障码存储
    struct ET_MotorFaultInfo
    {
        // 第一二维Vector存储电机的代码和速度超限
        std::vector<int> motorSpeedFaults; // 每个内层vector包含：机器人代码、电机代码、速度超限值
        // 第二二维Vector存储电机的代码和电流超限
        std::vector<int> motorCurrentFaults; // 每个内层vector包含：机器人代码、电机代码、电流超限值
        // 第二二维Vector存储电机的从EtherCAT拿回的故障码
        std::vector<int> motorET_ErrorCode; // 每个内层vector包含：机器人代码、电机代码、故障码
    };

    std::atomic<bool> hasSpeedFault, hasPositionFault, hasErrorCodeFault, ResumeFlag;
    std::mutex FaultWaitMtx;
    std::condition_variable FaultWait;

public:
    YinwMotion(/* args */);
    ~YinwMotion();
    void useDeviceParams(const YinwDeviceParas &deviceParas) const
    {
        // 你可以根据实际需求，打印更多的设备参数
        LOG(INFO) << "Flattening Steps: " << deviceParas.Paras.Flatten_.firstMotorApexPos << std::endl;
    }

    int InitRescueEherCAT(short TorqueLimit);
    int YwMotionPlanRun(YinwDeviceParas &deviceParas); // 控制核心线程
    // 机器人1
    typedef enum
    {
        YinwForgingMotor = 1,
        ThreeAxisYmotor,
        ThreeAxisXmotor,
        YinwFeedinANDFlatteningMOVEmotor,
        PolarPlateWarehouseMoveMotor,
        QualifiedProductWareHouseMoveMotor,
    } MotorSequence1;

    // 机器人2
    typedef enum
    {
        DiscardedProductWareHouseMoveMotor = 1,
        ThreeAxisZmotor,
        WeldingPosMoveMotor,
        WeldingShelfLeftRightMotor,
        WeldingShelfUpDownMotor,
    } MotorSequence2;
    /**
     * 将焊接银网的整个设备分成如下的动作
     * 1.启动作业:对整个设备进行作业启动
     * 2.给网及展平作业：控制展平模块对银网进行展平
     * 3.银网压片位移动：银网压片位动其他位置运动至待冲压位置或者夹取网位置
     * 4.银网冲制：从整块银网上面冲压下银网小块
     * 5.运输极片：三坐标龙门架将极片移动至红色圈位置，即焊接准备位
     * 6.运输银网：横移丝杠启动，将银网向前运输至绿色圈位置，即银网压片位
     * 7.展平网设备水平移动：为了使得同一段银网冲制多个子银网，进行展平机构的横移
     * 8.银网拾取：用三坐标龙门架进行银网的拾取
     * 9.安置银网：三坐标拾取银网成功之后，将银网放置在极片上
     * 10.安置极片：三坐标拾取极片放置在焊接准备位上面
     * 11.焊接：开始焊接
     * 12.在线检测：开始检测
     * 13.合格堆码：检测成功后会将极片放置到合格位置，三坐标龙门架记性移动
     * 14.废弃堆码：检测焊接失败会放置到废品位置，三坐标龙门架进行移动
     * 15.极片库运动：极片库进行丝杠运动控制，实现库内空间的变化
     * 16.合格品库运动：合格品库进行丝杠运动控制，实现库内空间的变化
     * 17.废品库运动：废品库进行丝杠运动控制，实现库内空间的变化
     *      *
     */
    //---------------------------------- EtherCAT级的控制-----------------------------//
    /**
     * fun：银网设备位置控制 简单平滑
     */
    int16_t YinwPosMotorControl(const char *deviceName, short MotorNUM, short MotorName, int MaxMovePos,
                                int ZeroPos, int MinMovePos, int AIMPOS, int dataMoveS, int Arrivethreshold);
    /**
     * fun：银网设备位置控制 优异平滑
     */
    int16_t YinwPosMotorControl_New(const char *deviceName, short MotorNUM, short MotorName, int MaxMovePos,
                                    int ZeroPos, int MinMovePos, int AIMPOS, int dataMoveS, int Arrivethreshold);
    /**
     * fun：每次新的动作开始时清零对应电机的速度
     */
    void resetET_MotorSpeed(const char *deviceName, short MotorName);

    /**
     * fun：银网设备速度控制
     */
    int16_t YinwVELMotorControl(const char *deviceName, short MotorName, int targetMoveVEL, int MaxMoveVEL, int MinMoveVEL);

    //---------------------------------- 三坐标单轴的控制-----------------------------//
    /**
     * fun：X轴控制
     */
    bool ThreeAxisX_ctrl(int X_AimPos);

    /**
     * fun： Y轴控制
     */
    bool ThreeAxisY_ctrl(int Y_AimPos);

    /**
     * fun： Z轴控制
     */
    bool ThreeAxisZ_ctrl(int Z_AimPos);

    //--------------------------------- 流程级控制-----------------------------//
    /**
     * fun： 流程自动控制
     */
    int16_t YWDeviceRunFlow(YinwDeviceParas &deviceParas);

    /**
     * fun： 设备主动寻零和复位
     */
    int16_t YWDeviceSeekZeroAndReset(const YinwDeviceParas &deviceParas);
    /**
     * fun： 设备复位
     */
    int16_t YWDeviceReset(const YinwDeviceParas &deviceParas);
    int16_t YW_EtherCAT_DeviceReset(const YinwDeviceParas &deviceParas);
    int16_t YW_ForgingReset(const YinwDeviceParas &deviceParas);
    int16_t YW_PolePlateRoomReset(const YinwDeviceParas &deviceParas);
    int16_t YW_DiscardedProductReset(const YinwDeviceParas &deviceParas);
    int16_t YW_QualifiedProductReset(const YinwDeviceParas &deviceParas);
    int16_t YW_WeldingShelfUpDownReset(const YinwDeviceParas &deviceParas);
    int16_t YW_WeldingShelfLeftRightReset(const YinwDeviceParas &deviceParas);

    //---------------------------------- 单步控制-------------------------------//
    // 1.银网上料 2.极片上料 3.设置作业要求 4.启动作业

    /**
     * fun：5. 银网给网及展平作业 应该是用在程序中直接写好的就行
     */
    int16_t YinwFeedinANDFlattening(const YinwDeviceParas &deviceParas);

    /**
     * fun：6. 18. 20. 22.焊台运动控制
     */
    bool WeldingDeskCtrl(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool WeldingDeskCtrlReset(const YinwDeviceParas &deviceParas, const char *CmdName); // 取走产品时后的复位用

    /**
     * fun：7. 17.焊架上下锁紧运动控制
     */
    bool WeldingDeskUpDownLock(const YinwDeviceParas &deviceParas, const char *CmdName);

    /**
     * fun：8. 16. 焊架左右运动控制
     */
    bool WeldingDeskLeftRightRun(const YinwDeviceParas &deviceParas, const char *CmdName);

    /**
     * fun：9. 11. 银网压制位运动控制
     */
    bool WeldingPressMove(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool WeldingPressMoveReset(const YinwDeviceParas &deviceParas, const char *CmdName);
    /**
     * fun：10.压机银网冲制
     */
    bool silverNetPunching(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool silverNetPunchGeneralFlow(const YinwDeviceParas &deviceParas);

    /**
     * fun：12. 三坐标从极片库取极片
     */
    bool polePlatePick(const YinwDeviceParas &deviceParas);
    bool polePlateSuckControl(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool polePlatePickAndSuckUp(const YinwDeviceParas &deviceParas);
    bool polePlatePickAndSuckUpThread(const YinwDeviceParas &deviceParas);

    /**
     * fun：13. 三坐标极片安置于焊台
     */
    bool polePlatePlace(const YinwDeviceParas &deviceParas);
    bool polePlatePickAndSuckDown(const YinwDeviceParas &deviceParas);

    /**
     * fun：14. 三坐标至银网压制位拾取银网
     */
    bool SliverNetPick(const YinwDeviceParas &deviceParas);
    bool SliverNetClamp(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool SliverNetPickAndClampClose(const YinwDeviceParas &deviceParas);

    /**
     * fun：15. 三坐标拾取单片银网至焊台
     */
    bool SliverNetPlace(const YinwDeviceParas &deviceParas);
    bool SliverNetPlaceAndClampOpen(const YinwDeviceParas &deviceParas);

    /**
     * fun：19.焊接过程
     */
    int16_t WeldingProcess(const char *deviceName, short MotorNUM, short MotorName);

    /**
     * fun：21.焊接在线检测
     */
    bool OnlineInspectionRecv(short Testflag);

    /**
     * fun：23.拾取产品
     */
    bool ProductPickUp(const YinwDeviceParas &deviceParas);
    bool ProductPickUpWait(const YinwDeviceParas &deviceParas);

    /**
     * fun：24.三座标废弃堆码
     */
    bool WasteStacking(const YinwDeviceParas &deviceParas);

    /**
     * fun：25.三座标合格堆码
     */
    bool QualifiedStacking(const YinwDeviceParas &deviceParas);

    /**
     * fun：26.展平机构移动
     */
    bool FlatteningMovement(const YinwDeviceParas &deviceParas, const char *CmdName);
    bool FlatteningStepMove(const YinwDeviceParas &deviceParas);

    /**
     * fun：27.极片库运动
     */
    bool polePlateRoomStep(const YinwDeviceParas &deviceParas);
    bool polePlateRoomStepping(const YinwDeviceParas &deviceParas);
    bool polePlateRoomReset(const YinwDeviceParas &deviceParas);

    /**
     * fun：28.合格品库运动
     */
    bool QualifiedRoom(const YinwDeviceParas &deviceParas);
    bool QualifiedRoomReset(const YinwDeviceParas &deviceParas);

    /**
     * fun：29.废品库运动
     */
    bool abandonedRoom(const YinwDeviceParas &deviceParas);
    bool abandonedRoomReset(const YinwDeviceParas &deviceParas);
    // fun：库的移动距离
    int16_t AllRoomMoveLen(const YinwDeviceParas &deviceParas, float thickness);

    // 没用
    /**
     * fun：三坐标设备控制 暂时没啥用!!!!!!!!!!!!
     */
    int16_t ThreeAxisDevicePlan(const YinwDeviceParas &deviceParas, const char *deviceName, short MotorNUM, int AImPosX, int AimPosY, int AimPosZ);

    /**
     * fun：零点矩阵的存储
     */
    void ZeroPointPosINISave(std::vector<std::vector<int>> ZeroPointValue);
    /**
     * fun：零点矩阵的存储
     */
    void SystemParasSettingSave(MsgHuman2RobFrame_ ParaSettingData);

    /**
     * fun：零点矩阵转存之后的读取
     */
    void ZeroPointPosINIRead(std::vector<std::vector<int>> &YWzeroNameANDpos);

    /**
     * fun：三坐标复位控制
     */
    bool ThreeAxisResetRun(const YinwDeviceParas &deviceParas);

    /**
     * fun：银网系统所有的标志位重置
     */
    int16_t YW_SystemFlowsReset();
    /**
     * fun：银网系统部分标志位重置
     */
    int16_t YW_PartlySystemFlowsReset();

    //-----钛虎----//
    /**
     * fun：1号钛虎电机步进运动
     */
    int16_t FirstTaiHuMotorStepControl(const YinwDeviceParas &deviceParas);

    /**
     * fun：2号钛虎电机步进运动
     */
    int16_t SecondTaiHuMotorStepControl(const YinwDeviceParas &deviceParas);

    /**
     * fun：4号钛虎电机步进运动
     */
    int16_t FourthTaiHuMotorStepControl(const YinwDeviceParas &deviceParas);

    /**
     * fun：存储控制流
     */
    void ControlStepflowSAVE(short value);
    /**
     * fun：控制流标志位重置
     */
    void ControlStepflowReset();

    /**
     * fun：银网电机故障检测
     */
    ET_MotorFaultInfo YinwET_MotorFaultDetct(const YinwDeviceParas &deviceParas, const char *deviceName, short SleepTime);
    void YinwSystemFaultDetct();
    int CombinedNum2Seen(int a, int b, int c);
};

#endif
