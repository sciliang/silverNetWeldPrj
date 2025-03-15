#ifndef YINWDEVICEPARAS
#define YINWDEVICEPARAS

#include <iostream>
#include <math.h>
#include <vector>
#include <array>
#include "TCP_Protocol.h"
class YinwDeviceParas
{
private:
    /* data */
    // 定义一个结构体来保存所有的展平参数
    // ----全局参数----
    struct Generalparams
    {
        int JinWangSuDu;            // 进网速度
        int DanJianChengPinShiJian; // 单件成平时间
        int ChuLiaoSuDu;            // 出料速度
        int ShuSongSuDu;            // 输送速度
        int MaxMovePos;             // 最大运动位置
        int MinMovePos;             // 最小运动位置
    };

    //----展平结构参数-----
    struct Flatten
    {
        int flatteningSteps;      // 展平运转的总步数
        int firstMotorApexPos;    // 1号电机顶点位置
        int secondMotorApexPos;   // 2号电机顶点位置
        int fourthMotorApexPos;   // 4号电机顶点位置
        int firstYWtAllThick;     // 1号电机上银网整个的厚度
        int fourthYWAllThick;     // 4号电机上银网整个的厚度
        int firstMotorCircleNum;  // 1号电机转圈的总数
        int fourthMotorCircleNum; // 4号电机转圈的总数
    };

    //----极片----
    struct PoleSheet
    {
        float JiPianHouDu;   // 极片厚度
        float JiPianNeiJing; // 极片内径
        float JiPianWaiJing; // 极片外径
    };

    //----银网----
    struct SilverNet
    {
        float YinWangHouDu;        // 银网厚度
        float YinWangSongLiaoSuDu; // 银网送料速度
        float YinWangNeiJing;      // 银网内径
        float YinWangWaiJing;      // 银网外径
        float YinWangZongChangDu;  // 银网总长度
    };

    //----焊接----
    struct Weld
    {
        int HanDianGaoDu;      // 焊点高度
        int HanJieWeiZhiFenBu; // 焊接位置分布
        int HanDianKuanDu;     // 焊点宽度
        int HanDianShuLiang;   // 焊点数量
    };

    //----银网压制位运动-钛虎-绝对位置--
    struct SilverNetPress
    {
        int SilverNetPressPos; // 银网压制的位置
        int SilverNetTransPos; // 银网送单片银网的位置
    };

    //----银网压制位运动-钛虎-绝对位置--
    struct ThreeDimensional
    {
        int PickUpPolePlateX;         // 取极片X
        int PickUpPolePlateY;         // 取极片Y
        int PickUpPolePlateZ;         // 取极片Z
        int TransSheet2WeldDeskX;     // 运输极片至焊台X
        int TransSheet2WeldDeskY;     // 运输极片至焊台Y
        int TransSheet2WeldDeskZ;     // 运输极片至焊台Z
        int PickUpSilverNetX;         // 拾取冲制好的银网X
        int PickUpSilverNetY;         // 拾取冲制好的银网Y
        int PickUpSilverNetZ;         // 拾取冲制好的银网Z
        int TransSilverNet2WeldDeskX; // 运输冲制好的银网到焊台X
        int TransSilverNet2WeldDeskY; // 运输冲制好的银网到焊台Y
        int TransSilverNet2WeldDeskZ; // 运输冲制好的银网到焊台Z
        int PickupProductX;           // 拾取产品X
        int PickupProductY;           // 拾取产品Y
        int PickupProductZ;           // 拾取产品Z
        int PlaceGoodProductX;        // 将成品放置成品库X
        int PlaceGoodProductY;        // 将成品放置成品库Y
        int PlaceGoodProductZ;        // 将成品放置成品库Z
        int PlaceBadProductX;         // 将废品放置废品库X
        int PlaceBadProductY;         // 将废品放置废品库Y
        int PlaceBadProductZ;         // 将废品放置废品库Z
        int XaxisHighest;             // X轴最右侧
        int XaxisHiglowest;           // X轴最左侧
        int YaxisHighest;             // Y轴最后侧
        int YaxisHiglowest;           // Y轴最前侧
        int ZaxisHighest;             // Z轴最高点
        int ZaxisHiglowest;           // Z轴最低点
    };

    //----银网压制位运动-钛虎-绝对位置--
    struct weldingfixture
    {
        int WeldPreparePos;           // 初始位置A,焊接准备位
        int WeldingPos;               // 中间位置B,焊接位置
        int WeldDetectPos;            // 中间位置C,检测位置
        int WeldProductPos;           // 末端位置D,产品取走位置
        int WeldingframelockingPos;   // 焊架上下运动锁紧位置
        int WeldingframetranslatePos; // 焊架左右运动锁紧位置
    };

    //----焊台误差补偿----
    struct weldingfixtureErrorCompensation
    {
        int weldingfixtureVelA;   // 焊台速度A
        int weldingfixtureErrorA; // 焊台速度A对应的补偿
        int weldingfixtureVelB;   // 焊台速度B
        int weldingfixtureErrorB; // 焊台速度B对应的补偿
        int weldingfixtureVelC;   // 焊台速度C
        int weldingfixtureErrorC; // 焊台速度C对应的补偿
        int weldingfixtureVelD;   // 焊台速度D
        int weldingfixtureErrorD; // 焊台速度D对应的补偿
    };

    //----焊架上下锁紧误差补偿----
    struct WeldingframelockingErrorCompensation
    {
        int WeldingframelockingVelA;   // 焊架上下锁紧速度A
        int WeldingframelockingErrorA; // 焊架上下锁紧速度A对应的补偿
        int WeldingframelockingVelB;   // 焊架上下锁紧速度B
        int WeldingframelockingErrorB; // 焊架上下锁紧速度B对应的补偿
        int WeldingframelockingVelC;   // 焊架上下锁紧速度C
        int WeldingframelockingErrorC; // 焊架上下锁紧速度C对应的补偿
        int WeldingframelockingVelD;   // 焊架上下锁紧速度D
        int WeldingframelockingErrorD; // 焊架上下锁紧速度D对应的补偿
    };

    //----焊架左右运输误差补偿----
    struct WeldingframetranslateErrorCompensation
    {
        int WeldingframetranslateVelA;   // 焊架左右运输速度A
        int WeldingframetranslateErrorA; // 焊架左右运输速度A对应的补偿
        int WeldingframetranslateVelB;   // 焊架左右运输速度B
        int WeldingframetranslateErrorB; // 焊架左右运输速度B对应的补偿
        int WeldingframetranslateVelC;   // 焊架左右运输速度C
        int WeldingframetranslateErrorC; // 焊架左右运输速度C对应的补偿
        int WeldingframetranslateVelD;   // 焊架左右运输速度D
        int WeldingframetranslateErrorD; // 焊架左右运输速度D对应的补偿
    };

    //----夹爪开的位置-和关的位置-绝对位置---
    struct ClampAimPos
    {
        int ClampOpenPos;
        int ClampClosePos;
    };

    //----冲压运动-相对位置---
    struct Stamping
    {
        int ChongYaUpWaitPos;    // 压机悬停位置A
        int ChongYaDownPressPos; // 压机重压位置B
        int ChongYaJuLi;         // 冲压距离
    };

    //----冲压误差补偿----
    struct StampingErrorCompensation
    {
        int StampingVelA;   // 冲压速度A
        int StampingErrorA; // 冲压速度A对应的补偿
        int StampingVelB;   // 冲压速度B
        int StampingErrorB; // 冲压速度B对应的补偿
        int StampingVelC;   // 冲压速度C
        int StampingErrorC; // 冲压速度C对应的补偿
        int StampingVelD;   // 冲压速度D
        int StampingErrorD; // 冲压速度D对应的补偿
    };

    //---存储的库运动--相对位置---
    struct storeroom
    {
        int PolePlateUpPos;     // 极片库上面的限位
        int PolePlateDownPos;   // 极片库下面的限位
        int GoodProductUpPos;   // 成品库上面的限位
        int GoodProductDownPos; // 成品库下面的限位
        int BadProductUpPos;    // 废品库上面的限位
        int BadProductDownPos;  // 废品库下面的限位
    };

    //---存储的库运动--相对位置-误差补偿--
    struct storeroomErrorCompensation
    {
        int storeroomVelA;   // 存储库速度A
        int storeroomErrorA; // 存储库速度A对应的补偿
        int storeroomVelB;   // 存储库速度B
        int storeroomErrorB; // 存储库速度B对应的补偿
        int storeroomVelC;   // 存储库速度C
        int storeroomErrorC; // 存储库速度C对应的补偿
        int storeroomVelD;   // 存储库速度D
        int storeroomErrorD; // 存储库速度D对应的补偿
    };

    //----展平机构平移运动--相对位置---
    struct FlattenMove
    {
        int FlattenMoveLeftPos;  // 展平移动最左边的位置
        int FlattenMoveRightPos; // 展平移动最右边的位置
    };

    //---存储的库运动--相对位置--误差补偿--
    struct FlattenMoveErrorCompensation
    {
        int FlattenMoveVelA;   // 展平移动速度A
        int FlattenMoveErrorA; // 展平移动速度A对应的补偿
        int FlattenMoveVelB;   // 展平移动速度B
        int FlattenMoveErrorB; // 展平移动速度B对应的补偿
        int FlattenMoveVelC;   // 展平移动速度C
        int FlattenMoveErrorC; // 展平移动速度C对应的补偿
        int FlattenMoveVelD;   // 展平移动速度D
        int FlattenMoveErrorD; // 展平移动速度D对应的补偿
    };

    //----钛虎电机----
    struct TaiHu
    {
        int encoder;        // 钛虎编码器
        int ReductionRatio; // 减速比
        int MotorDiameter;  // 电机直径
    };

    //----所有电机额定速度----
    struct ALLMotorsRatedVelocity
    {
        int YinwForgingMotorRatedVel;
        int ThreeAxisYmotorRatedVel;
        int ThreeAxisXmotorRatedVel;
        int YinwFeedinANDFlatteningMOVERatedVel;
        int PolarPlateWarehouseMoveRatedVel;
        int QualifiedProductWareHouseRatedVel;
        int DiscardedProductWareHouseRatedVel;
        int ThreeAxisZmotorRatedVel;
        int WeldingPosMoveRatedVel;
        int WeldingShelfLeftRightRatedVel;
        int WeldingShelfUpDownRatedVel;
        int TaihuFirstRatedVel;
        int TaihuSecondRatedVel;
        int TaihuThirdRatedVel;
        int TaihuFourthRatedVel;
        int TaihuFifthRatedVel;
    };

    //----所有电机额定电流----
    struct ALLMotorsRatedCurrent
    {
        int YinwForgingMotorRatedCur;
        int ThreeAxisYmotorRatedCur;
        int ThreeAxisXmotorRatedCur;
        int YinwFeedinANDFlatteningMOVERatedCur;
        int PolarPlateWarehouseMoveRatedCur;
        int QualifiedProductWareHouseRatedCur;
        int DiscardedProductWareHouseRatedCur;
        int ThreeAxisZmotorRatedCur;
        int WeldingPosMoveRatedCur;
        int WeldingShelfLeftRightRatedCur;
        int WeldingShelfUpDownRatedCur;
        int TaihuFirstRatedCur;
        int TaihuSecondRatedCur;
        int TaihuThirdRatedCur;
        int TaihuFourthRatedCur;
        int TaihuFifthRatedCur;
    };

    //----所有电机编码器的值----
    struct ALLMotorsEncoder
    {
        int YinwForgingMotorEncoder;
        int ThreeAxisYmotorEncoder;
        int ThreeAxisXmotorEncoder;
        int YinwFeedinANDFlatteningMOVEEncoder;
        int PolarPlateWarehouseMoveEncoder;
        int QualifiedProductWareHouseEncoder;
        int DiscardedProductWareHouseEncoder;
        int ThreeAxisZmotorEncoder;
        int WeldingPosMoveEncoder;
        int WeldingShelfLeftRightEncoder;
        int WeldingShelfUpDownEncoder;
        int TaihuFirstEncoder;
        int TaihuSecondEncoder;
        int TaihuThirdEncoder;
        int TaihuFourthEncoder;
        int TaihuFifthEncoder;
    };

public:
    struct YW_DeviceParas
    {
        Generalparams Generalparams_;
        Flatten Flatten_;
        PoleSheet PoleSheet_;
        SilverNet SilverNet_;
        Weld Weld_;
        SilverNetPress SilverNetPress_;
        ThreeDimensional ThreeDimensional_;
        weldingfixture weldingfixture_;
        weldingfixtureErrorCompensation weldingfixtureErrorCompensation_;
        WeldingframelockingErrorCompensation WeldingframelockingErrorCompensation_;
        WeldingframetranslateErrorCompensation WeldingframetranslateErrorCompensation_;
        Stamping Stamping_;
        ClampAimPos ClampAimPos_;
        StampingErrorCompensation StampingErrorCompensation_;
        storeroom storeroom_;
        storeroomErrorCompensation storeroomErrorCompensation_;
        FlattenMove FlattenMove_;
        FlattenMoveErrorCompensation FlattenMoveErrorCompensation_;
        TaiHu TaiHu_;
        ALLMotorsRatedVelocity ALLMotorsRatedVelocity_;
        ALLMotorsRatedCurrent ALLMotorsRatedCurrent_;
        ALLMotorsEncoder ALLMotorsEncoder_;
    };
    void loadConfig();
    YW_DeviceParas Paras;
    YinwDeviceParas(/* args */);
    ~YinwDeviceParas();
    int16_t YW_FlattenParaSave(YW_DeviceParas YW_DeviceParas_);
    int16_t ZeroPointPosINISave(std::vector<std::vector<int>> ZeroPointValue);
    int16_t SystemSettingParasSave(MsgHuman2RobFrame_ ParaSettingData);
    std::array<std::array<int, 2>, 8> loadZeroNameAndPos();
};
#endif