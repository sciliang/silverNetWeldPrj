#include <iostream>
#include <string>
#include "YinwDeviceParas.h"
#include <glog/logging.h>
#include "IniParser.h"
#include <unistd.h> // getcwd
#include <limits.h> // PATH_MAX
YinwDeviceParas::YinwDeviceParas(/* args */)
{
    loadConfig();
}

YinwDeviceParas::~YinwDeviceParas()
{
}

// 展平结构参数更新
void YinwDeviceParas::loadConfig()
{
    IniParser ini;
    char cwd[PATH_MAX];
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        std::string configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());
        // ini.load("/home/sia/workspace/YWHJ_20250103/Robotbase/config/config.ini");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    LOG(INFO) << "<<=====================ALL Parameters===================>>" << std::endl;
    //  全局参数
    Paras.Generalparams_.JinWangSuDu = std::stoi(ini.get("Generalparams", "JinWangSuDu", "defaultUser"));
    Paras.Generalparams_.DanJianChengPinShiJian = std::stoi(ini.get("Generalparams", "DanJianChengPinShiJian", "defaultUser"));
    Paras.Generalparams_.ChuLiaoSuDu = std::stoi(ini.get("Generalparams", "ChuLiaoSuDu", "defaultUser"));
    Paras.Generalparams_.ShuSongSuDu = std::stoi(ini.get("Generalparams", "ShuSongSuDu", "defaultUser"));
    Paras.Generalparams_.MaxMovePos = std::stoi(ini.get("Generalparams", "MaxMovePos", "defaultUser"));
    Paras.Generalparams_.MinMovePos = std::stoi(ini.get("Generalparams", "MinMovePos", "defaultUser"));
    LOG(INFO) << "Generalparams_.JinWangSuDu = " << Paras.Generalparams_.JinWangSuDu << std::endl
              << "Generalparams_.DanJianChengPinShiJian = " << Paras.Generalparams_.DanJianChengPinShiJian << std::endl
              << "Generalparams_.ChuLiaoSuDu = " << Paras.Generalparams_.ChuLiaoSuDu << std::endl
              << "Generalparams_.ShuSongSuDu = " << Paras.Generalparams_.ShuSongSuDu << std::endl
              << "Generalparams_.MaxMovePos = " << Paras.Generalparams_.MaxMovePos << std::endl
              << "Generalparams_.MinMovePos = " << Paras.Generalparams_.MinMovePos << std::endl;
    //  展平结构参数,所有的这些都是一个批次的参数变量，也就是从一卷新的银网安装之后开始算起
    //  当前展平的步数
    Paras.Flatten_.flatteningSteps = std::stoi(ini.get("Flatten", "flatteningSteps", "defaultUser"));
    // 第一台电机顶点的位置
    Paras.Flatten_.firstMotorApexPos = std::stoi(ini.get("Flatten", "firstMotorApexPos", "defaultUser"));
    // 第二台电机顶点的位置
    Paras.Flatten_.secondMotorApexPos = std::stoi(ini.get("Flatten", "secondMotorApexPos", "defaultUser"));
    // 第四台电机顶点的位置
    Paras.Flatten_.fourthMotorApexPos = std::stoi(ini.get("Flatten", "fourthMotorApexPos", "defaultUser"));
    // 第一台电机极片整个厚度(包含电机轴径)
    Paras.Flatten_.firstYWtAllThick = std::stoi(ini.get("Flatten", "firstYWtAllThick", "defaultUser"));
    // 第四台电机极片整个厚度(包含电机的轴径)
    Paras.Flatten_.fourthYWAllThick = std::stoi(ini.get("Flatten", "fourthYWAllThick", "defaultUser"));
    // 第一台电机旋转的总圈数
    Paras.Flatten_.firstMotorCircleNum = std::stoi(ini.get("Flatten", "firstMotorCircleNum", "defaultUser"));
    // 第四台电机旋转的总圈数
    Paras.Flatten_.fourthMotorCircleNum = std::stoi(ini.get("Flatten", "fourthMotorCircleNum", "defaultUser"));
    LOG(INFO) << "Flatten_.flatteningSteps = " << Paras.Flatten_.flatteningSteps << std::endl
              << "Flatten_.firstMotorApexPos = " << Paras.Flatten_.firstMotorApexPos << std::endl
              << "Flatten_.secondMotorApexPos = " << Paras.Flatten_.secondMotorApexPos << std::endl
              << "Flatten_.fourthMotorApexPos = " << Paras.Flatten_.fourthMotorApexPos << std::endl
              << "Flatten_.firstYWtAllThick = " << Paras.Flatten_.firstYWtAllThick << std::endl
              << "Flatten_.fourthYWAllThick = " << Paras.Flatten_.fourthYWAllThick << std::endl
              << "Flatten_.firstMotorCircleNum = " << Paras.Flatten_.firstMotorCircleNum << std::endl
              << "Flatten_.fourthMotorCircleNum = " << Paras.Flatten_.fourthMotorCircleNum << std::endl;

    //----极片----
    Paras.PoleSheet_.JiPianHouDu = std::stof(ini.get("PoleSheet", "JiPianHouDu", "defaultUser"));
    Paras.PoleSheet_.JiPianNeiJing = std::stof(ini.get("PoleSheet", "JiPianNeiJing", "defaultUser"));
    Paras.PoleSheet_.JiPianWaiJing = std::stof(ini.get("PoleSheet", "JiPianWaiJing", "defaultUser"));
    LOG(INFO) << "PoleSheet_.JiPianHouDu = " << Paras.PoleSheet_.JiPianHouDu << std::endl
              << "PoleSheet_.JiPianNeiJing = " << Paras.PoleSheet_.JiPianNeiJing << std::endl
              << "PoleSheet_.JiPianWaiJing = " << Paras.PoleSheet_.JiPianWaiJing << std::endl;

    //----银网----
    // 银网的厚度
    Paras.SilverNet_.YinWangHouDu = std::stof(ini.get("SilverNet", "YinWangHouDu", "defaultUser"));
    // 银网的外径
    Paras.SilverNet_.YinWangSongLiaoSuDu = std::stof(ini.get("SilverNet", "YinWangSongLiaoSuDu", "defaultUser"));
    // 钛虎电机的编码器
    Paras.SilverNet_.YinWangNeiJing = std::stof(ini.get("SilverNet", "YinWangNeiJing", "defaultUser"));
    // 钛虎电机的编码器
    Paras.SilverNet_.YinWangWaiJing = std::stof(ini.get("SilverNet", "YinWangWaiJing", "defaultUser"));
    // 钛虎电机的轴径
    Paras.SilverNet_.YinWangZongChangDu = std::stof(ini.get("SilverNet", "YinWangZongChangDu", "defaultUser"));
    LOG(INFO) << "SilverNet_.YinWangHouDu = " << Paras.SilverNet_.YinWangHouDu << std::endl
              << "SilverNet_.YinWangSongLiaoSuDu = " << Paras.SilverNet_.YinWangSongLiaoSuDu << std::endl
              << "SilverNet_.YinWangNeiJing = " << Paras.SilverNet_.YinWangNeiJing << std::endl
              << "SilverNet_.YinWangWaiJing = " << Paras.SilverNet_.YinWangWaiJing << std::endl
              << "SilverNet_.YinWangZongChangDu = " << Paras.SilverNet_.YinWangZongChangDu << std::endl;

    //----焊接----
    Paras.Weld_.HanDianGaoDu = std::stoi(ini.get("Weld", "HanDianGaoDu", "defaultUser"));
    Paras.Weld_.HanJieWeiZhiFenBu = std::stoi(ini.get("Weld", "HanJieWeiZhiFenBu", "defaultUser"));
    Paras.Weld_.HanDianKuanDu = std::stoi(ini.get("Weld", "HanDianKuanDu", "defaultUser"));
    Paras.Weld_.HanDianShuLiang = std::stoi(ini.get("Weld", "HanDianShuLiang", "defaultUser"));
    LOG(INFO) << "Weld_.HanDianGaoDu = " << Paras.Weld_.HanDianGaoDu << std::endl
              << "Weld_.HanJieWeiZhiFenBu = " << Paras.Weld_.HanJieWeiZhiFenBu << std::endl
              << "Weld_.HanDianKuanDu = " << Paras.Weld_.HanDianKuanDu << std::endl
              << "Weld_.HanDianShuLiang = " << Paras.Weld_.HanDianShuLiang << std::endl;

    //----银网压制位运动-钛虎-绝对位置--
    Paras.SilverNetPress_.SilverNetPressPos = std::stoi(ini.get("SilverNetPress", "SilverNetPressPos", "defaultUser"));
    Paras.SilverNetPress_.SilverNetTransPos = std::stoi(ini.get("SilverNetPress", "SilverNetTransPos", "defaultUser"));
    LOG(INFO) << "SilverNetPress_.SilverNetPressPos = " << Paras.SilverNetPress_.SilverNetPressPos << std::endl
              << "SilverNetPress_.SilverNetTransPos = " << Paras.SilverNetPress_.SilverNetTransPos << std::endl;

    //----夹爪抓取银网-钛虎-绝对位置--
    Paras.ClampAimPos_.ClampOpenPos = std::stoi(ini.get("clamp", "clampOpenPos", "defaultUser"));
    Paras.ClampAimPos_.ClampClosePos = std::stoi(ini.get("clamp", "clampClosePos", "defaultUser"));
    LOG(INFO) << "ClampAimPos_.ClampOpenPos = " << Paras.ClampAimPos_.ClampOpenPos << std::endl
              << "ClampAimPos_.ClampClosePos = " << Paras.ClampAimPos_.ClampClosePos << std::endl;

    //----三坐标的-绝对位置--
    Paras.ThreeDimensional_.PickUpPolePlateX = std::stoi(ini.get("ThreeDimensional", "PickUpPolePlateX", "defaultUser"));
    Paras.ThreeDimensional_.PickUpPolePlateY = std::stoi(ini.get("ThreeDimensional", "PickUpPolePlateY", "defaultUser"));
    Paras.ThreeDimensional_.PickUpPolePlateZ = std::stoi(ini.get("ThreeDimensional", "PickUpPolePlateZ", "defaultUser"));
    Paras.ThreeDimensional_.TransSheet2WeldDeskX = std::stoi(ini.get("ThreeDimensional", "TransSheet2WeldDeskX", "defaultUser"));
    Paras.ThreeDimensional_.TransSheet2WeldDeskY = std::stoi(ini.get("ThreeDimensional", "TransSheet2WeldDeskY", "defaultUser"));
    Paras.ThreeDimensional_.TransSheet2WeldDeskZ = std::stoi(ini.get("ThreeDimensional", "TransSheet2WeldDeskZ", "defaultUser"));
    Paras.ThreeDimensional_.PickUpSilverNetX = std::stoi(ini.get("ThreeDimensional", "PickUpSilverNetX", "defaultUser"));
    Paras.ThreeDimensional_.PickUpSilverNetY = std::stoi(ini.get("ThreeDimensional", "PickUpSilverNetY", "defaultUser"));
    Paras.ThreeDimensional_.PickUpSilverNetZ = std::stoi(ini.get("ThreeDimensional", "PickUpSilverNetZ", "defaultUser"));
    Paras.ThreeDimensional_.TransSilverNet2WeldDeskX = std::stoi(ini.get("ThreeDimensional", "TransSilverNet2WeldDeskX", "defaultUser"));
    Paras.ThreeDimensional_.TransSilverNet2WeldDeskY = std::stoi(ini.get("ThreeDimensional", "TransSilverNet2WeldDeskY", "defaultUser"));
    Paras.ThreeDimensional_.TransSilverNet2WeldDeskZ = std::stoi(ini.get("ThreeDimensional", "TransSilverNet2WeldDeskZ", "defaultUser"));
    Paras.ThreeDimensional_.PickupProductX = std::stoi(ini.get("ThreeDimensional", "PickupProductX", "defaultUser"));
    Paras.ThreeDimensional_.PickupProductY = std::stoi(ini.get("ThreeDimensional", "PickupProductY", "defaultUser"));
    Paras.ThreeDimensional_.PickupProductZ = std::stoi(ini.get("ThreeDimensional", "PickupProductZ", "defaultUser"));
    Paras.ThreeDimensional_.PlaceGoodProductX = std::stoi(ini.get("ThreeDimensional", "PlaceGoodProductX", "defaultUser"));
    Paras.ThreeDimensional_.PlaceGoodProductY = std::stoi(ini.get("ThreeDimensional", "PlaceGoodProductY", "defaultUser"));
    Paras.ThreeDimensional_.PlaceGoodProductZ = std::stoi(ini.get("ThreeDimensional", "PlaceGoodProductZ", "defaultUser"));
    Paras.ThreeDimensional_.PlaceBadProductX = std::stoi(ini.get("ThreeDimensional", "PlaceBadProductX", "defaultUser"));
    Paras.ThreeDimensional_.PlaceBadProductY = std::stoi(ini.get("ThreeDimensional", "PlaceBadProductY", "defaultUser"));
    Paras.ThreeDimensional_.PlaceBadProductZ = std::stoi(ini.get("ThreeDimensional", "PlaceBadProductZ", "defaultUser"));
    Paras.ThreeDimensional_.XaxisHighest = std::stoi(ini.get("ThreeDimensional", "XaxisHighest", "defaultUser"));
    Paras.ThreeDimensional_.XaxisHiglowest = std::stoi(ini.get("ThreeDimensional", "XaxisHiglowest", "defaultUser"));
    Paras.ThreeDimensional_.YaxisHighest = std::stoi(ini.get("ThreeDimensional", "YaxisHighest", "defaultUser"));
    Paras.ThreeDimensional_.YaxisHiglowest = std::stoi(ini.get("ThreeDimensional", "YaxisHiglowest", "defaultUser"));
    Paras.ThreeDimensional_.ZaxisHighest = std::stoi(ini.get("ThreeDimensional", "ZaxisHighest", "defaultUser"));
    Paras.ThreeDimensional_.ZaxisHiglowest = std::stoi(ini.get("ThreeDimensional", "ZaxisHiglowest", "defaultUser"));

    LOG(INFO) << "ThreeDimensional_.PickUpPolePlateX = " << Paras.ThreeDimensional_.PickUpPolePlateX << std::endl
              << "ThreeDimensional_.PickUpPolePlateY = " << Paras.ThreeDimensional_.PickUpPolePlateY << std::endl
              << "ThreeDimensional_.PickUpPolePlateZ = " << Paras.ThreeDimensional_.PickUpPolePlateZ << std::endl
              << "ThreeDimensional_.TransSheet2WeldDeskX = " << Paras.ThreeDimensional_.TransSheet2WeldDeskX << std::endl
              << "ThreeDimensional_.TransSheet2WeldDeskY = " << Paras.ThreeDimensional_.TransSheet2WeldDeskY << std::endl
              << "ThreeDimensional_.TransSheet2WeldDeskZ = " << Paras.ThreeDimensional_.TransSheet2WeldDeskZ << std::endl
              << "ThreeDimensional_.PickUpSilverNetX = " << Paras.ThreeDimensional_.PickUpSilverNetX << std::endl
              << "ThreeDimensional_.PickUpSilverNetY = " << Paras.ThreeDimensional_.PickUpSilverNetY << std::endl
              << "ThreeDimensional_.PickUpSilverNetZ = " << Paras.ThreeDimensional_.PickUpSilverNetZ << std::endl
              << "ThreeDimensional_.TransSilverNet2WeldDeskX = " << Paras.ThreeDimensional_.TransSilverNet2WeldDeskX << std::endl
              << "ThreeDimensional_.TransSilverNet2WeldDeskY = " << Paras.ThreeDimensional_.TransSilverNet2WeldDeskY << std::endl
              << "ThreeDimensional_.TransSilverNet2WeldDeskZ = " << Paras.ThreeDimensional_.TransSilverNet2WeldDeskZ << std::endl
              << "ThreeDimensional_.PickupProductX = " << Paras.ThreeDimensional_.PickupProductX << std::endl
              << "ThreeDimensional_.PickupProductY = " << Paras.ThreeDimensional_.PickupProductY << std::endl
              << "ThreeDimensional_.PickupProductZ = " << Paras.ThreeDimensional_.PickupProductZ << std::endl
              << "ThreeDimensional_.PlaceGoodProductX = " << Paras.ThreeDimensional_.PlaceGoodProductX << std::endl
              << "ThreeDimensional_.PlaceGoodProductY = " << Paras.ThreeDimensional_.PlaceGoodProductY << std::endl
              << "ThreeDimensional_.PlaceGoodProductZ = " << Paras.ThreeDimensional_.PlaceGoodProductZ << std::endl
              << "ThreeDimensional_.PlaceBadProductX = " << Paras.ThreeDimensional_.PlaceBadProductX << std::endl
              << "ThreeDimensional_.PlaceBadProductY = " << Paras.ThreeDimensional_.PlaceBadProductY << std::endl
              << "ThreeDimensional_.PlaceBadProductZ = " << Paras.ThreeDimensional_.PlaceBadProductZ << std::endl
              << "ThreeDimensional_.XaxisHighest = " << Paras.ThreeDimensional_.XaxisHighest << std::endl
              << "ThreeDimensional_.XaxisHiglowest = " << Paras.ThreeDimensional_.XaxisHiglowest << std::endl
              << "ThreeDimensional_.YaxisHighest = " << Paras.ThreeDimensional_.YaxisHighest << std::endl
              << "ThreeDimensional_.YaxisHiglowest = " << Paras.ThreeDimensional_.YaxisHiglowest << std::endl
              << "ThreeDimensional_.ZaxisHiglowest = " << Paras.ThreeDimensional_.ZaxisHiglowest << std::endl
              << "ThreeDimensional_.ZaxisHiglowest = " << Paras.ThreeDimensional_.ZaxisHiglowest << std::endl;

    //----焊台的-相对位置--
    Paras.weldingfixture_.WeldPreparePos = std::stoi(ini.get("weldingfixture", "WeldPreparePos", "defaultUser"));
    Paras.weldingfixture_.WeldingPos = std::stoi(ini.get("weldingfixture", "WeldingPos", "defaultUser"));
    Paras.weldingfixture_.WeldDetectPos = std::stoi(ini.get("weldingfixture", "WeldDetectPos", "defaultUser"));
    Paras.weldingfixture_.WeldProductPos = std::stoi(ini.get("weldingfixture", "WeldProductPos", "defaultUser"));
    Paras.weldingfixture_.WeldingframelockingPos = std::stoi(ini.get("weldingfixture", "WeldingframelockingPos", "defaultUser"));
    Paras.weldingfixture_.WeldingframetranslatePos = std::stoi(ini.get("weldingfixture", "WeldingframetranslatePos", "defaultUser"));
    LOG(INFO) << "weldingfixture_.WeldPreparePos = " << Paras.weldingfixture_.WeldPreparePos << std::endl
              << "weldingfixture_.WeldingPos = " << Paras.weldingfixture_.WeldingPos << std::endl
              << "weldingfixture_.WeldDetectPos = " << Paras.weldingfixture_.WeldDetectPos << std::endl
              << "weldingfixture_.WeldProjectPos = " << Paras.weldingfixture_.WeldProductPos << std::endl
              << "weldingfixture_.WeldingframelockingPos = " << Paras.weldingfixture_.WeldingframelockingPos << std::endl
              << "weldingfixture_.WeldingframetranslatePos = " << Paras.weldingfixture_.WeldingframetranslatePos << std::endl;

    //----焊台的-运动速度和匹配的误差补偿--
    Paras.weldingfixtureErrorCompensation_.weldingfixtureVelA = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureVelA", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorA = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureErrorA", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureVelB = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureVelB", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorB = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureErrorB", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureVelC = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureVelC", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorC = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureErrorC", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureVelD", "defaultUser"));
    Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorD = std::stoi(ini.get("weldingfixtureErrorCompensation", "weldingfixtureErrorD", "defaultUser"));
    LOG(INFO) << "weldingfixtureErrorCompensation_.weldingfixtureVelA = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureVelA << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureErrorA = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorA << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureVelB = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureVelB << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureErrorB = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorB << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureVelC = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureVelC << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureErrorC = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorC << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureVelD = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureVelD << std::endl
              << "weldingfixtureErrorCompensation_.weldingfixtureErrorD = " << Paras.weldingfixtureErrorCompensation_.weldingfixtureErrorD << std::endl;

    //----焊架上下锁紧的-运动速度和匹配的误差补偿--
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingVelA", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorA = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingErrorA", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelB = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingVelB", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorB = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingErrorB", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelC = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingVelC", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorC = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingErrorC", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelD = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingVelD", "defaultUser"));
    Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorD = std::stoi(ini.get("WeldingframelockingErrorCompensation", "WeldingframelockingErrorD", "defaultUser"));
    LOG(INFO) << "WeldingframelockingErrorCompensation_.weldingfixtureVelA = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelA << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureErrorA = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorA << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureVelB = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelB << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureErrorB = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorB << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureVelC = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelC << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureErrorC = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorC << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureVelD = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingVelD << std::endl
              << "WeldingframelockingErrorCompensation_.weldingfixtureErrorD = " << Paras.WeldingframelockingErrorCompensation_.WeldingframelockingErrorD << std::endl;

    //----焊架左右运动的-运动速度和匹配的误差补偿--
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateVelA", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorA = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateErrorA", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelB = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateVelB", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorB = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateErrorB", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelC = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateVelC", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorC = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateErrorC", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelD = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateVelD", "defaultUser"));
    Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorD = std::stoi(ini.get("WeldingframetranslateErrorCompensation", "WeldingframetranslateErrorD", "defaultUser"));
    LOG(INFO) << "WeldingframetranslateErrorCompensation_.weldingfixtureVelA = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelA << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureErrorA = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorA << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureVelB = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelB << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureErrorB = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorB << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureVelC = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelC << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureErrorC = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorC << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureVelD = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateVelD << std::endl
              << "WeldingframetranslateErrorCompensation_.weldingfixtureErrorD = " << Paras.WeldingframetranslateErrorCompensation_.WeldingframetranslateErrorD << std::endl;

    //----冲压运动-相对位置---
    Paras.Stamping_.ChongYaUpWaitPos = std::stoi(ini.get("Stamping", "ChongYaUpWaitPos", "defaultUser"));
    Paras.Stamping_.ChongYaDownPressPos = std::stoi(ini.get("Stamping", "ChongYaDownPressPos", "defaultUser"));
    Paras.Stamping_.ChongYaJuLi = std::stoi(ini.get("Stamping", "ChongYaJuLi", "defaultUser"));
    LOG(INFO) << "Stamping_.ChongYaUpWaitPos = " << Paras.Stamping_.ChongYaUpWaitPos << std::endl
              << "Stamping_.ChongYaDownPressPos = " << Paras.Stamping_.ChongYaDownPressPos << std::endl
              << "Stamping_.ChongYaJuLi = " << Paras.Stamping_.ChongYaJuLi << std::endl;

    //----冲压运动-运动速度和匹配的误差补偿--
    Paras.StampingErrorCompensation_.StampingVelA = std::stoi(ini.get("StampingErrorCompensation", "StampingVelA", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingErrorA = std::stoi(ini.get("StampingErrorCompensation", "StampingErrorA", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingVelB = std::stoi(ini.get("StampingErrorCompensation", "StampingVelB", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingErrorB = std::stoi(ini.get("StampingErrorCompensation", "StampingErrorB", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingVelC = std::stoi(ini.get("StampingErrorCompensation", "StampingVelC", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingErrorC = std::stoi(ini.get("StampingErrorCompensation", "StampingErrorC", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingVelD = std::stoi(ini.get("StampingErrorCompensation", "StampingVelD", "defaultUser"));
    Paras.StampingErrorCompensation_.StampingErrorD = std::stoi(ini.get("StampingErrorCompensation", "StampingErrorD", "defaultUser"));
    LOG(INFO) << "StampingErrorCompensation_.StampingVelA = " << Paras.StampingErrorCompensation_.StampingVelA << std::endl
              << "StampingErrorCompensation_.StampingErrorA = " << Paras.StampingErrorCompensation_.StampingErrorA << std::endl
              << "StampingErrorCompensation_.StampingVelB = " << Paras.StampingErrorCompensation_.StampingVelB << std::endl
              << "StampingErrorCompensation_.StampingErrorB = " << Paras.StampingErrorCompensation_.StampingErrorB << std::endl
              << "StampingErrorCompensation_.StampingVelC = " << Paras.StampingErrorCompensation_.StampingVelC << std::endl
              << "StampingErrorCompensation_.StampingErrorC = " << Paras.StampingErrorCompensation_.StampingErrorC << std::endl
              << "StampingErrorCompensation_.StampingVelD = " << Paras.StampingErrorCompensation_.StampingVelD << std::endl
              << "StampingErrorCompensation_.StampingErrorD = " << Paras.StampingErrorCompensation_.StampingErrorD << std::endl;

    //---存储的库运动--相对位置---
    Paras.storeroom_.PolePlateUpPos = std::stoi(ini.get("storeroom", "PolePlateUpPos", "defaultUser"));
    Paras.storeroom_.PolePlateDownPos = std::stoi(ini.get("storeroom", "PolePlateDownPos", "defaultUser"));
    Paras.storeroom_.GoodProductUpPos = std::stoi(ini.get("storeroom", "GoodProductUpPos", "defaultUser"));
    Paras.storeroom_.GoodProductDownPos = std::stoi(ini.get("storeroom", "GoodProductDownPos", "defaultUser"));
    Paras.storeroom_.BadProductUpPos = std::stoi(ini.get("storeroom", "BadProductUpPos", "defaultUser"));
    Paras.storeroom_.BadProductDownPos = std::stoi(ini.get("storeroom", "BadProductDownPos", "defaultUser"));
    LOG(INFO) << "storeroom_.PolePlateUpPos = " << Paras.storeroom_.PolePlateUpPos << std::endl
              << "storeroom_.PolePlateDownPos = " << Paras.storeroom_.PolePlateDownPos << std::endl
              << "storeroom_.GoodProductUpPos = " << Paras.storeroom_.GoodProductUpPos << std::endl
              << "storeroom_.GoodProductDownPos = " << Paras.storeroom_.GoodProductDownPos << std::endl
              << "storeroom_.BadProductUpPos = " << Paras.storeroom_.BadProductUpPos << std::endl
              << "storeroom_.BadProductDownPos = " << Paras.storeroom_.BadProductDownPos << std::endl;

    //----存储库运动-运动速度和匹配的误差补偿--
    Paras.storeroomErrorCompensation_.storeroomVelA = std::stoi(ini.get("storeroomErrorCompensation", "storeroomVelA", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomErrorA = std::stoi(ini.get("storeroomErrorCompensation", "storeroomErrorA", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomVelB = std::stoi(ini.get("storeroomErrorCompensation", "storeroomVelB", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomErrorB = std::stoi(ini.get("storeroomErrorCompensation", "storeroomErrorB", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomVelC = std::stoi(ini.get("storeroomErrorCompensation", "storeroomVelC", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomErrorC = std::stoi(ini.get("storeroomErrorCompensation", "storeroomErrorC", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomVelD = std::stoi(ini.get("storeroomErrorCompensation", "storeroomVelD", "defaultUser"));
    Paras.storeroomErrorCompensation_.storeroomErrorD = std::stoi(ini.get("storeroomErrorCompensation", "storeroomErrorD", "defaultUser"));
    LOG(INFO) << "storeroomErrorCompensation_.storeroomVelA = " << Paras.storeroomErrorCompensation_.storeroomVelA << std::endl
              << "storeroomErrorCompensation_.storeroomErrorA = " << Paras.storeroomErrorCompensation_.storeroomErrorA << std::endl
              << "storeroomErrorCompensation_.storeroomVelB = " << Paras.storeroomErrorCompensation_.storeroomVelB << std::endl
              << "storeroomErrorCompensation_.storeroomErrorB = " << Paras.storeroomErrorCompensation_.storeroomErrorB << std::endl
              << "storeroomErrorCompensation_.storeroomVelC = " << Paras.storeroomErrorCompensation_.storeroomVelC << std::endl
              << "storeroomErrorCompensation_.storeroomErrorC = " << Paras.storeroomErrorCompensation_.storeroomErrorC << std::endl
              << "storeroomErrorCompensation_.storeroomVelD = " << Paras.storeroomErrorCompensation_.storeroomVelD << std::endl
              << "storeroomErrorCompensation_.storeroomErrorD = " << Paras.storeroomErrorCompensation_.storeroomErrorD << std::endl;

    //----展平机构平移运动--相对位置---
    Paras.FlattenMove_.FlattenMoveLeftPos = std::stoi(ini.get("FlattenMove", "FlattenMoveLeftPos", "defaultUser"));
    Paras.FlattenMove_.FlattenMoveRightPos = std::stoi(ini.get("FlattenMove", "FlattenMoveRightPos", "defaultUser"));
    LOG(INFO) << "FlattenMove_.FlattenMoveLeftPos = " << Paras.FlattenMove_.FlattenMoveLeftPos << std::endl
              << "FlattenMove_.FlattenMoveRightPos = " << Paras.FlattenMove_.FlattenMoveRightPos << std::endl;

    //----展平机构运动-运动速度和匹配的误差补偿--
    Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveVelA", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorA = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveErrorA", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveVelB", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorB = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveErrorB", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveVelC = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveVelC", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorC = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveErrorC", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveVelD = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveVelD", "defaultUser"));
    Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorD = std::stoi(ini.get("FlattenMoveErrorCompensation", "FlattenMoveErrorD", "defaultUser"));
    LOG(INFO) << "FlattenMoveErrorCompensation_.FlattenMoveVelA = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveVelA << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveErrorA = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorA << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveVelB = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveVelB << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveErrorB = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorB << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveVelC = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveVelC << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveErrorC = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorC << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveVelD = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveVelD << std::endl
              << "FlattenMoveErrorCompensation_.FlattenMoveErrorD = " << Paras.FlattenMoveErrorCompensation_.FlattenMoveErrorD << std::endl;

    //----钛虎电机----
    Paras.TaiHu_.encoder = std::stoi(ini.get("TaiHu", "encoder", "defaultUser"));
    Paras.TaiHu_.ReductionRatio = std::stoi(ini.get("TaiHu", "ReductionRatio", "defaultUser"));
    Paras.TaiHu_.MotorDiameter = std::stoi(ini.get("TaiHu", "MotorDiameter", "defaultUser"));
    LOG(INFO) << "TaiHu_.encoder =" << Paras.TaiHu_.encoder << std::endl
              << "TaiHu_.ReductionRatio = " << Paras.TaiHu_.ReductionRatio << std::endl
              << "TaiHu_.MotorDiameter = " << Paras.TaiHu_.MotorDiameter << std::endl;

    //----所有电机的额定速度----
    Paras.ALLMotorsRatedVelocity_.YinwForgingMotorRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "YinwForgingMotorRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.ThreeAxisYmotorRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "ThreeAxisYmotorRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.ThreeAxisXmotorRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "ThreeAxisXmotorRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.YinwFeedinANDFlatteningMOVERatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "YinwFeedinANDFlatteningMOVERatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.PolarPlateWarehouseMoveRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "PolarPlateWarehouseMoveRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.QualifiedProductWareHouseRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "QualifiedProductWareHouseRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.DiscardedProductWareHouseRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "DiscardedProductWareHouseRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.ThreeAxisZmotorRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "ThreeAxisZmotorRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.WeldingPosMoveRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "WeldingPosMoveRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.WeldingShelfLeftRightRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "WeldingShelfLeftRightRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.WeldingShelfUpDownRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "WeldingShelfUpDownRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.TaihuFirstRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "TaihuFirstRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.TaihuSecondRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "TaihuSecondRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.TaihuThirdRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "TaihuThirdRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.TaihuFourthRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "TaihuFourthRatedVel", "defaultUser"));
    Paras.ALLMotorsRatedVelocity_.TaihuFifthRatedVel = std::stoi(ini.get("ALLMotorsRatedVelocity", "TaihuFifthRatedVel", "defaultUser"));
    LOG(INFO) << "YinwForgingMotorRatedVel = " << Paras.ALLMotorsRatedVelocity_.YinwForgingMotorRatedVel << std::endl
              << "ThreeAxisYmotorRatedVel = " << Paras.ALLMotorsRatedVelocity_.ThreeAxisYmotorRatedVel << std::endl
              << "ThreeAxisXmotorRatedVel = " << Paras.ALLMotorsRatedVelocity_.ThreeAxisXmotorRatedVel << std::endl
              << "YinwFeedinANDFlatteningMOVERatedVel = " << Paras.ALLMotorsRatedVelocity_.YinwFeedinANDFlatteningMOVERatedVel << std::endl
              << "PolarPlateWarehouseMoveRatedVel = " << Paras.ALLMotorsRatedVelocity_.PolarPlateWarehouseMoveRatedVel << std::endl
              << "QualifiedProductWareHouseRatedVel = " << Paras.ALLMotorsRatedVelocity_.QualifiedProductWareHouseRatedVel << std::endl
              << "DiscardedProductWareHouseRatedVel = " << Paras.ALLMotorsRatedVelocity_.DiscardedProductWareHouseRatedVel << std::endl
              << "ThreeAxisZmotorRatedVel = " << Paras.ALLMotorsRatedVelocity_.ThreeAxisZmotorRatedVel << std::endl
              << "WeldingPosMoveRatedVel = " << Paras.ALLMotorsRatedVelocity_.WeldingPosMoveRatedVel << std::endl
              << "WeldingShelfLeftRightRatedVel = " << Paras.ALLMotorsRatedVelocity_.WeldingShelfLeftRightRatedVel << std::endl
              << "WeldingShelfUpDownRatedVel = " << Paras.ALLMotorsRatedVelocity_.WeldingShelfUpDownRatedVel << std::endl
              << "TaihuFirstRatedVel = " << Paras.ALLMotorsRatedVelocity_.TaihuFirstRatedVel << std::endl
              << "TaihuSecondRatedVel = " << Paras.ALLMotorsRatedVelocity_.TaihuSecondRatedVel << std::endl
              << "TaihuThirdRatedVel = " << Paras.ALLMotorsRatedVelocity_.TaihuThirdRatedVel << std::endl
              << "TaihuFourthRatedVel = " << Paras.ALLMotorsRatedVelocity_.TaihuFourthRatedVel << std::endl
              << "TaihuFifthRatedVel = " << Paras.ALLMotorsRatedVelocity_.TaihuFifthRatedVel << std::endl;

    //----所有电机的额定电流----
    Paras.ALLMotorsRatedCurrent_.YinwForgingMotorRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "YinwForgingMotorRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.ThreeAxisYmotorRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "ThreeAxisYmotorRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.ThreeAxisXmotorRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "ThreeAxisXmotorRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.YinwFeedinANDFlatteningMOVERatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "YinwFeedinANDFlatteningMOVERatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.PolarPlateWarehouseMoveRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "PolarPlateWarehouseMoveRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.QualifiedProductWareHouseRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "QualifiedProductWareHouseRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.DiscardedProductWareHouseRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "DiscardedProductWareHouseRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.ThreeAxisZmotorRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "ThreeAxisZmotorRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.WeldingPosMoveRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "WeldingPosMoveRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.WeldingShelfLeftRightRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "WeldingShelfLeftRightRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.WeldingShelfUpDownRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "WeldingShelfUpDownRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.TaihuFirstRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "TaihuFirstRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.TaihuSecondRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "TaihuSecondRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.TaihuThirdRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "TaihuThirdRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.TaihuFourthRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "TaihuFourthRatedCur", "defaultUser"));
    Paras.ALLMotorsRatedCurrent_.TaihuFifthRatedCur = std::stoi(ini.get("ALLMotorsRatedCurrent", "TaihuFifthRatedCur", "defaultUser"));

    LOG(INFO) << "YinwForgingMotorRatedCur = " << Paras.ALLMotorsRatedCurrent_.YinwForgingMotorRatedCur << std::endl
              << "ThreeAxisYmotorRatedCur = " << Paras.ALLMotorsRatedCurrent_.ThreeAxisYmotorRatedCur << std::endl
              << "ThreeAxisXmotorRatedCur = " << Paras.ALLMotorsRatedCurrent_.ThreeAxisXmotorRatedCur << std::endl
              << "YinwFeedinANDFlatteningMOVERatedCur = " << Paras.ALLMotorsRatedCurrent_.YinwFeedinANDFlatteningMOVERatedCur << std::endl
              << "PolarPlateWarehouseMoveRatedCur = " << Paras.ALLMotorsRatedCurrent_.PolarPlateWarehouseMoveRatedCur << std::endl
              << "QualifiedProductWareHouseRatedCur = " << Paras.ALLMotorsRatedCurrent_.QualifiedProductWareHouseRatedCur << std::endl
              << "DiscardedProductWareHouseRatedCur = " << Paras.ALLMotorsRatedCurrent_.DiscardedProductWareHouseRatedCur << std::endl
              << "ThreeAxisZmotorRatedCur = " << Paras.ALLMotorsRatedCurrent_.ThreeAxisZmotorRatedCur << std::endl
              << "WeldingPosMoveRatedCur = " << Paras.ALLMotorsRatedCurrent_.WeldingPosMoveRatedCur << std::endl
              << "WeldingShelfLeftRightRatedCur = " << Paras.ALLMotorsRatedCurrent_.WeldingShelfLeftRightRatedCur << std::endl
              << "WeldingShelfUpDownRatedCur = " << Paras.ALLMotorsRatedCurrent_.WeldingShelfUpDownRatedCur << std::endl
              << "TaihuFirstRatedCur = " << Paras.ALLMotorsRatedCurrent_.TaihuFirstRatedCur << std::endl
              << "TaihuSecondRatedCur = " << Paras.ALLMotorsRatedCurrent_.TaihuSecondRatedCur << std::endl
              << "TaihuThirdRatedCur = " << Paras.ALLMotorsRatedCurrent_.TaihuThirdRatedCur << std::endl
              << "TaihuFourthRatedCur = " << Paras.ALLMotorsRatedCurrent_.TaihuFourthRatedCur << std::endl
              << "TaihuFifthRatedCur = " << Paras.ALLMotorsRatedCurrent_.TaihuFifthRatedCur << std::endl;

    //----所有电机的编码器的值----
    Paras.ALLMotorsEncoder_.YinwForgingMotorEncoder = std::stoi(ini.get("ALLMotorsEncoder", "YinwForgingMotorEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.ThreeAxisYmotorEncoder = std::stoi(ini.get("ALLMotorsEncoder", "ThreeAxisYmotorEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.ThreeAxisXmotorEncoder = std::stoi(ini.get("ALLMotorsEncoder", "ThreeAxisXmotorEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.YinwFeedinANDFlatteningMOVEEncoder = std::stoi(ini.get("ALLMotorsEncoder", "YinwFeedinANDFlatteningMOVEEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.PolarPlateWarehouseMoveEncoder = std::stoi(ini.get("ALLMotorsEncoder", "PolarPlateWarehouseMoveEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.QualifiedProductWareHouseEncoder = std::stoi(ini.get("ALLMotorsEncoder", "QualifiedProductWareHouseEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.DiscardedProductWareHouseEncoder = std::stoi(ini.get("ALLMotorsEncoder", "DiscardedProductWareHouseEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.ThreeAxisZmotorEncoder = std::stoi(ini.get("ALLMotorsEncoder", "ThreeAxisZmotorEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.WeldingPosMoveEncoder = std::stoi(ini.get("ALLMotorsEncoder", "WeldingPosMoveEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.WeldingShelfLeftRightEncoder = std::stoi(ini.get("ALLMotorsEncoder", "WeldingShelfLeftRightEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.WeldingShelfUpDownEncoder = std::stoi(ini.get("ALLMotorsEncoder", "WeldingShelfUpDownEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.TaihuFirstEncoder = std::stoi(ini.get("ALLMotorsEncoder", "TaihuFirstEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.TaihuSecondEncoder = std::stoi(ini.get("ALLMotorsEncoder", "TaihuSecondEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.TaihuThirdEncoder = std::stoi(ini.get("ALLMotorsEncoder", "TaihuThirdEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.TaihuFourthEncoder = std::stoi(ini.get("ALLMotorsEncoder", "TaihuFourthEncoder", "defaultUser"));
    Paras.ALLMotorsEncoder_.TaihuFifthEncoder = std::stoi(ini.get("ALLMotorsEncoder", "TaihuFifthEncoder", "defaultUser"));

    LOG(INFO) << "YinwForgingMotorEncoder = " << Paras.ALLMotorsEncoder_.YinwForgingMotorEncoder << std::endl
              << "ThreeAxisYmotorEncoder = " << Paras.ALLMotorsEncoder_.ThreeAxisYmotorEncoder << std::endl
              << "ThreeAxisXmotorEncoder = " << Paras.ALLMotorsEncoder_.ThreeAxisXmotorEncoder << std::endl
              << "YinwFeedinANDFlatteningMOVEEncoder = " << Paras.ALLMotorsEncoder_.YinwFeedinANDFlatteningMOVEEncoder << std::endl
              << "PolarPlateWarehouseMoveEncoder = " << Paras.ALLMotorsEncoder_.PolarPlateWarehouseMoveEncoder << std::endl
              << "QualifiedProductWareHouseEncoder = " << Paras.ALLMotorsEncoder_.QualifiedProductWareHouseEncoder << std::endl
              << "DiscardedProductWareHouseEncoder = " << Paras.ALLMotorsEncoder_.DiscardedProductWareHouseEncoder << std::endl
              << "ThreeAxisZmotorEncoder = " << Paras.ALLMotorsEncoder_.ThreeAxisZmotorEncoder << std::endl
              << "WeldingPosMoveEncoder = " << Paras.ALLMotorsEncoder_.WeldingPosMoveEncoder << std::endl
              << "WeldingShelfLeftRightEncoder = " << Paras.ALLMotorsEncoder_.WeldingShelfLeftRightEncoder << std::endl
              << "WeldingShelfUpDownEncoder = " << Paras.ALLMotorsEncoder_.WeldingShelfUpDownEncoder << std::endl
              << "TaihuFirstEncoder = " << Paras.ALLMotorsEncoder_.TaihuFirstEncoder << std::endl
              << "TaihuSecondEncoder = " << Paras.ALLMotorsEncoder_.TaihuSecondEncoder << std::endl
              << "TaihuThirdEncoder = " << Paras.ALLMotorsEncoder_.TaihuThirdEncoder << std::endl
              << "TaihuFourthEncoder = " << Paras.ALLMotorsEncoder_.TaihuFourthEncoder << std::endl
              << "TaihuFifthEncoder = " << Paras.ALLMotorsEncoder_.TaihuFifthEncoder << std::endl;

    LOG(INFO) << "<<=====================ALL Parameters==OVER=!!!!!!!!================>>" << std::endl;
    LOG(INFO) << "<<=====================^|^|^|^|^|^|^===================>>\n"
              << std::endl;
}

// 展平机构参数存储,更新ini文件
int16_t YinwDeviceParas::YW_FlattenParaSave(YW_DeviceParas Paras)
{
    IniParser ini;
    char cwd[PATH_MAX];
    std::string configPath;
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    // 将 int 转换为 string 并更新到 INI 文件
    ini.set("Flatten", "flatteningSteps", std::to_string(Paras.Flatten_.flatteningSteps));
    // ini.set("Flatten", "firstMotorApexPos", std::to_string(Paras.firstMotorApexPos));
    // ini.set("Flatten", "secondMotorApexPos", std::to_string(Paras.secondMotorApexPos));
    // ini.set("Flatten", "fourthMotorApexPos", std::to_string(Paras.fourthMotorApexPos));
    ini.set("Flatten", "firstYWtAllThick", std::to_string(Paras.Flatten_.firstYWtAllThick));
    ini.set("Flatten", "fourthYWAllThick", std::to_string(Paras.Flatten_.fourthYWAllThick));
    ini.set("Flatten", "firstMotorCircleNum", std::to_string(Paras.Flatten_.firstMotorCircleNum));
    ini.set("Flatten", "fourthMotorCircleNum", std::to_string(Paras.Flatten_.fourthMotorCircleNum));
    // 保存到文件
    try
    {
        ini.save("configPath.c_str()");
        LOG(INFO) << "4-Updated INI file saved successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

// 零点矩阵存储,更新ini文件
int16_t YinwDeviceParas::ZeroPointPosINISave(std::vector<std::vector<int>> ZeroPointValue)
{
    // 确保 ZeroPointValue 是 8x2 的矩阵
    if (ZeroPointValue.size() != 8 || ZeroPointValue[0].size() != 2)
    {
        // 尺寸不匹配时返回错误
        std::cerr << "Error: Matrix size is not 8x2. Actual size is: "
                  << ZeroPointValue.size() << "x" << ZeroPointValue[0].size() << std::endl;
        return -1;
    }

    IniParser ini;
    char cwd[PATH_MAX];
    std::string configPath;
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // 将 int 转换为 string 并更新到 INI 文件
    ini.set("ZeroPointPosValue", "weldDeskZeroPointName", std::to_string(ZeroPointValue[0][0]));
    ini.set("ZeroPointPosValue", "weldDeskZeroPointPosition", std::to_string(ZeroPointValue[0][1]));
    ini.set("ZeroPointPosValue", "FlattenMoveZeroPointName", std::to_string(ZeroPointValue[1][0]));
    ini.set("ZeroPointPosValue", "FlattenMoveZeroPointPosition", std::to_string(ZeroPointValue[1][1]));
    ini.set("ZeroPointPosValue", "StampingZeroPointName", std::to_string(ZeroPointValue[2][0]));
    ini.set("ZeroPointPosValue", "StampingZeroPointPosition", std::to_string(ZeroPointValue[2][1]));
    ini.set("ZeroPointPosValue", "PolePlateZeroPointName", std::to_string(ZeroPointValue[3][0]));
    ini.set("ZeroPointPosValue", "PolePlateZeroPointPosition", std::to_string(ZeroPointValue[3][1]));
    ini.set("ZeroPointPosValue", "BadProductPointName", std::to_string(ZeroPointValue[4][0]));
    ini.set("ZeroPointPosValue", "BadProductPointPosition", std::to_string(ZeroPointValue[4][1]));
    ini.set("ZeroPointPosValue", "GoodProductZeroPointName", std::to_string(ZeroPointValue[5][0]));
    ini.set("ZeroPointPosValue", "GoodProductZeroPointPosition", std::to_string(ZeroPointValue[5][1]));
    ini.set("ZeroPointPosValue", "WeldingframelockingZeroPointName", std::to_string(ZeroPointValue[6][0]));
    ini.set("ZeroPointPosValue", "WeldingframelockingZeroPointPosition", std::to_string(ZeroPointValue[6][1]));
    ini.set("ZeroPointPosValue", "WeldingframetranslateZeroPointName", std::to_string(ZeroPointValue[7][0]));
    ini.set("ZeroPointPosValue", "WeldingframetranslateZeroPointPosition", std::to_string(ZeroPointValue[7][1]));

    // 保存到文件
    try
    {
        ini.save(configPath.c_str());
        LOG(INFO) << "Updated INI file saved successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

// 对存好的零点文件进行读取
std::array<std::array<int, 2>, 8> YinwDeviceParas::loadZeroNameAndPos()
{
    IniParser ini;
    char cwd[PATH_MAX];
    std::array<std::array<int, 2>, 8> ZeroPointNameAndValue; // 固定大小的二维矩阵
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        std::string configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());

        LOG(INFO) << "<<=====================ZeroPointPosValue===================>>" << std::endl;
        //  全局参数
        ZeroPointNameAndValue[0][0] = std::stoi(ini.get("ZeroPointPosValue", "weldDeskZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[0][1] = std::stoi(ini.get("ZeroPointPosValue", "weldDeskZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[1][0] = std::stoi(ini.get("ZeroPointPosValue", "FlattenMoveZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[1][1] = std::stoi(ini.get("ZeroPointPosValue", "FlattenMoveZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[2][0] = std::stoi(ini.get("ZeroPointPosValue", "StampingZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[2][1] = std::stoi(ini.get("ZeroPointPosValue", "StampingZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[3][0] = std::stoi(ini.get("ZeroPointPosValue", "PolePlateZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[3][1] = std::stoi(ini.get("ZeroPointPosValue", "PolePlateZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[4][0] = std::stoi(ini.get("ZeroPointPosValue", "BadProductPointName", "defaultUser"));
        ZeroPointNameAndValue[4][1] = std::stoi(ini.get("ZeroPointPosValue", "BadProductPointPosition", "defaultUser"));
        ZeroPointNameAndValue[5][0] = std::stoi(ini.get("ZeroPointPosValue", "GoodProductZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[5][1] = std::stoi(ini.get("ZeroPointPosValue", "GoodProductZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[6][0] = std::stoi(ini.get("ZeroPointPosValue", "WeldingframelockingZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[6][1] = std::stoi(ini.get("ZeroPointPosValue", "WeldingframelockingZeroPointPosition", "defaultUser"));
        ZeroPointNameAndValue[7][0] = std::stoi(ini.get("ZeroPointPosValue", "WeldingframetranslateZeroPointName", "defaultUser"));
        ZeroPointNameAndValue[7][1] = std::stoi(ini.get("ZeroPointPosValue", "WeldingframetranslateZeroPointPosition", "defaultUser"));
        LOG(INFO) << "weldDeskZeroPointName = " << ZeroPointNameAndValue[0][0] << std::endl
                  << "weldDeskZeroPointPosition = " << ZeroPointNameAndValue[0][1] << std::endl
                  << "FlattenMoveZeroPointName = " << ZeroPointNameAndValue[1][0] << std::endl
                  << "FlattenMoveZeroPointPosition = " << ZeroPointNameAndValue[1][1] << std::endl
                  << "StampingZeroPointName = " << ZeroPointNameAndValue[2][0] << std::endl
                  << "StampingZeroPointPosition = " << ZeroPointNameAndValue[2][1] << std::endl
                  << "PolePlateZeroPointName = " << ZeroPointNameAndValue[3][0] << std::endl
                  << "PolePlateZeroPointPosition = " << ZeroPointNameAndValue[3][1] << std::endl
                  << "BadProductPointName = " << ZeroPointNameAndValue[4][0] << std::endl
                  << "BadProductPointPosition = " << ZeroPointNameAndValue[4][1] << std::endl
                  << "GoodProductZeroPointName = " << ZeroPointNameAndValue[5][0] << std::endl
                  << "GoodProductZeroPointPosition = " << ZeroPointNameAndValue[5][1] << std::endl
                  << "WeldingframelockingZeroPointName = " << ZeroPointNameAndValue[6][0] << std::endl
                  << "WeldingframelockingZeroPointPosition = " << ZeroPointNameAndValue[6][1] << std::endl
                  << "WeldingframetranslateZeroPointName = " << ZeroPointNameAndValue[7][0] << std::endl
                  << "WeldingframetranslateZeroPointPosition = " << ZeroPointNameAndValue[7][1] << std::endl;
        LOG(INFO) << "<<=====================ZeroPointPosValue OVER=!!!!!!!!!!!!==================>>" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return ZeroPointNameAndValue;
}

// 将界面上的参数存储进ini文件
int16_t YinwDeviceParas::SystemSettingParasSave(MsgHuman2RobFrame_ ParaSettingData)
{
    IniParser ini;
    char cwd[PATH_MAX];
    std::string configPath;
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    // 将 int 转换为 string 并更新到 INI 文件
    ini.set("Generalparams", "JinWangSuDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.jin_wang_su_du));                         // 进网速度--
    ini.set("PoleSheet", "JiPianHouDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.ji_pian_hou_du));                             // 极片厚度--
    ini.set("SilverNet", "YinWangHouDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.yin_wang_hou_du));                           // 银网厚度--
    ini.set("SilverNet", "YinWangNeiJing", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.yin_wang_nei_jing));                       // 银网内径--
    ini.set("SilverNet", "YinWangWaiJing", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.yin_wang_wai_jing));                       // 银网外径--
    ini.set("SilverNet", "YinWangZongChangDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.yin_wang_zong_chang_du));              // 银网总长度--
    ini.set("PoleSheet", "JiPianNeiJing", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.ji_pian_nei_jing));                         // 极片内径--
    ini.set("PoleSheet", "JiPianWaiJing", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.ji_pian_wai_jing));                         // 极片外径--
    ini.set("Generalparams", "DanJianChengPinShiJian", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.dan_jian_cheng_pin_shi_jian)); // 单件成品时间--
    ini.set("Stamping", "ChongYaJuLi", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.chong_ya_ju_li));                              // 冲压距离--
    ini.set("Weld", "HanDianShuLiang", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.han_dian_shu_liang));                          // 焊点数量--
    ini.set("Generalparams", "ChuLiaoSuDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.chu_liao_su_du));                         // 出料速度--
    ini.set("Generalparams", "ShuSongSuDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.shu_song_su_du));                         // 输送速度--
    ini.set("Weld", "HanJieWeiZhiFenBu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.han_jie_wei_zhi_fen_bu));                    // 焊点位置分布--
    ini.set("SilverNet", "YinWangSongLiaoSuDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.yin_wang_song_liao_su_du));           // 银网送料速度--
    ini.set("Weld", "HanDianGaoDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.han_dian_gao_du));                                // 焊点高度--
    ini.set("Weld", "HanDianKuanDu", std::to_string(ParaSettingData.MsgHuman2Rob.hr_params_setting_.han_dian_kuan_du));                              // 焊点宽度

    // 保存到文件
    try
    {
        ini.save(configPath.c_str());
        LOG(INFO) << "Updated INI file saved successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}