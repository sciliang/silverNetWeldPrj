#include <iostream>
#include <vector>
#include "TCP_Protocol.h"
#include "ControlCanFun.h"
#include "TcpTeleGlobalStruct.h"
#include "TcpSocket.h"
#include "YinwMotion.h"
#include <glog/logging.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

TcpSocket::TcpSocket(/* args */)
{
}

TcpSocket::~TcpSocket()
{
}

void TcpSocket::pkgReady(const std::vector<uint8_t> &data)
{
    const MsgHuman2Rob_ *msgHuman2Rob = reinterpret_cast<const MsgHuman2Rob_ *>(data.data());
    sendStateFeedBack(TaskState::Succeed, msgHuman2Rob->msgID);
    LOG(INFO) << "recv pack: " << msgHuman2Rob->msgID << std::endl;
    switch (msgHuman2Rob->msgID)
    {
    case HR_SHUTDOWN_ID: // 设备关机
        LOG(INFO) << "HR_SHUTDOWN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtyFifthStep;
        break;
    case HR_CONTROLLER_REBOOT_ID: // 控制器重启
        LOG(INFO) << "HR_CONTROLLER_REBOOT_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftySeventhStep;
        break;
    case HR_SCRAM_ID: // 设备急停
        LOG(INFO) << "HR_SCRAM_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtySixthStep;
        break;
    case HR_RESET_ID: // 设备复位
        LOG(INFO) << "HR_RESET_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtySeventhStep;
        break;
    case HR_PAUSE_ID: // 设备暂停
        LOG(INFO) << "HR_PAUSE_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtyEighthStep;
        break;
    case HR_CONTINUE_ID: // 设备继续
        LOG(INFO) << "HR_CONTINUE_ID" << std::endl;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtyNineStep;
        break;
    case HR_ONEKEY_START_ID: // 设备一键启动
        LOG(INFO) << "HR_ONEKEY_START_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 2; // 自主控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortiethStep;
        break;
    case HR_LAMP_ON_ID: // 设备照明灯开
        LOG(INFO) << "HR_LAMP_ON_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortyFirstStep;
        break;
    case HR_LAMP_OFF_ID: // 设备照明灯关
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortySecondStep;
        break;
    case HR_RED_LAMP_ON_ID: // 设备红灯开
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftyEighthStep;
        break;
    case HR_RED_LAMP_OFF_ID: // 设备红灯关
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftyNinethStep;
        break;
    case HR_GREEN_LAMP_ON_ID: // 设备绿灯开
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixtiehStep;
        break;
    case HR_GREEN_LAMP_OFF_ID: // 设备绿灯关
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixtyFirstStep;
        break;
    case HR_ORANGE_LAMP_ON_ID: // 设备橙色灯开
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixtySecondStep;
        break;
    case HR_ORANGE_LAMP_OFF_ID: // 设备橙色灯关
        LOG(INFO) << "HR_LAMP_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixtyThirdStep;
        break;
    case HR_FLATTEN_BUJIN_ID: // 展平机构步进控制
        LOG(INFO) << "HR_FLATTEN_BUJIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FifthStep;
        break;
    case HR_HAN_TAI_YUN_SHU_A_ID: // 焊台运输运动至A位置
        LOG(INFO) << "HR_HAN_TAI_YUN_SHU_A_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixthStep;
        break;
    case HR_HAN_TAI_YUN_SHU_B_ID: // 焊台运输运动至B位置
        LOG(INFO) << "HR_HAN_TAI_YUN_SHU_B_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = EighteenthStep;
        break;
    case HR_TRIAXIALWELDING_ON_ID: // 三轴焊接
        LOG(INFO) << "HR_TRIAXIALWELDING_ON_ID" << std::endl;
        LOG(INFO) << msgHuman2Rob->hr_triaxialwelding_on_.on << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = NineteenthStep;
        break;
    case HR_HAN_TAI_YUN_SHU_C_ID: // 焊台运输运动至C位置
        LOG(INFO) << "HR_HAN_TAI_YUN_SHU_C_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentysecondStep;
        break;
    case HR_HAN_TAI_YUN_SHU_D_ID: // 焊台运输运动至D位置
        LOG(INFO) << "HR_HAN_TAI_YUN_SHU_D_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyfourthStep;
        break;
    case HR_HAN_TAI_YUN_SHU_FUWEI_ID: // 焊台运输运动至复位位置
        LOG(INFO) << "HR_HAN_TAI_YUN_SHU_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixthStep;
        break;
    case HR_HAN_JIA_SHU_ZHI_SHENGQI_ID: // 焊架竖直机构升起
        LOG(INFO) << "HR_HAN_JIA_SHU_ZHI_SHENGQI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SeventhStep;
        break;
    case HR_HAN_JIA_SHU_ZHI_LUOXIA_ID: // 焊架竖直机构落下
        LOG(INFO) << "HR_HAN_JIA_SHU_ZHI_LUOXIA_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortyFifthStep;
        break;
    case HR_HAN_JIA_SHU_ZHI_FUWEI_ID: // 焊架竖直机构复位
        LOG(INFO) << "HR_HAN_JIA_SHU_ZHI_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SeventhStep;
        break;
    case HR_HAN_JIA_SHUI_PING_LEFT_ID: // 焊架水平机构运动至左侧
        LOG(INFO) << "HR_HAN_JIA_SHUI_PING_LEFT_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyfirstStep;
        break;
    case HR_HAN_JIA_SHUI_PING_RIGHT_ID: // 焊架水平机构运动至右侧
        LOG(INFO) << "HR_HAN_JIA_SHUI_PING_RIGHT_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = SixteenthStep;
        break;
    case HR_HAN_JIA_SHUI_PING_FUWEI_ID: // 焊架水平机构复位
        LOG(INFO) << "HR_HAN_JIA_SHUI_PING_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyfirstStep;
        break;
    case HR_YIN_WANG_YA_ZHI_YAWANG_ID: // 银网压制位压网位置
        LOG(INFO) << "HR_YIN_WANG_YA_ZHI_YAWANG_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = NinthStep;
        break;
    case HR_YIN_WANG_YA_ZHI_YUNWANG_ID: // 银网压制位运网位置
        LOG(INFO) << "HR_YIN_WANG_YA_ZHI_YUNWANG_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = EleventhStep;
        break;
    case HR_YIN_WANG_YA_ZHI_FUWEI_ID: // 银网压制位复位位置
        LOG(INFO) << "HR_YIN_WANG_YA_ZHI_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = NinthStep;
        break;
    case HR_YA_JI_CHONG_ZHI_CHONGWANG_ID: // 银网压机冲制位冲网位置
        LOG(INFO) << "HR_YA_JI_CHONG_ZHI_CHONGWANG_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TenthStep;
        break;
    case HR_YA_JI_CHONG_ZHI_DENGDAI_ID: // 银网压机冲制位等待位置
        LOG(INFO) << "HR_YA_JI_CHONG_ZHI_DENGDAI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortySixthStep;
        break;
    case HR_YA_JI_CHONG_ZHI_FUWEI_ID: // 银网压机冲制位复位位置
        LOG(INFO) << "HR_YA_JI_CHONG_ZHI_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortyFourthStep;
        break;
    case HR_SAN_ZUO_BIAO_QUJIPIAN_ID: // 三坐标取极片位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_QUJIPIAN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwelfthStep;
        break;
    case HR_SAN_ZUO_BIAO_FANGJIPIAN_ID: // 三坐标放极片位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_FANGJIPIAN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirteenthStep;
        break;
    case HR_SAN_ZUO_BIAO_QUYINWANG_ID: // 三坐标取银网位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_QUYINWANG_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FourteenthStep;
        break;
    case HR_SAN_ZUO_BIAO_FANGYINWANG_ID: // 三坐标放银网位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_FANGYINWANG_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FifteenthStep;
        break;
    case HR_SAN_ZUO_BIAO_QUCHANPIN_ID: // 三坐标取产品位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_QUCHANPIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyfifthStep;
        break;
    case HR_SAN_ZUO_BIAO_FANGCHENGPIN_ID: // 三坐标取放成品位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_FANGCHENGPIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyseventhStep;
        break;
    case HR_SAN_ZUO_BIAO_FANGFEIPIN_ID: // 三坐标取放废品位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_FANGFEIPIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentysixthStep;
        break;
    case HR_SAN_ZUO_BIAO_FUWEI_ID: // 三坐标复位位置
        LOG(INFO) << "HR_SAN_ZUO_BIAO_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = HR_SAN_ZUO_BIAO_FUWEI_ID;
        break;
    case HR_AOI_DETECT_ON_ID: // AOI检测
        LOG(INFO) << "HR_AOI_DETECT_ON_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentythirdStep;
        break;
    case HR_QI_BENG_ON_ID: // 控制气泵开
        LOG(INFO) << "HR_QI_BENG_ON_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;                                      // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftyFifthStep; // 气泵开
        break;
    case HR_QI_BENG_OFF_ID: // 控制气泵关
        LOG(INFO) << "HR_QI_BENG_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;                                      // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftySixthStep; // 气泵关
        break;
    case HR_JIA_ZHAO_ON_ID: // 控制夹爪开
        LOG(INFO) << "HR_JIA_ZHAO_ON_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;                                      // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftyThirdStep; // 夹爪开
        break;
    case HR_JIA_ZHAO_OFF_ID: // 控制夹爪关
        LOG(INFO) << "HR_JIA_ZHAO_OFF_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1;                                       // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftyFourthStep; // 夹爪关
        break;
    case HR_JI_PIAN_KU_BUJIN_ID: // 极片库步进
        LOG(INFO) << "HR_JI_PIAN_KU_BUJIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = TwentyninethStep;
        break;
    case HR_JI_PIAN_KU_FUWEI_ID: // 废片库复位
        LOG(INFO) << "HR_JI_PIAN_KU_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortySeventhStep;
        break;
    case HR_FEI_PIN_KU_BUJIN_ID: // 废品库步进
        LOG(INFO) << "HR_FEI_PIN_KU_BUJIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtyfirtStep;
        break;
    case HR_FEI_PIN_KU_FUWEI_ID: // 废品库复位
        LOG(INFO) << "HR_FEI_PIN_KU_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortyNinethStep;
        break;
    case HR_CHENGPINKU_BUJIN_ID: // 成品库步进
        LOG(INFO) << "HR_CHENGPINKU_BUJIN_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = ThirtiethStep;
        break;
    case HR_CHENGPINKU_FUWEI_ID: // 成品库复位
        LOG(INFO) << "HR_CHENGPINKU_FUWEI_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FortyEighthStep;
        break;
    case HR_PARAMS_SETTING_ID:
        LOG(INFO) << "HR_PARAMS_SETTING_ID" << std::endl;
        ShareMsgHuman2RobCMD_.DeviceControlMode = 1; // 步进控制
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.msgID = FiftySecondStep;
        LOG(INFO) << "jin_wang_su_du = " << msgHuman2Rob->hr_params_setting_.jin_wang_su_du << std::endl;
        LOG(INFO) << "yin_wang_hou_du = " << msgHuman2Rob->hr_params_setting_.yin_wang_hou_du << std::endl;
        LOG(INFO) << "ji_pian_hou_du = " << msgHuman2Rob->hr_params_setting_.ji_pian_hou_du << std::endl;
        LOG(INFO) << "yin_wang_nei_jing = " << msgHuman2Rob->hr_params_setting_.yin_wang_nei_jing << std::endl;
        LOG(INFO) << "yin_wang_wai_jing = " << msgHuman2Rob->hr_params_setting_.yin_wang_wai_jing << std::endl;
        LOG(INFO) << "yin_wang_zong_chang_du = " << msgHuman2Rob->hr_params_setting_.yin_wang_zong_chang_du << std::endl;
        LOG(INFO) << "ji_pian_nei_jing = " << msgHuman2Rob->hr_params_setting_.ji_pian_nei_jing << std::endl;
        LOG(INFO) << "ji_pian_wai_jing = " << msgHuman2Rob->hr_params_setting_.ji_pian_wai_jing << std::endl;
        LOG(INFO) << "dan_jian_cheng_pin_shi_jian = " << msgHuman2Rob->hr_params_setting_.dan_jian_cheng_pin_shi_jian << std::endl;
        LOG(INFO) << "chong_ya_ju_li = " << msgHuman2Rob->hr_params_setting_.chong_ya_ju_li << std::endl;
        LOG(INFO) << "han_dian_shu_liang = " << msgHuman2Rob->hr_params_setting_.han_dian_shu_liang << std::endl;
        LOG(INFO) << "chu_liao_su_du = " << msgHuman2Rob->hr_params_setting_.chu_liao_su_du << std::endl;
        LOG(INFO) << "shu_song_su_du = " << msgHuman2Rob->hr_params_setting_.shu_song_su_du << std::endl;
        LOG(INFO) << "han_jie_wei_zhi_fen_bu = " << msgHuman2Rob->hr_params_setting_.han_jie_wei_zhi_fen_bu << std::endl;
        LOG(INFO) << "yin_wang_song_liao_su_du = " << msgHuman2Rob->hr_params_setting_.yin_wang_song_liao_su_du << std::endl;
        LOG(INFO) << "han_dian_gao_du = " << msgHuman2Rob->hr_params_setting_.han_dian_gao_du << std::endl;
        LOG(INFO) << "han_dian_kuan_du = " << msgHuman2Rob->hr_params_setting_.han_dian_kuan_du << std::endl;
        // 赋值
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.jin_wang_su_du = msgHuman2Rob->hr_params_setting_.jin_wang_su_du;

        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.yin_wang_hou_du = msgHuman2Rob->hr_params_setting_.ji_pian_hou_du; // 这两个赋值是反的 目前是交换赋值的
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.ji_pian_hou_du = msgHuman2Rob->hr_params_setting_.yin_wang_hou_du; // 这两个赋值是反的 交换了顺序

        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.yin_wang_nei_jing = msgHuman2Rob->hr_params_setting_.yin_wang_nei_jing;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.yin_wang_wai_jing = msgHuman2Rob->hr_params_setting_.yin_wang_wai_jing;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.yin_wang_zong_chang_du = msgHuman2Rob->hr_params_setting_.yin_wang_zong_chang_du;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.ji_pian_nei_jing = msgHuman2Rob->hr_params_setting_.ji_pian_nei_jing;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.ji_pian_wai_jing = msgHuman2Rob->hr_params_setting_.ji_pian_wai_jing;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.dan_jian_cheng_pin_shi_jian = msgHuman2Rob->hr_params_setting_.dan_jian_cheng_pin_shi_jian;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.chong_ya_ju_li = msgHuman2Rob->hr_params_setting_.chong_ya_ju_li;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.han_dian_shu_liang = msgHuman2Rob->hr_params_setting_.han_dian_shu_liang;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.chu_liao_su_du = msgHuman2Rob->hr_params_setting_.chu_liao_su_du;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.shu_song_su_du = msgHuman2Rob->hr_params_setting_.shu_song_su_du;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.han_jie_wei_zhi_fen_bu = msgHuman2Rob->hr_params_setting_.han_jie_wei_zhi_fen_bu;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.yin_wang_song_liao_su_du = msgHuman2Rob->hr_params_setting_.yin_wang_song_liao_su_du;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.han_dian_gao_du = msgHuman2Rob->hr_params_setting_.han_dian_gao_du;
        ShareMsgHuman2RobCMD_.ShareMsgHuman2RobFrame.MsgHuman2Rob.hr_params_setting_.han_dian_kuan_du = msgHuman2Rob->hr_params_setting_.han_dian_kuan_du;
        break;
    default:
        LOG(INFO) << " pkgReady :default " << std::endl;
        break;
    }
    sleep(2);
    sendTaskStateFeedBack(msgHuman2Rob->msgID, TaskState::Succeed);
}

void TcpSocket::sendStateFeedBack(int rcvStatus, int taskID)
{
    Rob2MsgHumanFrame_ msgRob2HumanFrame;
    msgRob2HumanFrame.msgHeader.sig[0] = 0xff;
    msgRob2HumanFrame.msgHeader.sig[1] = 0xfd;
    msgRob2HumanFrame.msgHeader.sig[2] = 0xfe;
    msgRob2HumanFrame.msgHeader.sig[3] = 0xff;

    msgRob2HumanFrame.Rob2MsgHuman.msgID = RH_SEND_STATUS_FEEDBACK_ID;
    msgRob2HumanFrame.Rob2MsgHuman.rh_send_status_feedback_.status = rcvStatus;
    msgRob2HumanFrame.Rob2MsgHuman.rh_send_status_feedback_.id = taskID;
    //    msgRob2HumanFrame.Rob2MsgHuman.rh_send_status_feedback_.is_on   = is_on;
    msgRob2HumanFrame.msgHeader.len = sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader);
    msgRob2HumanFrame.msgHeader.cksum = calcCRC32((char *)&msgRob2HumanFrame.Rob2MsgHuman, sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader));

    int bytesSent = send(clientSocket, &msgRob2HumanFrame, sizeof(msgRob2HumanFrame), 0);
    if (bytesSent == -1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cerr << "Send failed." << std::endl;
    }
    else
    {
        LOG(INFO) << "send ret: " << bytesSent << std::endl;
    }
}

void TcpSocket::sendTaskStateFeedBack(int taskID, int taskState)
{
    Rob2MsgHumanFrame_ msgRob2HumanFrame;
    msgRob2HumanFrame.msgHeader.sig[0] = 0xff;
    msgRob2HumanFrame.msgHeader.sig[1] = 0xfd;
    msgRob2HumanFrame.msgHeader.sig[2] = 0xfe;
    msgRob2HumanFrame.msgHeader.sig[3] = 0xff;

    msgRob2HumanFrame.Rob2MsgHuman.msgID = RH_TASK_State_ID;
    msgRob2HumanFrame.Rob2MsgHuman.rh_task_state_.taskId = taskID;
    msgRob2HumanFrame.Rob2MsgHuman.rh_task_state_.taskState = TaskState::Succeed;

    msgRob2HumanFrame.msgHeader.len = sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader);

    msgRob2HumanFrame.msgHeader.cksum = calcCRC32((char *)&msgRob2HumanFrame.Rob2MsgHuman, sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader));

    int bytesSent = send(clientSocket, &msgRob2HumanFrame, sizeof(msgRob2HumanFrame), 0);
    if (bytesSent == -1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cerr << "Send failed." << std::endl;
    }
    else
    {
        LOG(INFO) << "task state send ret: " << bytesSent << std::endl;
    }
}

uint32_t TcpSocket::calcCRC32(const char *data_ptr, int size)
{
    static const uint32_t crc32Table[] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535,
        0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd,
        0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d,
        0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
        0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4,
        0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac,
        0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
        0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
        0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f,
        0x9fbfe4a5, 0xe8b8d433, 0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb,
        0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea,
        0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce,
        0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a,
        0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
        0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409,
        0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739,
        0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
        0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 0xf00f9344, 0x8708a3d2, 0x1e01f268,
        0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0,
        0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8,
        0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef,
        0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703,
        0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
        0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
        0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae,
        0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777, 0x88085ae6,
        0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
        0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d,
        0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5,
        0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605,
        0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

    uint32_t crc = 0xFFFFFFFF;
    for (int i = 0; i < size; ++i)
    {
        crc = crc32Table[(crc ^ data_ptr[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}