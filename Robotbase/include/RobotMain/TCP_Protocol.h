#ifndef INCLUDE_COMMUNICATIONPROTOCOL_H_
#define INCLUDE_COMMUNICATIONPROTOCOL_H_

#include <stdint.h>

enum MessageType
{
    HR_SHUTDOWN_ID = 0,
    HR_CONTROLLER_REBOOT_ID,
    HR_SCRAM_ID,
    HR_RESET_ID,
    HR_PAUSE_ID,
    HR_CONTINUE_ID,

    HR_ONEKEY_START_ID,

    HR_LAMP_ON_ID,
    HR_LAMP_OFF_ID,
    HR_RED_LAMP_ON_ID,
    HR_RED_LAMP_OFF_ID,
    HR_GREEN_LAMP_ON_ID,
    HR_GREEN_LAMP_OFF_ID,
    HR_ORANGE_LAMP_ON_ID,
    HR_ORANGE_LAMP_OFF_ID,

    HR_FLATTEN_BUJIN_ID,

    HR_HAN_TAI_YUN_SHU_A_ID,
    HR_HAN_TAI_YUN_SHU_B_ID,
    HR_HAN_TAI_YUN_SHU_C_ID,
    HR_HAN_TAI_YUN_SHU_D_ID,
    HR_HAN_TAI_YUN_SHU_FUWEI_ID,

    HR_HAN_JIA_SHU_ZHI_SHENGQI_ID,
    HR_HAN_JIA_SHU_ZHI_LUOXIA_ID,
    HR_HAN_JIA_SHU_ZHI_FUWEI_ID,

    HR_HAN_JIA_SHUI_PING_LEFT_ID,
    HR_HAN_JIA_SHUI_PING_RIGHT_ID,
    HR_HAN_JIA_SHUI_PING_FUWEI_ID,

    HR_YIN_WANG_YA_ZHI_YAWANG_ID,
    HR_YIN_WANG_YA_ZHI_YUNWANG_ID,
    HR_YIN_WANG_YA_ZHI_FUWEI_ID,

    HR_YA_JI_CHONG_ZHI_CHONGWANG_ID,
    HR_YA_JI_CHONG_ZHI_DENGDAI_ID,
    HR_YA_JI_CHONG_ZHI_FUWEI_ID,

    HR_SAN_ZUO_BIAO_QUJIPIAN_ID,
    HR_SAN_ZUO_BIAO_FANGJIPIAN_ID,
    HR_SAN_ZUO_BIAO_QUYINWANG_ID,
    HR_SAN_ZUO_BIAO_FANGYINWANG_ID,
    HR_SAN_ZUO_BIAO_QUCHANPIN_ID,
    HR_SAN_ZUO_BIAO_FANGCHENGPIN_ID,
    HR_SAN_ZUO_BIAO_FANGFEIPIN_ID,
    HR_SAN_ZUO_BIAO_FUWEI_ID,

    // HR_STAMPING_ON_ID,
    // HR_QU_JI_PIAN_ON_ID,
    // HR_QU_YA_PIAN_WANG_ON_ID,

    HR_TRIAXIALWELDING_ON_ID,
    HR_AOI_DETECT_ON_ID,

    HR_QI_BENG_ON_ID,
    HR_QI_BENG_OFF_ID,

    HR_JIA_ZHAO_ON_ID,
    HR_JIA_ZHAO_OFF_ID,

    HR_JI_PIAN_KU_BUJIN_ID,
    HR_JI_PIAN_KU_FUWEI_ID,

    HR_FEI_PIN_KU_BUJIN_ID,
    HR_FEI_PIN_KU_FUWEI_ID,

    HR_CHENGPINKU_BUJIN_ID,
    HR_CHENGPINKU_FUWEI_ID,

    // HR_JI_PIAN_KU_ID,
    // HR_HE_GE_PIN_KU_ID,
    // HR_FEI_PIN_KU_ID,

    HR_PARAMS_SETTING_ID,

    RH_SEND_STATUS_FEEDBACK_ID,
    RH_PRODUCTION_INFO_ID,

    RH_TASK_State_ID,
};

enum EquipmentStatus
{
    NORMAL_OPERATION = 101,
    MALFUNCTION,
    SHUTDOWN,
};

enum TaskState
{
    Succeed = 201,
    Failed,
    Unidentification,
};

#pragma pack(1)

struct HR_SHUTDOWN
{
    // null
};

struct HR_CSRAM
{
    // null
};

struct HR_RESET
{
    // null
};

struct HR_LAMP_ON
{
    bool on;
};

struct HR_FLATTEN_ON
{
    bool on;
};

struct HR_STAMPING_ON
{
    bool on;
};

struct HR_QUJIPIAN_ON
{
    bool on;
};

struct HR_QUYAPIANWANG_ON
{
    bool on;
};

struct HR_TRIAXIALWELDING_ON
{
    bool on;
};

struct HR_AOIDETECT_ON
{
    bool on;
};

struct HR_JIPIANKU
{
    bool on;
};

struct HR_HEGEPINKU
{
    bool on;
};

struct HR_FEIPINKU
{
    bool on;
};

struct HR_PARAMS_SETTING
{
    uint32_t jin_wang_su_du;
    uint32_t yin_wang_hou_du;
    uint32_t ji_pian_hou_du;
    uint32_t yin_wang_nei_jing;
    uint32_t yin_wang_wai_jing;
    uint32_t yin_wang_zong_chang_du;
    uint32_t ji_pian_nei_jing;
    uint32_t ji_pian_wai_jing;
    uint32_t dan_jian_cheng_pin_shi_jian;
    uint32_t chong_ya_ju_li;
    uint32_t han_dian_shu_liang;
    uint32_t chu_liao_su_du;
    uint32_t shu_song_su_du;
    uint32_t han_jie_wei_zhi_fen_bu;
    uint32_t yin_wang_song_liao_su_du;
    uint32_t han_dian_gao_du;
    uint32_t han_dian_kuan_du;
};

struct RH_SENDSTATUSFEEDBACK
{
    int status;
    int id;
    bool is_on; // 发送的命令 启动(True),关闭/复位(False),没有的写True
};

struct RH_Task_State
{
    int taskId;
    int taskState;
};

struct RH_PRODUCTIONINFO
{
    uint32_t she_bei_zhuang_tai;    // 设备状态
    uint32_t sheng_chan_zong_shu;   // 生产总数
    uint32_t he_ge_shu;             // 合格品数
    uint32_t bu_he_ge_shu;          // 废品数
    uint32_t jie_lun;               // 结论
    uint32_t ji_pian_ku_shang_liao; // 极片库上料
    uint32_t he_ge_pin_ku_qu_liao;  // 合格品库上料
    uint32_t fei_pin_ku_qu_liao;    // 废品库上料
};

struct tagPCHeader
{
    // uint8_t sig[4];
    // uint32_t cksum;
    // uint32_t len;
    int msgID;
    // float UTC;
};

struct tagChkPCHeader
{
    uint8_t sig[4];
    uint32_t cksum;
    uint32_t len;
    // int msgID;
    // float UTC;
};

typedef struct
{
    int msgID;
    union
    {
        // struct HR_SHUTDOWN           hr_shutdown_;
        // struct HR_CSRAM              hr_csram_;
        // struct HR_RESET              hr_reset_;
        struct HR_LAMP_ON hr_lamp_on_;
        struct HR_FLATTEN_ON hr_flatten_on_;
        struct HR_STAMPING_ON hr_stamping_on_;
        struct HR_QUJIPIAN_ON hr_qu_ji_pian_on_;
        struct HR_QUYAPIANWANG_ON hr_qu_ya_pian_wang_on_;
        struct HR_TRIAXIALWELDING_ON hr_triaxialwelding_on_;
        struct HR_AOIDETECT_ON hr_aoi_detect_on_;

        struct HR_JIPIANKU hr_ji_pian_ku_;
        struct HR_HEGEPINKU hr_he_ge_pin_ku_;
        struct HR_FEIPINKU hr_fei_pin_ku_;

        struct HR_PARAMS_SETTING hr_params_setting_;
    };
} MsgHuman2Rob_;

typedef struct
{
    // uint16_t crc16;
    struct tagChkPCHeader msgHeader;
    MsgHuman2Rob_ MsgHuman2Rob;
} MsgHuman2RobFrame_;

//-------------以上是有人端发送给机器人----------------//

typedef struct
{
    // struct tagPCHeader msgHeader;
    int msgID;
    union
    {
        struct RH_SENDSTATUSFEEDBACK rh_send_status_feedback_; // 反馈的信息
        struct RH_PRODUCTIONINFO rh_production_info_;          // 产品的信息
        struct RH_Task_State rh_task_state_;                   // 任务ID和状态
    };
} Rob2MsgHuman_;

typedef struct
{
    // uint16_t crc16;
    struct tagChkPCHeader msgHeader;
    Rob2MsgHuman_ Rob2MsgHuman;
} Rob2MsgHumanFrame_;

//---------------以上是机器人端发给有人端-----------------//

typedef struct
{
    short DeviceControlMode;                   // 控制模式 1:步进控制 2:自主控制
    MsgHuman2RobFrame_ ShareMsgHuman2RobFrame; // 接收有人端指令信息
} ShareMsgHuman2RobCMD;

#pragma pack()

#endif /* INCLUDE_COMMUNICATIONPROTOCOL_H_ */
