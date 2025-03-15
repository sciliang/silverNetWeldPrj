#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;
/**
 * fun：焊接过程
 */
int16_t YinwMotion::WeldingProcess(const char *deviceName, short MotorNUM, short MotorName)
{
    LOG(INFO) << "Device Name: " << deviceName << std::endl;
    return 0;
}
/**
 * fun：在线检测
 */
bool YinwMotion::OnlineInspectionRecv(short Testflag)
{
    if (Testflag)
    {
        LOG(INFO) << " Good Product !!!" << std::endl;
        return true;
    }
    else
    {
        LOG(INFO) << " Bad Product !!!" << std::endl;
        return false;
    }
}
