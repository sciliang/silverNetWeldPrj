#include <iostream>
#include <thread>
#include <glog/logging.h>
#include "communication.h"
#include "HYYRobotInterface.h"
#include "globalfun.h"
#include "device_interface.h"
#include "YinwMotion.h"
#include "YinwDeviceParas.h"
#include "IOdeviceControl.h"
#include "defineConfig.h"
int main(int argc, char *argv[])
{
    initGlog(argv[0]);
    YinwMotion YinwMotion_;
    communication communication_;
    YinwDeviceParas YinwDeviceParas_;
#ifdef ETHERCAT
    HYYRobotBase::command_arg arg;
    LOG_IF(ERROR, HYYRobotBase::commandLineParser(argc, argv, &arg) != 0)
        << "Project init error!";
    LOG_IF(ERROR, HYYRobotBase::system_initialize(&arg) != 0)
        << "System init error!";
    LOG_IF(ERROR, YinwMotion_.InitRescueEherCAT(1000) != 0)
        << "initEherCAT init error!";
#endif
    // 程序成功执行
    LOG(INFO) << "Initialization successful!";
    std::thread YwMotionPlan_Run(&YinwMotion::YwMotionPlanRun, &YinwMotion_, std::ref(YinwDeviceParas_));
    YinwMotion_.useDeviceParams(YinwDeviceParas_);
    std::thread SystemFaultDetct(&YinwMotion::YinwSystemFaultDetct, &YinwMotion_);
    std::thread CommunicationTCP(&communication::TCPTele, &communication_);
    std::thread CommunicationsendTCP(&communication::TCPSend, &communication_);
    YwMotionPlan_Run.join();
    CommunicationTCP.join();
    CommunicationsendTCP.join();
    SystemFaultDetct.join();
    return EXIT_SUCCESS;
}
