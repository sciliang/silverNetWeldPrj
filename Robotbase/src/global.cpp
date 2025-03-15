#include <iostream>
#include <math.h>
#include "globalfun.h"
#include <sys/stat.h>
#include <glog/logging.h>
#include <unistd.h> // For access()
bool bRunning_Flag = true;

void sigroutine(int dunno) // sigroutine是信号处理例程,其中dunno将会得到信号的值.
{
    LOG(INFO) << "receive an quit signal!";
    //   if (dunno == SIGEXIT)
    bRunning_Flag = false;
}

// 前期使用的
//  void initGlog(char *argv)
//  {
//      google::InitGoogleLogging(argv); //初始化
//      google::SetStderrLogging(google::GLOG_INFO);
//      // 设置INFO WARINNING ERROR FATALD等 级别以上的信息log文件的路径和前缀名
//      google::SetLogDestination(google::GLOG_INFO, "log/INFO_");
//      google::SetLogDestination(google::GLOG_WARNING, "log/WARNING_");
//      google::SetLogDestination(google::GLOG_ERROR, "log/ERROR_");
//      google::SetLogDestination(google::GLOG_FATAL, "log/FATAL_");
//      FLAGS_colorlogtostderr = true; // 开启终端颜色区分
//  }

void initGlog(char *argv)
{
    google::InitGoogleLogging(argv); // 初始化
    google::SetStderrLogging(google::GLOG_INFO);

    // 获取构建目录的路径
    std::string log_dir = LOG_PATH; // 使用 CMake 传递的 LOG_PATH 宏
    // 确保日志目录存在
    if (access(log_dir.c_str(), F_OK) == -1)
    {
        // 目录不存在，创建它
        if (mkdir(log_dir.c_str(), 0755) == -1)
        {
            std::cerr << "Failed to create log directory: " << log_dir << std::endl;
            return;
        }
    }

    // 设置INFO WARINNING ERROR FATALD等 级别以上的信息log文件的路径和前缀名
    google::SetLogDestination(google::GLOG_INFO, (log_dir + "/INFO_").c_str());
    google::SetLogDestination(google::GLOG_WARNING, (log_dir + "/WARNING_").c_str());
    google::SetLogDestination(google::GLOG_ERROR, (log_dir + "/ERROR_").c_str());
    google::SetLogDestination(google::GLOG_FATAL, (log_dir + "/FATAL_").c_str());

    FLAGS_colorlogtostderr = true; // 开启终端颜色区分

    // 打印日志目录确认
    LOG(INFO) << "Logs will be stored in: " << log_dir << std::endl;
}