#pragma onces
#include <iostream>
#include <math.h>
#include "MovePlan.h"
#include "RobotSystem.h"
#include <mutex>
using namespace std;
#define Angle2Rad (180.0 / M_PI)
#define RGI

/**
 * @brief 信号处理回调函数，安全退出进程时候使用
 *
 */
extern bool bRunning_Flag;
void sigroutine(int dunno);
void initGlog(char *argv);
