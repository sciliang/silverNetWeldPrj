#include <iostream>
#include <unistd.h>
#include "ControlCanFun.h"
#include "YinwMotion.h"
#include <cmath>
#include <string.h>
#include <vector> // 添加vector头文件
#include <cstdint>
#include "HYYRobotInterface.h"
#include "device_interface.h"

/**每加工一片银网，2号钛虎电机需要转过对应的角度，即 2号电机控制量的计算，步进量是不变的
 *para1：钛虎电机的减速比 51
 *para2：钛虎电机的轴径 22mm
 *para3：银网的直径长度
 **/
float SecondStepCtrlValue(float motorEnd_ReductionRatio, float MotorDiameter, float SilverNetD)
{
    SilverNetD = SilverNetD + 5;                                       // 给出极片直径的基础上5mm的余量
    float MotorEnd_Distance = SilverNetD;                              // 电机需要运动的距离
    float motorEnd_C = M_PI * MotorDiameter;                           // 电机的周长,电机外围没有网滚，因此是电机的周长
    float motorEnd_RunAngle = MotorEnd_Distance / motorEnd_C * 360.0;  // 电机需要转动的角度
    float motorRunAngle = motorEnd_RunAngle * motorEnd_ReductionRatio; // 电机端需要转过的角度值
    LOG(INFO) << "motorRunAngle = " << motorRunAngle << std::endl;
    return motorRunAngle;
}

/**每加工一片银网, 4号电机控制量的计算，4号网滚的直径是变化的，因此计算更复杂些
 *para1：钛虎电机的减速比 51
 *para2：钛虎电机的轴径 22 mm
 *para3：银网的直径长度 mm
 *para4：4号电机累积前进的圈数(电机前进的圈数是已知的)
 *para5：银网的厚度
 *para6：初始卷的银网的总厚度(包括轴径)(当下带着银网的整个的厚度是已知的)
 * */
float FourthStepCtrlValue(float motorEnd_ReductionRatio, float MotorDiameter, float SilverNetD,
                          int SilverNetCircleNum, float SilverNetThick, float SilverNetAllThick)
{
    // 给出极片直径的基础上5mm的余量
    float MotorEnd_Distance = SilverNetD + 5;
    // 求出当下的电机末端带银网的厚度(包括轴径)
    float MotorEnd_D = SilverNetAllThick - SilverNetCircleNum * 2 * SilverNetThick;
    if (MotorEnd_D - MotorDiameter <= 2 * SilverNetThick)
    {
        LOG(INFO) << "ERROR, SilverNet May handling over!" << std::endl;
        return -1;
    }
    else
    {
        // 整个4号电机网滚的周长
        float motorEnd_C = M_PI * MotorEnd_D;
        // 末端转过的圈数()
        float MotorEndRunAngle = MotorEnd_Distance / motorEnd_C * 360.0;
        // 电机需要转过的角度
        float MotorRunAngle = MotorEndRunAngle * motorEnd_ReductionRatio;
        LOG(INFO) << "MotorRunAngle = " << MotorRunAngle << std::endl;
        return MotorRunAngle;
    }
    return -1;
}

/**每加工一片银网, 1号电机控制量的计算，前进一片极片的距离
 *para1：钛虎电机的减速比 51
 *para2：银网的直径长度 mm
 *para3：1号电机最上面顶点转动的总圈数
 *para4：银网的厚度
 *para5：1号网滚的初始厚度)
 * */
float FirstStepCtrlValue(float motorEnd_ReductionRatio, float SilverNetD, int SilverNetCircleNum, float SilverNetThick, float SilverNetAllThick)
{
    float MotorEnd_Distance = SilverNetD + 5;                                       // 给出极片直径的基础上5mm的余量
    float MotorEnd_D = SilverNetAllThick + SilverNetCircleNum * 2 * SilverNetThick; // 求出当下的电机末端带银网的厚度(包括轴径)
    float motorEnd_C = M_PI * MotorEnd_D;                                           // 电机的周长
    float MotorEndRunAngle = MotorEnd_Distance / motorEnd_C * 360.0;                // 末端转过的角度度数
    float MotorRunAngle = MotorEndRunAngle * motorEnd_ReductionRatio;               // 电机需要转过的角度
    LOG(INFO) << "MotorRunAngle = " << MotorRunAngle << std::endl;
    return MotorRunAngle;
}
