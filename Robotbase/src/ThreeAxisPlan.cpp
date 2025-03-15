#include "YinwMotionGlobal.h"
using namespace HYYRobotBase;

/**
 * fun：这是给三坐标进行动态规划用的，暂时没用上，后来决定用示教的方式，就没有再继续弄了
 */
int16_t YinwMotion::ThreeAxisDevicePlan(const YinwDeviceParas &deviceParas, const char *deviceName, short MotorNUM, int AimPosX, int AimPosY, int AimPosZ)
{
    // 根据三坐标设备的长、宽、高，来求解三坐标对应的四个顶点的坐标
    int deviceLeftLowerP1[3] = {2000, 1500, 3000}; // 以这个点为坐标原点
    int deviceRightLowerP2[3] = {5000, 1500, 3000};
    int deviceRightFrontP3[3] = {5000, 4500, 3000};
    int deviceLeftFrontP4[3] = {2000, 4500, 3000};
    // 进行二维坐标映射（俯视图）
    int deviceLeftLowerP1_x = 0, deviceLeftLowerP1_y = 0;
    int deviceRightLowerP2_x = deviceRightLowerP2[0] - deviceLeftLowerP1[0];
    int deviceRightLowerP2_y = deviceRightLowerP2[1] - deviceLeftLowerP1[1];
    int deviceRightFrontP3_x = deviceRightFrontP3[0] - deviceLeftLowerP1[0];
    int deviceRightFrontP3_y = deviceRightFrontP3[1] - deviceLeftLowerP1[1];
    int deviceLeftFrontP4_x = deviceLeftFrontP4[0] - deviceLeftLowerP1[0];
    int deviceLeftFrontP4_y = deviceLeftFrontP4[0] - deviceLeftLowerP1[0];

    // 根据焊接设备的长、宽、高，来求解三坐标对应的四个顶点的坐标
    int weldLeftLowerP1[3] = {3000, 2500, 3000};
    int weldRightLowerP2[3] = {4000, 2500, 3000};
    int weldRightFrontP3[3] = {4000, 3500, 3000};
    int weldLeftFrontP4[3] = {3000, 3500, 3000};
    // 进行二维坐标映射（俯视图）
    int weldLeftLowerP1_x = weldLeftLowerP1[0] - deviceLeftLowerP1[0];
    int weldLeftLowerP1_y = weldLeftLowerP1[1] - deviceLeftLowerP1[1];
    int weldRightLowerP2_x = weldRightLowerP2[0] - deviceLeftLowerP1[0];
    int weldRightLowerP2_y = weldRightLowerP2[1] - deviceLeftLowerP1[1];
    int weldRightFrontP3_x = weldRightFrontP3[0] - deviceLeftLowerP1[0];
    int weldRightFrontP3_y = weldRightFrontP3[1] - deviceLeftLowerP1[1];
    int weldLeftFrontP4_x = weldLeftFrontP4[0] - deviceLeftLowerP1[0];
    int weldLeftFrontP4_y = weldLeftFrontP4[1] - deviceLeftLowerP1[1];
    // 获取三坐标当前的位置
    LOG(INFO) << "Device Name: " << deviceName << std::endl;
    int MotorCurrentPos[MotorNUM], curDevicePos[3];
    // 获取期望位置
    if (get_group_target_position(deviceName, MotorCurrentPos) != 0)
    {
        LOG(INFO) << "get_group_target_position ERROR!" << endl;
    }
    for (size_t i = 0; i < MotorNUM; i++)
    {
        LOG(INFO) << "MotorCurrentPos[" << MotorCurrentPos[i] << "]" << endl;
    }
    // 三坐标当前的位置：X、Y、Z
    curDevicePos[0] = MotorCurrentPos[ThreeAxisXmotor - 1];
    curDevicePos[1] = MotorCurrentPos[ThreeAxisYmotor - 1];
    curDevicePos[2] = MotorCurrentPos[ThreeAxisZmotor - 1];
    // 当前位置坐标映射
    int curDevicePos_x = curDevicePos[0] - deviceLeftLowerP1[0];
    int curDevicePos_y = curDevicePos[1] - deviceLeftLowerP1[1];
    // 目标位置坐标映射
    int _AimPosX = AimPosX - deviceLeftLowerP1[0];
    int _AimPosY = AimPosY - deviceLeftLowerP1[1];

    // 将交点的结果进行存储，以待后面使用
    std::vector<std::vector<int>> insectSequences;
    // 1 代表左侧交点，2代表右侧交点，3代表
    short insectionSpecies;

    // 构建当前位置到目标点的直线方程
    // 斜线
    if ((_AimPosX != curDevicePos_x) && (_AimPosY != curDevicePos_y))
    {
        LOG(INFO) << "Oblique line!" << endl;
        float linearSlope = (_AimPosY - curDevicePos_y) / (_AimPosX - curDevicePos_x);
        // 求该条直线和障碍物区域的交点
        // 与左侧边界的交点
        float leftEdgeInsecX = weldLeftLowerP1_x;
        float leftEdgeInsecY = linearSlope * (weldLeftLowerP1_x - curDevicePos_x) + curDevicePos_y;
        LOG(INFO) << "weldLeftLowerP1_y = " << weldLeftLowerP1_y << "leftEdgeInsecY = " << leftEdgeInsecY << endl;
        if ((leftEdgeInsecY >= weldLeftLowerP1_y) && (leftEdgeInsecY <= weldLeftFrontP4_y))
        {
            LOG(INFO) << "Left boundary has insection!!! leftEdgeInsecX = "
                      << leftEdgeInsecX << "leftEdgeInsecY = " << leftEdgeInsecY << endl;
        }

        // 与右侧边界的交点
        float rightEdgeInsecX = weldRightLowerP2_x;
        float rightEdgeInsecY = linearSlope * (weldRightLowerP2_x - curDevicePos_x) + curDevicePos_y;
        LOG(INFO) << "rightEdgeInsecX = " << rightEdgeInsecX << "leftEdgeInsecY = " << rightEdgeInsecX << endl;
        if ((rightEdgeInsecY >= weldRightLowerP2_y) && (rightEdgeInsecY <= weldRightFrontP3_y))
        {
            LOG(INFO) << "Right boundary has insection!!! rightEdgeInsecX = "
                      << rightEdgeInsecX << "rightEdgeInsecY = " << rightEdgeInsecY << endl;
        }

        // 与上边界的交点
        float upEdgeInsecX = (weldLeftFrontP4_y - curDevicePos_y) / linearSlope + curDevicePos_x;
        float upEdgeInsecY = weldLeftFrontP4_y;
        LOG(INFO) << "upEdgeInsecX = " << upEdgeInsecX << "upEdgeInsecY = " << upEdgeInsecY << endl;
        if ((upEdgeInsecX >= weldLeftFrontP4_x) && (upEdgeInsecX <= weldRightFrontP3_x))
        {
            LOG(INFO) << "upEdge boundary has insection!!! upEdgeInsecX = "
                      << upEdgeInsecX << "upEdgeInsecY = " << upEdgeInsecY << endl;
        }

        // 与下边界的交点
        float lowEdgeInsecX = (weldLeftLowerP1_y - curDevicePos_y) / linearSlope + curDevicePos_x;
        float lowEdgeInsecY = weldLeftLowerP1_y;
        LOG(INFO) << "lowEdgeInsecX = " << lowEdgeInsecX << "lowEdgeInsecY = " << lowEdgeInsecY << endl;
        if ((lowEdgeInsecX >= weldLeftLowerP1_x) && (lowEdgeInsecX <= weldRightLowerP2_x))
        {
            LOG(INFO) << "upEdge boundary has insection!!! upEdgeInsecX = "
                      << upEdgeInsecX << "upEdgeInsecY = " << upEdgeInsecY << endl;
        }
    }
    // 竖线
    else if ((_AimPosX == curDevicePos_x) && (_AimPosY != curDevicePos_y))
    {
        LOG(INFO) << "Vertical line!" << endl;
        // 上边界交点
        float upEdgeInsecX2 = curDevicePos_x;
        float upEdgeInsecY2 = weldLeftFrontP4_y;
        LOG(INFO) << "upEdgeInsecX2 = " << upEdgeInsecX2 << "upEdgeInsecY2 = " << upEdgeInsecY2 << endl;
        if ((upEdgeInsecX2 >= weldLeftFrontP4_x) && (upEdgeInsecX2 <= weldRightFrontP3_x))
        {
            LOG(INFO) << "upEdge boundary has insection!!! upEdgeInsecX2 = "
                      << upEdgeInsecY2 << "upEdgeInsecY2 = " << upEdgeInsecY2 << endl;
        }
        // 下边界交点
        float lowEdgeInsecX2 = curDevicePos_x;
        float lowEdgeInsecY2 = weldRightLowerP2_y;
        LOG(INFO) << "lowEdgeInsecX2 = " << lowEdgeInsecX2 << "lowEdgeInsecY2 = " << lowEdgeInsecY2 << endl;
        if ((lowEdgeInsecX2 >= weldLeftLowerP1_x) && (upEdgeInsecX2 <= weldRightLowerP2_x))
        {
            LOG(INFO) << "LowEdge boundary has insection!!! lowEdgeInsecX2 = "
                      << lowEdgeInsecX2 << "lowEdgeInsecY2 = " << lowEdgeInsecY2 << endl;
        }
    }
    // 横线
    else if ((_AimPosX != curDevicePos_x) && (_AimPosY == curDevicePos_y))
    {
        LOG(INFO) << "Transverse line!" << endl;
        // 左边界交点
        float leftEdgeInsecX2 = weldLeftLowerP1_x;
        float leftEdgeInsecY2 = curDevicePos_y;
        LOG(INFO) << "leftEdgeInsecX2 = " << leftEdgeInsecX2 << "leftEdgeInsecY2 = " << leftEdgeInsecY2 << endl;
        if ((leftEdgeInsecY2 >= weldLeftLowerP1_y) && (leftEdgeInsecY2 <= weldLeftFrontP4_y))
        {
            LOG(INFO) << "Left boundary has insection!!! leftEdgeInsecX2 = "
                      << leftEdgeInsecX2 << "leftEdgeInsecY2 = " << leftEdgeInsecY2 << endl;
        }
        // 右边界交点
        float rightEdgeInsecX2 = weldRightLowerP2_x;
        float rightEdgeInsecY2 = curDevicePos_y;
        LOG(INFO) << "rightEdgeInsecX2 = " << rightEdgeInsecX2 << "rightEdgeInsecY2 = " << rightEdgeInsecY2 << endl;
        if ((rightEdgeInsecY2 >= weldRightLowerP2_y) && (rightEdgeInsecY2 <= weldRightFrontP3_y))
        {
            LOG(INFO) << "Right boundary has insection!!! rightEdgeInsecX2 = "
                      << rightEdgeInsecX2 << "rightEdgeInsecY2 = " << rightEdgeInsecY2 << endl;
        }
    }
    else
    {
        LOG(INFO) << "The aimPos may be the same as currentPos!" << endl;
    }
    // 后面准备示教了 这部分就暂时不写了！
    return 0;
}
