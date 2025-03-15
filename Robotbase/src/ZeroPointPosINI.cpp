#include "YinwMotionGlobal.h"
#include "TCP_Protocol.h"

/**
 * fun：零点矩阵的存储
 */
void YinwMotion::ZeroPointPosINISave(std::vector<std::vector<int>> ZeroPointValue)
{
    deviceParas.ZeroPointPosINISave(ZeroPointValue);
}

/**
 * fun：系统参数的存储
 */
void YinwMotion::SystemParasSettingSave(MsgHuman2RobFrame_ ParaSettingData)
{
    deviceParas.SystemSettingParasSave(ParaSettingData);
}

/**
 * fun：零点矩阵转存之后的读取
 */
void YinwMotion::ZeroPointPosINIRead(std::vector<std::vector<int>> &YWzeroNameANDpos)
{
    // 获取返回值 (std::array<std::array<int, 2>, 8>)
    auto loadedZeroPoints = deviceParas.loadZeroNameAndPos();
    // 清空传入的 YWzeroNameANDpos 向量
    YWzeroNameANDpos.clear();
    // 将 loadedZeroPoints 转换为 std::vector<std::vector<int>>
    for (size_t i = 0; i < loadedZeroPoints.size(); ++i)
    {
        // 每一行都推入到 YWzeroNameANDpos
        YWzeroNameANDpos.push_back({loadedZeroPoints[i][0], loadedZeroPoints[i][1]});
    }
}