#pragma onces
#include <iostream>
#include <stdexcept>
#include <string>
#include <map>

// 自动控制流程步骤名称
enum YwDeviceAutoStep
{
    FifthStep = 5,
    SixthStep,
    SeventhStep,
    EighthStep,
    NinthStep,
    TenthStep,
    EleventhStep,
    TwelfthStep,
    ThirteenthStep,
    FourteenthStep,
    FifteenthStep,
    SixteenthStep,
    SeventeenthStep,
    EighteenthStep,
    NineteenthStep,
    TwentiethStep,
    TwentyfirstStep,
    TwentysecondStep,
    TwentythirdStep,
    TwentyfourthStep,
    TwentyfifthStep,
    TwentysixthStep,
    TwentyseventhStep,
    TwentyeighthStep,
    TwentyninethStep,
    ThirtiethStep,
    ThirtyfirtStep,
    ThirtySecondStep,
    ThirtyThirdStep,
    ThirtyFourthStep,
    ThirtyFifthStep,
    ThirtySixthStep,
    ThirtySeventhStep,
    ThirtyEighthStep,
    ThirtyNineStep,
    FortiethStep,
    FortyFirstStep,
    FortySecondStep,
    FortyThirdStep,
    FortyFourthStep,
    FortyFifthStep,
    FortySixthStep,
    FortySeventhStep,
    FortyEighthStep,
    FortyNinethStep,
    FiftiethStep,
    FiftyFistStep,
    FiftySecondStep,
    FiftyThirdStep,
    FiftyFourthStep,
    FiftyFifthStep,
    FiftySixthStep,
    FiftySeventhStep,
    FiftyEighthStep,
    FiftyNinethStep,
    SixtiehStep,
    SixtyFirstStep,
    SixtySecondStep,
    SixtyThirdStep,
};

int16_t YWDeviceRunFlow();
int16_t ThSetTargetVel_max(uint8_t *canidlist, uint32_t *vel);
int16_t ThSetTargetVel_min(uint8_t *canidlist, uint32_t *vel);
int16_t ThSetTargetPos(uint8_t *canidlist, uint32_t *pos);
std::int32_t THgetActualAxisPos(uint8_t Axis);
