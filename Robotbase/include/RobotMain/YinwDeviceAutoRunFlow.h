#ifndef DEVICEAUTORUNFLOW_H
#define DEVICEAUTORUNFLOW_H

float SecondStepCtrlValue(float motorEnd_ReductionRatio, float MotorDiameter, float SilverNetD);
float FirstStepCtrlValue(float motorEnd_ReductionRatio, float SilverNetD, int SilverNetCircleNum, float SilverNetThick, float SilverNetAllThick);
float FourthStepCtrlValue(float motorEnd_ReductionRatio, float MotorDiameter, float SilverNetD, int SilverNetCircleNum, float SilverNetThick, float SilverNetAllThick);

#endif
