#pragma onces
#include <iostream>
#include <string>
#include <unistd.h>
#include "controlcan.h"

bool init_can();
int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);
void toIntArray(int number, int *res, int size);
std::vector<int32_t> sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList);
void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
int TaiHuMotorControl();
int16_t TaiHuMotorVelLimitSet(int Vel_ID1, int Vel_ID2, int Vel_ID3, int Vel_ID4, int Vel_ID5);
int16_t TaiHuMotorEmergencyStop(int Vel_ID);
