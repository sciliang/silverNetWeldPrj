#ifndef IODEVICECONTROL_H
#define IODEVICECONTROL_H
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector> // 添加vector头文件
#include "controlcan.h"
#include <array>

void IODeviceTeleControl(const char *CmdName);
void send_IO_CanCommand(uint16_t CAN_ID, uint8_t *cmdLib);
std::vector<uint8_t> recv_IO_CanCommand();
void IO_X_PinsCmdSEND(const char *IO_name);
std::array<bool, 16> IO_X_PinsCmdGET();
uint8_t *createCanCommand(uint8_t Cmd2_1, uint8_t Cmd4_3, uint8_t Cmd6_5, uint8_t Cmd8_7,
                          uint8_t Cmd10_9, uint8_t Cmd12_11, uint8_t Cmd14_13, uint8_t Cmd16_15);
/**
 * 发的是1,就是IO板子1要相应的反馈;发的是2,就是板子2要相应的反馈
 * */
extern short IoBoardCanCmdName;
#endif