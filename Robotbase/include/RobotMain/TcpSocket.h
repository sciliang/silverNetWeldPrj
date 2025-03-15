#pragma once
#ifndef TCPSOCKET_H_
#define TCPSOCKET_H_
#include <iostream>
#include <vector>
#include "TCP_Protocol.h"
#include <iomanip>
#include <sys/socket.h>
#include <vector>

class TcpSocket
{
private:
public:
    int32_t DL[5000];    // 存储动力点
    float DL_time[5000]; // 存储动力点
    int32_t ZL[5000];    // 存储阻力点
    float ZL_time[5000]; // 存储动力点
    int clientSocket;
    TcpSocket(/* args */);
    ~TcpSocket();
    void pkgReady(const std::vector<uint8_t> &data);
    void sendStateFeedBack(int rcvStatus, int taskID);     // liuchen
    void sendTaskStateFeedBack(int taskID, int taskState); // liuchen
    uint32_t calcCRC32(const char *data_ptr, int size);
};
#endif