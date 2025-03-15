#pragma once
#ifndef COMUNICATION_H_
#define COMUNICATION_H_
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "msgPackage.h"

class communication
{
private:
    struct sockaddr_in serverAddr, clientAddr;
    int clientSocket, serverSocket;
    MsgPackage MsgPackage_;

public:
    communication(/* args */);
    ~communication();
    void processData(int clientSocket, MsgPackage &msgPackage_);
    void receiveMessage(int clientSock);
    int initSocket();
    int TCPTele();
    void sendStateFeedBack(int taskID);
    int TCPSend();
};
#endif
