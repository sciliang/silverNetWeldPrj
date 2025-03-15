#include "communication.h"
#include "TCP_Protocol.h"
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <glog/logging.h>
#include <thread>
#include <arpa/inet.h>
#define MAX_PACKET_SIZE 1049
#define PORT 8080
Rob2MsgHumanFrame_ msgRob2HumanFrame;       // 用于给界面端发送反馈信息
ShareMsgHuman2RobCMD ShareMsgHuman2RobCMD_; // 用于接收界面端的信息和指令

communication::communication(/* args */) : MsgPackage_(false)
{
}

communication::~communication()
{
}

int communication::initSocket()
{
    LOG(INFO) << "initSocket start.." << std::endl;
    socklen_t addrLen = sizeof(clientAddr);
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Failed to create socket\n";
    }
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(PORT);

    if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cerr << "Bind failed\n";
        close(serverSocket);
    }

    if (listen(serverSocket, 5) < 0)
    {
        std::cerr << "Listen failed\n";
        close(serverSocket);
    }

    // LOG(INFO) << "Server is listening on port " << PORT << std::endl;

    clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &addrLen);
    if (clientSocket < 0)
    {
        std::cerr << "Failed to accept connection\n";
        close(serverSocket);
    }
    LOG(INFO) << "Connection accepted\n";
    return 0;
}

void communication::processData(int clientSocket, MsgPackage &msgPackage_)
{
    uint8_t buffer[MAX_PACKET_SIZE];
    ssize_t bytesRead = 0;
    static int32_t recvNUM = 0;
    while ((bytesRead = read(clientSocket, buffer, sizeof(buffer))) > 0)
    {
        // 这部分有消息进来,就是Qt模式打开
        std::vector<uint8_t> data(buffer, buffer + bytesRead);
        msgPackage_.TcpSocket_.clientSocket = clientSocket; // liuchen
        msgPackage_.updateState(data);
        msgPackage_.unPkg3(data);
    }

    // 处理读取返回值为 0 或负值的情况
    if (bytesRead == 0)
    {
        // 连接关闭
        LOG(INFO) << "Connection closed by peer." << std::endl;
    }
    else if (bytesRead < 0)
    {
        // 读取失败，处理 errno
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            // 非阻塞模式下，没有可读取的数据
            LOG(INFO) << "No data available to read at the moment (non-blocking mode)." << std::endl;
        else
            // 其他错误，输出错误信息
            std::cerr << "Error reading from socket: " << strerror(errno) << std::endl;
    }
}

int communication::TCPTele()
{
    initSocket();
    processData(clientSocket, MsgPackage_);
    close(clientSocket);
    close(serverSocket);
    return 0;
}

int communication::TCPSend()
{
    ssize_t bytesSent;
    std::vector<MessageType> ids = {RH_PRODUCTION_INFO_ID};
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (MessageType id : ids)
        {
            MsgPackage_.send(id);
            bytesSent = send(clientSocket, &msgRob2HumanFrame, sizeof(msgRob2HumanFrame), 0);
            if (bytesSent == -1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // std::cerr << "Send failed." << std::endl;
            }
        }
    }
    return 0;
}