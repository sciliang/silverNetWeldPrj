#pragma once
#ifndef MSGPACKAGE_H_
#define MSGPACKAGE_H_
#include <iostream>
#include <vector>
#include "TcpSocket.h"
#include <cstring>

// MsgPackage 类的定义（包含之前的实现）
class MsgPackage
{
private:
  std::vector<uint8_t> sig;
  uint32_t len;
  uint32_t checksum;
  bool isCheckSum;
  std::vector<uint8_t> pkgData;
  std::vector<uint8_t> tmpPkgData;
  static const uint32_t MAX_DATA_LEN = 65536;
  // 将整数转换为字节数组
  std::vector<uint8_t> intToBytes(uint32_t i);
  // 将字节数组转换为整数
  uint32_t bytesToInt(const std::vector<uint8_t> &bytes);
  // 计算数据包的校验和
  uint32_t getCheckSum(const std::vector<uint8_t> &data);
  // 在数据中查找包头标志的位置
  int seekHeader(const std::vector<uint8_t> &data, int from);

  int32_t WG_position;
  int16_t WG_current;
  short TaiHuMsgFlag = 1;

public:
  int num = 10;
  TcpSocket TcpSocket_;
  MsgPackage(bool c = false);
  MsgPackage();
  ~MsgPackage();
  uint32_t calcCRC32(const char *data_ptr, int size);
  // 清空数据包内容
  void clearPkg();
  // 获取包头长度
  int headerLength() const;
  // 组装数据包
  std::vector<uint8_t> mkPkg(const std::vector<uint8_t> &data);
  // 解析数据包
  void unPkg(const std::vector<uint8_t> &data);
  // 返回包是否准备好
  int checkWhatToDo(const std::vector<uint8_t> &data) const;
  // 数据包已准备好
  void pkgReady(const std::vector<uint8_t> &data);
  void processHeader(const std::vector<uint8_t> &data, int index);
  void mergeDataAndProcess(const std::vector<uint8_t> &data);
  void processRemainingData(const std::vector<uint8_t> &data);
  uint32_t bytesToInt2(const uint8_t *bytes) const;
  void unPkg2(const std::vector<uint8_t> &data);
  void unPkg3(const std::vector<uint8_t> &data);
  void updateState(const std::vector<uint8_t> &data);
  void send(MessageType id);
};
#endif
