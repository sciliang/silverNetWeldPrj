#include "HYYRobotInterface.h"
#include "globalfun.h"
#include "TcpTeleGlobalStruct.h"
#include "msgPackage.h"
#include <glog/logging.h>
using namespace HYYRobotBase;

MsgPackage::MsgPackage()
{
    // 预设头标志和长度
    sig.resize(4);
    sig[0] = 0xff;
    sig[1] = 0xfd;
    sig[2] = 0xfe;
    sig[3] = 0xff;
    len = 0;
    checksum = 0;
    isCheckSum = false;
    pkgData.clear();
}

MsgPackage::MsgPackage(bool c)
{
    // 预设头标志和长度
    sig.resize(4);
    sig[0] = 0xff;
    sig[1] = 0xfd;
    sig[2] = 0xfe;
    sig[3] = 0xff;
    len = 0;
    checksum = 0;
    isCheckSum = c;
}

MsgPackage::~MsgPackage() {}

uint32_t MsgPackage::calcCRC32(const char *data_ptr, int size)
{
    static const uint32_t crc32Table[] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832,
        0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
        0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a,
        0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
        0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3,
        0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
        0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
        0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4,
        0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce, 0xa3bc0074,
        0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
        0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525,
        0x206f85b3, 0xb966d409, 0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615,
        0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76,
        0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
        0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b, 0xd80d2bda, 0xaf0a1b4c, 0x36034af6,
        0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
        0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
        0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7,
        0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278,
        0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
        0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330,
        0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

    uint32_t crc = 0xFFFFFFFF;
    for (int i = 0; i < size; ++i)
    {
        crc = crc32Table[(crc ^ data_ptr[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}

// 将整数转换为字节数组
std::vector<uint8_t> MsgPackage::intToBytes(uint32_t i)
{
    std::vector<uint8_t> bytes(sizeof(uint32_t));
    bytes[0] = static_cast<uint8_t>((i >> 24) & 0xFF);
    bytes[1] = static_cast<uint8_t>((i >> 16) & 0xFF);
    bytes[2] = static_cast<uint8_t>((i >> 8) & 0xFF);
    bytes[3] = static_cast<uint8_t>(i & 0xFF);
    return bytes;
}

// 将字节数组转换为整数
uint32_t MsgPackage::bytesToInt(const std::vector<uint8_t> &bytes)
{
    if (bytes.size() < sizeof(uint32_t))
    {
        return 0;
    }
    uint32_t value = 0;
    value |= static_cast<uint32_t>(bytes[0]) << 24;
    value |= static_cast<uint32_t>(bytes[1]) << 16;
    value |= static_cast<uint32_t>(bytes[2]) << 8;
    value |= static_cast<uint32_t>(bytes[3]);
    return value;
}

// 计算数据包的校验和
uint32_t MsgPackage::getCheckSum(const std::vector<uint8_t> &data)
{
    return calcCRC32(reinterpret_cast<const char *>(data.data()), data.size());
}

// 在数据中查找包头标志的位置
int MsgPackage::seekHeader(const std::vector<uint8_t> &data, int from)
{
    // LOG(INFO) << "seekHeader:" << std::endl;
    // LOG(INFO) << "sig.size():" << sig.size() << std::endl;
    // LOG(INFO) << "data.size():" << data.size() << std::endl;
    // LOG(INFO) << "sig content = ";
    // for (const auto &byte : sig)
    // {
    //     LOG(INFO) << std::hex << static_cast<int>(byte) << " ";
    // }
    // LOG(INFO) << std::dec << std::endl;
    for (int i = from; i <= static_cast<int>(data.size()) - static_cast<int>(sig.size()); ++i)
    {
        bool found = true;
        for (size_t j = 0; j < sig.size(); ++j)
        {
            if (data[i + j] != sig[j])
            {
                // LOG(INFO) << "find package ID failed!" << std::endl;
                found = false;
                break;
            }

            // 只是打印
            if (i + j < data.size())
            // LOG(INFO) << "data[" << i << " + " << j << "] = " << std::hex << static_cast<int>(data[i + j]) << std::endl;
            {
            }
            else
                LOG(INFO) << "Index out of range" << std::endl;
        }

        if (found)
        {
            // LOG(INFO) << "find package ID, POS = " << i << std::endl;
            return i; // 找到包头 返回起始位置索引
        }
    }
    return -1; // 没有找到包头
}

// 清空数据包内容
void MsgPackage::clearPkg()
{
    LOG(INFO) << "清空数据包:clearPkg now!" << std::endl;
    len = 0;
    checksum = 0;
    pkgData.clear();
    tmpPkgData.clear();
    LOG(INFO) << "\n";
}

// 组装数据包
std::vector<uint8_t> MsgPackage::mkPkg(const std::vector<uint8_t> &data)
{
    LOG(INFO) << "mkPkg start!\n";
    clearPkg();
    len = data.size();
    pkgData = data;
    if (isCheckSum)
    {
        checksum = getCheckSum(pkgData);
    }

    std::vector<uint8_t> packet;
    packet.insert(packet.end(), sig.begin(), sig.end());
    std::vector<uint8_t> lenBytes = intToBytes(len);
    packet.insert(packet.end(), lenBytes.begin(), lenBytes.end());
    if (isCheckSum)
    {
        std::vector<uint8_t> checksumBytes = intToBytes(checksum);
        packet.insert(packet.end(), checksumBytes.begin(), checksumBytes.end());
    }
    packet.insert(packet.end(), pkgData.begin(), pkgData.end());
    return packet;
}

// 获取包头长度
int MsgPackage::headerLength() const
{
    // LOG(INFO) << "(sig.size()) + sizeof(len) + sizeof(checksum) = " << std::dec << (sig.size()) + sizeof(len) + sizeof(checksum) << std::endl;
    return sig.size() + sizeof(uint32_t) + sizeof(uint32_t);
}

// 检测目前包的状态
// 检测len,pkgData,checksum,tmpHeaderData状态，判断是应该接收包头开始的数据还是包剩余数据
int MsgPackage::checkWhatToDo(const std::vector<uint8_t> &data) const
{
    /**
     len为0,意味着目前没有要处理的包,tmpPkgData为空(true),表示没有残留的临时数据,
     这种情况下，说明之前的数据包处理已经完成，现在需要等待一个新的完整的数据包
     ！等待接收新的包！
    */
    LOG(INFO) << "进入检测目前包的状态" << "\n";
    if (len == 0 && tmpPkgData.empty())
    {
        LOG(INFO) << "判断为情况1:len=0 and tmpPkgData.empty" << std::endl;
        LOG(INFO) << "\n";
        return 1;
    }
    /**
     len为0,意味着目前没有要处理的包,tmpPkgData为有数据(false),表示有残留的数据,
     这种情况下，说明之前接收到了一部分数据,但还没有形成一个完整的数据包,需要等待接收剩余的数据来完成这个包
     ！等待接收剩余的包数据！
    */
    else if (len == 0 && !tmpPkgData.empty())
    {
        LOG(INFO) << "判断为情况2:len=0 and !tmpPkgData.empty" << std::endl;
        LOG(INFO) << "\n";
        return 2;
    }
    /**
     len>0,意味着正处理的一个数据包,期望接收一个完整的数据包。tmpPkgData为空(true),表示当前没有残留的数据,
     这种情况下，说明已经开始接收一个新的数据包,且之前的数据包已经处理好,现在等待接收剩余的数据包
     ！等待接受剩余的包内容数据！
    */
    else if (len > 0 && tmpPkgData.empty())
    {
        LOG(INFO) << "判断为情况3:len>0 and tmpPkgData.empty" << std::endl;
        LOG(INFO) << "\n";
        return 3;
    }
    LOG(INFO) << "未知错误" << std::endl;
    LOG(INFO) << "\n";
    return -1; // 未知错误
}

void MsgPackage::processHeader(const std::vector<uint8_t> &data, int index)
{
    // 舍弃包头标志前面的脏数据
    std::vector<uint8_t> cleanedData(data.begin() + index, data.end());
    LOG(INFO) << "processHeader!" << std::endl;
    if (cleanedData.size() > headerLength())
    {
        // 获取包数据长度 包头大小+数据长度len大小
        len = bytesToInt2(cleanedData.data() + sig.size() + sizeof(uint32_t)); // 1037
        LOG(INFO) << "data len = " << len << std::endl;
        // 获取包校验和
        checksum = bytesToInt2(cleanedData.data() + sig.size());
        LOG(INFO) << "data checksum = " << checksum << std::endl;
        // 如果数据长度大于0,就进行解包
        if (len > 0)
        {
            unPkg2(std::vector<uint8_t>(cleanedData.begin() + headerLength(), cleanedData.end()));
        }
        // 如果数据小于等于0,就清空数据包[说明没有发过来数据]
        else
        {
            // 清空数据包
            LOG(INFO) << "strat clearPkg!" << std::endl;
            clearPkg();
        }
    }
    else
    {
        tmpPkgData = cleanedData;
    }
}

void MsgPackage::mergeDataAndProcess(const std::vector<uint8_t> &data)
{
    // 合并数据并处理
    std::vector<uint8_t> mergeData(tmpPkgData); // 把tmpPkgData给mergeData,tmpPkgData固定
    LOG(INFO) << "mergeDataAndProcess\n";
    mergeData.insert(mergeData.end(), data.begin(), data.end());
    // 清空数据包
    clearPkg(); // 把tmpPkgData清除了
    if (mergeData.size() < MAX_DATA_LEN)
    {
        unPkg2(mergeData);
    }
    else
    {
        LOG(INFO) << "[error] data is too bad!\n";
    }
}

void MsgPackage::processRemainingData(const std::vector<uint8_t> &data)
{
    TcpSocket TcpSocket_;
    int surplusLen = len - pkgData.size();
    LOG(INFO) << "processRemainingData\n";
    LOG(INFO) << "surplusLen = " << surplusLen << std::endl;

    // 如果实际发送的数据长度大于data的大小
    if (surplusLen > data.size())
    {
        LOG(INFO) << "surplusLen > data.size()" << std::endl;
        pkgData.insert(pkgData.end(), data.begin(), data.end());
    }
    // 如果实际发送的数据长度小于data的大小
    else
    {
        LOG(INFO) << "surplusLen < data.size()" << std::endl;
        pkgData.insert(pkgData.end(), data.begin(), data.begin() + surplusLen);
        LOG(INFO) << "pkgData:" << pkgData.size() << std::endl;
        TcpSocket_.pkgReady(pkgData);
        // 清空数据包
        clearPkg();
        unPkg2(std::vector<uint8_t>(data.begin() + surplusLen, data.end()));
    }
}

// 这段代码的作用是将长度为4字节的字节数组转换为 uint32_t 类型的整数
uint32_t MsgPackage::bytesToInt2(const uint8_t *bytes) const
{
    uint32_t value;
    std::memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

// 更新状态的方法
void MsgPackage::updateState(const std::vector<uint8_t> &data)
{
    // LOG(INFO) << "加入临时内存变量:MsgPackage updateState" << "\n";
    tmpPkgData.insert(tmpPkgData.end(), data.begin(), data.end());
    // LOG(INFO) << "\n";
}

void MsgPackage::unPkg2(const std::vector<uint8_t> &data)
{
    LOG(INFO) << "进入unPKg2" << std::endl;
    switch (checkWhatToDo(data))
    {
    // if (len == 0 && tmpPkgData.empty())情况
    case 1: // 等待接收新的包
    {
        // 寻找包头标志位置
        LOG(INFO) << "进入case1:in  case 1 !" << std::endl;
        int index = seekHeader(data, 0);
        // 舍弃包头标志前面的脏数据
        if (index >= 0)
        {
            LOG(INFO) << "find package header!, headerPOS =" << index << std::endl;
            // 如果数据长度len大于0,就进行解包,小于0 清空temp
            processHeader(data, index);
        }
        else
        {
            LOG(INFO) << "find package header failed!, headerPOS =" << index << std::endl;
            tmpPkgData = data;
        }
        break;
    }
    // if (len == 0 && !tmpPkgData.empty())情况
    case 2: // 等待接收剩余的包数据
    {
        LOG(INFO) << "进入case2:in  case 2 !" << std::endl;
        mergeDataAndProcess(data);
        break;
    }
    // if (len > 0 && tmpPkgData.empty())情况
    case 3: // 等待接受剩余的包内容数据
    {
        LOG(INFO) << "进入case3:in  case 3 !" << std::endl;
        processRemainingData(data);
        break;
    }
    default:
        LOG(INFO) << "进入case default:[error] unknown pack error\n";
        break;
    }
}

// 解析数据包
void MsgPackage::unPkg(const std::vector<uint8_t> &data)
{
    LOG(INFO) << "unPkg start..." << std::endl;
    tmpPkgData.insert(tmpPkgData.end(), data.begin(), data.end());
    // 寻找包头
    int headerPos = seekHeader(tmpPkgData, 0);
    if (headerPos == -1)
    {
        LOG(INFO) << "Can't find header" << std::endl;
        return; // 没有找到包头
    }

    if (tmpPkgData.size() < headerLength())
    {
        LOG(INFO) << "data isn't completed - 1" << std::endl;
        return; // 数据不完整
    }

    tmpPkgData.erase(tmpPkgData.begin(), tmpPkgData.begin() + headerPos);

    if (tmpPkgData.size() < headerLength())
    {
        LOG(INFO) << "data isn't completed - 2" << std::endl;
        return; // 数据不完整
    }

    if (!isCheckSum)
    {
        len = bytesToInt(std::vector<uint8_t>(tmpPkgData.begin() + sig.size(), tmpPkgData.begin() + sig.size() + sizeof(len)));
        pkgData = std::vector<uint8_t>(tmpPkgData.begin() + sig.size() + sizeof(len), tmpPkgData.end());
        tmpPkgData.clear();
    }
    else
    {
        len = bytesToInt(std::vector<uint8_t>(tmpPkgData.begin() + sig.size(), tmpPkgData.begin() + sig.size() + sizeof(len)));
        checksum = bytesToInt(std::vector<uint8_t>(tmpPkgData.begin() + sig.size() + sizeof(len), tmpPkgData.begin() + sig.size() + sizeof(len) + sizeof(checksum)));
        pkgData = std::vector<uint8_t>(tmpPkgData.begin() + sig.size() + sizeof(len) + sizeof(checksum), tmpPkgData.end());
        tmpPkgData.clear();
        uint32_t calcChecksum = getCheckSum(pkgData);
        if (calcChecksum == checksum)
        {
            LOG(INFO) << "CheckSum Ok!" << std::endl;
        }
        else
        {
            LOG(INFO) << "CheckSum Error!" << std::endl;
        }
    }
}

void MsgPackage::unPkg3(const std::vector<uint8_t> &data)
{
    int index = seekHeader(data, 0);
    std::vector<uint8_t> cleanedData(data.begin() + index, data.end());    // 清空脏数据，新的保存到cleanedData
    len = bytesToInt2(cleanedData.data() + sig.size() + sizeof(uint32_t)); // 获取当前包数据的长度
    checksum = bytesToInt2(cleanedData.data() + sig.size());
    TcpSocket_.pkgReady(std::vector<uint8_t>(cleanedData.begin() + headerLength(), cleanedData.end()));
}

void MsgPackage::send(MessageType id)
{
    // memset(&msgRob2HumanFrame, 0, sizeof(msgRob2HumanFrame));
    msgRob2HumanFrame.msgHeader.sig[0] = 0xff;
    msgRob2HumanFrame.msgHeader.sig[1] = 0xfd;
    msgRob2HumanFrame.msgHeader.sig[2] = 0xfe;
    msgRob2HumanFrame.msgHeader.sig[3] = 0xff;

    msgRob2HumanFrame.Rob2MsgHuman.msgID = id;

    msgRob2HumanFrame.msgHeader.len = sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader);

    msgRob2HumanFrame.msgHeader.cksum = MsgPackage::calcCRC32((char *)&msgRob2HumanFrame.Rob2MsgHuman, sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader));
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.she_bei_zhuang_tai = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.sheng_chan_zong_shu = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.he_ge_shu = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.bu_he_ge_shu = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.jie_lun = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.ji_pian_ku_shang_liao = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.he_ge_pin_ku_qu_liao = num;
    msgRob2HumanFrame.Rob2MsgHuman.rh_production_info_.fei_pin_ku_qu_liao = num;
    msgRob2HumanFrame.msgHeader.cksum = MsgPackage::calcCRC32((char *)&msgRob2HumanFrame.Rob2MsgHuman, sizeof(msgRob2HumanFrame) - sizeof(msgRob2HumanFrame.msgHeader));

    // packet_send.resize(sizeof(msgRob2HumanFrame),0);

    // memcpy(packet_send.data(),&msgRob2HumanFrame, sizeof(msgRob2HumanFrame));
}

// void MsgPackage::GetTaiHuFeedback()
// {
//     short ActuatorNUM = 3; // 执行器的数量，目前是我随便设置的
//     // 运动控制指令
//     uint8_t canidlist[ActuatorNUM] = {1}; // 指令列表
//     uint8_t cmd_pos[ActuatorNUM] = {30};  // 以位置模式进行控制
//     uint8_t cmd_vel[ActuatorNUM] = {29};  // 以速度模式进行控制
//     static int32_t ActualPos_save, ActualCur_save;
//     // LOG(INFO) << "TaiHuMsgFlag = " << TaiHuMsgFlag << std::endl;
//     switch (TaiHuMsgFlag)
//     {
//     case 1:
//     {
//         uint8_t cmd_get_pos[ActuatorNUM] = {8}; // 获取实际位置
//         int32_t ActualPos = motionControl_->sendSimpleCanCommand(1, canidlist, cmd_get_pos);
//         // LOG(INFO) << "ActualPos=" << ActualPos << std::endl;
//         WG_position = ActualPos;
//         // LOG(INFO) << "WG_position=" << WG_position << std::endl;
//         TaiHuMsgFlag = 2;
//         if (abs(WG_position) == 0)
//         {
//             WG_position = ActualPos_save;
//         }

//         break;
//     }
//     case 2:
//     {
//         uint8_t cmd_get_pos[ActuatorNUM] = {4}; // 获取实际电流
//         int32_t ActualCur = motionControl_->sendSimpleCanCommand(1, canidlist, cmd_get_pos);
//         msgRob2HumanFrame.Rob2MsgHuman.RH_WG_WZDL_.position = ActualCur;
//         // LOG(INFO) << "ActualCur=" << ActualCur << std::endl;
//         WG_current = ActualCur;
//         // LOG(INFO) << "WG_current=" << WG_current << std::endl;
//         TaiHuMsgFlag = 1;
//         if (abs(WG_current) == 0)
//         {
//             WG_current = ActualCur_save;
//         }
//         break;
//     }
//     default:
//         LOG(INFO) << "ERROR, unKnown Status" << std::endl;
//         break;
//     }
//     if (WG_position != 0)
//         ActualPos_save = WG_position;

//     if (WG_current != 0)
//         ActualCur_save = WG_current;
// }

// 数据包已准备好
// void MsgPackage::pkgReady(const std::vector<uint8_t> &data)
// {
//     if (isCheckSum)
//     {
//         // 验证校验和
//         uint32_t cksum = getCheckSum(data);
//         if (cksum == checksum)
//         {
//             LOG(INFO) << "[congratulation] check sum is right: " << cksum << " " << checksum << std::endl;
//             // 调用回调函数
//             if (cksum == checksum)
//             {
//                 // packageReadyCallback(data);
//             }
//         }
//         else
//         {
//             std::cerr << "[error pack] check sum error: " << cksum << " " << checksum << std::endl;
//         }
//     }
//     else
//     {
//         LOG(INFO) << "pkg ready, data length: " << data.size() << std::endl;
//         // 调用回调函数
//         // if (packageReadyCallback)
//         // {
//         //     packageReadyCallback(data);
//         // }
//     }
// }
