#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <glog/logging.h>
#include <sstream>
#include <stdexcept>

class IniParser
{
public:
    using Section = std::map<std::string, std::string>;
    using IniData = std::map<std::string, Section>;

    // 读取 INI 文件
    void load(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + filename);
        }

        std::string line;
        std::string currentSection;
        while (std::getline(file, line))
        {
            line = trim(line); // 去掉多余的空格

            // 跳过空行或注释
            if (line.empty() || line[0] == ';' || line[0] == '#')
                continue;

            // 如果是节 [Section]
            if (line[0] == '[' && line.back() == ']')
            {
                currentSection = line.substr(1, line.size() - 2);
                iniData[currentSection]; // 确保节存在
            }
            // 如果是 key=value
            else
            {
                size_t pos = line.find('=');
                if (pos != std::string::npos)
                {
                    std::string key = trim(line.substr(0, pos));
                    std::string value = trim(line.substr(pos + 1));
                    iniData[currentSection][key] = value;
                }
            }
        }
        file.close();
    }

    // 保存 INI 文件
    void save(const std::string &filename) const
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + filename);
        }

        // 遍历 iniData
        for (auto sectionIt = iniData.begin(); sectionIt != iniData.end(); ++sectionIt)
        {
            const std::string &section = sectionIt->first; // 获取节名
            const Section &keys = sectionIt->second;       // 获取键值对

            if (!section.empty())
            {
                file << "[" << section << "]\n";
            }

            // 遍历键值对
            for (auto keyIt = keys.begin(); keyIt != keys.end(); ++keyIt)
            {
                const std::string &key = keyIt->first;    // 获取键
                const std::string &value = keyIt->second; // 获取值
                file << key << "=" << value << "\n";
            }

            file << "\n"; // 节之间空行分隔
        }

        file.close();
    }

    // 获取指定节名和键的值
    std::string get(const std::string &section, const std::string &key, const std::string &defaultValue = "") const
    {
        auto secIt = iniData.find(section);
        if (secIt != iniData.end())
        {
            auto keyIt = secIt->second.find(key);
            if (keyIt != secIt->second.end())
            {
                return keyIt->second;
            }
        }
        return defaultValue;
    }

    // 设置值
    void set(const std::string &section, const std::string &key, const std::string &value)
    {
        iniData[section][key] = value;
    }

    // 打印 INI 数据
    // const修饰符，表示这个方法不会修改类的成员变量
    // 适用于只读操作，确保方法的安全性
    void print() const
    {
        // 遍历INI数据的节
        for (auto sectionIt = iniData.begin(); sectionIt != iniData.end(); ++sectionIt)
        {
            // 提取节名和键值对
            const std::string &section = sectionIt->first; // 获取节名
            const Section &keys = sectionIt->second;       // 获取键值对
            // 打印节名
            LOG(INFO) << "[" << section << "]\n";
            // 遍历节中的键值对
            for (auto keyIt = keys.begin(); keyIt != keys.end(); ++keyIt)
            {
                // 提取键和值
                const std::string &key = keyIt->first;    // 获取键
                const std::string &value = keyIt->second; // 获取值
                LOG(INFO) << key << "=" << value << "\n";
            }
            LOG(INFO) << "\n";
        }
    }

private:
    IniData iniData;

    // 工具函数：去掉首尾空格
    static std::string trim(const std::string &str)
    {
        const char *whitespace = " \t\n\r\f\v";
        size_t start = str.find_first_not_of(whitespace);
        size_t end = str.find_last_not_of(whitespace);
        return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
    }
};
int _ini_config();
