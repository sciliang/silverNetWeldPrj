#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <sstream>
#include <stdexcept>
#include <glog/logging.h>
#include "IniParser.h"
#include <unistd.h> // getcwd
#include <limits.h> // PATH_MAX
/**用来进行IniParser类来加载、读取、修改、保存和打印INI的配置文件
 *
 */
int _ini_config()
{
    // 定义和初始化
    IniParser ini;
    char cwd[PATH_MAX];
    std::string configPath;
    try
    {
        // 获取当前程序所在路径（运行时路径）
        if (getcwd(cwd, sizeof(cwd)) != nullptr)
            LOG(INFO) << "Current working directory: " << cwd << std::endl;
        else
            std::cerr << "Error getting current working directory" << std::endl;
        configPath = std::string(cwd) + "/../config/config.ini";
        LOG(INFO) << "Loading config from: " << configPath << std::endl;
        ini.load(configPath.c_str());
    }
    // 用于捕获在try中抛出的异常(决定能够捕获哪种类型的异常)
    // exception是标准库中的基类，用于描述所有标准异常，如果函数或者库抛出了派生出的异常，catch能捕获到
    // 传进去是引用，避免拷贝异常对象，提升效率。使用引用后，可以访问原始的异常对象及方法
    // const修饰符，确保异常对象在catch块中不被修改
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // 打印 INI 数据
    LOG(INFO) << "2-Initial INI data:" << std::endl;
    ini.print();

    // 获取值
    std::string username = ini.get("General", "username", "defaultUser");
    LOG(INFO) << "3-Username: " << username << std::endl;

    // 修改值
    ini.set("General", "username", "new_admin");
    ini.set("Settings", "volume", "90");

    // 添加新值
    ini.set("NewSection", "newKey", "newValue");
    try
    {
        ini.load(configPath.c_str());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // 打印修改后的 INI 数据
    LOG(INFO) << "5-Updated INI data:" << std::endl;
    ini.print();

    return 0;
}
