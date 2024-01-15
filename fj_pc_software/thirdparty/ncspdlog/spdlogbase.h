#pragma once
#include "ncspdlog_global.h"
#include <string>
#include <vector>
#include <iostream>
#include "spdlog/common.h"
#include "spdlog/spdlog.h"
#include "spdlog/fmt/fmt.h"
#include "spdlog/fmt/bundled/printf.h"

namespace NC
{
struct SpdlogBasePrivate;
/**
 * @brief Spdlog的封装的基类
 * 
 */
class NCSPDLOG_EXPORT SpdlogBase
{
  public:
    SpdlogBase();
    virtual ~SpdlogBase();

  public:
    /**
     * @brief 添加控制台日志输出接收器
     * 
     * @param logLevel 日志输出级别 trace = 0,debug = 1,info = 2,warn = 3,err = 4,critical = 5,off = 6
     * @param logPattern 日志输出格式
     */
    void appendConsoleSink(int logLevel,
                           const std::string &logPattern);
    /**
     * @brief 添加单日志文件接收器
     *
     * @param logLevel 日志输出级别 trace = 0,debug = 1,info = 2,warn = 3,err = 4,critical = 5,off = 6
     * @param logPattern 日志输出格式
     * @param fullFilePath 日志文件完整路径，包括日志文件名和日志文件后缀，指定的日志文件目录必须已经存在。
     * @param truncate 是否截断
     */
    void appendBasicFileSink(int logLevel,
                             const std::string &logPattern,
                             const std::string &fullFilePath,
                             bool truncate);
    /**
     * @brief 添加以日期为单位的日志文件接收器
     * 
     * @param logLevel 日志输出级别 trace = 0,debug = 1,info = 2,warn = 3,err = 4,critical = 5,off = 6
     * @param logPattern 日志输出格式
     * @param fullFilePath 日志文件完整路径，包括日志文件名和日志文件后缀，指定的日志文件目录必须已经存在。
     * @param rotationHour 日志文件生成的小时
     * @param rotationMinute 日志文件生成的分钟
     * @param truncate  是否截断
     */
    void appendDailyFileSink(int logLevel,
                             const std::string &logPattern,
                             const std::string &fullFilePath,
                             int rotationHour,
                             int rotationMinute,
                             bool truncate);
    /**
     * @brief 添加以文件大小为单位的日志文件接收器
     * 
     * @param logLevel 日志输出级别 trace = 0,debug = 1,info = 2,warn = 3,err = 4,critical = 5,off = 6
     * @param logPattern 日志输出格式
     * @param fullFilePath 日志文件完整路径，包括日志文件名和日志文件后缀，指定的日志文件目录必须已经存在。
     * @param maxSize 单个日志文件的最大大小，字节为单位
     * @param maxFiles 最大日志文件个数
     */
    void appendRotatingFileSink(int logLevel,
                                const std::string &logPattern,
                                const std::string &fullFilePath,
                                std::size_t maxSize,
                                std::size_t maxFiles);
    /**
     * @brief 追加自定义的sink
     * @param pSink
     */
    void appendCustomSink(spdlog::sink_ptr pSink);

    /**
     * @brief 初始化Logger对象
     * 
     */
    void setupLogger();  
    void setupLogger(const std::string &introMessage);  
        /**
     * @brief 关闭Logger对象
     * 
     */
    void shutdownLogger();
    void shutdownLogger(const std::string &extroMessage);
    /**
     * @brief 返回当前的Logger对象
     * 
     * @return std::shared_ptr<spdlog::logger> 
     */
    std::shared_ptr<spdlog::logger> logger();
    /**
    * @brief 刷新日志信息到文本
    */
    void flush();
    /**
    * @brief 设置Logger的唯一标示，必须在setupLogger之前调用
    *
    * @param loggerName  Logger的名称
    */
    void setLoggerName(const std::string &loggerName);
protected:
    /**
     * @brief 输出一条日志库初始化信息
     * 
     */
    virtual void logStartup();
    /**
     * @brief 输出一条日志关闭信息
     * 
     */
    virtual void logShutdown();
private:
    SpdlogBase(SpdlogBase const &) = delete;
    SpdlogBase& operator=(SpdlogBase const &) = delete;
protected:
    SpdlogBasePrivate *d = nullptr;
    
};

const char PATTERN_NORMAL_CONSOLE[]="%H:%M:%S.%e [%t] [%l] [%n] %v";                    ///< 控制台默认的日志输出格式
const char PATTERN_NORMAL_FILE[]="%Y-%m-%d %H:%M:%S.%e [%t] [%l] [%n] %v";              ///< 文件默认的日志输出格式
} // namespace NC
