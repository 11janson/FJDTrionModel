#pragma once
#include "ncspdlog_global.h"
#include <string>
#include <vector>
#include <iostream>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/fmt.h"
#include "spdlog/fmt/bundled/printf.h"
#include "spdlogbase.h"

namespace NC {
/**
 * @brief 模块级的spdlog封装类，可用于制定的Logger输出
 *
 */
class NCSPDLOG_EXPORT SpdlogModule:public SpdlogBase
{
public:
    /**
     * @brief 构造一个制定名称的Logger
     * 
     * @param loggerName Logger的名称
     */
    SpdlogModule(const std::string &loggerName);
    virtual ~SpdlogModule() ;
public:
    /**
     * @brief 获取制定名称的Logger对象
     * 
     * @param name Logger名称
     * @return std::shared_ptr<spdlog::logger> 
     */
    static std::shared_ptr<spdlog::logger> getLogger(const std::string &name);
    /**
     * @brief 检测指定名称Logger是否已经存在
     * @param name Logger的名称
     * @return
     */
    static bool containsLogger(const std::string &name);
private:
    SpdlogModule(SpdlogModule const &) = delete;
    SpdlogModule& operator=(SpdlogModule const &) = delete;
};
} // namespace NC

/**
 * @brief 跟踪日志信息
 * 
 */
#define MODULE_TRACE_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->trace(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_TRACE_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->trace(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_TRACE_LOG(LoggerName,...) MODULE_TRACE_LOG_ARG(LoggerName,__VA_ARGS__)
/**
 * 调试日志信息
 * 
 */
#define MODULE_DEBUG_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->debug(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_DEBUG_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->debug(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_DEBUG_LOG(LoggerName,...) MODULE_DEBUG_LOG_ARG(LoggerName,__VA_ARGS__)

/**
 * 普通日志信息
 *
 */
#define MODULE_INFO_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->info(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_INFO_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->info(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_INFO_LOG(LoggerName,...) MODULE_INFO_LOG_ARG(LoggerName,__VA_ARGS__)
/**
 * 警告日志信息
 *
 */
#define MODULE_WARN_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->warn(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "]  not exist." << std::endl;
#define MODULE_WARN_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->warn(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_WARN_LOG(LoggerName,...) MODULE_WARN_LOG_ARG(LoggerName,__VA_ARGS__)
/**
 * 错误日志信息
 *
 */

#define MODULE_ERROR_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->error(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_ERROR_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->error(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_ERROR_LOG(LoggerName,...) MODULE_ERROR_LOG_ARG(LoggerName,__VA_ARGS__)
/**
 * @brief 严重错误日志
 * 
 */
#define MODULE_CRITICAL_LOG_FMT(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->critical(TRACE_SUFFIX(fmt::format(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_CRITICAL_LOG_ARG(LoggerName,...) \
    if(nullptr != NC::SpdlogModule::getLogger(LoggerName)) \
        NC::SpdlogModule::getLogger(LoggerName)->critical(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__))); \
    else \
        std::cout << "WARING:Module Logger [" LoggerName << "] not exist." << std::endl;
#define MODULE_CRITICAL_LOG(LoggerName,...) MODULE_CRITICAL_LOG_ARG(LoggerName,__VA_ARGS__)
