#ifndef SPDLOGAPPLICATION_H
#define SPDLOGAPPLICATION_H
#include "ncspdlog_global.h"
#include "spdlog/spdlog.h"
#include "spdlog/fmt/fmt.h"
#include "spdlog/fmt/bundled/printf.h"
#include "spdlogbase.h"

namespace NC
{
/**
 * @brief 应用程序级的spdlog封装类，主要提供了一个单例模式的Logger
 *
 */
class NCSPDLOG_EXPORT SpdlogApplication : public SpdlogBase
{
public:
    /**
     * @brief 返回一个单例的对象实例
     *
     * @return SpdLogWrapper& splog封装类
     */
    static SpdlogApplication *instance();
    virtual ~SpdlogApplication();
private:
    SpdlogApplication();
    SpdlogApplication(SpdlogApplication const &) = delete;
    SpdlogApplication& operator=(SpdlogApplication const &) = delete;
private:
};
} // namespace NC

/**
 * @brief 跟踪日志信息
 *
 */
#define TRACE_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->trace(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define TRACE_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->trace(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define TRACE_LOG(...) TRACE_LOG_ARG(__VA_ARGS__)
/**
 * 调试日志信息
 *
 */
#define DEBUG_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->debug(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define DEBUG_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->debug(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define DEBUG_LOG(...) DEBUG_LOG_ARG(__VA_ARGS__)

/**
 * 普通日志信息
 *
 */
#define INFO_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->info(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define INFO_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->info(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define INFO_LOG(...) INFO_LOG_ARG(__VA_ARGS__)
/**
 * 警告日志信息
 *
 */
#define WARN_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->warn(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define WARN_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->warn(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define WARN_LOG(...) WARN_LOG_ARG(__VA_ARGS__)
/**
 * 错误日志信息
 *
 */

#define ERROR_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->error(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define ERROR_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->error(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define ERROR_LOG(...) ERROR_LOG_ARG(__VA_ARGS__)
/**
 * @brief 严重错误日志
 *
 */
#define CRITICAL_LOG_FMT(...) NC::SpdlogApplication::instance().logger()->critical(TRACE_SUFFIX(fmt::format(__VA_ARGS__)))
#define CRITICAL_LOG_ARG(...) NC::SpdlogApplication::instance().logger()->critical(TRACE_SUFFIX(fmt::sprintf(__VA_ARGS__)))
#define CRITICAL_LOG(...) CRITICAL_LOG_ARG(__VA_ARGS__)

#endif
