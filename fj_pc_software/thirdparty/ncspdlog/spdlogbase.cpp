#include "spdlogbase.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

//#include "consoleformatter.h"
namespace NC {
struct SpdlogBasePrivate
{
    std::shared_ptr<spdlog::logger> m_pLogger;  ///< 根日志对象
    std::string m_strLoggerName;                ///< Logger名称
    std::vector<spdlog::sink_ptr> m_vecSinks;   ///< Sinks列表

    SpdlogBasePrivate()
    {

    }
    ~SpdlogBasePrivate()
    {
    }
    SpdlogBasePrivate(SpdlogBasePrivate &&) = default;
    SpdlogBasePrivate(const SpdlogBasePrivate &) = default;
    SpdlogBasePrivate &operator=(SpdlogBasePrivate &&) = default;
    SpdlogBasePrivate &operator=(const SpdlogBasePrivate &) = default;
};
}
using namespace std;
using namespace NC;


/**
* @brief std::string 转为std::wstring 解决中文路径的不识别问题
*
* @param str std::string字符串
*/
#if defined(_WIN32) && defined(SPDLOG_WCHAR_FILENAMES)
std::wstring StringToWString(const std::string& str)
{
    int num = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, NULL, 0);
    wchar_t *wide = new wchar_t[num];
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, wide, num);
    std::wstring w_str(wide);
    delete[] wide;
    return w_str;
}
#endif

SpdlogBase::SpdlogBase()
    :d(new SpdlogBasePrivate())
{
}

SpdlogBase::~SpdlogBase()
{
   
}

void SpdlogBase::setLoggerName(const std::string &loggerName)
{
    d->m_strLoggerName = loggerName;
}

void SpdlogBase::appendConsoleSink(int logLevel, const std::string &logPattern)
{
    auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink->set_level(static_cast<spdlog::level::level_enum>(logLevel));
    //#if defined(__MINGW32__) || defined(__MINGW64__)
    //    sink->set_formatter(std::unique_ptr<spdlog::formatter>(new spdlog::console_formatter(logPattern)));
    //#else
    sink->set_pattern(logPattern);
    //#endif
    d->m_vecSinks.push_back(sink);
}

void SpdlogBase::appendBasicFileSink(int logLevel, const std::string &logPattern, const std::string &fullFilePath, bool truncate)
{
#if (defined(_WIN32) && defined(SPDLOG_WCHAR_FILENAMES))
    std::wstring strFullPath = StringToWString(fullFilePath);
    if (strFullPath.empty())
    {
        return;
    }
    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(strFullPath, truncate);
#else
    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(fullFilePath, truncate);
#endif
    sink->set_pattern(logPattern);
    sink->set_level(static_cast<spdlog::level::level_enum>(logLevel));
    d->m_vecSinks.push_back(sink);
}

void SpdlogBase::appendDailyFileSink(int logLevel, const std::string &logPattern, const std::string &fullFilePath, int rotationHour, int rotationMinute, bool truncate)
{
#if (defined(_WIN32) && defined(SPDLOG_WCHAR_FILENAMES))
    std::wstring strFullPath = StringToWString(fullFilePath);
    if (strFullPath.empty())
    {
        return;
    }
    auto sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(strFullPath, rotationHour, rotationMinute, truncate);
#else
    auto sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(fullFilePath, rotationHour, rotationMinute, truncate);
#endif
    sink->set_pattern(logPattern);
    sink->set_level(static_cast<spdlog::level::level_enum>(logLevel));
    d->m_vecSinks.push_back(sink);
}


void SpdlogBase::appendRotatingFileSink(int logLevel, const std::string &logPattern, const std::string &fullFilePath, std::size_t maxSize, std::size_t maxFiles)
{
#if (defined(_WIN32) && defined(SPDLOG_WCHAR_FILENAMES))
    std::wstring strFullPath = StringToWString(fullFilePath);
    if (strFullPath.empty())
    {
        return;
    }
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(strFullPath, maxSize, maxFiles);
#else
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(fullFilePath, maxSize, maxFiles);
#endif
    sink->set_pattern(logPattern);
    sink->set_level(static_cast<spdlog::level::level_enum>(logLevel));
    d->m_vecSinks.push_back(sink);
}

void SpdlogBase::appendCustomSink(spdlog::sink_ptr pSink)
{
    std::vector<spdlog::sink_ptr> & sinks = d->m_pLogger->sinks();
    sinks.push_back(pSink);
}

void SpdlogBase::setupLogger()
{
    setupLogger(d->m_strLoggerName + " logger is setup.");
    logStartup();
}

void SpdlogBase::setupLogger(const std::string &introMessage)
{
    try
    {
        if (d->m_vecSinks.empty())
        {
            // 如果客户端没有增加Sink，则默认增加一个控制台输出
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern(PATTERN_NORMAL_CONSOLE);
#if (defined(_DEBUG) || defined(DEBUG))
            console_sink->set_level(spdlog::level::trace);
#else
            console_sink->set_level(spdlog::level::debug);
#endif
            d->m_vecSinks.push_back(console_sink);
        }
        // 实例化Logger
        d->m_pLogger = std::make_shared<spdlog::logger>(d->m_strLoggerName, begin(d->m_vecSinks), end(d->m_vecSinks));
#if (defined(_DEBUG) || defined(DEBUG))
        d->m_pLogger->set_level(spdlog::level::trace);
#else
        d->m_pLogger->set_level(spdlog::level::debug);
#endif
        // 当日志输出有错误时的，回调函数
        d->m_pLogger->set_error_handler([this](const std::string &msg) {
            std::cout << "*** " << d->m_strLoggerName << " LOGGER HAS ERROR ***: " << msg << std::endl;
        });
        //register it if you need to access it globally
        spdlog::register_logger(d->m_pLogger);
        //设置当立刻刷新日志到disk的日志级别
        d->m_pLogger->flush_on(spdlog::level::warn);
        if (!introMessage.empty())
            d->m_pLogger->info(introMessage);
    } // Exceptions will only be thrown upon failed logger or sink construction (not during logging)
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Log init failed: unknow" << std::endl;
    }
}

void SpdlogBase::shutdownLogger()
{
    logShutdown();
    shutdownLogger(d->m_strLoggerName + " logger was shutdown.");
}

void SpdlogBase::shutdownLogger(const std::string &extroMessage)
{
    try
    {
        if (!extroMessage.empty())
            d->m_pLogger->info(extroMessage);
        // 执行spdlog最后的关闭
        spdlog::drop(d->m_strLoggerName);
    }
    // Exceptions will only be thrown upon failed logger or sink construction (not during logging)
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log shutdown failed: " << ex.what() << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cout << "Log shutdown failed: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Log shutdown failed: unknow" << std::endl;
    }
}

std::shared_ptr<spdlog::logger> SpdlogBase::logger()
{
    return d->m_pLogger;
}

void SpdlogBase::flush()
{
    d->m_pLogger->flush();
}

void SpdlogBase::logStartup()
{
    d->m_pLogger->info("####################################################");
    d->m_pLogger->info("#                    START                         #");
    d->m_pLogger->info("####################################################");
}

void SpdlogBase::logShutdown()
{
    d->m_pLogger->info("####################################################");
    d->m_pLogger->info("#                    STOP                          #");
    d->m_pLogger->info("####################################################");
}
