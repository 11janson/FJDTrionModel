#pragma once
#include "ncspdlog/spdlogbase.h"
#include "ncspdlog/spdlogapplication.h"
#include "ncspdlog/spdlogmodule.h"
#include "spdlog/logger.h"
#include "spdlog/fmt/fmt.h"
#include <QtGlobal>
#include <QByteArray>
#include <QString>
#include <QDebug>
#include <QDir>
#include <QStandardPaths>
#include <QFileInfo>
#include <QCoreApplication>
namespace spdlog
{
    static void qt_message_fatal(QtMsgType, const QMessageLogContext& context, const QString& message);
    static bool isFatal(QtMsgType msgType);

#ifdef Q_OS_WIN
    static inline void convert_to_wchar_t_elided(wchar_t* d, size_t space, const char* s) Q_DECL_NOEXCEPT
    {
        size_t len = qstrlen(s);
        if (len + 1 > space)
        {
            const size_t skip = len - space + 4; // 4 for "..." + '\0'
            s += skip;
            len -= skip;
            for (int i = 0; i < 3; ++i)
                * d++ = L'.';
        }
        while (len--)
            * d++ = *s++;
        *d++ = 0;
    }
#endif
    static bool isFatal(QtMsgType msgType)
    {
        if (msgType == QtFatalMsg)
            return true;

        if (msgType == QtCriticalMsg)
        {
            static bool fatalCriticals = !qEnvironmentVariableIsEmpty("QT_FATAL_CRITICALS");
            return fatalCriticals;
        }

        if (msgType == QtWarningMsg || msgType == QtCriticalMsg)
        {
            static bool fatalWarnings = !qEnvironmentVariableIsEmpty("QT_FATAL_WARNINGS");
            return fatalWarnings;
        }

        return false;
    }

    static void qt_message_fatal(QtMsgType, const QMessageLogContext & context, const QString & message)
    {
#if defined(Q_CC_MSVC) && defined(QT_DEBUG) && defined(_DEBUG) && defined(_CRT_ERROR)
        wchar_t contextFileL[256];
        // we probably should let the compiler do this for us, by declaring QMessageLogContext::file to
        // be const wchar_t * in the first place, but the #ifdefery above is very complex  and we
        // wouldn't be able to change it later on...
        convert_to_wchar_t_elided(contextFileL, sizeof contextFileL / sizeof * contextFileL,
            context.file);
        // get the current report mode
        int reportMode = _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_WNDW);
        _CrtSetReportMode(_CRT_ERROR, reportMode);

        int ret = _CrtDbgReportW(_CRT_ERROR, contextFileL, context.line, _CRT_WIDE(QT_VERSION_STR),
            reinterpret_cast<const wchar_t*>(message.utf16()));
        if ((ret == 0) && (reportMode & _CRTDBG_MODE_WNDW))
            return; // ignore
        else if (ret == 1)
            _CrtDbgBreak();
#else
        Q_UNUSED(context);
        Q_UNUSED(message);
#endif

        std::abort();
    }
    /**
     * @brief QDebug信息的重定向
     * @param type
     * @param context
     * @param rMessage
     */
    static void qtMessageHandler(QtMsgType type, const QMessageLogContext & context, const QString & rMessage)
    {
        spdlog::level::level_enum level;
        switch (type)
        {
        case QtDebugMsg:
            level = spdlog::level::debug;
            break;
        case QtWarningMsg:
            level = spdlog::level::warn;
            break;
        case QtCriticalMsg:
            level = spdlog::level::err;
            break;
        case QtFatalMsg:
            level = spdlog::level::critical;
            break;
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
        case QtInfoMsg:
            level = spdlog::level::info;
            break;
#endif
        default:
            level = spdlog::level::trace;
            break;
        }
#if defined(QT_MESSAGELOGCONTEXT) && !defined(NOVALOG_NO_OUTPUT_CONTEXT)
        NC::SpdlogApplication::instance()->logger()->log(level,
            qPrintable(QString("%1\n//%2:%3#%4")
                .arg(rMessage)
                .arg(QFileInfo(context.file).fileName())
                .arg(context.function)
                .arg(context.line))
            );
#else
        NC::SpdlogApplication::instance().logger()->log(level,qPrintable(rMessage));
#endif

        // Qt fatal behaviour copied from global.cpp qt_message_output()
        // begin {
        /// TODO 暂不提供对严重错误的自动判断
        //if (isFatal(type))
        //    qt_message_fatal(type, context, rMessage);

        // } end
    }
    /**
     * @brief 重定向QDebug信息至SpdlogApplication
     *
     * @see SpdlogApplication
     */
    static void installMessageHandler()
    {
        qInstallMessageHandler(qtMessageHandler);
    }
    /**
     * @brief 获取默认的日志输出目录
     * @param const QString & 应用程序名称
     * @param const QString & 日志文件名称
     * @param const QString & 公司名称（简称）
     * @return 返回默认的日志输出目录
     */
    static QString getLogDirForQt(const QString & appName,
        const QString & logFileName,
        const QString & strCompanyShortName = "NovaStar")
    {
        QString fullLogPath = QDir::cleanPath(QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)));
        QStringList dirNames;
        dirNames = fullLogPath.split('/');
        if(dirNames.count()<=0)
            dirNames = fullLogPath.split('\\');
        Q_ASSERT(dirNames.count() >0 );
        if (!(QCoreApplication::applicationName().isNull() || QCoreApplication::applicationName().isEmpty()))
        {
            dirNames.removeLast();
        }
        if (!(QCoreApplication::organizationName().isNull() || QCoreApplication::organizationName().isEmpty()))
        {
            dirNames.removeLast();
        }
        fullLogPath = QDir::cleanPath(dirNames.join(QDir::separator()));
        if (!fullLogPath.endsWith(QDir::separator()))
            fullLogPath.append(QDir::separator());
        fullLogPath.append(strCompanyShortName);
        fullLogPath.append(QDir::separator());
        fullLogPath.append(appName);
        fullLogPath.append(QDir::separator());
        fullLogPath.append("log");
        fullLogPath.append(QDir::separator());
        fullLogPath = QDir::toNativeSeparators(fullLogPath);
        QDir dir(fullLogPath);
        if (!dir.exists(fullLogPath))
            dir.mkpath(fullLogPath);
        fullLogPath.append(logFileName);
        return fullLogPath;
    }

} //namespace spdlog


/**
 * @brief 跟踪日志信息
 *
 * @param qMsg 为QString类型的消息
 */
#define TRACE_LOG_QT(qMsg) TRACE_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_TRACE_LOG_QT(LoggerName,qMsg) MODULE_TRACE_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
 /**
  * 调试日志信息
  *
  * @param qMsg 为QString类型的消息
  */
#define DEBUG_LOG_QT(qMsg) DEBUG_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_DEBUG_LOG_QT(LoggerName,qMsg) MODULE_DEBUG_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
  /**
   * 普通日志信息
   *
   * @param qMsg 为QString类型的消息
   */
#define INFO_LOG_QT(qMsg) INFO_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_INFO_LOG_QT(LoggerName,qMsg) MODULE_INFO_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
   /**
    * 警告日志信息
    *
    * @param qMsg 为QString类型的消息
    */
#define WARN_LOG_QT(qMsg) WARN_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_WARN_LOG_QT(LoggerName,qMsg) MODULE_WARN_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
    /**
     * 错误日志信息
     *
     * @param qMsg 为QString类型的消息
     */
#define ERROR_LOG_QT(qMsg) ERROR_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_ERROR_LOG_QT(LoggerName,qMsg) MODULE_ERROR_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
     /**
      * @brief 严重错误日志
      *
      * @param qMsg 为QString类型的消息
      */
#define CRITICAL_LOG_QT(qMsg) CRITICAL_LOG_FMT(qMsg.toLocal8Bit().data())
#define MODULE_CRITICAL_LOG_QT(LoggerName,qMsg) MODULE_CRITICAL_LOG_FMT(LoggerName,qMsg.toLocal8Bit().data())
