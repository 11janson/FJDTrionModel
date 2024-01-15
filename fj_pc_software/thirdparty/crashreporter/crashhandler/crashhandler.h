#pragma once

#include <QtCore/QString>
#include "crashreporter_global.h"
#include <functional>
typedef std::function<void(const QString &strDump)> crashFunCallbackType;
class CrashHandlerPrivate;
class CRASHREPORTER_EXPORT CrashHandler
{
public:
    static CrashHandler* instance();
    void Init(const QString&  reportPath, 
        const QString &appPath, 
        const QString &crashReportDir,
        crashFunCallbackType = nullptr);

    void setReportCrashesToSystem(bool report);
    bool writeMinidump();

private:
    CrashHandler();
    ~CrashHandler();
    Q_DISABLE_COPY(CrashHandler)
        CrashHandlerPrivate* d;
};

