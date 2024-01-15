#pragma once

namespace google_breakpad {
    class ExceptionHandler;
}

#include <QString>

class ExceptionHandler
{
public:
    typedef void(*CrashCallback)();

    ExceptionHandler(const QString& dumpFilePath,
        const QString& dumpReporterPath = QString::null,
        bool bIsReport = false,
        const QString& appVersion = "1.0.0",
        const QString& companyShortName = "NovaStar",
        CrashCallback callback = nullptr);

    ~ExceptionHandler();

    static QString getDumpDirDefault(const QString &appName,
        const QString &strCompanyShortName = "NovaStar");
private:
    void init(const QString& dumpFilePath,
        const QString& appVersion,
        const QString& dumpReporterPath,
        bool bIsReport,
        const QString& companyShortName,
        CrashCallback callback);
private:
    google_breakpad::ExceptionHandler *m_exceptionHandler = nullptr;

};



