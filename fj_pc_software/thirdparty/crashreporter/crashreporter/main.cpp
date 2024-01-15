/* 
 * wiSCADA Crash Reporter
 * based on source from https://github.com/RedisDesktop
 */
#include <iostream>
#include "crashreporter.h"
#include "appwindow.h"
#include "version.h"

#include <QApplication>
#include <QMessageBox>
#include <QUrl>
#include <QDebug>
#include <QTimer>

int main( int argc, char* argv[] )
{
    QApplication app( argc, argv );
    if (argc < 2 || !QFile::exists(QString(argv[1])) )
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Crash Reporter");
        msgBox.setText(
                    "<b>Usage:</b> <br />"
                    " crashReporter <i>minidumpPath</i> [<i>crashedApplicationPath</i>]\n"
                    "<br /> <br /> <b>Example:</b> <br />"
                    " crashReporter 0120EDSDSD3.dmp [/usr/share/appFolder/app]\n"
                    "<br /><br />"
                    );
        msgBox.exec();
        return 1;
    }

    Config crashReporterConfig = {
        "Nova Star",
        "http://www.novastar-led.cn//crash-report",
        QLatin1String(NQ::Constants::IDE_VERSION_LONG),
        QString(argv[1])
    };

    QApplication::setApplicationName(QString("%1 Crash Reporter").arg(crashReporterConfig.productName));
    QApplication::setApplicationVersion(QLatin1String(NQ::Constants::IDE_VERSION_LONG));

    CrashReporter reporter(crashReporterConfig);
    AppWindow window;

    QObject::connect(&reporter, &CrashReporter::success, &window, &AppWindow::onSuccess);
    QObject::connect(&reporter, &CrashReporter::error, &window, &AppWindow::onError);
    QObject::connect(&reporter, &CrashReporter::uploadingProgress, &window, &AppWindow::onProgress);
    /// TODO 只存储本地，不发送至服务器
    //QTimer::singleShot( 1, &reporter, SLOT( send() ) );
    window.show();
    return app.exec();
}
