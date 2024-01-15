//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <ccIncludeGL.h>

//Qt
#include <QDir>
#include <QMessageBox>
#include <QPixmap>
#include <QSettings>
#include "cloudcompareutils/icore.h"
#include <QSplashScreen>
#include <QTime>
#include <QTimer>
#include <QTranslator>
#ifdef CC_GAMEPAD_SUPPORT
#include <QGamepadManager>
#endif
#include <QPalette>
#include <QColor>
#include <QSettings>
#include <QStandardPaths>
#include <QSystemSemaphore>
#include <QTranslator>
#include <QDir>
#include <QFont>
#include <QFontMetrics>
#include <QFontDatabase>
#include <QTextCodec>
#include <QSharedMemory>
//qCC_db
#include <ccColorScalesManager.h>
#include <ccLog.h>
#include <ccNormalVectors.h>
//qCC_io
#include <FileIOFilter.h>
#include <ccGlobalShiftManager.h>
#include <QTimer>
#include <exception>

//local
#include "ccApplication.h"
#include "ccCommandLineParser.h"
#include "ccGuiParameters.h"
#include "ccPersistentSettings.h"
#include "mainwindow.h"
#include "ccTranslationManager.h"

//plugins
#include "ccPluginInterface.h"
#include "ccPluginManager.h"
#include <framelessmessagebox.h>
#ifdef USE_VLD
#include <vld.h>
#endif

#if Q_CC_MSVC
#include "crashreporter/crashhandler/crashhandler.h"
#include"writeminiDump.h"
#endif

#include "nqspdlog/nqspdlog.h"
#include "ncspdlog/spdlogbase.h"
#include "ncspdlog/spdlogapplication.h"
#include "cloudcompareutils/icore.h"
#include "csutils/hostosinfo.h"

#include "cloudcompareutils/publicutils.h"
#include "StackTracer.h"
#include <windows.h>
#include <excpt.h>
#include <stdio.h>
#include <QScreen>
#ifdef Q_OS_WIN32   //在windows系统下运行
#include <windows.h>
#include <process.h>
#include <Tlhelp32.h>
#include <winbase.h>
#include <string>
#include <tlhelp32.h>
#include <stdio.h>
#endif
using namespace std;
using namespace CS;

#ifdef _DEBUG
#pragma comment(linker, "/SUBSYSTEM:CONSOLE")
#endif


/**
* @brief dump文件存储路径
*/
static QString dumpDirPath()
{

	QString dumpPath = QString("%1/dump").arg(Core::ICore::getDefaultPath());
	QDir dirdump(dumpPath);
	if (!dirdump.exists(dumpPath))
		dirdump.mkpath(dumpPath);
	return dumpPath;
}

/**
* @brief 日志文件存储路径
*/
static QString logDirPath()
{
	return spdlog::getLogDirForQt(QLatin1String(IDE_TARGET_NAME), QString("%1.log").arg(QLatin1String(IDE_TARGET_NAME)),
		QLatin1String(""));
}


/**
* @brief 初始化系统日志配置
*/
static void initAppLogConfig()
{
	NC::SpdlogApplication::instance();
	QString logPath = logDirPath();
	QFile tgtfile(logPath);
	if (!tgtfile.exists())
	{
		if (tgtfile.open(QFile::ReadWrite | QFile::Text | QFile::Truncate))
		{
			tgtfile.close();
		}
	}
	qDebug() << "============================LOG PATH============================";
	qDebug() << logPath;
	qDebug() << "============================LOG PATH============================";
	NC::SpdlogApplication::instance()->appendConsoleSink(0, NC::PATTERN_NORMAL_CONSOLE);
	NC::SpdlogApplication::instance()->appendRotatingFileSink(0, NC::PATTERN_NORMAL_FILE, logPath.toStdString(), 1024 * 1024 * 20, 50);
	NC::SpdlogApplication::instance()->setupLogger();
	spdlog::installMessageHandler();

}

/**
* @brief 释放日志配置
*
*/
static void releaseApplicationLog()
{
	NC::SpdlogApplication::instance()->flush();
	NC::SpdlogApplication::instance()->shutdownLogger();
	spdlog::drop_all();
}


bool g_bAuthorizationEffective = false;


void HandleException()
{
    std::string str = StackTracer::GetExceptionMsg();
}

void StackTraceFunction()
{
    qDebug() << "crash stack trace";
    return;
}

static void crashCallBack(const QString &strDump)
{
    qDebug() << "###########################crash app##################################";

    //[!].打印崩溃堆栈轨迹
    StackTraceFunction();

    //[!].截取主框架图片
    releaseApplicationLog();// 关闭日志模块

    crashreturnfunction postObject;
    QProcess process;
    QStringList param;
    QString exePath = postObject.getCrashExePath();
    param = postObject.getCrashExePram();
    qDebug() << "param" << param.size();
    for (int i = 0; i < param.size(); i++)
    {
        qDebug() << "param:" << i << param.at(i);
    }
    process.startDetached(exePath, param);
    process.waitForStarted();

    return;
}

static void loadFonts()
{
	QString strPath = CS::Core::ICore::resourceSharePath()+ QLatin1String("/fonts/");
	const QDir dir(strPath);
	QStringList list;
	list << "*.ttf";
	list << "*.otf";

	foreach(const QFileInfo &fileInfo, dir.entryInfoList(list, QDir::Files))
	{
		QString str = fileInfo.absoluteFilePath();
		int id = QFontDatabase::addApplicationFont(fileInfo.absoluteFilePath());
	}
	if (dir.entryInfoList(list, QDir::Files).size() > 0) {
		QString msyh = QFontDatabase::applicationFontFamilies(0).at(0);
		QFont font(msyh);
		qApp->setFont(font);
	}
}
//加载全局样式表
static void loadGlobalqss()
{
	QFile qssFile(CS::Core::ICore::resourceSharePath() + "/dark/global.qss");
	if (qssFile.open(QIODevice::ReadOnly)) {
		qApp->setStyleSheet(qssFile.readAll());
		qssFile.close();
	}
	
}

#ifdef Q_OS_WIN32   //在windows系统下运行
static bool isProcessNameExist(const QString& strPrcessName)
{

	HANDLE hProcessSnap;
	PROCESSENTRY32 pe32;
	hProcessSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (hProcessSnap == INVALID_HANDLE_VALUE)
	{
		return(0);
	}
	pe32.dwSize = sizeof(PROCESSENTRY32);
	if (!Process32First(hProcessSnap, &pe32))
	{
		CloseHandle(hProcessSnap);          // clean the snapshot object
		return(0);
	}
	DWORD processId = 0;
	int processNum = 0;
	do
	{
		if (std::wstring(pe32.szExeFile) == strPrcessName.toStdWString())//进程名称
		{
			processId = pe32.th32ProcessID;//进程ID
			processNum++;
			//break;
		}
	} while (Process32Next(hProcessSnap, &pe32));
	CloseHandle(hProcessSnap);
	if (processNum > 1)
	{
		return true;
	}
	return false;
}
#endif

//判断软件是否以打开过
static bool isAlreadyRunning()
{
#ifdef Q_OS_WIN32   //在windows系统下运行
	QString applicationName = QString(IDE_TARGET_NAME) + QString(".exe");
	if (isProcessNameExist(applicationName))
	{
		return true;
	}
	
#endif


    static QSharedMemory sharedMemory;
    //设置共享内存的标识，这个标识是确定的
    sharedMemory.setKey(QString("TrionMetahub"));
    //尝试去attach由标识符指定的内存块，如果attach成功
    //说明这个内存段已经存在了，也就是说已经存在一个exe了那么就可以
    if (sharedMemory.attach())
    {
        return true;
    }
    else
    {
        //运行到此，说明还不存在exe，创建一个大小为1的共享内存段
        sharedMemory.create(1);
        return false;
    }
}

static const char *setHighDpiEnvironmentVariable()
{
	const char* envVarName = 0;
	static const char ENV_VAR_QT_DEVICE_PIXEL_RATIO[] = "QT_DEVICE_PIXEL_RATIO";

	if (Utils::HostOsInfo().isWindowsHost()
		&& !qEnvironmentVariableIsSet(ENV_VAR_QT_DEVICE_PIXEL_RATIO) // legacy in 5.6, but still functional
		&& !qEnvironmentVariableIsSet("QT_AUTO_SCREEN_SCALE_FACTOR")
		&& !qEnvironmentVariableIsSet("QT_SCALE_FACTOR")
		&& !qEnvironmentVariableIsSet("QT_SCREEN_SCALE_FACTORS"))
	{
		QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	}
	return envVarName;
}
#include <QApplication>
int main(int argc, char **argv)
{
    //qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "0");
    qputenv("QT_QUICK_BACKEND", QByteArray("software"));
	QStringList namelist;
	for (int i = 0; i < argc; ++i){
		namelist << QString::fromLocal8Bit(argv[i]);
	}

#ifdef _WIN32 //This will allow printf to function on windows when opened from command line	
	DWORD stdout_type = GetFileType(GetStdHandle(STD_OUTPUT_HANDLE));
	if (AttachConsole(ATTACH_PARENT_PROCESS))
	{
		if (stdout_type == FILE_TYPE_UNKNOWN) // this will allow std redirection (./executable > out.txt)
		{
			freopen("CONOUT$", "w", stdout);
			freopen("CONOUT$", "w", stderr);
		}
	}
#endif

#ifdef Q_OS_MAC
	// On macOS, when double-clicking the application, the Finder (sometimes!) adds a command-line parameter
	// like "-psn_0_582385" which is a "process serial number".
	// We need to recognize this and discount it when determining if we are running on the command line or not.

	int numRealArgs = argc;
	
	for ( int i = 1; i < argc; ++i )
	{
		if ( strncmp( argv[i], "-psn_", 5 ) == 0 )
		{
			--numRealArgs;
		}
	}
	
	bool commandLine = (numRealArgs > 1) && (argv[1][0] == '-');
#else
	bool commandLine = (argc > 1) && (argv[1][0] == '-');
#endif
   
	ccApplication::InitOpenGL();

#ifdef CC_GAMEPAD_SUPPORT
	QGamepadManager::instance(); //potential workaround to bug https://bugreports.qt.io/browse/QTBUG-61553
#endif
        
	
	
	ccApplication app(argc, argv, commandLine);

    if (isAlreadyRunning())
    {
        CS::Widgets::FramelessMessageBox::warning(nullptr, QObject::tr("Warning"), QObject::tr("The software is already running."));
        return -1;
    }
	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	const char *highDpiEnvironmentVariable = setHighDpiEnvironmentVariable();

	//store the log message until a valid logging instance is registered
	ccLog::EnableMessageBackup(true);

	if (highDpiEnvironmentVariable) {
        qunsetenv(highDpiEnvironmentVariable);
	}

	if (Utils::HostOsInfo().isWindowsHost()
		&& !qFuzzyCompare(qApp->devicePixelRatio(), 1.0))
	{
		QApplication::setStyle(QLatin1String("fusion"));
	}

	app.setAttribute(Qt::AA_UseHighDpiPixmaps);
	qputenv("QT_ENABLE_HIGHDPI_SCALING", "1");
	QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
	//[!] 初始化日志库
	initAppLogConfig();
	//[!]打印启动版本号
    qDebug() << "Model Start Version" << QString(FJDTRITONMETAHUB_VERSIONS);

	//[!] 初始化崩溃收集器

#if Q_CC_MSVC
	//QFileInfo appPath(QString::fromLocal8Bit(argv[0]));
	//const QString appDir(appPath.absoluteDir().path());


	//const QString crashReporterPath = QString("%1/crashreporter").arg(appDir.isEmpty() ? "." : appDir);
	//CrashHandler::instance()->Init(dumpDirPath(), QApplication::applicationFilePath(), crashReporterPath, crashCallBack);

    RunCrashHandlerLocal();
#endif


	//[!].初始化字体
	loadFonts();

	//[!].初始化样式表
	loadGlobalqss();
	//[!].初始化加密狗
	qDebug() << "Initialize the dongle!";

	//restore some global parameters
	{
		QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
		settings.beginGroup(ccPS::GlobalShift());
		double maxAbsCoord = settings.value(ccPS::MaxAbsCoord(), ccGlobalShiftManager::MaxCoordinateAbsValue()).toDouble();
		double maxAbsDiag = settings.value(ccPS::MaxAbsDiag(), ccGlobalShiftManager::MaxBoundgBoxDiagonal()).toDouble();
		settings.endGroup();

		ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2").arg(maxAbsCoord, 0, 'e', 0).arg(maxAbsDiag, 0, 'e', 0));
		
		ccGlobalShiftManager::SetMaxCoordinateAbsValue(maxAbsCoord);
		ccGlobalShiftManager::SetMaxBoundgBoxDiagonal(maxAbsDiag);
	}

	//specific commands
	int lastArgumentIndex = 1;
	if (commandLine)
	{
		//translation file selection
		if (QString(argv[lastArgumentIndex]).toUpper() == "-LANG")
		{
			QString langFilename = QString::fromLocal8Bit(argv[2]);

			ccTranslationManager::Get().loadTranslation(langFilename);
			commandLine = false;
			lastArgumentIndex += 2;
		}
	}

	//splash screen
	QScopedPointer<QSplashScreen> splash(nullptr);
	QTimer splashTimer;

	//standard mode
	if (!commandLine)
	{
		if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
		{
			CS::Widgets::FramelessMessageBox::critical(nullptr, "Error", "This application needs OpenGL 2.1 at least to run!");
			return EXIT_FAILURE;
		}

		//注释cc启动图标janson.yang2022.4.24
		////splash screen
		//QPixmap pixmap(QString::fromUtf8(":/CC/images/imLogoV2Qt.png"));
		//splash.reset(new QSplashScreen(pixmap, Qt::WindowStaysOnTopHint));
		//splash->show();
		//QApplication::processEvents();
	}

	//global structures initialization
	FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	//load the plugins
	ccPluginManager& pluginManager = ccPluginManager::Get();
	pluginManager.loadPlugins();
	
	int result = 0;

	//command line mode
	if (commandLine)
	{
		//command line processing (no GUI)
		result = ccCommandLineParser::Parse(argc, argv, pluginManager.pluginList());
	}
	else
	{
		//main window init.
		MainWindow* mainWindow = MainWindow::TheInstance();
		if (!mainWindow)
		{
			//CS::Widgets::FramelessMessageBox::critical(nullptr, "Error", "Failed to initialize the main application window?!");
			return EXIT_FAILURE;
		}

		mainWindow->initPlugins();
		mainWindow->show();
		QApplication::processEvents();
		//show current Global Shift parameters in Console
		{
			ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
				.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
				.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));
		}

		if (argc > lastArgumentIndex)
		{
			if (splash)
			{
				splash->close();
			}

			//any additional argument is assumed to be a filename --> we try to load it/them
			QStringList filenames;
			for (int i = lastArgumentIndex; i < argc; ++i)
			{
				QString arg = QString::fromLocal8Bit(argv[i]);
				//special command: auto start a plugin
				if (arg.startsWith(":start-plugin:"))
				{
					QString pluginName = arg.mid(14);
					QString pluginNameUpper = pluginName.toUpper();
					//look for this plugin
					bool found = false;
					for (ccPluginInterface* plugin : pluginManager.pluginList())
					{
						if (plugin->getName().replace(' ', '_').toUpper() == pluginNameUpper)
						{
							found = true;
							bool success = plugin->start();
							if (!success)
							{
								ccLog::Error(QString("Failed to start the plugin '%1'").arg(plugin->getName()));
							}
							break;
						}
					}

					if (!found)
					{
						ccLog::Error(QString("Couldn't find the plugin '%1'").arg(pluginName.replace('_', ' ')));
					}
				}
				else
				{
					//filenames << QString::fromLocal8Bit(argv[i]);
					filenames << namelist[i];
				}
			}

			mainWindow->addToDB(filenames);
		}
		else if (splash)
		{
			//count-down to hide the timer (only effective once the application will have actually started!)
			QObject::connect(&splashTimer, &QTimer::timeout, [&]() { if (splash) splash->close(); QCoreApplication::processEvents(); splash.reset(); });
			splashTimer.setInterval(1000);
			splashTimer.start();
		}

		//change the default path to the application one (do this AFTER processing the command line)
		QDir  workingDir = QCoreApplication::applicationDirPath();
		
	#ifdef Q_OS_MAC
		// This makes sure that our "working directory" is not within the application bundle	
		if ( workingDir.dirName() == "MacOS" )
		{
			workingDir.cdUp();
			workingDir.cdUp();
			workingDir.cdUp();
		}
	#endif
        qDebug() << "close end";
		QDir::setCurrent(workingDir.absolutePath());




        result = app.exec();


		//release the plugins
		for (ccPluginInterface* plugin : pluginManager.pluginList())
		{
			plugin->stop(); //just in case
		}
	}

    qDebug() << "app close!";
	//release global structures
	MainWindow::DestroyInstance();
	FileIOFilter::UnregisterAll();

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	//for debug purposes
	unsigned alive = CCShareable::GetAliveCount();
	if (alive > 1)
	{
		printf("Error: some shared objects (%u) have not been released on program end!",alive);
		system("PAUSE");
	}
#endif
    qDebug() << "app close release app";
	releaseApplicationLog();// 关闭日志模块
	return result;
}
