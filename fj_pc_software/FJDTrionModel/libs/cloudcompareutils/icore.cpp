/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of Qt Creator.
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 as published by the Free Software
** Foundation with exceptions as appearing in the file LICENSE.GPL3-EXCEPT
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
****************************************************************************/

#include "icore.h"

#include <QSysInfo>
#include <QApplication>
#include <QStandardPaths>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QStatusBar>
#include <QVariant>
#include <QFile>
#include "csutils/qtcassert.h"
#include "csutils/hostosinfo.h"

using namespace CS::Core;


ICore::ICore(QObject *parent /* = Q_NULLPTR */)
	:QObject(parent)
{
}
ICore * CS::Core::ICore::instance()
{
	static ICore instance;
	return &instance;
}

QString ICore::userInterfaceLanguage()
{
	return qApp->property("qtc_locale").toString();
}

QString ICore::resourcePath()
{
	 
	return QString(QCoreApplication::applicationDirPath() + '/');
}

QString ICore::resourceSharePath(void)
{
	QString strName = QString("%1%2%3")
		.arg(resourcePath()).arg(QDir::separator())
		.arg("share");
	return strName;
}

QSettings *ICore::settings(QSettings::Scope scope)
{
	/*if (scope == QSettings::UserScope)
		return PluginManager::settings();
	else
		return PluginManager::globalSettings();*/
	return nullptr;
}

QStatusBar *ICore::statusBar()
{
	return nullptr;
}


QString ICore::userResourcePath()
{
	// Create qtcreator dir if it doesn't yet exist
	const QString configDir = QFileInfo(settings(QSettings::UserScope)->fileName()).path();
	const QString urp = configDir + QLatin1String("/usrdata");

	if (!QFileInfo::exists(urp + QLatin1Char('/'))) {
		QDir dir;
		if (!dir.mkpath(urp))
			qWarning() << "could not create" << configDir;
	}

	return urp;
}

QString ICore::systemResourcePath()
{
	const QString configDir = QFileInfo(settings(QSettings::SystemScope)->fileName()).path();
	const QString urp = configDir + QLatin1String("/config");

	if (!QFileInfo::exists(urp + QLatin1Char('/'))) {
		QDir dir;
		if (!dir.mkpath(urp))
			qWarning() << "could not create" << configDir;
	}

	return urp;

}

static QString compilerString()
{
#if defined(Q_CC_CLANG) // must be before GNU, because clang claims to be GNU too
	QString isAppleString;
#if defined(__apple_build_version__) // Apple clang has other version numbers
	isAppleString = QLatin1String(" (Apple)");
#endif
	return QLatin1String("Clang ") + QString::number(__clang_major__) + QLatin1Char('.')
		+ QString::number(__clang_minor__) + isAppleString;
#elif defined(Q_CC_GNU)
	return QLatin1String("GCC ") + QLatin1String(__VERSION__);
#elif defined(Q_CC_MSVC)
	if (_MSC_VER > 1999)
		return QLatin1String("MSVC <unknown>");
	if (_MSC_VER >= 1910)
		return QLatin1String("MSVC 2017");
	if (_MSC_VER >= 1900)
		return QLatin1String("MSVC 2015");
#endif
	return QLatin1String("<unknown compiler>");
}

QString ICore::versionString()
{
	
	return QString();
}

QString ICore::buildCompatibilityString()
{
	return tr("Based on Qt %1 (%2, %3 bit)").arg(QLatin1String(qVersion()),
		compilerString(),
		QString::number(QSysInfo::WordSize));
}



void ICore::setOverrideLanguage(const QString & locale)
{
	QString strFileName = getDefaultPath() + "/config/language.ini";
	QSettings settings(strFileName, QSettings::IniFormat);
	settings.beginGroup(QStringLiteral("Translation"));
	{
		settings.setValue(QStringLiteral("Language"), locale);
	}
	settings.endGroup();
	settings.sync();
	return;
}




QString ICore::getOverrideLanguage(void)
{
	QString langCode;
	QString strFileName = getDefaultPath() + "/config/language.ini";
	QSettings settings(strFileName, QSettings::IniFormat);

	if (!QFile::exists(strFileName)) {
		//[!].文件不存在
		langCode = "en";
		setOverrideLanguage(langCode);
		
	}
	else {
		//[!].文件存在
		settings.beginGroup(QStringLiteral("Translation"));
		{
			langCode = settings.value(QStringLiteral("Language")).toString();
		}
		settings.endGroup();
	}
	return langCode;
}

QString ICore::systemInformation()
{
	QString result;
	result += versionString() + '\n';
	result += buildCompatibilityString() + '\n';
//#ifdef IDE_REVISION
//	result += QString("From revision %1\n").arg(QString::fromLatin1(Constants::IDE_REVISION_STR).left(10));
//#endif
//#ifdef QTC_SHOW_BUILD_DATE
//	result += QString("Built on %1 %2\n").arg(QLatin1String(__DATE__), QLatin1String(__TIME__));
//#endif

	return result;
}

QString ICore::getDefaultPath()
{
	QString strPath = QDir::cleanPath(QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)));
	QStringList dirNames;
	dirNames = strPath.split('/');
	if (dirNames.count() <= 0)
		dirNames = strPath.split('\\');
	Q_ASSERT(dirNames.count() > 0);
	if (!(QCoreApplication::applicationName().isNull() || QCoreApplication::applicationName().isEmpty()))
	{
		dirNames.removeLast();
	}
	if (!(QCoreApplication::organizationName().isNull() || QCoreApplication::organizationName().isEmpty()))
	{
		dirNames.removeLast();
	}
	strPath = QDir::cleanPath(dirNames.join(QDir::separator()));
	if (!strPath.endsWith(QDir::separator()))
		strPath.append(QDir::separator());
	strPath.append(QLatin1String(IDE_TARGET_NAME));
	strPath = QDir::toNativeSeparators(strPath);
	QDir dir(strPath);
	if (!dir.exists(strPath)) {
		dir.mkpath(strPath);
	}

	return strPath;

}

void ICore::raiseWindow(QWidget *widget)
{
	if (!widget)
		return;
	QWidget *window = widget->window();
	if (window ) {
	
	}
	else {
		window->raise();
		window->activateWindow();
	}
}

void ICore::saveSettings()
{


	ICore::settings(QSettings::SystemScope)->sync();
	ICore::settings(QSettings::UserScope)->sync();
}

QString CS::Core::ICore::resourceTheme(void)
{
	return QString("%1%2%3").arg(resourcePath()).arg(QDir::separator()).arg("theme");
}

QString CS::Core::ICore::themeStyleSheet(void)
{
	return  QString("%1%2%3").arg(resourceTheme())
		.arg(QDir::separator()).arg("themestyle.qss");
}


QIcon CS::Core::ICore::resourceThemeImage(const QString strKey)
{
	QString strName = QString("%1%2%3%4%5")
		.arg(resourceTheme()).arg(QDir::separator())
		.arg("qssimage").arg(QDir::separator()).arg(strKey);

	return QIcon(strName);
}


QString CS::Core::ICore::resourceThemeImagePath(const QString strKey)
{
    QString strName = QString("%1%2%3%4%5")
        .arg(resourceTheme()).arg(QDir::separator())
        .arg("qssimage").arg(QDir::separator()).arg(strKey);
    return strName;
}

void ICore::showStatusBarMsg(const QString &msg,
	const QColor &color,
	int timeout)
{
	QStatusBar *pStatusBar = statusBar();
	if (!pStatusBar)
		return;
	QPalette palette = pStatusBar->palette(); // get current palette
	palette.setColor(QPalette::Foreground, color); // modify palette
	pStatusBar->setPalette(palette); // apply modified palette
	pStatusBar->setAutoFillBackground(true); // tell widget to fill its background itself
	pStatusBar->showMessage(msg, timeout);
}
/**
* ��״̬������ʾĬ����ʾ��Ϣ
*
* @param  const QString & msg Ҫ��ʾ����Ϣ
* @param  int timeout ״̬����Ϣ����ʾʱ��
* @return void
*/
void ICore::showNormalStatusBarMsg(const QString &msg, int timeout /*= 0*/)
{
	showStatusBarMsg(msg, Qt::white, timeout);
}

/**
* ��״̬������ʾ������Ϣ
*
* @param  const QString & msg Ҫ��ʾ����Ϣ
* @param  int timeout ״̬����Ϣ����ʾʱ��
* @return void
*/
void ICore::showWarningStatusBarMsg(const QString &msg, int timeout /*= 0*/)
{
	showStatusBarMsg(msg, Qt::yellow, timeout);
}
/**
* ��״̬������ʾ������Ϣ
*
* @param  const QString & msg Ҫ��ʾ����Ϣ
* @param  int timeout ״̬����Ϣ����ʾʱ��
* @return void
*/
void ICore::showErrorStatusBarMsg(const QString &msg, int timeout /*= 0*/)
{
	showStatusBarMsg(msg, Qt::red, timeout);
}

QString ICore::findStyleSheet()
{
	QString appStyleSheet;
	const QLatin1String extension("*.css");
	return appStyleSheet;
}




