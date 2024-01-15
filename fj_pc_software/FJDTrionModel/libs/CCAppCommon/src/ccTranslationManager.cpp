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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

// Qt
#include <QDebug>
#include <QDir>
#include <QGlobalStatic>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSettings>
#include "cloudcompareutils/icore.h"
#include <QTranslator>

// ccPluginAPI
#include <ccPersistentSettings.h>

// Local
#include "ccApplicationBase.h"
#include "ccTranslationManager.h"
#include <framelessmessagebox.h>
#include "cloudcompareutils/icore.h"
class _ccTranslationManager : public ccTranslationManager {};	// trick for Q_GLOBAL_STATIC to access constructor
Q_GLOBAL_STATIC(_ccTranslationManager, s_translationmanager)

ccTranslationManager& ccTranslationManager::Get()
{
	return *s_translationmanager;
}

void ccTranslationManager::registerTranslatorFile(const QString &prefix, const QString &path)
{
	mTranslatorFileInfo.append({ prefix, path });
}

void ccTranslationManager::setTranslationPath(const QString strPath)
{
	m_translationPath = strPath;
	return;
}

void ccTranslationManager::loadTranslation(QString language)
{
	const QLocale locale(language);

	//[!].获取翻译目录下的所有翻译文件
	QDir dir(m_translationPath);
	QStringList filter;
	filter << QString("*_%1.qm").arg(language);
	filter << QString("*_%1_CN.qm").arg(language);
	dir.setNameFilters(filter);
	QList<QFileInfo> fileInfo = QList<QFileInfo>(dir.entryInfoList(filter, QDir::AllEntries));
	for (auto pqm : fileInfo) {
		QString strQm = pqm.absoluteFilePath();

		qDebug() << strQm;
		QTranslator *translator = new QTranslator();
		bool loaded = translator->load(strQm);
		if (loaded){
			ccApp->installTranslator(translator);
		}
		else{
			delete translator;
			translator = nullptr;
		}
	}
}

void ccTranslationManager::populateMenu(QMenu* menu, const QString& pathToTranslationFiles)
{
	const LanguageList languageList = availableLanguages(QStringLiteral("FJDTrionModel"), pathToTranslationFiles);

	QActionGroup* group = new QActionGroup(menu);
	group->setExclusive(true);

	

	//QAction* separator = new QAction(group);
	//separator->setSeparator(true);

	const QString currentLanguage = languagePref();

	for (const auto& langInfo : languageList)
	{
        QAction*action = group->addAction(langInfo.second);

		action->setCheckable(true);

		if (currentLanguage == langInfo.first)
		{
			action->setChecked(true);
			m_currentAction = action;
		}

		connect(action, &QAction::triggered, this, [=]()
			{
				setLanguagePref(langInfo.first, action);
			});
        if (langInfo.first == "zh_TW")
        {
            QAction* action = group->addAction(QCoreApplication::translate("MainWindow", "English", nullptr));
            action->setCheckable(true);
            if (currentLanguage == "en")
            {
                action->setChecked(true);
                m_currentAction = action;
            }
            connect(action, &QAction::triggered, this, [=]()
            {
                setLanguagePref(QStringLiteral("en"), action);
            });
        }

	}

	menu->addActions(group->actions());
	menu->setIcon(QIcon(QCoreApplication::applicationDirPath()+"/theme/qssimage/translatebutton.png"));
}

QString ccTranslationManager::languagePref() const
{
	return CS::Core::ICore::getOverrideLanguage();
}

ccTranslationManager::LanguageList ccTranslationManager::availableLanguages(const QString& appName, const QString& pathToTranslationFiles) const
{
	QDir dir(pathToTranslationFiles);

	const QString     filter = QStringLiteral("%1_*.qm").arg(appName);
	const QStringList fileNames = dir.entryList({ filter });

	// e.g. File name is "CloudCompare_es_AR.qm"
	//	Regexp grabs "es_AR" in the var "localeStr" (used to set our locale using QLocale)
	//	and if there is a country code (e.g. "AR"), capture that in "countryCode" (used for menu item)
	QRegularExpression regExp(QStringLiteral("%1_(?<localeStr>.{2}(_(?<countryCode>.{2}))?).*.qm").arg(appName));
	regExp.optimize();

	LanguageList languageList;
	for (const auto& fileName : fileNames)
	{
		QRegularExpressionMatch match = regExp.match(fileName);
		if (!match.hasMatch())
		{
			continue;
		}

		// Determine our locale
		const QString localeStr(match.captured(QStringLiteral("localeStr")));
		const QLocale locale(localeStr);

		// Grab our Langauge
		const QString language(locale.nativeLanguageName());

		if (language.isEmpty())
		{
			qWarning() << "Language not found for translation file" << fileName;
			continue;
		}

		// Uppercase first letter of language
		QString menuItem = QStringLiteral("%1%2").arg(language[0].toUpper(), language.mid(1));

		// Add country if it was part of our locale (e.g. "es_AR" -> "Argentina")
		const QString countryCode(match.captured(QStringLiteral("countryCode")));

		if (!countryCode.isEmpty())
		{
			menuItem += QStringLiteral(" (%1)").arg(locale.nativeCountryName());
		}
        if (menuItem.contains(L"(台灣)"))
        {
            menuItem = u8"繁體中文";
        }
        if (menuItem.contains(u8"Español de España"))
        {
            menuItem = u8"Español";
        }
		languageList += { localeStr, menuItem };
	}
    //[!]对语言进行排序
    bool language_zh = false;
    bool language_zh_TW = false;
    bool language_en= false;
    bool language_ja = false;
	bool language_ru = false;
    bool language_es = false;
    bool language_it = false;

    LanguageList languageListSort;
    foreach(const TranslationInfo &info, languageList) {
        QString language = info.first;
        QString translation = info.second;
        if (language == "zh")
        {
            language_zh = true;
        }
        if (language == "zh_TW")
        {
            language_zh_TW = true;
        }
        if (language == "ja")
        {
            language_ja = true;
        }
   //     if (language == "ru")
   //     {
			//language_ru = true;
   //     }
        if (language == "es")
        {
            language_es = true;
        }
        //if (language == "it")
        //{
        //    language_it = true;
        //}

    }
    if (language_zh)
    {
        foreach(const TranslationInfo &info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "zh")
            {
                languageListSort.append(info);
            }
        }
    }
    if (language_zh_TW)
    {
        foreach(const TranslationInfo &info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "zh_TW")
            {
                languageListSort.append(info);
            }
        }
    }

    if (language_ja)
    {
        foreach(const TranslationInfo &info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "ja")
            {
                languageListSort.append(info);
            }
        }
    }
    if (language_ru)
    {
        foreach(const TranslationInfo & info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "ru")
            {
                languageListSort.append(info);
            }
        }
    }
    if (language_es)
    {
        foreach(const TranslationInfo & info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "es")
            {
                languageListSort.append(info);
            }
        }
    }

    if (language_it)
    {
        foreach(const TranslationInfo & info, languageList) {
            QString language = info.first;
            QString translation = info.second;
            if (language == "it")
            {
                languageListSort.append(info);
            }
        }
    }

	
	return languageListSort;
}

void ccTranslationManager::setLanguagePref(const QString &languageCode, QAction *sender)
{
	if (languageCode == languagePref())
	{
		return;
	}
	//CS::Widgets::FramelessMessageBox::information(ccApp->activeWindow(),
	//	tr("Language Change"),
	//	tr("Language change will take effect when Trion Metahub is restarted"));
	QMessageBox::StandardButton result = CS::Widgets::FramelessMessageBox::question(ccApp->activeWindow(),
		tr("Language Change"),
		tr("The language will be changed when the software restarts. Are you sure you want to continue?"),
		QMessageBox::Ok | QMessageBox::Cancel,
		QMessageBox::Ok);

	//continue process?
	if (result == QMessageBox::Ok)
	{
		CS::Core::ICore::setOverrideLanguage(languageCode);
		m_currentAction = sender;
	}
	else
	{
		m_currentAction->setChecked(true);
	}
	return;
}
