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
#pragma once
#include "cloudcompareutils_global.h"

#include <QObject>
#include <QSettings>
#include <QIcon>
#include <functional>


QT_BEGIN_NAMESPACE
class QPrinter;
class QStatusBar;
class QWidget;
template <typename T> class QList;
QT_END_NAMESPACE

namespace CS {
	namespace Core {
		class TRIONMETAHUBUTILS_EXPORT ICore : public QObject
		{
			Q_OBJECT
		private:
			explicit ICore(QObject *parent = Q_NULLPTR);
		public:
			// This should only be used to acccess the signals, so it could
			// theoretically return an QObject *. For source compatibility
			// it returns a ICore.
			static ICore *instance();

			
			static QSettings *settings(QSettings::Scope scope = QSettings::UserScope);
	
	
			static QString userInterfaceLanguage();

			static QString resourcePath();
			static QString resourceSharePath(void);
			static QString userResourcePath();
			static QString systemResourcePath();
			

			static QString versionString();
			static QString buildCompatibilityString();

			
			static QStatusBar *statusBar();
			
			static void raiseWindow(QWidget *widget);

			
			/**
			* @brief    设置当前语言
			*/
			static void setOverrideLanguage(const QString &locale);

			/**
			*@brief 获取当前设置的语言
			*/
			static QString getOverrideLanguage(void);

			
			/**
			*@brief 获取默认路径
			*/
			static QString getDefaultPath();




			static QString systemInformation();


			/**
			* @brief    在状态栏上显示提示信息
			*/
			static void showStatusBarMsg(const QString &msg, const QColor &color, int timeout = 0);
			/**
			* @brief    在状态栏上默认提示信息
			*/
			static void showNormalStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			* @brief    在状态栏上警告提示信息
			*/
			static void showWarningStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			* @brief    在状态栏上错误提示信息
			*/
			static void showErrorStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			*@查找资源文件
			*/
			static QString ICore::findStyleSheet();

			static void saveSettings();

			/**
			*@brief 获取资源主题
			*/
			static QString  resourceTheme(void);

			/**
			*@brief 获取主题样式文件
			*/
			static QString themeStyleSheet(void);


			/**
			*@brief 图标资源
			*/
			static QIcon  resourceThemeImage(const QString strName);

            /**
            *@brief 获取资源路径
            */
            static QString resourceThemeImagePath(const QString strKey);


		private:
			//static void updateNewItemDialogState();
		};

	}
}