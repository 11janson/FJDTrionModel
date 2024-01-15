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
			* @brief    ���õ�ǰ����
			*/
			static void setOverrideLanguage(const QString &locale);

			/**
			*@brief ��ȡ��ǰ���õ�����
			*/
			static QString getOverrideLanguage(void);

			
			/**
			*@brief ��ȡĬ��·��
			*/
			static QString getDefaultPath();




			static QString systemInformation();


			/**
			* @brief    ��״̬������ʾ��ʾ��Ϣ
			*/
			static void showStatusBarMsg(const QString &msg, const QColor &color, int timeout = 0);
			/**
			* @brief    ��״̬����Ĭ����ʾ��Ϣ
			*/
			static void showNormalStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			* @brief    ��״̬���Ͼ�����ʾ��Ϣ
			*/
			static void showWarningStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			* @brief    ��״̬���ϴ�����ʾ��Ϣ
			*/
			static void showErrorStatusBarMsg(const QString &msg, int timeout = 0);
			/**
			*@������Դ�ļ�
			*/
			static QString ICore::findStyleSheet();

			static void saveSettings();

			/**
			*@brief ��ȡ��Դ����
			*/
			static QString  resourceTheme(void);

			/**
			*@brief ��ȡ������ʽ�ļ�
			*/
			static QString themeStyleSheet(void);


			/**
			*@brief ͼ����Դ
			*/
			static QIcon  resourceThemeImage(const QString strName);

            /**
            *@brief ��ȡ��Դ·��
            */
            static QString resourceThemeImagePath(const QString strKey);


		private:
			//static void updateNewItemDialogState();
		};

	}
}