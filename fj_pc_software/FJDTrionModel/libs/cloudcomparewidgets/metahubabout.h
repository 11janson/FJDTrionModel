#pragma once 

#include <QDialog>
#include <QWidget>
#include <QPushButton>
#include <QToolButton>
#include <QVariant>
#include <QDesktopWidget>
#include <QApplication>
#include <QLabel>
#include <QFile>
#include <QTextEdit>
#include "cloudcomparewidgets_global.h"
#include "metahublframelessdialog.h"
namespace CS {
	namespace MetahubWidgets {
		class MetahubAboutdialogPrivate;
		class  TRIONMETAHUBWIDGETS_EXPORT MetahubAboutdialog : public CS::MetahubWidgets::MetahubFramelessDialog
		{
			Q_OBJECT
		public:
			explicit MetahubAboutdialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
			virtual ~MetahubAboutdialog();
		public:
			/**
			*@brief 警告提示信息
			*/
			int critical(QWidget *parent, const QString &title,
				const QString &text);

			/**
		     *@brief 
		     */
			int critical(const QString &text);
		private:
			QPushButton		*m_pContentIconButton = nullptr;
			QTextEdit		*m_pTextEdit = nullptr;
		private:
			friend class CS::MetahubWidgets::MetahubAboutdialogPrivate;
			CS::MetahubWidgets::MetahubAboutdialogPrivate *m_dptr = nullptr;

		};
	}
}
