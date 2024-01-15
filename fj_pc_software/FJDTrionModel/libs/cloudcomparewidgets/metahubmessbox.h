#pragma once
#include <QWidget>
#include "cloudcomparewidgets_global.h"
#include "metahublframelessdialog.h"
#include <QMessageBox>

namespace CS {
	namespace MetahubWidgets {
		class MetahubMessboxPrivate;
		class TRIONMETAHUBWIDGETS_EXPORT MetahubMessbox : public CS::MetahubWidgets::MetahubFramelessDialog
		{
			Q_OBJECT
		public:
			explicit MetahubMessbox(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
			         MetahubMessbox(const QString &title,
				                    const QString &text,
			                     	QWidget *parent = Q_NULLPTR,
			                     	Qt::WindowFlags flags = Qt::Dialog | Qt::MSWindowsFixedSizeDialogHint);
			virtual ~MetahubMessbox();

		public:
			/*
			*@brief 执行显示
			*/
			int Show(void);
			/*
             *@brief 设置标题
             */
			void setTitle(QString);
			/*
             *@brief 设置内容
             */
			void setCenterTips(QString);
			/*
			 *@brief 设置弹窗类型（参数Alarm警告，Ask询问）
			 */
			void setWindowTitleIcon(QString);
			/*
             *@brief 获取OK按钮
             */
			QPushButton *getOkButton();
			/*
             *@brief 获取Cencal按钮
             */
			QPushButton *getCancelButton();
		public:
			static int information(QWidget *parent,
				const QString &title,
				const QString &text);
			static int warning(QWidget *parent,
				const QString &title,
				const QString &text);
		private:
			friend class CS::MetahubWidgets::MetahubMessboxPrivate;
			CS::MetahubWidgets::MetahubMessboxPrivate *m_dptr;
		};


	}
}