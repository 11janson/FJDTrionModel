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
			*@brief ִ����ʾ
			*/
			int Show(void);
			/*
             *@brief ���ñ���
             */
			void setTitle(QString);
			/*
             *@brief ��������
             */
			void setCenterTips(QString);
			/*
			 *@brief ���õ������ͣ�����Alarm���棬Askѯ�ʣ�
			 */
			void setWindowTitleIcon(QString);
			/*
             *@brief ��ȡOK��ť
             */
			QPushButton *getOkButton();
			/*
             *@brief ��ȡCencal��ť
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