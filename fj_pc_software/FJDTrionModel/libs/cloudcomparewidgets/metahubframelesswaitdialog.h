#pragma once 

#include <QTimer>
#include <QDialog>
#include <QCloseEvent>
#include <atomic>
#include <QPushButton>
#include <QFrame>
#include <QMovie>
#include "cloudcomparewidgets_global.h"

namespace WaitingDialog {
	class MetahublFramelessWaitingdialogPrivate;
	class  TRIONMETAHUBWIDGETS_EXPORT MetahublFramelessWaitingdialog : public QDialog
	{
		Q_OBJECT
	public:
		explicit MetahublFramelessWaitingdialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
		virtual ~MetahublFramelessWaitingdialog();
	public:
		/**
		*@ 获取等待框实例
		*/
		static MetahublFramelessWaitingdialog* instance(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
        /**
        *@brief 启动等待动画
        */
        void startWaiting(const QString&);
		/**
		*@brief 关闭等待动画
		*/
		void stopWaiting();

		//[!]在运行过程中修改提示
		void setWaitdialogDisplay(QString);

        /**
        *调到下帧
        */
        void jumpToNextFrame(void);
	protected:
		void showEvent(QShowEvent *);
		void keyPressEvent(QKeyEvent *event);
	private:
		int m_pImageRotate;
	private:
		friend class WaitingDialog::MetahublFramelessWaitingdialogPrivate;
		WaitingDialog::MetahublFramelessWaitingdialogPrivate *m_dptr = nullptr;
	private:
		int m_width = 0;
		int m_height = 0;
		bool m_showdpisize = true;
	};

}