#pragma once
#include <QDialog>
#include <QEvent>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLayout>
#include <QWidget>
#include <QShowEvent>
#include "cloudcomparewidgets_global.h"
#include "cswidgets/titlebar.h"
#include "cswidgets/framelesshelper.h"
namespace CS {
	namespace MetahubWidgets {
		class MetahubFramelessDialogPrivate;
		class TRIONMETAHUBWIDGETS_EXPORT MetahubFramelessDialog : public QDialog
		{
			Q_OBJECT
		public:
			explicit MetahubFramelessDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
			virtual ~MetahubFramelessDialog();

		public:
			/**
			*@brief 设置布局
			*/
			void setLayout(QLayout *layout);
			/**
			*@brief 获取布局
			*/
			QLayout *layout() const;

			/**
			*@brief 获取底部区域
			*/
			QWidget* bottomWidget(void);

			/**
			*@brief 中心区域
			*/
			QWidget *getContentHolder(void);

			// 底部的布局区
			QHBoxLayout *bottomLayout();

			/**
			*@brief 设置标题
			*/

			void setWindowTitle(const QString &title);
			/**
			*@brief 打开并获取最大化按钮
			*/
			QToolButton *getWindowTitleMaxButton();
			/**
             *@brief 获取顶部布局
             */
			QLayout  *getTilteBarLayout(void);
			/**
			*@brief 获取中央区域布局
			*/
			QLayout *getContentHolderLayout(void);
			/**
			*@brief 获取中部区域布局
			*/
			QHBoxLayout * getCenterHoderLayout(void);

			/**
			*@brief 获取底部确认按钮
			*/
			QPushButton *getOKButton(void);

			/**
			*@brief 获取取按钮
			*/
			QPushButton *getCancelButton(void);

            /**
            *@brief 获取应用按钮
            */
            QPushButton *getApplyButton(void);

			/**
			*@brief 获取主布局
			*/
			QLayout *mainLayout();

			/**
			*@brief 获取顶部状态栏
			*/
			CS::Widgets::TitleBar *getTitleBar(void);
            /**
            *@brief 获取FramelessHelper
            */
            CS::Widgets::FramelessHelper* getFramelessHelper(void);

            /**
            *@brief 中心区域主布局页面
            */
            QFrame *getCenterMainFrame(void);
        protected:
            virtual void resizeEvent(QResizeEvent *e)override;
		public slots:
			/**
			*@brief 确认按钮槽
			*/
			virtual void slotUIButtonOk(void);

			/**
			*@brief 取消按钮
			*/
			virtual void slotUIButtonCancel(void);

			/**
			*@brief 应用按钮
			*/
			virtual void slotUIButtonApply(void);
		protected:
			void showEvent(QShowEvent *event);
            virtual void keyPressEvent(QKeyEvent *event);
		private:
            QVBoxLayout *m_pContentHoderLayout = nullptr;
			friend class CS::MetahubWidgets::MetahubFramelessDialogPrivate;
			CS::MetahubWidgets::MetahubFramelessDialogPrivate *m_pDptr = nullptr;

		};


	}
}

