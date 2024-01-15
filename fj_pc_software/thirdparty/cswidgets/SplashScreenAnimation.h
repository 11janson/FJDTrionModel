#pragma once
#include "cswidgets_global.h"
#include <QSplashScreen>
#include <QPixmap>
#include <QTimer>


namespace CS {
	namespace Widgets {
		class SplashScreenAnimationPrivate;
		class CSWIDGETS_EXPORT SplashScreenAnimation : public QSplashScreen
		{
			Q_OBJECT
		private:
			explicit SplashScreenAnimation(QPixmap pixmap);
		public:
			virtual ~SplashScreenAnimation();
		public:
			/**
			*@brief 获取实例
			*/
			static SplashScreenAnimation* instance(void);
			/**
			*@brief 设置背景图片
			*/
			void setBackgroundPixmap(QPixmap pixmap);
			/**
			*@brief 设置背景图片
			*/
			void setBackgroundPixmap(const QString &strPath);
			/**
			*@brief 显示消息信息
			*/
			void showMessageInformation(const QString &strMessage);

			

		protected:
			virtual void showEvent(QShowEvent *event) Q_DECL_OVERRIDE;
			virtual void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
		private:
			static SplashScreenAnimation *m_instance;
		private:
			QPixmap m_Pixmap;
			QString m_strInfo;
			QString m_strDisplay;
		private:
			QTimer	m_timer;
			int m_nCount = 0;

		private:
			friend class  CS::Widgets::SplashScreenAnimationPrivate;
			CS::Widgets::SplashScreenAnimationPrivate *m_pDptr = nullptr;
		};
	}
}


