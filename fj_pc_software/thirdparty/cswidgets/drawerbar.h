#pragma once
#include "cswidgets_global.h"
#include <QFrame>
namespace CS {
	namespace Widgets {
		/*
		*@brief 面板
		*/
		class DrawerPanelPrivate;
		class CSWIDGETS_EXPORT DrawerPanel : public QFrame
		{
		public:
			explicit DrawerPanel(QWidget* parent = nullptr);
			virtual ~DrawerPanel();
		public:
			/*
			*@brief 设置面板标题
			*/
			void setPanelCaption(const QString& strCaption);

			QString getPanelCaption(void);
			/**
			*@brief 增加子控件
			*/
			void appendPanelWidget(QWidget* pWidget);
			/**
			*@brief 设置面板内容显示
			*/
			void setPanelContentVisible(bool bVisible);
		private:
			friend class DrawerPanelPrivate;
			CS::Widgets::DrawerPanelPrivate* m_dptr = nullptr;
		};
		/**
		*@brief 抽屉控件
		*/
		class DrawerBarPrivate;
		class CSWIDGETS_EXPORT DrawerBar : public QFrame
		{
		public:
			explicit DrawerBar(QWidget* parent = nullptr);
			virtual ~DrawerBar();
		public:
			void appendDrawerPanel(std::vector<DrawerPanel*> panels);
		private:
			friend class DrawerBarPrivate;
			CS::Widgets::DrawerBarPrivate* m_dptr = nullptr;

		};
	}
}