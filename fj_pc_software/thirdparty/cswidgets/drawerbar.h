#pragma once
#include "cswidgets_global.h"
#include <QFrame>
namespace CS {
	namespace Widgets {
		/*
		*@brief ���
		*/
		class DrawerPanelPrivate;
		class CSWIDGETS_EXPORT DrawerPanel : public QFrame
		{
		public:
			explicit DrawerPanel(QWidget* parent = nullptr);
			virtual ~DrawerPanel();
		public:
			/*
			*@brief ����������
			*/
			void setPanelCaption(const QString& strCaption);

			QString getPanelCaption(void);
			/**
			*@brief �����ӿؼ�
			*/
			void appendPanelWidget(QWidget* pWidget);
			/**
			*@brief �������������ʾ
			*/
			void setPanelContentVisible(bool bVisible);
		private:
			friend class DrawerPanelPrivate;
			CS::Widgets::DrawerPanelPrivate* m_dptr = nullptr;
		};
		/**
		*@brief ����ؼ�
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