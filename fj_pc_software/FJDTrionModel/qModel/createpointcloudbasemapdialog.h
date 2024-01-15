#pragma once
#include <QWidget>
#include <QDialog>
#include "ccHObject.h"
#include "cloudcomparewidgets/metahublframelessdialog.h"
namespace CS {
	namespace MetahubWidgets {
		class CreatePointCloudBasemapPrivate;
		class CreatePointCloudBasemapDialog : public CS::MetahubWidgets::MetahubFramelessDialog
		{
			Q_OBJECT
		public:
			explicit CreatePointCloudBasemapDialog(QWidget* parent = nullptr);
			virtual ~CreatePointCloudBasemapDialog(void);

		public slots:
			/**
			*@brief ȷ�ϰ�ť��
			*/
			virtual void slotUIButtonOk(void);

			/**
			*@brief ȡ����ť
			*/
			virtual void slotUIButtonCancel(void);

			/**
			*@brief Ӧ�ð�ť
			*/
			virtual void slotUIButtonApply(void);

		public:
			/**
			*@brief ���õ�ǰ�����ʵ��
			*/
			void setCurrentActiveEntity(ccHObject* pHobject);
			/**
			*@brief �������ݵ�UI��
			*/
			void updateUIFromModel(void);
		private:
			friend class CS::MetahubWidgets::CreatePointCloudBasemapPrivate;
			CS::MetahubWidgets::CreatePointCloudBasemapPrivate* m_dqtr = nullptr;
		};
	}
}