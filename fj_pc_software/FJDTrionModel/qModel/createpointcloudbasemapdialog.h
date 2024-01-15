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

		public:
			/**
			*@brief 设置当前激活的实体
			*/
			void setCurrentActiveEntity(ccHObject* pHobject);
			/**
			*@brief 更新数据到UI层
			*/
			void updateUIFromModel(void);
		private:
			friend class CS::MetahubWidgets::CreatePointCloudBasemapPrivate;
			CS::MetahubWidgets::CreatePointCloudBasemapPrivate* m_dqtr = nullptr;
		};
	}
}