#pragma once
#include <QWidget>
#include <QDialog>
#include "ccHObject.h"
#include "ccPickingListener.h"
#include "cloudcomparewidgets/metahublframelessdialog.h"
namespace CS {
	namespace MetahubWidgets {
		class CreatePointCloudOrthoImagePrivate;
		class CreatePointCloudOrthoImageDialog : public CS::MetahubWidgets::MetahubFramelessDialog , public ccPickingListener
		{
			Q_OBJECT
		public:
			explicit CreatePointCloudOrthoImageDialog(QWidget *parent  = nullptr);
			virtual ~CreatePointCloudOrthoImageDialog(void);

		private:
			//[!].拾取
			void onItemPicked(const PickedItem& pi) override;
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
			void setCurrentActiveEntity(ccHObject *pHobject);
			/**
			*@brief 更新数据到UI层
			*/
			void updateUIFromModel(void);
		private:
			friend class CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate;
			CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate	*m_dqtr = nullptr;

		};
	}
}




