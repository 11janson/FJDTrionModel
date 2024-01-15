#pragma once
#include "cloudcomparewidgets_global.h"
#include <QToolbutton>
#include <QPushButton>
#include <QImage>
#include <QEvent>

namespace CS {
    namespace MetahubWidgets {

        class GalleryButtonPrivate;
		class TRIONMETAHUBWIDGETS_EXPORT GalleryButton : public QPushButton
		{
			Q_OBJECT
        public:
            enum GalleryButtonType
            {
                Add = 0,
                Thumbnail

            };
		public:
			explicit GalleryButton(QWidget *parent = nullptr);
        public:
       
            /**
            *@brief 更新Model数据到UI层
            */
            void updateUIFromModel(void);

            /**
            *@brief 设置图库图片
            */
            void setGallery(const QImage& image);

            /**
            *@brief 获取图库图片
            */
            QImage getGallery(void);

            /**
            *@brief 获取图库类型
            */
            GalleryButtonType getGalleryButtonType(void);

            /**
            *@brief 设置类型
            */
            void setGalleryButtonType(GalleryButtonType enType);


           void retranslateUi();

        protected:
            virtual void enterEvent(QEvent *event);
            virtual void leaveEvent(QEvent *event);
        
       signals:
           /**
           *@brief 删除图库
           */
           void signalDeleteGalleryButton(void);
        private:
            friend class GalleryButtonPrivate;
            CS::MetahubWidgets::GalleryButtonPrivate *m_pDptr = nullptr;
		};
	}
}
