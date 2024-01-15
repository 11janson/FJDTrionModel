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
            *@brief ����Model���ݵ�UI��
            */
            void updateUIFromModel(void);

            /**
            *@brief ����ͼ��ͼƬ
            */
            void setGallery(const QImage& image);

            /**
            *@brief ��ȡͼ��ͼƬ
            */
            QImage getGallery(void);

            /**
            *@brief ��ȡͼ������
            */
            GalleryButtonType getGalleryButtonType(void);

            /**
            *@brief ��������
            */
            void setGalleryButtonType(GalleryButtonType enType);


           void retranslateUi();

        protected:
            virtual void enterEvent(QEvent *event);
            virtual void leaveEvent(QEvent *event);
        
       signals:
           /**
           *@brief ɾ��ͼ��
           */
           void signalDeleteGalleryButton(void);
        private:
            friend class GalleryButtonPrivate;
            CS::MetahubWidgets::GalleryButtonPrivate *m_pDptr = nullptr;
		};
	}
}
