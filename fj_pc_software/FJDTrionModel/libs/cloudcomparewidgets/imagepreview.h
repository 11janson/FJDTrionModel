#pragma once
#include "cloudcomparewidgets_global.h"
#include "metahublframelessdialog.h"
#include <QWidget>
#include <QImage>
#include <vector>
#include <QPushButton>


namespace CS {
    namespace MetahubWidgets {
        class TRIONMETAHUBWIDGETS_EXPORT ImageView : public CS::MetahubWidgets::MetahubFramelessDialog
        {
            Q_OBJECT

        public:
            ImageView(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            ~ImageView();


        public:
            /**
            *@brief 修改
            */
            void loadImage(const QImage &image);

            /**
            *@brief 设置图像列表
            */
            void setImageList(const std::vector<QImage> listImage);

            /**
            *@brief 设置当前激活的id
            */
            void setActivateIndex(const int &index);

            /**
            *@brief 更新数据到UI层
            */
            void updateUIFromModel(void);

        public slots:
            void slotNextButton(void);
            void slotPreviousButton(void);

        protected:
            void paintEvent(QPaintEvent *event) override;
            void wheelEvent(QWheelEvent *event) override;
            void mousePressEvent(QMouseEvent *event) override;
            void mouseMoveEvent(QMouseEvent *event) override;
            void mouseReleaseEvent(QMouseEvent *event) override;

        private:
            QImage m_Image;
            qreal m_ZoomValue = 1.0;
            int m_XPtInterval = 0;
            int m_YPtInterval = 0;
            QPoint m_OldPos;
            bool m_Pressed = false;

        private:
            int m_nAcitveIndex = 0;
            std::vector<QImage> m_listImageList;

        private:
            QPushButton *m_pNextButton = nullptr;
            QPushButton *m_pPreviousButton = nullptr;
        private slots:
            void onLoadImage(void);
            void onZoomInImage(void);
            void onZoomOutImage(void);
            void onPresetImage(void);
        };

    }
}



