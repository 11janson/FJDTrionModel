#pragma once
#include "cswidgets_global.h"
#include <QLabel>

namespace CS {
    namespace Widgets {
        class CSWIDGETS_EXPORT HyperlinksLabel :public QLabel
        {
            Q_OBJECT
        public:
            explicit HyperlinksLabel(QWidget *parent = 0);
            /**
            *@brief ���ó������ı���Ϣ
            */
            void setHyperlinksText(QString linkName);

            void setFileDir(QString strDir);
            /**
            *@brief ���û�������
            */
            void setBaseProperty(bool status);
            /**
            *@brief ��������
            */
            void createConnect();
        private:
            QString m_strDir = "";
        public slots:
            void slotOpenUrl(QString strUrl);

        };
    }
}