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
            *@brief 设置超链接文本信息
            */
            void setHyperlinksText(QString linkName);

            void setFileDir(QString strDir);
            /**
            *@brief 设置基础属性
            */
            void setBaseProperty(bool status);
            /**
            *@brief 创建连接
            */
            void createConnect();
        private:
            QString m_strDir = "";
        public slots:
            void slotOpenUrl(QString strUrl);

        };
    }
}