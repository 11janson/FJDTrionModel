#pragma once
#include "cswidgets_global.h"
#include <QDoubleSpinBox>
#include <QLineEdit>

namespace CS {
    namespace Widgets {
        class CSWIDGETS_EXPORT EditDoubleSpinBox :public QDoubleSpinBox
        {
            Q_OBJECT
        public:
            explicit EditDoubleSpinBox(QWidget *parent = 0);
            void getSpinBoxChild();
            void setPropertyValue(QString strValue);

        signals:
            void signalSpinBoxClicked(QString &);
        private:
            QLineEdit *m_pLineEdit;
            QString m_strValue = "";

        protected:
              bool eventFilter(QObject *obj, QEvent *e);
        };
    }
}