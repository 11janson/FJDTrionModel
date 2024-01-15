#pragma once
#include "cswidgets_global.h"
#include <QToolButton>

QT_BEGIN_NAMESPACE
class QMenu;
QT_END_NAMESPACE
namespace CS {
    namespace Widgets {
        class CSWIDGETS_EXPORT ToolButton : public QToolButton
        {
            Q_OBJECT
        public:
            explicit ToolButton(QWidget *parent = Q_NULLPTR);
            virtual ~ToolButton();

            void setMenu(QMenu *pMenu);

        signals:

            public slots :
                void recover();
        };
    }//namespace Widgets
}//namespace NQ
