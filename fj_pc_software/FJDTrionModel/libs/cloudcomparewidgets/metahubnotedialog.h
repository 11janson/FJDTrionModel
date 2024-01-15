#pragma once 
#include <QWidget>
#include "cloudcomparewidgets_global.h"
#include "metahublframelessdialog.h"


namespace CS {
	namespace MetahubWidgets {
        class MetahubNoteDialogPrivate;
        class  TRIONMETAHUBWIDGETS_EXPORT MetahubNoteDialog : public CS::MetahubWidgets::MetahubFramelessDialog
        {
            Q_OBJECT
        public:
            explicit MetahubNoteDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            virtual ~MetahubNoteDialog();
        public:
            /**
            *@brief 更新Model数据到UI层显示
            */
            void updateUIFromModel(void);

        public slots:
            /**
            *@brief 确认按钮槽
            */
            virtual void slotUIButtonOk(void);

        private:
            friend class CS::MetahubWidgets::MetahubNoteDialogPrivate;
            CS::MetahubWidgets::MetahubNoteDialogPrivate *m_dptr = nullptr;
        };
	}
}