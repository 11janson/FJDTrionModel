#pragma once
#include "cloudcomparewidgets_global.h"
#include "metahublframelessdialog.h"
namespace CS {
    namespace MetahubWidgets {
        class GalleryPreViewDialogPrivate;
        class  TRIONMETAHUBWIDGETS_EXPORT GalleryPreViewDialog : public CS::MetahubWidgets::MetahubFramelessDialog
        {
            Q_OBJECT
        public:
            explicit GalleryPreViewDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            virtual ~GalleryPreViewDialog();
        public:
            /**
            *@brief 更新Model数据到UI层显示
            */
            void updateUIFromModel(void);
        private:
            friend class CS::MetahubWidgets::GalleryPreViewDialogPrivate;
            CS::MetahubWidgets::GalleryPreViewDialogPrivate *m_dptr = nullptr;
        };
    }
}