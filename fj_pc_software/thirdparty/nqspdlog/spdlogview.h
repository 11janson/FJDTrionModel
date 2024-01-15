#pragma once
#include <QWidget>
#include <QMutex>
#include <QToolBar>
#include <QToolButton>
#include <QTextCharFormat>
#include <QFont>
#include <QMap>
#include "nqspdlog_global.h"
#include "csutils/outputlogview.h"

    namespace Utils {
        class OutputFormatter;
        class OutputWindow;
    }

namespace spdlog{
        // Log信息显示窗口。
        class NQSPDLOG_EXPORT SpdLogView : public Utils::OutputLogView
        {
            Q_OBJECT
        public:
            explicit SpdLogView(QWidget *parent = 0);
            ~SpdLogView();
        protected:
            virtual void initFormats() Q_DECL_OVERRIDE;
        };
}// namespace spdlog
