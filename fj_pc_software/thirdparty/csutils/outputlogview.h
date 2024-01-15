#pragma once
#include <QWidget>
#include <QMutex>
#include <QToolBar>
#include <QToolButton>
#include <QTextCharFormat>
#include <QFont>
#include <QMap>
#include "utils_global.h"

    namespace Utils {
        class OutputFormatter;
        class OutputWindow;
        // Log信息显示窗口。
        class CSUTILS_EXPORT OutputLogView : public QWidget
        {
            Q_OBJECT
        public:
            explicit OutputLogView(QWidget *parent = 0);
            ~OutputLogView();
            /**
             * @brief
             * @param msg
             * @param _severity_level
             */
            void submitAppendLog(const QString &msg, int _severity_level = 0);
        protected:
            /**
             * @brief
             */
            virtual void initFormats() = 0;
            /**
             * @brief mixColors
             * @param a
             * @param b
             */
            QColor mixColors(const QColor &a, const QColor &b);
        private:
            void createWidgets();
            void createConnects();

        public slots:
            void onAppendLog(QString pMessage, int _severity_level);
        private slots:
            void slotShowLogContextMenu(const QPoint&pt);
        signals:
            void signalAppendLog(const QString&, int);
        private:
            QMutex          mMutex;
            QToolBar        *m_pLogToolBar=nullptr;
        protected:
            OutputWindow    *m_pLogView=nullptr;
            QMap<int, QTextCharFormat> m_LevelForFormat;
            QFont            m_font;
        };
    }
