#pragma once
#include "utils_global.h"
#include "outputformat.h"
#include <QPlainTextEdit>
    namespace Utils {
        class OutputFormatter;
        class CSUTILS_EXPORT OutputWindow : public QPlainTextEdit
        {
            Q_OBJECT

        public:
            OutputWindow(QWidget *parent = 0);
            ~OutputWindow();

            Utils::OutputFormatter* formatter() const;
            void setFormatter(Utils::OutputFormatter *formatter);

            void appendMessage(const QString &out, Utils::OutputFormat format);
            /// appends a \p text using \p format without using formater
            void appendText(const QString &text, const QTextCharFormat &format = QTextCharFormat());

            void grayOutOldContent();


            void showEvent(QShowEvent *);

            void scrollToBottom();

            void setMaxLineCount(int count);
            int maxLineCount() const { return m_maxLineCount; }
            public slots:
            void setWordWrapEnabled(bool wrap);
            void clear();
        protected:
            bool isScrollbarAtBottom() const;

            virtual void mousePressEvent(QMouseEvent *e);
            virtual void mouseReleaseEvent(QMouseEvent *e);
            virtual void mouseMoveEvent(QMouseEvent *e);
            virtual void resizeEvent(QResizeEvent *e);
            virtual void keyPressEvent(QKeyEvent *ev);

        private:
            void enableUndoRedo();
            QString doNewlineEnfocement(const QString &out);
            Utils::OutputFormatter *m_formatter=nullptr;

            bool m_enforceNewline;
            bool m_scrollToBottom;
            bool m_linksActive;
            bool m_mousePressed;
            int m_maxLineCount;
        public:
            QAction *cutAction;
            QAction *copyAction;
            QAction *pasteAction;
            QAction *selectAllAction;
            QAction *clearAction;
        };
    }

