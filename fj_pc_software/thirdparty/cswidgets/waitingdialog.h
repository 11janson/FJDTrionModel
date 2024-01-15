#ifndef _WAITINGDIALOG_H_
#define _WAITINGDIALOG_H_

#include "cswidgets_global.h"
#include <QTimer>
#include <QDialog>
#include <QCloseEvent>
#include <atomic>
#include <QPushButton>
#include <QFrame>
#include <QMovie>
QT_BEGIN_NAMESPACE
class QLabel;
QT_END_NAMESPACE
namespace CS {
    namespace Widgets {
		class WaitingDialogPrivate;
        class CSWIDGETS_EXPORT WaitingDialog : public QDialog
        {
            Q_OBJECT
        public:
            enum IndicatorSize {
                Small,
                Medium,
                Large
            };
            enum TooltipLevel {
                Normal,
                Warning,
                Error
            };
            explicit WaitingDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            ~WaitingDialog();

            //private:
            static WaitingDialog* instance(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            static void deleteInstance();

        public:
            void setIndicatorSize(IndicatorSize size);
            IndicatorSize indicatorSize() const;

            QSize sizeHint() const;

            void attachToWidget(QWidget *parent);

            void showWaitingDialog(const QString &text, TooltipLevel level = TooltipLevel::Normal);
            void hideWaitingDialog();

            /**
            * @brief 返回提示消息
            */
            QString informativeText() const;
            /**
            * @brief 设置提示消息
            */
            void setInformativeText(const QString &text, TooltipLevel level = TooltipLevel::Normal);
            /**
            * @brief 设置按钮上的文本信息
            */
            void setButtonText(const QString &text);
            void setParentChanged(bool bChanged);
           

            virtual void changeEvent(QEvent *);
        signals:
          
            public Q_SLOTS:
            void hide();

        protected:
            void paintEvent(QPaintEvent *) Q_DECL_OVERRIDE;
            void showEvent(QShowEvent *) Q_DECL_OVERRIDE;
            void hideEvent(QHideEvent *) Q_DECL_OVERRIDE;
            void closeEvent(QCloseEvent *) Q_DECL_OVERRIDE;
            bool eventFilter(QObject *obj, QEvent *ev) Q_DECL_OVERRIDE;
            virtual void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
            virtual void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

        private:
            void step();
            void resizeToParent();
            void moveToCenter();

        private:
            static WaitingDialog*        m_pInstance;

            WaitingDialog::IndicatorSize m_size;
            int                          m_rotationStep;
            int                          m_rotation;
            QTimer                       m_timer;
            QPixmap                      m_pixmap;
            QLabel                      *m_pInformativeLabel;///< 消息提示 
			QLabel						*m_pWaitMoveLabel;
            bool                         m_bIsSetParent = false;     ///< 是否把父窗口改为了非mainwindow
            bool                         m_bAltKeyPressed = false;
			QMovie						*m_pMovie;
        public:
            static QColor               DEFAULT_BORDER_COLOR;       ///< 默认边框颜色
            static QColor               DEFAULT_BACKGROUND;         ///< 默认背景色
            static QColor               DEFAULT_SHADOW_COLOR;       ///< 阴影区域颜色
        };

    } // namespace Widgets
} // namespace Apollo

#endif