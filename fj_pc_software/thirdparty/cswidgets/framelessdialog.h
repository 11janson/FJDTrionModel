#pragma once
#include "cswidgets_global.h"
#include "titlebar.h"
#include "FJBaseWidget.h"
#include <QDialog>
#include <QShowEvent>
QT_BEGIN_NAMESPACE
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QAbstractButton;
QT_END_NAMESPACE
namespace CS {
    namespace Widgets {
        class FramelessHelper;
        class CSWIDGETS_EXPORT FramelessDialog : public FJBaseWidget
        {
            Q_OBJECT
        public:
            explicit FramelessDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
            virtual ~FramelessDialog();

			virtual void InitFJStyle();

			//隐藏标题图标
			void hideTitleIcon();
            // 设置按钮图标
            void setTitleButtonIcon(const int &buttonType, const QIcon &icon);
            /**
            * @brief 设置窗口是否可以移动
            */
            void setDialogtMovable(bool b);
            /**
            * @brief 设置窗口是否可以移动
            */
            void setDialogMovable(bool b);
            /**
            * @brief 设置是否支持橡皮筋风格的移动效果
            */
            void setRubberBandOnMove(bool b);
            /**
            * @brief 设置窗口是否可以缩放
            */
            void setDialogResizable(bool b);
            /**
            * @brief 设置是否支持橡皮筋风格的缩放效果
            */
            void setRubberBandOnResize(bool b);
            // 设置标题文本的对齐方式
            void setWindowTitleAlignment(Qt::Alignment align = Qt::AlignCenter);
            // 显示最小化
            void setMinButtonVisible(bool b);
            // 显示最大化按钮
            void setMaxButtonVisible(bool b);
            // 显示关闭按钮
            void setCloseButtonVisible(bool b);
            bool getCloseButtonVisible()const;
            // 确定按钮是否显示
            void setOKButtonVisible(bool b);
            // 取消按钮是否显示
            void setCancelButtonVisible(bool b);
            // 应用按钮是否显示
            void setApplyButtonVisible(bool b);
             // 设置Ok按钮是否为默认选中项
            void setOKButtonDefault(bool b);
            // 添加一个扩展的按钮
            void appendExtendTitleButton(QAction *pAction,
                QSize size = TitleBarButtonIconSize,
                QSize iconSize = TitleBarButtonIconSize);
            // 插入一个扩展按钮
            void insertExtendTitleButton(int idx,
                QAction *pAction,
                QSize size = TitleBarButtonIconSize,
                QSize iconSize = TitleBarButtonIconSize);
            // 非中心区域的高度
            int noCentralHeight();
            // 底部的布局区
            QHBoxLayout *bottomLayout() { return m_pBottomLayout; }
            // 底部Widget
            QWidget* bottomWidget(){ return m_pBottomWidget;}
            /**
             * @brief 设置下部工具条的隐藏与显示
             * @param b
             */
            void setBottomBarVisible(bool b=true);
            QVBoxLayout* getMainLayout();
			//设置分割线宽度
			void setHorizontalLineHeight(int iHeight);

			//设置Messagebox特殊类型
			void setMessageboxStyleSheet();

			void SetContentHolder(QWidget * widget);

			QPushButton * GetOKButton();

			QPushButton * GetCancelButton();

			QPushButton * GetApplyButton();

            void setOkButton(QPushButton*);

            TitleBar *getTitleBar();

            void setTitleBarLabelStyleSheet(QString);

            void setTtitleBarButtonStyle(const int &buttonType, QString);

            void setTtitleBarButtonSize(const int &buttonType,QSize);
        protected:
            // 获取内容区容器控件
            QWidget *getContentHolder();
            void drawShadowBorder(QPaintEvent *event);
            void drawSolidBorder(QPaintEvent *event);

        protected:
            virtual void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
            virtual void changeEvent(QEvent *event) Q_DECL_OVERRIDE;
            virtual void reTranslateUI();
            virtual void resizeEvent(QResizeEvent *e)override;
            virtual void showEvent(QShowEvent *e);

        public:
            // 覆盖基类
            void setLayout(QLayout *layout);
            QLayout *layout() const;
            void setWindowTitle(const QString &title);
            QString windowTitle() const;
            void setWindowIcon(const QIcon &icon);
            QIcon windowIcon() const;

            virtual bool eventFilter(QObject *watched, QEvent *event) override ;
        signals:
            public Q_SLOTS :
            void show();
            void showMinimized();
            void showMaximized();
            void showFullScreen();
            void showNormal();
            virtual void open() override ;
            virtual int exec() override ;
        private:
            void createWidgets();
            void createConnects();
            void swithMaxNormal();
        protected:
            TitleBar        *m_pTitleBar = nullptr;
            QVBoxLayout     *m_pMainLayout = nullptr;
            QHBoxLayout     *m_pBottomLayout = nullptr;
            QPushButton     *m_pOKButton = nullptr;
            QPushButton     *m_pCancelButton = nullptr;
            QPushButton     *m_pApplyButton = nullptr;
            QHBoxLayout     *m_pTitleBarLayout = nullptr;
            QFrame          *m_pHTopSeparator = nullptr;///< 顶部分隔线
            QFrame          *m_pHBottomSeparator = nullptr;///< 底部分隔线
            qint32          m_nSeparatorLineMarginLeft = 0;///< 分隔线的边距
            bool            m_isSupportShadowBorder = true;///< 是否支持阴影边框
        private:
            QWidget         *m_pContentHolder = nullptr;
            QWidget         *m_pBottomWidget = nullptr;
            QLayout         *m_pCentralLayout = nullptr;
            FramelessHelper *m_pFramelessHelper = nullptr;
        public:
            static QColor          m_defaultBackgroundColor;///< 默认背景色
            static QColor          m_defaultBorderColor;/// 默认边框色
            static QColor          m_defaultSpecialBackgroungColor;
            static QColor          m_defaultNormalBackgroungColor;
        };
    }//namespace Widgets
}// namespace NQ
