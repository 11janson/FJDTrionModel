#pragma once
#include <QFrame>
#include <functional>
#include <QHBoxLayout>
#include "cswidgets_global.h"
QT_BEGIN_NAMESPACE
class QIcon;
class QLabel;
class QMenu;
class QToolButton;
QT_END_NAMESPACE
namespace CS {
    namespace Widgets {
        // 标题栏图标大小
        const QSize TitleBarButtonIconSize = QSize(30, 20);

        class TitleBarPrivate;
        class CSWIDGETS_EXPORT TitleBar : public QFrame
        {
            Q_OBJECT
        public:
            using CloseFunction = std::function<bool()>;

        public:
            // 标题栏控件类型
            enum PredefinedButtonType
            {
                LogoButton,     ///< logo图标
                MinButton,      ///< 最小化按钮
                MaxButton,      ///< 最大化按钮
                RestoreButton,  ///< 默认显示按钮
                CloseButton,    ///< 关闭按钮
                PredefinedButtonTypeCnt
            };
        public:
            explicit TitleBar(QWidget* parent = Q_NULLPTR);
            virtual ~TitleBar();
        public:
			// 获取对应的按钮
			QToolButton* GetButtonByType(const int &buttonType); 
            // 命中测试
            bool hitTest(const int &x, const int &y) const;
            // 设置标题
            void setTitle(const QString &title);
            // 设置标题的对齐方式
            void setTitleAlignment(Qt::Alignment align = Qt::AlignCenter);
            // 返回标题
            QString title()const;
            QLabel* titleLabel();
            // 设置标题是否显示
            void setTitleVisible(bool visible);
            // 返回标题是否显示
            bool isTitleVisible();
            // 设置按钮图标
            void setTitleButtonIcon(const int &buttonType, const QIcon &icon);
            // 返回按钮图标
            QIcon titleButtonIcon(const int &buttonType) const;
            // 返回按钮
            QToolButton *titleButton(const int &buttonType);
            // 设置按钮是否显示
            void setTitleButtonVisible(const int &buttonType, bool visible);
            // 返回按钮是否显示
            bool isTitleButtonVisible(const int &buttonType);
            // 设置按钮提示
            void setTitleButtonToolTip(const int &buttonType, const QString &tip);
            // 返回按钮提示
            QString titleButtonToolTip(const int &buttonType);
            // 设置当前为最大化状态
            void setMaximized(bool b);
            // 添加一个扩展的按钮
            QToolButton* appendExtendTitleButton(QAction *pAction,
                QSize size = TitleBarButtonIconSize,
                QSize iconSize = TitleBarButtonIconSize);
            // 插入一个扩展按钮
            QToolButton* insertExtendTitleButton(int idx,
                QAction *pAction,
                QSize size = TitleBarButtonIconSize,
                QSize iconSize = TitleBarButtonIconSize);
            // 添加关闭事件
            void appendCloseEvent(CloseFunction);
            /**
            * @brief 实时翻译时，刷新UI文字显示
            */
            void retranslateUi();
            QHBoxLayout* getTitleVBarMainLayout();
			QHBoxLayout* getTopLayout();
			QWidget *getTopLeftWidget(void) { return m_pTopLeftContentWidget; }
        private:
            // 创建子控件
            void createWidgets();
            // 创建信号槽链接
            void createConnects();
        signals:
            // 最大化显示
            void maximumed();
            // 最小化显示
            void minimuned();
            // 默认显示
            void normaled();
            // 关闭
            void closed();

        public slots:
            // 切换最大,最小化显示状态
            void switchMaxMin();
            // 清除选中状态
            void clearChecked();
        private:
            TitleBarPrivate *d;
            QHBoxLayout* m_pMainlayout;
			QWidget *m_pTopLeftContentWidget = nullptr;
        };
    }// namespace Widgets
}//namespace NQ
