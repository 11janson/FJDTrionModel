#include "titlebar.h"
#include "toolbutton.h"
using namespace CS::Widgets;
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include <QStyle>
#include <QAction>
#include <QToolButton>
#include <QWidget>
#include <QCoreApplication>
#include "FJStyleManager.h"
namespace CS {
    namespace Widgets {
        // 标题栏高度
        const int TitleBarHeight = 48/*30*/;

        class TitleBarPrivate
        {
        public:
            TitleBarPrivate(TitleBar *pTitleBar);
            ~TitleBarPrivate();

            void clearChecked();

            TitleBar                    *pTitleBar;
            QLabel                      *titleLabel;
            bool                        isMaximized;
            QVector<QToolButton*>       buttons;
            QVector<bool>               buttonsVisible;
            QHBoxLayout                 *pExtendTitleButtonLayout;
            TitleBar::CloseFunction     closeFunciton = []() { return true; };//default
            void retranslateUi();
        };
    }
}
TitleBarPrivate::TitleBarPrivate(TitleBar *pTitleBar)
    :pTitleBar(pTitleBar)
{
    titleLabel = new QLabel(pTitleBar);
    //titleLabel->setStyleSheet("background-color:green;");
    titleLabel->setProperty("isTitleBarTitle", true);
    buttonsVisible.resize(TitleBar::PredefinedButtonTypeCnt);
    for (int i = 0; i < TitleBar::PredefinedButtonTypeCnt; i++)
    {
        QToolButton *button = new ToolButton(pTitleBar);
        button->setIconSize(QSize(16,16));
        button->setAutoRaise(true);
        //button->setAutoExclusive(true);
        button->setAutoFillBackground(true);
        button->setFocusPolicy(Qt::NoFocus);
        button->setCheckable(false);
        button->setFixedSize(20, 20);
        buttons.push_back(button);
        buttonsVisible[i] = true;
    }


    buttons[TitleBar::MinButton]->setProperty("isTitleBarMinButton", true);
    buttons[TitleBar::MaxButton]->setProperty("isTitleBarMaxButton", true);
    buttons[TitleBar::RestoreButton]->setProperty("isTitleBarRestoreButton", true);
    buttons[TitleBar::CloseButton]->setProperty("isTitleBarCloseButton", true);

	QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
	closeicon.addPixmap(QPixmap(QCoreApplication::applicationDirPath() + "qssimage/smallwindowcloseicon.png"), QIcon::Active, QIcon::Off);
	closeicon.addPixmap(QPixmap(QCoreApplication::applicationDirPath() + "qssimage/smallwindowcloseicon.png"), QIcon::Disabled, QIcon::Off);
	buttons[TitleBar::CloseButton]->setIcon(closeicon);
    QString closeButtonStyle = "QToolButton{background-color:transparent;border: 0px solid #585858;border-radius: 0px;}\n"
        "QToolButton:hover{background-color:#333333;}\n"
        "QToolButton:pressed{background-color:#333333;padding:0px 0px 0px 0px;}";
	buttons[TitleBar::CloseButton]->setStyleSheet(closeButtonStyle);
    buttons[TitleBar::MinButton]->setStyleSheet(closeButtonStyle);
    buttons[TitleBar::MaxButton]->setStyleSheet(closeButtonStyle);
    buttons[TitleBar::RestoreButton]->setStyleSheet(closeButtonStyle);

	QIcon maxicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/systembuttonmaximize.png");
	buttons[TitleBar::MaxButton]->setIcon(maxicon);
	QIcon noemalicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowmaxicon.png");
	buttons[TitleBar::RestoreButton]->setIcon(noemalicon);

    buttons[TitleBar::LogoButton]->setProperty("isTitleBarLogoButton", true);
    buttons[TitleBar::LogoButton]->setDisabled(false);
	buttons[TitleBar::LogoButton]->setFixedWidth(42);
	//buttons[TitleBar::LogoButton]->setStyleSheet("background-color: transparent;border:none;padding: 0px 0px 0px 20px;");
    isMaximized = false;

    pExtendTitleButtonLayout = new QHBoxLayout;
    pExtendTitleButtonLayout->setContentsMargins(0, 0, 0, 0);
    pExtendTitleButtonLayout->setSpacing(2);
    retranslateUi();
}


TitleBarPrivate::~TitleBarPrivate()
{

}

void TitleBarPrivate::clearChecked()
{
    for (int i = 0; i < TitleBar::PredefinedButtonTypeCnt; i++)
    {
        QToolButton *button = buttons[i];
        button->setChecked(false);
    }
}
void TitleBarPrivate::retranslateUi()
{
    //buttons[TitleBar::MinButton]->setToolTip(TitleBar::tr("Minimize"));
    //buttons[TitleBar::MaxButton]->setToolTip(TitleBar::tr("Maximization"));
    //buttons[TitleBar::RestoreButton]->setToolTip(TitleBar::tr("Restore"));
    //buttons[TitleBar::CloseButton]->setToolTip(TitleBar::tr("Close"));
}

////////////////////////////////////////////////////////////////////////
TitleBar::TitleBar(QWidget* parent)
    :QFrame(parent)
{
    d = new TitleBarPrivate(this);
    createWidgets();
    createConnects();
}

TitleBar::~TitleBar()
{
    delete d;
    d = nullptr;
}

QToolButton* TitleBar::GetButtonByType(const int &buttonType)
{
	return d->buttons[buttonType];
}

QHBoxLayout* TitleBar::getTitleVBarMainLayout()
{
    return m_pMainlayout;
}
QHBoxLayout* TitleBar::getTopLayout()
{
	return d->pExtendTitleButtonLayout;
}

void TitleBar::createWidgets()
{
    setFixedHeight(TitleBarHeight);
    setProperty("isTitleBar", true);
    setProperty("isForeground", true);
	m_pMainlayout = new QHBoxLayout;

	m_pTopLeftContentWidget = new QWidget(this);
	m_pTopLeftContentWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pTopLeftContentWidget->setVisible(false);
	m_pMainlayout->addWidget(m_pTopLeftContentWidget, Qt::AlignBottom | Qt::AlignLeft);
    m_pMainlayout->addWidget(d->buttons[TitleBar::LogoButton], Qt::AlignBottom);
    m_pMainlayout->addWidget(d->titleLabel);
	d->titleLabel->setObjectName("ccqfctitleLabel");
    m_pMainlayout->addLayout(d->pExtendTitleButtonLayout);
    m_pMainlayout->addWidget(d->buttons[TitleBar::MinButton], 0,  Qt::AlignBottom);
    m_pMainlayout->addWidget(d->buttons[TitleBar::MaxButton], 0, Qt::AlignBottom);
    m_pMainlayout->addWidget(d->buttons[TitleBar::RestoreButton], 0, Qt::AlignBottom);
    m_pMainlayout->addWidget(d->buttons[TitleBar::CloseButton], 0, Qt::AlignBottom);
    m_pMainlayout->setContentsMargins(0, 11, 24, 11);
    m_pMainlayout->setSpacing(0);
    setLayout(m_pMainlayout);
}

void TitleBar::createConnects()
{
    // connect(d->buttons[TitleBar::CloseButton], SIGNAL(clicked()), this, SIGNAL(closed()));

    connect(d->buttons[TitleBar::CloseButton], &QToolButton::clicked, [=]()
    {
        if (d->closeFunciton())
        {
            emit closed();
        }
        else
        {
            //
        }
    });

    connect(d->buttons[TitleBar::MinButton], SIGNAL(clicked()), this, SIGNAL(minimuned()));
    connect(d->buttons[TitleBar::MaxButton], SIGNAL(clicked()), this, SLOT(switchMaxMin()));
    connect(d->buttons[TitleBar::RestoreButton], SIGNAL(clicked()), this, SLOT(switchMaxMin()));
}

bool TitleBar::hitTest(const int &x, const int &y) const
{
    if (rect().contains(x, y)) {
        if (!this->childAt(x, y)) {
            return true;
        }
    }
    return false;
}


void TitleBar::switchMaxMin()
{
    if (d->buttonsVisible[MaxButton])
        d->buttons[TitleBar::MaxButton]->setVisible(d->isMaximized);
    if (d->buttonsVisible[RestoreButton])
        d->buttons[TitleBar::RestoreButton]->setVisible(!d->isMaximized);
    if (!d->isMaximized)
        emit maximumed();
    else
        emit normaled();
    d->isMaximized = !d->isMaximized;
}

void TitleBar::clearChecked()
{
    d->clearChecked();
}

void TitleBar::setTitle(const QString &title)
{
    d->titleLabel->setText(title);
	d->titleLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
}

void TitleBar::setTitleAlignment(Qt::Alignment align /*= Qt::AlignCenter*/)
{
    d->titleLabel->setAlignment(align);
}
QString TitleBar::title() const
{
    return d->titleLabel->text();
}
QLabel* TitleBar::titleLabel()
{
    return d->titleLabel;
}
void TitleBar::setTitleVisible(bool visible)
{
    d->titleLabel->setVisible(visible);
}

bool TitleBar::isTitleVisible()
{
    return d->titleLabel->isVisible();
}

void TitleBar::setTitleButtonIcon(const int &buttonType, const QIcon &icon)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        d->buttons[buttonType]->setIcon(icon);
	//d->buttons[buttonType]->setStyleSheet("padding-left:10px;");
}

QIcon TitleBar::titleButtonIcon(const int &buttonType) const
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        return d->buttons[buttonType]->icon();
    else
        return QIcon();
}

QToolButton *TitleBar::titleButton(const int &buttonType)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        return d->buttons[buttonType];
    else
        return nullptr;
}

void TitleBar::setTitleButtonVisible(const int &buttonType, bool visible)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
    {
        d->buttons[buttonType]->setVisible(visible);
        d->buttonsVisible[buttonType] = visible;
    }
}

bool TitleBar::isTitleButtonVisible(const int &buttonType)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        return d->buttons[buttonType]->isVisible();
    else
        return false;
}

void TitleBar::setTitleButtonToolTip(const int &buttonType, const QString &tip)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        return d->buttons[buttonType]->setToolTip(tip);
}

QString TitleBar::titleButtonToolTip(const int &buttonType)
{
    Q_ASSERT(buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt));
    if (buttonType >= 0 && buttonType < static_cast<int>(PredefinedButtonTypeCnt))
        return d->buttons[buttonType]->toolTip();
    else
        return QString::null;

}

void TitleBar::setMaximized(bool b)
{
    d->isMaximized = b;
    if (d->buttonsVisible[MaxButton])
        d->buttons[TitleBar::MaxButton]->setVisible(!d->isMaximized);
    if (d->buttonsVisible[RestoreButton])
        d->buttons[TitleBar::RestoreButton]->setVisible(d->isMaximized);
}

QToolButton* TitleBar::appendExtendTitleButton(QAction *pAction,
    QSize size,
    QSize iconSize)
{
    int cnt = d->pExtendTitleButtonLayout->count();
    return insertExtendTitleButton(cnt, pAction, size, iconSize);
}

QToolButton* TitleBar::insertExtendTitleButton(int idx,
    QAction *pAction,
    QSize size,
    QSize iconSize)
{
    QToolButton *pButton = new QToolButton(this);
    pButton->setIcon(pAction->icon());
    pButton->setIconSize(iconSize);
    pButton->setFixedSize(size);
    pButton->setToolTip(pAction->toolTip());
    pButton->setDefaultAction(pAction);
    d->pExtendTitleButtonLayout->insertWidget(idx, pButton);
    return pButton;
}

void TitleBar::appendCloseEvent(TitleBar::CloseFunction fun)
{
    d->closeFunciton = fun;
}

void TitleBar::retranslateUi()
{
    d->retranslateUi();
}
