#include "framelessdialog.h"
#include "framelesshelper.h"
#include "titlebar.h"
#include "frameline.h"
using namespace CS::Widgets;
#include <QLayout>
#include <QBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QIcon>
#include <QLayout>
#include <QGraphicsDropShadowEffect>
#include <QVariant>
#include <QDesktopWidget>
#include <QApplication>
#include <QPushButton>
#include <QAbstractButton>
#include <QDebug>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>
#include <QScreen>
#include "FJStyleManager.h"
QColor FramelessDialog::m_defaultBackgroundColor=qRgb(69,69,69);
QColor FramelessDialog::m_defaultBorderColor=qRgb(56,56,56);
QColor FramelessDialog::m_defaultSpecialBackgroungColor=qRgb(34,33,38);
QColor FramelessDialog::m_defaultNormalBackgroungColor=qRgb(56,58,63);
FramelessDialog::FramelessDialog(QWidget *parent, Qt::WindowFlags f)
    : FJBaseWidget(parent, f | Qt::X11BypassWindowManagerHint)
    , m_pContentHolder(nullptr)
    , m_pBottomWidget(nullptr)
    , m_pCentralLayout(nullptr)
{
    createWidgets();
    createConnects();
    setMinButtonVisible(false);
    setMaxButtonVisible(false);
	m_pTitleBar->setTitleButtonVisible(TitleBar::LogoButton, false);
}

FramelessDialog::~FramelessDialog()
{

}

void FramelessDialog::InitFJStyle()
{
}


void FramelessDialog::setTitleButtonIcon(const int &buttonType, const QIcon &icon)
{
    m_pTitleBar->setTitleButtonIcon(buttonType, icon);
}

void FramelessDialog::setDialogtMovable(bool b)
{
    m_pFramelessHelper->setWidgetMovable(b);  //设置窗体可移动
}

void FramelessDialog::setDialogMovable(bool b)
{
    m_pFramelessHelper->setWidgetMovable(b);  //设置窗体可移动
}

void FramelessDialog::setRubberBandOnMove(bool b)
{
    m_pFramelessHelper->setRubberBandOnMove(b);  //设置橡皮筋效果-可移动
}

void FramelessDialog::setDialogResizable(bool b)
{
    m_pFramelessHelper->setWidgetResizable(b);  //设置窗体可缩放
}

void FramelessDialog::setRubberBandOnResize(bool b)
{
    m_pFramelessHelper->setRubberBandOnResize(b);
}
void FramelessDialog::setLayout(QLayout *layout)
{
    QMargins margins = layout->contentsMargins();
    m_pCentralLayout = layout;
    m_pCentralLayout->setContentsMargins(margins.left() , margins.top(),
        margins.right() , margins.bottom());
    m_pMainLayout->insertLayout(2, m_pCentralLayout);
}
QLayout *FramelessDialog::layout() const
{
    if (!m_pCentralLayout && m_pContentHolder)
        return m_pContentHolder->layout();
    else
        return m_pCentralLayout;
}

void FramelessDialog::SetContentHolder(QWidget * widget)
{
	if (!m_pContentHolder)
	{
		//m_pContentHolder = new QWidget(this);
		//m_pContentHolder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		m_pMainLayout->insertWidget(2, widget);
	}
}

QWidget *FramelessDialog::getContentHolder()
{
    if (!m_pContentHolder)
    {
        m_pContentHolder = new QWidget(this);
        m_pContentHolder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        m_pMainLayout->insertWidget(2, m_pContentHolder);
    }
    return m_pContentHolder;
}
void FramelessDialog::setWindowTitle(const QString &title)
{
    m_pTitleBar->setTitle(title);
    m_pTitleBar->setStyleSheet("color:#F2F2F2;font-size:16px;background-color:#2B2B2B;");
}

void FramelessDialog::setWindowTitleAlignment(Qt::Alignment align /*= Qt::AlignCenter*/)
{
    m_pTitleBar->setTitleAlignment(align);
}

QString FramelessDialog::windowTitle() const
{
    return m_pTitleBar->title();
}

void FramelessDialog::setWindowIcon(const QIcon &icon)
{
    m_pTitleBar->setTitleButtonIcon(TitleBar::LogoButton, icon);
    QDialog::setWindowIcon(icon);
}

void FramelessDialog::hideTitleIcon()
{
	m_pTitleBar->setTitleButtonVisible(TitleBar::LogoButton,false);
}

QIcon FramelessDialog::windowIcon() const
{
    return m_pTitleBar->titleButtonIcon(TitleBar::LogoButton);
}

QVBoxLayout* FramelessDialog::getMainLayout()
{
    return  m_pMainLayout;
}

void FramelessDialog::createWidgets()
{
    setWindowFlags(Qt::FramelessWindowHint | windowFlags());
    setAttribute(Qt::WA_TranslucentBackground, true);
    setProperty("isFramelessDialog", true);

    m_pTitleBar = new TitleBar(this);
    m_pTitleBar->setProperty("isFramelessDialogTitleBar",true);
    m_pTitleBar->setTitleAlignment(Qt::AlignCenter);
    m_pTitleBarLayout = new QHBoxLayout();
    m_pTitleBarLayout->setSpacing(0);
    m_pTitleBarLayout->setContentsMargins(0, 0, 0, 0);
    m_pTitleBarLayout->addWidget(m_pTitleBar);
    QDesktopWidget* desktopWidget = QApplication::desktop();
    setMaximumSize(desktopWidget->availableGeometry().size());

    setContentsMargins(0, 0, 0, 0);

    installEventFilter(m_pTitleBar);

    m_pMainLayout = new QVBoxLayout();
    m_pMainLayout->setSpacing(0);

#ifdef Q_OS_LINUX
    m_pMainLayout->setContentsMargins(1, 1, 1, 1);
#else
#ifdef NO_USE_SHADOWBORDER // 不使用带阴影的边框
    m_pMainLayout->setContentsMargins(1, 1, 1, 1);
#else
    if (m_isSupportShadowBorder)
        m_pMainLayout->setContentsMargins(10, 10, 10, 10);
    else
        m_pMainLayout->setContentsMargins(1, 1, 1, 1);
#endif
#endif

    m_pMainLayout->addLayout(m_pTitleBarLayout);
    m_pHTopSeparator = FrameLine::createHorizontalLine("isBackgroundSeparator",
                                                       this,1, m_nSeparatorLineMarginLeft);
    m_pMainLayout->addWidget(m_pHTopSeparator);
    m_pHBottomSeparator = FrameLine::createHorizontalLine("isBackgroundSeparator",
                                                          this,1, m_nSeparatorLineMarginLeft);
    m_pMainLayout->addWidget(m_pHBottomSeparator);
    m_pBottomWidget = new QWidget(this);
    m_pBottomLayout = new QHBoxLayout(m_pBottomWidget);
    m_pBottomLayout->setSpacing(16);
    m_pBottomLayout->setContentsMargins(16, 16, 24, 16);
    m_pBottomLayout->addStretch();
    m_pOKButton = new QPushButton( m_pBottomWidget);
    m_pOKButton->setDefault(true);
	m_pOKButton->setProperty("metahubokbutton", true);
    connect(m_pOKButton, &QAbstractButton::clicked, this, &QDialog::accept);
    m_pCancelButton = new QPushButton(m_pBottomWidget);
	m_pCancelButton->setProperty("metahucancelbbutton", true);
    connect(m_pCancelButton, &QAbstractButton::clicked, this, &QDialog::reject);
    m_pApplyButton  = new QPushButton(m_pBottomWidget);
	m_pApplyButton->setProperty("metahucancelbbutton", true);
    setApplyButtonVisible(false);


	m_pOKButton->setText(QCoreApplication::translate("FramelessDialog", "OK", nullptr));
	m_pCancelButton->setText(QCoreApplication::translate("FramelessDialog", "Cancel", nullptr));
	m_pApplyButton->setText(QCoreApplication::translate("FramelessDialog", "Apply", nullptr));
	
	m_pBottomLayout->addWidget(m_pCancelButton);
	m_pBottomLayout->addWidget(m_pApplyButton);
    m_pBottomLayout->addWidget(m_pOKButton);
    m_pMainLayout->addWidget(m_pBottomWidget);
    QDialog::setLayout(m_pMainLayout);

    m_pFramelessHelper = new FramelessHelper(this);
    m_pFramelessHelper->activateOn(this);  //激活当前窗体
    m_pFramelessHelper->setTitleHeight(m_pTitleBar->height());  //设置窗体的标题栏高度
}

QPushButton * FramelessDialog::GetOKButton()
{
	return m_pOKButton;
}

QPushButton * FramelessDialog::GetCancelButton()
{
	return m_pCancelButton;
}

QPushButton * FramelessDialog::GetApplyButton()
{
	return m_pApplyButton;
}
TitleBar *FramelessDialog::getTitleBar()
{
    return m_pTitleBar;
}
void FramelessDialog::createConnects()
{
    connect(m_pTitleBar, &TitleBar::closed, this, &FramelessDialog::close);
    connect(m_pTitleBar, &TitleBar::minimuned, this, &FramelessDialog::showMinimized);
    connect(m_pTitleBar, &TitleBar::maximumed, this, &FramelessDialog::swithMaxNormal);
    connect(m_pTitleBar, &TitleBar::normaled, this, &FramelessDialog::swithMaxNormal);
}

void FramelessDialog::swithMaxNormal()
{
    if (isMaximized())
    {
        showNormal();
    }
    else
    {
        showMaximized();
    }
}

void FramelessDialog::show()
{
    QDialog::show();
    if (isMaximized())
        m_pTitleBar->setMaximized(true);
    else
        m_pTitleBar->setMaximized(false);
}

void FramelessDialog::showFullScreen()
{
    QDialog::showFullScreen();
}

void FramelessDialog::showMaximized()
{
    QDialog::showMaximized();
    m_pTitleBar->setMaximized(true);
}

void FramelessDialog::showMinimized()
{
    QDialog::showMinimized();
}

void FramelessDialog::showNormal()
{
    QDialog::showNormal();
    m_pTitleBar->setMaximized(false);
}

int FramelessDialog::exec()
{
    return QDialog::exec();
}
void FramelessDialog::open()
{
    return QDialog::open();
}
void FramelessDialog::setMinButtonVisible(bool b)
{
    m_pTitleBar->setTitleButtonVisible(TitleBar::MinButton, b);
}

void FramelessDialog::setMaxButtonVisible(bool b)
{
    m_pTitleBar->setTitleButtonVisible(TitleBar::MaxButton, b);
    m_pTitleBar->setTitleButtonVisible(TitleBar::RestoreButton, b);
    if (!b)
    {
        setDialogResizable(b);
    }
    else 
    {
        m_pTitleBar->setMaximized(isMaximized());
    }
}

void FramelessDialog::setCloseButtonVisible(bool b)
{
    m_pTitleBar->setTitleButtonVisible(TitleBar::CloseButton, b);
}

bool FramelessDialog::getCloseButtonVisible() const
{
    if(m_pTitleBar)
    {
        return m_pTitleBar->isTitleButtonVisible(TitleBar::CloseButton);
    }

    return false;
}

void FramelessDialog::setMessageboxStyleSheet()
{
	m_pTitleBar->GetButtonByType(TitleBar::LogoButton)->setStyleSheet("background-color: transparent;border:none;padding: 6px 0px 0px 20px;");
	m_pTitleBar->titleLabel()->setObjectName("Messageboxtitlelabel");
	m_pTitleBar->titleLabel()->setStyleSheet("background - color: #1C1C1C; border:none; font-size: 16px;color: #ffffff;padding: -10px 0px 5px 0px; ");
	m_pTitleBar->setTitleButtonVisible(TitleBar::LogoButton, true);
	//m_pTitleBar->setFixedHeight(23);
	//m_pTitleBar->setStyleSheet("padding: 3px 0px 0px 0px;");
	//m_pTitleBar->setFixedHeight(80);
}


void FramelessDialog::setApplyButtonVisible(bool b)
{
    m_pApplyButton->setVisible(b);
}

void FramelessDialog::setOKButtonVisible(bool b)
{
    m_pOKButton->setVisible(b);
}

void FramelessDialog::setOKButtonDefault(bool b)
{
    m_pOKButton->setDefault(b);
}
void FramelessDialog::setCancelButtonVisible(bool b)
{
    m_pCancelButton->setVisible(b);
}

void FramelessDialog::appendExtendTitleButton(QAction *pAction,
    QSize size /*= TitleBarButtonIconSize*/,
    QSize iconSize /*= TitleBarButtonIconSize*/)
{
    m_pTitleBar->appendExtendTitleButton(pAction, size, iconSize);
}

void FramelessDialog::insertExtendTitleButton(int idx,
    QAction *pAction,
    QSize size /*= TitleBarButtonIconSize*/,
    QSize iconSize /*= TitleBarButtonIconSize*/)
{
    m_pTitleBar->insertExtendTitleButton(idx, pAction, size, iconSize);
}

int FramelessDialog::noCentralHeight()
{
    return m_pTitleBar->height() + 50 + 4;
}

void FramelessDialog::setBottomBarVisible(bool b)
{
    m_pHBottomSeparator->setVisible(b);
    m_pBottomWidget->setVisible(b);
}

bool FramelessDialog::eventFilter(QObject *watched, QEvent *event)
{
    return QDialog::eventFilter(watched, event);
}

void FramelessDialog::drawShadowBorder(QPaintEvent *event)
{
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    path.addRect(10, 10, this->width() -20, this->height() - 20);

    QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.fillPath(path, QBrush(QColor(m_defaultBackgroundColor.red(),
		m_defaultBackgroundColor.green(),
		m_defaultBackgroundColor.blue())));
    //取消边框阴影绘制
	//QColor color(0, 0, 0, 35);
	//for (int i = 0; i < 10; i++)
	//{
	//	QPainterPath path;
	//	path.setFillRule(Qt::WindingFill);
	//	path.addRect(10 - i, 10 - i, this->width() - (10 - i) * 2, this->height() - (10 - i) * 2);
	//	color.setAlpha(150 - qSqrt(i) * 50);
	//	painter.setPen(color);
	//	painter.drawPath(path);
	//}

	QPen pen;
	pen.setWidth(1);
	pen.setColor(QColor(112, 112, 112));
	painter.setPen(pen);
	painter.drawRect(10, 10, this->width()-20, this->height()-20);
	painter.restore();
}

void FramelessDialog::drawSolidBorder(QPaintEvent *event)
{
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    path.addRect(1, 1, this->width()-2, this->height()-2);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillPath(path, QBrush(QColor(m_defaultBackgroundColor.red(),
                                         m_defaultBackgroundColor.green(),
                                         m_defaultBackgroundColor.blue())));
    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setColor(QColor(m_defaultBorderColor.red(),
                        m_defaultBorderColor.green(),
                        m_defaultBorderColor.blue()));

    painter.setPen(pen);
    painter.drawRect(0, 0, this->width(), this->height());
    painter.restore();
}

void FramelessDialog::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
#ifdef Q_OS_LINUX
    drawSolidBorder(event);
#else
#ifdef NO_USE_SHADOWBORDER // 不使用带阴影的边框
    drawSolidBorder(event);
#else
	if (m_isSupportShadowBorder)
		drawShadowBorder(event);
	else
		drawSolidBorder(event);
#endif
#endif
}

void FramelessDialog::setHorizontalLineHeight(int iHeight)
{
	if (m_pHTopSeparator)
	{
		m_pHTopSeparator->setFixedHeight(iHeight);
	}
	if (m_pHBottomSeparator)
	{
		m_pHBottomSeparator->setFixedHeight(iHeight);
	}
}

void FramelessDialog::changeEvent(QEvent *event)
{
    QWidget::changeEvent(event);
    switch(event->type())
    {
    case QEvent::LanguageChange:
        this->reTranslateUI();
        break;
    default:
        break;
    }
}

void FramelessDialog::reTranslateUI()
{}

void FramelessDialog::setTitleBarLabelStyleSheet(QString str)
{
    return m_pTitleBar->titleLabel()->setStyleSheet(str);
}

void FramelessDialog::setTtitleBarButtonStyle(const int &buttonType, QString str)
{
    m_pTitleBar->GetButtonByType(buttonType)->setStyleSheet(str);
}

void FramelessDialog::setTtitleBarButtonSize(const int &buttonType, QSize size)
{
    m_pTitleBar->GetButtonByType(buttonType)->setFixedSize(size);
}

void CS::Widgets::FramelessDialog::setOkButton(QPushButton* pButton)
{
    if (!pButton) {
        return;
    }
    if (m_pOKButton) {
        delete m_pOKButton;
    }
    m_pOKButton = pButton;
    m_pBottomLayout->addWidget(m_pOKButton);
    m_pOKButton->setText(QCoreApplication::translate("FramelessDialog", "OK", nullptr));
    m_pOKButton->setProperty("metahubokbutton", true);
    m_pOKButton->setDefault(true);
}

void CS::Widgets::FramelessDialog::resizeEvent(QResizeEvent *e)
{
    QDialog::resizeEvent(e);
}

void CS::Widgets::FramelessDialog::showEvent(QShowEvent *e)
{
    QDialog::showEvent(e);
    this->setFixedSize(size());
}
