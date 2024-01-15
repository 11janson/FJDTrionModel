#include "metahublframelessdialog.h"
#include "cswidgets/titlebar.h"
#include "cswidgets/framelesshelper.h"
#include <QPushButton>
#include <QToolButton>
#include <QVariant>
#include <QDesktopWidget>
#include <QApplication>
#include <QLabel>
#include <QTimer>
#include <QGraphicsDropShadowEffect>
using namespace CS;
using namespace CS::MetahubWidgets;
using namespace CS::Widgets;
namespace CS {
	namespace MetahubWidgets {
		class MetahubFramelessDialogPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit MetahubFramelessDialogPrivate(CS::MetahubWidgets::MetahubFramelessDialog *pQptr);
			virtual ~MetahubFramelessDialogPrivate();

		private:
			/**
			*@brief 创建界面
			*/
			void createWidget(void);
			/**
			*@brief 创建连接
			*/
			void createConnect(void);
			/**
			*@brief 创建翻译
			*/
			void retranslateUi();

		private:

			QFrame		    *m_pContentHoderFrame = nullptr;
			TitleBar        *m_pTitleBar = nullptr;						///<顶部导航
			QWidget			*m_pCentreContentWidget = nullptr;			///<中央区域
			QWidget			*m_pBottomContentWidget = nullptr;			///<底部区域

			QVBoxLayout     *m_pMainLayout = nullptr;
			QVBoxLayout     *m_pMainAllLayout = nullptr;
			QHBoxLayout     *m_pBottomLayout = nullptr;
			QHBoxLayout     *m_pTitleBarLayout = nullptr;

			QHBoxLayout     *m_pCenterHoderLayout = nullptr;
			QLayout			*m_pCenterContentLayout = nullptr;
			QPushButton     *m_pOKButton = nullptr;
			QPushButton     *m_pCancelButton = nullptr;
			QPushButton     *m_pApplyButton = nullptr;
		private:
			friend class CS::MetahubWidgets::MetahubFramelessDialog;
			CS::MetahubWidgets::MetahubFramelessDialog *m_pQptr = nullptr;
		private:
			FramelessHelper *m_pFramelessHelper = nullptr;

		};
	}
}


MetahubFramelessDialogPrivate::MetahubFramelessDialogPrivate(
	CS::MetahubWidgets::MetahubFramelessDialog *pQptr)
{
	m_pQptr = pQptr;
}
MetahubFramelessDialogPrivate::~MetahubFramelessDialogPrivate()
{

}

void MetahubFramelessDialogPrivate::createWidget(void)
{
	m_pMainAllLayout = new QVBoxLayout(m_pContentHoderFrame);
	m_pMainAllLayout->setContentsMargins(0, 0, 0, 0);
	m_pMainAllLayout->setSpacing(0);
	//[!].主布局
	m_pMainLayout = new QVBoxLayout(m_pContentHoderFrame);
	m_pMainLayout->setContentsMargins(24, 0, 24, 0);
	m_pMainLayout->setSpacing(0);


	//[!].顶部导航
	m_pTitleBar = new TitleBar(m_pQptr);
	m_pTitleBar->setProperty("isFramelessDialogTitleBar", true);
	m_pTitleBar->setTitleAlignment(Qt::AlignLeft);
	m_pTitleBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pTitleBarLayout = new QHBoxLayout();
	m_pTitleBarLayout->setSpacing(0);
	m_pTitleBarLayout->setContentsMargins(0, 0, 0, 0);
	m_pTitleBarLayout->addWidget(m_pTitleBar);
	m_pTitleBar->setTitleButtonVisible(TitleBar::CloseButton, true);
	m_pTitleBar->setFixedHeight(48);
	//m_pTitleBar->titleLabel()->setStyleSheet("padding-left:24px;");
	m_pTitleBar->setProperty("transparentbackgroundwidget", true);
	m_pTitleBar->GetButtonByType(TitleBar::CloseButton)->setProperty("transparentbackgroundbutton", true);
    m_pTitleBar->titleLabel()->setStyleSheet("background-color:transparent;");
    //m_pTitleBar->GetButtonByType(TitleBar::CloseButton)->setStyleSheet("background-color:rgb(43, 43, 43)");
	m_pTitleBar->titleLabel()->setProperty("metahubtitlecontent", true);
	m_pTitleBar->getTopLeftWidget()->setProperty("transparentbackgroundwidget", true);


	m_pTitleBar->setTitleButtonVisible(TitleBar::MaxButton, false);
	m_pTitleBar->setTitleButtonVisible(TitleBar::LogoButton, false);
	m_pTitleBar->setTitleButtonVisible(TitleBar::RestoreButton, false);
	m_pTitleBar->setTitleButtonVisible(TitleBar::MinButton, false);
	m_pTitleBar->setVisible(true);
	//m_pMainLayout->addLayout(m_pTitleBarLayout);

	//[!].中心区域
	m_pCenterHoderLayout = new QHBoxLayout();
	m_pCenterHoderLayout->setContentsMargins(0, 0, 0, 0);
	m_pCenterHoderLayout->setSpacing(0);
	m_pCentreContentWidget = new QWidget();
	m_pCentreContentWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pCentreContentWidget->setProperty("centercontentgrayframe", true);
	m_pCenterHoderLayout->addWidget(m_pCentreContentWidget);
	m_pMainLayout->addLayout(m_pCenterHoderLayout);

	//[!].底部区域
	m_pBottomContentWidget = new QWidget();
	m_pBottomContentWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pBottomContentWidget->setFixedHeight(64);
	m_pBottomContentWidget->setProperty("transparentbackgroundwidget", true);
	m_pBottomLayout = new QHBoxLayout(m_pBottomContentWidget);
	m_pBottomLayout->setContentsMargins(0, 16, 0, 16);
	m_pBottomLayout->setSpacing(16);
	m_pBottomLayout->addStretch();
	m_pOKButton = new QPushButton(m_pBottomContentWidget);
	m_pOKButton->setDefault(true);
	m_pApplyButton = new QPushButton(tr("Apply"), m_pBottomContentWidget);
	m_pCancelButton = new QPushButton( m_pBottomContentWidget);
	m_pBottomLayout->addWidget(m_pCancelButton);
	m_pBottomLayout->addWidget(m_pApplyButton);
	m_pBottomLayout->addWidget(m_pOKButton);
	m_pMainLayout->addWidget(m_pBottomContentWidget);
    m_pApplyButton->setVisible(false);
	//[!].无边框操作
	m_pFramelessHelper = new FramelessHelper(m_pQptr);
	m_pFramelessHelper->activateOn(m_pQptr);  
	m_pFramelessHelper->setTitleHeight(m_pTitleBar->height());
	m_pOKButton->setFixedWidth(85);
	m_pCancelButton->setFixedWidth(85);
	m_pOKButton->setProperty("metahubokbutton", true);
	m_pCancelButton->setProperty("metahucancelbbutton", true);
    m_pApplyButton->setProperty("metahucancelbbutton", true);
	m_pMainAllLayout->addLayout(m_pTitleBarLayout);
	m_pMainAllLayout->addLayout(m_pMainLayout);
	return;
}

void MetahubFramelessDialogPrivate::retranslateUi()
{
	
	m_pOKButton->setText(QCoreApplication::translate("metahublframelessdialog", "OK", nullptr));
	m_pCancelButton->setText(QCoreApplication::translate("metahublframelessdialog", "Cancel", nullptr));
}

void MetahubFramelessDialogPrivate::createConnect(void)
{
	connect(m_pTitleBar, &TitleBar::closed, m_pQptr, &MetahubFramelessDialog::slotUIButtonCancel);
	connect(m_pOKButton, &QAbstractButton::clicked, m_pQptr, &MetahubFramelessDialog::slotUIButtonOk);
	connect(m_pCancelButton, &QAbstractButton::clicked, m_pQptr, &MetahubFramelessDialog::slotUIButtonCancel);
	connect(m_pApplyButton, &QAbstractButton::clicked, m_pQptr, &MetahubFramelessDialog::slotUIButtonApply);
}

MetahubFramelessDialog::MetahubFramelessDialog(QWidget *parent
	,Qt::WindowFlags f)
	:QDialog(parent)
	,m_pDptr(new CS::MetahubWidgets::MetahubFramelessDialogPrivate(this))
{
	setWindowFlags(Qt::FramelessWindowHint | windowFlags());
    m_pContentHoderLayout = new QVBoxLayout();
    m_pContentHoderLayout->setContentsMargins(0, 0, 0, 0);
    m_pContentHoderLayout->setSpacing(0);
	m_pDptr->m_pContentHoderFrame = new QFrame(this);
    m_pDptr->m_pContentHoderFrame->setObjectName("m_pContentHoderFrame");
	m_pDptr->m_pContentHoderFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pDptr->m_pContentHoderFrame->setProperty("framehoderbackground", true);
    m_pContentHoderLayout->addWidget(m_pDptr->m_pContentHoderFrame);
    m_pDptr->m_pContentHoderFrame->setStyleSheet("QFrame#m_pContentHoderFrame { border: 1px solid #585858; }");
	QDialog::setLayout(m_pContentHoderLayout);
    QGraphicsDropShadowEffect *shadowEffect = new QGraphicsDropShadowEffect(this);
    shadowEffect->setBlurRadius(24);  
    shadowEffect->setOffset(0, 4);   
    shadowEffect->setColor(QColor(0, 0, 0, 0.5)); 

    this->setGraphicsEffect(shadowEffect);
	m_pDptr->createWidget();
	m_pDptr->createConnect();
	m_pDptr->retranslateUi();

}

MetahubFramelessDialog::~MetahubFramelessDialog()
{
	if (nullptr != m_pDptr) {
		delete m_pDptr;
	}
	m_pDptr = nullptr;
}

void CS::MetahubWidgets::MetahubFramelessDialog::setLayout(QLayout * layout)
{
	m_pDptr->m_pCenterContentLayout = layout;
	m_pDptr->m_pCentreContentWidget->setLayout(layout);
    m_pDptr->m_pCentreContentWidget->setObjectName("m_pCentreContentWidget");
	m_pDptr->m_pCentreContentWidget->setStyleSheet("QWidget{background-color:#363636;}");

	return;
}

void MetahubFramelessDialog::setWindowTitle(const QString &title)
{
	m_pDptr->m_pTitleBar->setTitle(title);
}

QPushButton * MetahubFramelessDialog::getCancelButton(void)
{

	return m_pDptr->m_pCancelButton;
}

QPushButton * MetahubFramelessDialog::getApplyButton(void)
{

    return m_pDptr->m_pApplyButton;
}

QLayout * CS::MetahubWidgets::MetahubFramelessDialog::mainLayout()
{
	return m_pDptr->m_pMainLayout;
	;
}

CS::Widgets::TitleBar * CS::MetahubWidgets::MetahubFramelessDialog::getTitleBar(void)
{
	return m_pDptr->m_pTitleBar;
}

QPushButton * MetahubFramelessDialog::getOKButton(void)
{
	return m_pDptr->m_pOKButton;
}

QLayout * MetahubFramelessDialog::layout() const
{
	if (!m_pDptr->m_pCenterContentLayout) {
		return m_pDptr->m_pCentreContentWidget->layout();
	}

	return m_pDptr->m_pCenterContentLayout;

}

QWidget * MetahubFramelessDialog::bottomWidget(void)
{
	return m_pDptr->m_pBottomContentWidget;
}

QWidget * MetahubFramelessDialog::getContentHolder(void)
{
	return m_pDptr->m_pCentreContentWidget;
}

QHBoxLayout * MetahubFramelessDialog::bottomLayout()
{
	return m_pDptr->m_pBottomLayout;
}

QLayout * MetahubFramelessDialog::getContentHolderLayout(void)
{
	return m_pDptr->m_pCenterContentLayout;
}

QHBoxLayout * CS::MetahubWidgets::MetahubFramelessDialog::getCenterHoderLayout(void)
{
	return m_pDptr->m_pCenterHoderLayout;
}

/**
*@brief 确认按钮槽
*/
void MetahubFramelessDialog::slotUIButtonOk(void)
{
	QDialog::accept();
}

/**
*@brief 取消按钮
*/
void MetahubFramelessDialog::slotUIButtonCancel(void)
{
	QDialog::reject();
}


void MetahubFramelessDialog::slotUIButtonApply(void)
{
	return;
}

QLayout * MetahubFramelessDialog::getTilteBarLayout()
{
	return m_pDptr->m_pTitleBarLayout;
}
QToolButton *MetahubFramelessDialog::getWindowTitleMaxButton()
{
	m_pDptr->m_pTitleBar->setTitleButtonVisible(TitleBar::MaxButton, true);
	return m_pDptr->m_pTitleBar->GetButtonByType(TitleBar::MaxButton);
}

FramelessHelper * CS::MetahubWidgets::MetahubFramelessDialog::getFramelessHelper(void)
{
    return m_pDptr->m_pFramelessHelper;
}

QFrame * CS::MetahubWidgets::MetahubFramelessDialog::getCenterMainFrame(void)
{
    return m_pDptr->m_pContentHoderFrame;
}


void CS::MetahubWidgets::MetahubFramelessDialog::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape) {
        return;
    }
    QDialog::keyPressEvent(event);
}

void CS::MetahubWidgets::MetahubFramelessDialog::showEvent(QShowEvent *e)
{
	QDialog::showEvent(e);
    this->setFixedSize(this->size());
}

void CS::MetahubWidgets::MetahubFramelessDialog::resizeEvent(QResizeEvent *e)
{
    QDialog::resizeEvent(e);
}

#include "metahublframelessdialog.moc"
