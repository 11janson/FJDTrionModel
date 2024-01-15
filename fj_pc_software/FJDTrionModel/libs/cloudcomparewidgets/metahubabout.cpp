#include <QEvent>
#include <QPainter>
#include <QPixmap>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDebug>
#include <QtMath>
#include <mutex>
#include <QApplication>

#include"metahubabout.h"
#include "cswidgets/titlebar.h"
#include "cswidgets/framelesshelper.h"
#include "cloudcompareutils/icore.h"
using namespace CS::Widgets;
using namespace CS::Core;
using namespace CS::MetahubWidgets;
namespace CS {
	namespace MetahubWidgets {
		class MetahubAboutdialogPrivate : public  QObject
		{
			Q_OBJECT
		public:
			explicit MetahubAboutdialogPrivate(CS::MetahubWidgets::MetahubAboutdialog *qptr);
			virtual ~MetahubAboutdialogPrivate();
		public:
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

			/**
			*@brief 创建样式
			*/
			void createStyleSheet(void);

          
		private:
			QFrame		    *m_pFrame = nullptr;
			QVBoxLayout     *m_pCenterLayout = nullptr;
			QLabel          *m_pLogoLabel = nullptr;
			QLabel          *m_pTextLabel = nullptr;
			QLabel          *m_pmiddleLabel = nullptr;
			QLabel          *m_pmiddleLogoLabel = nullptr;
			QLabel          *m_pDownLabel = nullptr;
			QLabel          *m_pDonLabel = nullptr;
			QLabel          *m_pDwnLabel = nullptr;
			QPushButton     *m_pCloseButton = nullptr;
			QString          m_pVersionQString;
		private:
			QPushButton		*m_pContentIconButton = nullptr;
			QTextEdit		*m_pTextEdit = nullptr;


		private:
			QLabel           *m_pVirsonLabel = nullptr;
		private:
			friend class CS::MetahubWidgets::MetahubAboutdialog;
			CS::MetahubWidgets::MetahubAboutdialog *m_qptr = nullptr;

		};
	}
}
MetahubAboutdialogPrivate::MetahubAboutdialogPrivate(
	CS::MetahubWidgets::MetahubAboutdialog *qptr)
{
	m_qptr = qptr;
}
MetahubAboutdialogPrivate::~MetahubAboutdialogPrivate()
{

}

void MetahubAboutdialogPrivate::createWidget()
{
	
	m_qptr->setFixedSize(400, 180);
	m_qptr->getCancelButton()->setVisible(false);
	m_qptr->getOKButton()->setVisible(false);
	m_qptr->bottomWidget()->setVisible(false);
	m_qptr->mainLayout()->setContentsMargins(0, 0, 0, 10);
	m_qptr->getTitleBar()->setTitleButtonVisible(TitleBar::PredefinedButtonType::LogoButton, true);
	m_qptr->getTitleBar()->GetButtonByType(TitleBar::PredefinedButtonType::LogoButton)->setFixedSize(QSize(16, 30));
	m_qptr->getTitleBar()->titleLabel()->setStyleSheet("padding-left:0px;background-color:transparent;");
	m_qptr->getTitleBar()->getTopLayout()->setSpacing(0);
	int nl;
	int nt;
	int nr;
	int nb;
	m_qptr->getTitleBar()->getTitleVBarMainLayout()->getContentsMargins(&nl, &nt, &nr, &nb);
	m_qptr->getTitleBar()->getTitleVBarMainLayout()->setContentsMargins(16, nt, nr, nb);
	m_qptr->getTitleBar()->titleLabel()->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
	
	QVBoxLayout *pMainLayout = new QVBoxLayout(m_qptr);
	pMainLayout->setContentsMargins( 0, 0, 0, 0);
	pMainLayout->addStretch(0);
	//[!].//中心图标
	QFrame *pContentFrame = new QFrame(m_qptr);
	pContentFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	QHBoxLayout *pContentHLayout = new QHBoxLayout(pContentFrame);
	pContentHLayout->setContentsMargins(0, 0, 0, 0);
	pContentHLayout->setSpacing(0);
	m_pContentIconButton = new QPushButton();
	m_pContentIconButton->setProperty("normalcybutton", true);
	m_pContentIconButton->setCheckable(false);
	m_pContentIconButton->setFixedSize(QSize(24, 24));
	m_pContentIconButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pContentIconButton->setIconSize(QSize(24, 24));
	pContentHLayout->addWidget(m_pContentIconButton);
	pContentHLayout->addSpacing(10);
	QLabel *pAppNameLabel = new QLabel(m_qptr);
	pAppNameLabel->setAlignment(Qt::AlignLeft);
	pAppNameLabel->setText(tr("FJD Trion Model"));
	pAppNameLabel->setStyleSheet("font-size : 20px");
	pContentHLayout->setContentsMargins(110, 0, 0, 10);
	pContentHLayout->addWidget(pAppNameLabel);
	pMainLayout->addWidget(pContentFrame);
	

	//[!].底部版本号
	m_pVirsonLabel = new QLabel(m_qptr);
	m_pVirsonLabel->setAlignment(Qt::AlignCenter);
	
	m_pVirsonLabel->setStyleSheet("background-color: rgb(42, 44, 45);");
	m_pVirsonLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pVirsonLabel->setFixedHeight(45);
	pMainLayout->addWidget(m_pVirsonLabel);
	m_qptr->setLayout(pMainLayout);


}
void MetahubAboutdialogPrivate::createConnect()
{

}
void MetahubAboutdialogPrivate::retranslateUi()
{
	
//	m_pDownLabel->setText(tr("Version:")+ m_pVersionQString);
	//m_pmiddleLabel->setText(tr("Trion Metahub"));
}
void CS::MetahubWidgets::MetahubAboutdialogPrivate::createStyleSheet(void)
{

	QString strStyleSheet = CS::Core::ICore::themeStyleSheet();
	QFile styleSheet(strStyleSheet);
	if (styleSheet.open(QIODevice::ReadOnly)) {
		m_qptr->setStyleSheet(styleSheet.readAll());
	}
	

	CS::Widgets::TitleBar *pTitleBar = m_qptr->getTitleBar();
	
	pTitleBar->setTitleButtonIcon(TitleBar::PredefinedButtonType::LogoButton,
		ICore::resourceThemeImage("logo.png"));
	return;
}

MetahubAboutdialog::MetahubAboutdialog(QWidget *parent
	, Qt::WindowFlags f)
	:MetahubFramelessDialog(parent)
	, m_dptr(new CS::MetahubWidgets::MetahubAboutdialogPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();
	m_dptr->createStyleSheet();

}

MetahubAboutdialog::~MetahubAboutdialog()
{
	if (nullptr != m_dptr) {
		delete m_dptr;
	}
	m_dptr = nullptr;
}
int CS::MetahubWidgets::MetahubAboutdialog::critical(QWidget * parent,
	const QString & title, const QString & text)
{
	setParent(parent);

	CS::Widgets::TitleBar *pTitleBar = getTitleBar();
	pTitleBar->setTitle(title);
	
	m_dptr->m_pVirsonLabel->setText(tr("Version:")+text);
	m_dptr->m_pContentIconButton->setIcon(ICore::resourceThemeImage("logo@2x.png"));

	return this->exec();
}
int CS::MetahubWidgets::MetahubAboutdialog::critical(const QString & text)
{
	return critical(nullptr, tr("About"), text);
}


#include "metahubabout.moc"