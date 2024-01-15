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
#include"framelesswaitdialog.h"
#include "cswidgets/framelesshelper.h"

using namespace CS::Widgets;
namespace CS {
	namespace Widgets {

		class FramelessWaitingdialogPrivate : public  QObject
		{
		public:
			explicit FramelessWaitingdialogPrivate(CS::Widgets::FramelessWaitingdialog *qptr);
			virtual ~FramelessWaitingdialogPrivate();
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
			QFrame		    *m_pFrame = nullptr;
			QVBoxLayout     *m_pCenterLayout = nullptr;
			QWidget			*m_pCentreContentWidget = nullptr;
			QLabel          *m_pWaitMoveLabel = nullptr;
			QLabel          *m_pWarningLabel = nullptr;
			QMovie			*m_pMovie = nullptr;
		private:
			friend class CS::Widgets::FramelessWaitingdialog;
			CS::Widgets::FramelessWaitingdialog *m_qptr = nullptr;

		};
	}

}
FramelessWaitingdialogPrivate::FramelessWaitingdialogPrivate(
	CS::Widgets::FramelessWaitingdialog *qptr)
{
	m_qptr = qptr;
}
FramelessWaitingdialogPrivate::~FramelessWaitingdialogPrivate()
{

}

void FramelessWaitingdialogPrivate::createWidget()
{
	m_qptr->setFixedSize(180, 190);
	QVBoxLayout *pContentHoderLayout = new QVBoxLayout();
	pContentHoderLayout->setContentsMargins(0, 0, 0, 0);
	pContentHoderLayout->setSpacing(0);
	m_pFrame = new QFrame();
	m_pFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pMovie = new QMovie(QApplication::applicationDirPath() + "/theme/qssimage/waiting.gif");
	m_pMovie->setSpeed(120);
	m_pMovie->setScaledSize(QSize(126, 126));
	m_pWaitMoveLabel = new QLabel();
	m_pWarningLabel = new QLabel();
	QHBoxLayout * m_pOnHLayout = new QHBoxLayout();
	QHBoxLayout * m_pDownHLayout = new QHBoxLayout();
	m_pDownHLayout->setContentsMargins(5, 0, 5, 0);
	m_pWarningLabel->setWordWrap(true);
	m_pWaitMoveLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pWaitMoveLabel->setFixedSize(QSize(126, 126));
	m_pWaitMoveLabel->setMovie(m_pMovie);
	m_pMovie->start();
	//[!].主布局
	m_pCenterLayout = new QVBoxLayout(m_pFrame);
	m_pOnHLayout->addWidget(m_pWaitMoveLabel, 0, Qt::AlignTop);
	m_pDownHLayout->addWidget(m_pWarningLabel, 0, Qt::AlignTop);
	m_pCenterLayout->setContentsMargins(0, 0, 0, 0);
	m_pCenterLayout->setSpacing(0);
	m_pCenterLayout->addLayout(m_pOnHLayout);
	m_pCenterLayout->addSpacing(5);
	m_pCenterLayout->addLayout(m_pDownHLayout);
	m_pCenterLayout->addStretch();
	m_pWaitMoveLabel->setStyleSheet("QLabel{border: 0px;}");
	m_pWarningLabel->setStyleSheet("QLabel{color: #F2F2F2;border: 0px;font-size: 14px;}");
	m_pWarningLabel->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
	m_pWarningLabel->setWordWrap(true);
	m_pFrame->setStyleSheet("QFrame{background-color: #2B2B2B;border-radius: 2px;border: 1px solid #585858;}");
	pContentHoderLayout->addWidget(m_pFrame);
	pContentHoderLayout->addLayout(m_pCenterLayout);
	m_qptr->setLayout(pContentHoderLayout);
}
void FramelessWaitingdialogPrivate::createConnect()
{

}
void FramelessWaitingdialogPrivate::retranslateUi()
{
	m_pWarningLabel->setText(QCoreApplication::translate("metahubframelesswaitdialog", "loading...", nullptr));
//	m_pWarningLabel->setText(tr("waiting..."));
}


FramelessWaitingdialog::FramelessWaitingdialog(QWidget *parent
	, Qt::WindowFlags f)
	:QDialog(parent, f)
	, m_dptr(new CS::Widgets::FramelessWaitingdialogPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();
	
}

FramelessWaitingdialog::~FramelessWaitingdialog()
{
	if (nullptr != m_dptr) {
		delete m_dptr;
	}
	m_dptr = nullptr;
}


void FramelessWaitingdialog::startWaiting(const QString& str)
{
    m_dptr->m_pWarningLabel->setText(str);
	setFixedSize(180, 190);
    setWindowFlags( Qt::FramelessWindowHint | Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setAttribute(Qt::WA_ShowModal, true);
    if (isHidden())
    {
        m_dptr->m_pMovie->start();
        QDialog::show();
    }
}

void FramelessWaitingdialog::jumpToNextFrame(void)
{
    int nCount = m_dptr->m_pMovie->frameCount();
    int nCountF = m_dptr->m_pMovie->currentFrameNumber();
    nCountF++;
    if (nCountF >= nCount)
    {
        nCountF = 0;
    }

    m_dptr->m_pMovie->jumpToFrame(nCountF);
    m_dptr->m_pWaitMoveLabel->repaint();
}
void FramelessWaitingdialog::stopWaiting()
{
	m_dptr->m_pMovie->stop();
    this->setParent(nullptr);
	QDialog::hide();
}

FramelessWaitingdialog* FramelessWaitingdialog::instance(QWidget *parent, Qt::WindowFlags f)
{
	static FramelessWaitingdialog instance(parent, f);
	return &instance;
}



void FramelessWaitingdialog::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Escape:
		break;
	default:
		QDialog::keyPressEvent(event);
	}
}

void FramelessWaitingdialog::setWaitdialogDisplay(QString str)
{
	m_dptr->m_pWarningLabel->setText(str);
}
