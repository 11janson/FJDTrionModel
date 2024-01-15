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
#include"metahubframelesswaitdialog.h"
#include "cswidgets/framelesshelper.h"

using namespace CS::Widgets;
using namespace WaitingDialog;

namespace WaitingDialog {
	
	class MetahublFramelessWaitingdialogPrivate : public  QObject
	{
		Q_OBJECT
	public:
		explicit MetahublFramelessWaitingdialogPrivate(WaitingDialog::MetahublFramelessWaitingdialog *qptr);
		virtual ~MetahublFramelessWaitingdialogPrivate();
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
		QVBoxLayout     *m_pCenterLayout = nullptr;
		QWidget			*m_pCentreContentWidget = nullptr;
		QLabel          *m_pWaitMoveLabel = nullptr;
		QLabel          *m_pWarningLabel = nullptr;
		QMovie			*m_pMovie=nullptr;
	private:
		friend class WaitingDialog::MetahublFramelessWaitingdialog;
		WaitingDialog::MetahublFramelessWaitingdialog *m_qptr = nullptr;

	};

   

}
MetahublFramelessWaitingdialogPrivate::MetahublFramelessWaitingdialogPrivate(
	WaitingDialog::MetahublFramelessWaitingdialog *qptr)
{
	m_qptr = qptr;
}
MetahublFramelessWaitingdialogPrivate::~MetahublFramelessWaitingdialogPrivate()
{

}

void MetahublFramelessWaitingdialogPrivate::createWidget()
{
	m_qptr->setFixedSize(228, 184);
	m_pMovie = new QMovie(QApplication::applicationDirPath() + "/theme/qssimage/waiting.gif");
	m_pMovie->setSpeed(120);
	m_pMovie->setScaledSize(QSize(102, 102));
	m_pWaitMoveLabel = new QLabel(m_qptr);
	m_pWarningLabel = new QLabel(m_qptr);
	m_pWarningLabel->setWordWrap(true);
	m_pWaitMoveLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pWaitMoveLabel->setFixedSize(QSize(102, 102));
	m_pWaitMoveLabel->setMovie(m_pMovie);
	m_pMovie->start();
	//[!].主布局
	m_pWaitMoveLabel->setStyleSheet("QLabel{border: 0px;}");
	m_pWarningLabel->setStyleSheet("QLabel{color: #F2F2F2;border: 0px;font-size: 16px;}");
	m_pWarningLabel->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
	m_pWarningLabel->setWordWrap(true);

    QHBoxLayout *onHLavout = new QHBoxLayout;
    onHLavout->addWidget(m_pWaitMoveLabel);
    onHLavout->setContentsMargins(62, 20, 62, 0);
    QHBoxLayout *downHLayout = new QHBoxLayout;
    downHLayout->addWidget(m_pWarningLabel);
    downHLayout->setContentsMargins(16, 0, 16, 0);
    m_pCenterLayout = new QVBoxLayout;
    m_pCenterLayout->addLayout(onHLavout);
    m_pCenterLayout->addLayout(downHLayout);
    m_pCenterLayout->setMargin(0);
    m_pCenterLayout->addStretch();
    m_qptr->setStyleSheet("QWidget{background-color:#2B2B2B;border:1px solid #585858;border-radius:2px;}");
	m_qptr->setLayout(m_pCenterLayout);
}
void MetahublFramelessWaitingdialogPrivate::createConnect()
{

}
void MetahublFramelessWaitingdialogPrivate::retranslateUi()
{
	m_pWarningLabel->setText(QCoreApplication::translate("metahubframelesswaitdialog", "loading...", nullptr));
//	m_pWarningLabel->setText(tr("waiting..."));
}


MetahublFramelessWaitingdialog::MetahublFramelessWaitingdialog(QWidget *parent
	, Qt::WindowFlags f)
	:QDialog(parent, f)
	, m_dptr(new WaitingDialog::MetahublFramelessWaitingdialogPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();
	
}

MetahublFramelessWaitingdialog::~MetahublFramelessWaitingdialog()
{
	if (nullptr != m_dptr) {
		delete m_dptr;
	}
	m_dptr = nullptr;
}


void MetahublFramelessWaitingdialog::startWaiting(const QString& str)
{
    m_dptr->m_pWarningLabel->setText(str);
    setWindowFlags( Qt::FramelessWindowHint | Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setAttribute(Qt::WA_ShowModal, true);
    if (isHidden())
    {
        m_dptr->m_pMovie->start();
        QDialog::show();
    }
}

void MetahublFramelessWaitingdialog::jumpToNextFrame(void)
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
void MetahublFramelessWaitingdialog::stopWaiting()
{
	m_dptr->m_pMovie->stop();
    this->setParent(nullptr);
	QDialog::hide();
}

MetahublFramelessWaitingdialog* MetahublFramelessWaitingdialog::instance(QWidget *parent, Qt::WindowFlags f)
{
	static MetahublFramelessWaitingdialog instance(parent, f);
	return &instance;
}

void MetahublFramelessWaitingdialog::showEvent(QShowEvent *event)
{
	QDialog::raise();
	QDialog::activateWindow();
	QDialog::showEvent(event);
}

void MetahublFramelessWaitingdialog::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Escape:
		break;
	default:
		QDialog::keyPressEvent(event);
	}


}

void MetahublFramelessWaitingdialog::setWaitdialogDisplay(QString str)
{
	m_dptr->m_pWarningLabel->setText(str);
}
#include "metahubframelesswaitdialog.moc"