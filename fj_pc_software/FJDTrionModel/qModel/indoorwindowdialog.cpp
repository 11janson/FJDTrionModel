#include "indoorwindowdialog.h"
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QComboBox> 
#include <QCheckBox>
#include <qvariant.h>
#include <QRegExpValidator>
#include<QDebug>
#include<QCoreApplication>
#define InDOORBUTTONSIZELENGTH 75  //box控件长度
#define InDOORBUTTONSIZEWIDTH 32   //box控件高度
using namespace SortIndoor;
namespace SortIndoor {
	class IndoorClassifiedWindowPrivate : public QObject
	{
		Q_OBJECT
	public:
		IndoorClassifiedWindowPrivate(IndoorClassifiedWindow *ptr);
	public:

		/**
		* @brief crateWidget 创建UI界面
		*/
		void createWidget(void);
		/**
		* @brief crateConnect 创建信号连接
		*/
		void createConnect(void);
		/**
		* @brief 切换翻译
		*/
		void retranslateUi();
		/**
		* @brief 获取checkbox状态
		*/
		void getCheckBox();

	public:
		friend class IndoorClassifiedWindow;
		IndoorClassifiedWindow *m_qptr = nullptr;
		std::vector<enFeatureDetect> m_listFeatureDetect = {};
	private slots:
		void slotUIOKButton();
	private:
		QCheckBox               *m_pCeilingCheckBox = nullptr;
		QCheckBox               *m_pFloorCheckBox = nullptr;
		QCheckBox               *m_pWallCheckBox = nullptr;
		//QCheckBox               *m_pOtherCheckBox = nullptr;

	};

}

IndoorClassifiedWindowPrivate::IndoorClassifiedWindowPrivate(IndoorClassifiedWindow *ptr)
{
	m_qptr = ptr;
	return;
}

void IndoorClassifiedWindowPrivate::createWidget(void)
{
	//[!].设置标题
	m_qptr->setWindowTitle(tr("Indoor classification extraction"));
	m_qptr->setFixedSize(400, 300);
	//[!].初始化button
	m_pCeilingCheckBox = new QCheckBox(m_qptr);

	m_pFloorCheckBox = new QCheckBox(m_qptr);

	m_pWallCheckBox = new QCheckBox(m_qptr);

	//m_pOtherCheckBox = new QCheckBox(m_qptr);
	//	m_pOtherCheckBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

		//[!].设置box大小
	m_pCeilingCheckBox->setFixedSize(InDOORBUTTONSIZELENGTH, InDOORBUTTONSIZEWIDTH);
	m_pFloorCheckBox->setFixedSize(InDOORBUTTONSIZELENGTH, InDOORBUTTONSIZEWIDTH);
	m_pWallCheckBox->setFixedSize(InDOORBUTTONSIZELENGTH, InDOORBUTTONSIZEWIDTH);
	//m_pOtherCheckBox->setFixedSize(InDOORBUTTONSIZELENGTH, InDOORBUTTONSIZEWIDTH);

//	m_pCeilingCheckBox->setStyleSheet("font: 12pt;}");
//	m_pFloorCheckBox->setStyleSheet("font: 12pt;}");
//	m_pWallCheckBox->setStyleSheet("font: 12pt;}");
//	m_pOtherCheckBox->setStyleSheet("font: 12pt;}");

	QHBoxLayout *m_pOneHLaylog = new  QHBoxLayout();
	QHBoxLayout *m_pTwoHLaylog = new  QHBoxLayout();
	QVBoxLayout *m_pMainVLaylog = new  QVBoxLayout();
	
//	m_pMainGridLayout->setSpacing(0);
//	m_pMainGridLayout->setMargin(24);
	m_pOneHLaylog->setContentsMargins(16, 0, 16, 0);
	m_pTwoHLaylog->setContentsMargins(16, 0, 16, 0);
	m_pOneHLaylog->addWidget(m_pWallCheckBox,0,Qt::AlignLeft);
	m_pOneHLaylog->addWidget(m_pFloorCheckBox,0,Qt::AlignHCenter);
	m_pOneHLaylog->addWidget(m_pCeilingCheckBox,0,Qt::AlignRight);
	//m_pTwoHLaylog->addWidget(m_pOtherCheckBox,0,Qt::AlignLeft);

	m_pMainVLaylog->setContentsMargins(0, 0, 0, 0);
	m_pMainVLaylog->addLayout(m_pOneHLaylog);
	m_pMainVLaylog->addLayout(m_pTwoHLaylog);
	m_qptr->setLayout(m_pMainVLaylog);
	return;
}

void IndoorClassifiedWindowPrivate::createConnect(void)
{
	connect(m_qptr->getOKButton(), &QPushButton::clicked, this, &IndoorClassifiedWindowPrivate::slotUIOKButton);
}

void IndoorClassifiedWindowPrivate::retranslateUi()
{
	m_qptr->setWindowTitle(QCoreApplication::translate("indoorwindowdialog", "Indoor Classification", nullptr));
	m_pCeilingCheckBox->setText(QCoreApplication::translate("indoorwindowdialog", "Ceiling", nullptr));
	m_pFloorCheckBox->setText(QCoreApplication::translate("indoorwindowdialog", "Floor", nullptr));
	m_pWallCheckBox->setText(QCoreApplication::translate("indoorwindowdialog", "Wall", nullptr));
	//m_pOtherCheckBox->setText(QCoreApplication::translate("indoorwindowdialog", "Other", nullptr));
	//m_pCeilingCheckBox->setText(tr("ceiling"));
	//m_pFloorCheckBox->setText(tr("floor"));
	//m_pWallCheckBox->setText(tr("wall"));
	//m_pOtherCheckBox->setText(tr("other"));
}
void IndoorClassifiedWindowPrivate::slotUIOKButton()
{
	if (m_pCeilingCheckBox->isChecked())
	{
		m_listFeatureDetect.push_back(enFeatureDetect::ceiling);
	}
	if (m_pFloorCheckBox->isChecked())
	{
		m_listFeatureDetect.push_back(enFeatureDetect::floor);
	}
	if (m_pWallCheckBox->isChecked())
	{
		m_listFeatureDetect.push_back(enFeatureDetect::wall);
	}
	//if (m_pOtherCheckBox->isChecked())
	//{
	//	m_listFeatureDetect.push_back(enFeatureDetect::other);
	//}
}
IndoorClassifiedWindow::IndoorClassifiedWindow(QWidget *parent)
	: MetahubFramelessDialog(parent)
	, m_dptr(new IndoorClassifiedWindowPrivate(this))
{
	//[!].创建UI界面
	m_dptr->createWidget();
	//[!].初始化翻译
	m_dptr->retranslateUi();
	//[!].创建连接
	m_dptr->createConnect();

}

IndoorClassifiedWindow::~IndoorClassifiedWindow()
{

}
std::vector<enFeatureDetect> SortIndoor::IndoorClassifiedWindow::getSelectFentrueDetect(void)
{
	return m_dptr->m_listFeatureDetect;
}

#include "indoorwindowdialog.moc"