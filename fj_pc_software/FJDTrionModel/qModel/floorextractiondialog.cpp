#include "floorextractiondialog.h"
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
#include <QSpinBox>
#include<QCoreApplication>
using namespace floorextraction;
namespace floorextraction {
	class FloorExtractiondialogPrivate : public QObject
	{
		Q_OBJECT
	public:
		FloorExtractiondialogPrivate(FloorExtractiondialog *ptr);
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
		* @brief 获取界面数据
		*/
		void getCheckBox();
	public:
		friend class FloorExtractiondialog;
		FloorExtractiondialog *m_qptr = nullptr;
	private slots:
		int slotUIOKButton();
	private:
		QHBoxLayout * m_pSuperiortitle;
		QHBoxLayout * m_pInputBox;
		QVBoxLayout *m_pMainGridLayout;
		QSpinBox *m_pFloorExtraction;
		QLabel   *m_pLabel;
        QLabel  *pTipsLabel;
	};	
}

FloorExtractiondialogPrivate::FloorExtractiondialogPrivate(FloorExtractiondialog *ptr)
{
	m_qptr = ptr;
}

void FloorExtractiondialogPrivate::createWidget()
{
	//[!].设置标题
	
	m_qptr->setFixedSize(400, 300);
	//[!].初始化box
	m_pFloorExtraction = new QSpinBox(m_qptr);
	m_pLabel = new QLabel(m_qptr);
	//[!]设置控件size
	m_pFloorExtraction->setFixedSize(320,32);
	m_pLabel->setFixedSize(300, 24);
//	m_pFloorExtraction->setStyleSheet("font: 12pt;");
//	m_pLabel->setStyleSheet("font: 12pt;");
	m_pFloorExtraction->setMinimum(0);
	m_pFloorExtraction->setMaximum(100);
	m_pFloorExtraction->setValue(60);
	m_pFloorExtraction->setSuffix("%");
	//[!].主布局
	m_pMainGridLayout = new QVBoxLayout();
	m_pMainGridLayout->setContentsMargins(16, 0, 16,0);
	//m_pMainGridLayout->setMargin(0);  
	m_pMainGridLayout->setSpacing(10); 
	m_pSuperiortitle = new QHBoxLayout();
	m_pInputBox      = new QHBoxLayout();
	m_pSuperiortitle->setContentsMargins(0, 9, 0, 0);
	m_pInputBox->setContentsMargins(0, 0, 0, 0);
	m_pSuperiortitle->addWidget(m_pLabel, 0, Qt::AlignLeft);
	m_pInputBox->addWidget(m_pFloorExtraction, 0, Qt::AlignLeft);
	m_pMainGridLayout->addLayout(m_pSuperiortitle);
	m_pMainGridLayout->addLayout(m_pInputBox);
    pTipsLabel = new QLabel(m_qptr);
    pTipsLabel->setFixedWidth(320);
    pTipsLabel->setWordWrap(true);
    m_pMainGridLayout->addWidget(pTipsLabel);
    m_pMainGridLayout->addStretch();
	m_qptr->setLayout(m_pMainGridLayout);
	return;
}

void FloorExtractiondialogPrivate::createConnect()
{
	connect(m_qptr->getOKButton(), &QPushButton::clicked, this, &FloorExtractiondialogPrivate::slotUIOKButton);
}
void FloorExtractiondialogPrivate::retranslateUi()
{

	m_qptr->setWindowTitle(QCoreApplication::translate("floorextractiondialog", "Floor extraction", nullptr));
	m_pLabel->setText(QCoreApplication::translate("floorextractiondialog", "Recursive Plane Point Proportion", nullptr));
    pTipsLabel->setText(QCoreApplication::translate("floorextractiondialog", "Note: The larger the parameter, the lower the detection rate of irregular floor.", nullptr));

	//m_qptr->setWindowTitle(tr("floor extraction"));
	//m_pLabel->setText(tr("Plane point recursion ratio"));
}
int FloorExtractiondialogPrivate::slotUIOKButton()
{
	return m_pFloorExtraction->value();
}

FloorExtractiondialog::FloorExtractiondialog(QWidget *parent)
	: MetahubFramelessDialog(parent)
	, m_dptr(new FloorExtractiondialogPrivate(this))
{
	//[!].创建UI界面
	m_dptr->createWidget();
	//[!].初始化翻译
	m_dptr->retranslateUi();
	//[!].创建连接
	m_dptr->createConnect();
}

FloorExtractiondialog::~FloorExtractiondialog()
{

}
int FloorExtractiondialog::getDialogData()
{
	return m_dptr->m_pFloorExtraction->value();
}

#include "floorextractiondialog.moc"