#include "outdoorwindowdialog.h"
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <qvariant.h>
#include<QDebug>
#include<QRadioButton>
#include<QApplication>
#define OUTDOORBUTTONSIZELENGTH 130  //box控件长度
#define OUTDOORBUTTONSIZEWIDTH 50   //box控件高度
using namespace SortOutdoor;
namespace SortOutdoor {



	class OutdoorClassifiedWindowPrivate : public QObject
	{
		Q_OBJECT
	public:
		OutdoorClassifiedWindowPrivate(OutdoorClassifiedWindow *ptr);
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
		* @brief 获取box选择
		*/
		void getCheckBox();
		/**
		* @brief 对外输出
		*/

	private slots:

		void slotUIOKButton(void);
		
	public:
		friend class OutdoorClassifiedWindow;
		OutdoorClassifiedWindow *m_qptr = nullptr;
		std::vector<enFeatureDetect> m_listFeatureDetect = {};

	private:
		QCheckBox               *m_pGroundCheckBox = nullptr;
		QCheckBox               *m_pTreesCheckBox = nullptr;
        QCheckBox               *m_pOtherCheckBox = nullptr;

        QRadioButton            *m_pLevelGroundButton = nullptr;
        QRadioButton            *m_pGentleSlopeButton = nullptr;
        QRadioButton            *m_pSteepSlopeButton = nullptr;

        QDoubleSpinBox               *m_pResolutionSpinBox = nullptr;
        QDoubleSpinBox               *m_pThresholdSpinBox = nullptr;

        QLabel                  *m_pGroundTypeLabel = nullptr;
        QLabel                  *m_pResolutionLabel = nullptr;
        QLabel                  *m_pThresholdLabel = nullptr;

	};	
}

OutdoorClassifiedWindowPrivate::OutdoorClassifiedWindowPrivate(OutdoorClassifiedWindow *ptr)
{
	m_qptr = ptr;
    return;
}

void OutdoorClassifiedWindowPrivate::createWidget(void)
{
    //[!].设置标题
    m_qptr->setMaximumSize(700, 289);
    m_qptr->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    //初始化
    m_pGroundCheckBox = new QCheckBox;
    m_pTreesCheckBox = new QCheckBox;
    m_pOtherCheckBox = new QCheckBox;

    m_pLevelGroundButton = new QRadioButton;
    m_pGentleSlopeButton = new QRadioButton;
    m_pSteepSlopeButton = new QRadioButton;

    m_pResolutionSpinBox = new QDoubleSpinBox;
    m_pThresholdSpinBox = new QDoubleSpinBox;

    m_pGroundTypeLabel = new QLabel;
    m_pResolutionLabel = new QLabel;
    m_pThresholdLabel = new QLabel;

	//[!].设置box大小
    m_pOtherCheckBox->setFixedSize(OUTDOORBUTTONSIZELENGTH, OUTDOORBUTTONSIZEWIDTH);
	m_pTreesCheckBox->setFixedSize(OUTDOORBUTTONSIZELENGTH, OUTDOORBUTTONSIZEWIDTH);

	//地面
	QVBoxLayout *pGroundVLayout = new QVBoxLayout();
    
    m_pLevelGroundButton->setChecked(true);
    m_pGroundCheckBox->setChecked(true);
    pGroundVLayout->addWidget(m_pGroundCheckBox);
    QLabel *pLine = new QLabel;
    pLine->setFixedHeight(2);
    pLine->setStyleSheet("border: 1px solid #585858;");
    pGroundVLayout->setSpacing(9);
    pGroundVLayout->addWidget(pLine);
    //类型选择
    QHBoxLayout *pTypeHLayout = new QHBoxLayout(); 

    QHBoxLayout *pRadioButtonLayout = new QHBoxLayout;
    pRadioButtonLayout->setSpacing(42);
    pRadioButtonLayout->addWidget(m_pLevelGroundButton);
    pRadioButtonLayout->addWidget(m_pGentleSlopeButton);
    pRadioButtonLayout->addWidget(m_pSteepSlopeButton);
    m_pGroundTypeLabel->setFixedHeight(20);
    pTypeHLayout->setContentsMargins(22, 16, 0, 0);
    pTypeHLayout->setSpacing(32);
    pTypeHLayout->addWidget(m_pGroundTypeLabel);
    pTypeHLayout->addLayout(pRadioButtonLayout);
    pTypeHLayout->addStretch();
    m_qptr->m_OutDoorDataMap.insert("GROUNDTYPE", "3");
    //布料分辨率
    QHBoxLayout *pParameterHLayout = new QHBoxLayout();

    QHBoxLayout *pResolutionLayout = new QHBoxLayout;
    QHBoxLayout *pThresholdLayout = new QHBoxLayout;
    
    m_pResolutionSpinBox->setMaximum(50);
    m_pResolutionSpinBox->setMinimum(0.1);
    m_pResolutionSpinBox->setValue(2.0); 
    m_qptr->m_OutDoorDataMap.insert("RESOLUTION", "2.0");
    m_pResolutionSpinBox->setFixedSize(152, 32);
    m_pResolutionSpinBox->setSingleStep(0.1);
    m_pResolutionSpinBox->setStyleSheet(QString::fromUtf8("QDoubleSpinBox\n"
        "{\n"
        "	color: #ffffff;\n"
        "	background-color: #343336;\n"
        "	border:1px solid #989898;\n"
        "	padding-left:9px;\n"
        "	border-radius: 2px;\n"
        "	min-height:32px;\n"
        "	max-height:32px;\n"
        "}\n"
        "QDoubleSpinBox:disabled\n"
        "{\n"
        "	color: #999999;\n"
        "	background-color: #3F3F3F;\n"
        "}"));
    //分类阈值
    m_pThresholdSpinBox->setMaximum(10);
    m_pThresholdSpinBox->setMinimum(0.1);
    m_pThresholdSpinBox->setProperty("doublespinbox", true);
    m_pThresholdSpinBox->setValue(0.5);
    m_qptr->m_OutDoorDataMap.insert("THRESHOLD", "0.5");
    m_pThresholdSpinBox->setSingleStep(0.1);
    m_pThresholdSpinBox->setFixedSize(152, 32);
    m_pThresholdSpinBox->setStyleSheet(QString::fromUtf8("QDoubleSpinBox\n"
        "{\n"
        "	color: #ffffff;\n"
        "	background-color: #343336;\n"
        "	border:1px solid #989898;\n"
        "	padding-left:9px;\n"
        "	border-radius: 2px;\n"
        "	min-height:32px;\n"
        "	max-height:32px;\n"
        "}\n"
        "QDoubleSpinBox:disabled\n"
        "{\n"
        "	color: #999999;\n"
        "	background-color: #3F3F3F;\n"
        "}"));

    
    pThresholdLayout->setSpacing(18);
    pThresholdLayout->addWidget(m_pThresholdLabel);
    pThresholdLayout->addWidget(m_pThresholdSpinBox);
    pResolutionLayout->setSpacing(18);
    pResolutionLayout->addWidget(m_pResolutionLabel);
    pResolutionLayout->addWidget(m_pResolutionSpinBox);
    pParameterHLayout->setSpacing(40);
    pParameterHLayout->setContentsMargins(22, 18, 2, 0);
    pParameterHLayout->addLayout(pResolutionLayout);
    pParameterHLayout->addLayout(pThresholdLayout);
    pParameterHLayout->addStretch();


	QVBoxLayout *m_pMainHBoxLayout = new QVBoxLayout();
	m_pMainHBoxLayout->setContentsMargins(16, 15, 16, 16);
    m_pMainHBoxLayout->addLayout(pGroundVLayout);
    m_pMainHBoxLayout->addLayout(pTypeHLayout);
    m_pMainHBoxLayout->addLayout(pParameterHLayout);
    m_pMainHBoxLayout->addStretch();
//    m_pMainHBoxLayout->addWidget(m_pTreesCheckBox);
//    m_pMainHBoxLayout->addWidget(m_pOtherCheckBox);
	m_qptr->setLayout(m_pMainHBoxLayout);

	return;
}

void OutdoorClassifiedWindowPrivate::createConnect(void)
{
	connect(m_qptr->getOKButton(), &QPushButton::clicked, 
		this, &OutdoorClassifiedWindowPrivate::slotUIOKButton);
    connect(m_pGroundCheckBox, &QCheckBox::stateChanged, this, [=]()
    {
        if (m_pGroundCheckBox->isChecked())
        {
            m_pLevelGroundButton->setEnabled(true);
            m_pGentleSlopeButton->setEnabled(true);
            m_pSteepSlopeButton->setEnabled(true);
            m_pResolutionSpinBox->setEnabled(true);
            m_pThresholdSpinBox->setEnabled(true);
        }
        else
        {
            m_pLevelGroundButton->setEnabled(false);
            m_pGentleSlopeButton->setEnabled(false);
            m_pSteepSlopeButton->setEnabled(false);
            m_pResolutionSpinBox->setEnabled(false);
            m_pThresholdSpinBox->setEnabled(false);
        }
    });
}

void OutdoorClassifiedWindowPrivate::retranslateUi()
{
    m_pOtherCheckBox->setText(QApplication::translate("outdoorwindowdialog", "Other", nullptr));
	m_pTreesCheckBox->setText(QApplication::translate("outdoorwindowdialog", "Trees", nullptr));
    m_pGroundCheckBox->setText(QApplication::translate("outdoorwindowdialog", "Ground", nullptr));
    m_pLevelGroundButton->setText(QApplication::translate("outdoorwindowdialog", "Flat", nullptr));
    m_pGentleSlopeButton->setText(QApplication::translate("outdoorwindowdialog", "Gentle slope", nullptr));
    m_pSteepSlopeButton->setText(QApplication::translate("outdoorwindowdialog", "Steep slope", nullptr));
    m_pGroundTypeLabel->setText(QApplication::translate("outdoorwindowdialog", "Ground type", nullptr));
    m_pResolutionLabel->setText(QApplication::translate("outdoorwindowdialog", "Extraction accuracy", nullptr));
    m_pThresholdLabel->setText(QApplication::translate("outdoorwindowdialog", "Ground Thickness", nullptr));
	m_qptr->setWindowTitle(QApplication::translate("outdoorwindowdialog", "Outdoor classification", nullptr));
}

void OutdoorClassifiedWindowPrivate::slotUIOKButton(void)
{
	m_listFeatureDetect.clear();

	//[!].其他提取
	if (m_pOtherCheckBox->isChecked()) {
		m_listFeatureDetect.push_back(enFeatureDetect::Other);
	}

	//[!].树提取
	if (m_pTreesCheckBox->isChecked()) {
		m_listFeatureDetect.push_back(enFeatureDetect::Trees);
        m_qptr->m_OutDoorDataMap.insert("TREES", "1");
	}
    else
    {
        m_qptr->m_OutDoorDataMap.insert("TREES", "0");
    }
    //[!]地面提取
    if (m_pGroundCheckBox->isChecked())
    {
        m_listFeatureDetect.push_back(enFeatureDetect::Ground);
        m_qptr->m_OutDoorDataMap.insert("GROUND", "1");
    }
    else
    {
        m_qptr->m_OutDoorDataMap.insert("GROUND", "0");
    }
    if (m_pLevelGroundButton->isChecked())
    {
         m_qptr->m_OutDoorDataMap.insert("GROUNDTYPE", "3");
    }
    if (m_pGentleSlopeButton->isChecked())
    {
        m_qptr->m_OutDoorDataMap.insert("GROUNDTYPE", "2");
    }
    if (m_pSteepSlopeButton->isChecked())
    {
        m_qptr->m_OutDoorDataMap.insert("GROUNDTYPE", "1");
    }
    connect(m_pResolutionSpinBox, &QDoubleSpinBox::textChanged, this, [=]()
    {
        m_qptr->m_OutDoorDataMap.insert("RESOLUTION", QString::number(m_pResolutionSpinBox->value()));
    });
    connect(m_pThresholdSpinBox, &QDoubleSpinBox::textChanged, this, [=]()
    {
        m_qptr->m_OutDoorDataMap.insert("THRESHOLD", QString::number(m_pThresholdSpinBox->value()));
    });
    
}



OutdoorClassifiedWindow::OutdoorClassifiedWindow(QWidget *parent)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent)
	, m_dptr(new OutdoorClassifiedWindowPrivate(this))
{
	//[!].创建UI界面
	m_dptr->createWidget();
	//[!].初始化翻译
	m_dptr->retranslateUi();
	//[!].创建连接
	m_dptr->createConnect();
}

OutdoorClassifiedWindow::~OutdoorClassifiedWindow()
{

}

std::vector<enFeatureDetect> SortOutdoor::OutdoorClassifiedWindow::getSelectFentrueDetect(void)
{
	return m_dptr->m_listFeatureDetect;
}


QMap<QString, QString> OutdoorClassifiedWindow::getOutExtractData()
{
    int currentType = 3;
    if (m_dptr->m_pLevelGroundButton->isChecked())
    {
        m_OutDoorDataMap.insert("GROUNDTYPE", "3");
    }
    else if (m_dptr->m_pGentleSlopeButton->isChecked())
    {
        m_OutDoorDataMap.insert("GROUNDTYPE", "2");
    }
    else if (m_dptr->m_pSteepSlopeButton->isChecked())
    {
        m_OutDoorDataMap.insert("GROUNDTYPE", "1");
    }
    m_OutDoorDataMap.insert("RESOLUTION", QString::number(m_dptr->m_pResolutionSpinBox->value()));
    m_OutDoorDataMap.insert("THRESHOLD", QString::number(m_dptr->m_pThresholdSpinBox->value()));
    return m_OutDoorDataMap;
}
#include "outdoorwindowdialog.moc"