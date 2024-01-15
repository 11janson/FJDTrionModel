#include "ipropertycenterwidget.h"
#include <QVBoxLayout>
#include <QFrame>
#include "frameline.h"
#include <QHeaderView>
#include <QMessageBox>
#include "cswidgets/framelessmessagebox.h"


using namespace CS;
using namespace CS::Widgets;
IPropertyCenterWidget::IPropertyCenterWidget(Utils::Id id, QWidget *parent)
	:ICenterWidget(id, parent)
{
	createWidgets();
	createConnects();
	retranslateUi();
}
IPropertyCenterWidget::~IPropertyCenterWidget()
{

}


void CS::Widgets::IPropertyCenterWidget::retranslateUi()
{
	ICenterWidget::retranslateUi();
	QStringList headers;
	headers << tr("Part type") << tr("Serial number") << tr("Welding feature");
	m_pDetailContentTabel->setHorizontalHeaderLabels(headers);
    QStringList listLabelStr;
    listLabelStr << tr("DepthDownSampling:") << tr("ConnectGradThreshold:") << tr("ConnectAreaRatioMin:") << \
        tr("GradFilterRatio:") << tr("SampleRatio:") << tr("FilterCountRatio:") << tr("IsolationRate:") << \
        tr("IsolationNeighborCount:") << tr("IsolationAngleDegree:") << tr("GeometrySmoothCount:") << \
        tr("GeometrySmoothRadius:") << tr("NormalSmoothCount:") << tr("NormalSmoothRadius:") << \
        tr("BehindDepthThreshod:");
    for (size_t i = 0; i < listLabelStr.size(); i++)
    {
        m_pLabelMap[i]->setText(listLabelStr[i]);
    }
    m_pPageSwitchButton->setText(tr("SeniorParam"));
    m_pAlterParamButton->setText(tr("ModifyParam"));
    m_pRevertParamButton->setText(tr("RevertParam"));
}


void IPropertyCenterWidget::setCaptionName(QString strName)
{
	m_pDisplayNameLabel->setText(strName);
}


void IPropertyCenterWidget::setDetailDescription(QStringList list)
{
	m_pDetailTextEdit->clear();
	for (auto Description : list) {
		m_pDetailTextEdit->append(Description);
	}
	return;
}

void IPropertyCenterWidget::setSpinBoxProperty(int nId, int nDecimals, double nMin, double nMax, double SingleStep)
{
    m_pSpinBoxMap[nId]->setDecimals(nDecimals);
    m_pSpinBoxMap[nId]->setRange(nMin, nMax);
    m_pSpinBoxMap[nId]->setSingleStep(SingleStep);
}

void IPropertyCenterWidget::createBasisParam()
{
    int totalNum = 0;
    QGridLayout *pMainHBoxLayout = new QGridLayout;
    for (int i = 0; i<14; i++)
    {
        QLabel *pCurrentLabel = new QLabel();
        pCurrentLabel->setAlignment(Qt::AlignRight);
        pCurrentLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        pCurrentLabel->setMinimumWidth(100);
        pCurrentLabel->setMaximumWidth(160);
        m_pLabelMap.insert(i, pCurrentLabel);
        QDoubleSpinBox *pCurrentSpinBox = new QDoubleSpinBox();
        pCurrentSpinBox->setProperty("iproperty", true);
        pCurrentSpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        pCurrentSpinBox->setMaximumWidth(80);
        pCurrentSpinBox->setFixedHeight(26);
        m_pSpinBoxMap.insert(i, pCurrentSpinBox);
    }

    int coloums = 0,rows = 0;
    for (int i = 0; i < m_pSpinBoxMap.size(); i++) {
        if (rows > 4) {
            coloums = coloums + 1;
            rows = 0;
        }
        pMainHBoxLayout->addWidget(m_pLabelMap[i], coloums, rows);
        pMainHBoxLayout->addWidget(m_pSpinBoxMap[i], coloums, rows + 1);
        rows = rows + 2;
    }

    setSpinBoxProperty(0, 0, 0, 1, 1);
    setSpinBoxProperty(1, 0, -10000, 10000, 1);
    setSpinBoxProperty(2, 3, 0, 1, 0.001);

    setSpinBoxProperty(3, 2, 0, 1, 0.01);
    setSpinBoxProperty(4, 2, 0, 1, 0.01);
    setSpinBoxProperty(5, 3, 0, 1, 0.001);
    setSpinBoxProperty(6, 2, 0, 1, 0.01);
    setSpinBoxProperty(7, 0, 0, 1000, 1);
    setSpinBoxProperty(8, 0, 0, 90, 1);
    setSpinBoxProperty(9, 0, 0, 100, 1);
    setSpinBoxProperty(10, 0, 0, 100, 1);
    setSpinBoxProperty(11, 0, 0, 100, 1);
    setSpinBoxProperty(12, 0, 0, 100, 1);
    setSpinBoxProperty(13, 1, -1000, 1000, 0.1);

	m_pSecondTabWidget->setLayout(pMainHBoxLayout);
}


/**
*@brief 更新SpainBox值
*/
void IPropertyCenterWidget::updataSpainBoxFromParam(QStringList listParam)
{
    for (size_t i = 0; i < listParam.size(); i++)
    {
        m_pSpinBoxMap[i]->setValue(listParam[i].toFloat());
    }
}

/**
*@brief 更新参数值
*/
QStringList IPropertyCenterWidget::updataParamFromSpainBox()
{
    QStringList listParam;
    for (size_t i = 0; i < m_pSpinBoxMap.size(); i++)
    {
        listParam.append(QString::number(m_pSpinBoxMap[i]->value()));
    }
    return listParam;
}

void CS::Widgets::IPropertyCenterWidget::createWidgets()
{
	QHBoxLayout *pMainHBoxLayout = new QHBoxLayout();
	pMainHBoxLayout->setContentsMargins(0, 0, 0, 0);
	pMainHBoxLayout->setSpacing(0);
	
	//[!].左边
	QFrame *pLeftContentFrame = new QFrame();
	pLeftContentFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pLeftContentFrame->setMinimumWidth(300);
	QVBoxLayout *pLeftContentVBoxLayout = new QVBoxLayout(pLeftContentFrame);
	pLeftContentVBoxLayout->setContentsMargins(0, 5, 0, 0);
	pMainHBoxLayout->addWidget(pLeftContentFrame);
	pMainHBoxLayout->addWidget(CS::Widgets::FrameLine::createVerticalLine("frameLine"));
	//[!].右边
	QFrame *pRightContentFrame = new QFrame();
	pRightContentFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pRightContentFrame->setMinimumWidth(300);
	QVBoxLayout *pRightContentVBoxLayout = new QVBoxLayout(pRightContentFrame);
	pRightContentVBoxLayout->setContentsMargins(0, 0, 0, 0);
	pRightContentVBoxLayout->setSpacing(0);
	pMainHBoxLayout->addWidget(pRightContentFrame);
	pMainHBoxLayout->addWidget(CS::Widgets::FrameLine::createVerticalLine("frameLine"));
	
	m_pContextStackedWidget = new QStackedWidget();
	m_pContextStackedWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pContextStackedWidget->setMaximumHeight(200);
	//ADD First Tab
	m_pFirstTabWidget = new QWidget();	
	QHBoxLayout *pFirstTabHBoxLayout = new QHBoxLayout(m_pFirstTabWidget);
	pFirstTabHBoxLayout->setContentsMargins(0, 2, 0, 0);	
	
	m_pShowParamPicLabel = new QLabel();
	m_pShowParamPicLabel->setProperty("ShowParamPic", true);
	m_pShowParamPicLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pFirstTabHBoxLayout->addWidget(m_pShowParamPicLabel);

	m_pSetParamWidget = new QWidget();
	m_pSetParamWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pFirstTabHBoxLayout->addWidget(m_pSetParamWidget);

	m_pFirstTabWidget->setLayout(pFirstTabHBoxLayout);
	m_pContextStackedWidget->addWidget(m_pFirstTabWidget);

	//ADD Second Tab
	m_pSecondTabWidget = new QWidget();	
	createBasisParam();
	m_pContextStackedWidget->addWidget(m_pSecondTabWidget);
	pRightContentVBoxLayout->addWidget(m_pContextStackedWidget, 0, 0);

	QHBoxLayout *pButtonVBoxLayout = new QHBoxLayout();
	pButtonVBoxLayout->addSpacerItem(new QSpacerItem(20, 20, QSizePolicy::Expanding));
	m_pAlterParamButton = new QPushButton();
	m_pAlterParamButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pAlterParamButton->setMaximumSize(110, 28);
	m_pAlterParamButton->setToolTip(tr("Determine whether to modify the parameters"));
	pButtonVBoxLayout->addWidget(m_pAlterParamButton);

    m_pRevertParamButton = new QPushButton();
    m_pRevertParamButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_pRevertParamButton->setMaximumSize(110, 28);
    m_pRevertParamButton->setToolTip(tr("Reset the parameter to the initial value"));
    pButtonVBoxLayout->addWidget(m_pRevertParamButton); 

	m_pPageSwitchButton = new QPushButton();
	m_pPageSwitchButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pPageSwitchButton->setMaximumSize(110,28);
	pButtonVBoxLayout->addWidget(m_pPageSwitchButton);
	pRightContentVBoxLayout->addLayout(pButtonVBoxLayout);
	pButtonVBoxLayout->addSpacerItem(new QSpacerItem(10, 20, QSizePolicy::Fixed));
	pRightContentVBoxLayout->addWidget(CS::Widgets::FrameLine::createHorizontalLine("frameLine"));
	m_pPageSwitchButton->setToolTip(tr("chang basis param and senior param"));	
	
	m_pDetailContentTabel = new QTableWidget();
    m_pDetailContentTabel->setProperty("DetailContent", true);
	m_pDetailContentTabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pRightContentVBoxLayout->addWidget(m_pDetailContentTabel);
	//m_pDetailContentTabel->setFrameShape(QFrame::Panel);//设置边框
	m_pDetailContentTabel->setColumnCount(3);
	m_pDetailContentTabel->setRowCount(18);
	m_pDetailContentTabel->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_pDetailContentTabel->verticalHeader()->setDefaultSectionSize(160);
	m_pDetailContentTabel->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_pDetailContentTabel->setEditTriggers(QAbstractItemView::NoEditTriggers);
	m_pDetailContentTabel->horizontalHeader()->setStretchLastSection(true); 
	m_pDetailContentTabel->horizontalHeader()->setMovable(true);
	m_pDetailContentTabel->verticalHeader()->setHidden(true);
	m_pDetailContentTabel->horizontalHeader()->setMinimumHeight(40);
	m_pDetailContentTabel->horizontalHeader()->setMaximumHeight(40);
	m_pDetailContentTabel->horizontalHeader()->setMinimumWidth(200);
	m_pDetailContentTabel->setColumnWidth(0, 200);
	m_pDetailContentTabel->setColumnWidth(1, 200);

	//[!].焊接包主标题
	m_pDisplayNameLabel = new QLabel();
	m_pDisplayNameLabel->setProperty("welderCaption", true);
	m_pDisplayNameLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_pDisplayNameLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pDisplayNameLabel->setFixedHeight(45);
	pLeftContentVBoxLayout->addWidget(m_pDisplayNameLabel);
	//[!].详情描述
	m_pDetailTextEdit = new QTextEdit();
	m_pDetailTextEdit->setReadOnly(true);
	m_pDetailTextEdit->setProperty("welderDetail", true);
	m_pDetailTextEdit->setAlignment(Qt::AlignTop | Qt::AlignLeft);
	m_pDetailTextEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_pDetailTextEdit->setMinimumHeight(80);
	m_pDetailTextEdit->setMaximumHeight(180);
	pLeftContentVBoxLayout->addWidget(m_pDetailTextEdit);
	pLeftContentVBoxLayout->addWidget(CS::Widgets::FrameLine::createHorizontalLine("frameLine"));

	//[!].内容区域
	m_pWeldElaborateWidget = new QWidget();
	m_pWeldElaborateWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	QHBoxLayout *pHBoxLayout = new QHBoxLayout(m_pWeldElaborateWidget);
	pHBoxLayout->setContentsMargins(20, 20, 20, 20);

	m_pThumbnailButton = new QPushButton(); 
	m_pThumbnailButton->setAttribute(Qt::WA_TransparentForMouseEvents, true);
	m_pThumbnailButton->setCheckable(false);
	m_pThumbnailButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
	m_pThumbnailButton->setProperty("nonebutton", true);
	m_pThumbnailButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pThumbnailButton->setIconSize(QSize(600, 600));
	pHBoxLayout->addWidget(m_pThumbnailButton);
	pLeftContentVBoxLayout->addWidget(m_pWeldElaborateWidget);
	setLayout(pMainHBoxLayout);
}

void CS::Widgets::IPropertyCenterWidget::createConnects()
{
	connect(m_pPageSwitchButton, &QPushButton::clicked, this, &IPropertyCenterWidget::changeParamInfo);
	connect(m_pAlterParamButton, &QPushButton::clicked, this, &IPropertyCenterWidget::confirmModifyParam);
    connect(m_pRevertParamButton, &QPushButton::clicked, this, &IPropertyCenterWidget::revertParam);
	connect(this, SIGNAL(signalModifyParam()), this, SLOT(slotModifyParam()));
    connect(this, SIGNAL(signalRevertParam()), this, SLOT(slotRevertParam()));
}


void CS::Widgets::IPropertyCenterWidget::changeParamInfo()
{
	nIndex = m_pContextStackedWidget->currentIndex();
	// 获取下一个需要显示的页面索引
	++nIndex;
	switch (nIndex) {
		case 1: {
			m_pPageSwitchButton->setText(tr("BasisParam"));
			break;
		}
		case 2: {
			m_pPageSwitchButton->setText(tr("SeniorParam"));
			break;
		}
		default:
			break;
	}
	// 当需要显示的页面索引大于等于总页面时，切换至首页
	if (nIndex >= m_pContextStackedWidget->count())
		nIndex = 0;
	m_pContextStackedWidget->setCurrentIndex(nIndex);
}

void  CS::Widgets::IPropertyCenterWidget::confirmModifyParam(void)
{
	auto  result = CS::Widgets::FramelessMessageBox::question(this, tr("Notice"),
		tr("Do you want to save the parameter changes?"));

	if (result == QMessageBox::Yes) {
		emit signalModifyParam();
	}	
	
}

void CS::Widgets::IPropertyCenterWidget::revertParam()
{
    emit signalRevertParam();
}

void CS::Widgets::IPropertyCenterWidget::slotModifyParam()
{

}

void CS::Widgets::IPropertyCenterWidget::slotRevertParam()
{

}

/**
*@brief 更新工件高级参数图片显示区
*/
void CS::Widgets::IPropertyCenterWidget::updateSeniorParamLabelStyle()
{
	m_pShowParamPicLabel->style()->unpolish(m_pShowParamPicLabel);
	m_pShowParamPicLabel->style()->polish(m_pShowParamPicLabel);
	m_pShowParamPicLabel->update();
}
/**
*@brief 更新详细参数显示
*/
void CS::Widgets::IPropertyCenterWidget::updataWeldElaLabelStyle()
{
	
}
/**
*@brief 添加文字到TableWidget
*/
void CS::Widgets::IPropertyCenterWidget::addStrToTableWidget(int colmn, QStringList &listPortry)
{
	for (int i = 0; i < listPortry.size(); i++)
	{
		m_pDetailContentTabel->setItem(i, colmn, new QTableWidgetItem(listPortry[i]));
        m_pDetailContentTabel->item(i, colmn)->setTextAlignment(Qt::AlignCenter);
	}
	
}

/**
*@brief 添加Pic到TableWidget
*/
void CS::Widgets::IPropertyCenterWidget::addPicToTableWidget(int row, QString &strName)
{
	QPushButton *pButton = new QPushButton();
	pButton = new QPushButton();
	pButton->setAttribute(Qt::WA_TransparentForMouseEvents, true);
	pButton->setCheckable(false);
	pButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
	pButton->setProperty("nonebutton", true);
	pButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pButton->setIconSize(QSize(120, 120));
	pButton->setIcon(QIcon(strName));
	m_pDetailContentTabel->setCellWidget(row, 0, pButton);
	return;
}


