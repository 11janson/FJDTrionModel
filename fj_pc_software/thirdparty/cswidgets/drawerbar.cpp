#include "drawerbar.h"
#include <QToolButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStackedLayout>
#include <QLabel>
#include <QWidget>
#include <QApplication>
#include <QScrollArea>
#include <QObjectUserData>
#include <QVariant>
#include "flowlayout.h"
#include "frameline.h"


using namespace CS;
using namespace CS::Widgets;


namespace CS {
	namespace Widgets {
		struct DrawerData :QObjectUserData {
			QWidget* pItem = nullptr;
		};
		class DrawerPanelPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit DrawerPanelPrivate(DrawerPanel *ptr);
			virtual ~DrawerPanelPrivate();
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
			FlowLayout* m_pFlowLayout = new FlowLayout();
			QWidget* m_pContentCenterWidget = nullptr;
		private:
			QString m_strName;
		private:
			friend class DrawerPanel;
			CS::Widgets::DrawerPanel* m_qptr = nullptr;

		};

		class DrawerBarPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit DrawerBarPrivate(DrawerBar *ptr);
			virtual ~DrawerBarPrivate();
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
			/**
			*@brief 增加添加抽屉面板
			*/
			void appendDrawerPanel(DrawerPanel* pPanel);

		private:
			void updateDrawerPanel(QToolButton* pButton);

		private slots:
			void slotUIDrawerPanel(void);

		
		private:
			QVBoxLayout* m_pPlanLayout = nullptr;
			QWidget* m_pContentCenterWidget = nullptr;
			std::vector<QWidget> m_listV;
		private:
			friend class DrawerBar;
			CS::Widgets::DrawerBar* m_qptr = nullptr;
		};

	}
}

CS::Widgets::DrawerPanelPrivate::DrawerPanelPrivate(DrawerPanel* ptr)
{
	m_qptr = ptr;
}
CS::Widgets::DrawerPanelPrivate::~DrawerPanelPrivate()
{

}

void DrawerPanelPrivate::retranslateUi()
{

}

void DrawerPanelPrivate::createConnect(void)
{
	
}

void DrawerPanel::appendPanelWidget(QWidget* pWidget)
{
	m_dptr->m_pFlowLayout->addWidget(pWidget);
}

void DrawerPanel::setPanelCaption(const QString& strCaption)
{
	m_dptr->m_strName = strCaption;
}


void DrawerPanelPrivate::createWidget(void)
{
	QVBoxLayout* pMainHBoxLayout = new QVBoxLayout();
	pMainHBoxLayout->setContentsMargins(0, 0, 0, 0);
	pMainHBoxLayout->setSpacing(0);

	////[!].增加分割线
	//QFrame* pLineFrame = CS::Widgets::FrameLine::createHorizontalLine("isBackgroundSeparator");
	//pLineFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	//pLineFrame->setStyleSheet("QFrame{background-color:#444444;}");
	//pMainHBoxLayout->addWidget(pLineFrame, 0, Qt::AlignTop);

	m_pContentCenterWidget = new QWidget();
	m_pContentCenterWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pFlowLayout = new FlowLayout(8, 8, 8);
	m_pContentCenterWidget->setLayout(m_pFlowLayout);
	pMainHBoxLayout->addWidget(m_pContentCenterWidget);
	m_pContentCenterWidget->setProperty("contentcenter", true);
	m_pContentCenterWidget->setStyleSheet("QWidget[contentcenter]{background-color:rgb(56,56,56); border:1px solid rgb(30,30,30);}");
	m_qptr->setLayout(pMainHBoxLayout);
	
}

CS::Widgets::DrawerPanel::DrawerPanel(QWidget* parent /*= nullptr*/)
	:QFrame(parent)
	,m_dptr(new DrawerPanelPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();
}

CS::Widgets::DrawerPanel::~DrawerPanel()
{

}


QString DrawerPanel::getPanelCaption(void)
{
	return m_dptr->m_strName;
}

void DrawerPanel::setPanelContentVisible(bool bVisible)
{

}

void DrawerBarPrivate::createWidget(void)
{
	m_qptr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	QVBoxLayout* pMainVBoxLayout = new QVBoxLayout();
	pMainVBoxLayout->setContentsMargins(0, 0, 0, 0);
	pMainVBoxLayout->setSpacing(0);
	//[!].中心滚动区域
	QScrollArea* pCenterScrollArea = new QScrollArea(m_qptr);
	pCenterScrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pCenterScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	pCenterScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	pCenterScrollArea->setWidgetResizable(true);

	m_pPlanLayout = new QVBoxLayout();
	m_pPlanLayout->setContentsMargins(0, 0, 0, 0);
	m_pPlanLayout->setSpacing(0);
	m_pContentCenterWidget = new QWidget(m_qptr);
	m_pContentCenterWidget->setLayout(m_pPlanLayout);
	m_pContentCenterWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	pCenterScrollArea->setWidget(m_pContentCenterWidget);
	pCenterScrollArea->setWidget(m_pContentCenterWidget);
	pMainVBoxLayout->addWidget(pCenterScrollArea);
	m_qptr->setLayout(pMainVBoxLayout);
	
	m_qptr->setProperty("drawerbar", true);
	m_qptr->setStyleSheet("QFrame[drawerbar]{background-color:rgb(56,56,56); border:1px solid rgb(30,30,30);}");
}

void DrawerBar::appendDrawerPanel(std::vector<DrawerPanel*> panels)
{
	for (int i = 0; i < panels.size(); i++){
		m_dptr->appendDrawerPanel(panels.at(i));
	}
	m_dptr->m_pPlanLayout->addStretch(0);
	return;
}

void DrawerBarPrivate::appendDrawerPanel(DrawerPanel* pPanel)
{
	if (!pPanel){
		return;
	}

	QToolButton* pButton = new QToolButton();
	pButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	pButton->setFixedHeight(32);
	pButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	pButton->setText(pPanel->getPanelCaption());
	pButton->setIconSize(QSize(26, 26));

	QString strPath = QApplication::applicationDirPath() + "/theme/qssimage/";
	QString strName = strPath + "triangleBackup.png";
	QIcon icon(strName);
	pButton->setIcon(icon);
	DrawerData* pData = new DrawerData();
	pData->pItem = pPanel;
	pButton->setUserData(Qt::UserRole, pData);
	m_pPlanLayout->addWidget(pButton);
	m_pPlanLayout->addWidget(pPanel);
	pButton->setProperty("VisiblePanel", true);
	connect(pButton, &QToolButton::clicked, this, &DrawerBarPrivate::slotUIDrawerPanel, Qt::UniqueConnection);
}
void DrawerBarPrivate::slotUIDrawerPanel(void)
{
	QToolButton *pButton = dynamic_cast<QToolButton*>(this->sender());
	if (!pButton){
		return;
	}
	
	bool bVisiblePanel = pButton->property("VisiblePanel").toBool();
	bVisiblePanel = !bVisiblePanel;
	pButton->setProperty("VisiblePanel", bVisiblePanel);
	updateDrawerPanel(pButton);
	return;

}

void DrawerBarPrivate::updateDrawerPanel(QToolButton* pButton)
{
	if (!pButton){
		return;
	}

	DrawerData* pData = (DrawerData*)(pButton->userData(Qt::UserRole));
	if (!pData){
		return;
	}
	if (!pData->pItem){
		return;
	}
	QString strPath = QApplication::applicationDirPath() + "/theme/qssimage/";
	QString strName = strPath + "triangleBackup.png";
	bool bVisiblePanel = pButton->property("VisiblePanel").toBool();
	if (bVisiblePanel){
		strName = strPath + "triangleBackup.png";
	}
	else {
		strName = strPath + "triangle.png";
	}
	QIcon icon(strName);
	pButton->setIcon(icon);
	pData->pItem->setVisible(bVisiblePanel);
	return;
}


void DrawerBarPrivate::createConnect(void)
{

}

void DrawerBarPrivate::retranslateUi()
{

}
DrawerBarPrivate::DrawerBarPrivate(DrawerBar* ptr)
{
	m_qptr = ptr;
}

DrawerBarPrivate::~DrawerBarPrivate()
{

}

DrawerBar::DrawerBar(QWidget* parent /*= nullptr*/)
	:QFrame(parent)
	,m_dptr(new DrawerBarPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();

}

DrawerBar::~DrawerBar()
{
	if (m_dptr){
		delete m_dptr;
		m_dptr = nullptr;
	}
}


#include "drawerbar.moc"