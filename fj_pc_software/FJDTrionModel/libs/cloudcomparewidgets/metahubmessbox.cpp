#include "metahubmessbox.h"
#include <QDialog>
#include <QWidget>
#include <QVariant>
#include <QApplication>
#include <QLabel>
#include <QFile>
#include <QTextEdit>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QProcess>
#include <QToolButton>

#include "cswidgets/titlebar.h"
#include "cswidgets/framelesshelper.h"
#include "cloudcompareutils/icore.h"



using namespace CS;
using namespace CS::MetahubWidgets;
using namespace CS::Widgets;
using namespace CS::Core;

namespace CS {
	namespace MetahubWidgets {
		class MetahubMessboxPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit MetahubMessboxPrivate(CS::MetahubWidgets::MetahubMessbox *qptr);
			virtual ~MetahubMessboxPrivate();

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
			/**
             *@brief 创建标题Icon
             */
			void createWindowTitleIcon(QString);
		public:
			/**
			*@brief 更新数据到UI层显示
			*/
			void updateUIFromModel(void);


		private:
			
			QLabel	*m_pLabelContent = nullptr;
     	private:
			friend class CS::MetahubWidgets::MetahubMessbox;
			CS::MetahubWidgets::MetahubMessbox *m_qptr = nullptr;
		};

	}
}

MetahubMessboxPrivate::MetahubMessboxPrivate(
	CS::MetahubWidgets::MetahubMessbox *qptr)
{
	m_qptr = qptr;
}
MetahubMessboxPrivate::~MetahubMessboxPrivate()
{

}

void CS::MetahubWidgets::MetahubMessboxPrivate::createWidget(void)
{
	m_qptr->setFixedSize(400, 180);
	//标题栏
	m_qptr->getTitleBar()->setTitleButtonVisible(TitleBar::PredefinedButtonType::LogoButton, true);
	m_qptr->getTitleBar()->titleLabel()->setStyleSheet("padding-left:0px;background-color:transparent;");
	m_qptr->getTitleBar()->GetButtonByType(TitleBar::PredefinedButtonType::LogoButton)->setFixedSize(QSize(16, 30));
	m_qptr->getTilteBarLayout()->setContentsMargins(24, 0, 0, 0);
	//中心位置
	QHBoxLayout *pMainVLayout = new QHBoxLayout();
	pMainVLayout->setContentsMargins(1, 0, 1, 0);
	m_pLabelContent = new QLabel();
	m_pLabelContent->setWordWrap(true);
	m_qptr->getContentHolder()->setStyleSheet("background-color:rgb(42, 44, 45);");
	m_pLabelContent->setStyleSheet("background-color:rgb(42,44,45);");
	pMainVLayout->addWidget(m_pLabelContent);
	m_qptr->setLayout(pMainVLayout);
	
}

void CS::MetahubWidgets::MetahubMessboxPrivate::createWindowTitleIcon(QString str)
{
	CS::Widgets::TitleBar *pTitleBar = m_qptr->getTitleBar();
	if (str == "Ask")
	{
		pTitleBar->setTitleButtonIcon(TitleBar::PredefinedButtonType::LogoButton,
			ICore::resourceThemeImage("ask.png"));
	}
	else if (str == "Alarm")
	{

		pTitleBar->setTitleButtonIcon(TitleBar::PredefinedButtonType::LogoButton,
			ICore::resourceThemeImage("warning.png"));
	}
	else
	{
		return;
	}
	
}
void CS::MetahubWidgets::MetahubMessboxPrivate::createConnect(void)
{
	

}

void CS::MetahubWidgets::MetahubMessboxPrivate::retranslateUi()
{

}

void CS::MetahubWidgets::MetahubMessboxPrivate::createStyleSheet(void)
{
	
}

void CS::MetahubWidgets::MetahubMessboxPrivate::updateUIFromModel(void)
{


}

MetahubMessbox::MetahubMessbox(QWidget *parent
	, Qt::WindowFlags f)
	:MetahubFramelessDialog(parent)
	, m_dptr(new CS::MetahubWidgets::MetahubMessboxPrivate(this))
{
	m_dptr->createWidget();
	m_dptr->createConnect();
	m_dptr->retranslateUi();
	m_dptr->createStyleSheet();
}
int MetahubMessbox::information(QWidget *parent,
	                             const QString &title,
	                             const QString& text)
{
	CS::MetahubWidgets::MetahubMessbox msg(parent);
	msg.setTitle(title);
	msg.setCenterTips(text);
	msg.setWindowTitleIcon("Ask");
	return msg.Show();

}
int MetahubMessbox::warning(QWidget *parent,
	const QString &title,
	const QString& text)
{
	CS::MetahubWidgets::MetahubMessbox msg(parent);
	msg.setTitle(title);
	msg.setCenterTips(text);
	msg.setWindowTitleIcon("Alarm");
	return msg.Show();

}
MetahubMessbox::~MetahubMessbox()
{
	if (nullptr != m_dptr) {
		delete m_dptr;
	}
	m_dptr = nullptr;
}

int CS::MetahubWidgets::MetahubMessbox::Show(void)
{
	int nRet = this->exec();
	return nRet;
}

void  CS::MetahubWidgets::MetahubMessbox::setTitle(QString str)
{
	CS::MetahubWidgets::MetahubFramelessDialog::setWindowTitle(str);
}
void CS::MetahubWidgets::MetahubMessbox::setCenterTips(QString str)
{
	m_dptr->m_pLabelContent->setText(str);
}
void CS::MetahubWidgets::MetahubMessbox::setWindowTitleIcon(QString str)
{
	m_dptr->createWindowTitleIcon(str);
}
QPushButton *CS::MetahubWidgets::MetahubMessbox::getOkButton()
{
	return CS::MetahubWidgets::MetahubFramelessDialog::getOKButton();
}
QPushButton *CS::MetahubWidgets::MetahubMessbox::getCancelButton()
{
	return CS::MetahubWidgets::MetahubFramelessDialog::getCancelButton();
}

#include "metahubmessbox.moc"

