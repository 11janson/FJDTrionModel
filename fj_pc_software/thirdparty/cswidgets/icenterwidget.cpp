#include "icenterwidget.h"
using namespace CS::Widgets;
using namespace Utils;
#include <QResizeEvent>
#include <QResizeEvent>
#include <QDebug>
ICenterWidget::ICenterWidget(Id id,QWidget *parent)
    : QFrame(parent)
    ,m_id(id)    
{
    createWidgets();
    createConnects();
	setProperty("ICenterWidget", true);
	this->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
	//this->installEventFilter
}

ICenterWidget::~ICenterWidget()
{
}

void ICenterWidget::raiseWindow()
{
   // raise();
   // activateWindow();
}

void ICenterWidget::setMode(Id )
{

}

void ICenterWidget::createWidgets()
{

}

void ICenterWidget::createConnects()
{

}

void ICenterWidget::resizeEvent(QResizeEvent *event)
{
    QFrame::resizeEvent(event);
}

void ICenterWidget::showEvent(QShowEvent *event)
{
    QFrame::showEvent(event);
}

void ICenterWidget::enterWindow()
{
   
}

void ICenterWidget::leaveWindow()
{
  
}

void ICenterWidget::retranslateUi()
{
}


void ICenterWidget::changeEvent(QEvent *e)
{
    QFrame::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        retranslateUi();
        break;
    default:
        break;
    }
}
