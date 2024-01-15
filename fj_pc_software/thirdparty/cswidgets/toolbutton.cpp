#include "toolbutton.h"
using namespace CS::Widgets;
#include <QMenu>
#include <QCursor>

ToolButton::ToolButton(QWidget *parent)
    : QToolButton(parent)
{

}

ToolButton::~ToolButton()
{

}

void ToolButton::setMenu(QMenu *pMenu)
{
    connect(pMenu, SIGNAL(aboutToHide()),
            this, SLOT(recover()));
    QToolButton::setMenu(pMenu);
}


void ToolButton::recover()
{
#ifdef Q_OS_LINUX
    bool flag = this->rect().contains(this->mapFromGlobal(QCursor::pos()));
    setAttribute(Qt::WA_UnderMouse, flag);
    update();
#endif
}
