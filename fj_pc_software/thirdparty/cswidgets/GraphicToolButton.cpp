#include "GraphicToolButton.h"
#include <qvariant.h>
#include <QLayout>

using namespace CS;
using namespace CS::Widgets;


GraphicToolButton::GraphicToolButton(QWidget *parent) : QToolButton(parent)
{
    setIconSize(QSize(65, 65));
    setFixedSize(280, 90);
    setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    setProperty("weldertoolbutton", true);
    setCheckable(true);
}

