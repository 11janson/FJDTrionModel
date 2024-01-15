#include "FJBaseWidget.h"
#include "FJStyleManager.h"


FJBaseWidget::FJBaseWidget(QWidget *parent, Qt::WindowFlags f)
	: QDialog(parent, f | Qt::X11BypassWindowManagerHint)
{
	FJStyleManager::Instance()->AddManagerWidget(this);
}


FJBaseWidget::~FJBaseWidget()
{
    disconnect();
}

void FJBaseWidget::InitStyleslot()
{
    this->InitFJStyle();
}
