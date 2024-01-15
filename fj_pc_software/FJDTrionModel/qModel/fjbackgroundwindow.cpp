#include "fjbackgroundwindow.h"
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QPixmap>
#include "FJStyleManager.h"
#include <QCoreApplication>
FJBackgroundWindow::FJBackgroundWindow(QWidget *parent)
    : QWidget(parent)
{
}

FJBackgroundWindow::~FJBackgroundWindow()
{
}

void FJBackgroundWindow::paintEvent(QPaintEvent *event)
{
	QPainter p(this);
	QPixmap pix(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/background.png");
	p.drawPixmap((width()-320)/2, (height() - 240) / 2-60,320,240,pix);
	QPen pen(QColor(221,221,221));
	p.setPen(pen);
	QString str(QCoreApplication::translate("FJBackgroundWindow", "No view, please select 3D / 2D view", nullptr));
	p.drawText((width() - 280) / 2, (height() - 240) / 2 +190, 280, 40, Qt::AlignCenter, str);
}