#include "fjdragablemdiarea.h"
#include <ccGLWindow.h>
#include <QDebug>
#include <QVBoxLayout>
FJDragableMdiArea::FJDragableMdiArea(QWidget *parent)
    : QWidget(parent)
{
	m_3dWindow = new QMdiSubWindow(this, Qt::FramelessWindowHint);
	m_2dWindow = new QMdiSubWindow(this, Qt::FramelessWindowHint);
	QVBoxLayout * layout = new QVBoxLayout;
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);
	m_pEditCenterSpliterWidget = new QSplitter(Qt::Vertical, 0);
	m_pEditCenterSpliterWidget->setOpaqueResize(false);
	m_pEditCenterSpliterWidget->setChildrenCollapsible(false);
	m_pEditCenterSpliterWidget->setStyleSheet("QWidget{background: rgb(30, 30, 30);}");
	m_pEditCenterSpliterWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	layout->addWidget(m_pEditCenterSpliterWidget);
	setMouseTracking(true);
	m_pEditCenterSpliterWidget->insertWidget(0, m_3dWindow);
	m_pEditCenterSpliterWidget->insertWidget(1, m_2dWindow);
	m_pEditCenterSpliterWidget->adjustSize();
	QList<int> sizes;
	sizes << 50000 << 50000;
	m_pEditCenterSpliterWidget->setSizes(sizes);
	setLayout(layout);
}

void FJDragableMdiArea::setDirection(Qt::Orientation orientation)
{
	m_pEditCenterSpliterWidget->setOrientation(orientation);
	update();
}

void FJDragableMdiArea::setWindowOrder(bool is3dfront)
{
	QWidget *widget1 = new QWidget(this);
	QWidget *widget2 = new QWidget(this);
	m_pEditCenterSpliterWidget->replaceWidget(0, widget1);
	m_pEditCenterSpliterWidget->replaceWidget(1, widget2);
	if (is3dfront)
	{
		m_pEditCenterSpliterWidget->replaceWidget(0, m_3dWindow);
		m_pEditCenterSpliterWidget->replaceWidget(1, m_2dWindow);
	}
	else
	{
		m_pEditCenterSpliterWidget->replaceWidget(0, m_2dWindow);
		m_pEditCenterSpliterWidget->replaceWidget(1, m_3dWindow);
	}
	delete widget1;
	widget1 = nullptr;
	delete widget2;
	widget2 = nullptr;
	m_pEditCenterSpliterWidget->refresh();
}

FJDragableMdiArea::~FJDragableMdiArea()
{
}

void FJDragableMdiArea::setActiveSubWindow(QMdiSubWindow *window)
{
	if (m_3dWindow == window)
	{
		m_is3DWindowActive = true;
		emit subWindowActivated(m_3dWindow);
	}
	else
	{
		m_is3DWindowActive = false;
		emit subWindowActivated(m_2dWindow);
	}
}

void FJDragableMdiArea::closeAllSubWindows()
{
	delete m_3dWindow;
	m_3dWindow = nullptr;
	delete m_2dWindow;
	m_2dWindow = nullptr;
}

QMdiSubWindow *FJDragableMdiArea::activeSubWindow() const
{
	if (m_is3DWindowActive)
	{
		return m_3dWindow;
	}
	else
	{
		return m_2dWindow;
	}
}

QMdiSubWindow * FJDragableMdiArea::get2dWindow()
{
	return m_2dWindow;
}

QMdiSubWindow * FJDragableMdiArea::get3dWindow()
{
	return m_3dWindow;
}

QList<QMdiSubWindow*> FJDragableMdiArea::subWindowList()
{
	QList<QMdiSubWindow*> list;
	list.push_back(m_3dWindow);
	list.push_back(m_2dWindow);
	return list;
}

void FJDragableMdiArea::setShowType(FJDragableMdiArea::MDIAREASHOWTYPE type)
{
	m_showType = type;
	refreashWindow();
	QList<int> sizes;
	sizes << 50000 << 50000;
	//setCursor(Qt::ArrowCursor);
	switch (m_showType)
	{
	case FJDragableMdiArea::SHOWONLY3D:
		m_3dWindow->setMinimumHeight(0);
		m_3dWindow->setMinimumWidth(0);
		m_2dWindow->setMinimumHeight(0);
		m_2dWindow->setMinimumWidth(0);
		m_is3DWindowActive = true;
		emit subWindowActivated(m_3dWindow);
		break;
	case FJDragableMdiArea::SHOWONLY2D:
		m_3dWindow->setMinimumHeight(0);
		m_3dWindow->setMinimumWidth(0);
		m_2dWindow->setMinimumHeight(0);
		m_2dWindow->setMinimumWidth(0);
		m_is3DWindowActive = false;
		emit subWindowActivated(m_2dWindow);
		break;
	case FJDragableMdiArea::SHOWALL:
		m_3dWindow->setMinimumHeight(100);
		m_3dWindow->setMinimumWidth(100);
		m_2dWindow->setMinimumHeight(100);
		m_2dWindow->setMinimumWidth(100);
		m_is3DWindowActive = true;
		m_pEditCenterSpliterWidget->setSizes(sizes);
		emit subWindowActivated(m_3dWindow);
		break;
	default:
		break;
	}
}

FJDragableMdiArea::MDIAREASHOWTYPE FJDragableMdiArea::getShowType()
{
	return m_showType;
}


QMdiSubWindow * FJDragableMdiArea::addMethubSubWindow(QWidget *widget, Qt::WindowFlags windowFlags, bool is3D)
{
	if (is3D)
	{
        widget->setObjectName("viewer3d");
		m_3dWindow->setWidget(widget);
		ccGLWindow* my3DWidget = qobject_cast<ccGLWindow*>(widget);
		if (my3DWidget)
		{
			connect(my3DWidget, &ccGLWindow::activeSelectWindow, this, [=]() {

				if (!m_is3DWindowActive)
				{
					m_is3DWindowActive = true;
					emit subWindowActivated(m_3dWindow);
				}
			});
		}
		return m_3dWindow;
	}
	else
	{
        widget->setObjectName("viewer2d");
		m_2dWindow->setWidget(widget);
		ccGLWindow* my2DWidget = qobject_cast<ccGLWindow*>(widget);
		if (my2DWidget)
		{
			connect(my2DWidget, &ccGLWindow::activeSelectWindow, this, [=]() {

				if (m_is3DWindowActive)
				{
					m_is3DWindowActive = false;
					emit subWindowActivated(m_2dWindow);
				}
			});
		}
		return m_2dWindow;
	}
}

void FJDragableMdiArea::resizeEvent(QResizeEvent *event)
{
	QWidget::resizeEvent(event);
	//refreashWindow();
}

void FJDragableMdiArea::paintEvent(QPaintEvent *event)
{
	QWidget::paintEvent(event);
}


void FJDragableMdiArea::refreashWindow()
{
	switch (m_showType)
	{
	case FJDragableMdiArea::SHOWONLY3D:
		m_3dWindow->show();
		m_2dWindow->hide();
		break;
	case FJDragableMdiArea::SHOWONLY2D:
		m_2dWindow->show();
		m_3dWindow->hide();
		break;
	case FJDragableMdiArea::SHOWALL:
		m_3dWindow->show();
		m_2dWindow->show();
		break;
	default:
		break;
	}
}