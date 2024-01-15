#include "ccPointPickingPosition.h"

//db
#include <cc2DLabel.h>
#include <ccPointCloud.h>


#include "ccGLWindow.h"
#include "mainwindow.h"

#include <ccPickingListener.h>

//system
#include <cassert>

ccPointPickingPosition::ccPointPickingPosition(ccPickingHub* pickingHub, QWidget* parent)
	: ccPointPickingGenericInterface(pickingHub, parent)
{
	//m_cloud = new ccPointCloud("pickedPoints");
}

ccPointPickingPosition::~ccPointPickingPosition()
{
}

bool ccPointPickingPosition::linkWith(ccGLWindow* win)
{

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccPointPickingGenericInterface::linkWith(win))
	{
		return false;
	}

	//old window?
	if (oldWin)
	{
		//oldWin->removeFromOwnDB(m_cloud);
		oldWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		oldWin->disconnect(this);
	}

	//m_cloud->clear();
	m_pickPointItems.clear();

	//new window?
	if (m_associatedWin)
	{
		//m_cloud->setPointSize(5.0);
		//m_associatedWin->addToOwnDB(m_cloud);
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA | ccGLWindow::INTERACT_SIG_RB_CLICKED | ccGLWindow::INTERACT_SIG_MB_CLICKED);//增加右键和中键响应设置

		connect(m_associatedWin, &ccGLWindow::rightButtonClicked, this, &ccPointPickingPosition::removeLastPick);
		connect(m_associatedWin, &ccGLWindow::middleButtonClicked, this, &ccPointPickingPosition::doProcess);
	}

	return true;
}

//void ccPointPickingPosition::linkWithEntity(ccHObject* entity)
//{
//	m_associatedEntity = entity;
//
//	if (m_associatedEntity)
//	{
//
//	}
//}

void ccPointPickingPosition::processPickedPoint(const PickedItem& picked)
{
	if (!picked.entity || /*picked.entity != m_associatedEntity ||*/ !MainWindow::TheInstance())
		return;

	if (picked.entity->isKindOf(CC_TYPES::POINT_CLOUD) || picked.entity->isKindOf(CC_TYPES::MESH))
	{
		m_pickPointItems.push_back(picked);
		//m_cloud->addPoint(picked.P3D);
	}
	else
	{
		return;
	}

	/*m_label->setVisible(true);
	m_label->setDisplayedIn2D(false);
	m_label->displayPointLegend(true);
	m_label->setCollapsed(true);
	ccGenericGLDisplay* display = m_associatedEntity->getDisplay();
	if (display)
	{
		m_label->setDisplay(display);
		QSize size = display->getScreenSize();
		m_label->setPosition(static_cast<float>(picked.clickPoint.x() + 20) / size.width(),
			static_cast<float>(picked.clickPoint.y() + 20) / size.height());
	}*/



	if (m_associatedWin)
		m_associatedWin->redraw();

}

void ccPointPickingPosition::cancelAndExit()
{
	

	stop(false);
}

vector<ccPickingListener::PickedItem>& ccPointPickingPosition::getPointList()
{


	return m_pickPointItems;
}

void ccPointPickingPosition::doProcess()
{
	vector<PickedItem> pickPoints = m_pickPointItems;

	int test = 0;
}

void ccPointPickingPosition::removeLastPick()
{
	if (m_pickPointItems.size() > 0)
	{
		m_pickPointItems.pop_back();


	}
}