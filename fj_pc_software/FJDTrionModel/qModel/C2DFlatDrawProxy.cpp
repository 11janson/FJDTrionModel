
#include "C2DFlatDrawProxy.h"
#include "ccReservedIDs.h"

#include <ccLineGroup.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccLines.h>
#include <cc2DArcLine.h>
#include <cc2DRound.h>
#include <cc2DRect.h>
#include <ccGenericPointCloud.h>
#include <ccTypes.h>

#include <ccGLWindow.h>

#include <QGuiApplication>
#include <QApplication>

C2DFlatDrawProxy::C2DFlatDrawProxy()
	
{
	/*m_polyVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));

	m_pContainer = new ccHObject("Container");*/

}

C2DFlatDrawProxy::~C2DFlatDrawProxy()
{
	/*if (m_pLineGroup)
		delete m_pLineGroup;
	m_pLineGroup = nullptr;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = nullptr;

	if (m_pContainer)
		delete m_pContainer;
	m_pContainer = nullptr;*/
}

void C2DFlatDrawProxy::setGraphicType(cc2DItemBase::EGraphicType type)
{
	/*if (m_pLineGroup)
	{
		delete m_pLineGroup;
		m_pLineGroup = nullptr;
	}
	m_Type = type;
	m_releaseMouse = true;


	switch (m_Type)
	{
	case LINE:
	case LINES:
		m_pLineGroup = new ccLines(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		break;
	case ARC:
		m_pLineGroup = new cc2DArcLine(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		break;
	case ROUND3:
		m_pLineGroup = new cc2DRound(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		dynamic_cast<cc2DRound*>(m_pLineGroup)->setRoundType(cc2DRound::ROUND3PTS);
		break;
	case ROUND2:
		m_pLineGroup = new cc2DRound(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		dynamic_cast<cc2DRound*>(m_pLineGroup)->setRoundType(cc2DRound::ROUND2PTS);
		break;
	case ROUNDRADIUS:
		m_pLineGroup = new cc2DRound(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		dynamic_cast<cc2DRound*>(m_pLineGroup)->setRoundType(cc2DRound::ROUND2PTSRADIUS);
		break;
	case RECT3:
		m_pLineGroup = new cc2DRect(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		dynamic_cast<cc2DRect*>(m_pLineGroup)->setRoundType(cc2DRect::RECT3PTS);
		break;
	case RECTACROSS:
		m_pLineGroup = new cc2DRect(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
		dynamic_cast<cc2DRect*>(m_pLineGroup)->setRoundType(cc2DRect::RECTACROSS);
		break;
	default:
		break;
	}
	if (m_pLineGroup)
	{
		m_pLineGroup->setForeground(true);
		m_pLineGroup->setColor(ccColor::green);

		m_pLineGroup->setColor(ccColor::red);
		m_pLineGroup->setWidth(PointCoordinateType(1.0));

		m_pLineGroup->showColors(true);
		m_pLineGroup->set2DMode(true);

		m_pLineGroup->setDisplay(m_associatedWin);
		m_associatedWin->addToOwnDB(m_pLineGroup);

	}*/
}

bool C2DFlatDrawProxy::linkWith(ccGLWindow* win)
{
	//ccGLWindow* oldWin = m_associatedWin;

	//if (!ccOverlayDialog::linkWith(win))
	//{
	//	return false;
	//}
	//if (oldWin)
	//{
	//	oldWin->disconnect(this);
	//	if (m_pLineGroup)
	//	{
	//		m_pLineGroup->setDisplay(nullptr);
	//	}
	//	if (polyContainer)
	//	{
	//		polyContainer->setDisplay(nullptr);
	//	}

	//}
	/*m_associatedWin = win;


	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked, this, &C2DFlatDrawProxy::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked, this, &C2DFlatDrawProxy::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved, this, &C2DFlatDrawProxy::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonReleased, this, &C2DFlatDrawProxy::closeRectangle);


		if (m_pContainer)
		{
			m_pContainer->setDisplay(m_associatedWin);
		}
		m_associatedWin->addToOwnDB(m_pContainer);

		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);

		m_state = STARTED;
		m_rectangularSelection = false;
	}*/

	return true;
}


void C2DFlatDrawProxy::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	////process not started yet?
	//if ((m_state & RUNNING) == 0)
	//{
	//	return;
	//}
	//if (!m_associatedWin)
	//{
	//	assert(false);
	//	return;
	//}

	//assert(m_polyVertices);
	//assert(m_pLineGroup);

	//unsigned vertCount = m_polyVertices->size();

	////new point (expressed relatively to the screen center)
	//QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	//CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
	//	static_cast<PointCoordinateType>(pos2D.y()),
	//	0);

	//if (m_state & RECTANGLE)
	//{
	//	//we need 4 points for the rectangle!
	//	if (vertCount != 4)
	//		m_polyVertices->resize(4);

	//	const CCVector3* A = m_polyVertices->getPointPersistentPtr(0);
	//	CCVector3* B = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(1));
	//	CCVector3* C = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(2));
	//	CCVector3* D = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(3));
	//	*B = CCVector3(A->x, P.y, 0);
	//	*C = P;
	//	*D = CCVector3(P.x, A->y, 0);

	//	if (vertCount != 4)
	//	{
	//		m_pLineGroup->clear();
	//		if (!m_pLineGroup->addPointIndex(0, 4))
	//		{
	//			ccLog::Error("Out of memory!");
	//			//allowPolylineExport(false);
	//			return;
	//		}
	//		m_pLineGroup->setClosed(true);
	//	}
	//}
	//else if (m_state & POLYLINE)
	//{
	//	if (vertCount < 2)
	//		return;
	//	//we replace last point by the current one
	//	CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
	//	*lastP = P;
	//}

	//m_associatedWin->redraw(true, false);
}

void C2DFlatDrawProxy::addPointToPolylineExt(int x, int y, bool allowClicksOutside)
{
	//if ((m_state & STARTED) == 0)
	//{
	//	return;
	//}
	//if (!m_associatedWin)
	//{
	//	assert(false);
	//	return;
	//}

	//if (!allowClicksOutside
	//	&& (x < 0 || y < 0 || x >= m_associatedWin->qtWidth() || y >= m_associatedWin->qtHeight())
	//	)
	//{
	//	//ignore clicks outside of the 3D view
	//	return;
	//}

	//assert(m_polyVertices);
	//assert(m_pLineGroup);
	//unsigned vertCount = m_polyVertices->size();

	////particular case: we close the rectangular selection by a 2nd click
	//if (m_rectangularSelection && vertCount == 4 && (m_state & RUNNING))
	//	return;


	////new point
	//QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	//CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
	//	static_cast<PointCoordinateType>(pos2D.y()),
	//	0);

	////CTRL key pressed at the same time?
	//bool ctrlKeyPressed = m_rectangularSelection || ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	////start new polyline?
	//if (((m_state & RUNNING) == 0) || vertCount == 0 || ctrlKeyPressed)
	//{
	//	//reset state
	//	m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
	//	m_state |= (STARTED | RUNNING);
	//	//reset polyline
	//	m_polyVertices->clear();
	//	if (!m_polyVertices->reserve(2))
	//	{
	//		ccLog::Error("Out of memory!");
	//		//allowPolylineExport(false);
	//		return;
	//	}
	//	//we add the same point twice (the last point will be used for display only)
	//	m_polyVertices->addPoint(P);
	//	m_polyVertices->addPoint(P);
	//	m_pLineGroup->clear();
	//	if (!m_pLineGroup->addPointIndex(0, 2))
	//	{
	//		ccLog::Error("Out of memory!");
	//		//allowPolylineExport(false);
	//		return;
	//	}
	//}
	//else //next points in "polyline mode" only
	//{
	//	//we were already in 'polyline' mode?
	//	if (m_state & POLYLINE)
	//	{
	//		if (!m_polyVertices->reserve(vertCount + 1))
	//		{
	//			ccLog::Error("Out of memory!");
	//			//allowPolylineExport(false);
	//			return;
	//		}

	//		//we replace last point by the current one
	//		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
	//		*lastP = P;
	//		//and add a new (equivalent) one
	//		m_polyVertices->addPoint(P);
	//		if (!m_pLineGroup->addPointIndex(vertCount))
	//		{
	//			ccLog::Error("Out of memory!");
	//			return;
	//		}
	//		m_pLineGroup->setClosed(true);

	//		unsigned vertCount = m_pLineGroup->size();
	//		if (m_Type == LINE )
	//		{
	//			//保存当前绘制图形
	//			cc2DItemBase* lines = new ccLines(dynamic_cast<ccLines&>(*m_pLineGroup));
	//			//poly->showVertices(false);
	//			m_pContainer->addChild(lines);

	//			m_pLineGroup->clear();
	//			m_polyVertices->clear();
	//			m_releaseMouse = true;


	//			if (m_associatedWin)
	//			{
	//				m_associatedWin->releaseMouse();
	//			}

	//			//stop
	//			//m_state &= (~RUNNING);
	//		}
	//		else if (m_Type == ARC && vertCount == 4)
	//		{
	//			//保存当前绘制图形
	//			cc2DItemBase* lines = new cc2DArcLine(dynamic_cast<cc2DArcLine&>(*m_pLineGroup));
	//			//poly->showVertices(false);
	//			m_pContainer->addChild(lines);

	//			m_pLineGroup->clear();
	//			m_polyVertices->clear();
	//			m_releaseMouse = true;


	//			if (m_associatedWin)
	//			{
	//				m_associatedWin->releaseMouse();
	//			}

	//			//stop
	//			//m_state &= (~RUNNING);
	//		}
	//		else if (m_Type == ROUND3 && vertCount == 4)
	//		{
	//			//保存当前绘制图形
	//			cc2DItemBase* lines = new cc2DRound(dynamic_cast<cc2DRound&>(*m_pLineGroup));
	//			//poly->showVertices(false);
	//			m_pContainer->addChild(lines);

	//			m_pLineGroup->clear();
	//			m_polyVertices->clear();
	//			m_releaseMouse = true;


	//			if (m_associatedWin)
	//			{
	//				m_associatedWin->releaseMouse();
	//			}

	//			//stop
	//			//m_state &= (~RUNNING);
	//		}
	//		else if (m_Type == ROUND2 && vertCount == 3)
	//		{
	//			//保存当前绘制图形
	//			cc2DItemBase* lines = new cc2DRound(dynamic_cast<cc2DRound&>(*m_pLineGroup));
	//			//poly->showVertices(false);
	//			m_pContainer->addChild(lines);

	//			m_pLineGroup->clear();
	//			m_polyVertices->clear();
	//			m_releaseMouse = true;


	//			if (m_associatedWin)
	//			{
	//				m_associatedWin->releaseMouse();
	//			}

	//			//stop
	//			//m_state &= (~RUNNING);
	//		}
	//		else if (m_Type == ROUNDRADIUS && vertCount == 3)
	//		{
	//			//保存当前绘制图形
	//			cc2DItemBase* lines = new cc2DRound(dynamic_cast<cc2DRound&>(*m_pLineGroup));
	//			//poly->showVertices(false);
	//			m_pContainer->addChild(lines);

	//			m_pLineGroup->clear();
	//			m_polyVertices->clear();
	//			m_releaseMouse = true;


	//			if (m_associatedWin)
	//			{
	//				m_associatedWin->releaseMouse();
	//			}

	//			//stop
	//			//m_state &= (~RUNNING);
	//		}


	//	}
	//	else //we must change mode
	//	{
	//		assert(false); //we shouldn't fall here?!
	//		m_state &= (~RUNNING);
	//		addPointToPolylineExt(x, y, allowClicksOutside);
	//		return;
	//	}
	//}

	////DGM: to increase the poll rate of the mouse movements in ccGLWindow::mouseMoveEvent
	////we have to completely grab the mouse focus!
	////(the only way to take back the control is to right-click now...)
	//if (!m_releaseMouse)
	//{
	//	m_associatedWin->grabMouse();
	//}
	//m_associatedWin->redraw(true, false);
}

void C2DFlatDrawProxy::closeRectangle()
{
	////only for rectangle selection in RUNNING mode
	//if ((m_state & RECTANGLE) == 0 || (m_state & RUNNING) == 0)
	//	return;

	//assert(m_pLineGroup);
	//unsigned vertCount = m_pLineGroup->size();
	//if (vertCount < 4)
	//{
	//	//first point only? we keep the real time update mechanism
	//	if (m_rectangularSelection)
	//		return;
	//	m_pLineGroup->clear();
	//	m_polyVertices->clear();
	//	//allowPolylineExport(false);
	//}
	//else
	//{
	//	//allowPolylineExport(true);

	//	//janson.yang
	//	//ccPolyline* poly = new ccPolyline(*m_pLineGroup);
	//	//poly->showVertices(false);
	//	//polyContainer->addChild(poly);
	//}



	////stop
	//m_state &= (~RUNNING);

	//if (m_associatedWin)
	//{
	//	m_associatedWin->releaseMouse();
	//	m_associatedWin->redraw(true, false);
	//}
	////inButton->setIcon(QIcon(QString::fromUtf8(":/CC/images/smallSegmentIn.png")));
	////outButton->setIcon(QIcon(QString::fromUtf8(":/CC/images/smallSegmentOut.png")));
	////inButton->setDisabled(false);
	////outButton->setDisabled(false);
}

void C2DFlatDrawProxy::closePolyLine(int, int)
{
	////only for polyline in RUNNING mode
	//if ((m_state & POLYLINE) == 0 || (m_state & RUNNING) == 0)
	//	return;

	//if (m_associatedWin)
	//{
	//	m_associatedWin->releaseMouse();
	//}

	//assert(m_pLineGroup);
	//unsigned vertCount = m_pLineGroup->size();
	////if (vertCount < 4)
	//if (m_Type == LINE)
	//{
	//	//保存当前绘制图形
	//	cc2DItemBase* lines = new ccLines(dynamic_cast<ccLines&>(*m_pLineGroup));
	//	//poly->showVertices(false);
	//	m_pContainer->addChild(lines);

	//	m_pLineGroup->clear();
	//	m_polyVertices->clear();
	//}
	////else if (vertCount < 3)
	////{
	////	m_pLineGroup->clear();
	////	m_polyVertices->clear();
	////}
	//else if (vertCount > 2 && m_Type == LINES)
	//{
	//	//remove last point!
	//	m_pLineGroup->resize(vertCount - 1); //can't fail --> smaller
	//	m_pLineGroup->setClosed(true);

	//	//保存当前绘制图形
	//	cc2DItemBase* lines = new ccLines(dynamic_cast<ccLines&>(*m_pLineGroup));
	//	//poly->showVertices(false);
	//	m_pContainer->addChild(lines);

	//	m_pLineGroup->clear();
	//	m_polyVertices->clear();
	//}


	////stop
	//m_state &= (~RUNNING);

	////set the default import/export icon to 'export' mode
	////loadSaveToolButton->setDefaultAction(actionExportSegmentationPolyline);
	////allowPolylineExport(m_pLineGroup->size() > 1);




	//if (m_associatedWin)
	//{
	//	m_associatedWin->redraw(true, false);
	//}
	////inButton->setIcon(QIcon(QString::fromUtf8(":/CC/images/smallSegmentIn.png")));
	////outButton->setIcon(QIcon(QString::fromUtf8(":/CC/images/smallSegmentOut.png")));
	////inButton->setDisabled(false);
	////outButton->setDisabled(false);
}
