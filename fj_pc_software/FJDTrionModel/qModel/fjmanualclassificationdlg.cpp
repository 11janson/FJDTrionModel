//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "fjmanualclassificationdlg.h"
#include <FJStyleManager.h>
#include <QDebug>
//Local
#include "mainwindow.h"
#include "ccItemSelectionDlg.h"
#include "ccReservedIDs.h"

//CCCoreLib
#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <cc2DViewportObject.h>

//for the helper (apply)
#include <cc2DLabel.h>
#include <ccCameraSensor.h>
#include <ccGBLSensor.h>
#include <ccSubMesh.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QButtonGroup>
#include <framelessmessagebox.h>
//System
#include <assert.h>
#include <QTimer>
#include "ccClipBox.h"
#include "fjpointcloudutil.h"
#include <QColorDialog>

static void SetButtonColor(QAbstractButton* button, const QColor &col)
{
	if (button != nullptr)
	{
		button->setStyleSheet(QStringLiteral("* { background-color: rgb(%1,%2,%3);border: 2px solid #989898; }")
			.arg(col.red())
			.arg(col.green())
			.arg(col.blue())
		);
	}
}

FJManualClassificationDlg::FJManualClassificationDlg(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::FJManualClassificationDlg()
	, m_state(0)
	, m_segmentationPoly(nullptr)
	, m_polyVertices(nullptr)
	, m_selectedPoints(nullptr)
	, m_layout(nullptr)
	, m_rectangularSelection(true)
{
	// Set QDialog background as transparent (DGM: doesn't work over an OpenGL context)
	//setAttribute(Qt::WA_NoSystemBackground);

	setupUi(this);
	initAllClassficationType();
	connect(comboBox_name, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int index) {
		updateTypeColor();
	});
	m_layout = new QGridLayout();
	m_layout->setContentsMargins(0, 6,0, 6);
	frame->setLayout(m_layout);
	connect(pushButton, &QPushButton::clicked, this, &FJManualClassificationDlg::applyClassfication);
	connect(pushButton_cancel, &QPushButton::clicked, this, &FJManualClassificationDlg::cancel);
	connect(radioButton_selectall, &QRadioButton::clicked, this, &FJManualClassificationDlg::selectedAllClassfication);
	connect(toolButton_typecolor, &QToolButton::clicked, this, &FJManualClassificationDlg::changeClassficationColor);
	connect(razButton, &QToolButton::clicked, [=]() {
		m_Invertselection = !m_Invertselection;
		if (m_Invertselection)
		{
			setActionIcon(razButton, "manualclassificationunselectlicked", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
		}
		else
		{
			setActionIcon(razButton, "manualclassificationunselectnormal", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
		}
	});
	connect(validButton, &QToolButton::clicked, this, &FJManualClassificationDlg::clearALL);
	connect(cancelButton, &QToolButton::clicked, this, &FJManualClassificationDlg::cancel);
	actionSetRectangularSelectionbtn->setToolTip(tr("Rectangle select"));
	actionSetPolylineSelectionbtn->setToolTip(tr("Polygon select"));
	connect(actionSetPolylineSelectionbtn, &QToolButton::clicked, this, &FJManualClassificationDlg::doSetPolylineSelection);
	connect(actionSetRectangularSelectionbtn, &QToolButton::clicked, this, &FJManualClassificationDlg::doSetRectangularSelection);
	connect(toolButton_lineup, &QToolButton::clicked, this, &FJManualClassificationDlg::doSetPolylineUpSelection);
	connect(toolButton_linedown, &QToolButton::clicked, this, &FJManualClassificationDlg::doSetPolylineDownSelection);
	connect(toolButton_lasso, &QToolButton::clicked, this, &FJManualClassificationDlg::doSetPolylineLassoSelection);
	addOverriddenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &FJManualClassificationDlg::onShortcutTriggered);
    connect(this, &ccOverlayDialog::shortcutDoubleBondTriggered, this, &FJManualClassificationDlg::onShortcutDoubleBondTriggered);
	connect(MainWindow::TheInstance(), &MainWindow::signalUpdateEnterCloudSize, this, &FJManualClassificationDlg::updateHeightLightPointSize);

	m_polyVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	m_selectedPoints = new ccPointCloud();
	m_selectedPoints->setVisible(true);
	m_segmentationPoly = new ccPolyline(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
	m_segmentationPoly->setForeground(true);

	m_segmentationPoly->setColor(ccColor::red);
	m_segmentationPoly->setWidth(PointCoordinateType(1.0));

	m_segmentationPoly->showColors(true);
	m_segmentationPoly->set2DMode(true);

	polyContainer = new ccHObject("polyContainer_segment");
	initcurrentStyle();
}

void FJManualClassificationDlg::updateHeightLightPointSize(bool state , float size)
{
    QSet<ccHObject*>::iterator it;
    for (it = m_toSegment.begin(); it != m_toSegment.end(); ++it) {
        ccPointCloud* trackcloud = dynamic_cast<ccPointCloud*>(*it);
        if (trackcloud)
        {
            float size = static_cast<float>(trackcloud->getPointSize());
            bool autoPointSize = trackcloud->getPointSizeAdaptively();
            if (m_selectedPoints) {
                m_selectedPoints->setPointSize(size);
                m_selectedPoints->setPointSizeAdaptively(autoPointSize);
            }
        }
    }
}

void FJManualClassificationDlg::InitFJStyle()
{
	ccOverlayDialog::InitFJStyle();
	initcurrentStyle();
}

void FJManualClassificationDlg::initcurrentStyle()
{
	QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
	cancelButton->setIcon(closeicon);
	setActionIcon(toolButton_lineup,"manualclassificationuplinenormal","manualclassificationuplineclicked","manualclassificationuplinedisabled");
	setActionIcon(toolButton_linedown, "manualclassificationdownlinenormal", "manualclassificationdownlineclicked", "manualclassificationdownlinedisabled");
	setActionIcon(actionSetRectangularSelectionbtn, "manualclassificationrectnormal", "manualclassificationrectclicked", "manualclassificationrectdisable");
	setActionIcon(actionSetPolylineSelectionbtn, "manualclassificationpolinenormal", "manualclassificationpolineclicked", "manualclassificationpolinedisabled");
	setActionIcon(toolButton_lasso, "manualclassificationscopenormal", "manualclassificationscopeclicked", "manualclassificationscopedisabled");
	setActionIcon(razButton, "manualclassificationunselectnormal", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
	setActionIcon(validButton, "manualclassificationclearnormal", "manualclassificationclearcicked", "manualclassificationcleardisabled");
}

void FJManualClassificationDlg::closeButtonState()
{
	setActionIcon(toolButton_lineup, "manualclassificationuplinenormal", "manualclassificationuplineclicked", "manualclassificationuplinedisabled");
	setActionIcon(toolButton_linedown, "manualclassificationdownlinenormal", "manualclassificationdownlineclicked", "manualclassificationdownlinedisabled");
	setActionIcon(actionSetRectangularSelectionbtn, "manualclassificationrectnormal", "manualclassificationrectclicked", "manualclassificationrectdisable");
	setActionIcon(actionSetPolylineSelectionbtn, "manualclassificationpolinenormal", "manualclassificationpolineclicked", "manualclassificationpolinedisabled");
	setActionIcon(toolButton_lasso, "manualclassificationscopenormal", "manualclassificationscopeclicked", "manualclassificationscopedisabled");
}

bool FJManualClassificationDlg::addEntity(ccHObject* entity, bool silent/*=false*/)
{
	if (!entity)
	{
		assert(false);
		return false;
	}

	if (!entity->isDisplayedIn(m_associatedWin) && !silent)
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Entity [%1] is not visible in the active 3D view!").arg(entity->getName()));
	}

	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

		ccGenericMesh* associatedMesh = nullptr;
		if (ccGenericMesh::IsCloudVerticesOfMesh(cloud, &associatedMesh))
		{
			assert(nullptr != associatedMesh);
			if (m_toSegment.contains(associatedMesh))
			{
				if (!silent)
				{
					ccLog::Warning(QString("[Graphical Segmentation Tool] The mesh associated to cloud %1 is already selected").arg(cloud->getName()));
				}
				return false;
			}

			// hide the associated mesh, as it will also be (graphically) segmented
			associatedMesh->pushDisplayState();
			associatedMesh->setVisible(false);
		}

		ccPolyline* associatedPolyline = nullptr;
		if (ccPolyline::IsCloudVerticesOfPolyline(cloud, &associatedPolyline))
		{
			assert(nullptr != associatedPolyline);
			if (m_toSegment.contains(associatedPolyline))
			{
				if (!silent)
				{
					ccLog::Warning(QString("[Graphical Segmentation Tool] The polyline associated to cloud %1 is already selected").arg(cloud->getName()));
				}
				return false;
			}

			// hide the associated polyline, as it will also be (graphically) segmented
			associatedPolyline->pushDisplayState();
			associatedPolyline->setVisible(false);
		}

		m_toSegment.insert(cloud);
		cloud->pushDisplayState();
		cloud->setVisible(true);
		cloud->setEnabled(true);
		//设置为类别渲染模式，如果没有则添加
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entity);
			if (pc)
			{
				int sfIdx = pc->getScalarFieldIndexByName("Classification");
				if (sfIdx < 0)
				{
					// create the scalar field Classification if needed
					sfIdx = pc->addScalarField("Classification");
					if (sfIdx < 0)
					{
						ccLog::Error(tr("Not enough memory"));
						return false;
					}
				}
				pc->showSF(true);
				pc->setCurrentCombinationMode(CLASSIFICATION);
				pc->setCurrentDisplayedScalarField(sfIdx);
				FJPointCloudUtil::updateClassficationDataFromDoc(entity);
			}
		}
		return true;
	}
	else if (entity->isKindOf(CC_TYPES::MESH))
	{
		if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			if (!silent)
			{
				ccLog::Warning("[FJManualClassificationDlg] Can't segment primitives yet! Sorry...");
			}
			return false;
		}
		if (entity->isKindOf(CC_TYPES::SUB_MESH))
		{
			if (!silent)
			{
				ccLog::Warning("[FJManualClassificationDlg] Can't segment sub-meshes! Select the parent mesh...");
			}
			return false;
		}

		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
		assert(mesh);
		ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
		if (!vertices)
		{
			assert(false);
			return false;
		}

		// Make sure the vertices of this mesh are not already in the 'to segment' list
		if (m_toSegment.contains(vertices))
		{
			//let's remove the vertices
			mesh->pushDisplayState(); // just in case the vertices were inserted before the mesh)
			vertices->popDisplayState();
			m_toSegment.remove(vertices);
		}

		vertices->resetVisibilityArray();
        ccPointCloud* cloudobj = static_cast<ccPointCloud*>(vertices);
        if (cloudobj)
        {
            cloudobj->visibilityArrayChange();
        }
		m_toSegment.insert(mesh);
		mesh->pushDisplayState();
		mesh->setVisible(true);
		mesh->setEnabled(true);

		return true;
	}
	else if (entity->isKindOf(CC_TYPES::POLY_LINE))
	{
		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccGenericPointCloud* verticesCloud = dynamic_cast<ccGenericPointCloud*>(poly->getAssociatedCloud());
		if (!verticesCloud)
		{
			assert(false);
			return false;
		}

		// Make sure the vertices of this polyline are not already in the 'to segment' list
		if (verticesCloud && m_toSegment.contains(verticesCloud))
		{
			//let's remove the vertices
			poly->pushDisplayState(); // just in case the vertices were inserted before the polyline)
			verticesCloud->popDisplayState();
			m_toSegment.remove(verticesCloud);
		}

		verticesCloud->resetVisibilityArray();
		m_toSegment.insert(poly);
		poly->pushDisplayState();
		poly->setVisible(true);
		poly->setEnabled(true);

		return true;
	}
	else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//automatically add the entities contained in the group
		bool result = false;
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
			result |= addEntity(entity->getChild(i));

		return result;
	}
	else
	{
		if (!silent)
		{
			ccLog::Warning("[FJManualClassificationDlg] Can't segment entity " + entity->getName());
		}
		return false;
	}
}

unsigned FJManualClassificationDlg::getNumberOfValidEntities() const
{
	return static_cast<unsigned>(m_toSegment.size());
}


FJManualClassificationDlg::~FJManualClassificationDlg()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = nullptr;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = nullptr;

	if (m_selectedPoints)
		delete m_selectedPoints;
	m_selectedPoints = nullptr;

	if (polyContainer)
		delete polyContainer;
	polyContainer = nullptr;
}

bool FJManualClassificationDlg::linkWith(ccGLWindow* win)
{
	assert(m_segmentationPoly);
	assert(polyContainer);

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(nullptr);
		}
		if (polyContainer)
		{
			polyContainer->setDisplay(nullptr);
		}

	}

	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonNoMovedRelease, this, &FJManualClassificationDlg::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonNoMovedRelease, this, &FJManualClassificationDlg::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved, this, &FJManualClassificationDlg::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonNoRightMovedRelease, this, &FJManualClassificationDlg::closeRectangle);
		connect(m_associatedWin, &ccGLWindow::buttonNoRightMovedRelease, this, &FJManualClassificationDlg::closeLasso);
		connect(m_associatedWin, &ccGLWindow::segmentPositionChanged, this, &FJManualClassificationDlg::updatePolylinePoint);

		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(m_associatedWin);
		}
		if (m_selectedPoints)
		{
			m_selectedPoints->setDisplay(m_associatedWin);
		}
		if (polyContainer)
		{
			polyContainer->setDisplay(m_associatedWin);
		}
	}

	return true;
}

bool FJManualClassificationDlg::start()
{
	initcurrentStyle();
	initAllClassficationType();
	m_Invertselection = false;
	ccGui::ParamStruct parameters = ccGui::Parameters();
	parameters.m_isManualClassificationOpen = true;
    parameters.m_ignoreSelectPoint = true;
	ccGui::Set(parameters);
	radioButton_selectall->setChecked(false);
	assert(m_polyVertices && m_segmentationPoly && m_selectedPoints);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}
	razButton->setDisabled(true);
	pushButton->setEnabled(false);
	m_cashTable.clear();
	m_segmentationPoly->clear();
	m_polyVertices->clear();
	m_selectedPoints->clear();

	polyContainer->removeAllChildren();

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->addToOwnDB(m_selectedPoints);
	m_associatedWin->addToOwnDB(polyContainer);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);

    QSet<ccHObject*>::iterator it;
    for (it = m_toSegment.begin(); it != m_toSegment.end(); ++it) {
        ccPointCloud* trackcloud = dynamic_cast<ccPointCloud*>(*it);
        if (trackcloud)
        {
            float size = static_cast<float>(trackcloud->getPointSize());
            bool autoPointSize = trackcloud->getPointSizeAdaptively();
            if (m_selectedPoints) {
                m_selectedPoints->setPointSize(size);
                m_selectedPoints->setPointSizeAdaptively(autoPointSize);
				m_selectedPoints->setCloudBigPointSize(3.0f);
            }
        }
    }
	qDebug() << "set cloud size success!";
	pauseSegmentationMode(true);


	m_rectangularSelection = true;

	reset();
	m_segmentMode = NOPICKINGMODE;
	return ccOverlayDialog::start();
}

void FJManualClassificationDlg::stop(bool accepted)
{
	ccGui::ParamStruct parameters = ccGui::Parameters();
	parameters.m_isManualClassificationOpen = false;
    parameters.m_ignoreSelectPoint = false;
	ccGui::Set(parameters);
	m_cashTable.clear();
	assert(m_segmentationPoly);
	assert(polyContainer);
	clearAllClassficationType();
	if (m_associatedWin)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		m_associatedWin->removeFromOwnDB(m_segmentationPoly);
		m_associatedWin->removeFromOwnDB(polyContainer);
		m_selectedPoints->clear();
		m_associatedWin->removeFromOwnDB(m_selectedPoints);
	}
	ccOverlayDialog::stop(accepted);
}

void FJManualClassificationDlg::updatePolylinePoint(int x, int y)
{
    if (polyContainer->getChildrenNumber() == 0 && m_3DPointData.size() == 0)
    {
        return;
    }
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;
	int childNum = polyContainer->getChildrenNumber();
	ccHObject* child = nullptr;
	for (int i = 0; i < polyContainer->getChildrenNumber(); i++)
	{
		child = polyContainer->getChild(i);
		if (child && child->getClassID() == CC_TYPES::POLY_LINE)
		{
			ccPolyline* curPoly = ccHObjectCaster::ToPolyline(child);
			if (curPoly)
			{
				std::vector<CCVector3d> data = curPoly->get3DPointData();
				int size = data.size();
				for (int j = 0; j < size; j++)
				{
					CCVector3d Q3D = data[j];
					CCVector3d Q2D;
					camera.project(Q3D, Q2D);
					CCVector3d P;
					P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
					P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
					P.z = 0;
					curPoly->setPointVec(j, P);
				}
			}
		}
	}
	if (m_segmentationPoly)
	{
		int size = m_3DPointData.size();
		for (int j = 0; j < size; j++)
		{
			CCVector3d Q3D = m_3DPointData[j];
			CCVector3d Q2D;
			camera.project(Q3D, Q2D);
			CCVector3d P;
			P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
			P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
			P.z = 0;
			m_segmentationPoly->setPointVec(j, P);
		}
		updatePolyLine(x, y, nullptr);
	}
	m_associatedWin->redraw(true, false);
}

void FJManualClassificationDlg::reset()
{
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
		m_associatedWin->releaseMouse();
	}
}

void FJManualClassificationDlg::cancel()
{
	if (m_selectedPoints->size()>0)
	{
		if (CS::Widgets::FramelessMessageBox::warning(MainWindow::TheInstance(),
			tr("Tip"),
			tr("Your selections will not be saved. Are you sure you want to exit?"),
			QMessageBox::Ok |
			QMessageBox::Cancel) != QMessageBox::Ok)
		{
			return;
		}
	}
	reset();
	stop(false);
}

void FJManualClassificationDlg::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Escape:
		removeLastStep();
		return;
	default:
		break;
	}
}

void FJManualClassificationDlg::removeLastStep()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_selectedPoints);
	assert(m_segmentationPoly);

	//if ((m_state & POLYLINE) == POLYLINE)
	//{
	//	closePolyLine();
	//	return;
	//}

	if (((m_state & RUNNING) != RUNNING) && (m_state & POLYLINE) == POLYLINE)
		return;
	m_segmentationPoly->clear();
	m_polyVertices->clear();
	m_3DPointData.clear();
	m_state &= (~RUNNING);
	if (m_associatedWin)
	{
		m_associatedWin->releaseMouse();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_associatedWin->redraw(true, false);
}


void FJManualClassificationDlg::removeAllEntities(bool unallocateVisibilityArrays)
{
	m_toSegment.clear();
}

void FJManualClassificationDlg::doSetPolylineSelection()
{
	if (m_segmentMode != POLYLINEMODE)
	{
		removeLastStep();
		m_state = STARTED;
	}
	closeButtonState();
	setActionIcon(actionSetPolylineSelectionbtn, "manualclassificationpolineclicked", "manualclassificationpolineclicked", "manualclassificationpolinedisabled");
	m_segmentMode = POLYLINEMODE;
	m_rectangularSelection = false;
}

void FJManualClassificationDlg::doSetRectangularSelection()
{
	if (m_segmentMode != RECTANGULAR)
	{
		removeLastStep();
		m_state = STARTED;
	}
	closeButtonState();
	setActionIcon(actionSetRectangularSelectionbtn, "manualclassificationrectclicked", "manualclassificationrectclicked", "manualclassificationrectdisable");
	m_segmentMode = RECTANGULAR;
	m_rectangularSelection = true;
}

void FJManualClassificationDlg::doSetPolylineUpSelection()
{
	if (m_segmentMode != LINEUPMODE)
	{
		removeLastStep();
		m_state = STARTED;
	}
	closeButtonState();
	setActionIcon(toolButton_lineup, "manualclassificationuplineclicked", "manualclassificationuplineclicked", "manualclassificationuplinedisabled");
	m_segmentMode = LINEUPMODE;
	m_rectangularSelection = false;
}

void FJManualClassificationDlg::doSetPolylineDownSelection()
{
	if (m_segmentMode != LINEDOWNMODE)
	{
		removeLastStep();
		m_state = STARTED;
	}
	closeButtonState();
	setActionIcon(toolButton_linedown, "manualclassificationdownlineclicked", "manualclassificationdownlineclicked", "manualclassificationdownlinedisabled");
	m_segmentMode = LINEDOWNMODE;
	m_rectangularSelection = false;
}

void FJManualClassificationDlg::doSetPolylineLassoSelection()
{
	if (m_segmentMode != LINELASSOMODE)
	{
		removeLastStep();
		m_state = STARTED;
	}
	closeButtonState();
	setActionIcon(toolButton_lasso, "manualclassificationscopeclicked", "manualclassificationscopeclicked", "manualclassificationscopedisabled");
	m_segmentMode = LINELASSOMODE;
	m_rectangularSelection = false;
}

void FJManualClassificationDlg::pauseSegmentationMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly && polyContainer && m_selectedPoints);

	if (!m_associatedWin)
		return;

	if (state/*=activate pause mode*/)
	{
		m_state = PAUSED;
		if (m_polyVertices->size() != 0)
		{
			m_segmentationPoly->clear();
			m_polyVertices->clear();
		}
		if (polyContainer->getChildrenNumber() > 0)
		{
			polyContainer->removeAllChildren();
		}
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	}
	else
	{
		m_state = STARTED;
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_SEGMENT);
	}
	m_associatedWin->redraw(!state);
}







void FJManualClassificationDlg::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	if (m_segmentMode == NOPICKINGMODE)
	{
		return;
	}
	//process not started yet?
	if ((m_state & RUNNING) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned vertCount = m_polyVertices->size();

	//new point (expressed relatively to the screen center)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (vertCount != 4)
			m_polyVertices->resize(4);

		const CCVector3* A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3* B = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3* C = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3* D = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x, P.y, 0);
		*C = P;
		*D = CCVector3(P.x, A->y, 0);

		if (vertCount != 4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0, 4))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE || m_state & POLYLINEUP || m_state & POLYLINEDOWN)
	{
		if (vertCount < 2)
			return;
		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
		*lastP = P;
		if (m_state & POLYLINE)
		{
			m_segmentationPoly->setClosed(true);
		}
		else
		{
			m_segmentationPoly->setClosed(false);
		}
	}
	else if(m_state & POLYLINELASSO)
	{
		if (!m_polyVertices->reserve(vertCount + 1))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		ccGLCameraParameters camera;
		m_associatedWin->getGLCameraParameters(camera);
		const double half_w = camera.viewport[2] / 2.0;
		const double half_h = camera.viewport[3] / 2.0;
		CCVector3d Q3D;
		CCVector3d ppp(P.x + half_w, P.y + half_h, P.z);
		bool pointInFrustum = false;
		camera.unproject(ppp, Q3D);
		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
		*lastP = P;
		m_3DPointData[m_3DPointData.size() - 1] = Q3D;
		//and add a new (equivalent) one
		m_3DPointData.push_back(Q3D);
		m_polyVertices->addPoint(P);
		if (!m_segmentationPoly->addPointIndex(vertCount))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		m_segmentationPoly->setClosed(true);
	}

	m_associatedWin->redraw(true, true);
}

void FJManualClassificationDlg::addPointToPolylineExt(int x, int y, bool allowClicksOutside)
{
	if ((m_state & STARTED) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (!allowClicksOutside
		&& (x < 0 || y < 0 || x >= m_associatedWin->qtWidth() || y >= m_associatedWin->qtHeight())
		)
	{
		//ignore clicks outside of the 3D view
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned vertCount = m_polyVertices->size();

	//particular case: we close the rectangular selection by a 2nd click
	if (m_rectangularSelection && vertCount == 4 && (m_state & RUNNING))
		return;
	if ((m_state & POLYLINELASSO) && vertCount > 0 && (m_state & RUNNING))
		return;

	//new point
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);





	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;
	CCVector3d Q3D;
	CCVector3d ppp(P.x + half_w, P.y + half_h, P.z);
	bool pointInFrustum = false;
	camera.unproject(ppp, Q3D);
	//CTRL key pressed at the same time?

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_SEGMENT);
		//reset state
		m_state = getCurrentSelectedMode();
		m_state |= (STARTED | RUNNING);
		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_3DPointData.push_back(Q3D);
		m_3DPointData.push_back(Q3D);
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE || m_state & POLYLINEUP || m_state & POLYLINEDOWN)
		{
			if (!m_polyVertices->reserve(vertCount + 1))
			{
				ccLog::Error("Out of memory!");
				return;
			}

			//we replace last point by the current one
			CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
			*lastP = P;
			m_3DPointData[m_3DPointData.size() - 1] = Q3D;
			//and add a new (equivalent) one
			m_3DPointData.push_back(Q3D);
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			if (m_state & POLYLINE)
			{
				m_segmentationPoly->setClosed(true);
			}
			else
			{
				m_segmentationPoly->setClosed(false);
			}
		}
		else //we must change mode
		{
			assert(false); //we shouldn't fall here?!
			m_state &= (~RUNNING);
			addPointToPolylineExt(x, y, allowClicksOutside);
			return;
		}
	}

	//DGM: to increase the poll rate of the mouse movements in ccGLWindow::mouseMoveEvent
	//we have to completely grab the mouse focus!
	//(the only way to take back the control is to right-click now...)
	m_associatedWin->grabMouse();
	m_associatedWin->redraw(true, true);

	m_isMousePressMoveValid = false;
	QTimer::singleShot(300, this, [&]() {
		m_isMousePressMoveValid = true;
	});
}

void FJManualClassificationDlg::closeRectangle()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & RECTANGLE) == 0 || (m_state & RUNNING) == 0)
	{
		if (m_associatedWin)
		{
			m_associatedWin->releaseMouse();
		}
		return;
	}
	if (!m_isMousePressMoveValid)
	{
		if (m_associatedWin)
		{
			m_associatedWin->releaseMouse();
		}
		return;
	}

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		//first point only? we keep the real time update mechanism
		if (m_rectangularSelection)
			return;
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		m_3DPointData.clear();
	}
	else
	{

		//janson.yang
		std::vector < CCVector3d> pointData;
		int pointNum = m_segmentationPoly->size();
		for (auto i = 0; i < pointNum; i++)
		{
			CCVector3 pp;
			m_segmentationPoly->getPoint(i,pp);
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;
			CCVector3d Q3D;
			CCVector3d ppp(pp.x + half_w, pp.y + half_h, pp.z);
			bool pointInFrustum = false;
			camera.unproject(ppp, Q3D);
			pointData.push_back(Q3D);
		}
		ccPolyline* poly = new ccPolyline(*m_segmentationPoly);
		poly->set3DPointData(pointData);



		poly->showVertices(false);
		polyContainer->addChild(poly);
		m_segmentationPoly->clear();
		m_3DPointData.clear();
	}



	//stop
	m_state &= (~RUNNING);

	calculateManualClassification();
	if (m_associatedWin)
	{
		m_associatedWin->releaseMouse();
		m_associatedWin->redraw(false, true);
	}
}

void FJManualClassificationDlg::closeLasso()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & POLYLINELASSO) == 0 || (m_state & RUNNING) == 0)
	{
		if (m_associatedWin)
		{
			m_associatedWin->releaseMouse();
		}
		return;
	}
	if (!m_isMousePressMoveValid)
	{
		return;
	}

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount > 0)
	{
		std::vector < CCVector3d> pointData;
		int pointNum = m_segmentationPoly->size();
		for (auto i = 0; i < pointNum; i++)
		{
			CCVector3 pp;
			m_segmentationPoly->getPoint(i, pp);
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;
			CCVector3d Q3D;
			CCVector3d ppp(pp.x + half_w, pp.y + half_h, pp.z);
			bool pointInFrustum = false;
			camera.unproject(ppp, Q3D);
			pointData.push_back(Q3D);
		}
		ccPolyline* poly = new ccPolyline(*m_segmentationPoly);
		poly->set3DPointData(pointData);



		poly->showVertices(false);
		polyContainer->addChild(poly);
		m_segmentationPoly->clear();
		m_3DPointData.clear();
	}
    

	//stop
	m_state &= (~RUNNING);

	calculateManualClassification();
	if (m_associatedWin)
	{
		m_associatedWin->releaseMouse();
		m_associatedWin->redraw(false, true);
	}
}

void FJManualClassificationDlg::closePolyLine(int, int)
{
	if ((m_state & RECTANGLE) == RECTANGLE)
	{
		removeLastStep();
		return;
	}
	//only for polyline in RUNNING mode
	if (((m_state & POLYLINE) == 0 && (m_state & POLYLINEUP) == 0 && (m_state & POLYLINEDOWN) == 0) || (m_state & RUNNING) == 0)
		return;

	//if (m_associatedWin)
	//{
	//	m_associatedWin->releaseMouse();
	//}
	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	//if (vertCount < 4)
	if (vertCount < 3)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		m_3DPointData.clear();
	}
	else
	{
		//remove last point!
		m_segmentationPoly->resize(vertCount - 1); //can't fail --> smaller
		//m_segmentationPoly->setClosed(true);


		if (m_state & POLYLINEUP || m_state & POLYLINEDOWN)
		{
			if (!m_polyVertices->reserve(vertCount + 1))
			{
				ccLog::Error("Out of memory!");
				return;
			}

			CCVector3 firstPoint;
			CCVector3 lastPoint;
			m_polyVertices->getPoint(0, firstPoint);
			m_polyVertices->getPoint(vertCount - 2, lastPoint);
			if (m_state & POLYLINEUP)
			{
				firstPoint.y = 99999;
				lastPoint.y = 99999;
			}
			else
			{
				firstPoint.y = -99999;
				lastPoint.y = -99999;
			}
			CCVector3* lastP2 = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount-1));
			*lastP2 = firstPoint;
			CCVector3* lastP1 = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount));
			*lastP1 = lastPoint;
			if (!m_segmentationPoly->addPointIndex(vertCount-1, vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
		}


		//janson.yang
		std::vector < CCVector3d> pointData;
		int pointNum = m_segmentationPoly->size();
		for (auto i =0;i < pointNum;i++)
		{
			CCVector3 pp;
			m_segmentationPoly->getPoint(i,pp);
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;
			CCVector3d Q3D;
			CCVector3d ppp(pp.x + half_w, pp.y + half_h, pp.z);
			bool pointInFrustum = false;
			camera.unproject(ppp, Q3D);
			pointData.push_back(Q3D);
		}
		ccPolyline* poly = new ccPolyline(*m_segmentationPoly);
		poly->set3DPointData(pointData);
		poly->showVertices(false);
		polyContainer->addChild(poly);
		m_3DPointData.clear();
	}

	//stop
	m_state &= (~RUNNING);

	//set the default import/export icon to 'export' mode
	//loadSaveToolButton->setDefaultAction(actionExportSegmentationPolyline);
	m_segmentationPoly->clear();


	calculateManualClassification();
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false, true);
	}
}


void FJManualClassificationDlg::calculateManualClassification()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (!m_segmentationPoly || !polyContainer || polyContainer->getChildrenNumber() == 0)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;


	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

		int cloudSize = static_cast<int>(cloud->size());

		ccGenericPointCloud::VisibilityTableType selectionTable;
		ccHObject *ccbox = cloud->getClippingBox();
		if (ccbox)
		{
			ccClipBox* box = static_cast<ccClipBox*>(ccbox);
			if (box)
			{
				try
				{
					selectionTable.resize(cloudSize);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Error("Not enough memory!");
				}
				box->flagPointsInside(cloud, &selectionTable);
			}
		}
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(*p);
		if (!pc)
		{
			ccLog::Warning("Can't apply classification to cloud " + (*p)->getName());
			continue;
		}
		int sfIdx = pc->getScalarFieldIndexByName("Classification");
		if (sfIdx < 0)
		{
			// create the scalar field Classification if needed
			sfIdx = pc->addScalarField("Classification");
			if (sfIdx < 0)
			{
				ccLog::Error(tr("Not enough memory"));
				return;
			}
		}
		CCCoreLib::ScalarField* classifSF = pc->getScalarField(sfIdx);

		bool isHasSelection = (m_cashTable.size() > 0) ? true : false;
		ccGenericPointCloud::VisibilityTableType oldCashTable = m_cashTable;

		try
		{
			m_cashTable.resize(cloudSize);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory!");
		}



		//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < cloudSize; ++i)
		{
            const CCVector3* P3D = cloud->getPoint(i);

			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P3D, Q2D, &pointInFrustum);

			bool pointInside = false;
			//屏幕外的点也计算在内carl12051519
			//if (pointInFrustum || !polyInsideFrustum) //we can only skip the test if the polyline is fully inside the frustum
			{
				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
					static_cast<PointCoordinateType>(Q2D.y - half_h));
				//janson.yang 2022.5.18
				ccHObject* child = nullptr;
				for (int i = 0; i < polyContainer->getChildrenNumber(); i++)
				{
					child = polyContainer->getChild(i);
					if (child && child->getClassID() == CC_TYPES::POLY_LINE)
					{
						ccPolyline* curPoly = ccHObjectCaster::ToPolyline(child);
						assert(curPoly);

						pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, curPoly);

						if (pointInside)
							break;
					}
				}
			}

			if (selectionTable.size() > 0)
			{
				if (m_Invertselection)
				{
					m_cashTable[i] = CCCoreLib::POINT_HIDDEN;
					if (isHasSelection && !pointInside && oldCashTable[i] == CCCoreLib::POINT_VISIBLE)
					{
						m_cashTable[i] = CCCoreLib::POINT_VISIBLE;
					}
					if (isHasSelection)
					{
						if (m_cashTable[i] != oldCashTable[i] && selectionTable[i] == CCCoreLib::POINT_HIDDEN)
						{
							m_cashTable[i] = oldCashTable[i];
						}
					}
					else
					{
						if (m_cashTable[i] == CCCoreLib::POINT_VISIBLE && selectionTable[i] == CCCoreLib::POINT_HIDDEN)
						{
							m_cashTable[i] = CCCoreLib::POINT_HIDDEN;
						}
					}
				}
				else
				{
					m_cashTable[i] = (pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
					if (isHasSelection && oldCashTable[i] == CCCoreLib::POINT_VISIBLE)
					{
						m_cashTable[i] = CCCoreLib::POINT_VISIBLE;
					}
					if (isHasSelection)
					{
						if (m_cashTable[i] != oldCashTable[i] && selectionTable[i] == CCCoreLib::POINT_HIDDEN)
						{
							m_cashTable[i] = oldCashTable[i];
						}
					}
					else
					{
						if (m_cashTable[i] == CCCoreLib::POINT_VISIBLE && selectionTable[i] == CCCoreLib::POINT_HIDDEN)
						{
							m_cashTable[i] = CCCoreLib::POINT_HIDDEN;
						}
					}
				}
			}
			else
			{
				if (m_Invertselection)
				{
					m_cashTable[i] = CCCoreLib::POINT_HIDDEN;
					if (isHasSelection && !pointInside && oldCashTable[i] == CCCoreLib::POINT_VISIBLE)
					{
						m_cashTable[i] = CCCoreLib::POINT_VISIBLE;
					}
				}
				else
				{
					m_cashTable[i] = (pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
					if (isHasSelection && oldCashTable[i] == CCCoreLib::POINT_VISIBLE)
					{
						m_cashTable[i] = CCCoreLib::POINT_VISIBLE;
					}
				}
			}
		}
		clearAllClassficationType();
		std::set<double> selectedClassfications;
		m_selectedPoints->clear();
		for (int i = 0; i < cloudSize; ++i)
		{
			if (m_cashTable[i] == CCCoreLib::POINT_VISIBLE)
			{
				CCVector3 curPoint;
				cloud->getPoint(i, curPoint);
				m_selectedPoints->addPoint(curPoint);
				selectedClassfications.insert(classifSF->getValue(i));
			}
		}
		if (m_selectedPoints->size() == 0)
		{
			m_Invertselection = false;
			setActionIcon(razButton, "manualclassificationunselectnormal", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
			razButton->setDisabled(true);
		}
		else
		{
			razButton->setDisabled(false);
		}
		m_selectedPoints->setTempColor(ccColor::Rgb(255, 0, 0));
		m_selectedPoints->showColors(true);
		setCurrentClassficationType(selectedClassfications);
	}


	if (m_polyVertices->size() != 0)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	if (polyContainer->getChildrenNumber() > 0)
	{
		polyContainer->removeAllChildren();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);

}

void FJManualClassificationDlg::selectedAllClassfication()
{
	bool isSelectedAll = radioButton_selectall->isChecked();
	for (auto box : m_checkBoxs)
	{
		box->setChecked(isSelectedAll);
	}
}


void FJManualClassificationDlg::clearAllClassficationType()
{
	if (m_layout)
	{
		for (auto box : m_checkBoxs)
		{
			m_layout->removeWidget(box);
			delete box;
			box = nullptr;
		}
		m_checkBoxs.clear();
		m_currentSelectType.clear();
		frame->setMaximumSize(99999, 99999);
		frame->setMinimumSize(0, 0);
	}
	pushButton->setEnabled(false);
	radioButton_selectall->setChecked(false);
}

void FJManualClassificationDlg::setCurrentClassficationType(const std::set<double> & selectedClassfications)
{
	int row = 0;
	int column = 0;
	std::vector<ClassficationData> savedData = FJPointCloudUtil::getClassficationDataFromDoc();
	std::map<int, QString> idToNameMap;
	for (auto curTypeData : savedData)
	{
		idToNameMap[curTypeData.id] = curTypeData.name;
	}
	int allrow = selectedClassfications.size() / 4;
	if ((selectedClassfications.size() % 4) != 0)
	{
		allrow++;
	}
	if (allrow >3)
	{
		frame->setFixedSize(412, 25 * allrow + 12);
	}
	for (auto curClassficationSet : selectedClassfications)
	{
		QCheckBox * newCheckBox = new QCheckBox(frame);
		newCheckBox->setText(QString::number(curClassficationSet));
		newCheckBox->setToolTip(QString::number(curClassficationSet));
		newCheckBox->setFixedSize(100,25);
		if (curClassficationSet == 0)
		{
			newCheckBox->setChecked(true);
			pushButton->setEnabled(true);
		}
		if (idToNameMap.find(int(curClassficationSet)) != idToNameMap.end())
		{
			newCheckBox->setText(idToNameMap[curClassficationSet]);
			newCheckBox->setToolTip(idToNameMap[curClassficationSet]);
		}
		connect(newCheckBox, QOverload<int>::of(&QCheckBox::stateChanged),this,&FJManualClassificationDlg::checkboxStateChanged);
		m_checkBoxs.push_back(newCheckBox);
		m_currentSelectType.push_back(curClassficationSet);
		m_layout->addWidget(newCheckBox, row, column,Qt::AlignLeft);
		if (column != 3)
		{
			column++;
		}
		else
		{
			row++;
			column = 0;
		}
	}
	checkboxStateChanged(0);
}

void FJManualClassificationDlg::checkboxStateChanged(int)
{
	bool isAllCheckBoxChecked = true;
	bool isHasOneCheckBoxChecked = false;
	for (int i = 0; i < m_checkBoxs.size(); ++i)
	{
		if (m_checkBoxs[i]->isChecked())
		{
			isHasOneCheckBoxChecked = true;
		}
		else
		{
			isAllCheckBoxChecked = false;
		}
	}
	radioButton_selectall->blockSignals(true);
	radioButton_selectall->setChecked(isAllCheckBoxChecked);
	if (m_checkBoxs.size() == 0)
	{
		radioButton_selectall->setChecked(false);
	}
	radioButton_selectall->blockSignals(false);
	pushButton->setEnabled(isHasOneCheckBoxChecked);
}

unsigned FJManualClassificationDlg::getCurrentSelectedMode()
{
	unsigned state = PAUSED;
    if (m_segmentMode == POLYLINEMODE)
    {
        addOverriddenShortcut(Qt::Key_Control);
        addOverriddenShortcut(Qt::Key_Z);
    }
    else
    {
        removeOverriddenShortcut(Qt::Key_Control);
        removeOverriddenShortcut(Qt::Key_Z);
    }
	switch (m_segmentMode)
	{
	case FJManualClassificationDlg::RECTANGULAR:
		state = RECTANGLE;
		break;
	case FJManualClassificationDlg::POLYLINEMODE:
		state = POLYLINE;
		break;
	case FJManualClassificationDlg::LINEUPMODE:
		state = POLYLINEUP;
		break;
	case FJManualClassificationDlg::LINEDOWNMODE:
		state = POLYLINEDOWN;
		break;
	case FJManualClassificationDlg::LINELASSOMODE:
		state = POLYLINELASSO;
		break;
	case FJManualClassificationDlg::NOPICKINGMODE:
		state = PAUSED;
		break;
	default:
		break;
	}
	return state;
}

void FJManualClassificationDlg::initAllClassficationType()
{
	comboBox_name->clear();
	std::vector<ClassficationData> data = FJPointCloudUtil::getClassficationDataFromDoc();
	int currentIndex = 0;
	for (auto curType : data)
	{
		//comboBox_name->addItem(curType.name);
		comboBox_name->insertItem(currentIndex,curType.name,QString::number(curType.id));
		currentIndex++;
	}
	updateTypeColor();
}

void FJManualClassificationDlg::updateTypeColor()
{
	int index = comboBox_name->currentIndex();
	int id = (comboBox_name->itemData(index, Qt::UserRole)).toInt();
	QColor color = FJPointCloudUtil::getColorByClassficationID(id);
	SetButtonColor(toolButton_typecolor, color);
}

void FJManualClassificationDlg::changeClassficationColor()
{
	int index = comboBox_name->currentIndex();
	int id = (comboBox_name->itemData(index, Qt::UserRole)).toInt();
	QColor oldcolor = FJPointCloudUtil::getColorByClassficationID(id);
	CS::Widgets::FramelessDialog outdlg(MainWindow::TheInstance());
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
	QColorDialog colordlg(oldcolor, this);
	connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
	colordlg.setOptions(QColorDialog::NoButtons);
	outdlg.SetContentHolder(&colordlg);
	if (!outdlg.exec())
	{
		return;
	}
	QColor newCol = colordlg.currentColor();
	if (newCol.isValid())
	{
		FJPointCloudUtil::setColorByClassficationID(id, newCol);
		SetButtonColor(toolButton_typecolor, newCol);
		FJPointCloudUtil::updateAllPointCloudClassficationDataFromDoc();
		if (MainWindow::TheInstance()->getActiveGLWindow())
		{
			MainWindow::TheInstance()->getActiveGLWindow()->redraw(false);
		}
	}
}

void FJManualClassificationDlg::applyClassfication()
{
	int cloudSize = m_cashTable.size();
	if (cloudSize == 0)
	{
		return;
	}
	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(*(m_toSegment.begin()));
	if (!pc)
	{
		return;
	}
	int sfIdx = pc->getScalarFieldIndexByName("Classification");
	if (sfIdx < 0)
	{
		return;
	}
	std::set<double> choiseTypes;
	for (int i = 0; i < m_checkBoxs.size(); ++i)
	{
		if (m_checkBoxs[i]->isChecked())
		{
			choiseTypes.insert(m_currentSelectType[i]);
		}
	}
	CCCoreLib::ScalarField* classifSF = pc->getScalarField(sfIdx);
	int cobmindex = comboBox_name->currentIndex();
	int toType = (comboBox_name->itemData(cobmindex, Qt::UserRole)).toInt();
	for (int i = 0; i < cloudSize; ++i)
	{
		if (m_cashTable[i] == CCCoreLib::POINT_VISIBLE)
		{
			if (choiseTypes.find(classifSF->getValue(i)) != choiseTypes.end())
			{
				classifSF->setValue(i, float(toType));
			}
		}
	}
	classifSF->computeMinAndMax();
	FJPointCloudUtil::updateClassficationDataFromDoc(*(m_toSegment.begin()));
	clearAllClassficationType();
	m_selectedPoints->clear();
	m_Invertselection = false;
	setActionIcon(razButton, "manualclassificationunselectnormal", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
	razButton->setDisabled(true);
	m_cashTable.clear();
	pushButton->setEnabled(false);
    pc->scaleFieldColorChange();
	m_associatedWin->redraw(false);
}

void FJManualClassificationDlg::clearALL()
{
	clearAllClassficationType();
	m_selectedPoints->clear();
	m_Invertselection = false;
	setActionIcon(razButton, "manualclassificationunselectnormal", "manualclassificationunselectlicked", "manualclassificationunselectdisable");
	razButton->setDisabled(true);
	m_cashTable.clear();
	m_associatedWin->redraw(false, true);

}

void FJManualClassificationDlg::setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
	action->setIconPixmap(normalPix, clickedPix, disabledPix);
}
void FJManualClassificationDlg::onShortcutDoubleBondTriggered(int, Qt::KeyboardModifiers)
{
    if (!m_segmentationPoly || !m_polyVertices) {
        return;
    }
    int index = m_segmentationPoly->size();
    int indexPointSize = m_polyVertices->size();

    if (index < 1 && indexPointSize < 1) {
        return;
    }
    ccGLWindow *glWindow = MainWindow::TheInstance()->getActiveGLWindow();
    if (!glWindow) {
        return;
    }
    if (index == 2 && indexPointSize == 2)
    {
        m_polyVertices->clear();
        m_3DPointData.clear();
        m_segmentationPoly->clear();
        glWindow->redraw(false);
        return;
    }

    m_segmentationPoly->removePointGlobalIndex(index - 1);
    m_polyVertices->resize(indexPointSize - 1);
    m_3DPointData.pop_back();
    QPoint globalPos = QCursor::pos();
    QPoint localPos = glWindow->mapFromGlobal(globalPos);
    int x = localPos.x();
    int y = localPos.y();
    updatePolyLine(x, y, Qt::MouseButton::NoButton);
    glWindow->redraw(false);

}