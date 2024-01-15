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

#include "ccPointPropertiesDlg.h"

//Local
#include "ccCommon.h"
#include "ccGLWindow.h"
#include "ccGuiParameters.h"
#include "ccReservedIDs.h"
#include "mainwindow.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <cc2DViewportLabel.h>
#include <cc2DLabel.h>
#include <FJStyleManager.h>
#include <ccClipBox.h>

//CCCoreLib
#include <ScalarField.h>

//Qt
#include <QInputDialog>

//System
#include <assert.h>


ccPointPropertiesDlg::ccPointPropertiesDlg(ccPickingHub* pickingHub, QWidget* parent)
	: ccPointPickingGenericInterface(pickingHub, parent)
	, Ui::PointPropertiesDlg()
{
	setupUi(this);

	InitStyle();
	
	connect(point_info_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		point_info_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/zuobiaodian@2x.png"));
		activatePointPropertiesDisplay();
		});
	connect(point_point_distance_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		point_point_distance_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/DistancePicking@2x.png"));
		activateDistanceDisplay();
		});
	connect(points_angle_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		points_angle_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/angleclick@2x.png"));
		activateAngleDisplay();
		});
	connect(height_dis_in_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		height_dis_in_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/heightdismeasure@2x.png"));
		activeHeightDisplay();
		});
	connect(areaMeasure_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		areaMeasure_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/aerameasure@2x.png"));
		activeAeraDisplay();
		});

	connect(revoke_action, &QToolButton::clicked, [this]() {
		initializeState();

		});
	connect(save_action, &QToolButton::clicked, [this]() {
		exportCurrentLabel();
		});
	connect(remove_all_measure_action_, &QToolButton::clicked, [this]() {
		InitStyle();
		if (m_associatedWin)
		{
			m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
			m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		}
		stop(true);
		});


	//for points picking
	m_label = new cc2DLabel();
	m_label->setSelected(true);

	m_measureLine = new ccMeasureLine();

	//for 2D zone picking

	//体积测量框
	m_clipBox = new ccClipBox(QString("clip box"), static_cast<unsigned>(ReservedIDs::CLIPPING_BOX));
	m_clipBox->setVisible(true);
	m_clipBox->setEnabled(true);
	m_clipBox->setSelected(true); 
	m_clipBox->setUseClipPlanes(false);
	addOverriddenShortcut(Qt::Key_Escape);
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccPointPropertiesDlg::onShortcutTriggered);
}

void ccPointPropertiesDlg::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Escape:
		initializeState();
		return;
	default:
		break;
	}
}

void ccPointPropertiesDlg::InitStyle()
{
	//单点测量


	QString m_DirName = FJStyleManager::Instance()->Getcurrentthemepath();
	QString action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Point", nullptr);
	QString action_icon_path = QString(m_DirName + "qssimage/locationPoint2x.png");
	QString objectName = "actionPointPicking";
	createAction(point_info_action_,action_text, action_icon_path, m_DirName + "qssimage/clickedpng/zuobiaodian@2x.png", m_DirName + "qssimage/disabledpng/zuobiaodian@2x.png", objectName);

	//距离测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Distance", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/distance@2x.png");
	objectName = "actionDistancePicking";
	createAction(point_point_distance_action_,action_text, action_icon_path, m_DirName + "qssimage/clickedpng/DistancePicking@2x.png", m_DirName + "qssimage/disabledpng/DistancePicking@2x.png", objectName);


	//角度测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Angle", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/angleMeasure@2x.png");
	objectName = "actionAnglePicking";
	createAction(points_angle_action_,action_text, action_icon_path, m_DirName + "qssimage/clickedpng/angleclick@2x.png", m_DirName + "qssimage/disabledpng/angledisabled@2x.png", objectName);


	//高度测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Height", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/heightMeasure@2x.png");
	objectName = "actionheightdismeasure";
	createAction(height_dis_in_action_, action_text, action_icon_path, m_DirName + "qssimage/clickedpng/heightdismeasure@2x.png", m_DirName + "qssimage/disabledpng/heightdismeasure@2x.png", objectName);


	//面积测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Area", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/aeraMeasure@2x.png");
	objectName = "actionaerameasure";
	createAction(areaMeasure_action_, action_text, action_icon_path, m_DirName + "qssimage/clickedpng/aerameasure@2x.png", m_DirName + "qssimage/disabledpng/aerameasure@2x.png", objectName);


	//还原测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Undo", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/measurereseticon.png");
	objectName = "actionrevorkpicking";
	createAction(revoke_action, action_text, action_icon_path, action_icon_path, action_icon_path, objectName);

	//保存测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Save", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/measuresaveicon.png");
	objectName = "actionsavepicking";
	createAction(save_action, action_text, action_icon_path, action_icon_path, action_icon_path, objectName);


	//关闭测量
	action_text = QCoreApplication::translate("ccPointPropertiesDlg", "Close", nullptr);
	action_icon_path = QString(m_DirName + "qssimage/measurecloseicon.png");
	objectName = "actionclosepicking";
	createAction(remove_all_measure_action_,action_text, action_icon_path, action_icon_path, action_icon_path, objectName);
}

void ccPointPropertiesDlg::createAction(QToolButton * button, const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName)
{
	button->setToolTip(text);
	QIcon pIconTest(iconurlnormal);
	pIconTest.addPixmap(QPixmap(iconurlclicked), QIcon::Active, QIcon::Off);
	pIconTest.addPixmap(QPixmap(iconurldisabled), QIcon::Disabled, QIcon::Off);

	button->setIcon(pIconTest);
}

ccPointPropertiesDlg::~ccPointPropertiesDlg()
{
	if (m_label)
		delete m_label;
	m_label = nullptr;

	if (m_measureLine)
		delete m_measureLine;
	m_measureLine = nullptr;

	if (m_clipBox)
		delete m_clipBox;
	m_clipBox = nullptr;
}

bool ccPointPropertiesDlg::linkWith(ccGLWindow* win)
{
	assert(m_label && m_clipBox && m_measureLine);

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccPointPickingGenericInterface::linkWith(win))
	{
		return false;
	}

	//old window?
	if (oldWin)
	{
		oldWin->removeFromOwnDB(m_label);
		oldWin->removeFromOwnDB(m_measureLine);
		oldWin->removeFromOwnDB(m_clipBox);
		//oldWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		oldWin->disconnect(this);
	}

	m_label->clear();
	m_label->clearMeasurePos();

	m_clipBox->setVisible(false);

	//new window?
	if (m_associatedWin)
	{
		m_associatedWin->addToOwnDB(m_label);
		m_associatedWin->addToOwnDB(m_clipBox);
		m_associatedWin->addToOwnDB(m_measureLine);
		connect(m_associatedWin, &ccGLWindow::sigTransScreenPosTo3DPos, this, &ccPointPropertiesDlg::slotMouseMovePos);
		connect(m_associatedWin, &ccGLWindow::rightButtonNoMovedRelease, this, &ccPointPropertiesDlg::closePolyLine);
        connect(m_associatedWin, &ccGLWindow::segmentPositionChanged, this, &ccPointPropertiesDlg::updatePolylinePoint);
	}

	

	return true;
}

void ccPointPropertiesDlg::updatePolylinePoint(int x, int y)
{
    if (m_measureMode == MEASURE_VOLUME || m_measureMode == MEASURE_POINT)
    {
        return;
    }
    slotMouseMovePos(CCVector3d(0, 0, 0), QPointF(x, y));
    
}

double ccPointPropertiesDlg::getDistanceMeasureResult()
{
	double result = 0;
	int count = m_label->getMeasurePointsNum();
	if (count >1)
	{
		std::vector<CCVector3d> pointList = m_label->getAreaMeasurePoints();
		for (int i = 1; i < count; i++)
		{
			double dx = pointList[i].x - pointList[i - 1].x;
			double dy = pointList[i].y - pointList[i - 1].y;
			double dz = pointList[i].z - pointList[i - 1].z;
			if (m_pointMode == POINT3DMODE)
			{
				double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
				result += distance;
			}
			else
			{
				double distance = std::sqrt(dx * dx + dy * dy);
				result += distance;
			}
		}
	}

	return result;
}

double ccPointPropertiesDlg::getAreaMeasureResult(std::vector<CCVector3d> pointList)
{
	double Area = 0.0;
	const int Length = pointList.size();
	if (Length < 3)
	{
		return Area;
	}

	// 求解常量
	CCVector3d Point1 = pointList[0], Point2 = pointList[1], Point3 = pointList[2];
	CCVector3d OA = Point1 - Point2;
	CCVector3d OB = Point1 - Point3;

	float Cx = OA.y * OB.z - OA.z * OB.y;
	float Cy = OA.z * OB.x - OA.x * OB.z;
	float Cz = OA.x * OB.y - OA.y * OB.x;

	double a = Cx * Cx + Cy * Cy + Cz * Cz;


	//double a = std::pow(OA.Cross(OB).Length(), 2);	
	const double cosnx = ((Point2.y - Point1.y) * (Point3.z - Point1.z) - (Point3.y - Point1.y) * (Point2.z - Point1.z)) / std::pow(a, 0.5);
	const double cosny = ((Point3.x - Point1.x) * (Point2.z - Point1.z) - (Point2.x - Point1.x) * (Point3.z - Point1.z)) / std::pow(a, 0.5);
	const double cosnz = ((Point2.x - Point1.x) * (Point3.y - Point1.y) - (Point3.x - Point1.x) * (Point2.y - Point1.y)) / std::pow(a, 0.5);

	// 迭代计算面积
	CCVector3d CurPoint, NextPoint;
	for (int i = 0; i < Length; ++i)
	{
		CurPoint = pointList[i];
		NextPoint = pointList[(i + 1) % Length];
		Area += cosnz * (CurPoint.x * NextPoint.y - NextPoint.x * CurPoint.y)
			+ cosnx * (CurPoint.y * NextPoint.z - NextPoint.y * CurPoint.z)
			+ cosny * (CurPoint.z * NextPoint.x - NextPoint.z * CurPoint.x);
	}
	return std::abs(0.5 * Area);

}

void ccPointPropertiesDlg::closePolyLine(int, int)
{
	if (m_measureMode == MEASURE_AERA || m_measureMode == MEASURE_DISTANCE)
	{
		if ((m_measureMode == MEASURE_AERA && m_measureLine->getCurrentPos().size() > 3) || (m_measureMode == MEASURE_DISTANCE && m_label->getMeasurePointsNum() > 1))
		{
			if (m_measureMode == MEASURE_AERA)
			{
				std::vector<CCVector3d> pointList = m_measureLine->getCurrentPos();
				pointList.pop_back();
				m_label->setMeasureResult(getAreaMeasureResult(pointList));
				m_label->setAreaMeasurePoints(pointList);
				if (m_associatedWin)
				{
					ccGLMatrixd viewMat = m_associatedWin->getViewportParameters().viewMat;
					m_label->setDirectionMatrixd(viewMat);
				}
			}
			else if (m_measureMode == MEASURE_DISTANCE)
			{
				m_label->setMeasureResult(getDistanceMeasureResult());
			}
			m_label->setVisible(true);
		}
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_measureLine->clearCurrentPos();
		m_measureLine->setVisible(false);
        m_associatedWin->redraw(false, true);
	}
}

void ccPointPropertiesDlg::slotMouseMovePos(CCVector3d pos, QPointF mousePos)
{
	if (m_measureMode == MEASURE_VOLUME || m_measureMode == MEASURE_POINT)
	{
		return;
	}
    ccGLCameraParameters camera;
    m_associatedWin->getGLCameraParameters(camera);
	bool isHasCloudPoint = true;
	if (fabs(pos.x - 0) < 0.00000001 && fabs(pos.z - 0) < 0.00000001 && fabs(pos.z - 0) < 0.00000001)
	{
		isHasCloudPoint = false;
	}
	switch (m_measureMode)
	{
	case MEASURE_DISTANCE:
		if (m_label->getMeasurePointsNum() > 0 && m_measureLine->isVisible())
		{
			m_measureLine->setLabelVisiable(isHasCloudPoint);
            if (isHasCloudPoint)
            {
                m_measureLine->setLastCurrentPos(pos);
            }
            else
            {
                m_measureLine->setLastCurrentPos(getPositionByScreenPos(mousePos));
            }
            std::vector < CCVector3d>  points2d;
            std::vector<CCVector3d> point3d= m_measureLine->getCurrentPos();
            CCVector3 P3D = point3d[point3d.size()-2].toFloat();
            CCVector3d P2D;
            camera.project(P3D, P2D);
            points2d.push_back(P2D);
            if (isHasCloudPoint)
            {
                camera.project(pos, P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
            }
            else
            {
                camera.project(getPositionByScreenPos(mousePos), P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
            }
		}
		break;
	case MEASURE_HEIGHT:
		if (m_label->getMeasurePointsNum() == 1 && m_measureLine->isVisible())
		{
			m_measureLine->setLabelVisiable(isHasCloudPoint);
            if (isHasCloudPoint)
            {
                m_measureLine->setLastCurrentPos(pos);
            }
            else
            {
                m_measureLine->setLastCurrentPos(getPositionByScreenPos(mousePos));
            }
            std::vector < CCVector3d>  points2d;

            CCVector3 P3D = (m_measureLine->getCurrentPos())[0].toFloat();
            CCVector3d P2D;
            camera.project(P3D, P2D);
            points2d.push_back(P2D);
			if (isHasCloudPoint)
			{
                camera.project(pos, P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
			}
			else
			{
                camera.project(getPositionByScreenPos(mousePos), P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
			}
		}
		break;
	case MEASURE_ANGLE:
		if (m_label->getMeasurePointsNum() > 0 && m_label->getMeasurePointsNum() < 3 && m_measureLine->isVisible())
		{
			m_measureLine->setLabelVisiable(isHasCloudPoint);
			if (isHasCloudPoint)
			{
				m_measureLine->setLastCurrentPos(pos);
			}
			else
			{
				m_measureLine->setLastCurrentPos(getPositionByScreenPos(mousePos));
			}
            std::vector < CCVector3d>  points2d;
            std::vector<CCVector3d> point3d = m_measureLine->getCurrentPos();
            CCVector3 P3D = point3d[0].toFloat();
            CCVector3d P2D;
            camera.project(P3D, P2D);
            points2d.push_back(P2D);
            if (point3d.size()==3)
            {
                P3D = point3d[1].toFloat();
                camera.project(P3D, P2D);
                points2d.push_back(P2D);
            }
            if (isHasCloudPoint)
            {
                camera.project(pos, P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
            }
            else
            {
                camera.project(getPositionByScreenPos(mousePos), P2D);
                points2d.push_back(P2D);
                m_measureLine->set2dPoint(points2d);
            }
		}
		break;
	case MEASURE_AERA:
		if (m_measureLine->isVisible() && m_measureLine->getCurrentPos().size()>0)
		{
			m_measureLine->setLabelVisiable(isHasCloudPoint);
			if (m_measureLine->getCurrentPos().size() > 3)
			{
				m_measureLine->setLastCurrentPos(getCurrentDirectionMousePos(getPositionByScreenPos(QPointF(mousePos.x(), mousePos.y()))));
			}
			else
			{
				m_measureLine->setLastCurrentPos((getPositionByScreenPos(QPointF(mousePos.x(), mousePos.y()))));
			}
			std::vector<CCVector3d> pointList = m_measureLine->getCurrentPos();
			m_measureLine->setMeasureAreaStr("area:" + QString::number(getAreaMeasureResult(pointList), 'f', 3));
            std::vector < CCVector3d>  points2d;
            CCVector3d P2D;
			int pointnum = pointList.size();
			camera.project(pointList[pointnum-2], P2D);
			points2d.push_back(P2D);
			camera.project(getPositionByScreenPos(QPointF(mousePos.x(), mousePos.y())), P2D);
			points2d.push_back(P2D);
			camera.project(pointList[0], P2D);
			points2d.push_back(P2D);
            m_measureLine->set2dPoint(points2d);
		}
		break;
	default:
		return;
	}
	if (m_associatedWin)
	{
		m_associatedWin->redraw(true,true);
	}
}

CCVector3d ccPointPropertiesDlg::getPositionByScreenPos(QPointF point)
{
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(point.x(), point.y());
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
	return Q3D;
}


bool ccPointPropertiesDlg::start()
{
    //[!]居中且初始化相机距离
    //MainWindow::TheInstance()->setGlobalZoom();
	point_info_action_->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/zuobiaodian@2x.png"));
	if (m_clipBox)
	{

		m_clipBox->reset();

		ccBBox oriBox = m_selectedEntities[0]->getBB_recursive();
		ccGLMatrix transformationOri;
		CCVector3 diagVec = oriBox.getDiagVec();
		diagVec.normalize();
		CCVector3 center = oriBox.getCenter();

		ccBBox extents(center - diagVec * 20.0, center + diagVec * 20.0);
		extents.setValidity(true);
		//[!]体积计算初始状态改为包围盒大小
		m_clipBox->set(oriBox, transformationOri);

		m_clipBox->setBBoxWidthColor(5.0, ccColor::red);
	}
	if (m_associatedWin)
	{
		m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
	}
	ccGui::ParamStruct param = ccGui::Parameters();
	param.m_isMeasureOn = true;
	param.m_isManualClassificationOpen = true;
	ccGui::Set(param);
	activatePointPropertiesDisplay();

	return ccPointPickingGenericInterface::start();
}

void ccPointPropertiesDlg::stop(bool state)
{
	assert(m_label);
	ccGui::ParamStruct param = ccGui::Parameters();
	param.m_isMeasureOn = false;
	param.m_isManualClassificationOpen = false;
	ccGui::Set(param);
	m_associatedWin->disconnect(this);
	m_label->clear();
	m_label->clearMeasurePos();
	m_measureLine->setVisible(false);
	ccPointPickingGenericInterface::stop(state);
}


void ccPointPropertiesDlg::activatePointPropertiesDisplay()
{
	if (m_measureMode != MEASURE_POINT)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_measureMode = MEASURE_POINT;
	m_label->setVisible(false);
	m_label->setMetaData("measuretype",int(MEASURE_POINT));
	m_label->setPickingMode(POINT_INFO);
	m_measureLine->setVisible(false);
	if (m_clipBox)
	{
		m_clipBox->setSelected(false);
		m_clipBox->setVisible(false);
	}
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::activateDistanceDisplay()
{
	if (m_measureMode != MEASURE_DISTANCE)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_measureLine->setCurrentMeasureMode(ccMeasureLine::MeasureLine_DISTANCE);
	m_measureMode = MEASURE_DISTANCE;
	m_label->setVisible(false);
	m_label->setMetaData("measuretype", int(MEASURE_DISTANCE));
	m_label->setPickingMode(POINT_POINT_DISTANCE);

	if (m_clipBox)
	{
		m_clipBox->setVisible(false);
	}
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::activateAngleDisplay()
{
	if (m_measureMode != MEASURE_ANGLE)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_measureLine->setCurrentMeasureMode(ccMeasureLine::MeasureLine_ANGLE);
	m_measureMode = MEASURE_ANGLE;
	m_label->setVisible(false);
	m_label->setMetaData("measuretype", int(MEASURE_ANGLE));
	m_label->setPickingMode(POINTS_ANGLE);

	if (m_clipBox)
	{
		m_clipBox->setVisible(false);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::activeHeightDisplay()
{
	if (m_measureMode != MEASURE_HEIGHT)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_measureLine->setCurrentMeasureMode(ccMeasureLine::MeasureLine_HEIGHT);
	m_measureMode = MEASURE_HEIGHT;
	m_label->setVisible(false);
	m_label->setMetaData("measuretype", int(MEASURE_HEIGHT));
	m_label->setPickingMode(POINT_HEIGHT);

	if (m_clipBox)
	{
		m_clipBox->setVisible(false);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::activeAeraDisplay()
{
	if (m_measureMode != MEASURE_AERA)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_measureLine->setCurrentMeasureMode(ccMeasureLine::MeasureLine_AERA);
	m_measureMode = MEASURE_AERA;
	m_label->setVisible(false);
	m_label->setMetaData("measuretype", int(MEASURE_AERA));
	m_label->setPickingMode(POINT_AERA);

	if (m_clipBox)
	{
		m_clipBox->setVisible(false);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::activeVolumeDisplay()
{
	if (m_measureMode != MEASURE_VOLUME)
	{
		m_measureLine->setVisible(false);
		m_measureLine->clearCurrentPos();
		m_label->clear();
		m_label->clearMeasurePos();
	}
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_measureMode = MEASURE_VOLUME;

	m_label->setMetaData("measuretype", int(MEASURE_VOLUME));
	m_label->setPickingMode(POINT_VOLUME);
	m_label->clear();
	m_label->clearMeasurePos();
	m_label->setVisible(true);
	m_label->setccBBox(m_clipBox->getBoxPtr());
	m_measureLine->setVisible(false);
    {
        m_clipBox->reset();

        ccBBox oriBox = m_selectedEntities[0]->getBB_recursive();
        ccGLMatrix transformationOri;
        CCVector3 diagVec = oriBox.getDiagVec();
        diagVec.normalize();
        CCVector3 center = oriBox.getCenter();

        ccBBox extents(center - diagVec * 20.0, center + diagVec * 20.0);
        extents.setValidity(true);
        //[!]体积计算初始状态改为包围盒大小
        m_clipBox->set(oriBox, transformationOri);

    }
	m_clipBox->setSelected(true);
	m_clipBox->setVisible(true);
	//修改体积测量标签位置carl20221209
	if (m_label)
	{
		CCVector3d centerPoint = m_clipBox->getBoxPtr()->getCenter();
		if (m_associatedWin)
		{
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;
			CCVector3d Q2D;
			camera.project(centerPoint, Q2D);
			CCVector3d P;
			const int retinaScale = m_associatedWin->devicePixelRatio();
			P.x = static_cast<PointCoordinateType>((Q2D.x)/ half_w/2.0);
			P.y = static_cast<PointCoordinateType>((Q2D.y) / half_h / 2.0);
			if (P.x <= 0)
			{
				P.x = 0.1;
			}
			if ( P.x >= 1)
			{
				P.x = 0.9;
			}
			if (P.y <= 0)
			{
				P.y = 0.9;
			}
			if ( P.y >= 1)
			{
				P.y = 0.1;
			}
			m_label->setPosition(P.x, P.y);
		}
	}
	if (m_associatedWin)
	{
		m_associatedWin->redraw(false);
	}
}

void ccPointPropertiesDlg::initializeState()
{
	m_label->clear();
	m_label->clearMeasurePos();
	m_clipBox->setVisible(false);
	m_measureLine->clearCurrentPos();
	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	if (m_associatedWin)
		m_associatedWin->redraw(false);
}

void ccPointPropertiesDlg::exportCurrentLabel()
{
	if (m_measureMode != MEASURE_VOLUME && !m_label->isVisible())
	{
		return;
	}
	ccHObject* labelObject = nullptr;
	if (m_measureMode == MEASURE_VOLUME)
	{
		ccBBox oldextents;
		ccGLMatrix oldtransformation;
		m_clipBox->get(oldextents, oldtransformation);
		m_label->saveBBox(oldtransformation);

		if (m_label->isShowBox && m_clipBox && m_clipBox->isVisible())
			labelObject = m_label;
		else
			labelObject = nullptr;
	}
	else
	{
		labelObject = (m_label ? m_label : nullptr);
	}
	
	if (!labelObject)
	{
		return;
	}

	//detach current label from window
	if (m_associatedWin)
		m_associatedWin->removeFromOwnDB(labelObject);
	labelObject->setSelected(false);
    m_clipBox->setVisible(false);
	ccHObject* newLabelObject = nullptr;

	if (m_measureMode == MEASURE_VOLUME)
	{
		if (m_clipBox)
		{
			m_clipBox->setVisible(false);

			m_clipBox->reset();

			ccBBox oriBox = m_selectedEntities[0]->getBB_recursive();
			ccGLMatrix transformationOri;
			CCVector3 diagVec = oriBox.getDiagVec();
			diagVec.normalize();
			CCVector3 center = oriBox.getCenter();

			ccBBox extents(center - diagVec * 20.0, center + diagVec * 20.0);
			extents.setValidity(true);
			m_clipBox->set(extents, transformationOri);

			m_clipBox->setBBoxWidthColor(5.0, ccColor::red);
		}

		m_selectedEntities[0]->addChild(labelObject);
	}
	else
		//attach old label to first point cloud by default
		m_object->addChild(labelObject);

	auto currentMode = m_label->getPickingMode();
	newLabelObject = m_label = new cc2DLabel();
	m_label->setSelected(true);
	m_label->setPickingMode(currentMode);
	m_label->setVisible(false);
	m_label->setMetaData("measuretype",int(m_measureMode));
	emit newLabel(labelObject);

	if (m_associatedWin)
	{
		m_associatedWin->addToOwnDB(newLabelObject);
		m_associatedWin->redraw(true, false);
	}	
}

CCVector3d ccPointPropertiesDlg::getPositionByScreenPosThroughCenter(QPointF point)
{
	ccBBox oriBox = m_selectedEntities[0]->getBB_recursive();
	CCVector3 center = oriBox.getCenter();
	CCVector3d firstPoint = center;
	//Ax+By+Cz+D=0 
	double A = m_directionVector.x;
	double B = m_directionVector.y;
	double C = m_directionVector.z;
	double D = (-A * firstPoint.x - B * firstPoint.y - C * firstPoint.z);
	CCVector3d newpoint = getPositionByScreenPos(point);
	double x1 = newpoint.x;
	double y1 = newpoint.y;
	double z1 = newpoint.z;
	firstPoint.y = (A*A*y1 + C * C*y1 - A * B*x1 - B * C*z1 - B * D) / (A*A + B * B + C * C);
	firstPoint.x = (B*B*x1 + C * C*x1 - A * B*y1 - A * C*z1 - A * D) / (A*A + B * B + C * C);
	firstPoint.z = (A*A*z1 + B * B*z1 - A * C*x1 - B * C*y1 - C * D) / (A*A + B * B + C * C);
	return firstPoint;
}

void ccPointPropertiesDlg::processPickedPoint(const PickedItem& picked)
{
	assert(picked.entity);
	assert(m_label);
	assert(m_associatedWin);
	if (std::find(m_selectedEntities.begin(), m_selectedEntities.end(), picked.entity) == m_selectedEntities.end())
	{
		return;
	}

	switch (m_measureMode)
	{
	case MEASURE_POINT:
		m_label->clear();
		m_object = picked.entity;
		m_label->setVisible(true);
		break;
	case MEASURE_DISTANCE:
		if (m_measureLine->getCurrentPos().size() == 0)
		{
			m_label->clearMeasurePos();
			m_object = picked.entity;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->addCurrentPos(picked.P3D);
			m_measureLine->addCurrentPos(picked.P3D);
		}
		else
		{
			m_measureLine->setLastCurrentPos(picked.P3D);
			m_measureLine->addCurrentPos(picked.P3D);
		}
        {
            ccGLCameraParameters camera;
            m_associatedWin->getGLCameraParameters(camera);
            std::vector < CCVector3d>  points2d;
            std::vector<CCVector3d> point3d = m_measureLine->getCurrentPos();
            CCVector3 P3D = point3d[point3d.size() - 2].toFloat();
            CCVector3d P2D;
            camera.project(P3D, P2D);
            points2d.push_back(P2D);
            camera.project(picked.P3D, P2D);
            points2d.push_back(P2D);
            m_measureLine->set2dPoint(points2d);
        }
		m_label->setVisible(false);
		break;
	case MEASURE_HEIGHT:
		if (m_label->getMeasurePointsNum() == 0)
		{
			m_object = picked.entity;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->setVisible(false);
			m_measureLine->addCurrentPos(picked.P3D);
		}
		else if (m_label->getMeasurePointsNum() == 1)
		{
			m_label->setVisible(true);
			m_measureLine->setVisible(false);
            m_measureLine->clearCurrentPos();
		}
		else if (m_label->getMeasurePointsNum() >= 2)
		{
			m_object = picked.entity;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->clearCurrentPos();
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->setVisible(false);
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->clearMeasurePos();
		}
		break;
	case MEASURE_ANGLE:
		if (m_label->getMeasurePointsNum()==0)
		{
			m_object = picked.entity;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->setVisible(false);
			m_measureLine->addCurrentPos(picked.P3D);
		}
		else if (m_label->getMeasurePointsNum() == 1)
		{
			m_measureLine->setLastCurrentPos(picked.P3D);
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->setVisible(false);
		}
		else if (m_label->getMeasurePointsNum() == 2)
		{
			m_label->setVisible(true);
			m_measureLine->setVisible(false);
            m_measureLine->clearCurrentPos();
		}
		else if (m_label->getMeasurePointsNum() >= 3)
		{
			m_object = picked.entity;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->clearCurrentPos();
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->setVisible(false);
			m_measureLine->addCurrentPos(picked.P3D);
			m_label->clearMeasurePos();
		}
		break;
	case MEASURE_AERA:
		m_measureLine->setVisible(true);

		if (m_measureLine->getCurrentPos().size() == 0)
		{
			CCVector3d newpoint = picked.P3D;
			m_measureLine->setVisible(true);
			m_measureLine->setLabelVisiable(false);
			m_measureLine->addCurrentPos(newpoint);
			m_label->setVisible(false);
			m_measureLine->addCurrentPos(newpoint);
			m_associatedWin->setInteractionMode(ccGLWindow::MODE_SEGMENT);
		}
		else if (m_measureLine->getCurrentPos().size() < 4)
		{
			auto curpointvec = m_measureLine->getCurrentPos();
			for (auto cursignalpos : curpointvec)
			{
				CCVector3d posdiff = cursignalpos - picked.P3D.toDouble();
				if (posdiff.norm() < 0.000001)
				{
					return;
				}
			}
			CCVector3d newpoint = picked.P3D;
			m_measureLine->setLastCurrentPos(newpoint);
			m_measureLine->addCurrentPos(newpoint);
			m_label->setVisible(false);
			if (m_measureLine->getCurrentPos().size() == 4)
			{
				getDirectionVector();
			}

		}
		else
		{
			CCVector3d newpoint = getCurrentDirectionMousePos(picked.P3D);
			m_measureLine->setLastCurrentPos(newpoint);
			m_measureLine->addCurrentPos(newpoint);
			m_label->setVisible(false);
		}
		if (m_measureLine->getCurrentPos().size() > 1)
		{
			std::vector < CCVector3d>  points2d;
			std::vector<CCVector3d> pointList = m_measureLine->getCurrentPos();
			CCVector3d P2D;
			int pointnum = pointList.size();
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			camera.project(pointList[pointnum - 2], P2D);
			points2d.push_back(P2D);
			camera.project(picked.P3D, P2D);
			points2d.push_back(P2D);
			camera.project(pointList[0], P2D);
			points2d.push_back(P2D);
			m_measureLine->set2dPoint(points2d);
		}
		break;
	case MEASURE_VOLUME:
		return;
	default:
		return;
	}
	if (m_measureMode == MEASURE_POINT)
	{
		if (picked.entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			m_label->addPickedPoint(static_cast<ccGenericPointCloud*>(picked.entity), picked.itemIndex, picked.entityCenter);
		}
		else if (picked.entity->isKindOf(CC_TYPES::MESH))
		{
			m_label->addPickedPoint(static_cast<ccGenericMesh*>(picked.entity), picked.itemIndex, CCVector2d(picked.uvw.x, picked.uvw.y), picked.entityCenter);
		}
	}
	else
	{
		m_label->addAreaMeasurePoints(picked.P3D);
	}

	
	m_label->displayPointLegend((m_label->getAreaMeasurePoints()).size() == 3 && m_measureMode == MEASURE_ANGLE); //we need to display 'A', 'B' and 'C' for 3-points labels
	//if (m_label->size() == 1 && m_associatedWin)
	{
		m_label->setPosition(static_cast<float>(picked.clickPoint.x() + 20) / m_associatedWin->glWidth(), static_cast<float>(picked.clickPoint.y() + 20) / m_associatedWin->glHeight());
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void ccPointPropertiesDlg::setCurrentCloudTypeMode(CloudTypeMode mode)
{ 
	m_pointMode = mode; 
	if (m_pointMode == POINT2DMODE)
	{
		m_measureLine->setIs3dPointMode(false);
	}
	else
	{
		m_measureLine->setIs3dPointMode(true);
	}
}



void ccPointPropertiesDlg::getDirectionVector()
{
	std::vector<CCVector3d> pointList = m_measureLine->getCurrentPos();
	if (pointList.size() < 3)
	{
		return;
	}
	CCVector3d v1 = pointList[0];
	CCVector3d v2 = pointList[1];
	CCVector3d v3 = pointList[2];
	float na = (v2[1] - v1[1]) * (v3[2] - v1[2]) - (v2[2] - v1[2]) * (v3[1] - v1[1]);
	float nb = (v2[2] - v1[2]) * (v3[0] - v1[0]) - (v2[0] - v1[0]) * (v3[2] - v1[2]);
	float nc = (v2[0] - v1[0]) * (v3[1] - v1[1]) - (v2[1] - v1[1]) * (v3[0] - v1[0]);
	m_directionVector = CCVector3d(na,nb,nc);
}

CCVector3d ccPointPropertiesDlg::getCurrentDirectionMousePos(CCVector3d point)
{
	CCVector3d firstPoint(0,0,0);
	if (m_measureLine->getCurrentPos().size()>0)
	{
		firstPoint = (m_measureLine->getCurrentPos())[0];
	}
	//Ax+By+Cz+D=0 
	double A = m_directionVector.x;
	double B = m_directionVector.y;
	double C = m_directionVector.z;
	double D = (-A * firstPoint.x - B * firstPoint.y - C * firstPoint.z);
	CCVector3d newpoint = point;
	double x1 = newpoint.x;
	double y1 = newpoint.y;
	double z1 = newpoint.z;
	firstPoint.y = (A*A*y1 + C*C*y1 -A*B*x1 - B*C*z1 - B*D) / (A*A +B*B +C*C);
	firstPoint.x = (B*B*x1 + C * C*x1 - A * B*y1 - A * C*z1 - A*D) / (A*A + B * B + C * C);
	firstPoint.z = (A*A*z1 + B * B*z1 - A * C*x1 - B * C*y1 - C*D) / (A*A + B * B + C * C);
	return firstPoint;
}