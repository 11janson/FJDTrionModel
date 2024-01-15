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

//Always first
#include "ccIncludeGL.h"

#include "ccClipBox.h"
//#include "ccReservedIDs.h"

//Local
#include "ccCone.h"
#include "ccCylinder.h"
#include "ccHObjectCaster.h"
#include "ccSphere.h"
#include "ccTorus.h"
#include "ccCuboids.h"
#include "qdebug.h"
#include "ccRasterGrid.h"
#include "fjdCube.h"
//system
#include <cassert>
//[!]Qt
#include <QFontMetrics>
//Components geometry
static QSharedPointer<ccCylinder> c_arrowShaft(nullptr);
//[!]拉杆
static QSharedPointer<ccCone> c_arrowHead(nullptr);
//[!]圆环上的圆锥
static QSharedPointer<ccCone> m_ConeOnRing(nullptr);
static QSharedPointer<ccTorus> c_torus(nullptr);
//[!]立方体
static QSharedPointer<fjdCube>m_fjdCube(nullptr);
//[!]立方体中的箭头
static QSharedPointer<ccCone> m_ConeInCube(nullptr);
static QSharedPointer<ccCylinder>m_CylinderInCube(nullptr);
//[!]球上的圆环
static QSharedPointer<ccTorus> m_Ring(nullptr);
static QSharedPointer<ccCylinder>g_SlideBarShaft(nullptr);
static QSharedPointer<ccCone> g_SlideBarHead(nullptr);
static QSharedPointer<ccSphere>c_centralSphere(nullptr);
static float m_SphereRadius = 1.0f;
void DrawDragLever(int ID, const CCVector3& start, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
	{
		glFunc->glLoadName(ID);
	}
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();


	ccGL::Translate(glFunc, start.x, start.y, start.z);
	ccGL::Scale(glFunc, scale, scale, scale);
	//we compute scalar prod between the two vectors
	CCVector3 Z(0.0, 0.0, 1.0);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = static_cast<PointCoordinateType>(180.0);

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees(acos(ps));

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!g_SlideBarShaft)
		g_SlideBarShaft = QSharedPointer<ccCylinder>(new ccCylinder(0.2f, 0.6f, nullptr, "ArrowShaft", 12, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities
	if (!g_SlideBarHead)
		g_SlideBarHead = QSharedPointer<ccCone>(new ccCone(0.3f, 0, 0.4f, 0, 0, nullptr, "ArrowHead", 24, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	glFunc->glTranslatef(0, 0, 0.25f);
	g_SlideBarShaft->setTempColor(col);
	g_SlideBarShaft->scaleFieldColorChange();
	g_SlideBarShaft->draw(context);
	glFunc->glTranslatef(0, 0, 0.2f + 0.2f);
	g_SlideBarHead->setTempColor(col);
	g_SlideBarHead->scaleFieldColorChange();
	g_SlideBarHead->draw(context);
	glFunc->glPopMatrix();
}
void DrawUnitArrow(int ID, const CCVector3& start, const CCVector3& direction, double scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
	{
		glFunc->glLoadName(ID);
	}
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();


	ccGL::Translate(glFunc, start.x, start.y, start.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0.0, 0.0, 1.0);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = static_cast<PointCoordinateType>(180.0);

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees(acos(ps));

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_arrowHead)
		c_arrowHead = QSharedPointer<ccCone>(new ccCone(0.2f, 0, 0.4f, 0, 0, nullptr, "ArrowHead", 32, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	ccGL::Translate(glFunc, 0, 0, 0.18f + 0.4f);
	c_arrowHead->setTempColor(col);
	c_arrowHead->showNormals(false);
	c_arrowHead->scaleFieldColorChange();
	c_arrowHead->draw(context);
	glFunc->glPopMatrix();
}

static void DrawUnitTorus(int ID, const CCVector3& center, const CCVector3& direction, double scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0, 0, 1);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = 180;

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees(acos(ps));

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}
	//圆环的内外径
	float ccTorusinsidediameter = 0.25f;
	float ccTorusoutsidediameter = 0.32f;
	//[!]圆锥的半径 高度
	float ConeRadius = 0.075f;
	float ConeHeight = 0.15f;

	if (!c_torus)
		c_torus = QSharedPointer<ccTorus>(new ccTorus(ccTorusinsidediameter, ccTorusoutsidediameter, 3.0f / 2.0f * M_PI, false, 0, nullptr, "Torus", 32, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities
	if (!m_ConeOnRing)
		m_ConeOnRing = QSharedPointer<ccCone>(new ccCone(ConeRadius, 0, ConeHeight, 0, 0, nullptr, "ConeOnRing", 32, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities


	glFunc->glTranslatef(0, 0, ccTorusinsidediameter);
	c_torus->setTempColor(col);
	c_torus->scaleFieldColorChange();
	c_torus->draw(context);

	glFunc->glRotatef(90.0f, 1.0f, 0, 0);
	glFunc->glTranslatef((ccTorusoutsidediameter - ccTorusinsidediameter) / 2.0f + ccTorusinsidediameter, 0, ConeHeight / 2.0f);
	m_ConeOnRing->setTempColor(col);
	m_ConeOnRing->scaleFieldColorChange();
	m_ConeOnRing->draw(context);

	glFunc->glPopMatrix();

}

static void DrawSimulateGyroscopeMotionOnSphere(int ID, const CCVector3& center, const CCVector3& direction, double scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0, 0, 1);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = 180;

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = CCCoreLib::RadiansToDegrees(acos(ps));

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!m_Ring)
		m_Ring = QSharedPointer<ccTorus>(new ccTorus(m_SphereRadius + 0.01f, m_SphereRadius + 0.05f, 2.0 * M_PI, false, 0, nullptr, "Torus", 12, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	glFunc->glTranslatef(0, 0, 0.25f);

	m_Ring->draw(context);
	glFunc->glPopMatrix();
}


//Unused function
static void DrawUnitSphere(int ID, const CCVector3& center, double radius, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, radius, radius, radius);

	if (!c_centralSphere)
		c_centralSphere = QSharedPointer<ccSphere>(new ccSphere(1, 0, "CentralSphere", 24, 0)); //we don't want to increase the unique ID counter for this 'invisible' entities

	c_centralSphere->setTempColor(col);
	c_centralSphere->draw(context);
	glFunc->glPopMatrix();

	DrawSimulateGyroscopeMotionOnSphere(ID, CCVector3(center.x, center.y, center.z), CCVector3(-1.0, 0.0, 0.0), radius, ccColor::cuttingBoxRed, context);
	DrawSimulateGyroscopeMotionOnSphere(ID, CCVector3(center.x, center.y, center.z), CCVector3(0.0, -1.0, 0.0), radius, ccColor::cuttingBoxGreen, context);
	DrawSimulateGyroscopeMotionOnSphere(ID, CCVector3(center.x, center.y, center.z), CCVector3(0.0, 0.0, -1.0), radius, ccColor::cuttingBoxBlue, context);

}
//Unused function
static void DrawUnitCube(int ID, const CCVector3& center, double radius, bool isUseColor, CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	//glFunc->glDisable(GL_DEPTH_TEST);
	glFunc->glEnable(GL_BLEND);
	glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, radius, radius, radius);

	float length = 0.9f;
	float alpha = 0.95f;
	float ccCylinder_radius = 0.02f;
	float ccCylinder_heght = 1.35f;
	float ccCone_radius = 0.06f;
	float ccCone_heght = 0.12f;

	if (!m_fjdCube) {
		m_fjdCube = QSharedPointer<fjdCube>(new fjdCube(length, alpha));
	}
	if (!m_CylinderInCube) {
		m_CylinderInCube = QSharedPointer<ccCylinder>(new ccCylinder(ccCylinder_radius, ccCylinder_heght, nullptr, "Cylinder", 32, 0));
	}
	if (!m_ConeInCube) {
		m_ConeInCube = QSharedPointer<ccCone>(new ccCone(ccCone_radius, 0, ccCone_heght, 0, 0, nullptr, "Cone", 32, 0));
	}
	//[!]x
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glRotatef(90.0f, 0, 1.0f, 0);
	glFunc->glTranslatef(length / 2.0f, -length / 2.0f, (ccCylinder_heght - length) / 2.0f);
	m_CylinderInCube->setTempColor(ccColor::red);
	m_CylinderInCube->scaleFieldColorChange();
	m_CylinderInCube->draw(context);
	glFunc->glTranslatef(0, 0, (ccCylinder_heght + ccCone_heght) / 2.0f);
	m_ConeInCube->setTempColor(ccColor::red);
	m_ConeInCube->scaleFieldColorChange();
	m_ConeInCube->draw(context);
	glFunc->glPopMatrix();
	//[!]y
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glRotatef(-90.0f, 1.0f, 0, 0);
	glFunc->glTranslatef(-length / 2.0f, length / 2.0f, (ccCylinder_heght - length) / 2.0f);
	m_CylinderInCube->setTempColor(ccColor::green);
	m_CylinderInCube->scaleFieldColorChange();
	m_CylinderInCube->draw(context);
	glFunc->glTranslatef(0, 0, (ccCylinder_heght + ccCone_heght) / 2.0f);
	m_ConeInCube->setTempColor(ccColor::green);
	m_ConeInCube->scaleFieldColorChange();
	m_ConeInCube->draw(context);
	glFunc->glPopMatrix();

	//[!]z
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(-length / 2.0f, -length / 2.0f, (ccCylinder_heght - length) / 2.0f);
	m_CylinderInCube->setTempColor(ccColor::cuttingBoxBlue);
	m_CylinderInCube->scaleFieldColorChange();
	m_CylinderInCube->draw(context);
	glFunc->glTranslatef(0, 0, (ccCylinder_heght + ccCone_heght) / 2.0f);
	m_ConeInCube->setTempColor(ccColor::cuttingBoxBlue);
	m_ConeInCube->scaleFieldColorChange();
	m_ConeInCube->draw(context);
	glFunc->glPopMatrix();

	//[!]立方体
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	isUseColor ? m_fjdCube->setCubeRgb(1.0f, 1.0f, 1.0f) : m_fjdCube->cubeRgbReset();;

	m_fjdCube->darwMe(context, center, radius);
	glFunc->glPopMatrix();

	glFunc->glDisable(GL_BLEND);
	glFunc->glPopMatrix();

}
static void DrawUnitCross(int ID, const CCVector3& center, double scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	scale /= 2;
	DrawDragLever(0, center, CCVector3(-1, 0, 0), scale, col, context);
	DrawDragLever(0, center, CCVector3(1, 0, 0), scale, col, context);
	DrawDragLever(0, center, CCVector3(0, -1, 0), scale, col, context);
	DrawDragLever(0, center, CCVector3(0, 1, 0), scale, col, context);
	DrawDragLever(0, center, CCVector3(0, 0, -1), scale, col, context);
	DrawDragLever(0, center, CCVector3(0, 0, 1), scale, col, context);
}

//default 'GetComponentIDFunction'
unsigned GetComponentID(ccClipBox::Components)
{
	return ccUniqueIDGenerator::InvalidUniqueID;
}

ccClipBox::ccClipBox(QString name/*= QString("clipping box")*/, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccHObject(name, uniqueID)
	, m_entityContainer("entities")
	, m_showBox(true)
	, m_activeComponent(NONE)
{
	isUseClipPlanes_ = true;
	setSelectionBehavior(SELECTION_IGNORED);

	box_color = ccColor::yellow;
}

ccClipBox::~ccClipBox()
{
	releaseAssociatedEntities();
}

void ccClipBox::update()
{
	if (m_entityContainer.getChildrenNumber() == 0 || !isUseClipPlanes_)//2022.4.20 janson.yang
	{
		return;
	}

	//remove any existing clipping plane
	{
		for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
		{
			m_entityContainer.getChild(ci)->removeAllClipPlanes();
		}
	}

	//now add the 6 box clipping planes
	ccBBox extents;
	ccGLMatrix transformation;
	get(extents, transformation);

	CCVector3 C = transformation * extents.getCenter();
	CCVector3 halfDim = extents.getDiagVec() / 2;

	//for each dimension
	for (unsigned d = 0; d < 3; ++d)
	{
		CCVector3 N = transformation.getColumnAsVec3D(d);
		//positive side
		{
			ccClipPlane posPlane;
			posPlane.equation.x = N.x;
			posPlane.equation.y = N.y;
			posPlane.equation.z = N.z;

			//compute the 'constant' coefficient knowing that P belongs to the plane if (P - (C - half_dim * N)).N = 0
			posPlane.equation.w = -static_cast<double>(C.dot(N)) + halfDim.u[d];
			for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
			{
				m_entityContainer.getChild(ci)->addClipPlanes(posPlane);
			}
		}

		//negative side
		{
			ccClipPlane negPlane;
			negPlane.equation.x = -N.x;
			negPlane.equation.y = -N.y;
			negPlane.equation.z = -N.z;

			//compute the 'constant' coefficient knowing that P belongs to the plane if (P - (C + half_dim * N)).N = 0
			//negPlane.equation.w = -(static_cast<double>(C.dot(N)) + halfDim.u[d]);
			negPlane.equation.w = static_cast<double>(C.dot(N)) + halfDim.u[d];
			for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
			{
				m_entityContainer.getChild(ci)->addClipPlanes(negPlane);
			}
		}
	}
}

void ccClipBox::reset()
{
	m_box.clear();
	resetGLTransformation();

	if (m_entityContainer.getChildrenNumber())
	{
		m_box = m_entityContainer.getBB_recursive();
	}

	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::set(const ccBBox& extents, const ccGLMatrix& transformation)
{
	m_box = extents;
	setGLTransformation(transformation);

	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::get(ccBBox& extents, ccGLMatrix& transformation)
{
	extents = m_box;

	if (isGLTransEnabled())
	{
		transformation = m_glTrans;
	}
	else
	{
		transformation.toIdentity();
	}
}

void ccClipBox::releaseAssociatedEntities()
{
	for (unsigned ci = 0; ci < m_entityContainer.getChildrenNumber(); ++ci)
	{
		m_entityContainer.getChild(ci)->removeAllClipPlanes();
	}
	m_entityContainer.removeAllChildren();
}

bool ccClipBox::addAssociatedEntity(ccHObject* entity)
{
	m_entityContainer.addChild(entity, DP_NONE); //no dependency!

	//no need to reset the clipping box if the entity has not a valid bounding-box
	//[!]包围盒
	if (entity->getBB_recursive().isValid())
	{
		reset();
	}

	return true;
}

void ccClipBox::setActiveComponent(int id)
{
	switch (id)
	{
	case 1:
		m_activeComponent = X_MINUS_ARROW;
		break;
	case 2:
		m_activeComponent = X_PLUS_ARROW;
		break;
	case 3:
		m_activeComponent = Y_MINUS_ARROW;
		break;
	case 4:
		m_activeComponent = Y_PLUS_ARROW;
		break;
	case 5:
		m_activeComponent = Z_MINUS_ARROW;
		break;
	case 6:
		m_activeComponent = Z_PLUS_ARROW;
		break;
	case 7:
		m_activeComponent = CROSS;
		break;
	case 8:
		m_activeComponent = CROSS /*SPHERE*/;
		break;
	case 9:
		m_activeComponent = X_MINUS_TORUS;
		break;
	case 10:
		m_activeComponent = Y_MINUS_TORUS;
		break;
	case 11:
		m_activeComponent = Z_MINUS_TORUS;
		break;
	case 12:
		m_activeComponent = X_PLUS_TORUS;
		break;
	case 13:
		m_activeComponent = Y_PLUS_TORUS;
		break;
	case 14:
		m_activeComponent = Z_PLUS_TORUS;
		break;
	default:
		m_activeComponent = NONE;
	}
}

int ccClipBox::getActiveComponent()
{
	return m_activeComponent;
}

static CCVector3d PointToVector(int x, int y, int screenWidth, int screenHeight)
{
	//convert mouse position to vector (screen-centered)
	CCVector3d v(static_cast<double>(2 * std::max(std::min(x, screenWidth - 1), -screenWidth + 1) - screenWidth) / static_cast<double>(screenWidth),
		static_cast<double>(screenHeight - 2 * std::max(std::min(y, screenHeight - 1), -screenHeight + 1)) / static_cast<double>(screenHeight),
		0);

	//square 'radius'
	double d2 = v.x * v.x + v.y * v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = sqrt(1 - d2);
	}

	return v;
}

bool ccClipBox::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	if (m_activeComponent != SPHERE || !m_box.isValid())
		return false;

	//convert mouse position to vector (screen-centered)
	CCVector3d currentOrientation = PointToVector(x, y, screenWidth, screenHeight);

	ccGLMatrixd rotMat = ccGLMatrixd::FromToRotation(m_lastOrientation, currentOrientation);

	CCVector3 C = m_box.getCenter();

	ccGLMatrixd transMat;
	transMat.setTranslation(-C);
	transMat = rotMat * transMat;
	transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

	//rotateGL(transMat);
	m_glTrans = ccGLMatrix(transMat.inverse().data()) * m_glTrans;
	enableGLTransformation(true);

	m_lastOrientation = currentOrientation;

	update();

	return true;
}

void ccClipBox::setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix)
{
	m_lastOrientation = PointToVector(x, y, screenWidth, screenHeight);
	m_viewMatrix = viewMatrix;
}

bool ccClipBox::move3D(const CCVector3d& uInput)
{
	emit siganlSetResetToolButtonEnabel(true);
	if (m_activeComponent == NONE || !m_box.isValid())
		return false;

	CCVector3d u = uInput;

	//Arrows
	if (m_activeComponent >= X_MINUS_ARROW && m_activeComponent <= CROSS)
	{
		if (m_glTransEnabled)
			m_glTrans.inverse().applyRotation(u);

		switch (m_activeComponent)
		{
		case X_MINUS_ARROW:
			m_box.minCorner().x += static_cast<PointCoordinateType>(u.x);
			//[!]限制裁剪框最小体积
			if ((m_box.maxCorner().x - m_box.minCorner().x) < 0.1)
			{
				m_box.minCorner().x -= static_cast<PointCoordinateType>(u.x);
			}
			else {
				if (m_box.minCorner().x > m_box.maxCorner().x)
					m_box.minCorner().x = m_box.maxCorner().x;
			}
			break;
		case X_PLUS_ARROW:
			m_box.maxCorner().x += static_cast<PointCoordinateType>(u.x);
			if ((m_box.maxCorner().x - m_box.minCorner().x) < 0.1) {
				m_box.maxCorner().x -= static_cast<PointCoordinateType>(u.x);
			}
			else {
				if (m_box.minCorner().x > m_box.maxCorner().x)
					m_box.maxCorner().x = m_box.minCorner().x;
			}
			break;
		case Y_MINUS_ARROW:
			m_box.minCorner().y += static_cast<PointCoordinateType>(u.y);
			if ((m_box.maxCorner().y - m_box.minCorner().y) < 0.1)
			{
				m_box.minCorner().y -= static_cast<PointCoordinateType>(u.y);
			}
			else {
				if (m_box.minCorner().y > m_box.maxCorner().y)
					m_box.minCorner().y = m_box.maxCorner().y;
			}
			break;
		case Y_PLUS_ARROW:
			m_box.maxCorner().y += static_cast<PointCoordinateType>(u.y);
			if ((m_box.maxCorner().y - m_box.minCorner().y) < 0.1)
			{
				m_box.maxCorner().y -= static_cast<PointCoordinateType>(u.y);
			}
			else
			{
				if (m_box.minCorner().y > m_box.maxCorner().y)
					m_box.maxCorner().y = m_box.minCorner().y;
			}

			break;
		case Z_MINUS_ARROW:
			m_box.minCorner().z += static_cast<PointCoordinateType>(u.z);
			if ((m_box.maxCorner().z - m_box.minCorner().z) < 0.1)
			{
				m_box.minCorner().z -= static_cast<PointCoordinateType>(u.z);
			}
			else
			{
				if (m_box.minCorner().z > m_box.maxCorner().z)
					m_box.minCorner().z = m_box.maxCorner().z;
			}
			break;
		case Z_PLUS_ARROW:
			m_box.maxCorner().z += static_cast<PointCoordinateType>(u.z);
			if ((m_box.maxCorner().z - m_box.minCorner().z) < 0.1)
			{
				m_box.maxCorner().z -= static_cast<PointCoordinateType>(u.z);
			}
			else
			{
				if (m_box.minCorner().z > m_box.maxCorner().z)
					m_box.maxCorner().z = m_box.minCorner().z;
			}
			break;
		case CROSS:
			m_box += u.toPC();
			break;
		default:
			assert(false);
			return false;
		}

		//send 'modified' signal
		emit boxModified(&m_box);
	}
	else if (m_activeComponent == SPHERE)
	{
		//handled by move2D!
		return false;
	}
	else if (m_activeComponent >= X_MINUS_TORUS && m_activeComponent <= Z_PLUS_TORUS)
	{
		//we guess the rotation order by comparing the current screen 'normal'
		//and the vector prod of u and the current rotation axis
		CCVector3d Rb(0, 0, 0);
		switch (m_activeComponent)
		{
		case X_MINUS_TORUS:
			Rb.x = -1;
			break;
		case X_PLUS_TORUS:
			Rb.x = 1;
			break;
		case Y_MINUS_TORUS:
			Rb.y = -1;
			break;
		case Y_PLUS_TORUS:
			Rb.y = 1;
			break;
		case Z_MINUS_TORUS:
			Rb.z = -1;
			break;
		case Z_PLUS_TORUS:
			Rb.z = 1;
			break;
		default:
			assert(false);
			return false;
		}

		CCVector3d R = Rb;
		if (m_glTransEnabled)
		{
			m_glTrans.applyRotation(R);
		}

		CCVector3d RxU = R.cross(u);

		//look for the most parallel dimension
		double maxDot = m_viewMatrix.getColumnAsVec3D(0).dot(RxU);
		for (int i = 1; i < 3; ++i)
		{
			double dot = m_viewMatrix.getColumnAsVec3D(i).dot(RxU);
			if (std::abs(dot) > std::abs(maxDot))
			{
				maxDot = dot;
			}
		}

		//angle is proportional to absolute displacement
		double angle_rad = u.norm() / m_box.getDiagNorm() * M_PI;
		if (maxDot < 0.0)
			angle_rad = -angle_rad;

		ccGLMatrixd rotMat;
		rotMat.initFromParameters(angle_rad, Rb, CCVector3d(0, 0, 0));

		CCVector3 C = m_box.getCenter();
		ccGLMatrixd transMat;
		transMat.setTranslation(-C);
		transMat = rotMat * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

		m_glTrans = m_glTrans * ccGLMatrix(transMat.inverse().data());
		enableGLTransformation(true);
	}
	else
	{
		assert(false);
		return false;
	}

	update();

	return true;
}

void ccClipBox::setBox(const ccBBox& box)
{
	m_box = box;

	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::shift(const CCVector3& v)
{
	m_box.minCorner() += v;
	m_box.maxCorner() += v;

	update();
	emit siganlSetResetToolButtonEnabel(true);
	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::flagPointsInside(ccGenericPointCloud* cloud,
	ccGenericPointCloud::VisibilityTableType* visTable,
	bool shrink/*=false*/) const
{
	if (!cloud || !visTable)
	{
		//invalid input
		assert(false);
		return;
	}
	if (cloud->size() != visTable->size())
	{
		///size mismatch
		assert(false);
		return;
	}

	int count = static_cast<int>(cloud->size());
	if (m_glTransEnabled)
	{
		ccGLMatrix transMat = m_glTrans.inverse();

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < count; ++i)
		{
			if (!shrink || visTable->at(i) == CCCoreLib::POINT_VISIBLE)
			{
				CCVector3 P = *cloud->getPoint(static_cast<unsigned>(i));
				transMat.apply(P);
				if (m_bDisPlayInorOutpoint)
				{
					visTable->at(i) = (m_box.contains(P) ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
				}
				else
				{
					visTable->at(i) = (m_box.contains(P) ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE);
				}
			}
		}
	}
	else
	{
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < count; ++i)
		{
			if (!shrink || visTable->at(i) == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3* P = cloud->getPoint(static_cast<unsigned>(i));
				if (m_bDisPlayInorOutpoint)
				{
					visTable->at(i) = (m_box.contains(*P) ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
				}
				else
				{
					visTable->at(i) = (m_box.contains(*P) ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE);
				}
			}
		}
	}
}

ccBBox ccClipBox::getOwnBB(bool withGLFeatures/*=false*/)
{
	ccBBox bbox = m_box;

	if (withGLFeatures)
	{
		PointCoordinateType scale = computeArrowsScale();
		bbox.minCorner() -= CCVector3(scale, scale, scale);
		bbox.maxCorner() += CCVector3(scale, scale, scale);
	}

	return bbox;
}

PointCoordinateType ccClipBox::computeArrowsScale() const
{
	PointCoordinateType scale = m_box.getDiagNorm() / 10;

	if (m_entityContainer.getChildrenNumber() != 0)
	{
		scale = std::max<PointCoordinateType>(scale, getBox().getDiagNorm() / 25);
	}

	return scale;
}

const ColorCompType c_lightComp = ccColor::MAX / 2;
const ccColor::Rgb c_lightRed(ccColor::MAX, c_lightComp, c_lightComp);
const ccColor::Rgb c_lightGreen(c_lightComp, ccColor::MAX, c_lightComp);
const ccColor::Rgb c_lightBlue(c_lightComp, c_lightComp, ccColor::MAX);

void ccClipBox::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	if (!m_box.isValid())
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//janson 2022.6.28
	if ((context.drawingFlags & CC_DRAW_FAST_NAMES_ONLY) != CC_DRAW_FAST_NAMES_ONLY)
	{
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());
		//project the point in 2D
		CCVector3 P3D = m_box.getCenter();
		camera.project(P3D, m_box.pos2D);

		//m_box.position2d_text = CCVector3d((m_box.pos2D.x + 20) / camera.viewport[2], (m_box.pos2D.y + 20) / camera.viewport[3], 0);
		m_box.position2d_text = CCVector3d((m_box.pos2D.x + 20) / context.glW, (m_box.pos2D.y + 20) / context.glH, 0);
	}
	if (m_showBox)
	{
		//m_box.draw(m_selected ? context.bbDefaultCol : ccColor::magenta);
		//m_box.draw(context, ccColor::yellow);
		m_box.draw(context, box_color);
	}

	if (!m_selected)
	{
		//nothing to show
		return;
	}

	//standard case: list names pushing (1st level)
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	//draw the interactors
	{
		const CCVector3& minC = m_box.minCorner();
		const CCVector3& maxC = m_box.maxCorner();
		const CCVector3 center = m_box.getCenter();

        ccViewportParameters viewParams = context.display->getViewportParameters();
        CCVector3d viewCenter = viewParams.getCameraCenter();
        double dis;
        double x, y, z;
        x = viewCenter.x - center.x;
        y = viewCenter.y - center.y;
        z = viewCenter.z - center.z;
        dis = sqrt((x * x + y * y + z * z));
		double initialscale = m_box.getDiagNormd() / static_cast<double>(8.0);

        if (m_entityContainer.getChildrenNumber() != 0)
        {
			initialscale = std::max<PointCoordinateType>(initialscale, getBox().getDiagNorm() / 25);
        }

        
		if (m_initialDistance == -1)
		{
			m_initialDistance = dis;
		}
		double scale = dis * initialscale / m_initialDistance;
		if (scale > initialscale)
		{
			scale = initialscale;
		}

		

		

		//custom arrow 'context'
		CC_DRAW_CONTEXT componentContext = context;
		componentContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the arrows don't push their own!
		componentContext.display = nullptr;

		if (pushName) //2nd level = sub-item
		{
			glFunc->glPushName(0); //fake ID, will be replaced by the arrows one if any
		}

		//force the light on
		glFunc->glPushAttrib(GL_LIGHTING_BIT);
		glFunc->glEnable(GL_LIGHT0);

        DrawUnitArrow(X_MINUS_ARROW * pushName, CCVector3(minC.x, center.y, center.z), CCVector3(-1.0, 0.0, 0.0), scale, (m_activeComponent == X_MINUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxRed, componentContext);
        DrawUnitArrow(X_PLUS_ARROW * pushName, CCVector3(maxC.x, center.y, center.z), CCVector3(1.0, 0.0, 0.0), scale, (m_activeComponent == X_PLUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxRed, componentContext);
        DrawUnitArrow(Y_MINUS_ARROW * pushName, CCVector3(center.x, minC.y, center.z), CCVector3(0.0, -1.0, 0.0), scale, (m_activeComponent == Y_MINUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxGreen, componentContext);
        DrawUnitArrow(Y_PLUS_ARROW * pushName, CCVector3(center.x, maxC.y, center.z), CCVector3(0.0, 1.0, 0.0), scale, (m_activeComponent == Y_PLUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxGreen, componentContext);
        DrawUnitArrow(Z_MINUS_ARROW * pushName, CCVector3(center.x, center.y, minC.z), CCVector3(0.0, 0.0, -1.0), scale, (m_activeComponent == Z_MINUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxBlue, componentContext);
        DrawUnitArrow(Z_PLUS_ARROW * pushName, CCVector3(center.x, center.y, maxC.z), CCVector3(0.0, 0.0, 1.0), scale, (m_activeComponent == Z_PLUS_ARROW) ? ccColor::cuttingHighlight : ccColor::cuttingBoxBlue, componentContext);
        DrawUnitCube(SPHERE * pushName, minC - /*maxC +*/ CCVector3(scale, scale, scale) / 2.0, scale / 2.0, (m_activeComponent == CROSS) ? true : false, componentContext);

        //DrawUnitCross(CROSS * pushName, minC - CCVector3(scale, scale, scale) / 2.0, scale, (m_activeComponent == CROSS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxYellow, componentContext);
        //DrawUnitSphere(SPHERE*pushName, maxC + CCVector3(scale, scale, scale) / 2.0, scale / 2.0, ccColor::yellow, componentContext);
        DrawUnitTorus(X_MINUS_TORUS * pushName, CCVector3(minC.x, center.y, center.z), CCVector3(-1.0, 0.0, 0.0), scale, (m_activeComponent == X_MINUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxRed, componentContext);
        DrawUnitTorus(Y_MINUS_TORUS * pushName, CCVector3(center.x, minC.y, center.z), CCVector3(0.0, -1.0, 0.0), scale, (m_activeComponent == Y_MINUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxGreen, componentContext);
        DrawUnitTorus(Z_MINUS_TORUS * pushName, CCVector3(center.x, center.y, minC.z), CCVector3(0.0, 0.0, -1.0), scale, (m_activeComponent == Z_MINUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxBlue, componentContext);
        DrawUnitTorus(X_PLUS_TORUS * pushName, CCVector3(maxC.x, center.y, center.z), CCVector3(1.0, 0.0, 0.0), scale, (m_activeComponent == X_PLUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxRed, componentContext);
        DrawUnitTorus(Y_PLUS_TORUS * pushName, CCVector3(center.x, maxC.y, center.z), CCVector3(0.0, 1.0, 0.0), scale, (m_activeComponent == Y_PLUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxGreen, componentContext);
        DrawUnitTorus(Z_PLUS_TORUS * pushName, CCVector3(center.x, center.y, maxC.z), CCVector3(0.0, 0.0, 1.0), scale, (m_activeComponent == Z_PLUS_TORUS) ? ccColor::cuttingHighlight : ccColor::cuttingBoxBlue, componentContext);


		drawPosition3D(context, CCVector3(maxC.x + scale, center.y, center.z),
			CCVector3(center.x, maxC.y + scale, center.z),
			CCVector3(center.x, center.y, maxC.z + scale));

		glFunc->glPopAttrib();

		if (pushName)
		{
			glFunc->glPopName();
		}
	}

	if (pushName)
	{
		glFunc->glPopName();
	}

}
void ccClipBox::drawPosition3D(CC_DRAW_CONTEXT& context, CCVector3 XPoition, CCVector3 YPoition, CCVector3 ZPoition)
{
	QFont titleFont(context.display->getTextDisplayFont());

	context.display->display3DLabel(QString::number(m_box.maxCorner().x - m_box.minCorner().x, 'f', 3),
		XPoition,
		&ccColor::cuttingBoxRed,
		titleFont);
	context.display->display3DLabel(QString::number(m_box.maxCorner().y - m_box.minCorner().y, 'f', 3),
		YPoition,
		&ccColor::cuttingBoxGreen,
		titleFont);
	context.display->display3DLabel(QString::number(m_box.maxCorner().z - m_box.minCorner().z, 'f', 3),
		ZPoition,
		&ccColor::cuttingBoxBlue,
		titleFont);
}

