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

#include "ccIncludeGL.h"

//Local

#include "ccBasicTypes.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"

//Qt
#include <QSharedPointer>
#include <qdebug.h>

//System
#include <assert.h>
#include <string.h>
#include "ccColorTypes.h"
#include "ccshowline.h"


ccShowLine::ccShowLine(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
{
	lockVisibility(false);
	setEnabled(true);
}


QString ccShowLine::getName() const
{
	return m_name;
}



void ccShowLine::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccShowLine::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;
	return true;
}

bool ccShowLine::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;
	return true;
}


void ccShowLine::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//2D foreground only
	if (!MACRO_Foreground(context))
		return;

	//Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
	if (MACRO_VirtualTransEnabled(context))
		return;

	if (MACRO_Draw3D(context))
		drawMeOnly3D(context);
	else if (MACRO_Draw2D(context))
		drawMeOnly2D(context);


}

void ccShowLine::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{

}

std::vector<CCVector3d> ccShowLine::get3DPointLists()
{
    std::vector<CCVector3d> points;
    points.push_back(m_startPoint);
    points.push_back(m_mousePoint);
    return points;
}

void ccShowLine::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr || m_point2dList.size()!=2)
    {
        return;
    }
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        glFunc->glPushName(getUniqueIDForDisplay());
    }
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;
    ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 100);
    glFunc->glPushAttrib(GL_POINT_BIT);
    glFunc->glPointSize(m_pointsize);
    ccGL::Color4v(glFunc, color.rgba);
    glFunc->glEnable(GL_POINT_SMOOTH);
    glFunc->glDisable(GL_DEPTH_TEST);
    glFunc->glBegin(GL_POINTS);
    glFunc->glVertex2d(m_point2dList[0].x - halfW, m_point2dList[0].y - halfH);
    glFunc->glEnd();
    glFunc->glPopAttrib(); //GL_POINT_BIT

    //»­Ïß
    if (m_isShowMousePos)
    {
        glFunc->glPushAttrib(GL_LINE_BIT);
        glFunc->glLineWidth(2.0f);
        ccGL::Color4v(glFunc, color.rgba);
        glFunc->glDisable(GL_DEPTH_TEST);
        glFunc->glBegin(GL_LINES);
        glFunc->glVertex2d(m_point2dList[0].x - halfW, m_point2dList[0].y - halfH);
        glFunc->glVertex2d(m_point2dList[1].x - halfW, m_point2dList[1].y - halfH);
        glFunc->glEnd();
        glFunc->glPopAttrib(); //GL_LINE_BIT
    }
    if (pushName)
    {
        glFunc->glPopName();
    }
}


void ccShowLine::setMousePos(const CCVector3d & mousePoint)
{
	m_mousePoint = mousePoint;
}

void ccShowLine::setMousePosVisiable(bool isShow)
{
	m_isShowMousePos = isShow;
}

void ccShowLine::setFirstPos(const CCVector3d & point)
{
	m_startPoint = point;
}

void ccShowLine::setPointSize(float size)
{
    m_pointsize = size;
}