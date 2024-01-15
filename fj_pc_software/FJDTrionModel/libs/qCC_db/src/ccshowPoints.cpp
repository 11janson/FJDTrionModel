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
#include "ccshowPoints.h"
#include "ccViewportParameters.h"
ccShowPoints::ccShowPoints(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
{
	lockVisibility(false);
	setEnabled(true);
}


QString ccShowPoints::getName() const
{
	return m_name;
}



void ccShowPoints::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccShowPoints::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;
	return true;
}

bool ccShowPoints::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;
	return true;
}


void ccShowPoints::drawMeOnly(CC_DRAW_CONTEXT& context)
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

void ccShowPoints::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		//not fast at all!
		if (MACRO_DrawFastNamesOnly(context))
		{
			return;
		}

		glFunc->glPushName(getUniqueIDForDisplay());
	}
	glFunc->glPushAttrib(GL_POINT_BIT);

    ccViewportParameters viewParams = context.display->getViewportParameters();
	float size;

	if (m_cloudSizeAdaptively)
	{
		size = viewParams.getAutoPointSize();
	}
	else
	{
		size = static_cast<float>(m_cloudsize);
	}

    glFunc->glPointSize(size + m_HeightLightPointSize);
	ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 255);
	ccGL::Color4v(glFunc, color.rgba);
	//glFunc->glEnable(GL_POINT_SMOOTH);
	glFunc->glDisable(GL_DEPTH);
	glFunc->glBegin(GL_POINTS);
	for (int i =0;i < m_points.size();i++)
	{
		glFunc->glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
	}
	glFunc->glEnd();
	glFunc->glPopAttrib(); //GL_POINT_BIT
	if (pushName)
	{
		glFunc->glPopName();
	}
}


void ccShowPoints::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{

}


