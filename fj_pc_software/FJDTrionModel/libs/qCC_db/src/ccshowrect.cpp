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
#include "ccshowrect.h"


ccShowRect::ccShowRect(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
{
	lockVisibility(false);
	setEnabled(true);
}


QString ccShowRect::getName() const
{
	return m_name;
}



void ccShowRect::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccShowRect::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;
	return true;
}

bool ccShowRect::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;
	return true;
}


void ccShowRect::drawMeOnly(CC_DRAW_CONTEXT& context)
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

void ccShowRect::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}
	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		//not particularly fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}
	if (m_PointParam.size() == 8)
	{
		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glFunc->glEnable(GL_BLEND);
		glFunc->glColor4ub(255, 200, 4, 80);

		glFunc->glBegin(GL_QUADS);
		glFunc->glVertex3f(m_PointParam[0].x, m_PointParam[0].y, m_PointParam[0].z);
		glFunc->glVertex3f(m_PointParam[1].x, m_PointParam[1].y, m_PointParam[1].z);
		glFunc->glVertex3f(m_PointParam[2].x, m_PointParam[2].y, m_PointParam[2].z);
		glFunc->glVertex3f(m_PointParam[3].x, m_PointParam[3].y, m_PointParam[3].z);
		glFunc->glEnd();

        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex3f(m_PointParam[4].x, m_PointParam[4].y, m_PointParam[4].z);
        glFunc->glVertex3f(m_PointParam[5].x, m_PointParam[5].y, m_PointParam[5].z);
        glFunc->glVertex3f(m_PointParam[6].x, m_PointParam[6].y, m_PointParam[6].z);
        glFunc->glVertex3f(m_PointParam[7].x, m_PointParam[7].y, m_PointParam[7].z);
        glFunc->glEnd();

        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex3f(m_PointParam[4].x, m_PointParam[4].y, m_PointParam[4].z);
        glFunc->glVertex3f(m_PointParam[5].x, m_PointParam[5].y, m_PointParam[5].z);
        glFunc->glVertex3f(m_PointParam[1].x, m_PointParam[1].y, m_PointParam[1].z);
        glFunc->glVertex3f(m_PointParam[0].x, m_PointParam[0].y, m_PointParam[0].z);
        glFunc->glEnd();

        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex3f(m_PointParam[5].x, m_PointParam[5].y, m_PointParam[5].z);
        glFunc->glVertex3f(m_PointParam[6].x, m_PointParam[6].y, m_PointParam[6].z);
        glFunc->glVertex3f(m_PointParam[2].x, m_PointParam[2].y, m_PointParam[2].z);
        glFunc->glVertex3f(m_PointParam[1].x, m_PointParam[1].y, m_PointParam[1].z);
        glFunc->glEnd();

        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex3f(m_PointParam[6].x, m_PointParam[6].y, m_PointParam[6].z);
        glFunc->glVertex3f(m_PointParam[7].x, m_PointParam[7].y, m_PointParam[7].z);
        glFunc->glVertex3f(m_PointParam[3].x, m_PointParam[3].y, m_PointParam[3].z);
        glFunc->glVertex3f(m_PointParam[2].x, m_PointParam[2].y, m_PointParam[2].z);
        glFunc->glEnd();

        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex3f(m_PointParam[4].x, m_PointParam[4].y, m_PointParam[4].z);
        glFunc->glVertex3f(m_PointParam[7].x, m_PointParam[7].y, m_PointParam[7].z);
        glFunc->glVertex3f(m_PointParam[3].x, m_PointParam[3].y, m_PointParam[3].z);
        glFunc->glVertex3f(m_PointParam[0].x, m_PointParam[0].y, m_PointParam[0].z);
        glFunc->glEnd();

		glFunc->glPopAttrib();
	}
	if (pushName)
	{
		glFunc->glPopName();
	}
}


void ccShowRect::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{

}

void ccShowRect::setPointData(std::vector<CCVector3d> param)
{
	m_PointParam = param;
}