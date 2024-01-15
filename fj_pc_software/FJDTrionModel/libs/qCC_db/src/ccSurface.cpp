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

#include <iostream>

#include "ccSurface.h"
#include "ccHObjectCaster.h"


#include "qCC_db/include/ccGenericGLDisplay.h"

ccSurface::ccSurface(const ccGLMatrix* transMat, QString name)
	//: ccHObject(name.isEmpty() ? "Surface" : name)
    : ccGenericPrimitive(name, transMat)
	, m_SurfaceHeight(0.1)
	, m_SurfacePos(0.0)
	, m_Mincorner(0.0, 0.0, 0.0)
	, m_Maxcorner(0.0, 0.0, 0.0)
	, m_SliceType(ZAXIS)
	, m_XOffset(0.0)
	, m_YOffset(0.0)
	, m_ZOffset(0.0)
	, m_DoubleClicked(false)
    , m_open(false)
    , m_scaling(1.0)
    , m_scalingOpen(false)
{
}

bool ccSurface::buildUp()
{
#if 0
    if (!init(4, false, 2, 1))
    {
        ccLog::Error("[ccSurface::buildUp] Not enough memory");
        return false;
    }

    ccPointCloud* verts = vertices();
    assert(verts);
    assert(m_triNormals);

    // B ------ C
    // |        |
    // A ------ D
    verts->addPoint(CCVector3(-m_xWidth / 2, -m_yWidth / 2, 0));
    verts->addPoint(CCVector3(-m_xWidth / 2, m_yWidth / 2, 0));
    verts->addPoint(CCVector3(m_xWidth / 2, m_yWidth / 2, 0));
    verts->addPoint(CCVector3(m_xWidth / 2, -m_yWidth / 2, 0));

    m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1)));

    addTriangle(0, 2, 1); //A C B
    addTriangleNormalIndexes(0, 0, 0);
    addTriangle(0, 3, 2); //A D C
    addTriangleNormalIndexes(0, 0, 0);

#endif
    return true;
}

ccGenericPrimitive* ccSurface::clone() const
{
#if 0
    return finishCloneJob(new ccSurface(m_xWidth, m_yWidth, &m_transformation, getName()));

#endif
    return nullptr;
}

void ccSurface::updateSlicePos(CCVector3d pos)
{
	m_SurfacePos = 0.0;
	m_pos = pos;
	m_DoubleClicked = true;
}

void ccSurface::setScaling(float scaling)
{
    m_scaling = scaling;
    m_scalingOpen = true;
}


void ccSurface::draw(CC_DRAW_CONTEXT& context/*, ccBBox *pBox*/)
{
    if (!m_visible)
    {
        return;
    }
    //call parent method
    ccGenericPrimitive::drawMeOnly(context);

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

    //ccGLWindow * win = dynamic_cast<ccGLWindow*>(context.display);
    QString name = context.display->windowName();
    if (context.display->windowName().compare("3D View") != 0)
    {
        return;
    }

    if (MACRO_Draw3D(context))
    {
        CCVector3 bboxDiag = m_CalcBBox.getDiagVec();
        CCVector3 bboxCenter = m_CalcBBox.getCenter();


        glFunc->glEnable(GL_DEPTH_TEST);
        glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glFunc->glEnable(GL_BLEND);
        glFunc->glColor4ub(255, 200, 4, 80);
        if (m_open)
        {
            //´¦Àíz-fighting
            glFunc->glEnable(GL_POLYGON_OFFSET_FILL);
            glFunc->glPolygonOffset(-1, -1);
        }



        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPushMatrix();
        glFunc->glTranslatef(bboxCenter.x, bboxCenter.y, bboxCenter.z);
        if (m_scalingOpen)
        {
			bboxDiag *= m_scaling;
        }

        switch (m_SliceType)
        {
        case XAXIS:
        {

            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(m_SurfacePos, 0.0, 0.0);

            if (m_DoubleClicked)
            {
                glFunc->glTranslatef(-m_SurfacePos, 0.0, 0.0);
                m_XOffset = bboxCenter.x - m_pos.x;
                m_halfBoxSize = m_SurfaceHeight / 2.0;
                glFunc->glTranslatef(-m_XOffset, 0.0, 0.0);
            }
            double halfHeight = m_SurfaceHeight / 2.0;

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, 0.0-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, 0.0-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, 0.0-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, 0.0-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, 0.0 - halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, 0.0 - halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, 0.0 - halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, m_SurfaceHeight-halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, 0.0 - halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(m_SurfaceHeight - halfHeight, 0.0, 0.0);
            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, -halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();
            glFunc->glPopMatrix();


            const CCVector3 minVec(0.0 - halfHeight, static_cast<double>(-bboxDiag.y / 2.0f), static_cast<double>(-bboxDiag.z / 2));
            const CCVector3 maxVec(m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.y / 2.0f), static_cast<double>(bboxDiag.z / 2));

            CCCoreLib::BoundingBox bbox = CCCoreLib::BoundingBox(minVec, maxVec);
            CCVector3 tempMin = bbox.minCorner();
            CCVector3 tempMax = bbox.maxCorner();
            if (m_DoubleClicked)
            {
                m_Mincorner = bbox.minCorner() + CCVector3(-m_SurfacePos, 0.0, 0.0) + CCVector3(-m_XOffset, 0.0, 0.0)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(m_SurfacePos, 0.0, 0.0);
                m_Maxcorner = bbox.maxCorner() + CCVector3(-m_SurfacePos, 0.0, 0.0) + CCVector3(-m_XOffset, 0.0, 0.0)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(m_SurfacePos, 0.0, 0.0);
            }
            else
            {
                m_Mincorner = bbox.minCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(m_SurfacePos, 0.0, 0.0);
                m_Maxcorner = bbox.maxCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(m_SurfacePos, 0.0, 0.0);
            }

            glFunc->glEnable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_ALWAYS);
            glFunc->glLineWidth(2.0);
            glFunc->glEnable(GL_LINE_SMOOTH);
            glFunc->glEnable(GL_BLEND);
            glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
            glFunc->glColor3ub(255, 200, 4);

            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINES);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_LESS);
            glFunc->glDisable(GL_BLEND);
            glFunc->glDisable(GL_LINE_SMOOTH);
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPopMatrix();
            glFunc->glPopAttrib();
            break;
        }

        case YAXIS:
        {
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(0.0, m_SurfacePos, 0.0);

            if (m_DoubleClicked)
            {
                glFunc->glTranslatef(0.0, -m_SurfacePos, 0.0);
                m_YOffset = bboxCenter.y - m_pos.y;
                m_halfBoxSize = m_SurfaceHeight / 2.0;
                glFunc->glTranslatef(0.0, -m_YOffset, 0.0);
            }
            double halfHeight = m_SurfaceHeight / 2.0;

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0-halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f),  0.0-halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f),  0.0-halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0-halfHeight, static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            glFunc->glEnd();

            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(0.0, m_SurfaceHeight, 0.0);
            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0-halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f),  0.0-halfHeight, static_cast<double>(-bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f),  0.0-halfHeight, static_cast<double>(bboxDiag.z / 2));
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0-halfHeight, static_cast<double>(bboxDiag.z / 2));
            glFunc->glEnd();
            glFunc->glPopMatrix();

            const CCVector3 minVec(static_cast<double>(-bboxDiag.x / 2.0f), 0.0 - halfHeight, static_cast<double>(-bboxDiag.z / 2));
            const CCVector3 maxVec(static_cast<double>(bboxDiag.x / 2.0f), m_SurfaceHeight - halfHeight, static_cast<double>(bboxDiag.z / 2));

            CCCoreLib::BoundingBox bbox = CCCoreLib::BoundingBox(minVec, maxVec);

            CCVector3 tempMin = bbox.minCorner();
            CCVector3 tempMax = bbox.maxCorner();

            if (m_DoubleClicked)
            {
                m_Mincorner = bbox.minCorner() + CCVector3(0.0, -m_SurfacePos, 0.0) + CCVector3(0.0, -m_YOffset, 0.0)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
                m_Maxcorner = bbox.maxCorner() + CCVector3(0.0, -m_SurfacePos, 0.0) + CCVector3(0.0, -m_YOffset, 0.0)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
            }
            else
            {
                m_Mincorner = bbox.minCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
                m_Maxcorner = bbox.maxCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
            }

            glFunc->glEnable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_ALWAYS);
            glFunc->glLineWidth(2.0);
            glFunc->glEnable(GL_LINE_SMOOTH);
            glFunc->glEnable(GL_BLEND);
            glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
            glFunc->glColor3ub(255, 200, 4);


            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINES);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_LESS);
            glFunc->glDisable(GL_BLEND);
            glFunc->glDisable(GL_LINE_SMOOTH);
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPopMatrix();
            glFunc->glPopAttrib();

            break;
        }
        case ZAXIS:
        {
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(0.0, 0.0, m_SurfacePos);

            if (m_DoubleClicked)
            {
                glFunc->glTranslatef(0.0, 0.0, -m_SurfacePos);
                m_ZOffset = bboxCenter.z - m_pos.z;
                m_halfBoxSize = m_SurfaceHeight / 2.0;
                glFunc->glTranslatef(0.0, 0.0, -m_ZOffset);
            }
            double halfHeight = m_SurfaceHeight / 2.0;

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f),  0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f),   0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f),  0.0-halfHeight);
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0 - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), 0.0 - halfHeight);
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0 - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0 - halfHeight);
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), 0.0 - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), 0.0 - halfHeight);
            glFunc->glEnd();

            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0 - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), 0.0 - halfHeight);
            glFunc->glEnd();

            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            glFunc->glTranslatef(0.0, 0.0, m_SurfaceHeight);
            glFunc->glBegin(GL_QUADS);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f),  0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f),   0.0-halfHeight);
            ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f),  0.0-halfHeight);
            glFunc->glEnd();
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPopMatrix();

            const CCVector3 minVec(static_cast<double>(-bboxDiag.x / 2.0f), static_cast<double>(-bboxDiag.y / 2.0f), 0.0 - halfHeight);
            const CCVector3 maxVec(static_cast<double>(bboxDiag.x / 2.0f), static_cast<double>(bboxDiag.y / 2.0f), m_SurfaceHeight - halfHeight);

            CCCoreLib::BoundingBox bbox = CCCoreLib::BoundingBox(minVec, maxVec);

            CCVector3 tempMin = bbox.minCorner();
            CCVector3 tempMax = bbox.maxCorner();

            if (m_DoubleClicked)
            {
                m_Mincorner = bbox.minCorner() + CCVector3(0.0, 0.0, -m_SurfacePos) + CCVector3(0.0, 0.0, -m_ZOffset)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
                m_Maxcorner = bbox.maxCorner() + CCVector3(0.0, 0.0, -m_SurfacePos) + CCVector3(0.0, 0.0, -m_ZOffset)
                    + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, m_SurfacePos, 0.0);
            }
            else
            {
                m_Mincorner = bbox.minCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, 0.0, m_SurfacePos);
                m_Maxcorner = bbox.maxCorner() + CCVector3(bboxCenter.x, bboxCenter.y, bboxCenter.z) + CCVector3(0.0, 0.0, m_SurfacePos);
            }

            glFunc->glEnable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_ALWAYS);
            glFunc->glLineWidth(2.0);
            glFunc->glEnable(GL_LINE_SMOOTH);
            glFunc->glEnable(GL_BLEND);
            glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
            glFunc->glColor3ub(255, 200, 4);

            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINE_LOOP);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glBegin(GL_LINES);
            ccGL::Vertex3v(glFunc, tempMin.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMin.y, tempMax.z);
            ccGL::Vertex3(glFunc, tempMax.x, tempMax.y, tempMin.z);
            ccGL::Vertex3v(glFunc, tempMax.u);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMin.z);
            ccGL::Vertex3(glFunc, tempMin.x, tempMax.y, tempMax.z);
            glFunc->glEnd();

            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_LESS);
            glFunc->glDisable(GL_BLEND);
            glFunc->glDisable(GL_LINE_SMOOTH);
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPopMatrix();
            glFunc->glPopAttrib();

            break;
        }
        default:
            break;
        }

        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPopMatrix();
        if (m_open)
        {
            glFunc->glDisable(GL_POLYGON_OFFSET_FILL);
        }
        glFunc->glPopAttrib();

    }
    else if (MACRO_Draw2D(context))
    {
        return;
    }
    else if (MACRO_Foreground(context))
    {
        return;
    }


}



