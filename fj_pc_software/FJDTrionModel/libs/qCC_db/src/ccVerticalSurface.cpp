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

#include "ccVerticalSurface.h"
#include "ccHObjectCaster.h"


#include "qCC_db/include/ccGenericGLDisplay.h"

#include <Eigen/Geometry>

ccVerticalSurface::ccVerticalSurface(const ccGLMatrix* transMat, QString name)
	//: ccHObject(name.isEmpty() ? "Surface" : name)
    : ccGenericPrimitive(name, transMat)
	, m_SurfaceHeight(0.1)
	, m_SurfacePos(0.0)
	, m_Mincorner(0.0, 0.0, 0.0)
	, m_Maxcorner(0.0, 0.0, 0.0)
	, exec(true)
	, m_SliceType(YAXIS)
	, m_XOffset(0.0)
	, m_YOffset(0.0)
	, m_ZOffset(0.0)
	, m_DoubleClicked(false)
    , m_open(true)
    , m_scaling(1.0)
    , m_scalingOpen(false)
{
}

bool ccVerticalSurface::buildUp()
{
    return true;
}

ccGenericPrimitive* ccVerticalSurface::clone() const
{
    return nullptr;
}

void ccVerticalSurface::initShader(CC_DRAW_CONTEXT& context)
{

}

void ccVerticalSurface::updateSlicePos(CCVector3d pos)
{
	m_SurfacePos = 0.0;
	m_pos = pos;
	m_DoubleClicked = true;
}

void ccVerticalSurface::setScaling(float scaling)
{
    m_scaling = scaling;
    m_scalingOpen = true;
}


void ccVerticalSurface::draw(CC_DRAW_CONTEXT& context/*, ccBBox *pBox*/)
{
    if (!m_visible)
    {
        return;
    }
	if (m_KeyPoints.size() < 3)
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
		m_BoxVertexs.clear();
        CCVector3 bboxDiag = m_CalcBBox.getDiagVec();
        CCVector3 bboxCenter = m_CalcBBox.getCenter();
		CCVector3 maxVertex = m_CalcBBox.maxCorner();
		CCVector3 minVertex = m_CalcBBox.minCorner();
		double diagLength = CCVector3::vdistance(maxVertex.u, minVertex.u);

        if (m_open)
        {
            glFunc->glEnable(GL_POLYGON_OFFSET_FILL);
            glFunc->glPolygonOffset(-1, -1);
        }


        if (m_scalingOpen)
        {
			bboxDiag *= m_scaling;
        }


        {
            glFunc->glMatrixMode(GL_MODELVIEW);

			CCVector3d midPoint = CCVector3d((m_KeyPoints[0].x + m_KeyPoints[2].x)/2.0f,
				(m_KeyPoints[0].y + m_KeyPoints[2].y) / 2.0f, (m_KeyPoints[0].z + m_KeyPoints[2].z) / 2.0f);

			CCVector3d active = m_KeyPoints[0] - midPoint;
			active.normalize();
			CCVector3d negative = m_KeyPoints[2] - midPoint;
			negative.normalize();

			CCVector3d v1 = active * (diagLength*0.8) + midPoint;
			CCVector3d v2 = negative * (diagLength*0.8) + midPoint;


			double distance = CCVector3d::vdistance(v1.u, v2.u);

			CCVector3d vectorFirst = m_KeyPoints[1] - m_KeyPoints[0];
			CCVector3d vectorSecond = m_KeyPoints[2] - m_KeyPoints[1];
			CCVector3d::vcross(vectorFirst.u, vectorSecond.u, normalPlane.u);
			normalPlane.normalize();

			CCVector3d translatePos = normalPlane * m_SurfacePos;

			glFunc->glPushMatrix();
			glFunc->glTranslatef(translatePos.x, translatePos.y, translatePos.z);

            v1 = v1 - normalPlane * m_SurfaceHeight / 2.0;
            v2 = v2 - normalPlane * m_SurfaceHeight / 2.0;


			CCVector3d v3 = normalPlane * m_SurfaceHeight + v1;
			CCVector3d v4 = normalPlane * m_SurfaceHeight + v2;
            m_topPlaneCenterPoint = 0.5 * (v3 + v4) + translatePos;
			m_BBox = ccBBox(CCVector3(v4.x, v4.y, v4.z), CCVector3(v1.x, v1.y, v1.z));

			CCVector3d OPos(0, 0, 0);
			double dis = CCVector3d::vdistance(midPoint.u, OPos.u);
			CCVector3d normal2Mid = normalPlane ;
			CCVector3d mid_v2 = midPoint - v2;
            CCVector3d mid_v1 = midPoint - v1;

			//CCVector3d::vcross(mid_v2.u, normal2Mid.u, newNormal.u);
            CCVector3d::vcross(mid_v1.u, normal2Mid.u, newNormal.u);

			newNormal.normalize();

			CCVector3d calcPos1 = newNormal* (diagLength*0.8) + midPoint - normalPlane * m_SurfaceHeight / 2.0;
			CCVector3d calcPos2 = newNormal * (-(diagLength*0.8)) + midPoint - normalPlane * m_SurfaceHeight / 2.0;

			CCVector3d v7 = normalPlane * m_SurfaceHeight + calcPos1;
			CCVector3d v8 = normalPlane * m_SurfaceHeight + calcPos2;

			glFunc->glEnable(GL_BLEND);
			glFunc->glColor4ub(255, 200, 4, 80);

			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, v1.x, v1.y, v1.z);
			ccGL::Vertex3(glFunc, calcPos1.x, calcPos1.y, calcPos1.z);
			ccGL::Vertex3(glFunc, v2.x, v2.y, v2.z);
			ccGL::Vertex3(glFunc, calcPos2.x, calcPos2.y, calcPos2.z);
            glFunc->glEnd();

			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, v3.x, v3.y, v3.z);
			ccGL::Vertex3(glFunc, v7.x, v7.y, v7.z);
			ccGL::Vertex3(glFunc, v4.x, v4.y, v4.z);
			ccGL::Vertex3(glFunc, v8.x, v8.y, v8.z);
			glFunc->glEnd();

            
			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, v1.x, v1.y, v1.z);
			ccGL::Vertex3(glFunc, calcPos1.x, calcPos1.y, calcPos1.z);
			ccGL::Vertex3(glFunc, v7.x, v7.y, v7.z);
			ccGL::Vertex3(glFunc, v3.x, v3.y, v3.z);
			glFunc->glEnd();

			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, calcPos2.x, calcPos2.y, calcPos2.z);
			ccGL::Vertex3(glFunc, v2.x, v2.y, v2.z);
			ccGL::Vertex3(glFunc, v4.x, v4.y, v4.z);
			ccGL::Vertex3(glFunc, v8.x, v8.y, v8.z);
			glFunc->glEnd();

			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, v1.x, v1.y, v1.z);
			ccGL::Vertex3(glFunc, calcPos2.x, calcPos2.y, calcPos2.z);
			ccGL::Vertex3(glFunc, v8.x, v8.y, v8.z);
			ccGL::Vertex3(glFunc, v3.x, v3.y, v3.z);
			glFunc->glEnd();

			glFunc->glBegin(GL_QUADS);
			ccGL::Vertex3(glFunc, calcPos1.x, calcPos1.y, calcPos1.z);
			ccGL::Vertex3(glFunc, v2.x, v2.y, v2.z);
			ccGL::Vertex3(glFunc, v4.x, v4.y, v4.z);
			ccGL::Vertex3(glFunc, v7.x, v7.y, v7.z);
			glFunc->glEnd();

			glFunc->glDisable(GL_BLEND);


			m_BoxVertexs.push_back(calcPos1);
			m_BoxVertexs.push_back(v1);
			m_BoxVertexs.push_back(calcPos2);
			m_BoxVertexs.push_back(v2);

			m_BoxVertexs.push_back(v7);
			m_BoxVertexs.push_back(v3);
			m_BoxVertexs.push_back(v8);
			m_BoxVertexs.push_back(v4);


            glFunc->glEnable(GL_DEPTH_TEST);
            //glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glFunc->glDepthFunc(GL_ALWAYS);

			glFunc->glColor4ub(255, 200, 4, 255);
			glFunc->glLineWidth(2.0);
			glFunc->glEnable(GL_LINE_SMOOTH);
			glFunc->glEnable(GL_BLEND);
			glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);

			glFunc->glBegin(GL_LINE_LOOP);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[0].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[1].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[2].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[3].u);
			glFunc->glEnd();

			glFunc->glBegin(GL_LINE_LOOP);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[4].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[5].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[6].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[7].u);
			glFunc->glEnd();

			glFunc->glBegin(GL_LINES);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[0].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[4].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[1].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[5].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[2].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[6].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[3].u);
			ccGL::Vertex3v(glFunc, m_BoxVertexs[7].u);
			glFunc->glEnd();

            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glDepthFunc(GL_LESS);


			for( int i = 0; i< m_BoxVertexs.size(); i++)
			{
				CCVector3d vertex = m_BoxVertexs[i];
				m_BoxVertexs[i] = vertex + translatePos;
			}
            calcMinMaxCorner();
			glFunc->glDisable(GL_BLEND);
			glFunc->glDisable(GL_LINE_SMOOTH);

			glFunc->glPopMatrix();
        }


        if (m_open)
        {
            glFunc->glDisable(GL_POLYGON_OFFSET_FILL);
        }

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



void ccVerticalSurface::calcMinMaxCorner()
{
	m_Mincorner = CCVector3(m_BoxVertexs[0].x, m_BoxVertexs[0].y, m_BoxVertexs[0].z);
	m_Maxcorner = CCVector3(m_BoxVertexs[0].x, m_BoxVertexs[0].y, m_BoxVertexs[0].z);

	for (int i = 0; i < m_BoxVertexs.size(); i++)
	{
		CCVector3d P = m_BoxVertexs[i];
		if (P.x < m_Mincorner.x)
			m_Mincorner.x = P.x;
		else if (P.x > m_Maxcorner.x)
			m_Maxcorner.x = P.x;

		if (P.y < m_Mincorner.y)
			m_Mincorner.y = P.y;
		else if (P.y > m_Maxcorner.y)
			m_Maxcorner.y = P.y;

		if (P.z < m_Mincorner.z)
			m_Mincorner.z = P.z;
		else if (P.z > m_Maxcorner.z)
			m_Maxcorner.z = P.z;
	}
}


bool ccVerticalSurface::pointIsInside(const CCVector3d& p)
{
	if (m_BoxVertexs.size() <=0)
	{
		return false;
	}
	//首先判断点是否在左右两面的中间 此时法线为y轴
	CCVector3d vector_DP;
	CCVector3d vector_EP;
	CCVector3d vector_DE;//法线y
	//计算向量DE
	vector_DE.x = m_BoxVertexs[7].x - m_BoxVertexs[4].x;
	vector_DE.y = m_BoxVertexs[7].y - m_BoxVertexs[4].y;
	vector_DE.z = m_BoxVertexs[7].z - m_BoxVertexs[4].z;
	//计算向量DP
	vector_DP.x = p.x - m_BoxVertexs[4].x;
	vector_DP.y = p.y - m_BoxVertexs[4].y;
	vector_DP.z = p.z - m_BoxVertexs[4].z;
	//计算向量EP
	vector_EP.x = p.x - m_BoxVertexs[7].x;
	vector_EP.y = p.y - m_BoxVertexs[7].y;
	vector_EP.z = p.z - m_BoxVertexs[7].z;
	//计算向量点乘的结果
	//DP点乘DE
	double DP_DE;
	DP_DE = vector_DP.x*vector_DE.x + vector_DP.y*vector_DE.y + vector_DP.z*vector_DE.z;
	//EP点乘DE
	double EP_DE;
	EP_DE = vector_EP.x*vector_DE.x + vector_EP.y*vector_DE.y + vector_EP.z*vector_DE.z;

	//然后判断点是否在上下两面的中间 此时法线为z轴
	/*XYZ vector_DP;*/ //DP已经存在了 直接用上面的
	CCVector3d vector_AP;
	CCVector3d vector_AD;//法线y
	//计算向量AP
	vector_AP.x = p.x - m_BoxVertexs[0].x;
	vector_AP.y = p.y - m_BoxVertexs[0].y;
	vector_AP.z = p.z - m_BoxVertexs[0].z;
	//计算向量AD
	vector_AD.x = m_BoxVertexs[4].x - m_BoxVertexs[0].x;
	vector_AD.y = m_BoxVertexs[4].y - m_BoxVertexs[0].y;
	vector_AD.z = m_BoxVertexs[4].z - m_BoxVertexs[0].z;
	//计算向量点乘的结果
	//AD AP
	double AD_AP;
	AD_AP = vector_AD.x*vector_AP.x + vector_AD.y*vector_AP.y + vector_AD.z*vector_AP.z;
	//AD DP
	double AD_DP;
	AD_DP = vector_AD.x*vector_DP.x + vector_AD.y*vector_DP.y + vector_AD.z*vector_DP.z;

	//最后判断点是否在前后两面的中间 此时法线为x轴 
	CCVector3d vector_OA;//法线
	CCVector3d vector_OP;
	/*XYZ vector_AP;*/ //已有
	vector_OA.x = m_BoxVertexs[0].x - m_BoxVertexs[1].x;
	vector_OA.y = m_BoxVertexs[0].y - m_BoxVertexs[1].y;
	vector_OA.z = m_BoxVertexs[0].z - m_BoxVertexs[1].z;
	vector_OP.x = p.x - m_BoxVertexs[1].x;
	vector_OP.y = p.y - m_BoxVertexs[1].y;
	vector_OP.z = p.z - m_BoxVertexs[1].z;
	//计算向量点乘的结果
	//OP OA
	double OP_OA;
	OP_OA = vector_OP.x*vector_OA.x + vector_OP.y*vector_OA.y + vector_OP.z*vector_OA.z;
	//AP OA 
	double AP_OA;
	AP_OA = vector_AP.x*vector_OA.x + vector_AP.y*vector_OA.y + vector_AP.z*vector_OA.z;

	if (DP_DE*EP_DE < 0 && AD_AP*AD_DP < 0 && OP_OA*AP_OA < 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
