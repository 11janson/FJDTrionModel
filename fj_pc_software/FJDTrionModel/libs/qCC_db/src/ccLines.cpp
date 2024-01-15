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

#include "ccLines.h"
#include "cc2DRound.h"
#include "cc2DRect.h"
#include "cc2DArcLine.h"
//Local
#include "ccCone.h"
#include "ccPointCloud.h"
#include "ccPoint.h"
#include "geometryUtils.h"

ccLines::ccLines(QString name)
	: cc2DItemBase(name)
{
	set2DMode(false);
	setForeground(true);
	setVisible(true);
	lockVisibility(true);
	setColor(ccColor::Rgbaf(1, 0, 0, 1));
	showVertices(true);
	setVertexMarkerWidth(3);
	setWidth(2);
	showArrow(false, 0, 0);
	setClosed(false);
	setLocked(true);
    setEnabled(true);
    m_isClosed = false;
}

ccLines::ccLines(const ccLines& poly)
	: cc2DItemBase()
{
	ccPointCloud* vertices = nullptr;
	initWith(vertices, poly);
}

ccLines* ccLines::clone() const
{
	ccLines* clonedPoly = new ccLines(*this);
	clonedPoly->setLocked(false); //there's no reason to keep the clone locked

    clonedPoly->setName(getName());
    clonedPoly->setMetaData(this->metaData());
    clonedPoly->m_keyPoints3d = m_keyPoints3d;
    clonedPoly->m_keyPoints2d = m_keyPoints2d;

    clonedPoly->m_Type = m_Type;
    clonedPoly->get3dPoints();
	return clonedPoly;
}

bool ccLines::initWith(ccPointCloud*& vertices, const ccLines& poly)
{
	bool success = true;
	if (!vertices)
	{
		ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(poly.m_theAssociatedCloud);
		ccPointCloud* clone = cloud ? cloud->partialClone(&poly) : ccPointCloud::From(&poly);
		if (clone)
		{
			if (cloud)
				clone->setName(cloud->getName()); //as 'partialClone' adds the '.extract' suffix by default
			else
				clone->setGLTransformationHistory(poly.getGLTransformationHistory());
		}
		else
		{
			//not enough memory?
			ccLog::Warning("[ccLines::initWith] Not enough memory to duplicate vertices!");
			success = false;
		}

		vertices = clone;
	}

	if (vertices)
	{
		setAssociatedCloud(vertices);
		//addChild(vertices);
		//vertices->setEnabled(false);
		assert(m_theAssociatedCloud);
		if (m_theAssociatedCloud)
		{
			if (!addPointIndex(0, m_theAssociatedCloud->size()))
			{
				ccLog::Warning("[ccLines::initWith] Not enough memory");
				success = false;
			}
		}
	}

	importParametersFrom(poly);

	return success;
}

void ccLines::importParametersFrom(const ccLines& poly)
{
	setClosed(poly.m_isClosed);
	set2DMode(poly.m_mode2D);
	setForeground(poly.m_foreground);
	setVisible(poly.isVisible());
	lockVisibility(poly.isVisibilityLocked());
	setColor(poly.m_rgbColor);
	setWidth(poly.m_width);
	showColors(poly.colorsShown());
	showVertices(poly.verticesShown());
	setVertexMarkerWidth(poly.getVertexMarkerWidth());
	copyGlobalShiftAndScale(poly);
	setGLTransformationHistory(poly.getGLTransformationHistory());
	setMetaData(poly.metaData());
}

void ccLines::showArrow(bool state, unsigned vertIndex, PointCoordinateType length)
{

}

ccBBox ccLines::getOwnBB(bool withGLFeatures/*=false*/)
{
    ccBBox box;
    if (m_keyPoints3d.size() == 0)
        return  box;

    for (int i = 0; i < m_keyPoints3d.size(); i++)
        box.add(CCVector3(m_keyPoints3d[i].x, m_keyPoints3d[i].y, m_keyPoints3d[i].z));
    box.setValidity(!isSelected());
	return box;
}

bool ccLines::hasColors() const
{
	return true;
}

void ccLines::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//invalidate the bounding-box
	//(and we hope the vertices will be updated as well!)
	invalidateBoundingBox();
}

//unit arrow
static QSharedPointer<ccCone> c_unitArrow(nullptr);

bool ccLines::updatePoints(CCVector3d pts)
{
	if (m_keyPoints3d.size() <= 0 )
		return false;

    m_keyPoints3d.erase(m_keyPoints3d.end()-1);
	addDisplayPoints(pts);
	return true;
}

bool ccLines::addNewPoints(CCVector3d pts)
{
	if (m_first)
	{
		addDisplayPoints(pts);
		addDisplayPoints(pts);

		m_first = false;
	}
	else if (m_Type == LINE && m_keyPoints3d.size() == 2)
	{
		m_finish = true;
	}
	else if(m_Type == LINES)
	{
		addDisplayPoints(pts);
		return true;
	}
    if (m_finish)
        get3dPoints();
	return !m_finish;
}

bool ccLines::removePoints()
{
	if (m_keyPoints3d.size() <= 1)
	{
		return false;
	}
	if (m_Type == LINES)
	{
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
	}

	//m_finish = true;
	return false;
}

bool ccLines::accpectClickedPoint(void)
{
    if (m_keyPoints3d.size() >= 2 && m_Type == LINE && m_finish)
	{
        return false;
    }
	else if (m_Type == LINES && m_finish)
	{
		return false;
	}
    else
	{
        return true;
    }
    return true;
}

void ccLines::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (m_keyPoints3d.size() == 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    cc2DItemBase::drawMeOnly(context);

    if (MACRO_Draw3D(context))
        drawIn3dView(context);
    else if (MACRO_Draw2D(context))
        drawIn2dView(context);
}

void ccLines::drawIn3dView(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (context.display->windowName().compare("3D View") != 0)
        return;

    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
        glFunc->glPushName(getUniqueIDForDisplay());

    if (m_width != 0)
    {
        glFunc->glEnable(GL_LINE_SMOOTH);
        glFunc->glEnable(GL_BLEND);
        glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
        glFunc->glLineWidth(static_cast<GLfloat>(m_width));
    }

    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);

    get3dPoints();

    glFunc->glBegin(GL_LINE_STRIP);
    for (unsigned i = 0; i < m_keyPoints3d.size(); ++i)
        glFunc->glVertex3f(m_keyPoints3d[i].x, m_keyPoints3d[i].y, m_keyPoints3d[i].z);
    if (m_isClosed && m_keyPoints3d.size() > 0)
        glFunc->glVertex3f(m_keyPoints3d[0].x, m_keyPoints3d[0].y, m_keyPoints3d[0].z);
    glFunc->glEnd();

    //if (m_keyPoints3d.size() == 2)
    //    renderText(context, (m_keyPoints3d[0].x + m_keyPoints3d[1].x) / 2.0, (m_keyPoints3d[0].y + m_keyPoints3d[1].y) / 2.0, "LINES");


    if (m_width != 0)
    {
        glFunc->glDisable(GL_BLEND);
        glFunc->glDisable(GL_LINE_SMOOTH);
        glFunc->glPopAttrib();
    }
    if (pushName)
        glFunc->glPopName();
}

void ccLines::drawLineTrim(CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();

    if (!m_bTrim)
    {
		glFunc->glLineWidth(static_cast<GLfloat>(m_width));
		drawLines2D(context, m_keyPoints2d, m_rgbColor);
        //if(m_keyPoints2d.size() == 2)
        //    renderText(context, (m_keyPoints2d[0].x + m_keyPoints2d[1].x)/2.0, (m_keyPoints2d[0].y + m_keyPoints2d[1].y) / 2.0, "LINES", getUniqueID());
    }
    else if (m_bTrim && m_mouse2dpos.size() > 0 && m_lstIntersections.size() > 0)
    {
        sortVector3d(m_keyPoints2d, m_keyPoints2d[0].z);
        double z = m_keyPoints2d[0].z;

        std::vector<CCVector3d> copyKeyPoints = m_keyPoints2d;
        std::vector<CCVector3d> trimPoints;
        CCVector3d endPoint1;
        CCVector3d endPoint2;

        bool is_mouse_inSegment = false;
        int index1;
        int index2;
        if (m_lstIntersections.size() >= 2)
        {
            sortVector3d(m_lstIntersections, m_keyPoints2d[0].z);

            double sumLength = 0;
            for (int i = 0; i < m_lstIntersections.size() - 1; i++)
            {
                CCVector3d intersection1 = m_lstIntersections[i];
                CCVector3d intersection2 = m_lstIntersections[i + 1];
                bool result = on_segment(m_mouse2dpos[0], intersection1, intersection2);
                if (result)
                {
                    index1 = i;
                    index2 = i + 1;
                    is_mouse_inSegment = true;
                    break;
                }
            }
        }

        if (!is_mouse_inSegment)
        {
            if (m_lstIntersections.size() == 1)
            {
                CCVector3d intersection = CCVector3d(m_lstIntersections[0].x, m_lstIntersections[0].y, z);
                bool resultLeft = on_segment(m_mouse2dpos[0], copyKeyPoints[0], intersection);
                bool resultRight = on_segment(m_mouse2dpos[0], copyKeyPoints[1], intersection);

                if (resultLeft)
                {
                    trimPoints.push_back(copyKeyPoints[0]);
                    trimPoints.push_back(intersection);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect.push_back(intersection);
                    m_pt2dUnselect.push_back(copyKeyPoints[1]);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else if (resultRight)
                {
                    trimPoints.push_back(copyKeyPoints[1]);
                    trimPoints.push_back(intersection);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect.push_back(intersection);
                    m_pt2dUnselect.push_back(copyKeyPoints[0]);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else
                {
                    m_bAbnormal = true;
                    m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                    drawLines2D(context, m_keyPoints2d, m_rgbColor);
                }
            }
            else if (m_lstIntersections.size() > 1)
            {
                CCVector3d front, back;
                if (copyKeyPoints[0].x < copyKeyPoints[1].x)
                {
                    front = copyKeyPoints[0];
                    back = copyKeyPoints[1];
                }
                else
                {
                    front = copyKeyPoints[1];
                    back = copyKeyPoints[0];
                }
                CCVector3d firstIntersection = CCVector3d(m_lstIntersections[0].x, m_lstIntersections[0].y, z);
                CCVector3d lastIntersection = CCVector3d(m_lstIntersections[m_lstIntersections.size() - 1].x,
                    m_lstIntersections[m_lstIntersections.size() - 1].y, z);
                bool resultLeft = on_segment(m_mouse2dpos[0], front, firstIntersection);
                bool resultRight = on_segment(m_mouse2dpos[0], back, lastIntersection);
                if (resultLeft)
                {
                    trimPoints.push_back(front);
                    trimPoints.push_back(firstIntersection);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect.push_back(firstIntersection);
                    m_pt2dUnselect.push_back(back);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else if (resultRight)
                {
                    trimPoints.push_back(lastIntersection);
                    trimPoints.push_back(back);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect.push_back(front);
                    m_pt2dUnselect.push_back(lastIntersection);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else
                {
                    m_bAbnormal = true;
                    m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                    drawLines2D(context, m_keyPoints2d, m_rgbColor);
                }
            }
            else
            {
                trimPoints.push_back(copyKeyPoints[0]);
                trimPoints.push_back(copyKeyPoints[1]);
                drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));
            }
        }
        
        else
        {
            m_pt2dUnselect.push_back(copyKeyPoints[0]);
            m_pt2dUnselect.push_back(CCVector3d(m_lstIntersections[index1].x, m_lstIntersections[index1].y, z));
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

            trimPoints.push_back(m_lstIntersections[index1]);
            trimPoints.push_back(m_lstIntersections[index2]);
            drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

            m_pt2dUnselect2.push_back(copyKeyPoints[1]);
            m_pt2dUnselect2.push_back(CCVector3d(m_lstIntersections[index2].x, m_lstIntersections[index2].y, z));
            drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
        }

    }
    else
    {
        drawLines2D(context, m_keyPoints2d, m_rgbColor);
    }
}
void ccLines::drawPolylineTrim(CC_DRAW_CONTEXT& context)
{
    if (!m_bTrim)
    {
        drawLines2D(context, m_keyPoints2d, m_rgbColor);
    }
    else if (m_bTrim && m_mouse2dpos.size() > 0 && m_lstIntersections.size() > 0)
    {
        if (m_keyPoints2d.size() == 2)
        {
            m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
            drawLineTrim(context);
            return;
        }
        //查找鼠标在多段线的哪段上
        std::vector<CCVector3d> copyKeyPoints = m_keyPoints2d;
        double z = copyKeyPoints[0].z;
        int originIndex1 = -1;
        int originIndex2 = -1;
        for (size_t i = 0; i < copyKeyPoints.size() - 1; i++)
        {
            CCVector3d p1 = copyKeyPoints[i];
            CCVector3d p2 = copyKeyPoints[i + 1];
            bool result = on_segment(m_mouse2dpos[0], p1, p2);
            if (result)
            {
                originIndex1 = i;
                originIndex2 = i + 1;
                break;
            }
        }

        if (originIndex1 < 0 && originIndex2 < 0)
        {
            m_bAbnormal = true;
            m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
            drawLines2D(context, m_keyPoints2d, m_rgbColor);
            return;
        }

        std::vector<CCVector3d> intersectionInSegment;
        //找到在鼠标所在段的交点集合
        if (m_lstIntersections.size() >= 1 && originIndex1 >= 0 && originIndex2 >= 0)
        {
			if (copyKeyPoints[originIndex1].x < copyKeyPoints[originIndex2].x)
				sortVector3d(m_lstIntersections, z);
			else
				sortVector3d(m_lstIntersections, z, false);

            double sumLength = 0;
            for (int i = 0; i < m_lstIntersections.size(); i++)
            {
                CCVector3d intersection = m_lstIntersections[i];
                bool result = on_segment(CCVector3d(intersection.x, intersection.y, copyKeyPoints[originIndex1].z),
                    copyKeyPoints[originIndex1], copyKeyPoints[originIndex2]);
                if (result)
                {
                    intersectionInSegment.push_back(intersection);
                }
            }
        }
        std::vector<CCVector3d> trimPoints;
        CCVector3d endPoint1;
        CCVector3d endPoint2;

        bool is_mouse_inSegment = false;
        int index1 = -1;
        int index2 = -1;
        //鼠标在哪两个交点间
        if (intersectionInSegment.size() >= 2)
        {
            double sumLength = 0;
            for (int i = 0; i < intersectionInSegment.size() - 1; i++)
            {
                CCVector3d intersection1 = intersectionInSegment[i];
                CCVector3d intersection2 = intersectionInSegment[i + 1];
                bool result = on_segment(m_mouse2dpos[0], intersection1, intersection2);
                if (result)
                {
                    index1 = i;
                    index2 = i + 1;
                    is_mouse_inSegment = true;
                    break;
                }
            }
        }
        //当前线段只有一个交点
        if (intersectionInSegment.size() == 1)
        {
            double distanceIntersection = CCVector3d::vdistanced(m_mouse2dpos[0].u, intersectionInSegment[0].u);
            CCVector3d endPointInter = CCVector3d(intersectionInSegment[0].x, intersectionInSegment[0].y, z);
            //head
            if (originIndex2 == copyKeyPoints.size() - 1)
            {
                bool resultLeft = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex1]);
                bool resultRight = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex2]);

                if (resultLeft)
                {
                    for (int i = 0; i <= originIndex1; i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

                    trimPoints.push_back(copyKeyPoints[originIndex1]);
                    trimPoints.push_back(endPointInter);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect2.push_back(endPointInter);
                    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
                    {
                        m_pt2dUnselect2.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else if (resultRight)
                {
                    for (int i = 0; i <= originIndex1; i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    m_pt2dUnselect.push_back(endPointInter);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

                    trimPoints.push_back(endPointInter);
                    trimPoints.push_back(copyKeyPoints[originIndex2]);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));
                }
                else
                {
                    m_bAbnormal = true;
                    m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                    drawPolylineNoIntersection(context);
                }
            }
            //tail
            else if (originIndex1 == 0)
            {
                bool resultLeft = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex1]);
                bool resultRight = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex2]);
                if (resultLeft)
                {
                    trimPoints.push_back(copyKeyPoints[originIndex1]);
                    trimPoints.push_back(endPointInter);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect.push_back(endPointInter);
                    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else if (resultRight)
                {
                    for (int i = 0; i <= originIndex1; i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    m_pt2dUnselect.push_back(endPointInter);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

                    trimPoints.push_back(endPointInter);
                    trimPoints.push_back(copyKeyPoints[originIndex2]);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
                    {
                        m_pt2dUnselect2.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else
                {
                    m_bAbnormal = true;
                    m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                    drawPolylineNoIntersection(context);
                }
            }
            //body
            else
            {
                bool resultLeft = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex1]);
                bool resultRight = on_segment(m_mouse2dpos[0], endPointInter, copyKeyPoints[originIndex2]);
                if (resultLeft)
                {
                    for (int i = 0; i <= originIndex1; i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

                    trimPoints.push_back(copyKeyPoints[originIndex1]);
                    trimPoints.push_back(endPointInter);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    m_pt2dUnselect2.push_back(endPointInter);
                    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
                    {
                        m_pt2dUnselect2.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else if (resultRight)
                {
                    for (int i = 0; i <= originIndex1; i++)
                    {
                        m_pt2dUnselect.push_back(copyKeyPoints[i]);
                    }
                    m_pt2dUnselect.push_back(endPointInter);
                    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

                    trimPoints.push_back(endPointInter);
                    trimPoints.push_back(copyKeyPoints[originIndex2]);
                    drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

                    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
                    {
                        m_pt2dUnselect2.push_back(copyKeyPoints[i]);
                    }
                    drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
                }
                else
                {
                    m_bAbnormal = true;
                    m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                    drawPolylineNoIntersection(context);
                }
            }



        }
        else if(intersectionInSegment.size() == 0)
        {
            for (int i = 0; i <= originIndex1; i++)
            {
                m_pt2dUnselect.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

            trimPoints.push_back(copyKeyPoints[originIndex1]);
            trimPoints.push_back(copyKeyPoints[originIndex2]);
            drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));

            for (int i = originIndex2; i < copyKeyPoints.size(); i++)
            {
                m_pt2dUnselect2.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
        }
        else
        {
            intersectionInSegment.push_back(copyKeyPoints[originIndex1]);
            intersectionInSegment.push_back(copyKeyPoints[originIndex2]);
            sortVector3d(intersectionInSegment, z);


            for (int i = 0; i < intersectionInSegment.size() - 1; i++)
            {
                CCVector3d intersection1 = intersectionInSegment[i];
                CCVector3d intersection2 = intersectionInSegment[i + 1];
                bool result = on_segment(m_mouse2dpos[0], intersection1, intersection2);
                if (result)
                {
                    index1 = i;
                    index2 = i + 1;
                    is_mouse_inSegment = true;
                    break;
                }
            }
            if (index1<0 && index2 < 0)
            {
                m_bAbnormal = true;
                m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                drawLines2D(context, m_keyPoints2d, m_rgbColor);
                return;
            }
            trimPoints.push_back(intersectionInSegment[index1]);
            trimPoints.push_back(intersectionInSegment[index2]);
            drawLines2D(context, trimPoints, ccColor::Rgbaf(1, 1, 1, 1));
            int nSize = intersectionInSegment.size() - 1;
            if (index1 == 0)
            {
                m_pt2dUnselect2.push_back(intersectionInSegment[index2]);
                m_pt2dUnselect2.push_back(intersectionInSegment[nSize]);
                drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
            }
            else if (index2 == nSize)
            {
                m_pt2dUnselect2.push_back(intersectionInSegment[0]);
                m_pt2dUnselect2.push_back(intersectionInSegment[index1]);
                drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
            }
            else
            {
                m_pt2dUnselect2.push_back(intersectionInSegment[0]);
                m_pt2dUnselect2.push_back(intersectionInSegment[index1]);
                drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));

                m_pt2dUnselect3.push_back(intersectionInSegment[index2]);
                m_pt2dUnselect3.push_back(intersectionInSegment[nSize]);
                drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));
            }



            for (int i = 0; i <= originIndex1; i++)
            {
                m_pt2dUnselect.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));



            for (int i = originIndex2; i < copyKeyPoints.size(); i++)
            {
                m_pt2dUnselect4.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect4, ccColor::Rgbaf(1, 0, 0, 1));

        }
    }
    else//无交点，分段修剪多段线                        
    {
        m_rgbColor = ccColor::Rgbaf(1, 1, 1, 1);
        drawPolylineNoIntersection(context);
    }
}

void ccLines::drawPolylineNoIntersection(CC_DRAW_CONTEXT& context)
{
    //查找鼠标在多段线的哪段上
    std::vector<CCVector3d> copyKeyPoints = m_keyPoints2d;
    double z = copyKeyPoints[0].z;
    int originIndex1 = -1;
    int originIndex2 = -1;
    for (int i = 0; i < copyKeyPoints.size() - 1; i++)
    {
        CCVector3d p1 = copyKeyPoints[i];
        CCVector3d p2 = copyKeyPoints[i + 1];
        bool result = on_segment(m_mouse2dpos[0], p1, p2);
        if (result)
        {
            originIndex1 = i;
            originIndex2 = i + 1;
            break;
        }
    }
    if (originIndex1<0 || originIndex2<0)
    {
        m_bAbnormal = true;
        m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
        drawLines2D(context, m_keyPoints2d, m_rgbColor);
        return;
    }
    std::vector<CCVector3d> trimPoints;

    for (int i = 0; i <= originIndex1; i++)
    {
        m_pt2dUnselect.push_back(copyKeyPoints[i]);
    }
    drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

    trimPoints.push_back(copyKeyPoints[originIndex1]);
    trimPoints.push_back(copyKeyPoints[originIndex2]);
    drawLines2D(context, trimPoints, m_rgbColor);

    for (int i = originIndex2; i < copyKeyPoints.size(); i++)
    {
        m_pt2dUnselect2.push_back(copyKeyPoints[i]);
    }
    drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));
}

void ccLines::renderText(CC_DRAW_CONTEXT& context, int x, int y, const QString & str, uint16_t uniqueID/*=0*/, const QFont & pFont/*=QFont()*/, const ccColor::Rgba* backcolor/*=QFont()*/)
{
    //if (m_activeFbo)
    //{
    //    m_activeFbo->start();
    //}

    //ccQOpenGLFunctions* glFunc = functions();
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();

    assert(glFunc);

    //retrieve the texture
    SharedTexture texture;
    if (uniqueID != 0)
    {
        if (m_uniqueTextures.contains(uniqueID))
        {
            //retrieve the texture
            texture = m_uniqueTextures[uniqueID];
        }
        else
        {
            //register it for later
            texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
            m_uniqueTextures.insert(uniqueID, texture);
        }
    }
    else
    {
        if (m_texturePoolLastIndex < m_texturePool.size())
        {
            //retrieve the texture
            texture = m_texturePool[m_texturePoolLastIndex++];
        }
        else
        {
            texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
            try
            {
                m_texturePool.push_back(texture);
                ++m_texturePoolLastIndex;
            }
            catch (const std::bad_alloc&)
            {
                //not enough memory to keep the texture?!
            }
        }
    }
    assert(texture);

    //compute the text bounding rect
    // This adjustment and the change to x & y are to work around a crash with Qt 5.9.
    // At the time I (Andy) could not determine if it is a bug in CC or Qt.
    //		https://bugreports.qt.io/browse/QTBUG-61863
    //		https://github.com/CloudCompare/CloudCompare/issues/543
    QRect textRect = QFontMetrics(pFont).boundingRect(str).adjusted(-1, -2, 1, 2);
    //ccLog::Print(QString("Texture rect = (%1 ; %2) --> (%3 x %4)").arg(textRect.x()).arg(textRect.y()).arg(textRect.width()).arg(textRect.height()));

    x -= 1;	// magic number!
    y += 3;	// magic number!

    QSize imageSize;
    if (texture->isStorageAllocated())
    {
        if (textRect.width() > texture->width() || textRect.height() > texture->height())
        {
            //we have to enlarge it
            texture->destroy();
            imageSize = textRect.size();
        }
        else
        {
            imageSize = QSize(texture->width(), texture->height());
        }
    }
    else
    {
        imageSize = textRect.size();
    }

    // We create a QImage from the text
    QImage textImage(imageSize.width(), imageSize.height(), QImage::Format::Format_RGBA8888);
    QRect imageRect = textImage.rect();
    //ccLog::Print(QString("Image rect = (%1 ; %2) --> (%3 x %4)").arg(imageRect.x()).arg(imageRect.y()).arg(imageRect.width()).arg(imageRect.height()));

    textImage.fill(Qt::transparent);
    {
        if (backcolor)
        {
            textImage.fill(QColor(backcolor->r, backcolor->g, backcolor->b));
        }
        QPainter painter(&textImage);

        float glColor[4];
        glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
        QColor color;
        color.setRgbF(glColor[0], glColor[1], glColor[2], glColor[3]);

        painter.setPen(color);
        painter.setFont(pFont);
        painter.drawText(imageRect, Qt::AlignLeft, str);
    }

    //and then we convert this QImage to a texture!
    {
        glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
        glFunc->glEnable(GL_BLEND);
        glFunc->glDisable(GL_DEPTH_TEST);

        //set ortho view with center in the upper-left corner
        glFunc->glMatrixMode(GL_PROJECTION);
        glFunc->glPushMatrix();
        glFunc->glLoadIdentity();
        glFunc->glOrtho(0, context.glW, 0, context.glH, -1, 1);
        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPushMatrix();
        glFunc->glLoadIdentity();
        {
            //move to the right position on the screen
            glFunc->glTranslatef(x, y/*context.glH - 1 - y*/, 0);

            glFunc->glEnable(GL_TEXTURE_2D);

            if (texture->height() < textRect.height())
            {
                //we have to re-create it!
                texture->destroy();
            }

            //In order to reduce the time ATI cards take to manage the texture ID generation
            //and switching, we re-use the textures as much as possible.
            //texture->setData(textImage, QOpenGLTexture::DontGenerateMipMaps);
            if (!texture->isStorageAllocated())
            {
                //ccLog::Print(QString("New texture allocated: %1 x %2").arg(imageRect.width()).arg(imageRect.height()));
                texture->setMinificationFilter(QOpenGLTexture::Linear);
                texture->setMagnificationFilter(QOpenGLTexture::Linear);
                texture->setFormat(QOpenGLTexture::RGBA8_UNorm);
                texture->setSize(imageRect.width(), imageRect.height());
                texture->setMipLevels(0);
                texture->allocateStorage();
            }
            texture->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt32_RGBA8_Rev, textImage.bits());
            texture->bind();

            glFunc->glColor4f(1.0f, 1.0f, 1.0f, 1.0f); //DGM: warning must be float colors to work properly?!
            glFunc->glBegin(GL_QUADS);
            float ratioW = textRect.width() / static_cast<float>(imageRect.width());
            float ratioH = textRect.height() / static_cast<float>(imageRect.height());
            glFunc->glTexCoord2f(0, ratioH); glFunc->glVertex3i(0, 0, 0);
            glFunc->glTexCoord2f(ratioW, ratioH); glFunc->glVertex3i(textRect.width(), 0, 0);
            glFunc->glTexCoord2f(ratioW, 0); glFunc->glVertex3i(textRect.width(), textRect.height(), 0);
            glFunc->glTexCoord2f(0, 0); glFunc->glVertex3i(0, textRect.height(), 0);
            glFunc->glEnd();

            texture->release();
        }

        glFunc->glMatrixMode(GL_PROJECTION);
        glFunc->glPopMatrix();
        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPopMatrix();

        glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT
    }
}

void ccLines::drawIn2dView(CC_DRAW_CONTEXT& context)
{
    if (m_bTrim && m_mouse2dpos.size() == 0)
    {
        return;
    }
    if (context.display->windowName().compare("2D View") != 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
        glFunc->glPushName(getUniqueIDForDisplay());

    if (m_width != 0)
    {
        glFunc->glEnable(GL_LINE_SMOOTH);
        glFunc->glEnable(GL_BLEND);
        glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
        glFunc->glLineWidth(static_cast<GLfloat>(m_width));
    }

    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(-halfW), static_cast<GLfloat>(-halfH), 0);

    m_pt2dUnselect4.clear();
    m_pt2dUnselect3.clear();
    m_pt2dUnselect2.clear();
    m_pt2dUnselect.clear();

    if (getItemType() == LINE)
    {
        drawLineTrim(context);
    }
    else if (getItemType() == LINES)
    {
        drawPolylineTrim(context);
    }  
    glFunc->glPopMatrix();

    if (m_width != 0)
    {
        glFunc->glDisable(GL_BLEND);
        glFunc->glDisable(GL_LINE_SMOOTH);
        glFunc->glPopAttrib();
    }

    if (pushName)
        glFunc->glPopName();
}


bool ccLines::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[ccLines::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
		return false;
	}
	uint32_t vertUniqueID = (m_theAssociatedCloud ? (uint32_t)vertices->getUniqueID() : 0);
	if (out.write((const char*)&vertUniqueID, 4) < 0)
		return WriteError();

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = size();
	if (out.write((const char*)&pointCount, 4) < 0)
		return WriteError();

	//points (references to) (dataVersion>=28)
	for (uint32_t i = 0; i < pointCount; ++i)
	{
		uint32_t pointIndex = getPointGlobalIndex(i);
		if (out.write((const char*)&pointIndex, 4) < 0)
			return WriteError();
	}

	//'global shift & scale' (dataVersion>=39)
	saveShiftInfoToFile(out);

	QDataStream outStream(&out);

	//Closing state (dataVersion>=28)
	outStream << m_isClosed;

	//RGB Color (dataVersion>=28)
	outStream << m_rgbColor.r;
	outStream << m_rgbColor.g;
	outStream << m_rgbColor.b;

	//2D mode (dataVersion>=28)
	outStream << m_mode2D;

	//Foreground mode (dataVersion>=28)
	outStream << m_foreground;

	//The width of the line (dataVersion>=31)
	outStream << m_width;

	return true;
}

bool ccLines::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	if (dataVersion < 28)
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by multiple polylines)
	//we only store its unique ID (dataVersion>=28) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_theAssociatedCloud) = vertUniqueID;

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = 0;
	if (in.read((char*)&pointCount, 4) < 0)
		return ReadError();
	if (!reserve(pointCount))
		return false;

	//points (references to) (dataVersion>=28)
	for (uint32_t i = 0; i < pointCount; ++i)
	{
		uint32_t pointIndex = 0;
		if (in.read((char*)&pointIndex, 4) < 0)
			return ReadError();
		addPointIndex(pointIndex);
	}

	//'global shift & scale' (dataVersion>=39)
	if (dataVersion >= 39)
	{
		if (!loadShiftInfoFromFile(in))
			return ReadError();
	}
	else
	{
		m_globalScale = 1.0;
		m_globalShift = CCVector3d(0,0,0);
	}

	QDataStream inStream(&in);

	//Closing state (dataVersion>=28)
	inStream >> m_isClosed;

	//RGB Color (dataVersion>=28)
	inStream >> m_rgbColor.r;
	inStream >> m_rgbColor.g;
	inStream >> m_rgbColor.b;

	//2D mode (dataVersion>=28)
	inStream >> m_mode2D;

	//Foreground mode (dataVersion>=28)
	inStream >> m_foreground;

	//Width of the line (dataVersion>=31)
	if (dataVersion >= 31)
		ccSerializationHelper::CoordsFromDataStream(inStream,flags,(PointCoordinateType*)&m_width,1);
	else
		m_width = 0;

	return true;
}

bool ccLines::split(	PointCoordinateType maxEdgeLength,
						std::vector<ccLines*>& parts)
{
	parts.clear();

	//not enough vertices?
	unsigned vertCount = size();
	if (vertCount <= 2)
	{
		parts.push_back(new ccLines(*this));
		return true;
	}

	unsigned startIndex = 0;
	unsigned lastIndex = vertCount-1;
	while (startIndex <= lastIndex)
	{
		unsigned stopIndex = startIndex;
		while (stopIndex < lastIndex && (*getPoint(stopIndex+1) - *getPoint(stopIndex)).norm() <= maxEdgeLength)
		{
			++stopIndex;
		}

		//number of vertices for the current part
		unsigned partSize = stopIndex-startIndex+1;

		//if the polyline is closed we have to look backward for the first segment!
		if (startIndex == 0)
		{
			if (isClosed())
			{
				unsigned realStartIndex = vertCount;
				while (realStartIndex > stopIndex && (*getPoint(realStartIndex-1) - *getPoint(realStartIndex % vertCount)).norm() <= maxEdgeLength)
				{
					--realStartIndex;
				}

				if (realStartIndex == stopIndex)
				{
					//whole loop
					parts.push_back(new ccLines(*this));
					return true;
				}
				else if (realStartIndex < vertCount)
				{
					partSize += (vertCount - realStartIndex);
					assert(realStartIndex != 0);
					lastIndex = realStartIndex-1;
					//warning: we shift the indexes!
					startIndex = realStartIndex; 
					stopIndex += vertCount;
				}
			}
			else if (partSize == vertCount)
			{
				//whole polyline
				parts.push_back(new ccLines(*this));
				return true;
			}
		}

		if (partSize > 1) //otherwise we skip that point
		{
			//create the corresponding part
			CCCoreLib::ReferenceCloud ref(m_theAssociatedCloud);
			if (!ref.reserve(partSize))
			{
				ccLog::Error("[ccLines::split] Not enough memory!");
				return false;
			}

			for (unsigned i=startIndex; i<=stopIndex; ++i)
			{
				ref.addPointIndex(i % vertCount);
			}

			//ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
			//ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
			//ccLines* part = new ccLines(subset);
			//part->initWith(subset, *this);
			//part->setClosed(false); //by definition!
			//parts.push_back(part);
		}

		//forward
		startIndex = (stopIndex % vertCount) + 1;
	}

	return true;
}

PointCoordinateType ccLines::computeLength() const
{
	PointCoordinateType length = 0;

	unsigned vertCount = size();
	if (vertCount > 1 && m_theAssociatedCloud)
	{
		unsigned lastVert = isClosed() ? vertCount : vertCount - 1;
		for (unsigned i = 0; i < lastVert; ++i)
		{
			CCVector3 A;
			getPoint(i, A);
			CCVector3 B;
			getPoint((i + 1) % vertCount, B);

			length += (B - A).norm();
		}
	}

	return length;
}

unsigned ccLines::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
}

unsigned ccLines::segmentCount() const
{
	unsigned count = size();
	if (count && !isClosed())
	{
		--count;
	}
	return count;
}

void ccLines::setGlobalShift(const CCVector3d& shift)
{
	ccShiftedObject::setGlobalShift(shift);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global shift info to the vertices
		pc->setGlobalShift(shift);
	}
}

void ccLines::setGlobalScale(double scale)
{
	ccShiftedObject::setGlobalScale(scale);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global scale info to the vertices
		pc->setGlobalScale(scale);
	}
}

const CCVector3d& ccLines::getGlobalShift() const
{
	const ccPointCloud* pc = dynamic_cast<const ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//by default we use the vertices global shift info
		return pc->getGlobalShift();
	}
	else
	{
		return ccShiftedObject::getGlobalShift();
	}
}

double ccLines::getGlobalScale() const
{
	const ccPointCloud* pc = dynamic_cast<const ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//by default we use the vertices global scale info
		return pc->getGlobalScale();
	}
	else
	{
		return ccShiftedObject::getGlobalScale();
	}
}

ccPointCloud* ccLines::samplePoints(	bool densityBased,
										double samplingParameter,
										bool withRGB)
{
	if (samplingParameter <= 0 || size() < 2)
	{
		assert(false);
		return nullptr;
	}

	//we must compute the total length of the polyline
	double L = this->computeLength();

	unsigned pointCount = 0;
	if (densityBased)
	{
		pointCount = static_cast<unsigned>(ceil(L * samplingParameter));
	}
	else
	{
		pointCount = static_cast<unsigned>(samplingParameter);
	}

	if (pointCount == 0)
	{
		assert(false);
		return nullptr;
	}

	//convert to real point cloud
	ccPointCloud* cloud = new ccPointCloud(getName() + "." + QObject::tr("sampled"));
	if (!cloud->reserve(pointCount))
	{
		ccLog::Warning("[ccLines::samplePoints] Not enough memory");
		delete cloud;
		return nullptr;
	}

	double samplingStep = L / pointCount;
	double s = 0.0; //current sampled point curvilinear position
	unsigned indexA = 0; //index of the segment start vertex
	double sA = 0.0; //curvilinear pos of the segment start vertex

	for (unsigned i = 0; i < pointCount; )
	{
		unsigned indexB = ((indexA + 1) % size());
		const CCVector3& A = *getPoint(indexA);
		const CCVector3& B = *getPoint(indexB);
		CCVector3 AB = B - A;
		double lAB = AB.normd();

		double relativePos = s - sA;
		if (relativePos >= lAB)
		{
			//specific case: last point
			if (i + 1 == pointCount)
			{
				assert(relativePos < lAB * 1.01); //it should only be a rounding issue in the worst case
				relativePos = lAB;
			}
			else //skip this segment
			{
				++indexA;
				sA += lAB;
				continue;
			}
		}

		//now for the interpolation work
		double alpha = relativePos / lAB;
		alpha = std::max(alpha, 0.0); //just in case
		alpha = std::min(alpha, 1.0);

		CCVector3 P = A + static_cast<PointCoordinateType>(alpha) * AB;
		cloud->addPoint(P);

		//proceed to the next point
		++i;
		s += samplingStep;
	}

	if (withRGB)
	{
		if (isColorOverridden())
		{
			//we use the default 'temporary' color
			cloud->setColor(getTempColor());
		}
		else if (colorsShown())
		{
			//we use the default color
			cloud->setColor(ccColor::Rgba(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a));
		}
	}

	//import parameters from the source
	cloud->copyGlobalShiftAndScale(*this);
	cloud->setGLTransformationHistory(getGLTransformationHistory());

	return cloud;
}

ccLines* ccLines::smoothChaikin(PointCoordinateType ratio, unsigned iterationCount) const
{
	if (iterationCount == 0)
	{
		assert(false);
		ccLog::Warning("[ccLines::smoothChaikin] Invalid input (iteration count)");
		return nullptr;
	}

	if (ratio < 0.05f || ratio > 0.45f)
	{
		assert(false);
		ccLog::Warning("[ccLines::smoothChaikin] invalid ratio");
		return nullptr;
	}

	if (size() < 3)
	{
		ccLog::Warning("[ccLines::smoothChaikin] not enough segments");
		return nullptr;
	}

	const CCCoreLib::GenericIndexedCloudPersist* currentIterationVertices = this; //a polyline is actually a ReferenceCloud!
	ccLines* smoothPoly = nullptr;

	bool openPoly = !isClosed();

	for (unsigned it = 0; it < iterationCount; ++it)
	{
		//reserve memory for the new vertices
		unsigned vertCount = currentIterationVertices->size();
		unsigned segmentCount = (openPoly ? vertCount - 1 : vertCount);

		ccPointCloud* newStateVertices = new ccPointCloud("vertices");
		if (!newStateVertices->reserve(segmentCount * 2))
		{
			ccLog::Warning("[ccLines::smoothChaikin] not enough memory");
			delete newStateVertices;
			newStateVertices = nullptr;
			delete currentIterationVertices;
			currentIterationVertices = nullptr;
			return nullptr;
		}

		if (openPoly)
		{
			//we always keep the first vertex
			newStateVertices->addPoint(*currentIterationVertices->getPoint(0));
		}

		for (unsigned i = 0; i < segmentCount; ++i)
		{
			unsigned iP = i;
			unsigned iQ = ((iP + 1) % vertCount);

			const CCVector3& P = *currentIterationVertices->getPoint(iP);
			const CCVector3& Q = *currentIterationVertices->getPoint(iQ);

			if (!openPoly || i != 0)
			{
				CCVector3 P0 = (CCCoreLib::PC_ONE - ratio) * P + ratio * Q;
				newStateVertices->addPoint(P0);
			}

			if (!openPoly || i + 1 != segmentCount)
			{
				CCVector3 P1 = ratio * P + (CCCoreLib::PC_ONE - ratio) * Q;
				newStateVertices->addPoint(P1);
			}
		}

		if (openPoly)
		{
			//we always keep the last vertex
			newStateVertices->addPoint(*currentIterationVertices->getPoint(currentIterationVertices->size() - 1));
		}

		if (currentIterationVertices != this)
		{
			delete currentIterationVertices;
			currentIterationVertices = nullptr;
		}
		currentIterationVertices = newStateVertices;

		//last iteration?
		if (it + 1 == iterationCount)
		{
			//smoothPoly = new ccLines(newStateVertices);
			//smoothPoly->addChild(newStateVertices);
			//newStateVertices->setEnabled(false);
			//if (!smoothPoly->reserve(newStateVertices->size()))
			//{
			//	ccLog::Warning("[ccLines::smoothChaikin] not enough memory");
			//	delete smoothPoly;
			//	return nullptr;
			//}
			//smoothPoly->addPointIndex(0, newStateVertices->size());

			////copy state
			//smoothPoly->importParametersFrom(*this);
			//smoothPoly->setName(getName() + QString(".smoothed (ratio=%1)").arg(ratio));
		}
	}

	return smoothPoly;
}

bool ccLines::IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, ccLines** polyline/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a polyline
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::POLY_LINE) && static_cast<ccLines*>(parent)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<ccLines*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::POLY_LINE) && static_cast<ccLines*>(child)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<ccLines*>(child);
			}
			return true;
		}
	}

	return false;
}

void ccLines::get3dPoints()
{
    
}

bool ccLines::createNewPolylinesFromSelection(std::vector<ccLines*>& output)
{
	if (!m_theAssociatedCloud)
	{
		assert(false);
		return false;
	}
	unsigned vertCount = m_theAssociatedCloud->size();
	
	//vertices visibility
	ccGenericPointCloud* verticesCloud = dynamic_cast<ccGenericPointCloud*>(getAssociatedCloud());
	if (!verticesCloud)
	{
		// no visibility table instantiated
		ccLog::Warning("[ccLines::createNewPolylinesFromSelection] Unsupported vertex cloud");
		return false;
	}
	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = verticesCloud->getTheVisibilityArray();
	if (verticesVisibility.size() < vertCount)
	{
		// no visibility table instantiated
		ccLog::Warning("[ccLines::createNewPolylinesFromSelection] No visibility table instantiated");
		return false;
	}

	bool success = true;
	{
		ccLines* chunkPoly = nullptr;
		ccPointCloud* chunkCloud = nullptr;

		unsigned maxIndex = (m_isClosed ? vertCount : vertCount - 1);
		for (unsigned i = 0; i < maxIndex; ++i)
		{
			unsigned nextIndex = ((i + 1) % vertCount);
			bool kept = false;
			if (verticesVisibility.at(i) == CCCoreLib::POINT_VISIBLE && verticesVisibility.at(nextIndex) == CCCoreLib::POINT_VISIBLE) // segment should be kept
			{
				kept = true;

				const CCVector3* P0 = getPoint(i);
				const CCVector3* P1 = getPoint(nextIndex);

				// recreate a chunk if none is ready yet
				static const unsigned DefaultPolySizeIncrement = 64;
				if (!chunkPoly)
				{
					/*chunkCloud = new ccPointCloud("vertices");
					chunkCloud->setEnabled(false);
					chunkPoly = new ccLines(chunkCloud);
					chunkPoly->addChild(chunkCloud);
					if (!chunkPoly->reserve(DefaultPolySizeIncrement) || !chunkCloud->reserve(DefaultPolySizeIncrement))
					{
						delete chunkCloud;
						success = false;
						break;
					}
					chunkPoly->addPointIndex(0);
					chunkCloud->addPoint(*P0);*/
				}
				else if (chunkPoly->size() == chunkPoly->capacity())
				{
					if (!chunkPoly->reserve(chunkPoly->size() + DefaultPolySizeIncrement) || !chunkCloud->reserve(chunkCloud->size() + DefaultPolySizeIncrement))
					{
						success = false;
						break;
					}
				}

				// add the next vertex
				chunkPoly->addPointIndex(chunkCloud->size());
				chunkCloud->addPoint(*P1);
			}

			if (!kept || i + 1 == maxIndex)
			{
				// store the active chunk (if any)
				if (chunkPoly)
				{
					chunkPoly->importParametersFrom(*this);
					chunkPoly->setName(getName() + QString(".segmented (part %1)").arg(output.size() + 1));
					chunkCloud->shrinkToFit();
					chunkPoly->resize(chunkPoly->size());
					try
					{
						output.push_back(chunkPoly);
					}
					catch (const std::bad_alloc&)
					{
						success = false;
						break;
					}
					chunkPoly = nullptr;
				}
			}
		}
	}

	if (!success)
	{
		ccLog::Warning("[ccLines::createNewPolylinesFromSelection] Not enough memory");
		// delete the already created polylines
		for (ccLines* poly : output)
		{
			delete poly;
		}
		output.clear();
	}

	return success;
}

std::vector<CCVector2d> ccLines::intersect(ccLines* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;
    Segment_2 s;
    Segment_2 s1;

    for (int i = 0; i < this->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[i].x, this->get2dKeyPoints()[i].y);
        CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[i + 1].x, this->get2dKeyPoints()[i + 1].y);
        if (m_bExtendInterect)
        {
            if (i != 0 && i != get2dKeyPoints().size() - 2)
                continue;
        }

        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line1(p1, p2);
        s = Segment_2(p1, p2);

        for (int j = 0; j < pItem->get2dKeyPoints().size() - 1; j++)
        {
            q1 = CCVector2d(pItem->get2dKeyPoints()[j].x, pItem->get2dKeyPoints()[j].y);
            q2 = CCVector2d(pItem->get2dKeyPoints()[j + 1].x, pItem->get2dKeyPoints()[j + 1].y);
            if (q1 == q2)
                continue;

            p1 = Point_2(q1.x, q1.y);
            p2 = Point_2(q2.x, q2.y);
            Line_2 line2(p1, p2);
            s1 = Segment_2(p1, p2);


            typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Line_2>::type Intersection_result;
            std::vector<Intersection_result> res;

            try
            {
                CGAL::intersection(line1, line2, std::back_inserter(res));
            }
            catch(...)
            {
                qDebug() << "The intersection calculation failed.";
                return lstIntersection;
            }
            if (res.size() <= 0)
            {
                continue;
            }
            using boostRetVal = CGAL::Point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >;

            for (auto element : res)
            {
                if (!element)
                {
                    continue;
                }
                auto algPoint = boost::get<Point_2>(&*element);
                if (algPoint)
                {
                    auto point = Point_2(CGAL::to_double(algPoint->x()), CGAL::to_double(algPoint->y()));
                    std::cout << point << std::endl;

                    double x = CGAL::to_double(point.x());
                    double y = CGAL::to_double(point.y());
                    CCVector2d pos(x, y);
                    if (pos.norm() > 0)
                    {
                        intersections.push_back(pos);
                    }
                    std::cout << "x = " << pos.x << "y = " << pos.y << std::endl;
                }
            }
            if (filter)
            {
                if (m_bExtendInterect)
                {
                    for (int k = intersections.size() - 1; k >= 0; k--)
                    {
                        CCVector2d pos = intersections[k];
                        Point_2 p(pos.x, pos.y);

                        if (!is_point_on_segment(p, s) && is_point_on_segment(p, s1))
                        {
                            std::cout << "The point is on the segment." << std::endl;
                        }
                        else
                        {
                            std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                                return abs(temp.norm() - pos.norm()) < EPSINON;
                            });
                            if (it != intersections.end())
                            {
                                intersections.erase(it);
                            }
                        }
                    }
                }
                else
                {
                    for (int k = intersections.size() - 1; k >= 0; k--)
                    {
                        CCVector2d pos = intersections[k];
                        Point_2 p(pos.x, pos.y);

                        if (!is_point_on_segment(p, s) || !is_point_on_segment(p, s1))
                        {
                            std::cout << "The point is on the segment." << std::endl;
                        }
                        else
                        {
                            std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                                return abs(temp.norm() - pos.norm()) < EPSINON;
                            });
                            if (it != intersections.end())
                            {
                                intersections.erase(it);
                            }
                        }
                    }
                }
                
            }
            else
            {
                for (int k = intersections.size() - 1; k >= 0; k--)
                {
                    CCVector2d pos = intersections[k];
                    Point_2 p(pos.x, pos.y);

                    if (is_point_on_segment(p, s) && is_point_on_segment(p, s1))
                    {
                        std::cout << "The point is on the segment." << std::endl;
                    }
                    else
                    {
                        std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                            return abs(temp.norm() - pos.norm()) < EPSINON;
                        });
                        if (it != intersections.end())
                        {
                            intersections.erase(it);
                        }
                    }
                }
            }
            
            foreach(CCVector2d intersection, intersections)
            {
                lstIntersection.push_back(intersection);
            }
        }
    }
    return lstIntersection;
}

std::vector<CCVector2d> ccLines::intersect(cc2DArcLine* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    for (int i = 0; i < this->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[i].x, this->get2dKeyPoints()[i].y);
        CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[i + 1].x, this->get2dKeyPoints()[i + 1].y);
        if (m_bExtendInterect)
        {
            if (i != 0 && i != get2dKeyPoints().size() - 2)
                continue;
        }

        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line(p1, p2);
        Segment_2 s(p1, p2);


        q1 = CCVector2d(pItem->get2dKeyPoints()[0].x, pItem->get2dKeyPoints()[0].y);
        q2 = CCVector2d(pItem->get2dKeyPoints()[1].x, pItem->get2dKeyPoints()[1].y);
        CCVector2d q3 = CCVector2d(pItem->get2dKeyPoints()[2].x, pItem->get2dKeyPoints()[2].y);

        p1 = Point_2(q1.x, q1.y);
        p2 = Point_2(q2.x, q2.y);
        Point_2 p3 = Point_2(q3.x, q3.y);

        if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
            std::swap(p1, p3);

        Circular_arc_2 arc;
		typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circular_arc_2>::type Intersection_result;
		std::vector<Intersection_result> res;
        try
        {
            arc = Circular_arc_2(p1, p2, p3);
            CGAL::intersection(line, arc, std::back_inserter(res));
        }
        catch (...)
        {
            qDebug() << "The intersection calculation failed.";
            return lstIntersection;
        }
        if (res.size() <= 0)
        {
            continue;
        }
        using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;

        for (const auto& element : res)
        {
            auto algPoint = std::get<0>(boost::get< boostRetVal >(element));

            if (&algPoint)
            {
                auto point = Point_2(CGAL::to_double(algPoint.x()), CGAL::to_double(algPoint.y()));
                std::cout << point << std::endl;

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                CCVector2d pos(x, y);
                intersections.push_back(pos);

                std::cout << pos.x << pos.y << std::endl;
            }
        }
        if (!filter)
        {
            for (int j = intersections.size() - 1; j >= 0; j--)
            {
                CCVector2d pos = intersections[j];
                Point_2 p(pos.x, pos.y);

                if (is_point_on_segment(p, s))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }
        if (m_bExtendInterect)
        {
            for (int k = intersections.size() - 1; k >= 0; k--)
            {
                CCVector2d pos = intersections[k];
                Point_2 p(pos.x, pos.y);

                if (!is_point_on_segment(p, s))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }
    }
    return lstIntersection;
}

std::vector<CCVector2d> ccLines::intersect(cc2DRect* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;
    Segment_2 s1;
    for (int i = 0; i < this->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[i].x, this->get2dKeyPoints()[i].y);
        CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[i + 1].x, this->get2dKeyPoints()[i + 1].y);
        if (m_bExtendInterect)
        {
            if (i != 0 && i != get2dKeyPoints().size() - 2)
                continue;
        }

        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line1(p1, p2);
        s1 = Segment_2(p1, p2);


        for (int j = 0; j < pItem->getDrawPoints().size()-1; j++)
        {
            q1 = CCVector2d(pItem->getDrawPoints()[j].x, pItem->getDrawPoints()[j].y);
            q2 = CCVector2d(pItem->getDrawPoints()[j + 1].x, pItem->getDrawPoints()[j + 1].y);


            p1 = Point_2(q1.x, q1.y);
            p2 = Point_2(q2.x, q2.y);
            Line_2 line2(p1, p2);
            Segment_2 s2(p1, p2);

            typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Line_2>::type Intersection_result;
            std::vector<Intersection_result> res;

            try
            {
                CGAL::intersection(line1, line2, std::back_inserter(res));
            }
            catch (...)
            {
                qDebug() << "The intersection calculation failed.";
                return lstIntersection;
            }
            if (res.size() <= 0)
            {
                return intersections;
            }
            using boostRetVal = CGAL::Point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >;

            for (auto element : res)
            {
                if (!element)
                {
                    continue;
                }
                auto algPoint = boost::get<Point_2>(&*element);
                if (algPoint)
                {
                    auto point = Point_2(CGAL::to_double(algPoint->x()), CGAL::to_double(algPoint->y()));
                    std::cout << point << std::endl;

                    double x = CGAL::to_double(point.x());
                    double y = CGAL::to_double(point.y());
                    CCVector2d pos(x, y);
                    if (pos.norm() > 0)
                    {
                        intersections.push_back(pos);
                    }
                    std::cout << "x = " << pos.x << "y = " << pos.y << std::endl;
                }
            }
            if (!filter)
            {
                for (int k = intersections.size() - 1; k >= 0; k--)
                {
                    CCVector2d pos = intersections[k];
                    Point_2 p(pos.x, pos.y);

                    if (is_point_on_segment(p, s1) && is_point_on_segment(p, s2))
                    {
                        std::cout << "The point is on the segment." << std::endl;
                    }
                    else
                    {
                        std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                            return abs(temp.norm() - pos.norm()) < EPSINON;
                        });
                        if (it != intersections.end())
                        {
                            intersections.erase(it);
                        }
                    }
                }
            }
            if (m_bExtendInterect)
            {
                for (int k = intersections.size() - 1; k >= 0; k--)
                {
                    CCVector2d pos = intersections[k];
                    Point_2 p(pos.x, pos.y);
                    bool result0 = is_point_on_segment(p, s1);
                    bool result1 = is_point_on_segment(p, s2);
                    if (!result0 && result1)
                    {
                        std::cout << "The point is on the segment." << std::endl;
                    }
                    else
                    {
                        std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                            return abs(temp.norm() - pos.norm()) < EPSINON;
                        });
                        if (it != intersections.end())
                        {
                            intersections.erase(it);
                        }
                    }
                }
            }

            foreach(CCVector2d intersection, intersections)
            {
                lstIntersection.push_back(intersection);
            }
        }


        int index = pItem->getDrawPoints().size() - 1;

        q1 = CCVector2d(pItem->getDrawPoints()[index].x, pItem->getDrawPoints()[index].y);
        q2 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);


        p1 = Point_2(q1.x, q1.y);
        p2 = Point_2(q2.x, q2.y);
        Line_2 line2(p1, p2);
        Segment_2 s2(p1, p2);

        typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Line_2>::type Intersection_result;
        std::vector<Intersection_result> res;

        try
        {
            CGAL::intersection(line1, line2, std::back_inserter(res));
        }
        catch (...)
        {
            qDebug() << "The intersection calculation failed.";
            return lstIntersection;
        }
        if (res.size() <= 0)
        {
            continue;
        }
        using boostRetVal = CGAL::Point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >;

        for (auto element : res)
        {
            if (!element)
            {
                continue;
            }
            auto algPoint = boost::get<Point_2>(&*element);
            if (algPoint)
            {
                auto point = Point_2(CGAL::to_double(algPoint->x()), CGAL::to_double(algPoint->y()));
                std::cout << point << std::endl;

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                CCVector2d pos(x, y);
                if (pos.norm() > 0)
                {
                    intersections.push_back(pos);
                }
                std::cout << "x = " << pos.x << "y = " << pos.y << std::endl;
            }
        }
        if (!filter)
        {
            for (int m = intersections.size() - 1; m >= 0; m--)
            {
                CCVector2d pos = intersections[m];
                Point_2 p(pos.x, pos.y);

                if (is_point_on_segment(p, s1) && is_point_on_segment(p, s2))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }
        
        if (m_bExtendInterect)
        {
            for (int k = intersections.size() - 1; k >= 0; k--)
            {
                CCVector2d pos = intersections[k];
                Point_2 p(pos.x, pos.y);

                bool result0 = is_point_on_segment(p, s1);
                bool result1 = is_point_on_segment(p, s2);
                if (!result0 && result1)
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }
        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }

    }
    return lstIntersection;
}

std::vector<CCVector2d> ccLines::intersect(cc2DRound* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    for (int i = 0; i < this->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        if (m_bExtendInterect)
        {
            if (i != 0 && i != get2dKeyPoints().size() - 2)
                continue;
        }

        if (pItem->getDrawPoints().size() <= 0)
        {
            return intersections;
        }
        CCVector2d q1 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);
        CCVector2d q2 = CCVector2d(pItem->getDrawPoints()[100].x, pItem->getDrawPoints()[100].y);
        CCVector2d q3 = CCVector2d(pItem->getDrawPoints()[150].x, pItem->getDrawPoints()[150].y);


        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Point_2 p3 = Point_2(q3.x, q3.y);

        Circle_2 circle(p1, p2, p3);


        q1 = CCVector2d(this->get2dKeyPoints()[i].x, this->get2dKeyPoints()[i].y);
        q2 = CCVector2d(this->get2dKeyPoints()[i + 1].x, this->get2dKeyPoints()[i + 1].y);


        p1 = Point_2(q1.x, q1.y);
        p2 = Point_2(q2.x, q2.y);
        Line_2 line(p1, p2);


        typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circle_2>::type Intersection_result;
        std::vector<Intersection_result> res;

        auto& output_iterator = std::back_inserter(res);

        try
        {
            CGAL::intersection(line, circle, output_iterator);
        }
        catch (...)
        {
            qDebug() << "The intersection calculation failed.";
            return lstIntersection;
        }

        if (res.size() <= 0)
        {
            continue;
        }
        using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;

        for (const auto& element : res)
        {
            auto algPoint = std::get<0>(boost::get< boostRetVal >(element));

            if (&algPoint)
            {
                auto point = Point_2(CGAL::to_double(algPoint.x()), CGAL::to_double(algPoint.y()));
                std::cout << point << std::endl;

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                CCVector2d pos(x, y);
                intersections.push_back(pos);

                std::cout << "x = " << pos.x << "y = " << pos.y << std::endl;
            }
        }
        Segment_2 s(p1, p2);
        if (!filter)
        {
            for (int j = intersections.size() - 1; j >= 0; j--)
            {
                CCVector2d pos = intersections[j];
                Point_2 p(pos.x, pos.y);

                if (is_point_on_segment(p, s))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }
        if (m_bExtendInterect)
        {
            for (int k = intersections.size() - 1; k >= 0; k--)
            {
                CCVector2d pos = intersections[k];
                Point_2 p(pos.x, pos.y);

                if (!is_point_on_segment(p, s))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
        }

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }
    }

    return lstIntersection;
}

bool endPointDoublication(CCVector2d p1, CCVector2d p2, CCVector2d p3, CCVector2d p4)
{
    if (p1 == p3)
    {
        return false;
    }

    if (p1 == p4)
    {
        return false;
    }

    if (p2 == p3)
    {
        return false;
    }

    if (p2 == p4)
    {
        return false;
    }

    return true;
}

void ccLines::intersectSelf(std::vector<CCVector2d>& lst)
{
    Segment_2 s;
    Segment_2 s1;

    for (int i = 0; i < this->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[i].x, this->get2dKeyPoints()[i].y);
        CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[i + 1].x, this->get2dKeyPoints()[i + 1].y);


        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line1(p1, p2);
        //s = Segment_2(p1, p2);

        for (int j = i + 1; j < this->get2dKeyPoints().size() - 1; j++)
        {
            CCVector2d q3 = CCVector2d(this->get2dKeyPoints()[j].x, this->get2dKeyPoints()[j].y);
            CCVector2d q4 = CCVector2d(this->get2dKeyPoints()[j + 1].x, this->get2dKeyPoints()[j + 1].y);

            if (!endPointDoublication(q1, q2, q3, q4))
            {
                continue;
            }

            p1 = Point_2(q3.x, q3.y);
            p2 = Point_2(q4.x, q4.y);
            Line_2 line2(p1, p2);
            //s1 = Segment_2(p1, p2);


            typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Line_2>::type Intersection_result;
            std::vector<Intersection_result> res;

            try
            {
                CGAL::intersection(line1, line2, std::back_inserter(res));
            }
            catch (...)
            {
                qDebug() << "The intersection calculation failed.";
                return;
            }
            if (res.size() <= 0)
            {
                continue;
            }
            using boostRetVal = CGAL::Point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >;

            for (auto element : res)
            {
                if (!element)
                {
                    continue;
                }
                auto algPoint = boost::get<Point_2>(&*element);
                if (algPoint)
                {
                    auto point = Point_2(CGAL::to_double(algPoint->x()), CGAL::to_double(algPoint->y()));
                    std::cout << point << std::endl;

                    double x = CGAL::to_double(point.x());
                    double y = CGAL::to_double(point.y());
                    CCVector2d pos(x, y);
                    if (pos.norm() > 0)
                    {
                        intersections.push_back(pos);
                    }
                    std::cout << "x = " << pos.x << "y = " << pos.y << std::endl;
                }
            }

            for (int k = intersections.size() - 1; k >= 0; k--)
            {
                CCVector2d pos = intersections[k];

                if (on_segment(CCVector3d(pos.x, pos.y, 0), CCVector3d(q1.x, q1.y, 0), CCVector3d(q2.x, q2.y, 0))
                    && on_segment(CCVector3d(pos.x, pos.y, 0), CCVector3d(q3.x, q3.y, 0), CCVector3d(q4.x, q4.y, 0)))
                {
                    std::cout << "The point is on the segment." << std::endl;
                }
                else
                {
                    std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
                        return abs(temp.norm() - pos.norm()) < EPSINON;
                    });
                    if (it != intersections.end())
                    {
                        intersections.erase(it);
                    }
                }
            }
            foreach(CCVector2d intersection, intersections)
            {
                lst.push_back(intersection);
            }
        }
    }
    return;
}
