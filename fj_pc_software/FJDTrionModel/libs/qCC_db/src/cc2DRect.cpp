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
// ReSharper disable All
#include "ccIncludeGL.h"

#include "cc2DRect.h"
#include "cc2DRound.h"
#include "ccLines.h"
#include "cc2DArcLine.h"
//Local
#include "ccCameraSensor.h"
#include "ccCone.h"
#include "ccPointCloud.h"
#include "ccPoint.h"
#include "geometryUtils.h"


cc2DRect::cc2DRect(QString name)
	: cc2DItemBase(name)
{
	set2DMode(false);
	setForeground(true);
	setVisible(true);
	lockVisibility(true);
	setColor(ccColor::Rgbaf(1, 1, 1, 1));
	showVertices(true);
	setVertexMarkerWidth(3);
	setWidth(0);
	showArrow(false, 0, 0);
	setClosed(false);
	setLocked(true);
}

cc2DRect::cc2DRect(const cc2DRect& poly)
	: cc2DItemBase()
{
	ccPointCloud* vertices = nullptr;
	initWith(vertices, poly);
}

cc2DRect* cc2DRect::clone() const
{
	cc2DRect* clonedPoly = new cc2DRect(*this);
	clonedPoly->setLocked(false); //there's no reason to keep the clone locked

	return clonedPoly;
}

bool cc2DRect::initWith(ccPointCloud*& vertices, const cc2DRect& poly)
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
			ccLog::Warning("[cc2DRect::initWith] Not enough memory to duplicate vertices!");
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
				ccLog::Warning("[cc2DRect::initWith] Not enough memory");
				success = false;
			}
		}
	}

	importParametersFrom(poly);

	return success;
}

void cc2DRect::importParametersFrom(const cc2DRect& poly)
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

void cc2DRect::showArrow(bool state, unsigned vertIndex, PointCoordinateType length)
{

}

ccBBox cc2DRect::getOwnBB(bool withGLFeatures/*=false*/)
{
    ccBBox box;
    if (m_keyPoints3d.size() == 0)
        return box;

    for (int i = 0; i < m_keyPoints3d.size(); i++)
        box.add(CCVector3(m_keyPoints3d[i].x, m_keyPoints3d[i].y, m_keyPoints3d[i].z));
    box.setValidity(!isSelected());
    return box;
}

bool cc2DRect::hasColors() const
{
	return true;
}

void cc2DRect::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//invalidate the bounding-box
	//(and we hope the vertices will be updated as well!)
	invalidateBoundingBox();
}

//unit arrow
static QSharedPointer<ccCone> c_unitArrow(nullptr);

bool cc2DRect::updatePoints(CCVector3d pts)
{
	if (m_Type == RECTACROSS && m_keyPoints3d.size() == 2)
	{
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
		addDisplayPoints(pts);
	}
	else if (m_Type == RECT3 && m_keyPoints3d.size() == 2)
	{
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
		addDisplayPoints(pts);
	}
	else if (m_Type == RECT3 && m_keyPoints3d.size() == 3)
	{
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
		addDisplayPoints(pts);
	}
	return true;
}

bool cc2DRect::removePoints()
{
	if (m_keyPoints3d.size() <= 1)
	{
		return false;
	}

	if (m_Type == RECT3)
	{
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
		return true;
	}
	else if (m_Type == RECTACROSS)
	{
		return false;
	}

	return true;
}

void cc2DRect::get3dPoints()
{
    m_drawPoints3d.clear();
    if(m_Type == RECTACROSS && m_keyPoints3d.size() == 2)
    {
        CCVector3d from = m_keyPoints3d[0];
        CCVector3d to = m_keyPoints3d[1];

        if ((from - to).norm() < 0.01)
            return ;

        ccGLMatrixd mat = m_viewMat;
        CCVector3d xVector(mat(0, 0), mat(0, 1), mat(0, 2));
        CCVector3d yVector(mat(1, 0), mat(1, 1), mat(1, 2));
        xVector.normalize();
        yVector.normalize();
        double x = (xVector).dot(to - from);
        double y = (yVector).dot(to - from);
        m_drawPoints3d.push_back(from);
        m_drawPoints3d.push_back(from + xVector * x);
        m_drawPoints3d.push_back(m_drawPoints3d[1] + yVector * y);
        m_drawPoints3d.push_back(from + yVector * y);
    }
    else if(m_Type == RECT3 && m_keyPoints3d.size() == 3)
    {
        CCVector3d A = m_keyPoints3d[0];
        CCVector3d B = m_keyPoints3d[1];
        CCVector3d C = m_keyPoints3d[2];
        CCVector3d D = A + C - B;
        m_drawPoints3d.push_back(A);
        m_drawPoints3d.push_back(B);
        m_drawPoints3d.push_back(C);
        m_drawPoints3d.push_back(D);
    }
    else if (m_Type == RECT3 && m_keyPoints3d.size() == 2)
    {
        m_drawPoints3d.push_back(m_keyPoints3d[0]);
        m_drawPoints3d.push_back(m_keyPoints3d[1]);
    }
}

void cc2DRect::get2dPoints(const ccGLCameraParameters camera)
{
    m_keyPoints2d.clear();
    get3dPoints();
    for(int i = 0; i < m_keyPoints3d.size(); i++)
    {
        CCVector3d P2D;
        CCVector3d P3D = m_keyPoints3d[i];
        camera.project(P3D, P2D);
        camera.unproject(P2D, P3D);
        m_keyPoints2d.push_back(P2D);
    }
    getDrawPoints2d(camera);
}


void cc2DRect::getDrawPoints2d(const ccGLCameraParameters camera)
{
    m_drawPoints2d.clear();
    for (int i = 0; i < m_drawPoints3d.size(); i++)
    {
        CCVector3d P2D;
        CCVector3d P3D = m_drawPoints3d[i];
        camera.project(P3D, P2D);
        camera.unproject(P2D, P3D);
        m_drawPoints2d.push_back(P2D);
    }
}

bool cc2DRect::addNewPoints(CCVector3d pts)
{
	if (m_first)
	{
		addDisplayPoints(pts);
		addDisplayPoints(pts);
		m_first = false;
	}
	else if (m_Type == RECT3 && m_keyPoints3d.size() == 2)
	{
		addDisplayPoints(pts);
		return true;
	}
	else if (m_Type == RECT3 && m_keyPoints3d.size() == 3)
	{
		m_finish = true;
	}
	else if(m_Type == RECTACROSS && m_keyPoints3d.size() == 2)
	{
		m_finish = true;
	}

    if (m_finish)
        get3dPoints();
	return !m_finish;
}

bool cc2DRect::accpectClickedPoint(void)
{
	if (m_keyPoints3d.size() == 2 && m_Type == RECTACROSS && m_finish)
	{
		return false;
	}
	else if (m_keyPoints3d.size() == 3 && m_Type == RECT3 && m_finish)
	{
		return false;
	}
	return true;
}

void cc2DRect::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (m_keyPoints3d.size() == 0 || m_drawPoints3d.size() == 0||(m_bTrim && m_mouse3dpos.size() == 0))
        return;


    cc2DItemBase::drawMeOnly(context);

	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);
	if (glFunc == nullptr)
		return;

    if (MACRO_Draw3D(context))
        drawIn3dView(context);
    else if (MACRO_Draw2D(context))
        drawIn2dView(context);
}


void cc2DRect::drawSelectivePart(CC_DRAW_CONTEXT& context, std::vector<CCVector3d>& lst, int index1, int index2)
{
    if (m_lstIntersections.size() > 0)
    {
        std::vector<CCVector3d> intersectionInSegment;

        double z = lst[0].z;
        CCVector3d mousePos = m_mouse2dpos[0];
        for (int i = 0; i < m_lstIntersections.size(); i++)
        {
            CCVector3d intersection = CCVector3d(m_lstIntersections[i].x, m_lstIntersections[i].y, z);
            bool result = on_segment(CCVector3d(intersection.x, intersection.y, z),
                lst[index1], lst[index2]);
            if (result)
            {
                intersectionInSegment.push_back(intersection);
            }
        }
        if (intersectionInSegment.size() == 0)
        {
            std::vector<CCVector3d> selectPart;
            selectPart.push_back(lst[index1]);
            selectPart.push_back(lst[index2]);
            drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));
        }
        else if (intersectionInSegment.size() == 1)
        {
            if (on_segment(mousePos, lst[index1], intersectionInSegment[0]))
            {
                std::vector<CCVector3d> selectPart;
                selectPart.push_back(lst[index1]);
                selectPart.push_back(intersectionInSegment[0]);
                drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));

                //std::vector<CCVector3d> UnSelectPart;
                m_pt2dUnselect3.push_back(lst[index2]);
                m_pt2dUnselect3.push_back(intersectionInSegment[0]);
                drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));
            }
            else
            {
                std::vector<CCVector3d> selectPart;
                selectPart.push_back(lst[index2]);
                selectPart.push_back(intersectionInSegment[0]);
                drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));

                //std::vector<CCVector3d> UnSelectPart;
                m_pt2dUnselect3.push_back(lst[index1]);
                m_pt2dUnselect3.push_back(intersectionInSegment[0]);
                drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));
            }
        }
        else
        {
            intersectionInSegment.push_back(lst[index1]);
            intersectionInSegment.push_back(lst[index2]);
            sortVector3d(intersectionInSegment, z);
            bool find = false;
            for (int i = 0; i < intersectionInSegment.size() - 1; i++)
            {
                CCVector3d p1 = intersectionInSegment[i];
                CCVector3d p2 = intersectionInSegment[i + 1];

                if (on_segment(mousePos, p1, p2))
                {
                    if (i == 0)
                    {
                        std::vector<CCVector3d> selectPart;
                        selectPart.push_back(p1);
                        selectPart.push_back(p2);
                        drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));

                        //std::vector<CCVector3d> UnSelectPart;
                        m_pt2dUnselect3.push_back(p2);
                        m_pt2dUnselect3.push_back(intersectionInSegment[intersectionInSegment.size() - 1]);
                        drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));
                    }
                    else if (i == intersectionInSegment.size() - 2)
                    {
                        std::vector<CCVector3d> selectPart;
                        selectPart.push_back(p1);
                        selectPart.push_back(p2);
                        drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));

                        //std::vector<CCVector3d> UnSelectPart;
                        m_pt2dUnselect3.push_back(intersectionInSegment[0]);
                        m_pt2dUnselect3.push_back(p1);
                        drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));
                    }
                    else
                    {
                        std::vector<CCVector3d> selectPart;
                        selectPart.push_back(p1);
                        selectPart.push_back(p2);
                        drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));

                        //std::vector<CCVector3d> UnSelectPart;
                        m_pt2dUnselect3.push_back(intersectionInSegment[0]);
                        m_pt2dUnselect3.push_back(p1);
                        drawLines2D(context, m_pt2dUnselect3, ccColor::Rgbaf(1, 0, 0, 1));

                        //std::vector<CCVector3d> UnSelectPart1;
                        m_pt2dUnselect4.push_back(p2);
                        m_pt2dUnselect4.push_back(intersectionInSegment[intersectionInSegment.size() - 1]);
                        drawLines2D(context, m_pt2dUnselect4, ccColor::Rgbaf(1, 0, 0, 1));
                    }
                    find = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if (!find)
            {
                m_bAbnormal = true;
                std::vector<CCVector3d> selectPart;
                selectPart.push_back(lst[index1]);
                selectPart.push_back(lst[index2]);
                drawLines2D(context, selectPart, ccColor::Rgbaf(1, 0, 0, 1));
            }
        }
    }
    else
    {
        std::vector<CCVector3d> selectPart;
        selectPart.push_back(lst[index1]);
        selectPart.push_back(lst[index2]);
        drawLines2D(context, selectPart, ccColor::Rgbaf(1, 1, 1, 1));
    }
}


void cc2DRect::drawRectTrim(CC_DRAW_CONTEXT& context)
{
    m_pt2dUnselect4.clear();
    m_pt2dUnselect3.clear();
    m_pt2dUnselect2.clear();
    m_pt2dUnselect.clear();

    std::vector<CCVector3d> copyKeyPoints = m_drawPoints2d;
    copyKeyPoints.push_back(copyKeyPoints[0]);

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

    if (originIndex1 > -1)
    {
        //std::vector<CCVector3d> bodyPart;

        if (originIndex1 == 0)
        {
            for (size_t i = originIndex2; i < copyKeyPoints.size(); i++)
            {
                m_pt2dUnselect.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
            drawSelectivePart(context, copyKeyPoints, originIndex1, originIndex2);
        }
        else if (originIndex1 == 3)
        {
            for (size_t i = 0; i <= originIndex1; i++)
            {
                m_pt2dUnselect.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));

            drawSelectivePart(context, copyKeyPoints, originIndex1, originIndex2);
        }
        else if (originIndex1 == 1)
        {
            m_pt2dUnselect.push_back(copyKeyPoints[0]);
            m_pt2dUnselect.push_back(copyKeyPoints[originIndex1]);
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
            //bodyPart.clear();

            for (size_t i = originIndex2; i < copyKeyPoints.size(); i++)
            {
                m_pt2dUnselect2.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));

            drawSelectivePart(context, copyKeyPoints, originIndex1, originIndex2);
        }
        else if (originIndex1 == 2)
        {
            for (size_t i = 0; i <= originIndex1; i++)
            {
                m_pt2dUnselect.push_back(copyKeyPoints[i]);
            }
            drawLines2D(context, m_pt2dUnselect, ccColor::Rgbaf(1, 0, 0, 1));
            //bodyPart.clear();

            m_pt2dUnselect2.push_back(copyKeyPoints[originIndex2]);
            m_pt2dUnselect2.push_back(copyKeyPoints[copyKeyPoints.size() - 1]);
            drawLines2D(context, m_pt2dUnselect2, ccColor::Rgbaf(1, 0, 0, 1));

            drawSelectivePart(context, copyKeyPoints, originIndex1, originIndex2);
        }
    }
    else
    {
        m_bAbnormal = true;
        m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
        drawRectNormal(context);
    }
}


void cc2DRect::drawIn2dView(CC_DRAW_CONTEXT & context)
{
    if (m_keyPoints2d.size() == 0 || (m_bTrim && m_mouse2dpos.size() == 0))
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

    if(!m_bTrim)
    {
        drawRectNormal(context);
    }
    else if (m_bTrim && m_mouse2dpos.size() > 0 )
    {
        drawRectTrim(context);
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

void cc2DRect::drawRectNormal(CC_DRAW_CONTEXT& context)
{
    if (m_Type == RECT3 && m_keyPoints3d.size() <= 3)
    {
        std::vector<CCVector3d> pts = m_drawPoints2d;
        if (m_drawPoints2d.size() == 4)
        {
            pts.push_back(m_drawPoints2d[3]);
            pts.push_back(m_drawPoints2d[0]);
        }
        drawLines2D(context, pts, m_rgbColor);
    }
    else
    {
        std::vector<CCVector3d> pts = m_drawPoints2d;
        if (m_drawPoints2d.size() > 0)
        {
            pts.push_back(m_drawPoints2d[0]);
        }
        drawLines2D(context, pts, m_rgbColor);
    }
}

void cc2DRect::drawIn3dView(CC_DRAW_CONTEXT & context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        if (MACRO_DrawFastNamesOnly(context))
            return;
        glFunc->glPushName(getUniqueIDForDisplay());
    }

    ccGLCameraParameters camera;
    glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
    glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
    glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

    if(!pushName)
        get2dPoints(camera);

    if (context.display->windowName().compare("3D View") != 0)
        return;

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

    if (m_drawPoints3d.size() > 0)
        m_drawPoints3d.push_back(m_drawPoints3d[0]);

    glFunc->glBegin(GL_LINE_STRIP);
    for (unsigned i = 0; i < m_drawPoints3d.size(); ++i)
        glFunc->glVertex3f(m_drawPoints3d[i].x, m_drawPoints3d[i].y, m_drawPoints3d[i].z);
    glFunc->glEnd();

    if (m_width != 0)
    {
        glFunc->glDisable(GL_BLEND);
        glFunc->glDisable(GL_LINE_SMOOTH);
        glFunc->glPopAttrib();
    }
    if (pushName)
        glFunc->glPopName();
}

bool cc2DRect::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[cc2DRect::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
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

bool cc2DRect::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
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

bool cc2DRect::split(	PointCoordinateType maxEdgeLength,
						std::vector<cc2DRect*>& parts)
{
	parts.clear();

	//not enough vertices?
	unsigned vertCount = size();
	if (vertCount <= 2)
	{
		parts.push_back(new cc2DRect(*this));
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
					parts.push_back(new cc2DRect(*this));
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
				parts.push_back(new cc2DRect(*this));
				return true;
			}
		}

		if (partSize > 1) //otherwise we skip that point
		{
			//create the corresponding part
			CCCoreLib::ReferenceCloud ref(m_theAssociatedCloud);
			if (!ref.reserve(partSize))
			{
				ccLog::Error("[cc2DRect::split] Not enough memory!");
				return false;
			}

			for (unsigned i=startIndex; i<=stopIndex; ++i)
			{
				ref.addPointIndex(i % vertCount);
			}

			//ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
			//ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
			//cc2DRect* part = new cc2DRect(subset);
			//part->initWith(subset, *this);
			//part->setClosed(false); //by definition!
			//parts.push_back(part);
		}

		//forward
		startIndex = (stopIndex % vertCount) + 1;
	}

	return true;
}

PointCoordinateType cc2DRect::computeLength() const
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

unsigned cc2DRect::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
}

unsigned cc2DRect::segmentCount() const
{
	unsigned count = size();
	if (count && !isClosed())
	{
		--count;
	}
	return count;
}

void cc2DRect::setGlobalShift(const CCVector3d& shift)
{
	ccShiftedObject::setGlobalShift(shift);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global shift info to the vertices
		pc->setGlobalShift(shift);
	}
}

void cc2DRect::setGlobalScale(double scale)
{
	ccShiftedObject::setGlobalScale(scale);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global scale info to the vertices
		pc->setGlobalScale(scale);
	}
}

const CCVector3d& cc2DRect::getGlobalShift() const
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

double cc2DRect::getGlobalScale() const
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

ccPointCloud* cc2DRect::samplePoints(	bool densityBased,
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
		ccLog::Warning("[cc2DRect::samplePoints] Not enough memory");
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

cc2DRect* cc2DRect::smoothChaikin(PointCoordinateType ratio, unsigned iterationCount) const
{
	if (iterationCount == 0)
	{
		assert(false);
		ccLog::Warning("[cc2DRect::smoothChaikin] Invalid input (iteration count)");
		return nullptr;
	}

	if (ratio < 0.05f || ratio > 0.45f)
	{
		assert(false);
		ccLog::Warning("[cc2DRect::smoothChaikin] invalid ratio");
		return nullptr;
	}

	if (size() < 3)
	{
		ccLog::Warning("[cc2DRect::smoothChaikin] not enough segments");
		return nullptr;
	}

	const CCCoreLib::GenericIndexedCloudPersist* currentIterationVertices = this; //a polyline is actually a ReferenceCloud!
	cc2DRect* smoothPoly = nullptr;

	bool openPoly = !isClosed();

	for (unsigned it = 0; it < iterationCount; ++it)
	{
		//reserve memory for the new vertices
		unsigned vertCount = currentIterationVertices->size();
		unsigned segmentCount = (openPoly ? vertCount - 1 : vertCount);

		ccPointCloud* newStateVertices = new ccPointCloud("vertices");
		if (!newStateVertices->reserve(segmentCount * 2))
		{
			ccLog::Warning("[cc2DRect::smoothChaikin] not enough memory");
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
			//smoothPoly = new cc2DRect(newStateVertices);
			//smoothPoly->addChild(newStateVertices);
			//newStateVertices->setEnabled(false);
			//if (!smoothPoly->reserve(newStateVertices->size()))
			//{
			//	ccLog::Warning("[cc2DRect::smoothChaikin] not enough memory");
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

bool cc2DRect::IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, cc2DRect** polyline/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a polyline
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DRect*>(parent)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<cc2DRect*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DRect*>(child)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<cc2DRect*>(child);
			}
			return true;
		}
	}

	return false;
}

bool cc2DRect::createNewPolylinesFromSelection(std::vector<cc2DRect*>& output)
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
		ccLog::Warning("[cc2DRect::createNewPolylinesFromSelection] Unsupported vertex cloud");
		return false;
	}
	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = verticesCloud->getTheVisibilityArray();
	if (verticesVisibility.size() < vertCount)
	{
		// no visibility table instantiated
		ccLog::Warning("[cc2DRect::createNewPolylinesFromSelection] No visibility table instantiated");
		return false;
	}

	bool success = true;
	{
		cc2DRect* chunkPoly = nullptr;
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
					//chunkCloud = new ccPointCloud("vertices");
					//chunkCloud->setEnabled(false);
					//chunkPoly = new cc2DRect(chunkCloud);
					//chunkPoly->addChild(chunkCloud);
					//if (!chunkPoly->reserve(DefaultPolySizeIncrement) || !chunkCloud->reserve(DefaultPolySizeIncrement))
					//{
					//	delete chunkCloud;
					//	success = false;
					//	break;
					//}
					//chunkPoly->addPointIndex(0);
					//chunkCloud->addPoint(*P0);
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
		ccLog::Warning("[cc2DRect::createNewPolylinesFromSelection] Not enough memory");
		// delete the already created polylines
		for (cc2DRect* poly : output)
		{
			delete poly;
		}
		output.clear();
	}

	return success;
}

std::vector<CCVector2d> cc2DRect::intersect(ccLines* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;
    Segment_2 s1;
    for (int i = 0; i < pItem->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(pItem->get2dKeyPoints()[i].x, pItem->get2dKeyPoints()[i].y);
        CCVector2d q2 = CCVector2d(pItem->get2dKeyPoints()[i + 1].x, pItem->get2dKeyPoints()[i + 1].y);
        if (q1 == q2)
            continue;

        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line1(p1, p2);
        s1 = Segment_2(p1, p2);

        for (int j = 0; j < this->getDrawPoints().size()-1; j++)
        {
            q1 = CCVector2d(this->getDrawPoints()[j].x, this->getDrawPoints()[j].y);
            q2 = CCVector2d(this->getDrawPoints()[j + 1].x, this->getDrawPoints()[j + 1].y);


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

            foreach(CCVector2d intersection, intersections)
            {
                lstIntersection.push_back(intersection);
            }
        }



        int index = this->getDrawPoints().size() - 1;

        q1 = CCVector2d(this->getDrawPoints()[index].x, this->getDrawPoints()[index].y);
        q2 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);


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

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }

    }
    return lstIntersection;
}

std::vector<CCVector2d> cc2DRect::intersect(cc2DArcLine* pItem, bool filter)
{
    typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circular_arc_2>::type Intersection_result;
    using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;


    std::vector<CCVector2d> lstIntersection;

    CCVector2d q1 = CCVector2d(pItem->get2dKeyPoints()[0].x, pItem->get2dKeyPoints()[0].y);
    CCVector2d q2 = CCVector2d(pItem->get2dKeyPoints()[1].x, pItem->get2dKeyPoints()[1].y);
    CCVector2d q3 = CCVector2d(pItem->get2dKeyPoints()[2].x, pItem->get2dKeyPoints()[2].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);

    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

    Circular_arc_2 arc;
    try
    {
        arc = Circular_arc_2(p1, p2, p3);
    }
    catch (...)
    {
        return lstIntersection;
        qDebug() << "COUNTERCLOCKWISE faided!" << "\n";
    }
    for (int i = 0; i < this->getDrawPoints().size()-1; i++)
    {
        std::vector<CCVector2d> intersections;

        CCVector2d q1 = CCVector2d(this->getDrawPoints()[i].x, this->getDrawPoints()[i].y);
        CCVector2d q2 = CCVector2d(this->getDrawPoints()[i+1].x, this->getDrawPoints()[i+1].y);


        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line(p1, p2);
        Segment_2 s(p1, p2);

        std::vector<Intersection_result> res;

        try
        {
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
                std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) 
                {
                    return abs(temp.norm() - pos.norm()) < EPSINON;
                });
                if(it != intersections.end())
                {
                    intersections.erase(it);
                }
            }
        }

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }
    }
    
    std::vector<CCVector2d> intersections;
    int index = this->getDrawPoints().size() - 1;
    q1 = CCVector2d(this->getDrawPoints()[index].x, this->getDrawPoints()[index].y);
    q2 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);


    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    Line_2 line2(p1, p2);
    Segment_2 s(p1, p2);

    std::vector<Intersection_result> res;
    try
    {
        CGAL::intersection(line2, arc, std::back_inserter(res));
    }
    catch (...)
    {
        qDebug() << "The intersection calculation failed.";
        return lstIntersection;
    }

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

    for (int i = intersections.size() - 1; i >= 0; i--)
    {
        CCVector2d pos = intersections[i];
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

    foreach(CCVector2d intersection, intersections)
    {
        lstIntersection.push_back(intersection);
    }


    return lstIntersection;
}

std::vector<CCVector2d> cc2DRect::intersect(cc2DRect* pItem, bool filter)
{
    typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Line_2>::type Intersection_result;

    std::vector<CCVector2d> lstIntersection;
    std::vector<CCVector3d> lstThis = this->getDrawPoints();
    lstThis.push_back(lstThis[0]);

    std::vector<CCVector3d> lstItem = pItem->getDrawPoints();
    lstItem.push_back(lstItem[0]);

    Segment_2 s1;
    for (int i = 0; i < lstThis.size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        CCVector2d q1 = CCVector2d(lstThis[i].x, lstThis[i].y);
        CCVector2d q2 = CCVector2d(lstThis[i + 1].x, lstThis[i + 1].y);


        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Line_2 line1(p1, p2);
        s1 = Segment_2(p1, p2);

        for (int j = 0; j < lstItem.size() - 1; j++)
        {
            q1 = CCVector2d(lstItem[j].x, lstItem[j].y);
            q2 = CCVector2d(lstItem[j + 1].x, lstItem[j + 1].y);


            p1 = Point_2(q1.x, q1.y);
            p2 = Point_2(q2.x, q2.y);
            Line_2 line2(p1, p2);
            Segment_2 s2(p1, p2);

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

            try
            {
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
            }
            catch (...)
            {
                qDebug() << "The intersection calculation failed.";
                return lstIntersection;
            }


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

            foreach(CCVector2d intersection, intersections)
            {
                lstIntersection.push_back(intersection);
            }
        }
    }

    return lstIntersection;
}

std::vector<CCVector2d> cc2DRect::intersect(cc2DRound* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    if (pItem->getDrawPoints().size() <= 0)
    {
        return lstIntersection;
    }
    CCVector2d q1 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(pItem->getDrawPoints()[100].x, pItem->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(pItem->getDrawPoints()[150].x, pItem->getDrawPoints()[150].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circle_2 circle(p1, p2, p3);

    for (int i = 0; i < this->getDrawPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        q1 = CCVector2d(this->getDrawPoints()[i].x, this->getDrawPoints()[i].y);
        q2 = CCVector2d(this->getDrawPoints()[i + 1].x, this->getDrawPoints()[i + 1].y);


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

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }
    }

        std::vector<CCVector2d> intersections;

        int index = this->getDrawPoints().size() - 1;

        q1 = CCVector2d(this->getDrawPoints()[index].x, this->getDrawPoints()[index].y);
        q2 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);


        p1 = Point_2(q1.x, q1.y);
        p2 = Point_2(q2.x, q2.y);
        Line_2 line2(p1, p2);
        Segment_2 s(p1, p2);

        typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circle_2>::type Intersection_result;
        std::vector<Intersection_result> res;

        try
        {
            CGAL::intersection(line2, circle, std::back_inserter(res));
        }
        catch (...)
        {
            qDebug() << "The intersection calculation failed.";
            return lstIntersection;
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

        for (int i = intersections.size() - 1; i >= 0; i--)
        {
            CCVector2d pos = intersections[i];
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

        foreach(CCVector2d intersection, intersections)
        {
            lstIntersection.push_back(intersection);
        }

        return lstIntersection;
}