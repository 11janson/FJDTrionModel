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

#include "cc2DItemBase.h"

//Local
#include "ccCone.h"
#include "ccPointCloud.h"
#include "geometryUtils.h"
#include "ccLines.h"
//qt
#include <QVector>
#include <QVector2d>


cc2DItemBase::cc2DItemBase(QString name)
    :Polyline(nullptr)
    , ccShiftedObject(name)
    , m_first(true)
    , m_Type(NONE)
    , m_finish(false)
    , m_bTrim(false)
    , m_bParticipatingIntersection(true)
	, m_pointColor(0, 1, 0, 1)
	, m_bHoverSelect(false)
    , m_bExtendInterect(false)
{
    m_gripPointIndex = -1;
    m_keyPoints2d.clear();
    m_keyPoints3d.clear();
    m_ptOsnapPoints.clear();
}

bool cc2DItemBase::updatePoints(CCVector3d pts)
{
    get3dPoints();
	return false;
}

bool cc2DItemBase::addNewPoints(CCVector3d pts)
{
    get3dPoints();
	return false;
}

void cc2DItemBase::copyTransformPos(CCVector3d pos)
{
    for (int i = 0; i< m_keyPoints3d.size(); i++)
    {
        CCVector3d& p = m_keyPoints3d[i];
        p += pos;
    }
    get3dPoints();
    //redrawDisplay();
}

bool cc2DItemBase::segmentIntersect(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& p3)
{
    if (isnan(p0.x) || isnan(p1.x) || isnan(p2.x) || isnan(p3.x))
    {
        return false;
    }
    Segment_2 s1(Point_2(p0.x, p0.y), Point_2(p1.x, p1.y));
    Segment_2 s2(Point_2(p2.x, p2.y), Point_2(p3.x, p3.y));

    bool result = CGAL::do_intersect(s1, s2);
    return result;
}

bool cc2DItemBase::lineIntersect(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& p3)
{
    if (isnan(p0.x) || isnan(p1.x) || isnan(p2.x) || isnan(p3.x))
    {
        return false;
    }
    Line_2 s1(Point_2(p0.x, p0.y), Point_2(p1.x, p1.y));
    Line_2 s2(Point_2(p2.x, p2.y), Point_2(p3.x, p3.y));

    bool result = CGAL::do_intersect(s1, s2);
    return result;
}

CCVector2d cc2DItemBase::getPerpendicularPoint(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2)
{
    if (isnan(p0.x)|| isnan(p0.y))
    {
        return CCVector2d();
    }
    CCVector2d result;
    float k = 0.0;
    if (p1.x == p2.x)
    {
        result.x = result.x;
        result.y = p0.y;
        return result;
    }
    k = (p2.y - p1.y) * 1.0 / (p2.x - p1.x);
    float A = k;
    float B = -1.0;
    float C = p1.y - k * p1.x;

    result.x = (B * B * p0.x - A * B * p0.y - A * C) / (A * A + B * B);
    result.y = (A * A * p0.y - A * B * p0.x - B * C) / (A * A + B * B);
    return result;
}

void cc2DItemBase::setHoverSelect(bool state)
{
    if (isSelected())
        return;
	m_bHoverSelect = state;
    m_rgbColor = state == true ? ccColor::Rgbaf(1, 1, 1, 1) : ccColor::Rgbaf(1, 0, 0, 1);
    redrawDisplay();
}

void cc2DItemBase::setSelected(bool state)
{
    m_selected = state;
    m_rgbColor = m_selected == true ? ccColor::Rgbaf(0, 1, 0, 1) : ccColor::Rgbaf(1, 0, 0, 1);
    redrawDisplay();
}

int cc2DItemBase::clearDrawGraphicsPoints(void)
{
    m_keyPoints3d.clear();
    return 0;
}

std::vector<CCVector3d> cc2DItemBase::getDrawGraphicsPoints(void)
{
    std::vector<CCVector3d> list;
    for (int i = 0; i < m_keyPoints3d.size(); i++){
        list.push_back(m_keyPoints3d[i]);
    }
    return list;
}

int cc2DItemBase::setUpdateDrawGraphicsPoints(std::vector<CCVector3d> list)
{

    for (int i = 0; i < list.size(); i++)
    {
        if (i < m_keyPoints3d.size())
            m_keyPoints3d[i] = list[i];
        else
            addDisplayPoints(list[i]);
    }
    get3dPoints();
    return 0;
}

bool cc2DItemBase::accpectClickedPoint(void)
{
    return true;
}

void cc2DItemBase::addDisplayPoints(CCVector3d pt)
{
	m_keyPoints3d.push_back(pt);
    get3dPoints();
}

bool cc2DItemBase::removePoints()
{
    return true;
}

void cc2DItemBase::getdrawPoints3d()
{
}

unsigned cc2DItemBase::getUniqueIDForDisplay() const
{
    return getUniqueID();
}

cc2DItemBase * cc2DItemBase::clone()
{
    cc2DItemBase* result = new cc2DItemBase();
    result->setName(getName());
    result->setMetaData(this->metaData());
    result->m_keyPoints3d = m_keyPoints3d;
    result->m_keyPoints2d = m_keyPoints2d;

    result->m_Type = m_Type;
    result->get3dPoints();
    return result;
}

void cc2DItemBase::setPointsVisble(bool visible)
{

}

void cc2DItemBase::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (m_keyPoints3d.size() == 0)
        return;

    if (!MACRO_Foreground(context)) 
        return;

    if (MACRO_Draw3D(context)) 
        drawMeOnly3D(context);
    else if (MACRO_Draw2D(context)) 
        drawMeOnly2D(context);
}

std::vector<CCVector3d> cc2DItemBase::projectTo2d(const std::vector<CCVector3d>& points3d, const ccGLCameraParameters camera)
{
    std::vector<CCVector3d> pts2d;
    for (int i = 0; i < points3d.size(); i++)
    {
        CCVector3d pt;
        camera.project(points3d[i], pt);
        pts2d.push_back(pt);
    }
    return pts2d;
}

std::vector<CCVector3d> cc2DItemBase::unprojectTo3d(const std::vector<CCVector3d>& points2d, const ccGLCameraParameters camera)
{
    std::vector<CCVector3d> pts3d;
    for (int i = 0; i < points2d.size(); i++)
    {
        CCVector3d pt;
        camera.unproject(points2d[i], pt);
        pts3d.push_back(pt);
    }
    return pts3d;
}


void cc2DItemBase::get3dPoints()
{
}

void cc2DItemBase::get2dPoints()
{
}

bool cc2DItemBase::getOsnapPonits(const CCVector3d & pt, OsnapMode mode, CCVector3d& result, double scale, CCVector2d& screen2d)
{
    m_ptOsnapPoints.clear();
	bool isOsnap_keyPoint = false;

	if (mode == OsnapMode::kKeyPoint)
	{
		isOsnap_keyPoint = geometryUtils::getOsnap_KeyPoint(this, pt, result, mode, screen2d);
		if (isOsnap_keyPoint && m_keyPoints3d.size() >=3 
			&& CCVector3d::vdistance(result.u, m_keyPoints3d[m_keyPoints3d.size() - 2].u) > 10e-4)
		{
			m_OsnapFlag = 1;
		}
		else
		{
			isOsnap_keyPoint = false;
			m_OsnapFlag = -2;
		}
	}
	if (mode == OsnapMode::kCenterpoint)
	{
		isOsnap_keyPoint = geometryUtils::getOsnap_CenterPoint(this, pt, result, scale, mode, screen2d);
		if (isOsnap_keyPoint)
			m_OsnapFlag = 1;
		else
			m_OsnapFlag = -2;
	}
	else if (mode == OsnapMode::kAnyPoint)
	{
		m_OsnapFlag = geometryUtils::getOsnapPoint(this, pt, result, scale, screen2d);
	}
    if (m_OsnapFlag >-1 || isOsnap_keyPoint)
    {
        m_ptOsnapPoints.push_back(result);
        return true;
    }
    m_OsnapFlag = -1;
    m_ptOsnapPoints.clear();
    return false;
}

void cc2DItemBase::setMoveGripPoint(CCVector3d pt)
{
    if(m_gripPointIndex == -1 || m_gripPointIndex > m_keyPoints3d.size() - 1)
        return;

    m_keyPoints3d[m_gripPointIndex] = pt;
    m_pointsTmp.clear();
    m_pointsTmp.push_back(m_ptOrginal);
    m_pointsTmp.push_back(pt);
    setSelected(false);
    get3dPoints();
}

void cc2DItemBase::bakOrginal(bool needBak)
{
    if (!needBak)
    {
        m_keyPoints3d_orginal.clear();
        m_pointsTmp.clear();
        m_ptOsnapPoints.clear();
    }
    else
    {
        m_keyPoints3d_orginal = m_keyPoints3d;
    }
    setSelected(!needBak);
}

void cc2DItemBase::setTrimMouse3DPos(CCVector3d mouse3Dpos)
{ 
    m_mouse3dpos.clear();
    m_mouse3dpos.push_back(mouse3Dpos);
}

void cc2DItemBase::sortVector3d(std::vector<CCVector3d>& lst, double z, bool order/* = true*/)
{
    if (lst.size() <= 0)
    {
        return;
    }
    QVector<QVector2D> lstQvec2d;
    foreach (CCVector3d pos, lst)
    {
        lstQvec2d.push_back(QVector2D(pos.x, pos.y));
    }
    qSort(lstQvec2d.begin(), lstQvec2d.end(), [&](const QVector2D &v1, const QVector2D &v2){
        //升序
        if (order)
        {
            if (abs(v1.x() - v2.x()) > EPSINON)
            {
                return v1.x() < v2.x();
            }
            else
            {
                return v1.y() < v2.y();
            }
        }
        //降序
        else
        {
            if (abs(v1.x() - v2.x()) > EPSINON)
            {
                return v1.x() > v2.x();
            }
            else
            {
                return v1.y() > v2.y();
            }
        }

    });
    lst.clear();
    foreach(QVector2D pos2d, lstQvec2d)
    {
        lst.push_back(CCVector3d(pos2d.x(), pos2d.y(), z));
    }
}

bool cc2DItemBase::on_parallel(cc2DItemBase * item0, cc2DItemBase* item1)
{
    ccLines* pLine0 = dynamic_cast<ccLines*>(item0);
    ccLines* pLine1 = dynamic_cast<ccLines*>(item1);

    if (!pLine0 || !pLine1)
        return false;

    CCVector3d AB = pLine0->get2dKeyPoints()[1] - pLine0->get2dKeyPoints()[0];
    CCVector3d CD = pLine1->get2dKeyPoints()[1] - pLine1->get2dKeyPoints()[0];

    CCVector3d cross;
    CCVector3d::vcross(AB.u, CD.u, cross.u);
    double normal = cross.norm();
    bool result = normal < 10e-3 ? true : false;

    return result;
}

bool cc2DItemBase::on_segment(const CCVector3d& mousePos, const CCVector3d& interPos1, const CCVector3d& interPos2)
{
    // 计算点到线段两个端点的距离
    auto d1 = CCVector3d::vdistance(mousePos.u, interPos1.u);
    auto d2 = CCVector3d::vdistance(mousePos.u, interPos2.u);

    double sum = d1 + d2;

    // 计算线段长度
    auto length = CCVector3d::vdistance(interPos1.u, interPos2.u);

    double result = sum - length;

    double theta = atan2(interPos1.x - mousePos.x, interPos1.y - mousePos.y)- atan2(interPos2.x - mousePos.x, interPos2.y - mousePos.y);
    if (theta > M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;

    theta = abs(theta*180.0 / M_PI);
    if (theta < 177 && theta > 183)
    {
        return false;
    }

    return result <= 10e-2 && theta>=177 && theta <= 183;
}

bool cc2DItemBase::is_point_on_segment(const Point_2& p, const Segment_2& s)
{
    // 计算点到线段两个端点的距离的平方
    auto d1 = CGAL::squared_distance(p, s.source());
    auto d2 = CGAL::squared_distance(p, s.target());

    double m1 = CGAL::to_double(d1);
    double m2 = CGAL::to_double(d2);

    double sum = sqrt(m1) + sqrt(m2);

    // 计算线段长度的平方
    auto length_squared = CGAL::squared_distance(s.source(), s.target());

    double length_double = CGAL::to_double(length_squared);

    double result = (sum - sqrt(length_double));
    return result < 10e-1;
}

void cc2DItemBase::drawLines2D(CC_DRAW_CONTEXT & context, const std::vector<CCVector3d>& pts, ccColor::Rgbaf color, bool isDashed)
{
    QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (!glFunc)
        return;

    glFunc->glColor4f(color.r, color.g, color.b, color.a);

    if (pts.size() != 0)
    {
        if (isDashed)
        {
            glFunc->glEnable(GL_LINE_STIPPLE);
            glFunc->glLineStipple(1, 0x0F0F);
        }
        glFunc->glBegin(GL_LINE_STRIP);
        for (unsigned i = 0; i < pts.size(); ++i)
            glFunc->glVertex2f(pts[i].x, pts[i].y);
        if (m_isClosed && pts.size() > 0)
            glFunc->glVertex2f(pts[0].x, pts[0].y);
        glFunc->glEnd();

        if (isDashed)
            glFunc->glDisable(GL_LINE_STIPPLE);
    }
}



void cc2DItemBase::drawMeOnly3D(CC_DRAW_CONTEXT & context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    bool pushName = MACRO_DrawEntityNames(context);

    //ccGLCameraParameters camera;
    glFunc->glGetIntegerv(GL_VIEWPORT, m_3dCamera.viewport);
    glFunc->glGetDoublev(GL_PROJECTION_MATRIX, m_3dCamera.projectionMat.data());
    glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, m_3dCamera.modelViewMat.data());

    if (!pushName)
    {
        m_keyPoints2d = projectTo2d(m_keyPoints3d, m_3dCamera);
        //m_keyPoints2d_orginal = projectTo2d(m_keyPoints3d_orginal, camera);
        m_pointsTmp2d = projectTo2d(m_pointsTmp, m_3dCamera);
        m_ptOsnampsTemp2d = projectTo2d(m_ptOsnapPoints, m_3dCamera);
        m_mouse2dpos = projectTo2d(m_mouse3dpos, m_3dCamera);
    }

    if (pushName)
    {
        if (MACRO_DrawFastNamesOnly(context))
            return;
        glFunc->glPushName(getUniqueIDForDisplay());
    }

    if (context.display->windowName().compare("3D View") != 0)
    {
        if (pushName)
            glFunc->glPopName();
        return;
    }

    glFunc->glMatrixMode(GL_MODELVIEW);
    GLdouble pSize;
    glFunc->glGetDoublev(GL_POINT_SIZE, &pSize);
    glFunc->glPointSize(10.0f);
    glFunc->glColor4f(m_pointColor.r, m_pointColor.g, m_pointColor.b, m_pointColor.a);                 // bule
    glFunc->glBegin(GL_POINTS);
    if (isSelected() && !m_bTrim)
    {
        for (int i = 0; i < m_keyPoints3d.size(); i++)
            glFunc->glVertex3d(m_keyPoints3d[i].x, m_keyPoints3d[i].y, m_keyPoints3d[i].z);
    }
    glFunc->glEnd();
    glFunc->glDisable(GL_BLEND);
    glFunc->glDisable(GL_LINE_SMOOTH);
    glFunc->glPopAttrib();

    if (pushName)
        glFunc->glPopName();
    glFunc->glPointSize(pSize);

}

void drawMidPointOsnapFlag(QOpenGLFunctions_2_1 *glFunc, CCVector3d pt)
{
    glFunc->glVertex2d(pt.x , pt.y + 12);
    glFunc->glVertex2d(pt.x - 10, pt.y - 8);
    glFunc->glVertex2d(pt.x + 10, pt.y - 8);
    glFunc->glVertex2d(pt.x, pt.y + 12);
}

void drawKeyPointOsnapFlag(QOpenGLFunctions_2_1 *glFunc, CCVector3d pt)
{
    glFunc->glVertex2d(pt.x - 8, pt.y - 8);
    glFunc->glVertex2d(pt.x + 8, pt.y - 8);
    glFunc->glVertex2d(pt.x + 8, pt.y + 8);
    glFunc->glVertex2d(pt.x - 8, pt.y + 8);
    glFunc->glVertex2d(pt.x - 8, pt.y - 8);
}

void drawClosestPointOsnapFlag(QOpenGLFunctions_2_1 *glFunc, CCVector3d pt)
{
    glFunc->glVertex3d(pt.x - 8, pt.y - 8,pt.z);
    glFunc->glVertex3d(pt.x + 8, pt.y - 8,pt.z);
    glFunc->glVertex3d(pt.x - 8, pt.y + 8,pt.z);
    glFunc->glVertex3d(pt.x + 8, pt.y + 8,pt.z);
    glFunc->glVertex3d(pt.x - 8, pt.y - 8,pt.z);
}

void cc2DItemBase::drawMeOnly2D(CC_DRAW_CONTEXT & context)
{
    if (context.display->windowName().compare("2D View") != 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
        glFunc->glPushName(getUniqueIDForDisplay());

    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(-halfW), static_cast<GLfloat>(-halfH), 0);
    GLdouble pSize;
    glFunc->glGetDoublev(GL_POINT_SIZE, &pSize);
    glFunc->glPointSize(10.0f);
    glFunc->glColor4f(m_pointColor.r, m_pointColor.g, m_pointColor.b, m_pointColor.a);                 // green

    if (m_keyPoints2d.size() != 0)
    {
        glFunc->glBegin(GL_POINTS);
        if (isSelected() && !m_bTrim)
        {
            for (unsigned i = 0; i < m_keyPoints2d.size(); ++i)
                glFunc->glVertex2d(m_keyPoints2d[i].x, m_keyPoints2d[i].y);
        }
        glFunc->glEnd();
    }

    if (m_pointsTmp2d.size() != 0)
    {
        if (m_pointsTmp2d.size() % 2 == 0)
        {
            for (int i = 0; i < m_pointsTmp2d.size() / 2; i++)
            {
                vector<CCVector3d> pts;
                pts.push_back(m_pointsTmp2d[2 * i]);
                pts.push_back(m_pointsTmp2d[2 * i + 1]);
                drawLines2D(context, pts, ccColor::Rgbaf(1, 1, 1, 1), true);
            }
        }
    }

    if (m_ptOsnampsTemp2d.size() != 0)
    {
        glFunc->glEnable(GL_BLEND);
        glFunc->glLineWidth(static_cast<GLfloat>(m_width));
        glFunc->glColor4f(1.0, 0.784, 0.015, 1.0);
        glFunc->glBegin(GL_LINE_STRIP);
        for (CCVector3d pt : m_ptOsnampsTemp2d)
        {
            // 0:mid point----1:key point---2:closest point
            if (m_OsnapFlag == 0)
				drawKeyPointOsnapFlag(glFunc, pt);
            if (m_OsnapFlag == 1)
                drawKeyPointOsnapFlag(glFunc, pt);
            if (m_OsnapFlag == 2)
                drawClosestPointOsnapFlag(glFunc, pt);

        }
        glFunc->glEnd();
		glFunc->glDisable(GL_BLEND);
    }

    glFunc->glPopMatrix();
    if (pushName)
        glFunc->glPopName();

    glFunc->glPointSize(pSize);
}
