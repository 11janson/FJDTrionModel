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

#include "cc2DRound.h"
#include "cc2DRect.h"
#include "ccLines.h"
#include "cc2DArcLine.h"
//Local
#include "ccCone.h"
#include "ccPointCloud.h"
#include "geometryUtils.h"

#include <cmath>

//qt
#include <QVector>
#include <QVector2d>

#define PI 3.14159265359
#define SIN(c)					(sin(c * PI / 180.0))			//弧度=角度乘以π后再除以180
#define COS(c)					(cos(c * PI / 180.0))

cc2DRound::cc2DRound(QString name)
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

cc2DRound::cc2DRound(const cc2DRound& poly)
    : cc2DItemBase()
{
    ccPointCloud* vertices = nullptr;
    initWith(vertices, poly);
}

cc2DRound* cc2DRound::clone() const
{
    cc2DRound* clonedPoly = new cc2DRound(*this);
    clonedPoly->setLocked(false); //there's no reason to keep the clone locked

    return clonedPoly;
}

bool cc2DRound::initWith(ccPointCloud*& vertices, const cc2DRound& poly)
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
            ccLog::Warning("[cc2DRound::initWith] Not enough memory to duplicate vertices!");
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
                ccLog::Warning("[cc2DRound::initWith] Not enough memory");
                success = false;
            }
        }
    }

    importParametersFrom(poly);

    return success;
}

void cc2DRound::importParametersFrom(const cc2DRound& poly)
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

void cc2DRound::showArrow(bool state, unsigned vertIndex, PointCoordinateType length)
{
}

ccBBox cc2DRound::getOwnBB(bool withGLFeatures/*=false*/)
{
    CCCoreLib::BoundingBox box;
    if (m_keyPoints3d.size() == 0)
        return box;

    if (m_drawPoints3d.size() == 200)
    {
        box.add(CCVector3(m_drawPoints3d[0].x, m_drawPoints3d[0].y, m_drawPoints3d[0].z));
        box.add(CCVector3(m_drawPoints3d[50].x, m_drawPoints3d[50].y, m_drawPoints3d[50].z));
        box.add(CCVector3(m_drawPoints3d[100].x, m_drawPoints3d[100].y, m_drawPoints3d[100].z));
        box.add(CCVector3(m_drawPoints3d[150].x, m_drawPoints3d[150].y, m_drawPoints3d[150].z));
    }
    box.setValidity(!isSelected());
    return box;
}

bool cc2DRound::hasColors() const
{
    return true;
}

void cc2DRound::applyGLTransformation(const ccGLMatrix& trans)
{
    //transparent call
    ccHObject::applyGLTransformation(trans);

    //invalidate the bounding-box
    //(and we hope the vertices will be updated as well!)
    invalidateBoundingBox();
}

//unit arrow
static QSharedPointer<ccCone> c_unitArrow(nullptr);

bool cc2DRound::updatePoints(CCVector3d pts)
{
    if (m_keyPoints3d.size() <= 0)
    {
        return false;
    }
    m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
    addDisplayPoints(pts);
    return true;
}

bool cc2DRound::addNewPoints(CCVector3d pts)
{
    if (m_first)
    {
        addDisplayPoints(pts);
        addDisplayPoints(pts);

        m_first = false;
    }
    else if (m_Type == ROUND3 && m_keyPoints3d.size() == 2)
    {
        addDisplayPoints(pts);
        return true;
    }
    else if (m_Type == ROUND3 && m_keyPoints3d.size() == 3)
    {
        m_finish = true;
    }
    else if ((m_Type == ROUND2 || m_Type == ROUNDRADIUS) && m_keyPoints3d.size() == 2)
    {
        m_finish = true;
    }

    if (m_finish)
        get3dPoints();

    return !m_finish;
}

bool cc2DRound::removePoints()
{
    if (m_keyPoints3d.size() <= 1)
    {
        return false;
    }

    if (m_Type == ROUND3)
    {
        m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
        return true;
    }
    else if (m_Type == ROUND2 || m_Type == ROUNDRADIUS)
    {
        return false;
    }


    //m_finish = true;
    return true;
}

bool cc2DRound::accpectClickedPoint(void)
{
    if (m_keyPoints3d.size() >= 2 && (m_Type == ROUND2 || m_Type == ROUNDRADIUS) && m_finish)
    {
        return false;
    }
    else if (m_keyPoints3d.size() >= 3 && m_Type == ROUND3 && m_finish)
    {
        return false;
    }
    else
    {
        return true;
    }
    return true;
}

void cc2DRound::get3dPoints()
{
    if (m_keyPoints3d.size() < 2)
        return;

    m_drawPoints3d.clear();
    QVariant var = getMetaData("axistype");
    QVector3D axis = var.value<QVector3D>();
    CCVector3d updir(axis.x(), axis.y(), axis.z());
    std::vector<CCVector3d> pts = m_keyPoints3d;

    if (m_Type == ROUND3 && pts.size() == 3)
        geometryUtils::getCenterOfCircle(pts, m_center, m_radius);

    if (m_Type == ROUND2 && pts.size() == 2)
        m_center = 0.5 * (pts[0] + pts[1]);

    if (m_Type == ROUNDRADIUS && pts.size() == 2)
        m_center = pts[0];

    m_radius = (m_center - pts[1]).norm();

    if(pts.size() >=2)
        m_drawPoints3d = geometryUtils::getCircleSamplePoints(updir, 200, m_center, pts[1]);
}



void cc2DRound::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (m_keyPoints3d.size() == 0 || (m_bTrim && m_mouse3dpos.size() == 0))
        return;

    cc2DItemBase::drawMeOnly(context);

    if (MACRO_Draw3D(context))
        drawIn3dView(context);
    else if (MACRO_Draw2D(context))
        drawIn2dView(context);
}

std::vector<CCVector2d> cc2DRound::centerIntersectArc(const CCVector2d& center, const CCVector2d& perpendi)
{
    std::vector<CCVector2d> intersections;
    if (isnan(center.x) || isnan(center.y))
    {
        return intersections;
    }
    if (this->getDrawPoints().size() <= 0)
    {
        return intersections;
    }
    CCVector2d q1 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->getDrawPoints()[100].x, this->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(this->getDrawPoints()[150].x, this->getDrawPoints()[150].y);


    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    Circle_2 circle(p1, p2, p3);


    q1 = CCVector2d(center.x, center.y);
    q2 = CCVector2d(perpendi.x, perpendi.y);


    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    Line_2 line(p1, p2);


    typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circle_2>::type Intersection_result;
    std::vector<Intersection_result> res;

    auto& output_iterator = std::back_inserter(res);


    CGAL::intersection(line, circle, output_iterator);
    if (res.size() <= 0)
    {
        return intersections;
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
    return intersections;
}

bool less(QVector2D a, QVector2D b, QVector2D center)
{
    if (a.x() - center.x() >= 0 && b.x() - center.x() < 0)
        return true;
    if (a.x() - center.x() < 0 && b.x() - center.x() >= 0)
        return false;
    if (a.x() - center.x() == 0 && b.x() - center.x() == 0) {
        if (a.y() - center.y() >= 0 || b.y() - center.y() >= 0)
            return a.y() > b.y();
        return b.y() > a.y();
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    int det = (a.x() - center.x()) * (b.y() - center.y()) - (b.x() - center.x()) * (a.y() - center.y());
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    int d1 = (a.x() - center.x()) * (a.x() - center.x()) + (a.y() - center.y()) * (a.y() - center.y());
    int d2 = (b.x() - center.x()) * (b.x() - center.x()) + (b.y() - center.y()) * (b.y() - center.y());
    return d1 > d2;
}

void cc2DRound::sortPosInCircle(std::vector<CCVector3d>& lst, CCVector3d centerPos)
{
    if (lst.size() <= 0)
    {
        return;
    }
    QVector<QVector2D> lstQvec2d;
    foreach(CCVector3d pos, lst)
    {
        lstQvec2d.push_back(QVector2D(pos.x, pos.y));
    }
    QVector2D center = QVector2D(centerPos.x, centerPos.y);

    qSort(lstQvec2d.begin(), lstQvec2d.end(), [&](const QVector2D &a, const QVector2D &b) {
        if (a.x() - center.x() >= 0 && b.x() - center.x() < 0)
            return true;
        if (a.x() - center.x() < 0 && b.x() - center.x() >= 0)
            return false;
        if (a.x() - center.x() == 0 && b.x() - center.x() == 0) {
            if (a.y() - center.y() >= 0 || b.y() - center.y() >= 0)
                return a.y() > b.y();
            return b.y() > a.y();
        }

        // compute the cross product of vectors (center -> a) x (center -> b)
        int det = (a.x() - center.x()) * (b.y() - center.y()) - (b.x() - center.x()) * (a.y() - center.y());
        if (det < 0)
            return true;
        if (det > 0)
            return false;

        // points a and b are on the same line from the center
        // check which point is closer to the center
        int d1 = (a.x() - center.x()) * (a.x() - center.x()) + (a.y() - center.y()) * (a.y() - center.y());
        int d2 = (b.x() - center.x()) * (b.x() - center.x()) + (b.y() - center.y()) * (b.y() - center.y());
        return d1 > d2;
    });
    lst.clear();
    foreach(QVector2D pos2d, lstQvec2d)
    {
        lst.push_back(CCVector3d(pos2d.x(), pos2d.y(), centerPos.z));
    }
    lst.push_back(lst[0]);
}

std::vector<CCVector3d> cc2DRound::getCenter3D()
{
	std::vector<CCVector3d> lst;
	lst.push_back(getCenter());

	std::vector<CCVector3d> result = unprojectTo3d(lst, m_3dCamera);
	return result;
}



void cc2DRound::drawIn2dView(CC_DRAW_CONTEXT& context)
{
    if (m_bTrim && m_mouse2dpos.size() == 0)
    {
        return;
    }
    if (m_keyPoints2d.size() == 0)
        return;

    QString name = context.display->windowName();
    if (context.display->windowName().compare("2D View") != 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    CCVector3d updir(0, 0, 1);
    CCVector3d center;
    double dRadius = 0;
    std::vector<CCVector3d> pts, drawPoints;
    for (int i = 0; i < m_keyPoints2d.size(); i++)
        pts.push_back(m_keyPoints2d[i]);

    if (m_Type == ROUND3 && pts.size() == 3)
        geometryUtils::getCenterOfCircle(pts, center, dRadius);



    if (m_Type == ROUND2 && pts.size() == 2)
        center = 0.5 * (pts[0] + pts[1]);

    if (m_Type == ROUNDRADIUS && pts.size() == 2)
        center = pts[0];

    if (m_Type == ROUND3 && pts.size() == 2)
        drawPoints = pts;
    else
    {
        if (pts.size() >= 2)
        {
            drawPoints = geometryUtils::getCircleSamplePoints(updir, 200, center, pts[1]);
            if (drawPoints.size() > 0 && (isnan(drawPoints[0].x) || isnan(drawPoints[0].y) || isnan(drawPoints[0].z)))
            {
                drawPoints.clear();
                drawPoints = pts;
            }
        }

        else
            return;
    }

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
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(-halfW), static_cast<GLfloat>(-halfH), 0);


    m_drawPoints2d = drawPoints;
    if (drawPoints.size() > 0)
    {
        m_radius = CCVector3d::vdistance(center.u, drawPoints[0].u);
        m_center = center;
    }

    if (!m_bTrim)
    {
        glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
        if (drawPoints.size() != 0)
        {
			if (drawPoints.size() > 2)
				drawPoints.push_back(CCVector3d(drawPoints[2].x, drawPoints[2].y, drawPoints[2].z));
			drawLines2D(context, drawPoints, m_rgbColor);

        }
    }
    else if (m_bTrim && m_mouse2dpos.size() > 0 )
    {
        double z = drawPoints[0].z;
        drawRoundTrim(context, z, drawPoints);
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

void cc2DRound::drawRoundTrim(CC_DRAW_CONTEXT& context, double z, std::vector<CCVector3d>& pts)
{
    if (m_mouse2dpos.size() == 0)
    {
        return;
    }
    m_pt2dUnselect2.clear();
    m_pt2dUnselect.clear();
    int result = m_lstIntersections.size() - 1;
    if (result <= 1)
    {
        drawRoundOnly1Intersection(context, z, pts);
    }
    else if (result == 2)
    {
        drawRoundOnly2Intersection(context, z);
    }
    else
    {
        drawRoundMore2Intersection(context, z);
    }
}

void cc2DRound::drawRoundOnly1Intersection(CC_DRAW_CONTEXT& context, double z, std::vector<CCVector3d>& pts)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;
    glFunc->glColor4f(1, 1, 1, 1);

    glFunc->glBegin(GL_LINE_STRIP);
    for (int i = 0; i < pts.size(); i++)
        glFunc->glVertex2f(pts[i].x, pts[i].y);
    if (pts.size() > 0)
        glFunc->glVertex2f(pts[0].x, pts[0].y);
    glFunc->glEnd();
}

void cc2DRound::drawRoundOnly2Intersection(CC_DRAW_CONTEXT& context, double z)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    CCVector3d updir(0, 0, 1);
    if (isnan(m_center.x) || isnan(m_center.y))
    {
        drawRoundAbnormal(context);
        return;
    }
    bool is_on_arc = false;
    for (int i = 0; i < m_lstIntersections.size() - 1; i++)
    {
        CCVector3d p13d = m_lstIntersections[i];
        CCVector3d p23d = m_lstIntersections[i + 1];

        CCVector2d p12d = CCVector2d(p13d.x, p13d.y);
        CCVector2d p22d = CCVector2d(p23d.x, p23d.y);

        CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p12d, p22d);
        std::vector<CCVector2d> arcInters = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
        if (arcInters.size() >0 &&(isnan(arcInters[0].x)||isnan(arcInters[arcInters.size() - 1].x) || isnan(perpendi.x)))
        {
            continue;
        }
        CCVector2d mouse2d = CCVector2d(m_mouse2dpos[0].x, m_mouse2dpos[0].y);
        if (isnan(mouse2d.x) || isnan(mouse2d.y))
        {
            continue;
        }
        int j = 0;
        int k = -1;
        for (; j < arcInters.size();)
        {
            if (segmentIntersect(p12d, arcInters[j], CCVector2d(m_center.x, m_center.y), mouse2d)
                || segmentIntersect(p22d, arcInters[j], CCVector2d(m_center.x, m_center.y), mouse2d))
            {
                is_on_arc = true;
                break;
            }
            j++;
        }
        if (!is_on_arc)
        {
            continue;
        }

        k = j ? 0 : 1;

        CCVector3d select = CCVector3d(arcInters[j].x, arcInters[j].y, z);
        CCVector3d unselect = CCVector3d(arcInters[k].x, arcInters[k].y, z);

        CCVector3d tempCenter;
        double tempRadius;
        double sweepAngle;

        std::vector<CCVector3d> drawPointsSelect, drawPointsUnselect;

        std::vector<CCVector3d> ptSelect;
        ptSelect.push_back(CCVector3d(p12d.x, p12d.y, z));
        ptSelect.push_back(select);
        ptSelect.push_back(CCVector3d(p22d.x, p22d.y, z));
        drawPointsSelect = geometryUtils::getCircleArcSamplePoints(updir, 200, ptSelect, tempCenter, tempRadius, sweepAngle);
        drawPointsSelect.push_back(CCVector3d(p22d.x, p22d.y, z));

        if (drawPointsSelect.size() > 0)
        {
            glFunc->glColor4f(1, 1, 1, 1);
            glFunc->glBegin(GL_LINE_STRIP);
            for (int j = 0; j < drawPointsSelect.size(); j++)
                glFunc->glVertex2f(drawPointsSelect[j].x, drawPointsSelect[j].y);
            glFunc->glEnd();
        }
        else
            is_on_arc = false;

        m_pt2dUnselect.push_back(CCVector3d(p12d.x, p12d.y, z));
        m_pt2dUnselect.push_back(unselect);
        m_pt2dUnselect.push_back(CCVector3d(p22d.x, p22d.y, z));
        drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, tempCenter, tempRadius, sweepAngle);
        drawPointsUnselect.push_back(CCVector3d(p22d.x, p22d.y, z));

        m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
        if (drawPointsUnselect.size() > 0)
        {
            glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
            glFunc->glBegin(GL_LINE_STRIP);
            for (int j = 0; j < drawPointsUnselect.size(); j++)
                glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
            glFunc->glEnd();
        }
        else
            is_on_arc = false;

        if (is_on_arc)
        {
            break;
        }
    }
    if (!is_on_arc)
    {
        drawRoundAbnormal(context);
    }
}

void cc2DRound::drawRoundMore2Intersection(CC_DRAW_CONTEXT& context, double z)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    CCVector3d updir(0, 0, 1);
    bool find = false;

    for (int i = 0; i < m_lstIntersections.size() - 1; i++)
    {
        CCVector3d p13d = m_lstIntersections[i];
        CCVector3d p23d = m_lstIntersections[i + 1];

        CCVector2d p12d = CCVector2d(p13d.x, p13d.y);
        CCVector2d p22d = CCVector2d(p23d.x, p23d.y);

        CCVector2d mouse2d = CCVector2d(m_mouse2dpos[0].x, m_mouse2dpos[0].y);

        // 计算圆心坐标
        double centerX = (p12d.x + p22d.x) / 2.0;
        double centerY = (p12d.y + p22d.y) / 2.0;

        // 计算圆弧的弧度
        double chord = sqrt(pow((p22d.x - p12d.x), 2) + pow((p22d.y - p12d.y), 2));
        double sagitta = m_radius - sqrt(pow(m_radius, 2) - pow((chord / 2), 2));
        double theta = 2.0 * asin(chord / (2.0 * m_radius));
        if (isnan(chord) || isnan(sagitta) || isnan(theta))
            continue;
        if (abs(chord - 0) < EPSINON)
            continue;

        // 根据顺时针方向，计算中点坐标
        CCVector2d arcCenter;
        arcCenter.x = centerX + (sagitta * (p12d.y - p22d.y)) / chord;
        arcCenter.y = centerY + (sagitta * (p22d.x - p12d.x)) / chord;

        if (isnan(arcCenter.x) || isnan(mouse2d.x))
        {
            continue;
        }

        if (!(segmentIntersect(p12d, arcCenter, CCVector2d(m_center.x, m_center.y), mouse2d) ||
            segmentIntersect(p22d, arcCenter, CCVector2d(m_center.x, m_center.y), mouse2d)))
        {
            continue;
        }

        CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p12d, p22d);
        std::vector<CCVector2d> arcInters = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
        if (arcInters.size() > 0 && (isnan(arcInters[0].x) || isnan(arcInters[arcInters.size() - 1].x)|| isnan(perpendi.x)))
        {
            continue;
        }
        double mouseArcInterDistance1 = CCVector3d::vdistance(m_mouse2dpos[0].u, CCVector3d(arcInters[0].x, arcInters[0].y, m_center.z).u);
        double mouseArcInterDistance2 = CCVector3d::vdistance(m_mouse2dpos[0].u, CCVector3d(arcInters[1].x, arcInters[1].y, m_center.z).u);

        CCVector2d nearPos;
        CCVector2d farPos;
        if (mouseArcInterDistance1 < mouseArcInterDistance2)
        {
            nearPos = arcInters[0];
            farPos = arcInters[1];
        }
        else
        {
            nearPos = arcInters[1];
            farPos = arcInters[0];
        }


        if (!(segmentIntersect(p12d, nearPos, CCVector2d(m_center.x, m_center.y), mouse2d)
            || segmentIntersect(p22d, nearPos, CCVector2d(m_center.x, m_center.y), mouse2d)))
        {
            find = false;
            continue;
        }
        else
        {
            find = true;
        }

        CCVector3d select = CCVector3d(nearPos.x, nearPos.y, z);
        CCVector3d unselect = CCVector3d(farPos.x, farPos.y, z);

        CCVector3d tempCenter;
        double tempRadius;
        double sweepAngle;

        std::vector<CCVector3d> drawPointsSelect, drawPointsUnselect;

        std::vector<CCVector3d> ptSelect;
        ptSelect.push_back(CCVector3d(p12d.x, p12d.y, z));
        ptSelect.push_back(select);
        ptSelect.push_back(CCVector3d(p22d.x, p22d.y, z));
        drawPointsSelect = geometryUtils::getCircleArcSamplePoints(updir, 200, ptSelect, tempCenter, tempRadius, sweepAngle);
        drawPointsSelect.push_back(CCVector3d(p22d.x, p22d.y, z));

        if (drawPointsSelect.size() > 0)
        {
            glFunc->glColor4f(1, 1, 1, 1);
            glFunc->glBegin(GL_LINE_STRIP);
            for (int j = 0; j < drawPointsSelect.size(); j++)
                glFunc->glVertex2f(drawPointsSelect[j].x, drawPointsSelect[j].y);
            glFunc->glEnd();
        }
        else
            find = false;


        m_pt2dUnselect.push_back(CCVector3d(p12d.x, p12d.y, z));
        m_pt2dUnselect.push_back(unselect);
        m_pt2dUnselect.push_back(CCVector3d(p22d.x, p22d.y, z));
        drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, tempCenter, tempRadius, sweepAngle);
        drawPointsUnselect.push_back(CCVector3d(p22d.x, p22d.y, z));

        m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
        if (drawPointsUnselect.size() > 0)
        {
            glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
            glFunc->glBegin(GL_LINE_STRIP);
            for (int j = 0; j < drawPointsUnselect.size(); j++)
                glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
            glFunc->glEnd();
        }
        else
            find = false;

    }
    if (!find)
    {
        drawRoundAbnormal(context);
    }
}
void cc2DRound::drawRoundAbnormal(CC_DRAW_CONTEXT& context)
{
    m_bAbnormal = true;
	m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;
    CCVector3d updir(0, 0, 1);

    std::vector<CCVector3d> pts, drawPoints;
    for (int i = 0; i < m_keyPoints2d.size(); i++)
        pts.push_back(m_keyPoints2d[i]);
    drawPoints = geometryUtils::getCircleSamplePoints(updir, 200, m_center, pts[1]);

    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);

    glFunc->glBegin(GL_LINE_STRIP);
    for (int i = 0; i < drawPoints.size(); i++)
        glFunc->glVertex2f(drawPoints[i].x, drawPoints[i].y);
    if (drawPoints.size() > 0)
        glFunc->glVertex2f(drawPoints[0].x, drawPoints[0].y);
    glFunc->glEnd();
}


void cc2DRound::drawIn3dView(CC_DRAW_CONTEXT& context)
{
    if (context.display->windowName().compare("3D View") != 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;


    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
        glFunc->glPushName(getUniqueIDForDisplay());

    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);

    if (m_width != 0)
    {
        glFunc->glEnable(GL_LINE_SMOOTH);
        glFunc->glEnable(GL_BLEND);
        glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);
        glFunc->glLineWidth(static_cast<GLfloat>(m_width));
    }

    draw_circle_in3d_view(context);

    if (m_width != 0)
    {
        glFunc->glDisable(GL_BLEND);
        glFunc->glDisable(GL_LINE_SMOOTH);
        glFunc->glPopAttrib();
    }
    if (pushName)
        glFunc->glPopName();
}

void cc2DRound::draw_circle_in3d_view(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (!glFunc)
        return;

    getdrawPoints3d();
    glFunc->glBegin(GL_LINE_STRIP);
    for (int i = 0; i < m_drawPoints3d.size(); i++)
        glFunc->glVertex3f(m_drawPoints3d[i].x, m_drawPoints3d[i].y, m_drawPoints3d[i].z);
    if (m_drawPoints3d.size() > 0)
        glFunc->glVertex3f(m_drawPoints3d[0].x, m_drawPoints3d[0].y, m_drawPoints3d[0].z);
    glFunc->glEnd();
}

bool cc2DRound::toFile_MeOnly(QFile& out) const
{
    if (!ccHObject::toFile_MeOnly(out))
        return false;

    //we can't save the associated cloud here (as it may be shared by multiple polylines)
    //so instead we save it's unique ID (dataVersion>=28)
    //WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
    ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
    if (!vertices)
    {
        ccLog::Warning("[cc2DRound::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
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

bool cc2DRound::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
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
        m_globalShift = CCVector3d(0, 0, 0);
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
        ccSerializationHelper::CoordsFromDataStream(inStream, flags, (PointCoordinateType*)&m_width, 1);
    else
        m_width = 0;

    return true;
}

bool cc2DRound::split(PointCoordinateType maxEdgeLength,
    std::vector<cc2DRound*>& parts)
{
    parts.clear();

    //not enough vertices?
    unsigned vertCount = size();
    if (vertCount <= 2)
    {
        parts.push_back(new cc2DRound(*this));
        return true;
    }

    unsigned startIndex = 0;
    unsigned lastIndex = vertCount - 1;
    while (startIndex <= lastIndex)
    {
        unsigned stopIndex = startIndex;
        while (stopIndex < lastIndex && (*getPoint(stopIndex + 1) - *getPoint(stopIndex)).norm() <= maxEdgeLength)
        {
            ++stopIndex;
        }

        //number of vertices for the current part
        unsigned partSize = stopIndex - startIndex + 1;

        //if the polyline is closed we have to look backward for the first segment!
        if (startIndex == 0)
        {
            if (isClosed())
            {
                unsigned realStartIndex = vertCount;
                while (realStartIndex > stopIndex && (*getPoint(realStartIndex - 1) - *getPoint(realStartIndex % vertCount)).norm() <= maxEdgeLength)
                {
                    --realStartIndex;
                }

                if (realStartIndex == stopIndex)
                {
                    //whole loop
                    parts.push_back(new cc2DRound(*this));
                    return true;
                }
                else if (realStartIndex < vertCount)
                {
                    partSize += (vertCount - realStartIndex);
                    assert(realStartIndex != 0);
                    lastIndex = realStartIndex - 1;
                    //warning: we shift the indexes!
                    startIndex = realStartIndex;
                    stopIndex += vertCount;
                }
            }
            else if (partSize == vertCount)
            {
                //whole polyline
                parts.push_back(new cc2DRound(*this));
                return true;
            }
        }

        if (partSize > 1) //otherwise we skip that point
        {
            //create the corresponding part
            CCCoreLib::ReferenceCloud ref(m_theAssociatedCloud);
            if (!ref.reserve(partSize))
            {
                ccLog::Error("[cc2DRound::split] Not enough memory!");
                return false;
            }

            for (unsigned i = startIndex; i <= stopIndex; ++i)
            {
                ref.addPointIndex(i % vertCount);
            }

            //ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
            //ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
            //cc2DRound* part = new cc2DRound(subset);
            //part->initWith(subset, *this);
            //part->setClosed(false); //by definition!
            //parts.push_back(part);
        }

        //forward
        startIndex = (stopIndex % vertCount) + 1;
    }

    return true;
}

PointCoordinateType cc2DRound::computeLength() const
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

unsigned cc2DRound::getUniqueIDForDisplay() const
{
    if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
        return m_parent->getParent()->getUniqueID();
    else
        return getUniqueID();
}

unsigned cc2DRound::segmentCount() const
{
    unsigned count = size();
    if (count && !isClosed())
    {
        --count;
    }
    return count;
}

void cc2DRound::setGlobalShift(const CCVector3d& shift)
{
    ccShiftedObject::setGlobalShift(shift);

    ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
    if (pc && pc->getParent() == this)
    {
        //auto transfer the global shift info to the vertices
        pc->setGlobalShift(shift);
    }
}

void cc2DRound::setGlobalScale(double scale)
{
    ccShiftedObject::setGlobalScale(scale);

    ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
    if (pc && pc->getParent() == this)
    {
        //auto transfer the global scale info to the vertices
        pc->setGlobalScale(scale);
    }
}

const CCVector3d& cc2DRound::getGlobalShift() const
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

double cc2DRound::getGlobalScale() const
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

ccPointCloud* cc2DRound::samplePoints(bool densityBased,
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
        ccLog::Warning("[cc2DRound::samplePoints] Not enough memory");
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

cc2DRound* cc2DRound::smoothChaikin(PointCoordinateType ratio, unsigned iterationCount) const
{
    if (iterationCount == 0)
    {
        assert(false);
        ccLog::Warning("[cc2DRound::smoothChaikin] Invalid input (iteration count)");
        return nullptr;
    }

    if (ratio < 0.05f || ratio > 0.45f)
    {
        assert(false);
        ccLog::Warning("[cc2DRound::smoothChaikin] invalid ratio");
        return nullptr;
    }

    if (size() < 3)
    {
        ccLog::Warning("[cc2DRound::smoothChaikin] not enough segments");
        return nullptr;
    }

    const CCCoreLib::GenericIndexedCloudPersist* currentIterationVertices = this; //a polyline is actually a ReferenceCloud!
    cc2DRound* smoothPoly = nullptr;

    bool openPoly = !isClosed();

    for (unsigned it = 0; it < iterationCount; ++it)
    {
        //reserve memory for the new vertices
        unsigned vertCount = currentIterationVertices->size();
        unsigned segmentCount = (openPoly ? vertCount - 1 : vertCount);

        ccPointCloud* newStateVertices = new ccPointCloud("vertices");
        if (!newStateVertices->reserve(segmentCount * 2))
        {
            ccLog::Warning("[cc2DRound::smoothChaikin] not enough memory");
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
            //smoothPoly = new cc2DRound(newStateVertices);
            //smoothPoly->addChild(newStateVertices);
            //newStateVertices->setEnabled(false);
            //if (!smoothPoly->reserve(newStateVertices->size()))
            //{
            //	ccLog::Warning("[cc2DRound::smoothChaikin] not enough memory");
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

bool cc2DRound::IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, cc2DRound** polyline/*=nullptr*/)
{
    if (!cloud)
    {
        assert(false);
        return false;
    }

    // check whether the input point cloud acts as the vertices of a polyline
    {
        ccHObject* parent = cloud->getParent();
        if (parent && parent->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DRound*>(parent)->getAssociatedCloud() == cloud)
        {
            if (polyline)
            {
                *polyline = static_cast<cc2DRound*>(parent);
            }
            return true;
        }
    }

    // now check the children
    for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
    {
        ccHObject* child = cloud->getChild(i);
        if (child && child->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DRound*>(child)->getAssociatedCloud() == cloud)
        {
            if (polyline)
            {
                *polyline = static_cast<cc2DRound*>(child);
            }
            return true;
        }
    }

    return false;
}

bool cc2DRound::createNewPolylinesFromSelection(std::vector<cc2DRound*>& output)
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
        ccLog::Warning("[cc2DRound::createNewPolylinesFromSelection] Unsupported vertex cloud");
        return false;
    }
    const ccGenericPointCloud::VisibilityTableType& verticesVisibility = verticesCloud->getTheVisibilityArray();
    if (verticesVisibility.size() < vertCount)
    {
        // no visibility table instantiated
        ccLog::Warning("[cc2DRound::createNewPolylinesFromSelection] No visibility table instantiated");
        return false;
    }

    bool success = true;
    {
        cc2DRound* chunkPoly = nullptr;
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
                    chunkPoly = new cc2DRound(chunkCloud);
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
        ccLog::Warning("[cc2DRound::createNewPolylinesFromSelection] Not enough memory");
        // delete the already created polylines
        for (cc2DRound* poly : output)
        {
            delete poly;
        }
        output.clear();
    }
    return success;
}

std::vector<CCVector2d> cc2DRound::intersect(ccLines* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    for (int i = 0; i < pItem->get2dKeyPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;

        if (this->getDrawPoints().size() <= 0)
        {
            return intersections;
        }
        CCVector2d q1 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);
        CCVector2d q2 = CCVector2d(this->getDrawPoints()[100].x, this->getDrawPoints()[100].y);
        CCVector2d q3 = CCVector2d(this->getDrawPoints()[150].x, this->getDrawPoints()[150].y);

        Point_2 p1 = Point_2(q1.x, q1.y);
        Point_2 p2 = Point_2(q2.x, q2.y);
        Point_2 p3 = Point_2(q3.x, q3.y);
        if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
            std::swap(p1, p3);
        Circle_2 circle(p1, p2, p3);


        q1 = CCVector2d(pItem->get2dKeyPoints()[i].x, pItem->get2dKeyPoints()[i].y);
        q2 = CCVector2d(pItem->get2dKeyPoints()[i + 1].x, pItem->get2dKeyPoints()[i + 1].y);
        if (q1 == q2)
            continue;

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

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                if (isnan(x) || isnan(y))
                    continue;
                CCVector2d pos(x, y);
                intersections.push_back(pos);
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

    return lstIntersection;
}

std::vector<CCVector2d> cc2DRound::intersect(cc2DArcLine* pItem, bool filter)
{
    std::vector<CCVector2d> intersections;

    if (this->getDrawPoints().size() <= 0)
    {
        return intersections;
    }
    CCVector2d q1 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->getDrawPoints()[100].x, this->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(this->getDrawPoints()[150].x, this->getDrawPoints()[150].y);


    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circle_2 circle1(p1, p2, p3);

    q1 = CCVector2d(pItem->get2dKeyPoints()[0].x, pItem->get2dKeyPoints()[0].y);
    q2 = CCVector2d(pItem->get2dKeyPoints()[1].x, pItem->get2dKeyPoints()[1].y);
    q3 = CCVector2d(pItem->get2dKeyPoints()[2].x, pItem->get2dKeyPoints()[2].y);

    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circular_arc_2 arc;


	typedef typename CGAL::CK2_Intersection_traits<K, Circular_arc_2, Circle_2>::type Intersection_result;
	std::vector<Intersection_result> res;
    try
    {
        arc = Circular_arc_2(p1, p2, p3);
        CGAL::intersection(arc, circle1, std::back_inserter(res));
    }
    catch (...)
    {
        qDebug() << "The intersection calculation failed.";
        return intersections;
    }
    if (res.size() <= 0)
    {
        return intersections;
    }

    using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;

    for (const auto& element : res)
    {
        auto algPoint = std::get<0>(boost::get< boostRetVal >(element));

        if (&algPoint)
        {
            auto point = Point_2(CGAL::to_double(algPoint.x()), CGAL::to_double(algPoint.y()));

            double x = CGAL::to_double(point.x());
            double y = CGAL::to_double(point.y());
            if (isnan(x) || isnan(y))
                continue;
            CCVector2d pos(x, y);
            intersections.push_back(pos);
        }
    }

    std::cout << res.size() << '\n';
    return intersections;
}

std::vector<CCVector2d> cc2DRound::intersect(cc2DRect* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    if (this->getDrawPoints().size() <= 0)
    {
        return lstIntersection;
    }
    CCVector2d q1 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->getDrawPoints()[100].x, this->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(this->getDrawPoints()[150].x, this->getDrawPoints()[150].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circle_2 circle(p1, p2, p3);

    for (int i = 0; i < pItem->getDrawPoints().size() - 1; i++)
    {
        std::vector<CCVector2d> intersections;
        q1 = CCVector2d(pItem->getDrawPoints()[i].x, pItem->getDrawPoints()[i].y);
        q2 = CCVector2d(pItem->getDrawPoints()[i + 1].x, pItem->getDrawPoints()[i + 1].y);


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

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                if (isnan(x) || isnan(y))
                    continue;
                CCVector2d pos(x, y);
                intersections.push_back(pos);
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

    int index = pItem->getDrawPoints().size() - 1;

    q1 = CCVector2d(pItem->getDrawPoints()[index].x, pItem->getDrawPoints()[index].y);
    q2 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);


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

            double x = CGAL::to_double(point.x());
            double y = CGAL::to_double(point.y());
            if (isnan(x) || isnan(y))
                continue;
            CCVector2d pos(x, y);
            intersections.push_back(pos);
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

std::vector<CCVector2d> cc2DRound::intersect(cc2DRound* pItem, bool filter)
{
    std::vector<CCVector2d> intersections;

    if (this->getDrawPoints().size() <= 0)
    {
        return intersections;
    }
    CCVector2d q1 = CCVector2d(this->getDrawPoints()[0].x, this->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->getDrawPoints()[100].x, this->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(this->getDrawPoints()[150].x, this->getDrawPoints()[150].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circle_2 circle1(p1, p2, p3);

    if (pItem->getDrawPoints().size() <= 0)
    {
        return intersections;
    }
    q1 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);
    q2 = CCVector2d(pItem->getDrawPoints()[100].x, pItem->getDrawPoints()[100].y);
    q3 = CCVector2d(pItem->getDrawPoints()[150].x, pItem->getDrawPoints()[150].y);

    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
    Circle_2 circle2(p1, p2, p3);


    typedef typename CGAL::CK2_Intersection_traits<K, Circle_2, Circle_2>::type Intersection_result;
    std::vector<Intersection_result> res;

    try
    {
        CGAL::intersection(circle1, circle2, std::back_inserter(res));
    }
    catch (...)
    {
        qDebug() << "The intersection calculation failed.";
        return intersections;
    }
    if (res.size() <= 0)
    {
        return intersections;
    }
    using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;
    try
    {
        for (const auto& element : res)
        {
            auto algPoint = std::get<0>(boost::get< boostRetVal >(element));

            if (&algPoint)
            {
                auto point = Point_2(CGAL::to_double(algPoint.x()), CGAL::to_double(algPoint.y()));

                double x = CGAL::to_double(point.x());
                double y = CGAL::to_double(point.y());
                if (isnan(x) || isnan(y))
                    continue;
                CCVector2d pos(x, y);
                intersections.push_back(pos);
            }
        }
    }
    catch (...)
    {
        return intersections;
    }
    std::cout << res.size() << '\n';
    return intersections;
}