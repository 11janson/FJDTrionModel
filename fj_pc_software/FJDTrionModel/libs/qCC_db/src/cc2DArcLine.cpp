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

#include "cc2DArcLine.h"
#include "cc2DRound.h"
#include "cc2DRect.h"
#include "ccLines.h"

//Local
#include "ccCone.h"
#include "ccPointCloud.h"
#include "geometryUtils.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

//qt
#include <QtMath>

#define PI 3.14159265359
#define SIN(c)					(sin(c * PI / 180.0))			//弧度=角度乘以π后再除以180
#define COS(c)					(cos(c * PI / 180.0))

struct PointDistance
{
    CCVector2d pos2d;
    double distance;
    PointDistance()
    {
        pos2d = CCVector2d(0, 0);
        distance = 0;
    }
    PointDistance(CCVector2d pos, double dis)
    {
        pos2d = pos;
        distance = dis;
    }
};


cc2DArcLine::cc2DArcLine(QString name)
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

    qRegisterMetaType<PointDistance>("PointDistance");
    qRegisterMetaType<PointDistance>("PointDistance&");

    qRegisterMetaType<CCVector2d>("CCVector2d");
    qRegisterMetaType<CCVector2d>("CCVector2d&");
}

cc2DArcLine::cc2DArcLine(const cc2DArcLine& poly)
	: cc2DItemBase()
{
	ccPointCloud* vertices = nullptr;
	initWith(vertices, poly);
}

cc2DArcLine* cc2DArcLine::clone() const
{
	cc2DArcLine* clonedPoly = new cc2DArcLine(*this);
	clonedPoly->setLocked(false); //there's no reason to keep the clone locked

	return clonedPoly;
}

bool cc2DArcLine::initWith(ccPointCloud*& vertices, const cc2DArcLine& poly)
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
			ccLog::Warning("[cc2DArcLine::initWith] Not enough memory to duplicate vertices!");
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
				ccLog::Warning("[cc2DArcLine::initWith] Not enough memory");
				success = false;
			}
		}
	}

	importParametersFrom(poly);

	return success;
}

void cc2DArcLine::importParametersFrom(const cc2DArcLine& poly)
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

void cc2DArcLine::showArrow(bool state, unsigned vertIndex, PointCoordinateType length)
{

}

ccBBox cc2DArcLine::getOwnBB(bool withGLFeatures/*=false*/)
{
    CCCoreLib::BoundingBox box;
    if (m_keyPoints3d.size() == 0)
        return box;

    if (m_drawPoints3d.size() == 200)
    {
        box.add(CCVector3(m_drawPoints3d[0].x, m_drawPoints3d[0].y, m_drawPoints3d[0].z));
        box.add(CCVector3(m_drawPoints3d[100].x, m_drawPoints3d[100].y, m_drawPoints3d[100].z));
        box.add(CCVector3(m_drawPoints3d[199].x, m_drawPoints3d[199].y, m_drawPoints3d[199].z));
    }        
    box.setValidity(!isSelected());
    return box;
}

bool cc2DArcLine::hasColors() const
{
	return true;
}

void cc2DArcLine::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//invalidate the bounding-box
	//(and we hope the vertices will be updated as well!)
	invalidateBoundingBox();
}

//unit arrow
static QSharedPointer<ccCone> c_unitArrow(nullptr);

bool cc2DArcLine::updatePoints(CCVector3d pt)
{
	if (m_keyPoints3d.size() <= 0)
		return false;

    m_keyPoints3d.erase(m_keyPoints3d.end() - 1);
	addDisplayPoints(pt);
	return true;
}

bool cc2DArcLine::addNewPoints(CCVector3d pts)
{
	if (m_first)
	{
		addDisplayPoints(pts);
		addDisplayPoints(pts);
		m_first = false;
	}
	else if(m_keyPoints3d.size() == 2 )
	{
		addDisplayPoints(pts);
		return true;
	}
	else if (m_keyPoints3d.size() == 3)
	{
		m_finish = true;
	}

    if (m_finish)
        get3dPoints();

	return !m_finish;
}

bool cc2DArcLine::removePoints()
{
	if (m_keyPoints3d.size() <= 1)
	{
		return false;
	}

    m_keyPoints3d.erase(m_keyPoints3d.end() - 1);

	return true;
}

bool cc2DArcLine::accpectClickedPoint(void)
{
	if (m_keyPoints3d.size() == 3  && m_finish)
	{
		return false;
	}
	else
	{
		return true;
	}
	return true;
}

void cc2DArcLine::get3dPoints()
{
    if(m_keyPoints3d.size() <= 2)
        return;

    m_drawPoints3d.clear();
    QVariant var = getMetaData("axistype");
    QVector3D axis = var.value<QVector3D>();
    CCVector3d updir(axis.x(), axis.y(), axis.z());
    m_threeArcPoints.clear();

    for (int i = 0; i < m_keyPoints3d.size(); i++)
    {
        CCVector3d pt = m_keyPoints3d[i];
        pt = geometryUtils::projToplane(CCVector3d(axis.x(), axis.y(), axis.z()), m_keyPoints3d[1], pt);
        m_threeArcPoints.push_back(pt);
    }

    if (m_threeArcPoints.size() != 3)
        return ;

    m_drawPoints3d = geometryUtils::getCircleArcSamplePoints(updir, 200, m_threeArcPoints, m_center, m_radius, m_sweepAngle);
}

void cc2DArcLine::drawMeOnly(CC_DRAW_CONTEXT& context)
{

    if (m_keyPoints3d.size() == 0 || (m_bTrim && m_mouse3dpos.size() == 0))
        return;

    cc2DItemBase::drawMeOnly(context);

    if (MACRO_Draw3D(context))
        drawIn3dView(context);
    else if (MACRO_Draw2D(context))
        drawIn2dView(context);
}


std::vector<CCVector2d> cc2DArcLine::centerIntersectArc(const CCVector2d& center, const CCVector2d& perpendi)
{
    std::vector<CCVector2d> intersections;
    if (isnan(center.x)|| isnan(center.y))
    {
        return intersections;
    }
    CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[0].x, this->get2dKeyPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[1].x, this->get2dKeyPoints()[1].y);
    CCVector2d q3 = CCVector2d(this->get2dKeyPoints()[2].x, this->get2dKeyPoints()[2].y);


    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    //Circle_2 circle(p1, p2, p3);

    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

    Circular_arc_2 arc;
    try
    {
        arc = Circular_arc_2(p1, p2, p3);
    }
    catch (...)
    {
        return intersections;
        qDebug() << "COUNTERCLOCKWISE faided!" << "\n";
    }
    q1 = CCVector2d(center.x, center.y);
    q2 = CCVector2d(perpendi.x, perpendi.y);


    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    Line_2 line(p1, p2);


    typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circular_arc_2>::type Intersection_result;
    std::vector<Intersection_result> res;

    auto& output_iterator = std::back_inserter(res);


    try
    {
        CGAL::intersection(line, arc, output_iterator);
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

CCVector2d cc2DArcLine::getArcMidpointByClockwise(const CCVector2d& p1, const CCVector2d& p2)
{
	CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p1, p2);
	CCVector2d mid;
	std::vector<CCVector2d> arcIntersSelect = lineArcIntersections(perpendi);
	//确认端点、交点、中点是顺时针的
	if (arcIntersSelect.size() == 1)
	{
		mid = arcIntersSelect[0];
	}
	else if (arcIntersSelect.size() > 1)
	{
		bool find = false;
		for (int i = 0; i < arcIntersSelect.size(); i++)
		{
			CCVector2d p = arcIntersSelect[i];
			Point_2 point1 = Point_2(p1.x, p1.y);
			Point_2 point2 = Point_2(p.x, p.y);
			Point_2 point3 = Point_2(p2.x, p2.y);
			if (CGAL::orientation(point1, point2, point3) == CGAL::CLOCKWISE)
			{
				mid = arcIntersSelect[i];
				find = true;
				break;
			}
		}
		if (!find)
			return CCVector2d();
	}
	else
		return CCVector2d();

	return mid;
}


CCVector2d cc2DArcLine::getArcMidpoint(const CCVector2d& p12d, const CCVector2d& p22d)
{
    // 计算连线中点坐标
	double centerX = (p12d.x + p22d.x) / 2.0;
    double centerY = (p12d.y + p22d.y) / 2.0;

    // 计算圆弧的弧度
    double chord = sqrt(pow((p22d.x - p12d.x), 2) + pow((p22d.y - p12d.y), 2));
    double sagitta = m_radius - sqrt(pow(m_radius, 2) - pow((chord / 2), 2));
    double theta = 2.0 * asin(chord / (2.0 * m_radius));
    if (abs(chord - 0) < EPSINON)
    {
        return CCVector2d();
    }

    // 根据顺时针方向，计算中点坐标
    CCVector2d arcCenter;
    arcCenter.x = centerX + (sagitta * (p12d.y - p22d.y)) / chord;
    arcCenter.y = centerY + (sagitta * (p22d.x - p12d.x)) / chord;
    return arcCenter;
}

CCVector2d cc2DArcLine::getArcMidPointByPerpendicular(const CCVector2d& p12d, const CCVector2d& p22d)
{
	CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p12d, p22d);
	std::vector<CCVector2d> arcIntersSelect = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);

	double chord = sqrt(pow((p22d.x - p12d.x), 2) + pow((p22d.y - p12d.y), 2));
	double theta = 2.0 * asin(chord / (2.0 * m_radius));
	double angle = qRadiansToDegrees(theta);

	if (arcIntersSelect.size() == 1)
	{
		return arcIntersSelect[0];
	}
	return CCVector2d();
}

std::vector<CCVector2d> sortPointDistance(std::vector<PointDistance>& lst)
{
    std::vector<CCVector2d> result;
    if (lst.size() <= 0)
    {
        return result;
    }
    QVector<PointDistance> lstPoint;
    foreach(PointDistance point, lst)
    {
        lstPoint.push_back(point);
    }
    qSort(lstPoint.begin(), lstPoint.end(), [&](const PointDistance &v1, const PointDistance &v2) {
        return v1.distance < v2.distance;
    });

    foreach(PointDistance point, lstPoint)
    {
        result.push_back(point.pos2d);
    }
    return result;
}

double distanceCCVector2d(CCVector2d p1, CCVector2d p2)
{
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

double angle(CCVector2d p1, CCVector2d p2, CCVector2d p3)
{
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p1.x;
    double dy2 = p3.y - p1.y;
    return acos((dx1*dx2 + dy1 * dy2) / (distanceCCVector2d(p1, p2)*distanceCCVector2d(p1, p3)));
}

double getAngleWith360(CCVector2d cen, CCVector2d first, CCVector2d second)
{
    //计算角度
    double ma_x = first.x - cen.x;
    double ma_y = first.y - cen.y;
    double mb_x = second.x - cen.x;
    double mb_y = second.y - cen.y;
    double v1 = (ma_x * mb_x) + (ma_y * mb_y);
    double ma_val = sqrt(ma_x * ma_x + ma_y * ma_y);
    double mb_val = sqrt(mb_x * mb_x + mb_y * mb_y);
    double cosM = v1 / (ma_val * mb_val);
    if (cosM < -1)
    {
        cosM = -1;
    }
    else if (cosM > 1)
    {
        cosM = 1;
    }
    double angleAMB = acos(cosM) * 180 / PI;

    //计算两条线向量的顺逆时针方向
    CCVector2d lineFirst = CCVector2d(first.x - cen.x, first.y - cen.y);
    CCVector2d lineSecond = CCVector2d(second.x - cen.x, second.y - cen.y);
    bool isClockwise = lineFirst.x * lineSecond.y - lineSecond.x * lineFirst.y < 0;
    if (isClockwise)
    {
        angleAMB = 360 - angleAMB;
    }
    return angleAMB;
}

std::vector<CCVector2d> cc2DArcLine::sortPointInArc(std::vector<CCVector2d>& lst, CCVector2d start, CCVector2d end)
{
    std::vector<CCVector2d> result;
    if (lst.size() <= 0)
    {
        return result;
    }

    std::vector<double> angles(lst.size());
    for (int i = 0; i < lst.size(); i++)
    {
        angles[i] = getAngleWith360(CCVector2d(m_center.x, m_center.y), start, lst[i]);

        if (isnan(angles[i]))
        {
            angles[i] = 0;
        }

    }
    std::vector<int> indices(lst.size());
    for (int i = 0; i < lst.size(); i++)
    {
        indices[i] = i;
    }
    qSort(indices.begin(), indices.end(), [&](int a, int b) {
        return angles[a] > angles[b];
    });

    std::vector<CCVector2d> sortedPoints(lst.size());

    for (int i = 0; i < lst.size(); i++)
    {
        sortedPoints[i] = lst[indices[i]];
    }

    return sortedPoints;
}

std::vector<CCVector3d> cc2DArcLine::getCenter3D()
{
	std::vector<CCVector3d> lst;
	lst.push_back(getCenter());

	std::vector<CCVector3d> result = unprojectTo3d(lst, m_3dCamera);
	return result;
}


void cc2DArcLine::drawIn2dView(CC_DRAW_CONTEXT& context)
{
    if (m_bTrim && m_mouse2dpos.size() == 0)
        return;

    if(m_keyPoints2d.size() == 0 )
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
    std::vector<CCVector3d> pts, drawPoints;
    for (int i = 0; i < m_keyPoints2d.size(); i++)
        pts.push_back(m_keyPoints2d[i]);

    if (pts.size() == 1 || pts.size() > 3)
    {
        return;
    }

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

    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(-halfW), static_cast<GLfloat>(-halfH), 0);


    if (pts.size() == 2)
        drawPoints = pts;
    else if (pts.size() == 3)
    {
		drawPoints = geometryUtils::getCircleArcSamplePoints(updir, 200, pts, m_center, m_radius, m_sweepAngle);
        if (drawPoints.size() > 0 && (isnan(drawPoints[0].x) || isnan(drawPoints[0].y) || isnan(drawPoints[0].z)))
        {
            drawPoints.clear();
            drawPoints = pts;
        }
    }
	if (m_bExtrendLineShow)
	{
		std::vector<CCVector3d> drawExtrendPoints;
		drawExtrendPoints = geometryUtils::getCircleArcSamplePoints(updir, 200, m_extrendLinePts, m_center, m_radius, m_sweepAngle);
		glFunc->glLineWidth(static_cast<GLfloat>(1.0));
		drawLines2D(context, drawExtrendPoints, ccColor::Rgbaf(1, 1, 1, 1), true);
	}
	glFunc->glLineWidth(static_cast<GLfloat>(m_width));


    double z;
    if (drawPoints.size() > 0)
    {
        z = drawPoints[0].z;
    }
    if (!m_bTrim)
    {
        glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
        if (drawPoints.size() > 0)
        {
			if (drawPoints.size() > 0 && pts.size() == 3)
				drawPoints.push_back(CCVector3d(pts[2].x, pts[2].y, z));
			drawLines2D(context, drawPoints, m_rgbColor);
        }
    }
    else if (m_bTrim && m_mouse2dpos.size() > 0 )
    {
        drawArcTrim(context, z);
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

void cc2DArcLine::removeNearEndpointIntersections(double z)
{
    CCVector3d start, end;
    if (m_keyPoints2d.size() > 0)
    {
        start = m_keyPoints2d[0];
        end = m_keyPoints2d[m_keyPoints2d.size() -1];

    }
    for (int i = m_lstIntersections.size() - 1; i>=0; i--)
    {
        CCVector3d p = m_lstIntersections[i];
        if (CCVector3d::vdistance(start.u, CCVector3d(p.x, p.y, z).u) < 10e-3 ||
            CCVector3d::vdistance(end.u, CCVector3d(p.x, p.y, z).u) < 10e-3)
        {
            std::vector<CCVector3d>::const_iterator it = std::find_if(m_lstIntersections.cbegin(), m_lstIntersections.cend(), [&](CCVector3d temp) {
                return abs(temp.norm() - p.norm()) < EPSINON;
            });
            if (it != m_lstIntersections.end())
            {
                m_lstIntersections.erase(it);
            }
        }
    }
}

void cc2DArcLine::drawArcTrim(CC_DRAW_CONTEXT& context, double z)
{
    m_pt2dUnselect2.clear();
    m_pt2dUnselect.clear();
    removeNearEndpointIntersections(z);
    int result = m_lstIntersections.size();
    if (result == 0)
    {
        drawArcNoIntersection(context, z);
    }
    else if (result == 1)
    {
        drawArcOnly1Intersection(context, z);
    }
    else if (result > 1)
    {
        drawArcMore1Intersection(context, z);
    }
    else
    {
        drawArcAbnormal(context);
    }
}

void cc2DArcLine::drawArcNoIntersection(CC_DRAW_CONTEXT& context, double z)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;
    CCVector3d updir(0, 0, 1);

    std::vector<CCVector3d> pts, drawPoints;
    for (int i = 0; i < m_keyPoints2d.size(); i++)
        pts.push_back(m_keyPoints2d[i]);

    drawPoints = geometryUtils::getCircleArcSamplePoints(updir, 200, pts, m_center, m_radius, m_sweepAngle);

    glFunc->glColor4f(1, 1, 1, 1);
    if (drawPoints.size() > 0)
    {
        glFunc->glBegin(GL_LINE_STRIP);
        for (int i = 0; i < drawPoints.size(); i++)
            glFunc->glVertex2f(drawPoints[i].x, drawPoints[i].y);
        glFunc->glVertex2f(pts[2].x, pts[2].y);
        glFunc->glEnd();
    }

}

std::vector<CCVector2d> cc2DArcLine::lineArcIntersections(CCVector2d perpendi)
{
	std::vector<CCVector2d> lstIntersection;
	std::vector<CCVector2d> intersections;

	if (m_keyPoints2d.size() == 3)
	{
		CCVector2d q1 = CCVector2d(m_keyPoints2d[0].x, m_keyPoints2d[0].y);
		CCVector2d q2 = CCVector2d(m_keyPoints2d[1].x, m_keyPoints2d[1].y);
		CCVector2d q3 = CCVector2d(m_keyPoints2d[2].x, m_keyPoints2d[2].y);
		Point_2 p1 = Point_2(q1.x, q1.y);
		Point_2 p2 = Point_2(q2.x, q2.y);
		Point_2 p3 = Point_2(q3.x, q3.y);
		if (CGAL::orientation(p1, p2, p3) != CGAL::CLOCKWISE)
			std::swap(p1, p3);
		Circle_2 circle(p1, p2, p3);

		q1 = CCVector2d(perpendi.x, perpendi.y);
		q2 = CCVector2d(m_center.x, m_center.y);
		p1 = Point_2(q1.x, q1.y);
		p2 = Point_2(q2.x, q2.y);
		Line_2 line(p1, p2);
		Segment_2 s(p1, p2);


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
			}
		}
	}
	return intersections;
}

bool cc2DArcLine::isClockwise(CCVector2d p1, CCVector2d p2)
{
	CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p1, p2);
	CCVector2d mid;
	std::vector<CCVector2d> arcIntersSelect = lineArcIntersections(perpendi);
	//确认端点、交点、中点是顺时针的
	if (arcIntersSelect.size() == 1)
	{
		mid = arcIntersSelect[0];
	}
	else if (arcIntersSelect.size() > 1)
	{
		bool find = false;
		for (int i = 0; i < arcIntersSelect.size(); i++)
		{
			CCVector2d p = arcIntersSelect[i];
			//if (segmentIntersect(p1, p2, CCVector2d(m_center.x, m_center.y), perpendi))
			Point_2 point1 = Point_2(p1.x, p1.y);
			Point_2 point2 = Point_2(p.x, p.y);
			Point_2 point3 = Point_2(p2.x, p2.y);
			if (CGAL::orientation(point1, point2, point3) == CGAL::CLOCKWISE)
			{
				mid = arcIntersSelect[i];
				find = true;
				break;
			}
		}
		if(!find)
			return false;
	}
	else
		return false;


	Point_2 point1 = Point_2(p1.x, p1.y);
	Point_2 point2 = Point_2(mid.x, mid.y);
	Point_2 point3 = Point_2(p2.x, p2.y);
	if (CGAL::orientation(point1, point2, point3) != CGAL::CLOCKWISE)
		return false;
	else
		return true;

}


void cc2DArcLine::drawArcOnly1Intersection(CC_DRAW_CONTEXT& context, double z)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    CCVector3d updir(0, 0, 1);
    CCVector3d p13d = m_lstIntersections[0];
    CCVector2d p12d = CCVector2d(p13d.x, p13d.y);
    CCVector2d mouse2d = CCVector2d(m_mouse2dpos[0].x, m_mouse2dpos[0].y);

	CCVector2d keyPointHead = CCVector2d(m_keyPoints2d[0].x, m_keyPoints2d[0].y);
	CCVector2d keyPointTail = CCVector2d(m_keyPoints2d[m_keyPoints2d.size() - 1].x, m_keyPoints2d[m_keyPoints2d.size() - 1].y);

	Point_2 point1 = Point_2(keyPointHead.x, keyPointHead.y);
	Point_2 point2 = Point_2(m_lstIntersections[0].x, m_lstIntersections[0].y);
	Point_2 point3 = Point_2(keyPointTail.x, keyPointTail.y);
	bool clockwise = true;
	if (CGAL::orientation(point1, point2, point3) != CGAL::CLOCKWISE)
	{
		std::swap(point1, point3);
		keyPointHead = CCVector2d(CGAL::to_double(point1.x()), CGAL::to_double(point1.y()));
		keyPointTail = CCVector2d(CGAL::to_double(point3.x()), CGAL::to_double(point3.y()));
		clockwise = false;
	}

    CCVector3d select;
    CCVector3d unselect;
    bool valid = false;

	CCVector2d midpointL = isClockwise(keyPointHead, p12d) ? getArcMidpointByClockwise(keyPointHead, p12d) : getArcMidpointByClockwise(p12d, keyPointHead);
	CCVector2d midpointR = isClockwise(p12d, keyPointTail) ? getArcMidpointByClockwise(p12d, keyPointTail) : getArcMidpointByClockwise(keyPointTail, p12d);

    if (segmentIntersect(midpointL, keyPointHead, mouse2d, CCVector2d(m_center.x, m_center.y))
        || segmentIntersect(p12d, midpointL, mouse2d, CCVector2d(m_center.x, m_center.y)))
    {
        select = CCVector3d(keyPointHead.x, keyPointHead.y, z);
        unselect = CCVector3d(keyPointTail.x, keyPointTail.y, z);
        valid = true;
    }
    else if (segmentIntersect(midpointR, keyPointTail, mouse2d, CCVector2d(m_center.x, m_center.y))
        || segmentIntersect(p12d, midpointR, mouse2d, CCVector2d(m_center.x, m_center.y)))
    {
        select = CCVector3d(keyPointTail.x, keyPointTail.y, z);
        unselect = CCVector3d(keyPointHead.x, keyPointHead.y, z);
        valid = true;
    }
    if (!valid)
    {
        drawArcAbnormal(context);
        return;
    }
    std::vector<CCVector3d> drawPointsSelect, drawPointsUnselect;
    std::vector<CCVector3d> ptSelect;

    CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p12d, CCVector2d(select.x, select.y));
    std::vector<CCVector2d> arcIntersSelect = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
    if (arcIntersSelect.size() > 0)
    {
        int i = 0;
        bool bCycle = false;
        for (; i < arcIntersSelect.size(); i++)
        {
            CCVector2d p = arcIntersSelect[i];
            if (segmentIntersect(p12d, p, CCVector2d(m_center.x, m_center.y), mouse2d) ||
                segmentIntersect(CCVector2d(select.x, select.y), p, CCVector2d(m_center.x, m_center.y), mouse2d))
            {
                bCycle = true;
                break;
            }
        }
        if (i>=arcIntersSelect.size() || !bCycle)
        {
            drawArcAbnormal(context);
            return;
        }

        ptSelect.push_back(select);
        ptSelect.push_back(CCVector3d(arcIntersSelect[i].x, arcIntersSelect[i].y, z));
        ptSelect.push_back(CCVector3d(p12d.x, p12d.y, z));

        drawPointsSelect = geometryUtils::getCircleArcSamplePoints(updir, 200, ptSelect, m_center, m_radius, m_sweepAngle);
        drawPointsSelect.push_back(CCVector3d(p12d.x, p12d.y, z));
        glFunc->glColor4f(1, 1, 1, 1);
        if (drawPointsSelect.size() > 0)
        {
            glFunc->glBegin(GL_LINE_STRIP);
            for (int j = 0; j < drawPointsSelect.size(); j++)
                glFunc->glVertex2f(drawPointsSelect[j].x, drawPointsSelect[j].y);
            glFunc->glEnd();
        }
        else
        {
            drawArcAbnormal(context);
            return;
        }
    }
    else
    {
        drawArcAbnormal(context);
        return;
    }


    perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p12d, CCVector2d(unselect.x, unselect.y));
    std::vector<CCVector2d> arcIntersUnselect = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
    if (arcIntersUnselect.size() > 0)
    {
        int i = 0;
        bool bCycle = false;

        for (; i < arcIntersUnselect.size(); i++)
        {
            CCVector2d p = arcIntersUnselect[i];

            if (!(segmentIntersect(p12d, p, CCVector2d(m_center.x, m_center.y), mouse2d) ||
                segmentIntersect(CCVector2d(unselect.x, unselect.y), p, CCVector2d(m_center.x, m_center.y), mouse2d)))
            {
                bCycle = true;
                break;
            }
        }
        if (i >= arcIntersUnselect.size() || !bCycle)
        {
            drawArcAbnormal(context);
            return;
        }

        m_pt2dUnselect.push_back(CCVector3d(p12d.x, p12d.y, z));
        m_pt2dUnselect.push_back(CCVector3d(arcIntersUnselect[i].x, arcIntersUnselect[i].y, z));
        m_pt2dUnselect.push_back(unselect);

        drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, m_center, m_radius, m_sweepAngle);
        m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
        if (drawPointsUnselect.size() > 0)
        {
            glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
            if (drawPointsUnselect.size() > 0)
            {
                glFunc->glBegin(GL_LINE_STRIP);
                for (int j = 0; j < drawPointsUnselect.size(); j++)
                    glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
                glFunc->glEnd();
            }
        }
        else
        {
            drawArcAbnormal(context);
            return;
        }
    }
    else
    {
        drawArcAbnormal(context);
        return;
    }
}

void cc2DArcLine::drawArcMore1Intersection(CC_DRAW_CONTEXT& context, double z)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    CCVector3d updir(0, 0, 1);

    CCVector2d mouse2d = CCVector2d(m_mouse2dpos[0].x, m_mouse2dpos[0].y);
    CCVector2d keyPointHead = CCVector2d(m_keyPoints2d[0].x, m_keyPoints2d[0].y);
    CCVector2d keyPointTail = CCVector2d(m_keyPoints2d[m_keyPoints2d.size() - 1].x, m_keyPoints2d[m_keyPoints2d.size() - 1].y);
    if (isnan(mouse2d.x) || isnan(m_center.x))
    {
        drawArcAbnormal(context);
        return;
    }
    std::vector<CCVector2d> lst;
    for (int i = 0; i < m_lstIntersections.size(); i++)
    {
        CCVector3d p = m_lstIntersections[i];
        lst.push_back(CCVector2d(p.x, p.y));
    }
    Point_2 point1 = Point_2(keyPointHead.x, keyPointHead.y);
    Point_2 point2 = Point_2(m_lstIntersections[0].x, m_lstIntersections[0].y);
    Point_2 point3 = Point_2(keyPointTail.x, keyPointTail.y);

	if (CGAL::orientation(point1, point2, point3) != CGAL::CLOCKWISE)
	{
		std::swap(point1, point3);
		keyPointHead = CCVector2d(CGAL::to_double(point1.x()), CGAL::to_double(point1.y()));
		keyPointTail = CCVector2d(CGAL::to_double(point3.x()), CGAL::to_double(point3.y()));
	}


    std::vector<CCVector2d> lstResult = sortPointInArc(lst, keyPointHead, keyPointTail);
    lstResult.insert(lstResult.begin(), keyPointHead);
    lstResult.push_back(keyPointTail);

    bool find = false;
    for (int i = 0; i < lstResult.size() - 1; i++)
    {
        CCVector2d p1 = lstResult[i];
        CCVector2d p2 = lstResult[i + 1];

        CCVector2d mouse2d = CCVector2d(m_mouse2dpos[0].x, m_mouse2dpos[0].y);
        CCVector2d head = CCVector2d(keyPointHead.x, keyPointHead.y);
        CCVector2d tail = CCVector2d(keyPointTail.x, keyPointTail.y);

        CCVector3d select;
        CCVector3d unselect;

		CCVector2d midpoint = isClockwise(p1, p2) ? getArcMidpointByClockwise(p1, p2) : getArcMidpointByClockwise(p2, p1);
        if (isnan(midpoint.x) || isnan(midpoint.y))
        {
            continue;
        }
        if (segmentIntersect(midpoint, p1, mouse2d, CCVector2d(m_center.x, m_center.y))
            || segmentIntersect(p2, midpoint, mouse2d, CCVector2d(m_center.x, m_center.y))
            /*|| segmentIntersect(p1, p2, mouse2d, CCVector2d(m_center.x, m_center.y))*/)
        {
            find = true;
        }
        else
        {
            find = false;
            continue;
        }

        std::vector<CCVector3d> drawPointsSelect, drawPointsUnselect;
        std::vector<CCVector3d> ptSelect;

        CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p1, p2);

        std::vector<CCVector2d> arcIntersSelect = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
        if (arcIntersSelect.size() > 0 && (isnan(arcIntersSelect[0].x) || isnan(arcIntersSelect[arcIntersSelect.size() - 1].x)))
        {
            find = false;
            continue;
        }
        if (arcIntersSelect.size() > 0)
        {
            int m = 0;
            bool bCycle = false;
            for (; m < arcIntersSelect.size(); m++)
            {
                CCVector2d p = arcIntersSelect[m];
                if (segmentIntersect(p1, p, CCVector2d(m_center.x, m_center.y), mouse2d) ||
                    segmentIntersect(p2, p, CCVector2d(m_center.x, m_center.y), mouse2d))
                //if (segmentIntersect(p1, p2, CCVector2d(m_center.x, m_center.y), p))
                {
                    bCycle = true;
                    break;
                }
            }
            if (m > arcIntersSelect.size()-1 || !bCycle)
            {
                find = false;
                continue;
            }
            ptSelect.push_back(CCVector3d(p1.x, p1.y, z));
            ptSelect.push_back(CCVector3d(arcIntersSelect[m].x, arcIntersSelect[m].y, z));
            ptSelect.push_back(CCVector3d(p2.x, p2.y, z));

            drawPointsSelect = geometryUtils::getCircleArcSamplePoints(updir, 200, ptSelect, m_center, m_radius, m_sweepAngle);
            drawPointsSelect.insert(drawPointsSelect.begin(), CCVector3d(p1.x, p1.y, z));
            drawPointsSelect.push_back(CCVector3d(p2.x, p2.y, z));
            glFunc->glColor4f(1, 1, 1, 1);
            if (drawPointsSelect.size() > 0 )
            {
                glFunc->glBegin(GL_LINE_STRIP);
                for (int j = 0; j < drawPointsSelect.size(); j++)
                    glFunc->glVertex2f(drawPointsSelect[j].x, drawPointsSelect[j].y);
                glFunc->glEnd();
            }
            else
            {
                find = false;
                continue;
            }


            if (i == 0)
            {
                CCVector2d result = getActualArcMousePoint(p2, tail, mouse2d);
                m_pt2dUnselect.push_back(CCVector3d(p2.x, p2.y, z));
                m_pt2dUnselect.push_back(CCVector3d(result.x, result.y, z));
                m_pt2dUnselect.push_back(CCVector3d(tail.x, tail.y, z));

                drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, m_center, m_radius, m_sweepAngle);
                drawPointsUnselect.insert(drawPointsUnselect.begin(), drawPointsSelect[drawPointsSelect.size() - 1]);
                m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                if (drawPointsUnselect.size() > 0)
                {
                    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
                    glFunc->glBegin(GL_LINE_STRIP);
                    for (int j = 0; j < drawPointsUnselect.size(); j++)
                        glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
                    glFunc->glEnd();
                }
                break;
            }
            else if (i == lstResult.size() - 2)
            {
                CCVector2d result = getActualArcMousePoint(head, p1, mouse2d);
                m_pt2dUnselect.push_back(CCVector3d(head.x, head.y, z));
                m_pt2dUnselect.push_back(CCVector3d(result.x, result.y, z));
                m_pt2dUnselect.push_back(CCVector3d(p1.x, p1.y, z));

                drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, m_center, m_radius, m_sweepAngle);
                drawPointsUnselect.push_back(drawPointsSelect[0]);
                m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                if (drawPointsUnselect.size() > 0)
                {
                    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
                    glFunc->glBegin(GL_LINE_STRIP);
                    for (int j = 0; j < drawPointsUnselect.size(); j++)
                        glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
                    glFunc->glEnd();
                }
                break;
            }
            else
            {
                CCVector2d result = getActualArcMousePoint(head, p1, mouse2d);

                m_pt2dUnselect.push_back(CCVector3d(head.x, head.y, z));
                m_pt2dUnselect.push_back(CCVector3d(result.x, result.y, z));
                m_pt2dUnselect.push_back(CCVector3d(p1.x, p1.y, z));

                drawPointsUnselect = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect, m_center, m_radius, m_sweepAngle);
                m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                if (drawPointsUnselect.size() > 0)
                {
                    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
                    glFunc->glBegin(GL_LINE_STRIP);
                    for (int j = 0; j < drawPointsUnselect.size(); j++)
                        glFunc->glVertex2f(drawPointsUnselect[j].x, drawPointsUnselect[j].y);
                    glFunc->glEnd();
                }


                result = getActualArcMousePoint(p2, tail, mouse2d);
                std::vector<CCVector3d> drawPointsUnselect2;
                
                m_pt2dUnselect2.push_back(CCVector3d(p2.x, p2.y, z));
                m_pt2dUnselect2.push_back(CCVector3d(result.x, result.y, z));
                m_pt2dUnselect2.push_back(CCVector3d(tail.x, tail.y, z));

                drawPointsUnselect2 = geometryUtils::getCircleArcSamplePoints(updir, 200, m_pt2dUnselect2, m_center, m_radius, m_sweepAngle);
                drawPointsUnselect2.insert(drawPointsUnselect2.begin(), drawPointsSelect[drawPointsSelect.size() -1]);
                m_rgbColor = ccColor::Rgbaf(1, 0, 0, 1);
                if (drawPointsUnselect2.size() > 0)
                {
                    glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
                    glFunc->glBegin(GL_LINE_STRIP);
                    for (int j = 0; j < drawPointsUnselect2.size(); j++)
                        glFunc->glVertex2f(drawPointsUnselect2[j].x, drawPointsUnselect2[j].y);
                    glFunc->glEnd();
                }
                break;
            }
        }
        else 
        {
            find = false;
            continue;
        }
        if (find)
        {
            break;
        }
    }
    if (!find)
    {
        drawArcAbnormal(context);
    }
}

void cc2DArcLine::drawArcAbnormal(CC_DRAW_CONTEXT& context)
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

    drawPoints = geometryUtils::getCircleArcSamplePoints(updir, 200, pts, m_center, m_radius, m_sweepAngle);
    if (drawPoints.size() > 0)
    {
        glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);
        glFunc->glBegin(GL_LINE_STRIP);
        for (int i = 0; i < drawPoints.size(); i++)
            glFunc->glVertex2f(drawPoints[i].x, drawPoints[i].y);
        if (drawPoints.size() > 0 && pts.size() == 3)
            glFunc->glVertex2f(pts[2].x, pts[2].y);
        glFunc->glEnd();
    }

}

CCVector2d cc2DArcLine::getActualArcMousePoint(const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& mouse2d)
{
    //垂直点
    CCVector2d perpendi = getPerpendicularPoint(CCVector2d(m_center.x, m_center.y), p1, p2);

    std::vector<CCVector2d> arcInters = centerIntersectArc(CCVector2d(m_center.x, m_center.y), perpendi);
    if (arcInters.size() > 0)
    {
        int i = 0;
        for (; i < arcInters.size(); i++)
        {
            CCVector2d p = arcInters[i];

            if (!(segmentIntersect(p1, p, CCVector2d(m_center.x, m_center.y), mouse2d) ||
                segmentIntersect(p2, p, CCVector2d(m_center.x, m_center.y), mouse2d)))
            {
                return arcInters[i];
            }
        }
        return CCVector2d();
    }
    return CCVector2d();

}

void cc2DArcLine::drawIn3dView(CC_DRAW_CONTEXT& context)
{
    if (context.display->windowName().compare("3D View") != 0)
        return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    assert(glFunc != nullptr);
    if (glFunc == nullptr)
        return;

    getdrawPoints3d();
    if (m_drawPoints3d.size() <= 0)
    {
        get3dPoints();
    }
    if (m_drawPoints3d.size() <= 0)
    {
        return;
    }

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

    draw_circleArc_in3d_view(context);

    if (m_width != 0)
    {
        glFunc->glDisable(GL_BLEND);
        glFunc->glDisable(GL_LINE_SMOOTH);
        glFunc->glPopAttrib();
    }
    if (pushName)
        glFunc->glPopName();
}

void cc2DArcLine::draw_circleArc_in3d_view(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (!glFunc)
        return;


    glFunc->glBegin(GL_LINE_STRIP);
    for (int i = 0; i < m_drawPoints3d.size(); i++)
        glFunc->glVertex3f(m_drawPoints3d[i].x, m_drawPoints3d[i].y, m_drawPoints3d[i].z);
    if (m_threeArcPoints.size() > 0)
        glFunc->glVertex3f(m_threeArcPoints[2].x, m_threeArcPoints[2].y, m_threeArcPoints[2].z);
    glFunc->glEnd();
}

bool cc2DArcLine::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[cc2DArcLine::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
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

bool cc2DArcLine::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
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

bool cc2DArcLine::split(	PointCoordinateType maxEdgeLength,
						std::vector<cc2DArcLine*>& parts)
{
	parts.clear();

	//not enough vertices?
	unsigned vertCount = size();
	if (vertCount <= 2)
	{
		parts.push_back(new cc2DArcLine(*this));
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
					parts.push_back(new cc2DArcLine(*this));
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
				parts.push_back(new cc2DArcLine(*this));
				return true;
			}
		}

		if (partSize > 1) //otherwise we skip that point
		{
			//create the corresponding part
			CCCoreLib::ReferenceCloud ref(m_theAssociatedCloud);
			if (!ref.reserve(partSize))
			{
				ccLog::Error("[cc2DArcLine::split] Not enough memory!");
				return false;
			}

			for (unsigned i=startIndex; i<=stopIndex; ++i)
			{
				ref.addPointIndex(i % vertCount);
			}

			ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
			ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
			//cc2DArcLine* part = new cc2DArcLine(subset);
			//part->initWith(subset, *this);
			//part->setClosed(false); //by definition!
			//parts.push_back(part);
		}

		//forward
		startIndex = (stopIndex % vertCount) + 1;
	}

	return true;
}

PointCoordinateType cc2DArcLine::computeLength() const
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

unsigned cc2DArcLine::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
}

unsigned cc2DArcLine::segmentCount() const
{
	unsigned count = size();
	if (count && !isClosed())
	{
		--count;
	}
	return count;
}

void cc2DArcLine::setGlobalShift(const CCVector3d& shift)
{
	ccShiftedObject::setGlobalShift(shift);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global shift info to the vertices
		pc->setGlobalShift(shift);
	}
}

void cc2DArcLine::setGlobalScale(double scale)
{
	ccShiftedObject::setGlobalScale(scale);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global scale info to the vertices
		pc->setGlobalScale(scale);
	}
}

const CCVector3d& cc2DArcLine::getGlobalShift() const
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

double cc2DArcLine::getGlobalScale() const
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

ccPointCloud* cc2DArcLine::samplePoints(	bool densityBased,
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
		ccLog::Warning("[cc2DArcLine::samplePoints] Not enough memory");
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

cc2DArcLine* cc2DArcLine::smoothChaikin(PointCoordinateType ratio, unsigned iterationCount) const
{
	if (iterationCount == 0)
	{
		assert(false);
		ccLog::Warning("[cc2DArcLine::smoothChaikin] Invalid input (iteration count)");
		return nullptr;
	}

	if (ratio < 0.05f || ratio > 0.45f)
	{
		assert(false);
		ccLog::Warning("[cc2DArcLine::smoothChaikin] invalid ratio");
		return nullptr;
	}

	if (size() < 3)
	{
		ccLog::Warning("[cc2DArcLine::smoothChaikin] not enough segments");
		return nullptr;
	}

	const CCCoreLib::GenericIndexedCloudPersist* currentIterationVertices = this; //a polyline is actually a ReferenceCloud!
	cc2DArcLine* smoothPoly = nullptr;

	bool openPoly = !isClosed();

	for (unsigned it = 0; it < iterationCount; ++it)
	{
		//reserve memory for the new vertices
		unsigned vertCount = currentIterationVertices->size();
		unsigned segmentCount = (openPoly ? vertCount - 1 : vertCount);

		ccPointCloud* newStateVertices = new ccPointCloud("vertices");
		if (!newStateVertices->reserve(segmentCount * 2))
		{
			ccLog::Warning("[cc2DArcLine::smoothChaikin] not enough memory");
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
			//smoothPoly = new cc2DArcLine(newStateVertices);
			//smoothPoly->addChild(newStateVertices);
			//newStateVertices->setEnabled(false);
			//if (!smoothPoly->reserve(newStateVertices->size()))
			//{
			//	ccLog::Warning("[cc2DArcLine::smoothChaikin] not enough memory");
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

bool cc2DArcLine::IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, cc2DArcLine** polyline/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a polyline
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DArcLine*>(parent)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<cc2DArcLine*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::POLY_LINE) && static_cast<cc2DArcLine*>(child)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<cc2DArcLine*>(child);
			}
			return true;
		}
	}

	return false;
}

bool cc2DArcLine::createNewPolylinesFromSelection(std::vector<cc2DArcLine*>& output)
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
		ccLog::Warning("[cc2DArcLine::createNewPolylinesFromSelection] Unsupported vertex cloud");
		return false;
	}
	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = verticesCloud->getTheVisibilityArray();
	if (verticesVisibility.size() < vertCount)
	{
		// no visibility table instantiated
		ccLog::Warning("[cc2DArcLine::createNewPolylinesFromSelection] No visibility table instantiated");
		return false;
	}

	bool success = true;
	{
		cc2DArcLine* chunkPoly = nullptr;
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
					//chunkPoly = new cc2DArcLine(chunkCloud);
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
		ccLog::Warning("[cc2DArcLine::createNewPolylinesFromSelection] Not enough memory");
		// delete the already created polylines
		for (cc2DArcLine* poly : output)
		{
			delete poly;
		}
		output.clear();
	}

	return success;
}

std::vector<CCVector2d> cc2DArcLine::intersect(ccLines* pItem, bool filter)
{
    std::vector<CCVector2d> lstIntersection;

    for (int i = 0; i < pItem->get2dKeyPoints().size() - 1; i++)
    {
		std::vector<CCVector2d> intersections;

		CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[0].x, this->get2dKeyPoints()[0].y);
		CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[1].x, this->get2dKeyPoints()[1].y);
        CCVector2d q3 = CCVector2d(this->get2dKeyPoints()[2].x, this->get2dKeyPoints()[2].y);

		Point_2 p1 = Point_2(q1.x, q1.y);
		Point_2 p2 = Point_2(q2.x, q2.y);
        Point_2 p3 = Point_2(q3.x, q3.y);

        if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
            std::swap(p1, p3);

		CCVector2d q4 = CCVector2d(pItem->get2dKeyPoints()[i].x, pItem->get2dKeyPoints()[i].y);
		CCVector2d q5 = CCVector2d(pItem->get2dKeyPoints()[i + 1].x, pItem->get2dKeyPoints()[i + 1].y);
        if (q4 == q5)
            continue;

		Point_2 p4 = Point_2(q4.x, q4.y);
		Point_2 p5 = Point_2(q5.x, q5.y);
		Line_2 line(p4, p5);
		Segment_2 s(p4, p5);

		if (filter)
		{
			typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circle_2>::type Intersection_result;
			std::vector<Intersection_result> res;

			auto& output_iterator = std::back_inserter(res);

			try
			{
				Circle_2 circle(p1, p2, p3);
				CGAL::intersection(line, circle, output_iterator);
			}
			catch (...)
			{
				qDebug() << "The intersection calculation failed.";
				return lstIntersection;
			}

			if (res.size() <= 0)
				continue;
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
				}
			}

			removePointsOnArc(intersections);
			removePointsNotOnLine(intersections, CCVector3d(q4.x, q4.y, 0), CCVector3d(q5.x, q5.y, 0));
		}
		else
		{
			typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circular_arc_2>::type Intersection_result;
			std::vector<Intersection_result> res;

			Circular_arc_2 arc;
			try
			{
				arc = Circular_arc_2(p1, p2, p3);
				CGAL::intersection(line, arc, std::back_inserter(res));
			}
			catch (...)
			{
				qDebug() << "The intersection calculation failed.";
				return intersections;
			}

			if (res.size() <= 0)
				continue;
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
				}
			}
			removePointsNotOnLine(intersections, CCVector3d(q4.x, q4.y, 0), CCVector3d(q5.x, q5.y, 0));
		}
        
        foreach(CCVector2d intersection, intersections)
            lstIntersection.push_back(intersection);
    }
    return lstIntersection;
}

std::vector<CCVector2d> cc2DArcLine::intersect(cc2DArcLine* pItem, bool filter)
{
    std::vector<CCVector2d> intersections;
    CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[0].x, this->get2dKeyPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[1].x, this->get2dKeyPoints()[1].y);
    CCVector2d q3 = CCVector2d(this->get2dKeyPoints()[2].x, this->get2dKeyPoints()[2].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);
	Circle_2 circle;
	Circular_arc_2 arc1;

    try
    {
		circle = Circle_2(p1, p2, p3);
		arc1 = Circular_arc_2(p1, p2, p3);
    }
    catch (...)
    {
        return intersections;
        qDebug() << "COUNTERCLOCKWISE faided!" << "\n";
    }

    q1 = CCVector2d(pItem->get2dKeyPoints()[0].x, pItem->get2dKeyPoints()[0].y);
    q2 = CCVector2d(pItem->get2dKeyPoints()[1].x, pItem->get2dKeyPoints()[1].y);
    q3 = CCVector2d(pItem->get2dKeyPoints()[2].x, pItem->get2dKeyPoints()[2].y);

    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

	if (filter)
	{
		typedef typename CGAL::CK2_Intersection_traits<K, Circular_arc_2, Circle_2>::type Intersection_result;
		std::vector<Intersection_result> res;
		auto& output_iterator = std::back_inserter(res);
		using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;


		Circular_arc_2 arc(p1, p2, p3);
		try
		{
			CGAL::intersection(arc, circle, output_iterator);
			if (res.size() <= 0)
				return intersections;

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
				}
			}
		}
		catch (...)
		{
			qDebug() << "The intersection calculation failed.";
			return intersections;
		}
		removePointsOnArc(intersections);
	}
	else
	{
		typedef typename CGAL::CK2_Intersection_traits<K, Circular_arc_2, Circular_arc_2>::type Intersection_result;
		std::vector<Intersection_result> res;
		using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;

		Circular_arc_2 arc2;
		try
		{
			arc2 = Circular_arc_2(p1, p2, p3);
			CGAL::intersection(arc1, arc2, std::back_inserter(res));

			if (res.size() <= 0)
			{
				return intersections;
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
				}
			}
		}
		catch (...)
		{
			qDebug() << "The intersection calculation failed.";
			return intersections;
		}
		
	}
    return intersections;
}

std::vector<CCVector2d> cc2DArcLine::intersect(cc2DRect* pItem, bool filter)
{
    using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > >, unsigned>;

    std::vector<CCVector2d> lstIntersection;
    CCVector2d q1 = CCVector2d(this->get2dKeyPoints()[0].x, this->get2dKeyPoints()[0].y);
    CCVector2d q2 = CCVector2d(this->get2dKeyPoints()[1].x, this->get2dKeyPoints()[1].y);
    CCVector2d q3 = CCVector2d(this->get2dKeyPoints()[2].x, this->get2dKeyPoints()[2].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);

    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

	std::vector<CCVector3d> lst;
	lst = pItem->getDrawPoints();
	lst.push_back(lst[0]);

	if (filter)
	{
		Circle_2 circle;
		try
		{
			circle = Circle_2(p1, p2, p3);
		}
		catch (...)
		{
			qDebug() << "COUNTERCLOCKWISE faided!" << "\n";
			return lstIntersection;
		}
		for (int i = 0; i < lst.size() - 1; i++)
		{
			std::vector<CCVector2d> intersections;
			CCVector2d q1 = CCVector2d(lst[i].x, lst[i].y);
			CCVector2d q2 = CCVector2d(lst[i + 1].x, lst[i + 1].y);

			Point_2 p1 = Point_2(q1.x, q1.y);
			Point_2 p2 = Point_2(q2.x, q2.y);
			Line_2 line(p1, p2);
			Segment_2 s(p1, p2);

			typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circle_2>::type Intersection_result;
			std::vector<Intersection_result> res;

			try
			{
				CGAL::intersection(line, circle, std::back_inserter(res));
			}
			catch (...)
			{
				qDebug() << "The intersection calculation failed.";
				return intersections;
			}
			if (res.size() <= 0)
				continue;

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
				}
			}
			removePointsOnArc(intersections);
			removePointsNotOnLine(intersections, CCVector3d(q1.x, q1.y, 0), CCVector3d(q2.x, q2.y, 0));


			foreach(CCVector2d intersection, intersections)
				lstIntersection.push_back(intersection);
		}
	}
	else
	{
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

		for (int i = 0; i < lst.size() - 1; i++)
		{
			std::vector<CCVector2d> intersections;
			CCVector2d q1 = CCVector2d(lst[i].x, lst[i].y);
			CCVector2d q2 = CCVector2d(lst[i + 1].x, lst[i + 1].y);

			Point_2 p1 = Point_2(q1.x, q1.y);
			Point_2 p2 = Point_2(q2.x, q2.y);
			Line_2 line(p1, p2);
			Segment_2 s(p1, p2);

			typedef typename CGAL::CK2_Intersection_traits<K, Line_2, Circular_arc_2>::type Intersection_result;
			std::vector<Intersection_result> res;

			try
			{
				CGAL::intersection(line, arc, std::back_inserter(res));
			}
			catch (...)
			{
				qDebug() << "The intersection calculation failed.";
				return intersections;
			}
			if (res.size() <= 0)
				continue;

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
				}
			}

			//for (int j = intersections.size() - 1; j >= 0; j--)
			//{
			//	CCVector2d pos = intersections[j];
			//	Point_2 p(pos.x, pos.y);

			//	if (is_point_on_segment(p, s))
			//		std::cout << "The point is on the segment." << std::endl;
			//	else
			//	{
			//		std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp)
			//		{
			//			return abs(temp.norm() - pos.norm()) < EPSINON;
			//		});
			//		if (it != intersections.end())
			//			intersections.erase(it);
			//	}
			//}
			removePointsNotOnLine(intersections, CCVector3d(q1.x, q1.y, 0), CCVector3d(q2.x, q2.y, 0));

			foreach(CCVector2d intersection, intersections)
				lstIntersection.push_back(intersection);
		}
	}



    return lstIntersection;
}

std::vector<CCVector2d> cc2DArcLine::intersect(cc2DRound* pItem, bool filter)
{
    std::vector<CCVector2d> intersections;
    if (pItem->getDrawPoints().size() <= 0)
        return intersections;

    CCVector2d q1 = CCVector2d(pItem->getDrawPoints()[0].x, pItem->getDrawPoints()[0].y);
    CCVector2d q2 = CCVector2d(pItem->getDrawPoints()[100].x, pItem->getDrawPoints()[100].y);
    CCVector2d q3 = CCVector2d(pItem->getDrawPoints()[150].x, pItem->getDrawPoints()[150].y);

    Point_2 p1 = Point_2(q1.x, q1.y);
    Point_2 p2 = Point_2(q2.x, q2.y);
    Point_2 p3 = Point_2(q3.x, q3.y);
    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

    Circle_2 circle1(p1, p2, p3);

    q1 = CCVector2d(this->get2dKeyPoints()[0].x, this->get2dKeyPoints()[0].y);
    q2 = CCVector2d(this->get2dKeyPoints()[1].x, this->get2dKeyPoints()[1].y);
    q3 = CCVector2d(this->get2dKeyPoints()[2].x, this->get2dKeyPoints()[2].y);

    p1 = Point_2(q1.x, q1.y);
    p2 = Point_2(q2.x, q2.y);
    p3 = Point_2(q3.x, q3.y);

    if (CGAL::orientation(p1, p2, p3) != CGAL::COUNTERCLOCKWISE)
        std::swap(p1, p3);

	if (filter)
	{
		typedef typename CGAL::CK2_Intersection_traits<K, Circle_2, Circle_2>::type Intersection_result;
		std::vector<Intersection_result> res;
		auto& output_iterator = std::back_inserter(res);

		Circle_2 circle(p1, p2, p3);
		try
		{
			CGAL::intersection(circle1, circle, output_iterator);
		}
		catch (...)
		{
			qDebug() << "The intersection calculation failed.";
			return intersections;
		}

		if (res.size() <= 0)
			return intersections;
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
			}
		}
		removePointsOnArc(intersections);
	}
	else
	{
		typedef typename CGAL::CK2_Intersection_traits<K, Circular_arc_2, Circle_2>::type Intersection_result;
		std::vector<Intersection_result> res;
		Circular_arc_2 arc;
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
				std::cout << point << std::endl;

				double x = CGAL::to_double(point.x());
				double y = CGAL::to_double(point.y());
				CCVector2d pos(x, y);
				intersections.push_back(pos);
			}
		}
	}
    return intersections;
}

void cc2DArcLine::removePointsNotOnLine(std::vector<CCVector2d>& lst, const CCVector3d& p1, const CCVector3d& p2)
{
	for (int i = lst.size() - 1; i >= 0; i--)
	{
		CCVector2d pos = lst[i];

		if (on_segment(CCVector3d(pos.x, pos.y, 0), p1, p2))
		{
			std::cout << "The point is on the segment." << std::endl;
		}
		else
		{
			std::vector<CCVector2d>::const_iterator it = std::find_if(lst.cbegin(), lst.cend(), [&](CCVector2d temp) {
				return abs(temp.norm() - pos.norm()) < EPSINON;
			});
			if (it != lst.end())
			{
				lst.erase(it);
			}
		}
	}
}

void cc2DArcLine::removePointsOnArc(std::vector<CCVector2d>& lst)
{
	CCVector2d start = CCVector2d(m_keyPoints2d[0].x, m_keyPoints2d[0].y);
	CCVector2d mid = CCVector2d(m_keyPoints2d[1].x, m_keyPoints2d[1].y);
	CCVector2d end = CCVector2d(m_keyPoints2d[2].x, m_keyPoints2d[2].y);

	Point_2 p1 = Point_2(start.x, start.y);
	Point_2 p2 = Point_2(mid.x, mid.y);
	Point_2 p3 = Point_2(end.x, end.y);
	CCVector2d arcMidPoint2D;
	if (CGAL::orientation(p1, p2, p3) != CGAL::CLOCKWISE)
	{
		CCVector2d temp = start;
		start = end;
		end = temp;
	}


	for (int i = lst.size() - 1; i >= 0; i--)
	{
		CCVector2d pos = lst[i];
		Point_2 p(pos.x, pos.y);

		CCVector2d midpoint = getArcMidPointByPerpendicular(start, end);//getArcMidpoint(start, end);
		if (isnan(midpoint.x) || isnan(midpoint.y))
			break;

		if (segmentIntersect(midpoint, start, pos, CCVector2d(m_center.x, m_center.y))
			|| segmentIntersect(midpoint, end, pos, CCVector2d(m_center.x, m_center.y)))
		{
			std::vector<CCVector2d>::const_iterator it = std::find_if(lst.cbegin(), lst.cend(), [&](CCVector2d temp) {
				return abs(temp.norm() - pos.norm()) < EPSINON;
			});
			if (it != lst.end())
			{
				lst.erase(it);
			}
		}
		else
		{
			std::cout << "The point is not on the arc." << std::endl;


		}
	}
}