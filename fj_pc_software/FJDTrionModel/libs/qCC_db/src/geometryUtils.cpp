#include "geometryUtils.h"
#include "ccMath.h"
#define PI 3.14159265359
#include "cc2DArcLine.h"
#include "cc2DRect.h"
#include "cc2DRound.h"
#include "ccLines.h"

#include <QtMath>
using namespace std;

// 根据三点圆计算圆心半径(3点必须共面，考虑了z值)
bool geometryUtils::getCenterOfCircle(const std::vector<CCVector3d>& points, CCVector3d& center, double& radius)
{
    if (points.size() != 3)
        return false;

    double x1 = points[0].x,
           x2 = points[1].x,
           x3 = points[2].x;
    double y1 = points[0].y,
           y2 = points[1].y,
           y3 = points[2].y;
    double z1 = points[0].z,
           z2 = points[1].z,
           z3 = points[2].z;

    double a1 = (y1 * z2 - y2 * z1 - y1 * z3 + y3 * z1 + y2 * z3 - y3 * z2),
           b1 = -(x1 * z2 - x2 * z1 - x1 * z3 + x3 * z1 + x2 * z3 - x3 * z2),
           c1 = (x1 * y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2),
           d1 = -(x1 * y2 * z3 - x1 * y3 * z2 - x2 * y1 * z3 + x2 * y3 * z1 + x3 * y1 * z2 - x3 * y2 * z1);

    double a2 = 2 * (x2 - x1),
           b2 = 2 * (y2 - y1),
           c2 = 2 * (z2 - z1),
           d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

    double a3 = 2 * (x3 - x1),
           b3 = 2 * (y3 - y1),
           c3 = 2 * (z3 - z1),
           d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

    double cx = -(b1 * c2 * d3 - b1 * c3 * d2 - b2 * c1 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b3 * c2 * d1)
        / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);
    double cy = (a1 * c2 * d3 - a1 * c3 * d2 - a2 * c1 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a3 * c2 * d1)
        / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);
    double cz = -(a1 * b2 * d3 - a1 * b3 * d2 - a2 * b1 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a3 * b2 * d1)
        / (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);

    center = CCVector3d(cx, cy, cz);
    radius = (center - points[0]).norm();

    return true;
}

// 两向量夹角
double geometryUtils::getAngle(CCVector3d v1, CCVector3d v2)
{
    double angle = acos(v1.dot(v2) / fabs(v1.norm() * v2.norm()));
    return angle;
}

// 将点投影至指定平面（平面由法向量和平面上的点确定）
CCVector3d geometryUtils::projToplane(const CCVector3d& axis, const CCVector3d onPlanePoint, CCVector3d pos)
{
    CCVector3d pose3D;
    double a = axis.x;
    double b = axis.y;
    double c = axis.z;
    double t = a * onPlanePoint.x + b * onPlanePoint.y + c * onPlanePoint.z - (a * pos.x + b * pos.y + c * pos.z);
    t /= a * a + b * b + c * c;
    pose3D.x = pos.x + a * t;
    pose3D.y = pos.y + b * t;
    pose3D.z = pos.z + c * t;
    return pose3D;
}

// 指定平面旋转,updir是法向量
CCVector3d geometryUtils::rotate(CCVector3d& v, CCVector3d& up_dir, double angle)
{
    //https://zhuanlan.zhihu.com/p/380237903
    return v * cos(angle) + (1 - cos(angle)) * (up_dir.dot(v)) * up_dir + sin(angle) * (up_dir.cross(v));
}

// 弧线离散化
std::vector<CCVector3d> geometryUtils::getCircleArcSamplePoints(CCVector3d dir, int num,
                                                                const std::vector<CCVector3d>& points,
                                                                CCVector3d& dCenter, double& radius, double& angle,
                                                                bool isCircle)
{
    std::vector<CCVector3d> result;
    if (points.size() != 3)
        return result;

    CCVector3d center;
    double dRadius;
    getCenterOfCircle(points, center, dRadius);
    dCenter = center;
    radius = dRadius;

    // 如果是圆就直接绕dir旋转一下任一点与圆心的向量,避免重复计算
    if (isCircle)
        return getCircleSamplePoints(dir, num, center, points[0]);

    double sweepAngle = getAngle(points[0] - center, points[2] - center);
    double ang1 = getAngle(points[0] - center, points[1] - center);
    double ang2 = getAngle(points[1] - center, points[2] - center);
    angle = CCCoreLib::RadiansToDegrees(sweepAngle);

    if (fabs(ang1 + ang2 - sweepAngle) > 0.001)
        sweepAngle = 2 * PI - sweepAngle;

    double ang = sweepAngle / num;
    for (int i = 0; i < num; i++)
    {
        // https://www.zhihu.com/question/488858855
        // https://www.zhihu.com/question/393585740

        CCVector3d v1 = points[0] - center;
        CCVector3d v2 = points[2] - center;
        CCVector3d vRotated = sin(sweepAngle - ang * i) / sin(sweepAngle) * v1 + sin(ang * i) / sin(sweepAngle) * v2;
        CCVector3d pt = vRotated + center;

        result.push_back(pt);
    }
    return result;
}

// 圆离散化
std::vector<CCVector3d> geometryUtils::getCircleSamplePoints(CCVector3d dir, int num, const CCVector3d& center,
                                                             const CCVector3d& ptOn)
{
    std::vector<CCVector3d> result;
    double sweepAngle = 2 * PI;
    double ang = 2 * PI / num;
    dir.normalize();
    for (int i = 0; i < num; i++)
    {
        CCVector3d v1 = ptOn - center;
        CCVector3d v = rotate(v1, dir, ang * i);
        CCVector3d pt = v + center;
        result.push_back(pt);
    }
    return result;
}

// 圆的最近点
CCVector3d geometryUtils::getCircleClosestPoint(CCVector3d center, double radius, const CCVector3d& givePt)
{
    CCVector3d result;
    CCVector3d v = givePt - center;
    double distance = v.norm();
    if (distance < 0.001)
    {
        result = center;
    }
    else
    {
        result.x = center.x + radius * v.x / distance;
        result.y = center.y + radius * v.y / distance;
        result.z = center.z + radius * v.z / distance;
    }
    return result;
}

// 计算捕捉点
// 0:mid point----1:key point---2:closest point
int geometryUtils::getOsnapPoint(cc2DItemBase* item, const CCVector3d& givePt, CCVector3d& result, double scale, CCVector2d screen2d)
{
    if (!item)
        return -1;
    if (getOsnap_MidPoint(item, givePt, result, screen2d) == true)
        return 0;
    if (getOsnap_KeyPoint(item, givePt, result, cc2DItemBase::kAnyPoint, screen2d) == true)
        return 1;
    if (getOsnap_ClosestPoint(item, givePt, result, scale) == true)
        return 2;
    return -1;
}

// 节点捕捉
bool geometryUtils::getOsnap_KeyPoint(cc2DItemBase* item, const CCVector3d& givePt, CCVector3d& result
	, cc2DItemBase::OsnapMode mode, CCVector2d screen2d)
{
    auto rect = dynamic_cast<cc2DRect*>(item);
    std::vector<CCVector3d> points = rect == nullptr ? item->m_keyPoints2d : rect->m_drawPoints2d;

    auto circle = dynamic_cast<cc2DRound*>(item);
    auto arc = dynamic_cast<cc2DArcLine*>(item);
	if (mode == cc2DItemBase::kKeyPoint)
		points.pop_back();

	if (rect)
		return get_Point_KeyPoint(item, points, rect->m_drawPoints3d, givePt, result, screen2d);

    return get_Point_KeyPoint(item, points, item->m_keyPoints3d, givePt, result, screen2d);
}


bool geometryUtils::getOsnap_CenterPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale,
	cc2DItemBase::OsnapMode mode, CCVector2d screen2d)
{
	std::vector<CCVector3d> points;
	double radius;
	CCVector3d center;

	if (item->m_keyPoints3d.size() < 2)
		return false;

	cc2DArcLine* pArc = dynamic_cast<cc2DArcLine*>(item);
	if (pArc)
	{
		getCenterOfCircle(pArc->m_keyPoints3d, center, radius);
		points.push_back(center);
	}

	cc2DRound* pRound = dynamic_cast<cc2DRound*>(item);
	if (pRound)
	{
		if (pRound->getItemType() == cc2DItemBase::ROUND3)
			getCenterOfCircle(pRound->m_keyPoints3d, center, radius);
		else if (pRound->getItemType() == cc2DItemBase::ROUND2)
			center = 0.5 * (pRound->m_keyPoints3d[0] + pRound->m_keyPoints3d[1]);
		else if (pRound->getItemType() == cc2DItemBase::ROUNDRADIUS)
			center = pRound->m_keyPoints3d[0];

		points.push_back(center);
	}
	return get_Point(item, points, givePt, result, scale, screen2d);
}

// 最近点查询
bool geometryUtils::get_Point(cc2DItemBase* item, const std::vector<CCVector3d>& points, const CCVector3d& givePt,
	CCVector3d& result, double scale, CCVector2d screen2d)
{
	if (points.size() == 0)
		return false;

	int index = -1;
	double dMaxLen = 99999999.0;

	for (int i = 0; i < points.size(); i++)
	{
		double dLen = (givePt - points[i]).norm();
		if (dLen < dMaxLen)
		{
			dMaxLen = dLen;
			index = i;
		}
	}
	double dist = (points[index] - givePt).norm() / scale;
	if (dist > 0.01)
		index = -1;
	else if (dist <= 0.01)
		item->m_ptOsnapPoints.push_back(points[index]);


	if (index != -1)
	{
		result = points[index];
		return true;
	}
	return false;
}


// 最近点查询
bool geometryUtils::get_Point_KeyPoint(cc2DItemBase* item, const std::vector<CCVector3d>& points, const std::vector<CCVector3d>& points3d,
		const CCVector3d& givePt,CCVector3d& result, CCVector2d screen2d)
{
    if (points.size() < 2 )
        return false;

    int index = -1;
    double dMaxLen = 99999999.0;

    for (int i = 0; i < points.size(); i++)
    {
        double dLen = (screen2d - CCVector2d(points[i].x, points[i].y)).norm();
        if (dLen < dMaxLen)
        {
            dMaxLen = dLen;
            index = i;
        }
    }
    if (index>points.size()-1)
    {
        return false;
    }

	if (screen2d.x <= points[index].x + 8 && screen2d.x >= points[index].x - 8
		&& screen2d.y <= points[index].y + 8 && screen2d.y >= points[index].y - 8)
	{
		item->m_ptOsnapPoints.push_back(points3d[index]);
	}
	else
	{
		index = -1;
	}

    if (index != -1)
    {
        result = points3d[index];
		item->m_ptScreen2dPoints.push_back(screen2d);
        return true;
    }
    return false;
}


// 最近点查询
bool geometryUtils::get_Point_MidPoint(cc2DItemBase* item, const std::vector<CCVector3d>& points2d, const std::vector<CCVector3d>& points3d,
	const CCVector3d& givePt, CCVector3d& result, CCVector2d screen2d)
{
	if (points2d.size() == 0 || points3d.empty())
		return false;

	int index = -1;
	double dMaxLen = 99999999.0;

	for (int i = 0; i < points2d.size(); i++)
	{
		double dLen = (screen2d - CCVector2d(points2d[i].x, points2d[i].y)).norm();
		if (dLen < dMaxLen)
		{
			dMaxLen = dLen;
			index = i;
		}
	}

	if (index < 0 || index >= points2d.size() || index >= points3d.size()){
		return false;
	}

	

	if (screen2d.x <= points2d[index].x + 8 && screen2d.x >= points2d[index].x - 8
		&& screen2d.y <= points2d[index].y + 8 && screen2d.y >= points2d[index].y - 8)
	{
		item->m_ptOsnapPoints.push_back(points3d[index]);
	}
	else
	{
		index = -1;
	}

	if (index != -1)
	{
		result = points3d[index];
		item->m_ptScreen2dPoints.push_back(screen2d);
		return true;
	}
	return false;
}

// 最近点计算
bool geometryUtils::getOsnap_ClosestPoint(cc2DItemBase* item, const CCVector3d& givePt,
                                          CCVector3d& result, double scale)
{
    if (getOsnap_Line_ClosestPoint(item, givePt, result, scale))
        return true;
    if (getOsnap_Circle_ClosestPoint(item, givePt, result, scale))
        return true;
    if (getOsnap_Arc_ClosestPoint(item, givePt, result, scale))
        return true;
    if (getOsnap_Rectangle_ClosestPoint(item, givePt, result, scale))
        return true;
    return false;
}

// 多线段上指定点的最近点
CCVector3d getClosestPointOnPolyline(std::vector<CCVector3d> polyLine, CCVector3d p)
{
    CCVector3d closestPoint(polyLine[0]);
    double shortestDistance = (p - polyLine[0]).norm();
    for (size_t i = 1; i < polyLine.size(); i++)
    {
        auto v = CCVector3d(polyLine[i].x - polyLine[i - 1].x, polyLine[i].y - polyLine[i - 1].y,
                            polyLine[i].z - polyLine[i - 1].z);
        auto w = CCVector3d(p.x - polyLine[i - 1].x, p.y - polyLine[i - 1].y, p.z - polyLine[i - 1].z);
        double dot = w.x * v.x + w.y * v.y + w.z * v.z;
        double len2 = v.x * v.x + v.y * v.y + v.z * v.z;
        double t = len2 == 0 ? 0 : dot / len2;
        auto projection = CCVector3d(polyLine[i - 1].x + v.x * t, polyLine[i - 1].y + v.y * t,
                                     polyLine[i - 1].z + v.z * t);
        double distanceToProjection = (p - projection).norm();
        if (distanceToProjection < shortestDistance)
        {
            shortestDistance = distanceToProjection;
            closestPoint = projection;
        }
    }
    return closestPoint;
}

bool geometryUtils::getOsnap_Line_ClosestPoint(cc2DItemBase* item, const CCVector3d& givePt,
                                               CCVector3d& result, double scale)
{
    auto line = dynamic_cast<ccLines*>(item);
    if (!line)
        return false;

    std::vector<CCVector3d> points = item->m_keyPoints3d;
    if (points.size() == 0)
        return false;
    if (line->m_isClosed)
        points.push_back(*points.begin());

    result = getClosestPointOnPolyline(points, givePt);
    if (!isOn(item, result))
        return false;

    double dist = (result - givePt).norm() / 100.0;
    if (dist < 0.01)
        return true;

    return false;
}

bool geometryUtils::getOsnap_Rectangle_ClosestPoint(cc2DItemBase* item, const CCVector3d& givePt,
                                                    CCVector3d& result, double scale)
{
    auto rect = dynamic_cast<cc2DRect*>(item);
    if (!rect)
        return false;

    std::vector<CCVector3d> points = rect->m_drawPoints3d;
    if (points.size() == 0)
        return false;
    points.push_back(*points.begin());

    result = getClosestPointOnPolyline(points, givePt);
    if (!isOn(item, result))
        return false;

    double dist = (result - givePt).norm() / scale;
    if (dist < 0.01)
        return true;

    return false;
}

bool geometryUtils::getOsnap_Circle_ClosestPoint(cc2DItemBase* item, const CCVector3d& givePt,
                                                 CCVector3d& result, double scale)
{
    auto circle = dynamic_cast<cc2DRound*>(item);
    if (!circle)
        return false;

    CCVector3d center;
    double radius;

	if (circle->getItemType() == cc2DItemBase::ROUND3)
	{
		getCenterOfCircle(circle->m_keyPoints3d, center, radius);
	}
	else if (circle->getItemType() == cc2DItemBase::ROUND2)
	{
		center = 0.5 * (circle->m_keyPoints3d[0] + circle->m_keyPoints3d[1]);
		radius = (circle->m_keyPoints3d[0] - circle->m_keyPoints3d[1]).norm()/2.0;
	}
	else if (circle->getItemType() == cc2DItemBase::ROUNDRADIUS)
	{
		center = circle->m_keyPoints3d[0];
		radius = (circle->m_keyPoints3d[0] - circle->m_keyPoints3d[1]).norm();
	}
	result = getCircleClosestPoint(center, radius, givePt);


    double dist = (result - givePt).norm() / scale;
    if (dist < 0.01)
        return true;

    return false;
}

bool geometryUtils::getOsnap_Arc_ClosestPoint(cc2DItemBase* item, const CCVector3d& givePt,
                                              CCVector3d& result, double scale)
{
    auto arc = dynamic_cast<cc2DArcLine*>(item);
    if (!arc)
        return false;

    if (arc->m_keyPoints3d.size() != 3)
        return false;

    if (arc->m_drawPoints3d.size() == 0)
        return false;

    return get_Point(item, arc->m_drawPoints3d, givePt, result, scale);
}

bool geometryUtils::getOsnap_MidPoint(cc2DItemBase* item, const CCVector3d& givePt, CCVector3d& result, CCVector2d screen2d)
{
    if (item->getName().compare("Arc") == 0 || item->getName().compare("Round") == 0)
        return false;

    int index = -1;
    double dMaxLen = 99999999.0;
    auto rect = dynamic_cast<cc2DRect*>(item);
    auto line = dynamic_cast<ccLines*>(item);
    std::vector<CCVector3d> tmp = rect == nullptr ? item->m_keyPoints2d : rect->m_drawPoints2d;

    if (rect && item->m_keyPoints2d.size() > 0)
        tmp.push_back(item->m_keyPoints2d[0]);

    if (line && item->m_keyPoints2d.size() > 0)
        if (line->m_isClosed)
            tmp.push_back(item->m_keyPoints2d[0]);

    std::vector<CCVector3d> points2d;
    for (int i = 0; i < tmp.size() - 1; i++)
    {
        CCVector3d pt = 0.5 * (tmp[i] + tmp[i + 1]);
		points2d.push_back(pt);
    }

	tmp.clear();
	tmp = rect == nullptr ? item->m_keyPoints3d : rect->m_drawPoints3d;

	if (rect && item->m_keyPoints3d.size() > 0)
		tmp.push_back(item->m_keyPoints3d[0]);

	if (line && item->m_keyPoints3d.size() > 0)
		if (line->m_isClosed)
			tmp.push_back(item->m_keyPoints3d[0]);

	std::vector<CCVector3d> points3d;
	for (int i = 0; i < tmp.size() - 1; i++)
	{
		CCVector3d pt = 0.5 * (tmp[i] + tmp[i + 1]);
		points3d.push_back(pt);
	}

    return get_Point_MidPoint(item, points2d, points3d, givePt, result, screen2d);
}

// 点是否在线段上
bool isBetweenPoints(const CCVector3d& pt1, const CCVector3d& pt2, const CCVector3d& testPt)
{
    double epsilon = 0.00001;
    double dist = (pt1 - pt2).norm();
    double dist1 = (pt1 - testPt).norm();
    double dist2 = (pt2 - testPt).norm();

    if (fabs(dist1 + dist2 - dist) <= epsilon)
    {
        return true;
    }
    return false;
}

bool geometryUtils::isOn(cc2DItemBase* item, const CCVector3d& givePt)
{
    vector<CCVector3d> pts = item->m_keyPoints3d;
    if (item->getName().compare("Rect") == 0)
    {
        auto rect = dynamic_cast<cc2DRect*>(item);
        if (!rect)
            return false;
        pts = rect->m_drawPoints3d;
        pts.push_back(*pts.begin());
    }

    bool ison = false;
    for (int i = 0; i < pts.size() - 1; i++)
    {
        CCVector3d pt1 = pts[i];
        CCVector3d pt2 = pts[i + 1];
        if (isBetweenPoints(pt1, pt2, givePt))
        {
            ison = true;
            break;
        }
    }
    return ison;
}

void geometryUtils::getThreePointsByArcInfo(std::vector<CCVector3d>& pts, CCVector3d norm, const CCVector3d& center,
                                            double dRadius, double startAngle, double endAngle)
{
    // hard to return back 3 points on arc in 3d space
}

void geometryUtils::getArcAngleInfo(const std::vector<CCVector3d>& points, CCVector3d& center, double& dRadius,
                                    double& startAngle, double& endAngle, CCVector3d norm)
{
    //hard to calc the arc info in 3d space
}


struct PolylineExtendInfo
{
    CCVector3d point; // 待延伸点
    int polylineIndex; // 待延伸点是输入多线段的索引
    int vertexIndex; // 包含可延伸点的多段线上最近的顶点在该多段线的顶点数组中的下标
    CCVector3d extendPoint; // 可延伸点在另一个多段线上的延伸位置
};


bool extendLine(const vector<CCVector3d>& line1, const vector<CCVector3d>& line2, CCVector3d& canExtendPoint,
                CCVector3d& extendPoint)
{
    CCVector3d p1 = line1[0];
    CCVector3d p2 = line1[1];
    CCVector3d q1 = line2[0];
    CCVector3d q2 = line2[1];
    CCVector3d u = p2 - p1;
    CCVector3d v = q2 - q1;
    CCVector3d w = p1 - q1;
    double a = u.dot(u);
    double b = u.dot(v);
    double c = v.dot(v);
    double d = u.dot(w);
    double e = v.dot(w);
    double D = a * c - b * b;

	double result = (p1 - q1).cross(p2 - q1).norm();
	double result1 = (p1 - q2).cross(p2 - q2).norm();
	//共线返回false
	if (result <=0.01 && result1 <= 0.01)
	{
		return false;
	}

    if (D <= 0)
        return false;

    extendPoint = p1 + ((b * e - c * d) / D) * u;

    if (!isBetweenPoints(p1, p2, extendPoint))
        return false;

    if (isBetweenPoints(q1, q2, extendPoint))
        return false;

    double dist1 = (extendPoint - q1).norm();
    double dist2 = (extendPoint - q2).norm();
    if (dist1 < dist2)
        canExtendPoint = q1;
    else
        canExtendPoint = q2;

    CCVector3d v1 = extendPoint - canExtendPoint;
    v1.normalize();
    v.normalize();
    if (v1.dot(v) < 0) // >0 同向， < 0反向， = 0垂直
        return false;
    return true;
}

std::vector<PolylineExtendInfo> calculateExtensionInfo(const std::vector<std::vector<CCVector3d>>& polylines)
{
    std::vector<PolylineExtendInfo> result;

    for (int i = 0; i < polylines.size(); i++)
    {
        const auto& srcPolyline = polylines[i];
        if (srcPolyline.size() < 2)
            continue;

        for (int j = 0; j < polylines.size(); j++)
        {
            std::vector<PolylineExtendInfo> result1, result2;

            const auto& dstPolyline = polylines[j];

            if (j == i || dstPolyline.size() < 2)
                continue;

            double minDistance = std::numeric_limits<double>::max();
            int minIndex = -1;

            vector<CCVector3d> ptsA{srcPolyline[1], srcPolyline[0]};
            for (int k = 0; k < dstPolyline.size() - 1; k++)
            {
                CCVector3d toExtendPoint, extendPoint;
                vector<CCVector3d> pts2{dstPolyline[k], dstPolyline[k + 1]};
                bool ok = extendLine(pts2, ptsA, toExtendPoint, extendPoint);
                if (!ok)
                    continue;
                double distance = (toExtendPoint - extendPoint).norm();
                if (distance < minDistance)
                {
                    minDistance = distance;
                    minIndex++;
                }

                if (minIndex >= 0)
                {
                    PolylineExtendInfo info;
                    info.point = toExtendPoint;
                    info.polylineIndex = i;
                    info.vertexIndex = (toExtendPoint - srcPolyline[1]).norm() < (toExtendPoint - srcPolyline[0]).norm()
                                           ? 1
                                           : 0;
                    info.extendPoint = extendPoint;
                    result1.push_back(info);
                }
            }
            if (minIndex >= 0)
                result.push_back(result1[minIndex]);

            minIndex = -1;
            minDistance = std::numeric_limits<double>::max();
            vector<CCVector3d> ptsB{srcPolyline[srcPolyline.size() - 2], srcPolyline[srcPolyline.size() - 1]};
            for (int k = 0; k < dstPolyline.size() - 1; k++)
            {
                CCVector3d toExtendPoint, extendPoint;
                vector<CCVector3d> pts2{dstPolyline[k], dstPolyline[k + 1]};
                bool ok = extendLine(pts2, ptsB, toExtendPoint, extendPoint);
                if (!ok)
                    continue;
                double distance = (toExtendPoint - extendPoint).norm();
                if (distance < minDistance)
                {
                    minDistance = distance;
                    minIndex++;
                }

                if (minIndex >= 0)
                {
                    PolylineExtendInfo info;
                    info.point = toExtendPoint;
                    info.polylineIndex = i;
                    info.vertexIndex = srcPolyline.size() - 1;
                    info.extendPoint = extendPoint;
                    result2.push_back(info);
                }
            }
            if (minIndex >= 0)
                result.push_back(result2[minIndex]);
        }
    }
    return result;
}

// 两两间能否延伸，暂时保留
cc2DItemBase* geometryUtils::editor_extend_ex(std::vector<cc2DItemBase*> items, const CCVector3d pos, bool modify)
{
    vector<vector<CCVector3d>> polylines;
    for (cc2DItemBase* item : items)
        polylines.push_back(item->m_keyPoints3d);

    std::vector<PolylineExtendInfo> infos = calculateExtensionInfo(polylines);
    for (PolylineExtendInfo info : infos)
    {
        cc2DItemBase* item = items[info.polylineIndex];
        //item->m_pointsTmp.clear();
        item->m_pointsTmp.push_back(info.point);
        item->m_pointsTmp.push_back(info.extendPoint);
        item->redrawDisplay();
    }
    return nullptr;
}

bool getExtendPoint(const vector<CCVector3d> dstPolyline, const vector<CCVector3d> ptsA, CCVector3d& ptExtend)
{
    double minDistance = std::numeric_limits<double>::max();
    int minIndex = -1;
    for (int k = 0; k < dstPolyline.size() - 1; k++)
    {
        CCVector3d toExtendPoint, extendPoint;
        vector<CCVector3d> pts2{dstPolyline[k], dstPolyline[k + 1]};
        bool ok = extendLine(pts2, ptsA, toExtendPoint, extendPoint);
        if (!ok)
            continue;

        double distance = (toExtendPoint - extendPoint).norm();
        if (distance < minDistance)
        {
            minDistance = distance;
            minIndex++;
            ptExtend = extendPoint;
        }
    }
    if (minIndex != -1)
        return true;

    return false;
}

// 将矢量数据填充至polylines数组
void fillPoints(vector<vector<CCVector3d>>& polylines, std::vector<cc2DItemBase*> items, cc2DItemBase* item)
{
    for (cc2DItemBase* it : items)
    {
        if (it == item)
            continue;

        if (it->getName().compare("Rect") == 0)
        {
            auto rect = dynamic_cast<cc2DRect*>(it);
            if (!rect)
                continue;
            vector<CCVector3d> pts = rect->m_drawPoints3d;
            pts.push_back(*pts.begin());
            polylines.push_back(pts);
            continue;
        }

        if (it->getName().compare("Round") == 0)
        {
            auto circle = dynamic_cast<cc2DRound*>(it);
            if (!circle)
                continue;
            polylines.push_back(circle->m_drawPoints3d);
            continue;
        }

        if (it->getName().compare("Arc") == 0)
        {
            auto arc = dynamic_cast<cc2DArcLine*>(it);
            if (!arc)
                continue;
            polylines.push_back(arc->m_drawPoints3d);
            continue;
        }

        polylines.push_back(it->m_keyPoints3d);
    }
}

bool isOnArc(cc2DItemBase* arc, const CCVector3d givePt)
{
    CCVector3d result;
    bool ok = geometryUtils::getOsnap_Arc_ClosestPoint(arc, givePt, result, 100.0);
    return ok;
}

// 圆与直线相交-返回交点
std::vector<CCVector3d> intersectCircleLine(cc2DItemBase* item, const CCVector3d& center, double radius,
                                            const CCVector3d& normal, const CCVector3d& lineStart,
                                            const CCVector3d& lineEnd)
{
    std::vector<CCVector3d> intersections;

    CCVector3d lineDir = lineEnd - lineStart;

    double t = lineDir.dot(center - lineStart) / lineDir.norm2();
    CCVector3d intersection = lineStart + lineDir * t;

    double dist = (intersection - center).norm();
    if (std::abs(dist - radius) < 1e-6)
    {
        intersections.push_back(intersection);
    }
    else if (dist < radius)
    {
        lineDir.normalize();
        double d = std::sqrt(radius * radius - dist * dist);
        CCVector3d pt1 = intersection + lineDir * d;
        CCVector3d pt2 = intersection - lineDir * d;
        if (isBetweenPoints(lineStart, lineEnd, pt1))
            intersections.push_back(intersection + lineDir * d);
        if (isBetweenPoints(lineStart, lineEnd, pt2))
            intersections.push_back(intersection - lineDir * d);
    }

    return intersections;
}

// 圆弧与直线段的相交之后判断起点--->延伸点或终点->延伸点（取一组）
bool getArcExtendPoint(const vector<CCVector3d> ptIntersects, const CCVector3d pos, CCVector3d& toExtendPoint,
                       CCVector3d& extendPoint, const CCVector3d arcStart, const CCVector3d arcEnd)
{
    if (ptIntersects.size() == 0)
        return false;

    toExtendPoint = (arcStart - pos).norm() < (arcEnd - pos).norm() ? arcStart : arcEnd;
    if (ptIntersects.size() == 1)
    {
        extendPoint = ptIntersects[0];
        return true;
    }
    if (ptIntersects.size() == 2)
    {
        double dist0 = (toExtendPoint - ptIntersects[0]).norm();
        double dist1 = (toExtendPoint - ptIntersects[1]).norm();
        extendPoint = dist0 < dist1 ? ptIntersects[0] : ptIntersects[1];
        return true;
    }
    return false;
}

CCVector2d getArcMidpoint(const CCVector2d& p12d, const CCVector2d& p22d, double radius)
{
	// 计算连线中点坐标
	double centerX = (p12d.x + p22d.x) / 2.0;
	double centerY = (p12d.y + p22d.y) / 2.0;

	// 计算圆弧的弧度
	double chord = sqrt(pow((p22d.x - p12d.x), 2) + pow((p22d.y - p12d.y), 2));
	double sagitta = radius - sqrt(pow(radius, 2) - pow((chord / 2), 2));
	double theta = 2.0 * asin(chord / (2.0 * radius));
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

std::vector<CCVector2d> lineCircleIntersections(std::vector<CCVector2d> lstLine, std::vector<CCVector2d> lstCircle)
{
	std::vector<CCVector2d> lstIntersection;
	std::vector<CCVector2d> intersections;

	if(lstCircle.size() == 3 && lstLine.size() == 2)
	{
		CCVector2d q1 = CCVector2d(lstCircle[0].x, lstCircle[0].y);
		CCVector2d q2 = CCVector2d(lstCircle[1].x, lstCircle[1].y);
		CCVector2d q3 = CCVector2d(lstCircle[2].x, lstCircle[2].y);
		Point_2 p1 = Point_2(q1.x, q1.y);
		Point_2 p2 = Point_2(q2.x, q2.y);
		Point_2 p3 = Point_2(q3.x, q3.y);
		Circle_2 circle(p1, p2, p3);

		q1 = CCVector2d(lstLine[0].x, lstLine[0].y);
		q2 = CCVector2d(lstLine[1].x, lstLine[1].y);
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

bool geometryUtils::doArcExtend(cc2DItemBase* item, std::vector<cc2DItemBase*> items, const CCVector3d& pos, 
	bool modify)
{
	std::vector<CCVector2d> intersections;
	std::vector<cc2DItemBase *>::const_iterator it = std::find_if(items.cbegin(), items.cend(), [&](cc2DItemBase * rhs) {
		return rhs && dynamic_cast<cc2DItemBase*>(rhs) == item;
	});
	if (it != items.cend() && (*it)->getItemType() != cc2DItemBase::LINES)
		items.erase(it);

	for (int i = 0; i < items.size(); i++)
	{
		cc2DItemBase* pItemTemp = dynamic_cast<cc2DItemBase*>(items[i]);
		std::vector<CCVector2d> lst;
		if (pItemTemp && pItemTemp->participatingIntersection())
		{
			cc2DItemBase::EGraphicType type = pItemTemp->getItemType();
			switch (type)
			{
			case cc2DItemBase::NONE:
				break;
			case cc2DItemBase::LINE:
			case cc2DItemBase::LINES:
			{
				ccLines* pLineTemp = dynamic_cast<ccLines*>(pItemTemp);
				if (pLineTemp)
					lst = item->intersect(pLineTemp, true);
			}
			break;
			case cc2DItemBase::ARC:
			{
				cc2DArcLine* pArcTemp = dynamic_cast<cc2DArcLine*>(pItemTemp);
				if (pArcTemp)
					lst = item->intersect(pArcTemp, true);
			}
			break;
			case cc2DItemBase::RECTACROSS:
			case cc2DItemBase::RECT3:
			{
				cc2DRect* pRectTemp = dynamic_cast<cc2DRect*>(pItemTemp);
				if (pRectTemp)
					lst = item->intersect(pRectTemp, true);
			}
			break;
			case cc2DItemBase::ROUND3:
			case cc2DItemBase::ROUND2:
			case cc2DItemBase::ROUNDRADIUS:
			{
				cc2DRound* pRoundTemp = dynamic_cast<cc2DRound*>(pItemTemp);
				if (pRoundTemp)
					lst = item->intersect(pRoundTemp, true);
			}
			break;
			default:
				break;
			}
		}
		foreach(CCVector2d intersection, lst)
			intersections.push_back(intersection);
	}
	
	CCVector3d arcStart = item->m_keyPoints3d[0];
	CCVector3d arcSecPoint = item->m_keyPoints3d[1];
	CCVector3d arcEnd = item->m_keyPoints3d[2];

	CCVector3d arcStart2D = item->m_keyPoints2d[0];
	CCVector3d arcSecPoint2D = item->m_keyPoints2d[1];
	CCVector3d arcEnd2D = item->m_keyPoints2d[2];

	int index = (arcStart - pos).norm() < (arcEnd - pos).norm() ? 0 : 2;
	if (item->m_keyPoints2d.size() <= 0)
		return false;

	double z = item->m_keyPoints2d[0].z;
	double minDistance = std::numeric_limits<double>::max();
	CCVector3d dotlineEndPoint;
	if (intersections.size() == 0)
		return false;

	CCVector3d center;
	double radius;
	getCenterOfCircle(item->m_keyPoints2d, center, radius);

	//筛选和圆弧端点重合的交点
	for (int i = intersections.size() - 1; i >= 0; i--)
	{
		CCVector2d pos = intersections[i];

		double length1 = CCVector3d::vdistance(CCVector3d(pos.x, pos.y, 0).u, CCVector3d(arcEnd2D.x, arcEnd2D.y, 0).u);
		double length2 = CCVector3d::vdistance(CCVector3d(pos.x, pos.y, 0).u, CCVector3d(arcStart2D.x, arcStart2D.y, 0).u);
		if (length1 <= 10e-5 ||
			length2 <= 10e-5)
		{
			std::vector<CCVector2d>::const_iterator it = std::find_if(intersections.cbegin(), intersections.cend(), [&](CCVector2d temp) {
				return abs(temp.norm() - pos.norm()) < 10e-5;
			});

			if (it != intersections.end())
				intersections.erase(it);
		}
	}

	//筛选离当前鼠标最近端点的最近交点
	for (int i = 0; i < intersections.size(); i++)
	{
		CCVector2d pos = intersections[i];
		CCVector3d inter = CCVector3d(pos.x, pos.y, z);
		double distance = CCVector3d::vdistance(inter.u, item->m_keyPoints2d[index].u);
		if (distance < minDistance)
		{
			minDistance = distance;
			dotlineEndPoint = inter;
		}
	}

	CCVector2d start = CCVector2d(item->m_keyPoints2d[index].x, item->m_keyPoints2d[index].y);
	CCVector2d end = CCVector2d(dotlineEndPoint.x, dotlineEndPoint.y);

	cc2DArcLine* pArc = dynamic_cast<cc2DArcLine*>(item);

	CCVector2d perpendi = pArc->getPerpendicularPoint(CCVector2d(center.x, center.y), start, end);
	std::vector<CCVector2d> lstCircle;
	if (pArc->get2dKeyPoints().size() == 3)
	{
		lstCircle.push_back(CCVector2d(pArc->get2dKeyPoints()[0].x, pArc->get2dKeyPoints()[0].y));
		lstCircle.push_back(CCVector2d(pArc->get2dKeyPoints()[1].x, pArc->get2dKeyPoints()[1].y));
		lstCircle.push_back(CCVector2d(pArc->get2dKeyPoints()[2].x, pArc->get2dKeyPoints()[2].y));
	}

	std::vector<CCVector2d> lstLine;
	lstLine.push_back(CCVector2d(center.x, center.y));
	lstLine.push_back(perpendi);
	//计算待延伸端点和交点构成的直线和待延伸弧线构成的圆的交点
	std::vector<CCVector2d> arcIntersSelect = lineCircleIntersections(lstLine, lstCircle);//pArc->centerIntersectArc(CCVector2d(center.x, center.y), perpendi);
	//确认端点、交点、中点是顺时针的
	if (arcIntersSelect.size() > 0)
	{
		int i = 0;
		for (; i < arcIntersSelect.size(); i++)
		{
			CCVector2d p = arcIntersSelect[i];
			if (pArc->segmentIntersect(start, end, CCVector2d(center.x, center.y), p))
				break;
		}
		if (i >= arcIntersSelect.size())
			return false;

		Point_2 p1 = Point_2(start.x, start.y);
		Point_2 p2 = Point_2(arcIntersSelect[i].x, arcIntersSelect[i].y);
		Point_2 p3 = Point_2(end.x, end.y);
		if (CGAL::orientation(p1, p2, p3) != CGAL::CLOCKWISE)
		{
			CCVector2d temp = start;
			start = end;
			end = temp;
		}
	}
	else
		return false;

	CCVector2d dotlineMidPoint = getArcMidpoint(start, end, radius);
	if (isnan(dotlineMidPoint.x))
		return false;
    if ((dotlineMidPoint-CCVector2d(0,0)).norm()<10e-5 
        || (CCVector2d(dotlineEndPoint.x, dotlineEndPoint.y) - CCVector2d(0, 0)).norm() < 10e-5)
        return false;
	if (!modify)
	{
		if (pArc)
		{
			pArc->setExtrendLineShow(true);
			pArc->clearExtrendLinePts();

			Point_2 p1 = Point_2(item->m_keyPoints2d[index].x, item->m_keyPoints2d[index].y);
			Point_2 p2 = Point_2(dotlineMidPoint.x, dotlineMidPoint.y);
			Point_2 p3 = Point_2(dotlineEndPoint.x, dotlineEndPoint.y);
			if (CGAL::orientation(p1, p2, p3) != CGAL::CLOCKWISE)
			{
				pArc->addExtrendLinePts(CCVector3d(dotlineEndPoint.x, dotlineEndPoint.y, 0));
				pArc->addExtrendLinePts(CCVector3d(dotlineMidPoint.x, dotlineMidPoint.y, 0));
				pArc->addExtrendLinePts(CCVector3d(item->m_keyPoints2d[index].x, item->m_keyPoints2d[index].y, 0));
			}
			else
			{
				pArc->addExtrendLinePts(CCVector3d(item->m_keyPoints2d[index].x, item->m_keyPoints2d[index].y, 0));
				pArc->addExtrendLinePts(CCVector3d(dotlineMidPoint.x, dotlineMidPoint.y, 0));
				pArc->addExtrendLinePts(CCVector3d(dotlineEndPoint.x, dotlineEndPoint.y, 0));
			}
		}
	}
	else
	{
		std::vector<CCVector3d> lst;
		lst.push_back(dotlineEndPoint);
		std::vector<CCVector3d> result = item->unprojectTo3d(lst, item->get3dCamera());
		if (result.size() <= 0)
			return false;
        double length = 0.0;
        length = (item->m_keyPoints3d[index] - result[0]).norm();
		if (length > 0.01)
		{
			pArc->setExtrendLineShow(false);
			item->m_keyPoints3d[index] = result[0];
			item->redrawDisplay();
		}
		else
			return false; //三个点中有两个重合的时候，无法确定圆弧，就不undo redo，不修改节点
	}

	return true;
}

//多段线延伸方向确定
int extendDirection(CCVector3d pos3dEntity, cc2DItemBase* item)
{
    if (item->getItemType() != cc2DItemBase::LINES)
        return -1;
    int left = -1;
    int right = -1;

    for (int i = 0; i < item->get3dKeyPoints().size() - 1; i++)
    {
        if (cc2DItemBase::on_segment(pos3dEntity, item->m_keyPoints3d[i], item->m_keyPoints3d[i + 1]))
        {
            left = i;
            right = i + 1;
            break;
        }
    }
    if (-1 == left || -1 == right)
    {
        return -1;
    }
    double leftLength, rightLength;
    for (int i = 0; i < left; i++)
    {
        leftLength += CCVector3d::vdistance(item->m_keyPoints3d[i].u, item->m_keyPoints3d[i + 1].u);
    }
    leftLength += CCVector3d::vdistance(item->m_keyPoints3d[left].u, pos3dEntity.u);

    for (int i = right; i < item->m_keyPoints3d.size() - 1; i++)
    {
        rightLength += CCVector3d::vdistance(item->m_keyPoints3d[i].u, item->m_keyPoints3d[i + 1].u);
    }
    rightLength += CCVector3d::vdistance(item->m_keyPoints3d[right].u, pos3dEntity.u);

    if (leftLength > rightLength)
        return 0;
    else
        return 1;
}

// 获取两个向量的夹角
double getTwoVectorsAngle(const CCVector2d &v1,
    const CCVector2d &v2) {
    double cosine_value = v1.dot(v2) / (v1.norm() * v2.norm());
    // 归约到-1到1,防止出错
  //  if (cosine_value > 1)
     // cosine_value = 1;
    //else if (cosine_value < -1)
      //cosine_value = -1;

  //弧度
    double value = acos(cosine_value);

    return value;
}


bool isThreePointsInLine(const CCVector2d &p1, const CCVector2d &p2,
    const CCVector2d &p3, double radian_deviation) {
    CCVector2d p1_p2 = p2 - p1;
    CCVector2d p1_p3 = p3 - p1;
    // 有重合点直接返回true
    if (p1_p2.norm() == 0 || p1_p3.norm() == 0) {
        return true;
    }

    double radian_value_of_p1 = getTwoVectorsAngle(p1_p2, p1_p3);


    CCVector2d p3_p1 = p1 - p3;
    CCVector2d p3_p2 = p2 - p3;
    // 有重合点直接返回true
    if (p3_p1.norm() == 0 || p3_p2.norm() == 0) {
        return true;
    }

    double radian_value_of_p3 = getTwoVectorsAngle(p3_p1, p3_p2);

    if (radian_value_of_p1 <= radian_deviation &&
        radian_value_of_p3 <= radian_deviation) {
        return true;
    }
    else {
        return false;
    }
}



bool doLineExtend(cc2DItemBase* item, std::vector<cc2DItemBase*> items, const CCVector3d& pos, bool modify, CCVector3d pos3dEntity, ccGLCameraParameters params)
{
    item->m_pointsTmp.clear();


    //封闭多线段图元关闭延伸
    if (item->getItemType() == cc2DItemBase::EGraphicType::LINES)
    {
        bool resutl = abs(item->m_keyPoints3d[0].norm() - item->m_keyPoints3d[item->m_keyPoints3d.size() - 1].norm()) <= EPSINON
            ? true
            : false;
        if (resutl)
            return false;
    }



    vector<vector<CCVector3d>> polylines;
    fillPoints(polylines, items, item);

    // 多段线可自己延伸至自己
    if (item->getItemType() == cc2DItemBase::EGraphicType::LINES)
        polylines.push_back(item->m_keyPoints3d);

    int size = item->m_keyPoints3d.size();
    //CCVector3d pt = getClosestPointOnPolyline(item->m_keyPoints3d, pos);//错误的，应判断在哪两段连续端点之间

    CCVector3d ptToextend;
    int startIndex, endIndex;

    if (size>0)
    {
        pos3dEntity = CCVector3d(pos3dEntity.x, pos3dEntity.y, item->m_keyPoints3d[0].z);
    }

    if (item->getItemType() == cc2DItemBase::EGraphicType::LINE)
    {
        // 根据鼠标位置找到位置最近的点，起点或终点是待延伸点
        ptToextend = (pos - item->m_keyPoints3d[0]).norm() < (pos - item->m_keyPoints3d[1]).norm()
            ? item->m_keyPoints3d[0]
            : item->m_keyPoints3d[1];
        // 待延伸点的索引
        startIndex = (pos - item->m_keyPoints3d[0]).norm() < (pos - item->m_keyPoints3d[size - 1]).norm()
            ? 1
            : 0;

        endIndex = (pos - item->m_keyPoints3d[0]).norm() < (pos - item->m_keyPoints3d[size - 1]).norm()
            ? 0
            : 1;
    }
	else if(item->getItemType() == cc2DItemBase::EGraphicType::LINES)
	{
        bool onFirstSeg = false;
        bool onLastSeg = false;
        if (cc2DItemBase::on_segment(pos3dEntity, item->m_keyPoints3d[0], item->m_keyPoints3d[1]))
        {
            onFirstSeg = true;
            ptToextend = item->m_keyPoints3d[0];
            if (cc2DItemBase::on_segment(pos3dEntity, item->m_keyPoints3d[0], item->m_keyPoints3d[1]))
                startIndex = 1;
            else
                return false;
        }
        if (!onFirstSeg)
        {
            if (cc2DItemBase::on_segment(pos3dEntity, item->m_keyPoints3d[size - 1], item->m_keyPoints3d[size - 2]))
            {
                onLastSeg = true;
                ptToextend = item->m_keyPoints3d[size - 1];
                if (cc2DItemBase::on_segment(pos3dEntity, item->m_keyPoints3d[size - 1], item->m_keyPoints3d[size - 2]))
                    startIndex = size - 2;
                else
                    return false;
            }
        }
        if (!onFirstSeg&&!onLastSeg)
        {
            int result = extendDirection(pos3dEntity, item);
            if (1 == result)
            {
                ptToextend = item->m_keyPoints3d[0];
                startIndex = 1;
            }
            else if (0 == result)
            {
                ptToextend = item->m_keyPoints3d[size - 1];
                startIndex = size - 2;
            }
            else
            {
                return false;
            }
        }
	}



    // 延伸线（有向）
    vector<CCVector3d> ptsA{item->m_keyPoints3d[startIndex], ptToextend};
    // 延伸后的点
    CCVector3d ptExtend_need;
    // 多处延伸时取最短距离
    double minDistance = std::numeric_limits<double>::max();
    int minIndex = -1;

    std::vector<CCVector2d> intersections;
    for (int i = 0; i < items.size(); i++)
    {
        cc2DItemBase* pItemTemp = dynamic_cast<cc2DItemBase*>(items[i]);
        if (pItemTemp == item)
            continue;
        item->setExtendInterect(true);
        std::vector<CCVector2d> lst;
        if (pItemTemp)
        {
            cc2DItemBase::EGraphicType type = pItemTemp->getItemType();
            switch (type)
            {
            case cc2DItemBase::NONE:
                break;
            case cc2DItemBase::LINE:
            case cc2DItemBase::LINES:
            {
                ccLines* pLineTemp = dynamic_cast<ccLines*>(pItemTemp);
                if (pLineTemp)
                    lst = item->intersect(pLineTemp, true);
            }
            break;
            case cc2DItemBase::ARC:
            {
                cc2DArcLine* pArcTemp = dynamic_cast<cc2DArcLine*>(pItemTemp);
                if (pArcTemp)
                    lst = item->intersect(pArcTemp, true);
            }
            break;
            case cc2DItemBase::RECTACROSS:
            case cc2DItemBase::RECT3:
            {
                cc2DRect* pRectTemp = dynamic_cast<cc2DRect*>(pItemTemp);
                if (pRectTemp)
                    lst = item->intersect(pRectTemp, true);
            }
            break;
            case cc2DItemBase::ROUND3:
            case cc2DItemBase::ROUND2:
            case cc2DItemBase::ROUNDRADIUS:
            {
                cc2DRound* pRoundTemp = dynamic_cast<cc2DRound*>(pItemTemp);
                if (pRoundTemp)
                    lst = item->intersect(pRoundTemp, true);
            }
            break;
            default:
                break;
            }
        }
        foreach(CCVector2d intersection, lst)
            intersections.push_back(intersection);
    }
    std::vector<CCVector3d> lst2d;
    
    for (CCVector2d pt2d : intersections)
    {
        lst2d.push_back(CCVector3d(pt2d.x, pt2d.y, item->m_keyPoints2d[0].z));
    }
    std::vector<CCVector3d> lst3d = item->unprojectTo3d(lst2d, params);
    for (CCVector3d pt3d : lst3d)
    {
        double result = (ptsA[0] - pt3d).cross(ptsA[1] - pt3d).norm();
        if (result > 0.01)
        {
            continue;
        }
        CCVector3d v1 = ptsA[1] - pt3d;
        CCVector3d v = ptsA[0] - ptsA[1];
        v1.normalize();
        v.normalize();
        if (v1.dot(v) < 0) // >0 同向， < 0反向， = 0垂直
            continue;

        double distance = (ptToextend - pt3d).norm();
        if (distance<10e-3)
            continue;
        if (item->getItemType() == cc2DItemBase::EGraphicType::LINE)
        {
            double length = (item->m_keyPoints3d[startIndex] - pt3d).norm();
            if (length < distance)
                continue;
        }
        if (distance < minDistance)
        {
            ptExtend_need = pt3d;
            minDistance = distance;
            minIndex++;
        }
    }
    //for (int i = 0; i < polylines.size(); i++)
    //{
    //    CCVector3d ptExtend;
    //    const auto& dstPolyline = polylines[i];
    //    if (dstPolyline.size() < 2)
    //        continue;

    //    // 计算延伸点，可延长（不相交）的点
    //    bool ok = getExtendPoint(dstPolyline, ptsA, ptExtend);
    //    if (!ok)
    //        continue;

    //    double distance = (ptToextend - ptExtend).norm();
    //    if (distance < minDistance)
    //    {
    //        ptExtend_need = ptExtend;
    //        minDistance = distance;
    //        minIndex++;
    //    }
    //}
    if (minIndex == -1)
        return false;
    //判断延伸点共线
    //if (!isThreePointsInLine(CCVector2d(ptExtend_need.x, ptExtend_need.y), CCVector2d(ptsA[0].x, ptsA[0].y),
    //    CCVector2d(ptsA[1].x, ptsA[1].y), qDegreesToRadians(5.0)))
    //    return false;


    if (!modify)
    {
        item->m_pointsTmp.clear();
        item->m_pointsTmp.push_back(ptExtend_need);
        item->m_pointsTmp.push_back(ptToextend);
        item->redrawDisplay();
    }


    if (modify)
    {
        item->m_pointsTmp.clear();
        int index = (ptToextend - item->m_keyPoints3d[0]).norm() < 0.001 ? 0 : size - 1;
        item->m_keyPoints3d[index] = ptExtend_need;
        item->redrawDisplay();
        item->setExtendInterect(false);
    }
    return true;
}

// 延伸，根据鼠标位置，查找item起点或终点最近可延伸点的，modify为true时将延伸点延伸
bool geometryUtils::editor_extend_ex_ex(cc2DItemBase* item, std::vector<cc2DItemBase*> items, const CCVector3d& pos,
                                        bool modify, CCVector3d pos3dEntity, ccGLCameraParameters params)
{
    // 圆和矩形不可延伸
    if (item->getName().compare("Rect") == 0 ||
        item->getName().compare("Round") == 0)
        return false;

    int size = item->m_keyPoints3d.size();
    if (size < 2)
        return false;

    // 圆弧延伸 
    if (item->getName().compare("Arc") == 0)
        return doArcExtend(item, items, pos, modify);

    // 多线段延伸
    return doLineExtend(item, items, pos, modify, pos3dEntity, params);
}
