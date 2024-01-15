#ifndef GEO_UTILS_HEADER
#define GEO_UTILS_HEADER

#include <vector>
#include "CCGeom.h"
#include "qCC_db.h"

#include "cc2DItemBase.h"

using namespace std;

class QCC_DB_LIB_API geometryUtils
{
public:
    static void getThreePointsByArcInfo(std::vector<CCVector3d>& pts, CCVector3d norm, const CCVector3d& center, double dRadius, double startAngle, double endAngle);

    static void getArcAngleInfo(const std::vector<CCVector3d>& pts, CCVector3d& center, double& dRadius, double& startAngle, double& endAngle, CCVector3d norm);

    static CCVector3d rotate(CCVector3d& v, CCVector3d& up_dir, double ang);

    static bool getCenterOfCircle(const std::vector<CCVector3d>& points, CCVector3d& center, double& dRadius);

    static double getAngle(CCVector3d v1, CCVector3d v2);

    static CCVector3d projToplane(const CCVector3d& axis, const CCVector3d onPlanePoint, CCVector3d pos);

    static std::vector<CCVector3d> getCircleArcSamplePoints(CCVector3d dir, int num, const std::vector<CCVector3d>& points, CCVector3d& dCenter, double& radius, double& angle, bool isCircle = false);

    static std::vector<CCVector3d> getCircleSamplePoints(CCVector3d dir, int num, const CCVector3d& center, const CCVector3d& ptOn);

    static CCVector3d getCircleClosestPoint(CCVector3d center, double radius, const CCVector3d& givePt);
    static int getOsnapPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale, CCVector2d screen2d = CCVector2d());
    static bool getOsnap_KeyPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, cc2DItemBase::OsnapMode mode = cc2DItemBase::kAnyPoint, CCVector2d screen2d = CCVector2d());
	static bool getOsnap_CenterPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale, cc2DItemBase::OsnapMode mode = cc2DItemBase::kCenterpoint, CCVector2d screen2d = CCVector2d());

	static bool get_Point(cc2DItemBase* item, const std::vector<CCVector3d>& points, const CCVector3d & givePt, CCVector3d& result, double scale, CCVector2d screen2d = CCVector2d());
    static bool get_Point_KeyPoint(cc2DItemBase* item, const std::vector<CCVector3d>& points, const std::vector<CCVector3d>& points3d, const CCVector3d & givePt, CCVector3d& result, CCVector2d screen2d = CCVector2d());
	static bool get_Point_MidPoint(cc2DItemBase* item, const std::vector<CCVector3d>& points2d, const std::vector<CCVector3d>& points3d,
		const CCVector3d & givePt, CCVector3d& result, CCVector2d screen2d = CCVector2d());

    static bool getOsnap_ClosestPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale);
    static bool getOsnap_Line_ClosestPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale);
    static bool getOsnap_Circle_ClosestPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale);
    static bool getOsnap_Rectangle_ClosestPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale);
    static bool getOsnap_Arc_ClosestPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, double scale);
    static bool getOsnap_MidPoint(cc2DItemBase* item, const CCVector3d & givePt, CCVector3d& result, CCVector2d screen2d = CCVector2d());

    static bool isOn(cc2DItemBase* item, const CCVector3d& givePt);

    static cc2DItemBase* editor_extend_ex(std::vector<cc2DItemBase*> items, const CCVector3d pos, bool modify = false);

    static bool doArcExtend(cc2DItemBase* item, std::vector<cc2DItemBase*> items, const CCVector3d & pos, bool modify);
    static bool editor_extend_ex_ex(cc2DItemBase* item, std::vector<cc2DItemBase*> items, const CCVector3d& pos, bool modify = false,
		CCVector3d pos3dEntity = CCVector3d(), ccGLCameraParameters params = ccGLCameraParameters());
};
#endif


