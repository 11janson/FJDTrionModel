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

#ifndef CC_GL_2DITEMBASE_HEADER
#define CC_GL_2DITEMBASE_HEADER

//CCCoreLib
#include <Polyline.h>

//qt
#include <QVector3D>
#include <QDebug>

//Local
#include "ccShiftedObject.h"
#include "ccPoint.h"
#include "ccInteractor.h"

#include "ccGenericGLDisplay.h"

//CGAL
#include <CGAL/Circle_2.h>
#include <CGAL/Line_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Line_arc_2.h>
#include <CGAL/Circular_arc_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Circular_kernel_2/Circular_arc_point_2.h>
#include <CGAL/intersections.h>
#include <CGAL/enum.h>

typedef CGAL::Exact_circular_kernel_2 K;
typedef CGAL::Point_2<K>                 Point_2;
typedef CGAL::Circle_2<K>                Circle_2;
typedef CGAL::Line_2<K>                  Line_2;
typedef CGAL::Segment_2<K>               Segment_2;
typedef CGAL::Polygon_2<K>               Polygon_2_K;
typedef CGAL::Circular_arc_2<K>          Circular_arc_2;
typedef CGAL::Line_arc_2<K>              Line_arc_2;
typedef CGAL::Circular_arc_point_2<K>    Circular_arc_point_2;

class ccPointCloud;
class ccGenericPointCloud;
class ccLines;
class cc2DRect;
class cc2DRound;
class cc2DArcLine;
const float EPSINON = 0.00000001;
#define DELTA 8
#define LABELMARKSIZE 0.002

//! Colored polyline
/** Extends the CCCoreLib::Polyline class
**/
class QCC_DB_LIB_API cc2DItemBase : public CCCoreLib::Polyline, public ccShiftedObject, public ccInteractor
{
public:
	enum EGraphicType
	{
		NONE = 0,		  //
		LINE = 1,		  //线段
		LINES = 2,		  //折线
		ARC = 3,		  //弧线
		RECTACROSS = 4,	  //对角矩形
		RECT3 = 5,		  //三点矩形
		ROUND3 = 6,	  //三点圆
		ROUND2 = 7,	  //两点圆
		ROUNDRADIUS = 8,//半径圆
	};

	enum  EAxisType
	{
		XAXIS = 1,
		YAXIS,
		ZAXIS,
        ANY
	};

    enum OsnapMode
    {
        kKeyPoint = 0x00,
        kMidPoint = 0x01,
        kNearstPoint = 0x02,
		kCenterpoint = 0x03,
		kAnyPoint = 0x04
    };
	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param uniqueID unique ID (handle with care)
	**/
	explicit cc2DItemBase(QString name = QString());

	//! Copy constructor
	/** \param poly polyline to copy/clone
	**/
	//cc2DItemBase(const cc2DItemBase& poly);

	//! Destructor
	virtual ~cc2DItemBase() override = default;

	virtual bool updatePoints(CCVector3d pts);

	virtual bool addNewPoints(CCVector3d pts);

    /**
    *@brief 清清除点数据
    */
    virtual int clearDrawGraphicsPoints(void);

    /**
    *@brief 获取绘制控制点坐标
    */
    virtual std::vector<CCVector3d> getDrawGraphicsPoints(void);

    /**
    *@brief 更新绘或者控制点坐标
    */
    virtual int setUpdateDrawGraphicsPoints(std::vector<CCVector3d> list);

    /**
    *@brief 是否继续接收鼠标点
    */
    virtual bool accpectClickedPoint(void);

	void addDisplayPoints(CCVector3d pts);

    virtual bool removePoints();

    virtual void getdrawPoints3d();

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override {return CC_TYPES::OBJECT;}

	//inherited methods (ccHObject)
	virtual bool isSerializable() const override { return true; }
	virtual bool hasColors() const override{return false;}
	virtual void applyGLTransformation(const ccGLMatrix& trans) override{return;}
    virtual unsigned getUniqueIDForDisplay() const ;

	//inherited methods (ccShiftedObject)
	virtual void setGlobalShift(const CCVector3d& shift) override{return;}
	virtual void setGlobalScale(double scale) override{return;}
	virtual const CCVector3d& getGlobalShift() const override{ return CCVector3d(); }
	virtual double getGlobalScale() const override{ return 0.0;}

	//! Clones this polyline
    virtual cc2DItemBase* clone();

	//! Defines if the polyline is considered as 2D or 3D
	/** \param state if true, the polyline is 2D
	**/
	void set2DMode(bool state) {m_mode2D = state;}

	//! Returns whether the polyline is considered as 2D or 3D
	inline bool is2DMode() const { return m_mode2D; }

	//! Defines if the polyline is drawn in background or foreground
	/** \param state if true, the polyline is drawn in foreground
	**/
	void setForeground(bool state) {m_foreground = state;}

	//! Sets the polyline color
	/** \param col RGB color
	**/
	inline void setColor(const ccColor::Rgbaf& col) { m_rgbColor = col; }

	//! Sets the width of the line
	/**  \param width the desired width
	**/
	void setWidth(PointCoordinateType width) {m_width = width;}

	//! Returns the width of the line
	/** \return the width of the line in pixels
	**/
	inline PointCoordinateType getWidth() const { return m_width; }

	//! Returns the polyline color
	/** \return a pointer to the polyline RGB color
	**/
	inline const ccColor::Rgbf& getColor() const { return m_rgbColor; }

	//! Sets whether to display or hide the polyline vertices
	void showVertices(bool state) { m_showVertices = state; }
	//! Whether the polyline vertices should be displayed or not
	bool verticesShown() const { return m_showVertices; }

	//! Sets the width of vertex markers
	void setVertexMarkerWidth(int width) { m_vertMarkWidth = width; }
	//! Returns the width of vertex markers
	int getVertexMarkerWidth() const { return m_vertMarkWidth; }

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override{return ccBBox();}
	inline virtual void drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) override{return;}

    //设置图元类型
	void setItemType(EGraphicType type) { m_Type = type; }
	//获取图元类型
	EGraphicType getItemType() { return m_Type; }

	//设置控制点是否可见
	void setPointsVisble(bool visible);
    //设置是否第一次添加点
    void setIsFirst(bool state) { m_first = state; }
    void setIsFinish(bool state) { m_first = state; }

    void copyTransformPos(CCVector3d pos);

    void setViewMatWhenStartDrawing(ccGLMatrixd mat) { m_viewMat = mat; }

    ccGLMatrixd getViewMatWhenStartDrawing() { return m_viewMat; }

    std::vector<CCVector3d>& get3dCalcPoints() { return m_keyPoints3d; }

    void setHoverSelect(bool state);

    virtual void setSelected(bool state);


    void setTrimMouse3DPos(CCVector3d mouse3Dpos);

    void setTrimEnable(bool state) { m_bTrim = state; }

    bool getTrimEnable() { return m_bTrim; }

    void setAbnormal(bool state) { m_bAbnormal = state; }

    bool abnormal() { return m_bAbnormal; }

    void clearIntersections() { m_lstIntersections.clear(); }

    void addIntersections(CCVector2d intersection) { m_lstIntersections.push_back(CCVector3d(intersection.x, intersection.y, m_mouse3dpos[0].z)); }

    void sortVector3d(std::vector<CCVector3d>& lst, double z, bool order = true);

    static bool is_point_on_segment(const Point_2& p, const Segment_2& s);

	static bool on_parallel(cc2DItemBase * item0, cc2DItemBase* item1);

    static bool on_segment(const CCVector3d& mousePos, const CCVector3d& interPos1, const CCVector3d& interPos2);
public:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override{return false;}
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override{return false;}

	//inherited methods (ccHObject)
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

    static std::vector<CCVector3d> projectTo2d(const std::vector<CCVector3d>& points3d, const ccGLCameraParameters camera);
    static std::vector<CCVector3d> unprojectTo3d(const std::vector<CCVector3d>& points3d, const ccGLCameraParameters camera);

    virtual void get3dPoints();

    virtual void get2dPoints();

    virtual bool getOsnapPonits(const CCVector3d& pt, OsnapMode mode, CCVector3d& result, double scale, CCVector2d& screen2d);

    void setMoveGripPoint(CCVector3d pt);

    void bakOrginal(bool needBak = false);

	void setPointColor(ccColor::Rgbaf color) { m_pointColor = color; }

    std::vector<CCVector3d>& get2dKeyPoints() { return m_keyPoints2d; }
    std::vector<CCVector3d>& get3dKeyPoints() { return m_keyPoints3d; }
    std::vector<CCVector3d>& getIntersectionList() { return m_lstIntersections; }

    static bool segmentIntersect(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& p3);
	static bool lineIntersect(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& p3);


    CCVector2d getPerpendicularPoint(const CCVector2d& p0, const CCVector2d& p1, const CCVector2d& p2);

public:
    virtual std::vector<CCVector2d> intersect(ccLines* pItem, bool filter = false) { return std::vector<CCVector2d>(); }

    virtual std::vector<CCVector2d> intersect(cc2DArcLine* pItem, bool filter = false) { return std::vector<CCVector2d>(); }

    virtual std::vector<CCVector2d> intersect(cc2DRect* pItem, bool filter = false) { return std::vector<CCVector2d>(); }

    virtual std::vector<CCVector2d> intersect(cc2DRound* pItem, bool filter = false) { return std::vector<CCVector2d>(); }

    std::vector<CCVector3d> getUnselectList() { return m_pt2dUnselect; }
    std::vector<CCVector3d> getUnselect2List() { return m_pt2dUnselect2; }
    std::vector<CCVector3d> getUnselect3List() { return m_pt2dUnselect3; }
    std::vector<CCVector3d> getUnselect4List() { return m_pt2dUnselect4; }

    void setParticipatingIntersection(bool state) { m_bParticipatingIntersection = state; }
    bool participatingIntersection() { return m_bParticipatingIntersection; }

    void setExtendInterect(bool state) { m_bExtendInterect = state; }
public:
	//! Unique RGB color
	ccColor::Rgbaf m_rgbColor;

	//! Width of the line
	double m_width;

	//! Whether polyline should be considered as 2D (true) or 3D (false)
	bool m_mode2D;

	//! Whether polyline should draws itself in background (false) or foreground (true)
	bool m_foreground;
	
	//! Whether vertices should be displayed or not
	bool m_showVertices;

	//! Vertex marker width
	int m_vertMarkWidth;

    //！ item type
	EGraphicType m_Type;

	//! first add tow points
	bool m_first;

	//! finish this item
	bool m_finish;

    int m_OsnapFlag = -1;

    ccGLMatrixd m_viewMat;
    std::vector<CCVector3d> m_keyPoints3d;
    std::vector<CCVector3d> m_keyPoints2d;
    int m_gripPointIndex;

    std::vector<CCVector3d> m_keyPoints3d_orginal, m_keyPoints2d_orginal, m_pointsTmp, m_pointsTmp2d;
    CCVector3d m_ptOrginal;

    std::vector<CCVector3d> m_ptOsnapPoints, m_ptOsnampsTemp2d;
	std::vector<CCVector2d> m_ptScreen2dPoints;


    void drawLines2D(CC_DRAW_CONTEXT & context, const std::vector<CCVector3d>& pts, ccColor::Rgbaf color, bool isDashed = false);
    const ccGLCameraParameters& get3dCamera() { return m_3dCamera; }
public:
    std::vector<CCVector3d> m_pt2dUnselect;
    std::vector<CCVector3d> m_pt2dUnselect2;
    std::vector<CCVector3d> m_pt2dUnselect3;
    std::vector<CCVector3d> m_pt2dUnselect4;

private:
    void drawMeOnly3D(CC_DRAW_CONTEXT & context);
    void drawMeOnly2D(CC_DRAW_CONTEXT & context);

protected:
    std::vector<CCVector3d> m_lstIntersections;
    bool m_bTrim;
	bool m_bHoverSelect;
    bool m_bSort = false;
    bool m_bAbnormal = false;
	ccColor::Rgbaf m_pointColor;
    std::vector<CCVector3d> m_mouse3dpos;
    std::vector<CCVector3d> m_mouse2dpos;

    ccGLCameraParameters m_3dCamera;

    //设置是否参与求交运算
    bool m_bParticipatingIntersection;
    //延伸求交
    bool m_bExtendInterect;
};

#endif //CC_GL_2DITEMBASE_HEADER
