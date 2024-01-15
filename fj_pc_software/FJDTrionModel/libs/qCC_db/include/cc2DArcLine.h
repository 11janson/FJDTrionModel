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

#ifndef CC_GL_2DARCLINE_HEADER
#define CC_GL_2DARCLINE_HEADER

//CCCoreLib
#include <Polyline.h>

//Local
#include "ccShiftedObject.h"
#include "ccInteractor.h"
#include "cc2DItemBase.h"

class ccPointCloud;
class ccGenericPointCloud;
class ccLines;
class cc2DRect;
class cc2DRound;
//! Colored polyline
/** Extends the CCCoreLib::Polyline class
**/
class QCC_DB_LIB_API cc2DArcLine : public cc2DItemBase//CCCoreLib::Polyline, public ccShiftedObject, public ccInteractor
{
public:

	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param uniqueID unique ID (handle with care)
	**/
	explicit cc2DArcLine(QString name = "Arc");

	//! Copy constructor
	/** \param poly polyline to copy/clone
	**/
	cc2DArcLine(const cc2DArcLine& poly);

	//! Destructor
	virtual ~cc2DArcLine() override = default;

	virtual bool updatePoints(CCVector3d pt);

	virtual bool addNewPoints(CCVector3d pts);

	virtual bool removePoints();


	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override {return CC_TYPES::LINES;}

	//inherited methods (ccHObject)
	bool isSerializable() const override { return true; }
	bool hasColors() const override;
	void applyGLTransformation(const ccGLMatrix& trans) override;
	unsigned getUniqueIDForDisplay() const override;

	//inherited methods (ccShiftedObject)
	void setGlobalShift(const CCVector3d& shift) override;
	void setGlobalScale(double scale) override;
	const CCVector3d& getGlobalShift() const override;
	double getGlobalScale() const override;

	//! Clones this polyline
	cc2DArcLine* clone() const;

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;
	inline virtual void drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) override
	{
		//DGM: only for 3D polylines!
		if (!is2DMode())
			ccShiftedObject::drawBB(context, col);
	}


	//! Splits the polyline into several parts based on a maximum edge length
	/** \warning output polylines set (parts) may be empty if all the vertices are too far from each other!
		\param[in]	maxEdgeLength	maximum edge length
		\param[out]	parts			output polyline parts
		\return success
	**/
	bool split(	PointCoordinateType maxEdgeLength,
				std::vector<cc2DArcLine*>& parts );

	//! Computes the polyline length
	PointCoordinateType computeLength() const;

	//! Initializes the polyline with a given set of vertices and the parameters of another polyline
	/** \warning Even the 'closed' state is copied as is!
		\param vertices set of vertices (can be null, in which case the polyline vertices will be cloned)
		\param poly polyline
		\return success
	**/
	bool initWith(ccPointCloud*& vertices, const cc2DArcLine& poly);

	//! Copy the parameters from another polyline
	void importParametersFrom(const cc2DArcLine& poly);

	//! Shows an arrow in place of a given vertex
	void showArrow(bool state, unsigned vertIndex, PointCoordinateType length);

	//! Returns the number of segments
	unsigned segmentCount() const;

	//! Samples points on the polyline
	ccPointCloud* samplePoints(	bool densityBased,
								double samplingParameter,
								bool withRGB);

	//! Smoothes the polyline (Chaikin algorithm)
	/** \param ratio between 0 and 0.5 (excluded)
		\param iterationCount of iteration
		\return smoothed polyline
	**/
	cc2DArcLine* smoothChaikin(	PointCoordinateType ratio,
								unsigned iterationCount) const;


	//! Creates a polyline mesh with the selected vertices only
	/** This method is called after a graphical segmentation.
		It creates one or several new polylines with the segments having their two
		vertices tagged as "visible" (see ccGenericPointCloud::visibilityArray).
	**/
	bool createNewPolylinesFromSelection(std::vector<cc2DArcLine*>& output);

	//! Helper to determine if the input cloud acts as vertices of a polyline
	static bool IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, cc2DArcLine** polyline = nullptr);

	virtual bool accpectClickedPoint(void);
    std::vector<CCVector3d> m_threeArcPoints;
    std::vector<CCVector3d> m_drawPoints3d;
    virtual void get3dPoints();
	
	std::vector<CCVector2d> centerIntersectArc(const CCVector2d& center, const CCVector2d& perpendi);

    //获取圆心
    CCVector3d getCenter() { return m_center; }
    //获取半径
    double getRadius() { return m_radius; }
    //获取角度
    double getAngle() { return m_sweepAngle; }

	std::vector<CCVector3d> getCenter3D();

	void addExtrendLinePts(CCVector3d pos) { m_extrendLinePts.push_back(pos); }

	void clearExtrendLinePts() { m_extrendLinePts.clear(); }

	void setExtrendLineShow(bool state) { m_bExtrendLineShow = state; }

	bool isClockwise(CCVector2d p1, CCVector2d p2);

	std::vector<CCVector2d> lineArcIntersections(CCVector2d perpendi);


    virtual std::vector<CCVector2d> intersect(ccLines* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DArcLine* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DRect* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DRound* pItem, bool filter = false);

    std::vector<CCVector2d> sortPointInArc(std::vector<CCVector2d>& lst, CCVector2d start, CCVector2d end);

    void removeNearEndpointIntersections(double z);
public: //meta-data keys
	
	//! Meta data key: vertical direction (for 2D polylines, contour plots, etc.)
	/** Expected value: 0(=X), 1(=Y) or 2(=Z) as int
	**/
	static QString MetaKeyUpDir()			{ return "up.dir"; }
	//! Meta data key: contour plot constant altitude (for contour plots, etc.)
	/** Expected value: altitude as double
	**/
	static QString MetaKeyConstAltitude()	{ return "contour.altitude"; }
	//! Meta data key: profile abscissa along generatrix
	static QString MetaKeyAbscissa()		{ return "profile.abscissa"; }
	//! Meta data key (prefix): intersection point between profile and its generatrix
	/** Expected value: 3D vector
		\warning must be followed by '.x', '.y' or '.z'
	**/
	static QString MetaKeyPrefixCenter()	{ return "profile.center"; }
	//! Meta data key (prefix): generatrix orientation at the point of intersection with the profile
	/** Expected value: 3D vector
		\warning must be followed by '.x', '.y' or '.z'
	**/
	static QString MetaKeyPrefixDirection()	{ return "profile.direction"; }

protected:

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (ccHObject)
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;
    void drawIn2dView(CC_DRAW_CONTEXT& context);
    void drawIn3dView(CC_DRAW_CONTEXT& context);
    void draw_circleArc_in3d_view(CC_DRAW_CONTEXT& context);

    CCVector2d getArcMidpoint(const CCVector2d& p1, const CCVector2d& p2);

	CCVector2d getArcMidPointByPerpendicular(const CCVector2d& p1, const CCVector2d& p2);

	CCVector2d getArcMidpointByClockwise(const CCVector2d& p1, const CCVector2d& p2);

    CCVector2d getActualArcMousePoint(const CCVector2d& p1, const CCVector2d& p2, const CCVector2d& mouse2d);

    void drawArcTrim(CC_DRAW_CONTEXT& context, double z);
    void drawArcNoIntersection(CC_DRAW_CONTEXT& context, double z);
    void drawArcOnly1Intersection(CC_DRAW_CONTEXT& context, double z);
    void drawArcMore1Intersection(CC_DRAW_CONTEXT& context, double z);
    void drawArcAbnormal(CC_DRAW_CONTEXT& context);

	void removePointsOnArc(std::vector<CCVector2d>& lst);
	void removePointsNotOnLine(std::vector<CCVector2d>& lst, const CCVector3d& interPos1, const CCVector3d& interPos2);
public:
	double m_radius;
	double m_xPos;
	double m_yPos;
	double m_zPos;
	double m_startAngle;
	double m_endAngle;
	//绘制方向 true为顺时针 false为逆时针
	bool m_bCross;
	bool m_drawUpdate = true;
	CCVector3d m_center;
    double m_sweepAngle;
	bool m_bExtrendLineShow;
	std::vector<CCVector3d> m_extrendLinePts;
};

#endif //CC_GL_2DARCLINE_HEADER
