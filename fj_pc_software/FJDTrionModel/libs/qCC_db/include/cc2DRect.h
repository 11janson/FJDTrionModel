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

#ifndef CC_GL_2DRECT_HEADER
#define CC_GL_2DRECT_HEADER

//CCCoreLib
#include <Polyline.h>

//Local
#include "ccShiftedObject.h"
#include "ccInteractor.h"
#include "cc2DItemBase.h"
#include "ccGenericGLDisplay.h"


class ccPointCloud;
class ccGenericPointCloud;
class ccPoint;
class ccLines;
class cc2DArcLine;
class cc2DRound;
//! Colored polyline
/** Extends the CCCoreLib::Polyline class
**/
class QCC_DB_LIB_API cc2DRect : public cc2DItemBase//CCCoreLib::Polyline, public ccShiftedObject, public ccInteractor
{
public:
	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param uniqueID unique ID (handle with care)
	**/
	explicit cc2DRect(QString name = "Rect");

	//! Copy constructor
	/** \param poly polyline to copy/clone
	**/
	cc2DRect(const cc2DRect& poly);

	//! Destructor
	virtual ~cc2DRect() override = default;

	virtual bool updatePoints(CCVector3d pts);

	virtual bool addNewPoints(CCVector3d pts);

	virtual bool removePoints();

    virtual void get3dPoints();
    void get2dPoints(const ccGLCameraParameters camera);
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
	cc2DRect* clone() const;

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
				std::vector<cc2DRect*>& parts );

	//! Computes the polyline length
	PointCoordinateType computeLength() const;

	//! Initializes the polyline with a given set of vertices and the parameters of another polyline
	/** \warning Even the 'closed' state is copied as is!
		\param vertices set of vertices (can be null, in which case the polyline vertices will be cloned)
		\param poly polyline
		\return success
	**/
	bool initWith(ccPointCloud*& vertices, const cc2DRect& poly);

	//! Copy the parameters from another polyline
	void importParametersFrom(const cc2DRect& poly);

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
	cc2DRect* smoothChaikin(	PointCoordinateType ratio,
								unsigned iterationCount) const;


	//! Creates a polyline mesh with the selected vertices only
	/** This method is called after a graphical segmentation.
		It creates one or several new polylines with the segments having their two
		vertices tagged as "visible" (see ccGenericPointCloud::visibilityArray).
	**/
	bool createNewPolylinesFromSelection(std::vector<cc2DRect*>& output);

	//! Helper to determine if the input cloud acts as vertices of a polyline
	static bool IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, cc2DRect** polyline = nullptr);

	virtual bool accpectClickedPoint(void);

    std::vector<CCVector3d> getDrawPoints() { return m_drawPoints2d; }

    void clearDrawPoints() { return m_drawPoints2d.clear(); }

    virtual std::vector<CCVector2d> intersect(ccLines* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DArcLine* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DRect* pItem, bool filter = false);

    virtual std::vector<CCVector2d> intersect(cc2DRound* pItem, bool filter = false);
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
    std::vector<CCVector3d> m_drawPoints3d;
    std::vector<CCVector3d> m_drawPoints2d;
    void getDrawPoints2d(const ccGLCameraParameters camera);

protected:
	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (ccHObject)
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

    void drawIn2dView(CC_DRAW_CONTEXT& context);
    void drawIn3dView(CC_DRAW_CONTEXT& context);

    void drawRectTrim(CC_DRAW_CONTEXT& context);
    void drawSelectivePart(CC_DRAW_CONTEXT& context, std::vector<CCVector3d>&lst, int index1, int index2);
    void drawRectNormal(CC_DRAW_CONTEXT& context);


};

#endif //CC_GL_2DRECT_HEADER
