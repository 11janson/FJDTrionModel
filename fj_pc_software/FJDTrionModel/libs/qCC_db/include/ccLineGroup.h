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

#ifndef CC_LINEGROUP_HEADER
#define CC_LINEGROUP_HEADER

//Local
#include "ccHObject.h"
#include "ccInteractor.h"
#include "ccGenericGLDisplay.h"
#include "ccShiftedObject.h"

//CCCoreLib
#include <Polyline.h>

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

//! Picking mode
enum PickingMode
{
	POINT_INFO1,
	POINT_POINT_DISTANCE1,
	POINTS_ANGLE1,
	RECT_ZONE1,

	//janson.yang 2022.5.9
	POINT_HEIGHT1,
	POINT_AERA1,
	POINT_VOLUME1
};

//! 2D label (typically attached to points)
class QCC_DB_LIB_API ccLineGroup : public ccInteractor, public CCCoreLib::Polyline, public ccShiftedObject
{
public:
	//! Default constructor
	ccLineGroup(GenericIndexedCloudPersist* associatedCloud, unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID, QString name = QString("lineGroup"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::LINEGROUP; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	//! Gets label content (as it will be displayed)
	/** \param precision displayed numbers precision
		\return label body (one string per line)
	**/
	QStringList getLabelContent(int precision) const;

	//! Returns the (3D) label title
	/** \param precision displayed numbers precision
		\return label title
	**/
	QString getTitle(int precision, CC_DRAW_CONTEXT& context) const;

	//inherited from ccInteractor
	virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

	//! Sets relative position
	void setPosition(float x, float y);

	//! Returns relative position
	inline const float* getPosition() const { return m_screenPos; }

	//! Clears label
	void clear(bool ignoreDependencies = false);

	//! Returns current size
	inline unsigned size() const { return static_cast<unsigned>(m_pickedPoints.size()); }

	//! Adds a point to this label
	/** Adding a point to a label will automatically make it 'mutate'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPickedPoint(ccGenericPointCloud* cloud, unsigned pointIndex, bool entityCenter = false);

	//! Adds a point to this label
	/** Adding a point to a label will automatically make it 'mutate'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPickedPoint(ccGenericMesh* mesh, unsigned triangleIndex, const CCVector2d& uv, bool entityCenter = false);

	bool addPickedPoint(ccLineGroup* label, unsigned pointIndex);

	//! Whether to collapse label or not
	inline void setCollapsed(bool state) { m_showFullBody = !state; }

	//! Returns whether the label is collapsed or not
	inline bool isCollapsed() const { return !m_showFullBody; }

	//! Whether to display the point(s) legend (title only)
	inline void displayPointLegend(bool state) { m_dispPointsLegend = state; }

	//! Returns whether the point(s) legend is displayed
	inline bool isPointLegendDisplayed() const { return m_dispPointsLegend; }

	//! Whether to display the label in 2D
	inline void setDisplayedIn2D(bool state) { m_dispIn2D = state; }

	//! Returns whether the label is displayed in 2D
	inline bool isDisplayedIn2D() const { return m_dispIn2D; }

	//janson.yang 2022.5.9
	void setPickingMode(PickingMode pickingMode) { m_pickingMode = pickingMode; }
	void setccBBox(ccBBox* box) { 
		if (box)
		{
			m_box = box; /*m_box->setValidity(false); */setPosition(m_box->position2d_text.x, m_box->position2d_text.y);
		}
	 }

	void saveBBox() { 
		if (m_box)
		{
			ccBBox* savebox = new ccBBox;
			*savebox = *m_box;
			m_box = savebox;
			isShowBox = true;
		}
		else
			isShowBox = false;
	}
	bool isShowBox;

	//! Picked point descriptor
	/** Label 'points' can be shared between multiple entities
	**/
	struct QCC_DB_LIB_API PickedPoint
	{
		//! Cloud
		ccGenericPointCloud* _cloud;
		//! Mesh
		ccGenericMesh* _mesh;
		//! Point/triangle index
		unsigned index;
		//! Last known '2D' position (i.e. in screen space)
		/** This position is updated on each call to drawMeOnly3D
		**/
		CCVector3d pos2D;
		//! Last known marker scale
		float markerScale;
		//! Barycentric coordinates (for triangles)
		CCVector2d uv;
		//! Entity center mode (index will be invalid)
		bool entityCenterPoint;

		//! Returns the point position (3D)
		CCVector3 getPointPosition() const;
		//! Returns the cloud or the mesh vertices
		ccGenericPointCloud* cloudOrVertices() const;
		//! Returns the cloud or the mesh unique ID
		unsigned getUniqueID() const;
		//! Returns the associated entity (cloud or mesh)
		ccHObject* entity() const;
		//! Returns the item 'title' (either its index or 'Center' if it's a center point)
		QString itemTitle() const;
		//! Returns the point prefix ('Point' or 'Point@Tri' or 'IDXX Center')
		QString prefix(const char* pointTag) const;

		//! Default constructor
		PickedPoint()
			: _cloud(nullptr)
			, _mesh(nullptr)
			, index(0)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(0, 0)
			, entityCenterPoint(false)
		{}

		//! Constructor from a point and its index
		PickedPoint(ccGenericPointCloud* _cloud, unsigned pointIndex, bool centerPoint = false)
			: _cloud(_cloud)
			, _mesh(nullptr)
			, index(pointIndex)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(0, 0)
			, entityCenterPoint(centerPoint)
		{}

		//! Constructor from a triangle, its index and barycentric coordinates
		PickedPoint(ccGenericMesh* _mesh, unsigned triIindex, const CCVector2d& _uv, bool centerPoint = false)
			: _cloud(nullptr)
			, _mesh(_mesh)
			, index(triIindex)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(_uv)
			, entityCenterPoint(centerPoint)
		{}
	};

	//! Adds a point to this label (direct - handle with care)
	bool addPickedPoint(const PickedPoint& pp);

	//! Returns a given point
	inline const PickedPoint& getPickedPoint(unsigned index) const { return m_pickedPoints[index]; }

	//! Sets marker (relative) scale
	/** Default value: 1.0
	**/
	inline void setRelativeMarkerScale(float scale) { m_relMarkerScale = scale; }

	//! Point (marker) picking
	bool pointPicking(	const CCVector2d& clickPos,
						const ccGLCameraParameters& camera,
						int& nearestPointIndex,
						double& nearestSquareDist) const;

protected:

	//! One-point label info
	struct LabelInfo1
	{
		bool hasNormal;
		CCVector3 normal;
		bool hasRGB;
		ccColor::Rgba color;
		bool hasSF;
		ScalarType sfValue;
		double sfShiftedValue;
		bool sfValueIsShifted;
		QString sfName;
		//! Default constructor
		LabelInfo1()
			: hasNormal(false)
			, normal(0, 0, 0)
			, hasRGB(false)
			, color(0, 0, 0, 0)
			, hasSF(false)
			, sfValue(0)
			, sfShiftedValue(0)
			, sfValueIsShifted(false)
		{}
	};
	
	//! Returns one-point label info
	void getLabelInfo1(LabelInfo1& info) const;
	
	//! Returns the SF value as a string
	/** Handles:
		- NaN values
		- shifted SF
	**/
	static QString GetSFValueAsString(const LabelInfo1& info, int precision);

	//! Two-points label info
	struct LabelInfo2
	{
		CCVector3 diff;
		//! Default constructor
		LabelInfo2()
			: diff(0, 0, 0)
		{}
	};
	//! Gets two-points label info
	void getLabelInfo2(LabelInfo2& info) const;

	//! Three-points label info
	struct LabelInfo3
	{
		CCVector3 normal;
		PointCoordinateType area;
		CCVector3d angles;
		CCVector3d edges;
		//! Default constructor
		LabelInfo3()
			: normal(0, 0, 0)
			, area(0)
			, angles(0, 0, 0)
			, edges(0, 0, 0)
		{}
	};
	//! Gets three-points label info
	void getLabelInfo3(LabelInfo3& info) const;

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	virtual void onDeletionOf(const ccHObject* obj) override;

	//! Draws the entity only (not its children) - 2D version
	void drawMeOnly2D(CC_DRAW_CONTEXT& context);
	//! Draws the entity only (not its children) - 3D version
	void drawMeOnly3D(CC_DRAW_CONTEXT& context);

	//! Picked points
	std::vector<PickedPoint> m_pickedPoints;

	//! Updates the label 'name'
	void updateName();

	//! Whether to show full label body or not
	bool m_showFullBody;

	//! label ROI
	/** ROI is displayed relatively to m_screenPos
		(projected in displayed screen space). m_screenPos
		corresponds to its top-left corner. So ROI[1] is
		the distance to the upper border and ROI[3] is the
		distance to the lower border (relatively to m_screenPos).
	**/
	QRect m_labelROI;

	//! close button ROI
	//int m_closeButtonROI[4];

	//! Label position (percentage of screen size)
	float m_screenPos[2];

	//! Label position at last display (absolute)
	int m_lastScreenPos[2];

	//! Whether to display the point(s) legend
	bool m_dispPointsLegend;

	//! Whether to display the label in 2D
	bool m_dispIn2D;

	//! Relative marker scale
	float m_relMarkerScale;

	//! Current picking mode
    PickingMode m_pickingMode;
	ccBBox* m_box;//绘制长方体体积测量框
	

	std::vector<double> m_lengthunitshift{1.0,10.0,100.0,1000.0,0.001,0.000621,3.280839,39.370078};
	std::vector<double> m_angleunitshift{1.0,60.0,3600.0};
	std::vector<double> m_areaunitshift{1.0,100.0,10000.0,1000000.0,10.763910,1550.0031};
	std::vector<double> m_volumnunitshift{ 1.0,1000.0,1000000.0,1000000000.0,35.31472,61023.844502 };

	QStringList m_diameterlist = (QStringList() << u8"m" << u8"dm" << u8"cm" << u8"mm" << u8"km" << u8"mile" << u8"ft" << u8"in");
	QStringList m_anglelist = (QStringList() << u8"°" << u8"′" << u8"″");
	QStringList m_arealist = (QStringList() << u8"m²" << u8"dm²" << u8"cm²" << u8"mm²" << u8"ft²" << u8"in²");
	QStringList m_volumelist = (QStringList() << u8"m³" << u8"dm³" << u8"cm³" << u8"mm³" << u8"ft³" << u8"in³");
};

#endif //CC_LINEGROUP_HEADER
