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

#ifndef CC_GL_LINES_HEADER
#define CC_GL_LINES_HEADER

//CCCoreLib
#include <Polyline.h>

//Local
#include "ccShiftedObject.h"
#include "ccInteractor.h"

//qt 
#include <QFont>

class ccPointCloud;
class ccGenericPointCloud;
class ccGLCameraParameters;
//! Colored polyline
/** Extends the CCCoreLib::Polyline class
**/
class QCC_DB_LIB_API ccText : public CCCoreLib::Polyline, public ccShiftedObject//, public ccShiftedObject, public ccInteractor
{
public:

	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccText(GenericIndexedCloudPersist* associatedCloud, QString name = "text");

	//! Copy constructor
	/** \param poly polyline to copy/clone
	**/
    ccText(const ccText& poly);

	//! Destructor
	virtual ~ccText() override = default;

    //set paint data
    void setTextData(QString str) { m_str = str; }

    //set scale factor
    void setScale(double scale) { m_dScale = scale; }

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
    ccText* clone() const;

    //! Defines if the polyline is considered as 2D or 3D
    /** \param state if true, the polyline is 2D
    **/
    void set2DMode(bool state);

    //! Defines if the polyline is drawn in background or foreground
    /** \param state if true, the polyline is drawn in foreground
    **/
    void setForeground(bool state);

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;
	//inline virtual void drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) override
	//{
	//	//DGM: only for 3D polylines!
	//	if (!is2DMode())
	//		ccShiftedObject::drawBB(context, col);
	//}


	//! Splits the polyline into several parts based on a maximum edge length
	/** \warning output polylines set (parts) may be empty if all the vertices are too far from each other!
		\param[in]	maxEdgeLength	maximum edge length
		\param[out]	parts			output polyline parts
		\return success
	**/
	bool split(	PointCoordinateType maxEdgeLength,
				std::vector<ccText*>& parts );

	//! Computes the polyline length
	PointCoordinateType computeLength() const;


	//! Initializes the polyline with a given set of vertices and the parameters of another polyline
	/** \warning Even the 'closed' state is copied as is!
		\param vertices set of vertices (can be null, in which case the polyline vertices will be cloned)
		\param poly polyline
		\return success
	**/
	bool initWith(ccPointCloud*& vertices, const ccText& poly);

	//! Copy the parameters from another polyline
	void importParametersFrom(const ccText& poly);

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
    ccText* smoothChaikin(	PointCoordinateType ratio,
								unsigned iterationCount) const;


	//! Creates a polyline mesh with the selected vertices only
	/** This method is called after a graphical segmentation.
		It creates one or several new polylines with the segments having their two
		vertices tagged as "visible" (see ccGenericPointCloud::visibilityArray).
	**/
	bool createNewPolylinesFromSelection(std::vector<ccText*>& output);

	//! Helper to determine if the input cloud acts as vertices of a polyline
	static bool IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, ccText** polyline = nullptr);


public:
    bool m_isClosed;

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
    void drawIn3dView(CC_DRAW_CONTEXT& context);
    void drawIn2dView(CC_DRAW_CONTEXT& context);

    void renderText(CC_DRAW_CONTEXT& context, ccGLCameraParameters camera, double x, double y, double z,
        const QString & str, uint16_t uniqueID = 0, const QFont & pFont = QFont(), const ccColor::Rgba* backcolo = nullptr);

protected:
	std::vector<CCVector2d> vertexs;

    //! Unique RGB color
    ccColor::Rgbaf m_rgbColor;

    using SharedTexture = QSharedPointer< QOpenGLTexture>;
    //! Reserved textures (for renderText)
    QMap<uint16_t, SharedTexture> m_uniqueTextures;

    //! Texture pool (for renderText)
    std::vector<SharedTexture> m_texturePool;

    //! Last texture pool index
    size_t m_texturePoolLastIndex;

    //! Whether polyline should be considered as 2D (true) or 3D (false)
    bool m_mode2D;

    //! Whether polyline should draws itself in background (false) or foreground (true)
    bool m_foreground;

    QString m_str;

    double m_dScale;
};

#endif //CC_GL_LINES_HEADER