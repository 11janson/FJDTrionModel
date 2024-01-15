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

#ifndef __CCSECTIONANALYSISLINE_H__
#define __CCSECTIONANALYSISLINE_H__

//Local
#include "ccHObject.h"
#include "ccInteractor.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
#include <QObject>

class ccGenericPointCloud;
class ccGenericMesh;

enum ProjectionDirection
{
	LOOKDOWN,  //俯视图投影
	LOOKFACE   //正视图投影
};

//! 2D label (typically attached to points)
class QCC_DB_LIB_API ccSectionAnalysisLine : public ccHObject, public ccInteractor
{
public:
	//! Default constructor
	ccSectionAnalysisLine(QString name = QString("label"));

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::RECT_3D; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }


	//inherited from ccInteractor
	virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

	void setCurrentData(const std::vector<CCVector3d> & currentPointData);
	void getCurrentData(std::vector<CCVector3d> & currentPointData);
	void setCurrentColor(QColor color) { m_color = color; }

	void setPickPoint(const QPointF & startPoint,const QPointF & endPoint);
	void getPickPoint(QPointF & startPoint,QPointF & endPoint);

	bool getIsPickPoint();
	void setIsPickPoint(bool ispick);

	bool getIsMeasureOpen();
	void setIsMeasureOpen(bool ispick);

	double getThickness();
	void setThickness(const double &  value);

    CCVector3d getOffset();
	void setOffset(const CCVector3d & value);

	void setRangeParam(const double &  heightMin, const double &  heightMax);

	void getCurrentPointDataBoxVertex(std::vector <std::vector<QPointF>> & data);

	//! Point (marker) picking
	bool pointPicking(const QPointF& clickPos,const ccGLCameraParameters& camera);

	void changePosition(QPointF clickpos, const ccGLCameraParameters& camera);

	void setMousePos(const std::vector<CCVector3d> & mousePoint);

	void setMousePosVisiable(bool isShow);
	bool getMousePosVisiable();

	void setProjectionMode(ProjectionDirection projectionMode);
	ProjectionDirection getProjectionMode();

	std::vector<CCVector3d> getCurrentOffsetPointData();

	void setIsSelected(bool isselect);
	bool getIsSelected();
protected:


	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	virtual void onDeletionOf(const ccHObject* obj) override;

	//! Draws the entity only (not its children) - 2D version
	void drawMeOnly2D(CC_DRAW_CONTEXT& context);
	//! Draws the entity only (not its children) - 3D version
	void drawMeOnly3D(CC_DRAW_CONTEXT& context);

private:
	QColor m_color = QColor(Qt::red);
	double m_thickness = 0.1;
    CCVector3d m_offset = CCVector3d(0, 0, 0);
	QPointF m_startPoint;
	QPointF m_endPoint;
	bool m_ispickPoint = false;
	double m_heightMin = 0;
	double m_heightMax = 0;
	std::vector<CCVector3d> m_CurrentPointData;
	int m_movePart = 0;
	bool m_moveUpPart = true;
    std::vector<CCVector3d> m_mousePoint;  
	bool m_isShowMousePos = false;
	bool m_isSelected = true;
	ProjectionDirection m_ProjectionMode = LOOKDOWN;
	bool m_isMeasureOpen = true;
};

#endif //CC_2D_LABEL_HEADER
