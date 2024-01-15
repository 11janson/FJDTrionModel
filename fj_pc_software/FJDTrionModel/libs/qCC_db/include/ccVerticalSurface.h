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

/******************************************************************************
@brief : �и�ƽ�湦��ͷ�ļ� *
@author : Mike.Li *
@date : 2022/7/26 15:55 *
@version : ver 1.0 *
@inparam :  *
@outparam :  
*****************************************************************************/

#ifndef CC_VERTICALSURFACE_HEADER
#define CC_VERTICALSURFACE_HEADER

//Local
#include "ccDrawableObject.h"

//CCCoreLib

#include "ccHObject.h"
#include "ccBBox.h"
#include "ccGenericPointCloud.h"
#include "qCC_db.h"

#include <BoundingBox.h>
#include "ccGenericPrimitive.h"
#include "ccPlanarEntityInterface.h"


//! Bounding box structure
/** Supports several operators such as addition (to a matrix or a vector) and
	multiplication (by a matrix or a scalar).
**/
class QCC_DB_LIB_API ccVerticalSurface : public ccGenericPrimitive, public ccPlanarEntityInterface/*: public ccHObject*/
{
public:
	enum EPivotType
	{
		XAXIS = 1,
		YAXIS,
		ZAXIS
	};
	//! Default constructor
	ccVerticalSurface(const ccGLMatrix* transMat = nullptr, QString name = QString("Surface"));

	//! Draws bounding box (OpenGL)
	/** \param context OpenGL context
	 *  \param col (R,G,B) color
	**/
	void draw(CC_DRAW_CONTEXT& context/*, ccBBox *pBox*/);

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SURFACE; }

    //inherited from ccGenericPrimitive
    virtual QString getTypeName() const override { return "Surface"; }

	void setWidth(float width) { width_ = width; }//�����߿�

    CCVector3 getMinCorner() { return m_Mincorner; }
    CCVector3 getMaxCorner() { return m_Maxcorner; }


	bool pointIsInside(const CCVector3d& p);

	bool contains(const CCVector3d& P) const
	{
		return (P.x >= m_Mincorner.x && P.x <= m_Maxcorner.x &&
			P.y >= m_Mincorner.y && P.y <= m_Maxcorner.y &&
			P.z >= m_Mincorner.z && P.z <= m_Maxcorner.z);
	}

	void addKeyPoints(CCVector3d pos) { m_KeyPoints.push_back(pos); }

    void clearKeyPoints() { m_KeyPoints.clear(); }

    int getKeyPointsSize() { return m_KeyPoints.size(); }


    //inherited from ccPlanarEntityInterface
    CCVector3 getNormal() const override { return m_transformation.getColumnAsVec3D(2); }

    virtual ccGenericPrimitive* clone() const override;
	// 更新平面位置
	void updatePos(double value) { m_SurfacePos = value; m_DoubleClicked = false; }
	// 更新平面厚度
	void updateHeight(double value) { m_SurfaceHeight = value; }
	// 更新平面垂直方向
	void updateSliceDirection(EPivotType type) { m_SliceType = type; }
	// 更新平面位置
	void updateSlicePos(CCVector3d pos);
	// 设置切割平面包围盒
	void setCalcBBox(ccBBox& box) { m_CalcBBox = box; }
	// 获取当前平面位置
	CCVector3d& currentPos() { return m_pos; }
	// 设置平面开启Z-Fighting
	void setOpenZFighting(bool open) { m_open = open; }
	// 设置平面缩放,基础尺寸为1,扩大10%即为1.1
	void setScaling(float scaling);
    //获取平面单位法向量
	CCVector3d getNormalPlane() { return normalPlane; }
    //获取相机观察up单位向量
	CCVector3d getNewNormal() { return newNormal; }

    CCVector3d getTopPlaneCenter() { return m_topPlaneCenterPoint; }

    CCVector3d m_topPlaneCenterPoint;
	CCVector3d pos2D;
	CCVector3d position2d_text;

protected:
    bool buildUp() override;

private:
	void initShader(CC_DRAW_CONTEXT& context);

	void calcMinMaxCorner();
private:
	float width_;
	double m_SurfaceHeight;
	double m_SurfacePos;

	//! Clipping box
	ccBBox m_BBox;
    ccBBox m_CalcBBox;

	CCVector3 m_Mincorner;
	CCVector3 m_Maxcorner;
	
	bool exec;

	EPivotType m_SliceType;
	CCVector3d m_pos;
	float m_XOffset;
	float m_YOffset;
	float m_ZOffset;


	bool m_DoubleClicked;
	float m_halfBoxSize;
    bool m_open;
    float m_scaling;
    bool m_scalingOpen;

	std::vector<CCVector3d> m_KeyPoints;
	std::vector<CCVector3d> m_BoxVertexs;

	//垂直屏幕平面单位法线向量
	CCVector3d normalPlane;
	//垂直平面法向量、平行与平面
	CCVector3d newNormal;

};

#endif //CC_VERTICALSURFACE_HEADER
