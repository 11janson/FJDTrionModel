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
@brief : 切割平面功能头文件 *
@author : Mike.Li *
@date : 2022/7/26 15:55 *
@version : ver 1.0 *
@inparam :  *
@outparam :  
*****************************************************************************/

#ifndef CC_SURFACE_HEADER
#define CC_SURFACE_HEADER

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
class QCC_DB_LIB_API ccSurface : public ccGenericPrimitive, public ccPlanarEntityInterface/*: public ccHObject*/
{
public:
	enum EPivotType
	{
		XAXIS = 1,
		YAXIS,
		ZAXIS
	};
	//! Default constructor
	ccSurface(const ccGLMatrix* transMat = nullptr, QString name = QString("Surface"));

	//! Draws bounding box (OpenGL)
	/** \param context OpenGL context
	 *  \param col (R,G,B) color
	**/
	void draw(CC_DRAW_CONTEXT& context/*, ccBBox *pBox*/);

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SURFACE; }

    //inherited from ccGenericPrimitive
    virtual QString getTypeName() const override { return "Surface"; }

	void setWidth(float width) { width_ = width; }//设置线宽

    CCVector3 getMinCorner() { return m_Mincorner; }
    CCVector3 getMaxCorner() { return m_Maxcorner; }

    //inherited from ccPlanarEntityInterface
    CCVector3 getNormal() const override { return m_transformation.getColumnAsVec3D(2); }

    virtual ccGenericPrimitive* clone() const override;
    // 更新平面位置
	void updatePos(double value) { m_SurfacePos = value; m_DoubleClicked = false;}
    // 更新平面厚度
	void updateHeight(double value) { m_SurfaceHeight = value; }
    // 更新平面垂直方向
	void updateSliceDirection(EPivotType type) { m_SliceType = type; }
    // 更新平面位置
	void updateSlicePos(CCVector3d pos); 
    // 设置平面是否可见
	void setSurfaceVisible(bool visible) { m_visible = visible; }
    // 设置切割平面包围盒
    void setCalcBBox(ccBBox& box) { m_CalcBBox = box; }
    // 获取当前平面位置
    CCVector3d& currentPos() { return m_pos; }
    // 设置平面开启Z-Fighting
    void setOpenZFighting(bool open) { m_open = open; }
	// 设置平面缩放,基础尺寸为1,扩大10%即为1.1
    void setScaling(float scaling);

	CCVector3d pos2D;
	CCVector3d position2d_text;

protected:
    bool buildUp() override;
private:
	float width_;

	//! Clipping box
	ccBBox m_bbox;
    ccBBox m_CalcBBox;

	CCVector3 m_Mincorner;
	CCVector3 m_Maxcorner;

    double m_SurfaceHeight;     //切面厚度
    double m_SurfacePos;        //滑动条参数控制切面位置
	EPivotType m_SliceType;     //切面类型
	CCVector3d m_pos;           //切面更新后位置
	float m_XOffset;            //X轴切面偏移量
	float m_YOffset;            //Y轴切面偏移量
	float m_ZOffset;            //Z轴切面偏移量


	bool m_DoubleClicked;       //是否开启选点控制切面位置
	double m_halfBoxSize;       //切面厚度/2
    bool m_open;                //是否处理z-fighting
    float m_scaling;            //平面缩放比例
    bool m_scalingOpen;         //是否开启平面缩放
};

#endif //CC_SURFACE_HEADER
