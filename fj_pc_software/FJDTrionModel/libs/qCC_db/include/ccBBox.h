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

#ifndef CC_BBOX_HEADER
#define CC_BBOX_HEADER

//Local
#include "ccDrawableObject.h"

//CCCoreLib
#include <BoundingBox.h>

//! Bounding box structure
/** Supports several operators such as addition (to a matrix or a vector) and
	multiplication (by a matrix or a scalar).
**/
class QCC_DB_LIB_API ccBBox : public CCCoreLib::BoundingBox
{
public:
	//增加线宽，提升效果
	//! Default constructor
	ccBBox() : CCCoreLib::BoundingBox() { setWidth(2.0); }
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCVector3& bbMinCorner, const CCVector3& bbMaxCorner) : CCCoreLib::BoundingBox(bbMinCorner, bbMaxCorner) { setWidth(2.0); }
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCCoreLib::BoundingBox& bbox) : CCCoreLib::BoundingBox(bbox) { setWidth(2.0); }

	//! Applies transformation to the bounding box
	const ccBBox operator * (const ccGLMatrix& mat);
	//! Applies transformation to the bounding box
	const ccBBox operator * (const ccGLMatrixd& mat);

	void setGLMatrixd(const ccGLMatrix& mat) {m_drawmat = mat; m_isNeedDrawMat = true;}

	//! Draws bounding box (OpenGL)
	/** \param context OpenGL context
	 *  \param col (R,G,B) color
	**/
	void draw(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) const;

	void setWidth(float width) { width_ = width; }//设置线宽

	CCVector3d pos2D;
	CCVector3d position2d_text;
	ccGLMatrix m_drawmat;//视图旋转
	bool m_isNeedDrawMat = false; //是否需要旋转
	
private:
	float width_;
};

#endif //CC_BBOX_HEADER
