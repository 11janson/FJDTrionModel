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
#pragma once
//Local
#include "ccGenericPrimitive.h"
#include "CCTypes.h"

//! Sphere (primitive)
/** 3D sphere primitive
**/
class QCC_DB_LIB_API fjdCube 
{
public:

    fjdCube(float length = 1.0f,float aplha = 1.0f);

    CC_CLASS_ENUM getClassID() { return CC_TYPES::CLIPPING_BOX_CUBE; }

    QString getTypeName()  { return "Cube"; }

    void darwMe(CC_DRAW_CONTEXT& context, const CCVector3& center, PointCoordinateType radius);

    void setCubeRgb(float r, float g, float b) {
        m_rgb.push_back(r); 
        m_rgb.push_back(g);
        m_rgb.push_back(b);
    }
    void cubeRgbReset() {
        m_rgb.clear();
    }
private:
    PointCoordinateType m_Length	= 1.0f;
    PointCoordinateType m_Alpha		= 1.0f;
    std::vector<float>m_rgb;
};


