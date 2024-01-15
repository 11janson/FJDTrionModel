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

#ifndef __CCIMAGEICON_H__
#define __CCIMAGEICON_H__
#include "ccHObject.h"

//Local



//! Torus (primitive)
/** 3D torus primitive (with circular or rectangular section)
**/
class QCC_DB_LIB_API ccImageIcon : public ccHObject
{
public:
    //! Default constructor
    ccImageIcon(QString name = QString("Icon"));
    ~ccImageIcon();
    //inherited from ccObject
    virtual QString getName() const override;
    //inherited from ccHObject
    inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::OBJECT; }
    inline virtual bool isSerializable() const override { return true; }

    //! Returns 'raw' name (no replacement of default keywords)
    inline QString getRawName() const { return m_name; }

    void setImage(QImage image);

    void setCenterPos(CCVector3d centerPos);
    void setDirection(CCVector3d direction);
    void deleteVertexArrays();
    void setTransparency(float num);
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

    bool m_isinit = false;
    GLuint m_textureRef;
    unsigned int m_VAO;
    unsigned int m_VBO;
    unsigned int m_TBO;
    unsigned int m_EBO;
    bool m_needinittexture = false;
    std::vector<int> ind;
    std::vector<float> pvalues;
    std::vector<float> tvalues;
    int m_numVertices;
    QImage m_image;
    CCVector3d m_centerPos = CCVector3d(0,0,0);
    CCVector3d m_direction = CCVector3d(0.9, 0.9, 0.1);
    float m_transparency = 1.0;
};

#endif //__ccImageIcon_H__
