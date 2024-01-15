
#include "ccIncludeGL.h"

//Local

#include "ccBasicTypes.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"

//Qt
#include <QSharedPointer>
#include <qdebug.h>
#include <QOpenGLFunctions_3_3_Core>
//System
#include <assert.h>
#include <string.h>
#include "ccColorTypes.h"
#include "ccImageMesh.h"
#include "FJStyleManager.h"


ccImageMesh::ccImageMesh(QString name/*=QString()*/)
    : ccHObject(name.isEmpty() ? "label" : name)
{
    lockVisibility(false);
    setEnabled(true);
    m_image = QImage("C:\\Users\\carl.wang\\Desktop\\20230824-152407.jpg");
}


QString ccImageMesh::getName() const
{
    return m_name;
}

void ccImageMesh::setRotateMat(Eigen::Matrix3d mat)
{
    m_mat = mat;
}

void ccImageMesh::onDeletionOf(const ccHObject* obj)
{
    ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccImageMesh::toFile_MeOnly(QFile& out) const
{
    if (!ccHObject::toFile_MeOnly(out))
        return false;
    return true;
}

bool ccImageMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
    if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
        return false;
    return true;
}


void ccImageMesh::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    //2D foreground only
    if (!MACRO_Foreground(context))
        return;

    //Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
    if (MACRO_VirtualTransEnabled(context))
        return;

    if (MACRO_Draw3D(context))
        drawMeOnly3D(context);
    else if (MACRO_Draw2D(context))
        drawMeOnly2D(context);


}

void ccImageMesh::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_3_3_Core* glFunc3 = context.glFunctions<QOpenGLFunctions_3_3_Core>();
    QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr || glFunc3 == nullptr) {
        return;
    }
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        if (MACRO_DrawFastNamesOnly(context))
            return;
        glFunc->glPushName(getUniqueIDForDisplay());
    }
    if (!m_isinit)
    {
        // 创建纹理对象
        glFunc3->glGenTextures(1, &m_textureRef);
        glFunc3->glBindTexture(GL_TEXTURE_2D, m_textureRef);
        // 设置纹理参数
        glFunc3->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glFunc3->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFunc3->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glFunc3->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        //m_image = QImage("C:\\Users\\carl.wang\\Desktop\\20230824-152407.jpg");
        m_image = m_image.convertToFormat(QImage::Format_RGBA8888);
        auto width = m_image.width();
        auto height = m_image.height();
        glFunc3->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image.width(), m_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.constBits());
        // 解绑纹理对象
        glFunc3->glBindTexture(GL_TEXTURE_2D, 0);


        //ccSphereModel mySphere = ccSphereModel(48);

        //ind = mySphere.getIndices();
        //std::vector<CCVector3> vert = mySphere.getVertices();
        //std::vector<CCVector2> tex = mySphere.getTexCoords();
        //m_numVertices = mySphere.getNumIndices();
        //for (int i = 0; i < m_numVertices; i++) {
        //    pvalues.push_back((vert[ind[i]]).x);
        //    pvalues.push_back((vert[ind[i]]).y);
        //    pvalues.push_back((vert[ind[i]]).z);
        //    tvalues.push_back((tex[ind[i]]).x);
        //    tvalues.push_back((tex[ind[i]]).y);
        //}

        double radius = 25.0;
        unsigned int rings = 100;
        unsigned int sectors = 100;
        float const R = 1.0f / (float)(rings - 1);
        float const S = 1.0f / (float)(sectors - 1);

        for (unsigned int r = 0; r < rings; ++r)
        {
            for (unsigned int s = 0; s < sectors; ++s)
            {
                double const yy = sin(-3.1415926535 / 2.0 + 3.1415926535 * r * R);
                double const xx = cos(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);
                double const zz = sin(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);

                double const x = m_mat(0, 0) * xx + m_mat(0, 1) * yy + m_mat(0, 2) *zz;
                double const y = m_mat(1, 0) * xx + m_mat(1, 1) * yy + m_mat(1, 2) *zz;
                double const z = m_mat(2, 0) * xx + m_mat(2, 1) * yy + m_mat(2, 2) *zz;

                //float const y = sin(-3.1415926535 / 2.0 + 3.1415926535 * r * R);
                //float const x = cos(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);
                //float const z = sin(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);

                pvalues.push_back(x * radius + m_centerPos.x);
                pvalues.push_back(y * radius + m_centerPos.y);
                pvalues.push_back(z * radius + m_centerPos.z);
                tvalues.push_back(1 - s * S);
                tvalues.push_back(r * R);
            }
        }

        for (unsigned int r = 0; r < rings - 1; ++r)
        {
            for (unsigned int s = 0; s < sectors - 1; ++s)
            {
                ind.push_back(r * sectors + s);
                ind.push_back(r * sectors + (s + 1));
                ind.push_back((r + 1) * sectors + (s + 1));
                ind.push_back((r + 1) * sectors + (s + 1));
                ind.push_back((r + 1) * sectors + s);
                ind.push_back(r * sectors + s);
            }
        }
        std::reverse(ind.begin(), ind.end());
        m_numVertices = ind.size();


        glFunc3->glGenVertexArrays(1, &m_VAO);
        glFunc3->glGenBuffers(1, &m_VBO);
        glFunc3->glGenBuffers(1, &m_TBO);
        glFunc3->glGenBuffers(1, &m_EBO);
              
        glFunc3->glBindVertexArray(m_VAO);

        glFunc3->glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glFunc3->glBufferData(GL_ARRAY_BUFFER, pvalues.size() * sizeof(float), pvalues.data(), GL_STATIC_DRAW);
        glFunc3->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glFunc3->glEnableVertexAttribArray(0);

        glFunc3->glBindBuffer(GL_ARRAY_BUFFER, m_TBO);
        glFunc3->glBufferData(GL_ARRAY_BUFFER, tvalues.size() * sizeof(float), tvalues.data(), GL_STATIC_DRAW);
        glFunc3->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glFunc3->glEnableVertexAttribArray(1);

        glFunc3->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
        glFunc3->glBufferData(GL_ELEMENT_ARRAY_BUFFER, ind.size() * sizeof(int), ind.data(), GL_STATIC_DRAW);

        glFunc3->glBindVertexArray(0);
        m_isinit = true;
    }    
    if (m_needinitvbo)
    {
        pvalues.clear();
        double radius = 25.0;
        unsigned int rings = 100;
        unsigned int sectors = 100;
        float const R = 1.0f / (float)(rings - 1);
        float const S = 1.0f / (float)(sectors - 1);

        for (unsigned int r = 0; r < rings; ++r)
        {
            for (unsigned int s = 0; s < sectors; ++s)
            {
                double const yy = sin(-3.1415926535 / 2.0 + 3.1415926535 * r * R);
                double const xx = cos(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);
                double const zz = sin(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);

                double const x = m_mat(0, 0) * xx + m_mat(0, 1) * yy + m_mat(0, 2) *zz;
                double const y = m_mat(1, 0) * xx + m_mat(1, 1) * yy + m_mat(1, 2) *zz;
                double const z = m_mat(2, 0) * xx + m_mat(2, 1) * yy + m_mat(2, 2) *zz;

                //float const y = sin(-3.1415926535 / 2.0 + 3.1415926535 * r * R);
                //float const x = cos(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);
                //float const z = sin(2 * 3.1415926535 * s * S) * sin(3.1415926535 * r * R);

                pvalues.push_back(x * radius + m_centerPos.x);
                pvalues.push_back(y * radius + m_centerPos.y);
                pvalues.push_back(z * radius + m_centerPos.z);
            }
        }
        glFunc3->glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glFunc3->glBufferSubData(GL_ARRAY_BUFFER, 0, pvalues.size() * sizeof(float), pvalues.data());
        m_needinitvbo = false;
    }
    if (m_needinittexture)
    {
        glFunc3->glBindTexture(GL_TEXTURE_2D, m_textureRef);
        m_image = m_image.convertToFormat(QImage::Format_RGBA8888);
        glFunc3->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image.width(), m_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.constBits());
        m_needinittexture = false;
    }
    glFunc->glPushAttrib(GL_LIGHTING_BIT | GL_TRANSFORM_BIT | GL_ENABLE_BIT);
    //glFunc->glEnable(GL_DEPTH_TEST);

    glFunc3->glEnable(GL_CULL_FACE);     
    glFunc3->glCullFace(GL_FRONT);
    glFunc3->glFrontFace(GL_CCW);  
    glFunc3->glUseProgram(context.imageMeshShaderProgram);

    int viewLoc = glFunc3->glGetUniformLocation(context.imageMeshShaderProgram, "view");
    int projectionLoc = glFunc3->glGetUniformLocation(context.imageMeshShaderProgram, "projection");
    int textureLoc = glFunc3->glGetUniformLocation(context.imageMeshShaderProgram, "textureSampler");
    int transparencyLoc = glFunc3->glGetUniformLocation(context.imageMeshShaderProgram, "transparency");
    GLdouble * mvparams = new GLdouble[16];
    GLdouble * projectionparams = new GLdouble[16];
    glFunc3->glGetDoublev(GL_PROJECTION_MATRIX, projectionparams);
    glFunc3->glGetDoublev(GL_MODELVIEW_MATRIX, mvparams);
    glFunc3->glUniform1f(transparencyLoc, 1.0);
    GLfloat * mvparams2 = new GLfloat[16];
    GLfloat * projectionparams2 = new GLfloat[16];
    std::vector<GLfloat> data1;
    std::vector<GLfloat> data2;
    for (int i = 0; i < 16; ++i)
    {
        mvparams2[i] = static_cast<float>(mvparams[i]);
        data1.push_back(mvparams2[i]);
        projectionparams2[i] = static_cast<float>(projectionparams[i]);
        data2.push_back(projectionparams2[i]);
    }

    glFunc3->glUniformMatrix4fv(viewLoc, 1, GL_FALSE, mvparams2);
    glFunc3->glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, projectionparams2);
    glFunc3->glActiveTexture(GL_TEXTURE0);
    glFunc3->glBindTexture(GL_TEXTURE_2D, m_textureRef);
    glFunc3->glUniform1f(textureLoc, 0);
    glFunc3->glBindVertexArray(m_VAO);
    glFunc3->glDrawElements(GL_TRIANGLES, ind.size(), GL_UNSIGNED_INT, 0);
    glFunc3->glBindVertexArray(0);
    glFunc3->glUseProgram(0);
    //glFunc3->glBindTexture(GL_TEXTURE_2D, 0);
    glFunc->glPopAttrib(); //GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT | GL_POINT_BIT --> will switch the light off
    if (pushName)
    {
        glFunc->glPopName();
    }
}

ccBBox ccImageMesh::getOwnBB(bool withGLFeatures/*=false*/)
{
    ccBBox box;

    box.minCorner() = (m_centerPos - CCVector3d(25, 25, 25)).toFloat();
    box.minCorner() = (m_centerPos + CCVector3d(25, 25, 25)).toFloat();
    box.setValidity(true);
    return box;
}

void ccImageMesh::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{

}

void ccImageMesh::setImage(QImage image)
{ 
    m_image = image; 
    m_needinittexture = true;
}

void ccImageMesh::setCenterPos(CCVector3d centerPos) 
{ 
    m_centerPos = centerPos; 
    m_needinitvbo = true;
}

void ccImageMesh::deleteVertexArrays()
{
    if (m_isinit)
    {
        FJStyleManager::Instance()->deleteVertexArrays(m_VAO);
        FJStyleManager::Instance()->deleteBuffers(m_VBO);
        FJStyleManager::Instance()->deleteBuffers(m_TBO);
        FJStyleManager::Instance()->deleteBuffers(m_EBO);
        m_isinit = false;
    }
}

ccImageMesh::~ccImageMesh()
{
    deleteVertexArrays();
}