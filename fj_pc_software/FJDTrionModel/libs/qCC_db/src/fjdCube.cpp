
#include "fjdCube.h"

//Local
#include <QOpenGLFunctions_2_1>

fjdCube::fjdCube(float length /*= 1.0f*/, 
    float aplha /*= 1.0f*/)
    :m_Length(length)
    ,m_Alpha(aplha)

{
}

void fjdCube::darwMe(CC_DRAW_CONTEXT& context, const CCVector3& center, PointCoordinateType radius )
{
    QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr) {
        return;
    }

    float halfLength = m_Length / 2.0f;
    CCVector3 vertices[8] = {
        { -halfLength, -halfLength, halfLength },  // 0
        { halfLength, -halfLength, halfLength },   // 1
        { halfLength, -halfLength, -halfLength },  // 2
        { -halfLength, -halfLength, -halfLength }, // 3
        { -halfLength, halfLength, halfLength },   // 4
        { halfLength, halfLength, halfLength },    // 5
        { halfLength, halfLength, -halfLength },   // 6
        { -halfLength, halfLength, -halfLength }   // 7
    };
    glFunc->glLineWidth(2.0f); // 设置线段宽度为2个像素
    glFunc->glBegin(GL_LINES);
    float r = 1.0f;
    float g = 1.0f;
    float b = 0;
    if (!m_rgb.empty())
    {
        r = m_rgb[0];
        g = m_rgb[1];
        b = m_rgb[2];
    }
    

    // 底面
    glColor3f(r, g, b); 
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[0]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[1]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[1]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[2]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[2]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[3]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[3]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[0]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[4]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[5]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[5]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[6]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[6]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[7]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[7]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[4]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[0]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[4]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[1]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[5]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[2]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[6]));

    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[3]));
    glVertex3fv(reinterpret_cast<const GLfloat*>(&vertices[7]));

    glEnd();

    glBegin(GL_QUADS);

    // 前面
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(-halfLength, -halfLength, halfLength);
    glVertex3f(halfLength, -halfLength, halfLength);
    glVertex3f(halfLength, halfLength, halfLength);
    glVertex3f(-halfLength, halfLength, halfLength);

    // 后面
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(-halfLength, -halfLength, -halfLength);
    glVertex3f(halfLength, -halfLength, -halfLength);
    glVertex3f(halfLength, halfLength, -halfLength);
    glVertex3f(-halfLength, halfLength, -halfLength);

    // 左侧
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(-halfLength, -halfLength, halfLength);
    glVertex3f(-halfLength, -halfLength, -halfLength);
    glVertex3f(-halfLength, halfLength, -halfLength);
    glVertex3f(-halfLength, halfLength, halfLength);

    // 右侧
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(halfLength, -halfLength, halfLength);
    glVertex3f(halfLength, -halfLength, -halfLength);
    glVertex3f(halfLength, halfLength, -halfLength);
    glVertex3f(halfLength, halfLength, halfLength);

    // 顶面
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(-halfLength, halfLength, halfLength);
    glVertex3f(halfLength, halfLength, halfLength);
    glVertex3f(halfLength, halfLength, -halfLength);
    glVertex3f(-halfLength, halfLength, -halfLength);

    // 底面
    glColor4f(r, g, b, m_Alpha);
    glVertex3f(-halfLength, -halfLength, halfLength);
    glVertex3f(halfLength, -halfLength, halfLength);
    glVertex3f(halfLength, -halfLength, -halfLength);
    glVertex3f(-halfLength, -halfLength, -halfLength);
    
    glEnd();

}




