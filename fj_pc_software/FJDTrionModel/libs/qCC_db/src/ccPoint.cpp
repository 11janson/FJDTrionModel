#include "ccPoint.h"
#include <QDebug>
#include "ccIncludeGL.h"
#include "ccBasicTypes.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"
#include "cloudcompareutils/icore.h"
#include "cc2DItemBase.h"

ccPoint::ccPoint(QString name)
    : ccHObject(name.isEmpty() ? "Point" : name)
{
   
    m_screenPos[0] = m_screenPos[1] = 0.05f;
    m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
    setVisible(true);
    setName("Point");
    m_strName = "Point";
    lockVisibility(true);
    setEnabled(true);
	setLocked(true);
    //[!].端点图片
    m_activaImage = QImage(CS::Core::ICore::resourceThemeImagePath("graphics_circle.png"));
    m_normalImage = QImage(CS::Core::ICore::resourceThemeImagePath("graphics_circle.png"));
}

void ccPoint::setPosition(CCVector3d positon)
{
    m_Position = positon;
}

void ccPoint::setScreenPos(QPointF pos)
{
    m_screenPos[0] = pos.x();
    m_screenPos[1] = pos.y();
}

CCVector3d ccPoint::getPositon(void)
{
    return m_Position;
}

CCVector3d ccPoint::get2DPostion(void)
{
    return m_Position2D;
}

QString ccPoint::getName() const
{
    return m_strName;
}



bool ccPoint::acceptClick(int x, int y, Qt::MouseButton button)
{
    //if (button == Qt::RightButton)
    {
        //if (m_.contains(x - m_lastScreenPos[0], y - m_lastScreenPos[1]))
        //{
        //    //toggle collapse state
        //    return true;
        //}
    }

    return false;
}

bool ccPoint::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{

    /* m_Position.x += static_cast<float>(dx) / screenWidth;
     m_Position.y += static_cast<float>(dy) / screenHeight;*/
    return false;
}

bool ccPoint::move3D(const CCVector3d & u)
{
    m_Position.x += u.x;
    m_Position.y += u.y;
    m_Position.z += u.z;
    return true;
}

void ccPoint::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (!m_visible)
    {
        return;
    }
    if (!MACRO_Foreground(context)) {
        return;
    }

    if (MACRO_Draw3D(context)) {
        drawMeOnly3D(context);
    }
    else if (MACRO_Draw2D(context)) {
        drawMeOnly2D(context);
    }

    return;
}

void ccPoint::drawMeOnly2D(CC_DRAW_CONTEXT & context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr) {
        assert(false);
        return;
    }

  
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        glFunc->glPushName(getUniqueIDForDisplay());
    }

    //[!].配置数据
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;


    //[!].高亮样式配置
    bool highlighted = isSelected();
    unsigned char alpha = static_cast<unsigned char>((context.labelOpacity / 100.0) * 255);
    ccColor::Rgbaub defaultBkgColor(context.labelDefaultBkgCol, alpha);

    ccColor::Rgbaub defaultBorderColor(ccColor::red, 255);
    // if (!highlighted) 
    {

        unsigned char halfAlpha = static_cast<unsigned char>((50.0 + context.labelOpacity / 200.0) * 255);
        defaultBorderColor = ccColor::Rgbaub(context.labelDefaultBkgCol, halfAlpha);
    }

    ccColor::Rgba defaultTextColor;
    if (context.labelOpacity < 40)
    {
        //under a given opacity level, we use the default text color instead!
        defaultTextColor = context.textDefaultCol;
    }
    else
    {
        defaultTextColor = ccColor::Rgba(255 - context.labelDefaultBkgCol.r,
            255 - context.labelDefaultBkgCol.g,
            255 - context.labelDefaultBkgCol.b,
            context.labelDefaultBkgCol.a);
    }

    m_screenPos[0] = m_Position2D.x;
    m_screenPos[1] = m_Position2D.y;

    const int xStart = static_cast<int>(context.glW * m_screenPos[0]);
    const int yStart = static_cast<int>(context.glH * (1.0f - m_screenPos[1]));
    m_lastScreenPos[0] = xStart;
    m_lastScreenPos[1] = yStart;


    glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
    glFunc->glEnable(GL_BLEND);
    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    m_TranslatefPoint.setX(static_cast<GLfloat>(-halfW));
    m_TranslatefPoint.setY(static_cast<GLfloat>(-halfH));
    glFunc->glTranslatef(m_TranslatefPoint.x(), m_TranslatefPoint.y(), 0);

   

    CCVector3d arrowDest2D(0, 0, 0);
    arrowDest2D = m_Position2D;
    int iArrowDestX = static_cast<int>(arrowDest2D.x);
    int iArrowDestY = static_cast<int>(arrowDest2D.y);

    //[!].画端点位置
    QImage image;
    image = m_normalImage;
    if (highlighted)
    {
        image = m_activaImage;
    }

    int isize = 12;
    {
        glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
        glFunc->glEnable(GL_BLEND);
        glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glFunc->glPushAttrib(GL_ENABLE_BIT);
        glFunc->glEnable(GL_TEXTURE_2D);

        QOpenGLTexture texture(image);
        texture.bind();
        {

            glFunc->glColor4f(1, 1, 1, 1.0);
            //isize = 64;
            glFunc->glBegin(GL_QUADS);
            glFunc->glTexCoord2f(1, 1); glFunc->glVertex2f(iArrowDestX + int(isize / 2), iArrowDestY - int(isize / 2));
            glFunc->glTexCoord2f(1, 0); glFunc->glVertex2f(iArrowDestX + int(isize / 2), iArrowDestY + int(isize / 2));
            glFunc->glTexCoord2f(0, 0); glFunc->glVertex2f(iArrowDestX - int(isize / 2), iArrowDestY + int(isize / 2)); 
            glFunc->glTexCoord2f(0, 1); glFunc->glVertex2f(iArrowDestX - int(isize / 2), iArrowDestY - int(isize / 2));
            glFunc->glEnd();
        }
        texture.release();
        glFunc->glPopAttrib();
        glFunc->glPopAttrib();
    }

    glFunc->glPopAttrib();
    glFunc->glPopMatrix();
    if (pushName)
    {
        glFunc->glPopName();
    }
}

static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);
void ccPoint::drawMeOnly3D(CC_DRAW_CONTEXT & context)
{
    //get the set of OpenGL functions (version 2.1)
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
    {
        assert(false);
        return;
    }

    //standard case: list names pushing
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        //not particularly fast
        if (MACRO_DrawFastNamesOnly(context))
            return;
        glFunc->glPushName(getUniqueIDForDisplay());
    }

    //we always project the points in 2D (maybe useful later, even when displaying the label during the 2D pass!)
    ccGLCameraParameters camera;
    //we can't use the context 'ccGLCameraParameters' (viewport, modelView matrix, etc. )
    //because it doesn't take the temporary 'GL transformation' into account!
    //context.display->getGLCameraParameters(camera);
    glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
    glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
    glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

    //don't do this in picking mode!
    if (!pushName)
    {

        //[!].计算3D位置
        CCVector3d P2D(m_screenPos[0], m_screenPos[1], m_Position.z);
       // camera.unproject(P2D, m_Position);

        //project the point in 2D
        CCVector3d P3D = m_Position;
        camera.project(P3D, m_Position2D);
    }

    if (!c_unitPointMarker)
    {
        c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, nullptr, "PointMarker", 12));
        c_unitPointMarker->showColors(true);
        c_unitPointMarker->setVisible(true);
        c_unitPointMarker->setEnabled(true);
    }

    //build-up point maker own 'context'
    CC_DRAW_CONTEXT markerContext = context;
    markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
    markerContext.display = nullptr;



    c_unitPointMarker->setTempColor(context.labelDefaultMarkerCol);
    const ccViewportParameters& viewportParams = context.display->getViewportParameters();

    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    CCVector3d P = m_Position;
    ccGL::Translate(glFunc, P.x, P.y, P.z);
    float scale = context.labelMarkerSize * m_relMarkerScale;
    if (viewportParams.perspectiveView && viewportParams.zFar > 0)
    {
        //in perspective view, the actual scale depends on the distance to the camera!
        double d = (camera.modelViewMat * P).norm();
        double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
        scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
    }
    glFunc->glScalef(scale, scale, scale);
    //c_unitPointMarker->draw(markerContext);
    glFunc->glPopMatrix();


    if (pushName)
    {
        glFunc->glPopName();
    }

    return;
}


