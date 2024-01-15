#include "ccStaticLabel.h"
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
#include <QImage>

static const int c_margin = 5;
static const int c_arrowBaseSize = 3;
ccStaticLabel::ccStaticLabel(QString name)
    : ccHObject(name.isEmpty() ? "StaticLabel" : name)
{


    setVisible(true);
    setName("StaticLabel");
    m_strName = "Point";
    lockVisibility(true);
    setEnabled(true);

    m_titleContent = "StaticLabel";
}

void ccStaticLabel::setSelectColor(QColor color)
{
    m_selecColor = color;
}

void ccStaticLabel::setNormalColor(QColor color)
{
    m_normlColor = color;
}

void ccStaticLabel::setTitleContent(const QString &strTitleContent)
{
    m_titleContent = strTitleContent;
}

void ccStaticLabel::setPosition(CCVector3d positon)
{
    m_Position = positon;
}
CCVector3d ccStaticLabel::getPositon(void)
{
    return m_Position;
}
QString ccStaticLabel::getName() const
{
    return m_strName;
}
bool ccStaticLabel::acceptClick(int x, int y, Qt::MouseButton button)
{
  

    return true;
}
bool ccStaticLabel::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
    return false;
}
bool ccStaticLabel::move3D(const CCVector3d & u)
{
  
    return false;
}

void ccStaticLabel::drawMeOnly(CC_DRAW_CONTEXT& context)
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



void ccStaticLabel::drawMeOnly2D(CC_DRAW_CONTEXT & context)
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

    //[!].≈‰÷√ ˝æ›
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    //[!].◊÷ÃÂ≈‰÷√
    int titleHeight = 0;
    int rowHeight = 0;
    QFont bodyFont = context.display->getLabelDisplayFont(); //takes rendering zoom into account!
    QFont titleFont = bodyFont; //takes rendering zoom into account!
    QFontMetrics titleFontMetrics(titleFont);
    titleHeight = titleFontMetrics.height();
    QFontMetrics bodyFontMetrics(bodyFont);
    rowHeight = bodyFontMetrics.height();
    int dx = 100;
    int dy = 0;
    QFontMetrics fm(titleFont);
    QRect rect = fm.boundingRect(m_titleContent);
    dx = rect.width() + 5;
   
    //[!].Œª÷√∆´“∆º∆À„
    dx = std::max(dx, rect.width());
    dy += titleHeight;	//title



    //[!].∏ﬂ¡¡—˘ Ω≈‰÷√
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

    ccColor::Rgba bkgColor = ccColor::FromQColora(QColor(30, 30, 30, 128));
    //[!].ª≠±≥æ∞
    QRect borderRect(iArrowDestX - dx / 2, iArrowDestY - dy / 2, dx, dy);
    glFunc->glColor4ubv(bkgColor.rgba);
    glFunc->glBegin(GL_QUADS);
    glFunc->glVertex2i(borderRect.left(), borderRect.top());
    glFunc->glVertex2i(borderRect.left(), borderRect.bottom());
    glFunc->glVertex2i(borderRect.right() + 10, borderRect.bottom());
    glFunc->glVertex2i(borderRect.right() + 10, borderRect.top());
    glFunc->glEnd();

    //[!].ª≠±ﬂøÚ
    //highlighted ∏ﬂ¡¡
    ccColor::Rgba bordercolor;
    if (highlighted){
        bordercolor = ccColor::FromQColora(QColor(255, 200, 4));
    }
    else
    {
        bordercolor = ccColor::FromQColora(QColor(88, 88, 88));
    }
    glFunc->glPushAttrib(GL_LINE_BIT);
    glFunc->glLineWidth(2);
    glFunc->glColor4ubv(bordercolor.rgba);
    glFunc->glBegin(GL_LINE_LOOP);
    glFunc->glVertex2i(borderRect.left(), borderRect.top());
    glFunc->glVertex2i(borderRect.left(), borderRect.bottom());
    glFunc->glVertex2i(borderRect.right() + 10, borderRect.bottom());
    glFunc->glVertex2i(borderRect.right() + 10, borderRect.top());
    glFunc->glEnd();
    glFunc->glPopAttrib(); //GL_LINE_BIT

    //[!].–¥Œƒ◊÷
 
    //[!].Œƒ◊÷◊™ªªŒ™Õº∆¨
    QImage textImage(borderRect.width(), borderRect.height(), QImage::Format::Format_RGBA8888);
    QRect imageRect = textImage.rect();
    textImage.fill(Qt::transparent);
    {
        QPainter painter(&textImage);
        float glColor[4];
        glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
        painter.setPen(QColor(255, 255, 255));
        painter.setFont(titleFont);
        painter.drawText(imageRect, Qt::AlignHCenter | Qt::AlignVCenter, m_titleContent);
    }
    //Œ∆¿ÌÃ˘Õº
    {
        glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
        glFunc->glEnable(GL_BLEND);
        glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glFunc->glPushAttrib(GL_ENABLE_BIT);
        glFunc->glEnable(GL_TEXTURE_2D);

        QOpenGLTexture texture(textImage);
        texture.bind();
        {
            glFunc->glColor4f(1, 1, 1, 1.0);
            //isize = 64;
            glFunc->glBegin(GL_QUADS);
            glFunc->glTexCoord2f(1, 1);  glFunc->glVertex2i(borderRect.right() + 10, borderRect.top());
            glFunc->glTexCoord2f(1, 0);  glFunc->glVertex2i(borderRect.right() + 10, borderRect.bottom());
            glFunc->glTexCoord2f(0, 0); glFunc->glVertex2i(borderRect.left(), borderRect.bottom());
            glFunc->glTexCoord2f(0, 1); glFunc->glVertex2i(borderRect.left(), borderRect.top());
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
void ccStaticLabel::drawMeOnly3D(CC_DRAW_CONTEXT & context)
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
    if (!pushName){

        //[!].º∆À„3DŒª÷√
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


