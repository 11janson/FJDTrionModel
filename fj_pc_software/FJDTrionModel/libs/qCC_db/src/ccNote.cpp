#include "ccIncludeGL.h"

//Local
#include "ccNote.h"
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
#include <QToolTip>

//System
#include <assert.h>
#include <string.h>
#include "cloudcompareutils/publicutils.h"
#include "cloudcompareutils/icore.h"

static const int c_margin = 5;
static const int c_arrowBaseSize = 3;

ccNote::ccNote(QString name)
    : ccHObject(name.isEmpty() ? "Note" : name)
{
    m_screenPos[0] = m_screenPos[1] = 0.05f;
    m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
    m_labelROI = QRect(0, 0, 0, 0);

    setMetaData("notetypeadd", false);
    setMetaData("NoteNode", true);

    setVisible(false);
    setName("Note");
    m_strTitle = "Note";
    clear(false);
    lockVisibility(false);
    setEnabled(true);

    //[!].
    m_activaImage = QImage(CS::Core::ICore::resourceThemeImagePath("notemouse.png"));
    m_normalImage = QImage(CS::Core::ICore::resourceThemeImagePath("notenormal.png"));

}



void ccNote::setPosition(float x, float y)
{
    m_screenPos[0] = x;
    m_screenPos[1] = y;
}

bool ccNote::acceptClick(int x, int y, Qt::MouseButton button)
{
    //if (button == Qt::RightButton)
    {
        if (m_labelROI.contains(x - m_lastScreenPos[0], y - m_lastScreenPos[1]))
        {
            //toggle collapse state
            return true;
        }
    }

    return false;
}

bool ccNote::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
    assert(screenHeight > 0 && screenWidth > 0);

    m_screenPos[0] += static_cast<float>(dx) / screenWidth;
    m_screenPos[1] += static_cast<float>(dy) / screenHeight;

    m_offsetPoint.setX(dx + m_offsetPoint.x());
    m_offsetPoint.setY(dy + m_offsetPoint.y());
    return true;
}

void ccNote::drawMeOnly(CC_DRAW_CONTEXT& context)
{

    if (m_pickingMode == POINT_VOLUME && MACRO_Draw3D(context))
    {

        

    }

   
    if (!MACRO_Foreground(context)){
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

void ccNote::onDeletionOf(const ccHObject * obj)
{


    ccHObject::onDeletionOf(obj); //remove dependencies, etc.

    //check that associated clouds are not about to be deleted!
    size_t pointsToRemove = 0;
    if (m_stPickedPoint.entity() == obj) {
        ++pointsToRemove;
    }

    if (pointsToRemove == 0)
        return;



    clear(true);



}

bool ccNote::toFile_MeOnly(QFile & out) const
{
    if (!ccHObject::toFile_MeOnly(out))
        return false;

    //[!].写版本号 
    int nVersion = 103;

    uint32_t iVer = static_cast<uint32_t>(nVersion);
    if (out.write((const char*)&iVer, 4) < 0)
        return WriteError();

    //point index
    uint32_t index = static_cast<uint32_t>(m_stPickedPoint.index);
    if (out.write((const char*)&index, 4) < 0)
        return WriteError();

    //cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
    uint32_t cloudID = static_cast<uint32_t>(m_stPickedPoint._cloud ? m_stPickedPoint._cloud->getUniqueID() : 0);
    if (out.write((const char*)&cloudID, 4) < 0)
        return WriteError();

 
    //uv coordinates in the triangle (dataVersion >= 49)
    if (out.write((const char*)m_stPickedPoint.uv.u, sizeof(double) * 2) < 0)
        return WriteError();


    //entity center point (dataVersion >= 50)
    if (out.write((const char*)&(m_stPickedPoint.entityCenterPoint), sizeof(bool)) < 0)
        return WriteError();


    if (out.write((const char*)m_screenPos, sizeof(float) * 2) < 0)
        return WriteError();

    //[!].保存标题
    QByteArray data = Utils::PublicUtils::GBKtoGb(m_strTitle);
    int nLenght = data.size();
    if (out.write((const char*)&nLenght, 4) < 0)
        return WriteError();
    if (out.write((const char*)data.data(), nLenght) < 0)
        return WriteError();

    //[!].保存内容
    data = Utils::PublicUtils::GBKtoGb(m_strContent);
    nLenght = data.size();
    if (out.write((const char*)&nLenght, 4) < 0)
        return WriteError();
    if (out.write((const char*)data.data(), nLenght) < 0)
        return WriteError();


   
    //[!].是否有图片
    bool bBmp = m_listNoteImages.empty() ? false : true;
    if (out.write((const char*)&bBmp, sizeof(bool)) < 0)
        return WriteError();

    if (bBmp){
        //[!].有图片保存图片
        
        int nCount = m_listNoteImages.size();
        //[!].先保存图片个数
        if (out.write((const char*)&nCount, 4) < 0)
            return WriteError();

        //[!].保存图片
        for (auto image : m_listNoteImages){
            int nByteCount = image.byteCount();
            int iF = image.format();
            int iW = image.width();
            int iH = image.height();

            if (out.write((const char*)&iW, 4) < 0)
                return WriteError();

            if (out.write((const char*)&iH, 4) < 0)
                return WriteError();

            //[!].字节大小
            if (out.write((const char*)&nByteCount, 4) < 0)
                return WriteError();
          
            //[!].格式
            if (out.write((const char*)&iF, 4) < 0)
                return WriteError();

            //[!].数据
            if (out.write((const char*)image.bits(), nByteCount) < 0)
                return WriteError();
        }
       
    }


    int nData = 10;
    if (out.write((const char*)&nData, 4) < 0)
        return WriteError();

    //[!].写偏移
    nData = m_offsetPoint.x();
    if (out.write((const char*)&nData, 4) < 0)
        return WriteError();

    nData = m_offsetPoint.y();
    if (out.write((const char*)&nData, 4) < 0)
        return WriteError();


    return true;

}

bool ccNote::fromFile_MeOnly(QFile & in, short dataVersion, int flags, LoadedIDMap & oldToNewIDMap)
{

    if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
        return false;

    //[!].读取版本
    uint32_t iVer;
    if (in.read((char*)&iVer, 4) < 0)
        return ReadError();

    //point index
    uint32_t index = 0;
    if (in.read((char*)&index, 4) < 0)
        return ReadError();



    uint32_t cloudID = 0;
    if (in.read((char*)&cloudID, 4) < 0)
        return ReadError();

    if (cloudID != 0)
    {
        try
        {
            m_stPickedPoint.index = static_cast<unsigned>(index);
            //[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'PickedPoint::_cloud' pointer!!!
            *(uint32_t*)(&m_stPickedPoint._cloud) = cloudID;
        }
        catch (const std::bad_alloc)
        {
            return MemoryError();
        }
    }

    //uv coordinates in the triangle (dataVersion >= 49)
    CCVector2d uv;
    if (in.read((char*)uv.u, sizeof(double) * 2) < 0)
        return ReadError();

    //entity center point (dataVersion >= 50)
    bool entityCenterPoint = false;
    if (in.read((char*)&entityCenterPoint, sizeof(bool)) < 0)
        return ReadError();
    m_stPickedPoint.entityCenterPoint = entityCenterPoint;


    //Relative screen position (dataVersion >= 20)
    if (in.read((char*)m_screenPos, sizeof(float) * 2) < 0)
        return ReadError();


    //[!].保存标题
    int nLenght =0;
    if (in.read((char*)&nLenght, 4) < 0)
        return WriteError();

    QByteArray data;
    data.resize(nLenght);
    if (in.read((char*)data.data(), nLenght) < 0)
        return WriteError();
    m_strTitle = Utils::PublicUtils::GBKtoGb(data);

    nLenght = 0;
    if (in.read((char*)&nLenght, 4) < 0)
        return WriteError();
    data.resize(nLenght);
    if (in.read((char*)data.data(), nLenght) < 0)
        return WriteError();
    m_strContent = Utils::PublicUtils::GBKtoGb(data);


    //[!].是否有图片
    bool bBmp = false;
    if (in.read((char*)&bBmp, sizeof(bool)) < 0)
        return ReadError();

    m_listNoteImages.clear();
    if (bBmp) {
        //[!].有图片保存图片

        int nCount = 0;
        //[!].先读取图片总数
        if (in.read((char*)&nCount, 4) < 0)
            return WriteError();
        m_listNoteImages.resize(nCount);
        for (int i = 0; i < nCount; i++) {
            int iByteCount = 0;
            int iW = 0;
            int iH = 0;
            int iF = 0;
            
            if (in.read((char*)&iW, 4) < 0)
                return WriteError();

            if (in.read((char*)&iH, 4) < 0)
                return WriteError();

            if (in.read((char*)&iByteCount, 4) < 0)
                return WriteError();

            if (in.read((char*)&iF, 4) < 0)
                return WriteError();

          
            QByteArray bit;
            bit.resize(iByteCount);
            if (in.read((char*)bit.data(), iByteCount) < 0)
                return WriteError();

            QImage image((uchar *)bit.data(), iW, iH, QImage::Format(iF));
            m_listNoteImages[i] = image.copy(image.rect());
    
        }
    }
   
    int nData = 10;
    if (in.read((char*)&nData, 4) < 0)
        return WriteError();

    nData = 0;
    if (in.read((char*)&nData, 4) < 0)
        return WriteError();

    m_offsetPoint.setX(nData);

    nData = 0;
    if (in.read((char*)&nData, 4) < 0)
        return WriteError();

    m_offsetPoint.setY(nData);
    return true;
}

void ccNote::drawMeOnly2D(CC_DRAW_CONTEXT & context)
{
    
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr){
        assert(false);
        return;
    }

    //[!].内容
    QString strContent = QString("%1").arg( "  "+m_strTitle);
    //standard case: list names pushing
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        glFunc->glPushName(getUniqueIDForDisplay());
    }

    //[!].配置数据
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    int titleHeight = 0;
    int rowHeight = 0;
    int margin = static_cast<int>(c_margin        * context.renderZoom);
    int arrowBaseSize = static_cast<int>(c_arrowBaseSize * context.renderZoom);

    //[!].字体配置
    QFont bodyFont = context.display->getLabelDisplayFont(); //takes rendering zoom into account!
    QFont titleFont = bodyFont; //takes rendering zoom into account!
    QFontMetrics titleFontMetrics(titleFont);
    titleHeight = titleFontMetrics.height();
    QFontMetrics bodyFontMetrics(bodyFont);
    rowHeight = bodyFontMetrics.height();
    int dx = 100;
    int dy = 0;
    QFontMetrics fm(titleFont);
    QRect rect = fm.boundingRect(strContent);
    dx = rect.width();
 
    //[!].位置偏移计算
    dx = std::max(dx, rect.width());
    dy += margin;		//top vertical margin
    dy += titleHeight;	//title
    dx += margin * 2;
    m_labelROI = QRect(0, 0, dx, dy);
    const int xStart = static_cast<int>(context.glW * m_screenPos[0]);
    const int yStart = static_cast<int>(context.glH * (1.0f - m_screenPos[1]));
    m_lastScreenPos[0] = xStart;
    m_lastScreenPos[1] = yStart - m_labelROI.height();

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


    QImage image;
    image = m_normalImage;
    if (highlighted)
    {
        image = m_activaImage;
    }


    glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
    glFunc->glEnable(GL_BLEND);
    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(xStart - halfW), static_cast<GLfloat>(yStart - halfH), 0);

    CCVector3d arrowDest2D(0, 0, 0);
    arrowDest2D = m_stPickedPoint.pos2D;
    int iArrowDestX = static_cast<int>(arrowDest2D.x - xStart);
    int iArrowDestY = static_cast<int>(arrowDest2D.y - yStart);

    //[!].画便签
    float isize = 32;
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
            glFunc->glTexCoord2f(1, 1); glFunc->glVertex2f(iArrowDestX + isize, iArrowDestY);
            glFunc->glTexCoord2f(1, 0); glFunc->glVertex2f(iArrowDestX + isize, iArrowDestY + isize);
            glFunc->glTexCoord2f(0, 0); glFunc->glVertex2f(iArrowDestX, iArrowDestY + isize);
            glFunc->glTexCoord2f(0, 1); glFunc->glVertex2f(iArrowDestX, iArrowDestY);
            glFunc->glEnd();
        }
        texture.release();
        glFunc->glPopAttrib();
        glFunc->glPopAttrib();
    }

    int arrowBaseConfig = 0;
    //[!].画尾标
    QRect borderRect(iArrowDestX + 20 + m_offsetPoint.x(), iArrowDestY - dy - 5 - m_offsetPoint.y(), dx, dy);
    if (!pushName)
    {
        if (isSelected())
        {
          

            m_labelROI = borderRect;

            if (iArrowDestX < m_labelROI.left()) //left
                arrowBaseConfig += 0;
            else if (iArrowDestX > m_labelROI.right()) //Right
                arrowBaseConfig += 2;
            else  //Middle
                arrowBaseConfig += 1;

            if (iArrowDestY > -m_labelROI.bottom()) //Top
                arrowBaseConfig += 0;
            else if (iArrowDestY < -m_labelROI.top()) //Bottom
                arrowBaseConfig += 6;
            else  //Middle
                arrowBaseConfig += 3;

            if (arrowBaseConfig != 4) //4 = label above point!
            {
                glFunc->glColor4ubv(defaultBorderColor.rgba);
                glFunc->glBegin(GL_LINE_LOOP);
                glFunc->glLineWidth(2.0f);
                glFunc->glVertex2i(iArrowDestX, iArrowDestY);
                switch (arrowBaseConfig)
                {
                case 0: //top-left corner
                    glFunc->glVertex2i(m_labelROI.left(), m_labelROI.bottom());
                    break;
                case 1: //top-middle edge
                    glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), m_labelROI.bottom());
                    break;
                case 2: //top-right corner
                    glFunc->glVertex2i(m_labelROI.right(), m_labelROI.bottom());
                    break;
                case 3: //middle-left edge
                    glFunc->glVertex2i(m_labelROI.left(), std::max(-m_labelROI.top(), iArrowDestY - arrowBaseSize));
                    break;
                case 4: //middle of rectangle!
                    break;
                case 5: //middle-right edge
                    glFunc->glVertex2i(m_labelROI.right(), std::min(m_labelROI.bottom(), iArrowDestY + arrowBaseSize));
                    break;
                case 6: //bottom-left corner
                    glFunc->glVertex2i(m_labelROI.left(), m_labelROI.top());
                    break;
                case 7: //bottom-middle edge
                    glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), m_labelROI.top());
                    break;
                case 8: //bottom-right corner
                    glFunc->glVertex2i(m_labelROI.right(), m_labelROI.top());
                    break;
                }
                glFunc->glEnd();
            }
        }
       
    }

    if (isSelected())
    {
        //[!].画背景
        glFunc->glColor4ubv(defaultBkgColor.rgba);
        glFunc->glBegin(GL_QUADS);
        glFunc->glVertex2i(borderRect.left(), borderRect.top());
        glFunc->glVertex2i(borderRect.left(), borderRect.bottom());
        glFunc->glVertex2i(borderRect.right() + 10, borderRect.bottom());
        glFunc->glVertex2i(borderRect.right() + 10, borderRect.top());
        glFunc->glEnd();

        //[!].画边框
        //highlighted 高亮
        glFunc->glPushAttrib(GL_LINE_BIT);
        glFunc->glLineWidth(3.0f * context.renderZoom);
        glFunc->glColor4ubv(defaultBorderColor.rgba);
        glFunc->glBegin(GL_LINE_LOOP);
        glFunc->glVertex2i(borderRect.left(), borderRect.top());
        glFunc->glVertex2i(borderRect.left(), borderRect.bottom());
        glFunc->glVertex2i(borderRect.right() + 10, borderRect.bottom());
        glFunc->glVertex2i(borderRect.right() + 10, borderRect.top());
        glFunc->glEnd();
        glFunc->glPopAttrib(); //GL_LINE_BIT

        //[!].写文字
        ccColor::Rgba defaultTextColor;
        if (context.labelOpacity < 40) {
            //under a given opacity level, we use the default text color instead!
            defaultTextColor = context.textDefaultCol;
        }
        else {
            defaultTextColor = ccColor::Rgba(255 - context.labelDefaultBkgCol.r,
                255 - context.labelDefaultBkgCol.g,
                255 - context.labelDefaultBkgCol.b,
                context.labelDefaultBkgCol.a);
        }


        //[!].文字转换为图片
        QImage textImage(borderRect.width(), borderRect.height(), QImage::Format::Format_RGBA8888);
        QRect imageRect = textImage.rect();
        textImage.fill(Qt::transparent);
        {
            QPainter painter(&textImage);
            float glColor[4];
            glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
            QColor color;
            color.setRgbF(glColor[0], glColor[1], glColor[2], glColor[3]);
            painter.setPen(QColor(defaultTextColor.r, defaultTextColor.g, defaultTextColor.b));
            painter.setFont(titleFont);
            painter.drawText(imageRect, Qt::AlignLeft | Qt::AlignVCenter, strContent);
        }
        //纹理贴图
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
    }


   

   
    glFunc->glPopAttrib(); 
    glFunc->glPopMatrix();
    if (pushName)
    {
        glFunc->glPopName();
    }

}

static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);
void ccNote::drawMeOnly3D(CC_DRAW_CONTEXT & context)
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
        //project the point in 2D
        CCVector3 P3D = m_stPickedPoint.getPointPosition();
        camera.project(P3D, m_stPickedPoint.pos2D);
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
    CCVector3 P = m_stPickedPoint.getPointPosition();
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
    m_stPickedPoint.markerScale = scale;
    c_unitPointMarker->draw(markerContext);
    glFunc->glPopMatrix();
   
       
    if (pushName)
    {
        glFunc->glPopName();
    }

    return;
}

void ccNote::setPickingMode(Mode pickingMode)
{
    m_pickingMode = pickingMode;
}

void ccNote::setPickedPoint(const cc2DLabel::PickedPoint & pp)
{

    if (pp.entity()) {
        pp.entity()->addDependency(this, DP_NOTIFY_OTHER_ON_DELETE);
    } 

    m_stPickedPoint = pp;
    return ;
}

cc2DLabel::PickedPoint& ccNote::getPickedPoint(void)
{
    return  m_stPickedPoint;
}

void ccNote::clear(bool ignoreDependencies)
{
    if (ignoreDependencies)
    {
       
    }
    else
    {
        //remove all dependencies first!
        if (m_stPickedPoint._cloud)
            m_stPickedPoint.entity()->removeDependencyWith(this);
    }

    m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
    m_labelROI = QRect(0, 0, 0, 0);
    setVisible(false);
}

void ccNote::setTitle(const QString strTitle)
{
    m_strTitle = strTitle;
}

QString ccNote::getTitle(void)
{
    return m_strTitle;
}

void ccNote::setContent(const QString strContext)
{
    m_strContent = strContext;
}


QString ccNote::getContent(void)
{
    return m_strContent;
}

QString ccNote::getName() const
{
    return m_name;
}

/**
  *@brief 获取注释图片
  */
std::vector<QImage> ccNote::getNoteImages(void)
{
    return m_listNoteImages;
}

/**
*@brief 设置注释图片
*/
void ccNote::setNoteImages(std::vector<QImage> listImage)
{
    m_listNoteImages = listImage;
}

bool ccNote::appendImage(const QImage &image)
{
    if (m_listNoteImages.size() >= 19){
        return false;
    }
    m_listNoteImages.push_back(image);
    return true;
}

