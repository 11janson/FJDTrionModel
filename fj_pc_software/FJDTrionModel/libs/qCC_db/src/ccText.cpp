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

//Always first
#include "ccIncludeGL.h"

#include "ccText.h"

//Local
#include "ccCone.h"
#include "ccPointCloud.h"
#include "ccPoint.h"
#include "geometryUtils.h"
#include "cc2DItemBase.h"
#include "ccGLDrawContext.h"
#include "ccGenericGLDisplay.h"

ccText::ccText(GenericIndexedCloudPersist* associatedCloud, QString name)
	: Polyline(associatedCloud)
    , ccShiftedObject(name)
{
	set2DMode(false);
	setForeground(true);
    set2DMode(false);
	setVisible(true);
	lockVisibility(true);
	//setColor(ccColor::Rgbaf(1, 0, 0, 1));
	//showVertices(true);
	//setVertexMarkerWidth(3);
	//setWidth(2);
	showArrow(false, 0, 0);
	//setClosed(false);
	setLocked(true);
    setEnabled(true);
    m_isClosed = false;

    ccGenericPointCloud* cloud = dynamic_cast<ccGenericPointCloud*>(associatedCloud);
    if (cloud)
    {
        //no need to call ccPolyline::the copyGlobalShiftAndScalemethod
        //as it will try to set the Global Shift & Scale info on the associated cloud!
        ccShiftedObject::copyGlobalShiftAndScale(*cloud);
    }
}

ccText::ccText(const ccText& poly)
	: ccShiftedObject()
    , Polyline(nullptr)
{
	ccPointCloud* vertices = nullptr;
	initWith(vertices, poly);
}

void ccText::set2DMode(bool state)
{
    m_mode2D = state;
}

void ccText::setForeground(bool state)
{
    m_foreground = state;
}

ccText* ccText::clone() const
{
    ccText* clonedPoly = new ccText(*this);
	clonedPoly->setLocked(false); //there's no reason to keep the clone locked

    clonedPoly->setName(getName());
    clonedPoly->setMetaData(this->metaData());
    //clonedPoly->m_keyPoints3d = m_keyPoints3d;
    //clonedPoly->m_keyPoints2d = m_keyPoints2d;

    //clonedPoly->m_Type = m_Type;
    //clonedPoly->get3dPoints();
	return clonedPoly;
}

bool ccText::initWith(ccPointCloud*& vertices, const ccText& poly)
{
	bool success = true;
	if (!vertices)
	{
		ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(poly.m_theAssociatedCloud);
		ccPointCloud* clone = cloud ? cloud->partialClone(&poly) : ccPointCloud::From(&poly);
		if (clone)
		{
			if (cloud)
				clone->setName(cloud->getName()); //as 'partialClone' adds the '.extract' suffix by default
			else
				clone->setGLTransformationHistory(poly.getGLTransformationHistory());
		}
		else
		{
			//not enough memory?
			ccLog::Warning("[ccText::initWith] Not enough memory to duplicate vertices!");
			success = false;
		}

		vertices = clone;
	}

	if (vertices)
	{
		setAssociatedCloud(vertices);
		//addChild(vertices);
		//vertices->setEnabled(false);
		assert(m_theAssociatedCloud);
		if (m_theAssociatedCloud)
		{
			if (!addPointIndex(0, m_theAssociatedCloud->size()))
			{
				ccLog::Warning("[ccLines::initWith] Not enough memory");
				success = false;
			}
		}
	}

	importParametersFrom(poly);

	return success;
}

void ccText::importParametersFrom(const ccText& poly)
{
	//setClosed(poly.m_isClosed);
	set2DMode(poly.m_mode2D);
	setForeground(poly.m_foreground);
	setVisible(poly.isVisible());
	lockVisibility(poly.isVisibilityLocked());
	//setColor(poly.m_rgbColor);
	//setWidth(poly.m_width);
	showColors(poly.colorsShown());
	//showVertices(poly.verticesShown());
	//setVertexMarkerWidth(poly.getVertexMarkerWidth());
	copyGlobalShiftAndScale(poly);
	setGLTransformationHistory(poly.getGLTransformationHistory());
	setMetaData(poly.metaData());
}

void ccText::showArrow(bool state, unsigned vertIndex, PointCoordinateType length)
{

}

ccBBox ccText::getOwnBB(bool withGLFeatures/*=false*/)
{
    ccBBox box;
    //if (m_keyPoints3d.size() == 0)
    //    return  box;

    //for (int i = 0; i < m_keyPoints3d.size(); i++)
    //    box.add(CCVector3(m_keyPoints3d[i].x, m_keyPoints3d[i].y, m_keyPoints3d[i].z));
    box.setValidity(!isSelected());
	return box;
}

bool ccText::hasColors() const
{
	return true;
}

void ccText::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//invalidate the bounding-box
	//(and we hope the vertices will be updated as well!)
	invalidateBoundingBox();
}

//unit arrow
static QSharedPointer<ccCone> c_unitArrow(nullptr);

void ccText::drawMeOnly(CC_DRAW_CONTEXT& context)
{

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
        return;

    //if (MACRO_Draw3D(context))
    //    drawIn3dView(context);
    //else if (MACRO_Draw2D(context))
    //    drawIn2dView(context);


    bool draw = false;

    if (MACRO_Draw3D(context))
    {
        draw = !m_mode2D;
    }
    else if (m_mode2D)
    {
        bool drawFG = MACRO_Foreground(context);
        draw = ((drawFG && m_foreground) || (!drawFG && !m_foreground));
    }

    if (!draw)
        return;

    drawIn2dView(context);
}

void ccText::drawIn3dView(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    //if (context.display->windowName().compare("3D View") != 0)
    //    return;

    //bool pushName = MACRO_DrawEntityNames(context);
    //if (pushName)
    //    glFunc->glPushName(getUniqueIDForDisplay());



    //glFunc->glMatrixMode(GL_MODELVIEW);
    //glFunc->glColor4f(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a);


    //glFunc->glBegin(GL_LINE_STRIP);

    //glFunc->glEnd();


    //if (pushName)
    //    glFunc->glPopName();
}

void ccText::drawIn2dView(CC_DRAW_CONTEXT& context)
{
    //if (context.display->windowName().compare("2D View") != 0)
    //    return;

    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    bool pushName = MACRO_DrawEntityNames(context);


    ccGLCameraParameters camera;
    glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
    glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
    glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;
    glFunc->glMatrixMode(GL_MODELVIEW);
    glFunc->glPushMatrix();
    glFunc->glTranslatef(static_cast<GLfloat>(-halfW), static_cast<GLfloat>(-halfH), 0);


    const CCVector3* P0 = getAssociatedCloud()->getPoint(0);
    //CCVector3d P02D = viewPara.viewMat * (*P0);
    std::vector<CCVector3d> pts3D;
    pts3D.push_back(*P0);
    std::vector<CCVector3d> pts2D;
    if (!pushName)
    {
        pts2D = cc2DItemBase::projectTo2d(pts3D, camera);
    }

    if(pts2D.size() > 0)
        renderText(context, camera, static_cast<double>(pts2D[0].x), static_cast<double>(pts2D[0].y),
            static_cast<double>(pts2D[0].z), m_str, getUniqueID());

    if (pushName)
        glFunc->glPushName(getUniqueIDForDisplay());

    glFunc->glPopMatrix();


    if (pushName)
        glFunc->glPopName();
}


void ccText::renderText(CC_DRAW_CONTEXT& context, ccGLCameraParameters camera, double x, double y, double z,
    const QString & str, uint16_t uniqueID, const QFont & pFont, const ccColor::Rgba* backcolor)
{
    //if (m_activeFbo)
    //{
    //    m_activeFbo->start();
    //}

    //ccQOpenGLFunctions* glFunc = functions();
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();

    assert(glFunc);

    //retrieve the texture
    SharedTexture texture;
    if (uniqueID != 0)
    {
        if (m_uniqueTextures.contains(uniqueID))
        {
            //retrieve the texture
            texture = m_uniqueTextures[uniqueID];
        }
        else
        {
            //register it for later
            texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
            m_uniqueTextures.insert(uniqueID, texture);
        }
    }
    else
    {
        if (m_texturePoolLastIndex < m_texturePool.size())
        {
            //retrieve the texture
            texture = m_texturePool[m_texturePoolLastIndex++];
        }
        else
        {
            texture.reset(new QOpenGLTexture(QOpenGLTexture::Target2D));
            try
            {
                m_texturePool.push_back(texture);
                ++m_texturePoolLastIndex;
            }
            catch (const std::bad_alloc&)
            {
                //not enough memory to keep the texture?!
            }
        }
    }
    assert(texture);

    //compute the text bounding rect
    // This adjustment and the change to x & y are to work around a crash with Qt 5.9.
    // At the time I (Andy) could not determine if it is a bug in CC or Qt.
    //		https://bugreports.qt.io/browse/QTBUG-61863
    //		https://github.com/CloudCompare/CloudCompare/issues/543
    QRect textRect = QFontMetrics(pFont).boundingRect(str).adjusted(-1, -2, 1, 2);
    //ccLog::Print(QString("Texture rect = (%1 ; %2) --> (%3 x %4)").arg(textRect.x()).arg(textRect.y()).arg(textRect.width()).arg(textRect.height()));

    x -= 1;	// magic number!
    y += 3;	// magic number!

    QSize imageSize;
    if (texture->isStorageAllocated())
    {
        if (textRect.width() > texture->width() || textRect.height() > texture->height())
        {
            //we have to enlarge it
            texture->destroy();
            imageSize = textRect.size();
        }
        else
        {
            imageSize = QSize(texture->width(), texture->height());
        }
    }
    else
    {
        imageSize = textRect.size();
    }

    // We create a QImage from the text
    QImage textImage(imageSize.width(), imageSize.height(), QImage::Format::Format_RGBA8888);
    QRect imageRect = textImage.rect();
    //ccLog::Print(QString("Image rect = (%1 ; %2) --> (%3 x %4)").arg(imageRect.x()).arg(imageRect.y()).arg(imageRect.width()).arg(imageRect.height()));

    textImage.fill(Qt::transparent);
    {
        if (backcolor)
        {
            textImage.fill(QColor(backcolor->r, backcolor->g, backcolor->b));
        }
        QPainter painter(&textImage);

        float glColor[4];
        glFunc->glGetFloatv(GL_CURRENT_COLOR, glColor);
        QColor color;
        color.setRgbF(glColor[0], glColor[1], glColor[2], glColor[3]);

        painter.setPen(color);
        painter.setFont(pFont);
        painter.drawText(imageRect, Qt::AlignLeft, str);
    }

    //and then we convert this QImage to a texture!
    {

        glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
        glFunc->glEnable(GL_BLEND);
        glFunc->glDisable(GL_DEPTH_TEST);

        //set ortho view with center in the upper-left corner
        glFunc->glMatrixMode(GL_PROJECTION);
        glFunc->glPushMatrix();
        glFunc->glLoadIdentity();
        glFunc->glOrtho(0, context.glW, 0, context.glH, -1, 1);
        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPushMatrix();
        glFunc->glLoadIdentity();
        {

            //move to the right position on the screen
            glFunc->glTranslatef(x, y/*context.glH - 1 - y*/, 0);
            ////glFunc->glScalef(m_dScale, 0, 0);

            //const ccViewportParameters& viewportParams = context.display->getViewportParameters();
            //CCVector3 P(x, y, z);

            //float scale = context.labelMarkerSize /** m_relMarkerScale*/;
            //if (viewportParams.perspectiveView && viewportParams.zFar > 0)
            //{
            //    //in perspective view, the actual scale depends on the distance to the camera!
            //    double d = (camera.modelViewMat * P).norm();
            //    double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
            //    scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
            //}
            //glFunc->glScalef(scale, scale, scale);



            glFunc->glEnable(GL_TEXTURE_2D);

            if (texture->height() < textRect.height())
            {
                //we have to re-create it!
                texture->destroy();
            }

            //In order to reduce the time ATI cards take to manage the texture ID generation
            //and switching, we re-use the textures as much as possible.
            //texture->setData(textImage, QOpenGLTexture::DontGenerateMipMaps);
            if (!texture->isStorageAllocated())
            {
                //ccLog::Print(QString("New texture allocated: %1 x %2").arg(imageRect.width()).arg(imageRect.height()));
                texture->setMinificationFilter(QOpenGLTexture::Linear);
                texture->setMagnificationFilter(QOpenGLTexture::Linear);
                texture->setFormat(QOpenGLTexture::RGBA8_UNorm);
                texture->setSize(imageRect.width(), imageRect.height());
                texture->setMipLevels(0);
                texture->allocateStorage();
            }
            texture->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt32_RGBA8_Rev, textImage.bits());
            texture->bind();

            glFunc->glColor4f(1.0f, 1.0f, 1.0f, 1.0f); //DGM: warning must be float colors to work properly?!
            glFunc->glBegin(GL_QUADS);
            float ratioW = textRect.width() / static_cast<float>(imageRect.width());
            float ratioH = textRect.height() / static_cast<float>(imageRect.height());
            glFunc->glTexCoord2f(0, ratioH); glFunc->glVertex3i(0, 0, 0);
            glFunc->glTexCoord2f(ratioW, ratioH); glFunc->glVertex3i(textRect.width(), 0, 0);
            glFunc->glTexCoord2f(ratioW, 0); glFunc->glVertex3i(textRect.width(), textRect.height(), 0);
            glFunc->glTexCoord2f(0, 0); glFunc->glVertex3i(0, textRect.height(), 0);
            glFunc->glEnd();

            texture->release();
        }

        glFunc->glMatrixMode(GL_PROJECTION);
        glFunc->glPopMatrix();
        glFunc->glMatrixMode(GL_MODELVIEW);
        glFunc->glPopMatrix();

        glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT
    }
}



bool ccText::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[ccLines::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
		return false;
	}
	uint32_t vertUniqueID = (m_theAssociatedCloud ? (uint32_t)vertices->getUniqueID() : 0);
	if (out.write((const char*)&vertUniqueID, 4) < 0)
		return WriteError();

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = size();
	if (out.write((const char*)&pointCount, 4) < 0)
		return WriteError();

	//points (references to) (dataVersion>=28)
	for (uint32_t i = 0; i < pointCount; ++i)
	{
		uint32_t pointIndex = getPointGlobalIndex(i);
		if (out.write((const char*)&pointIndex, 4) < 0)
			return WriteError();
	}

	//'global shift & scale' (dataVersion>=39)
	saveShiftInfoToFile(out);

	QDataStream outStream(&out);

	//Closing state (dataVersion>=28)
	outStream << m_isClosed;

	//RGB Color (dataVersion>=28)
	//outStream << m_rgbColor.r;
	//outStream << m_rgbColor.g;
	//outStream << m_rgbColor.b;

	//2D mode (dataVersion>=28)
	//outStream << m_mode2D;

	//Foreground mode (dataVersion>=28)
	//outStream << m_foreground;

	//The width of the line (dataVersion>=31)
	//outStream << m_width;

	return true;
}

bool ccText::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	if (dataVersion < 28)
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by multiple polylines)
	//we only store its unique ID (dataVersion>=28) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_theAssociatedCloud) = vertUniqueID;

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = 0;
	if (in.read((char*)&pointCount, 4) < 0)
		return ReadError();
	if (!reserve(pointCount))
		return false;

	//points (references to) (dataVersion>=28)
	for (uint32_t i = 0; i < pointCount; ++i)
	{
		uint32_t pointIndex = 0;
		if (in.read((char*)&pointIndex, 4) < 0)
			return ReadError();
		addPointIndex(pointIndex);
	}

	//'global shift & scale' (dataVersion>=39)
	if (dataVersion >= 39)
	{
		if (!loadShiftInfoFromFile(in))
			return ReadError();
	}
	else
	{
		m_globalScale = 1.0;
		m_globalShift = CCVector3d(0,0,0);
	}

	QDataStream inStream(&in);

	//Closing state (dataVersion>=28)
	inStream >> m_isClosed;

	//RGB Color (dataVersion>=28)
	//inStream >> m_rgbColor.r;
	//inStream >> m_rgbColor.g;
	//inStream >> m_rgbColor.b;

	//2D mode (dataVersion>=28)
	//inStream >> m_mode2D;

	//Foreground mode (dataVersion>=28)
	//inStream >> m_foreground;

	//Width of the line (dataVersion>=31)
	//if (dataVersion >= 31)
	//	ccSerializationHelper::CoordsFromDataStream(inStream,flags,(PointCoordinateType*)&m_width,1);
	//else
	//	m_width = 0;

	return true;
}

bool ccText::split(	PointCoordinateType maxEdgeLength,
						std::vector<ccText*>& parts)
{
	parts.clear();

	//not enough vertices?
	unsigned vertCount = size();
	if (vertCount <= 2)
	{
		parts.push_back(new ccText(*this));
		return true;
	}

	unsigned startIndex = 0;
	unsigned lastIndex = vertCount-1;
	while (startIndex <= lastIndex)
	{
		unsigned stopIndex = startIndex;
		while (stopIndex < lastIndex && (*getPoint(stopIndex+1) - *getPoint(stopIndex)).norm() <= maxEdgeLength)
		{
			++stopIndex;
		}

		//number of vertices for the current part
		unsigned partSize = stopIndex-startIndex+1;

		//if the polyline is closed we have to look backward for the first segment!
		if (startIndex == 0)
		{
			if (isClosed())
			{
				unsigned realStartIndex = vertCount;
				while (realStartIndex > stopIndex && (*getPoint(realStartIndex-1) - *getPoint(realStartIndex % vertCount)).norm() <= maxEdgeLength)
				{
					--realStartIndex;
				}

				if (realStartIndex == stopIndex)
				{
					//whole loop
					parts.push_back(new ccText(*this));
					return true;
				}
				else if (realStartIndex < vertCount)
				{
					partSize += (vertCount - realStartIndex);
					assert(realStartIndex != 0);
					lastIndex = realStartIndex-1;
					//warning: we shift the indexes!
					startIndex = realStartIndex; 
					stopIndex += vertCount;
				}
			}
			else if (partSize == vertCount)
			{
				//whole polyline
				parts.push_back(new ccText(*this));
				return true;
			}
		}

		if (partSize > 1) //otherwise we skip that point
		{
			//create the corresponding part
			CCCoreLib::ReferenceCloud ref(m_theAssociatedCloud);
			if (!ref.reserve(partSize))
			{
				ccLog::Error("[ccText::split] Not enough memory!");
				return false;
			}

			for (unsigned i=startIndex; i<=stopIndex; ++i)
			{
				ref.addPointIndex(i % vertCount);
			}

			//ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
			//ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
			//ccLines* part = new ccLines(subset);
			//part->initWith(subset, *this);
			//part->setClosed(false); //by definition!
			//parts.push_back(part);
		}

		//forward
		startIndex = (stopIndex % vertCount) + 1;
	}

	return true;
}

PointCoordinateType ccText::computeLength() const
{
	PointCoordinateType length = 0;

	unsigned vertCount = size();
	if (vertCount > 1 && m_theAssociatedCloud)
	{
		unsigned lastVert = isClosed() ? vertCount : vertCount - 1;
		for (unsigned i = 0; i < lastVert; ++i)
		{
			CCVector3 A;
			getPoint(i, A);
			CCVector3 B;
			getPoint((i + 1) % vertCount, B);

			length += (B - A).norm();
		}
	}

	return length;
}

unsigned ccText::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
}

unsigned ccText::segmentCount() const
{
	unsigned count = size();
	if (count && !isClosed())
	{
		--count;
	}
	return count;
}

void ccText::setGlobalShift(const CCVector3d& shift)
{
	ccShiftedObject::setGlobalShift(shift);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global shift info to the vertices
		pc->setGlobalShift(shift);
	}
}

void ccText::setGlobalScale(double scale)
{
	ccShiftedObject::setGlobalScale(scale);

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//auto transfer the global scale info to the vertices
		pc->setGlobalScale(scale);
	}
}

const CCVector3d& ccText::getGlobalShift() const
{
	const ccPointCloud* pc = dynamic_cast<const ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//by default we use the vertices global shift info
		return pc->getGlobalShift();
	}
	else
	{
		return ccShiftedObject::getGlobalShift();
	}
}

double ccText::getGlobalScale() const
{
	const ccPointCloud* pc = dynamic_cast<const ccPointCloud*>(m_theAssociatedCloud);
	if (pc && pc->getParent() == this)
	{
		//by default we use the vertices global scale info
		return pc->getGlobalScale();
	}
	else
	{
		return ccShiftedObject::getGlobalScale();
	}
}

ccPointCloud* ccText::samplePoints(	bool densityBased,
										double samplingParameter,
										bool withRGB)
{
	if (samplingParameter <= 0 || size() < 2)
	{
		assert(false);
		return nullptr;
	}

	//we must compute the total length of the polyline
	double L = this->computeLength();

	unsigned pointCount = 0;
	if (densityBased)
	{
		pointCount = static_cast<unsigned>(ceil(L * samplingParameter));
	}
	else
	{
		pointCount = static_cast<unsigned>(samplingParameter);
	}

	if (pointCount == 0)
	{
		assert(false);
		return nullptr;
	}

	//convert to real point cloud
	ccPointCloud* cloud = new ccPointCloud(getName() + "." + QObject::tr("sampled"));
	if (!cloud->reserve(pointCount))
	{
		ccLog::Warning("[ccText::samplePoints] Not enough memory");
		delete cloud;
		return nullptr;
	}

	double samplingStep = L / pointCount;
	double s = 0.0; //current sampled point curvilinear position
	unsigned indexA = 0; //index of the segment start vertex
	double sA = 0.0; //curvilinear pos of the segment start vertex

	for (unsigned i = 0; i < pointCount; )
	{
		unsigned indexB = ((indexA + 1) % size());
		const CCVector3& A = *getPoint(indexA);
		const CCVector3& B = *getPoint(indexB);
		CCVector3 AB = B - A;
		double lAB = AB.normd();

		double relativePos = s - sA;
		if (relativePos >= lAB)
		{
			//specific case: last point
			if (i + 1 == pointCount)
			{
				assert(relativePos < lAB * 1.01); //it should only be a rounding issue in the worst case
				relativePos = lAB;
			}
			else //skip this segment
			{
				++indexA;
				sA += lAB;
				continue;
			}
		}

		//now for the interpolation work
		double alpha = relativePos / lAB;
		alpha = std::max(alpha, 0.0); //just in case
		alpha = std::min(alpha, 1.0);

		CCVector3 P = A + static_cast<PointCoordinateType>(alpha) * AB;
		cloud->addPoint(P);

		//proceed to the next point
		++i;
		s += samplingStep;
	}

	if (withRGB)
	{
		if (isColorOverridden())
		{
			//we use the default 'temporary' color
			cloud->setColor(getTempColor());
		}
		else if (colorsShown())
		{
			//we use the default color
			//cloud->setColor(ccColor::Rgba(m_rgbColor.r, m_rgbColor.g, m_rgbColor.b, m_rgbColor.a));
		}
	}

	//import parameters from the source
	cloud->copyGlobalShiftAndScale(*this);
	cloud->setGLTransformationHistory(getGLTransformationHistory());

	return cloud;
}

ccText* ccText::smoothChaikin(PointCoordinateType ratio, unsigned iterationCount) const
{
	if (iterationCount == 0)
	{
		assert(false);
		ccLog::Warning("[ccText::smoothChaikin] Invalid input (iteration count)");
		return nullptr;
	}

	if (ratio < 0.05f || ratio > 0.45f)
	{
		assert(false);
		ccLog::Warning("[ccText::smoothChaikin] invalid ratio");
		return nullptr;
	}

	if (size() < 3)
	{
		ccLog::Warning("[ccText::smoothChaikin] not enough segments");
		return nullptr;
	}

	const CCCoreLib::GenericIndexedCloudPersist* currentIterationVertices = this; //a polyline is actually a ReferenceCloud!
    ccText* smoothPoly = nullptr;

	bool openPoly = !isClosed();

	for (unsigned it = 0; it < iterationCount; ++it)
	{
		//reserve memory for the new vertices
		unsigned vertCount = currentIterationVertices->size();
		unsigned segmentCount = (openPoly ? vertCount - 1 : vertCount);

		ccPointCloud* newStateVertices = new ccPointCloud("vertices");
		if (!newStateVertices->reserve(segmentCount * 2))
		{
			ccLog::Warning("[ccText::smoothChaikin] not enough memory");
			delete newStateVertices;
			newStateVertices = nullptr;
			delete currentIterationVertices;
			currentIterationVertices = nullptr;
			return nullptr;
		}

		if (openPoly)
		{
			//we always keep the first vertex
			newStateVertices->addPoint(*currentIterationVertices->getPoint(0));
		}

		for (unsigned i = 0; i < segmentCount; ++i)
		{
			unsigned iP = i;
			unsigned iQ = ((iP + 1) % vertCount);

			const CCVector3& P = *currentIterationVertices->getPoint(iP);
			const CCVector3& Q = *currentIterationVertices->getPoint(iQ);

			if (!openPoly || i != 0)
			{
				CCVector3 P0 = (CCCoreLib::PC_ONE - ratio) * P + ratio * Q;
				newStateVertices->addPoint(P0);
			}

			if (!openPoly || i + 1 != segmentCount)
			{
				CCVector3 P1 = ratio * P + (CCCoreLib::PC_ONE - ratio) * Q;
				newStateVertices->addPoint(P1);
			}
		}

		if (openPoly)
		{
			//we always keep the last vertex
			newStateVertices->addPoint(*currentIterationVertices->getPoint(currentIterationVertices->size() - 1));
		}

		if (currentIterationVertices != this)
		{
			delete currentIterationVertices;
			currentIterationVertices = nullptr;
		}
		currentIterationVertices = newStateVertices;

		//last iteration?
		if (it + 1 == iterationCount)
		{
			//smoothPoly = new ccLines(newStateVertices);
			//smoothPoly->addChild(newStateVertices);
			//newStateVertices->setEnabled(false);
			//if (!smoothPoly->reserve(newStateVertices->size()))
			//{
			//	ccLog::Warning("[ccLines::smoothChaikin] not enough memory");
			//	delete smoothPoly;
			//	return nullptr;
			//}
			//smoothPoly->addPointIndex(0, newStateVertices->size());

			////copy state
			//smoothPoly->importParametersFrom(*this);
			//smoothPoly->setName(getName() + QString(".smoothed (ratio=%1)").arg(ratio));
		}
	}

	return smoothPoly;
}

bool ccText::IsCloudVerticesOfPolyline(ccGenericPointCloud* cloud, ccText** polyline/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a polyline
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::POLY_LINE) && static_cast<ccText*>(parent)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<ccText*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::POLY_LINE) && static_cast<ccText*>(child)->getAssociatedCloud() == cloud)
		{
			if (polyline)
			{
				*polyline = static_cast<ccText*>(child);
			}
			return true;
		}
	}

	return false;
}

bool ccText::createNewPolylinesFromSelection(std::vector<ccText*>& output)
{
	if (!m_theAssociatedCloud)
	{
		assert(false);
		return false;
	}
	unsigned vertCount = m_theAssociatedCloud->size();
	
	//vertices visibility
	ccGenericPointCloud* verticesCloud = dynamic_cast<ccGenericPointCloud*>(getAssociatedCloud());
	if (!verticesCloud)
	{
		// no visibility table instantiated
		ccLog::Warning("[ccText::createNewPolylinesFromSelection] Unsupported vertex cloud");
		return false;
	}
	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = verticesCloud->getTheVisibilityArray();
	if (verticesVisibility.size() < vertCount)
	{
		// no visibility table instantiated
		ccLog::Warning("[ccText::createNewPolylinesFromSelection] No visibility table instantiated");
		return false;
	}

	bool success = true;
	{
        ccText* chunkPoly = nullptr;
		ccPointCloud* chunkCloud = nullptr;

		unsigned maxIndex = (m_isClosed ? vertCount : vertCount - 1);
		for (unsigned i = 0; i < maxIndex; ++i)
		{
			unsigned nextIndex = ((i + 1) % vertCount);
			bool kept = false;
			if (verticesVisibility.at(i) == CCCoreLib::POINT_VISIBLE && verticesVisibility.at(nextIndex) == CCCoreLib::POINT_VISIBLE) // segment should be kept
			{
				kept = true;

				const CCVector3* P0 = getPoint(i);
				const CCVector3* P1 = getPoint(nextIndex);

				// recreate a chunk if none is ready yet
				static const unsigned DefaultPolySizeIncrement = 64;
				if (!chunkPoly)
				{
					/*chunkCloud = new ccPointCloud("vertices");
					chunkCloud->setEnabled(false);
					chunkPoly = new ccLines(chunkCloud);
					chunkPoly->addChild(chunkCloud);
					if (!chunkPoly->reserve(DefaultPolySizeIncrement) || !chunkCloud->reserve(DefaultPolySizeIncrement))
					{
						delete chunkCloud;
						success = false;
						break;
					}
					chunkPoly->addPointIndex(0);
					chunkCloud->addPoint(*P0);*/
				}
				else if (chunkPoly->size() == chunkPoly->capacity())
				{
					if (!chunkPoly->reserve(chunkPoly->size() + DefaultPolySizeIncrement) || !chunkCloud->reserve(chunkCloud->size() + DefaultPolySizeIncrement))
					{
						success = false;
						break;
					}
				}

				// add the next vertex
				chunkPoly->addPointIndex(chunkCloud->size());
				chunkCloud->addPoint(*P1);
			}

			if (!kept || i + 1 == maxIndex)
			{
				// store the active chunk (if any)
				if (chunkPoly)
				{
					chunkPoly->importParametersFrom(*this);
					chunkPoly->setName(getName() + QString(".segmented (part %1)").arg(output.size() + 1));
					chunkCloud->shrinkToFit();
					chunkPoly->resize(chunkPoly->size());
					try
					{
						output.push_back(chunkPoly);
					}
					catch (const std::bad_alloc&)
					{
						success = false;
						break;
					}
					chunkPoly = nullptr;
				}
			}
		}
	}

	if (!success)
	{
		ccLog::Warning("[ccText::createNewPolylinesFromSelection] Not enough memory");
		// delete the already created polylines
		for (ccText* poly : output)
		{
			delete poly;
		}
		output.clear();
	}

	return success;
}
