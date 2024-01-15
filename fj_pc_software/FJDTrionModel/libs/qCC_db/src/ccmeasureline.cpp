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

//System
#include <assert.h>
#include <string.h>
#include "ccColorTypes.h"
#include "ccmeasureline.h"
#include "ccGLDrawContext.h"
#include <QFontMetrics>


ccMeasureLine::ccMeasureLine(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
{
	lockVisibility(false);
	setEnabled(true);
}


QString ccMeasureLine::getName() const
{
	return m_name;
}



void ccMeasureLine::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccMeasureLine::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;
	return true;
}

bool ccMeasureLine::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;
	return true;
}


void ccMeasureLine::drawMeOnly(CC_DRAW_CONTEXT& context)
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

void ccMeasureLine::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
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


	int pointNum = m_pointData.size();
	ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 255);

	if(m_measureMode == MeasureLine_DISTANCE)
	{
		if (pointNum >= 2)
		{
			//画点
			glFunc->glPushAttrib(GL_POINT_BIT);
			glFunc->glPointSize(5.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glEnable(GL_POINT_SMOOTH);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_POINTS);
			for (int i =0;i < pointNum -2;i++)
			{
				glFunc->glVertex3f(m_pointData[i].x, m_pointData[i].y, m_pointData[i].z);
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_POINT_BIT

			//画线
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(2.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_LINES);
			glFunc->glVertex3f(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z);

			for (int ii = 1; ii < pointNum-1; ii++)
			{
				glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
				if (ii != (pointNum - 1))
				{
					glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
				}
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_LINE_BIT
		}
	}
	else if (m_measureMode == MeasureLine_ANGLE)
	{
		if (pointNum >= 2)
		{
			//画点
			glFunc->glPushAttrib(GL_POINT_BIT);
			glFunc->glPointSize(5.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glEnable(GL_POINT_SMOOTH);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_POINTS);
			for (int i = 0; i < pointNum - 1; i++)
			{
				glFunc->glVertex3f(m_pointData[i].x, m_pointData[i].y, m_pointData[i].z);
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_POINT_BIT

			//画线
			if (pointNum == 3)
			{
				glFunc->glPushAttrib(GL_LINE_BIT);
				glFunc->glLineWidth(2.0f);
				ccGL::Color4v(glFunc, color.rgba);
				glFunc->glDisable(GL_DEPTH_TEST);
				glFunc->glBegin(GL_LINES);
				glFunc->glVertex3f(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z);
				glFunc->glVertex3f(m_pointData[1].x, m_pointData[1].y, m_pointData[1].z);
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_LINE_BIT
			}
			//if (m_isShowLabel && pointNum == 3)
			//{
			//	QFont titleFont(context.display->getTextDisplayFont());
			//	double distancea = std::sqrt((m_pointData[1].x - m_pointData[0].x) * (m_pointData[1].x - m_pointData[0].x) + (m_pointData[1].y - m_pointData[0].y) * (m_pointData[1].y - m_pointData[0].y) + (m_pointData[1].z - m_pointData[0].z) * (m_pointData[1].z - m_pointData[0].z));
			//	double distanceb = std::sqrt((m_pointData[1].x - m_pointData[2].x) * (m_pointData[1].x - m_pointData[2].x) + (m_pointData[1].y - m_pointData[2].y) * (m_pointData[1].y - m_pointData[2].y) + (m_pointData[1].z - m_pointData[2].z) * (m_pointData[1].z - m_pointData[2].z));
			//	double distancec = std::sqrt((m_pointData[2].x - m_pointData[0].x) * (m_pointData[2].x - m_pointData[0].x) + (m_pointData[2].y - m_pointData[0].y) * (m_pointData[2].y - m_pointData[0].y) + (m_pointData[2].z - m_pointData[0].z) * (m_pointData[2].z - m_pointData[0].z));
			//	double cosA = (distancec *distancec + distanceb * distanceb - distancea * distancea) / 2.0 / distanceb / distancec;
			//	double cosB = (distancec *distancec + distancea * distancea - distanceb * distanceb) / 2.0 / distancea / distancec;
			//	double cosC = (distancea *distancea + distanceb * distanceb - distancec * distancec) / 2.0 / distanceb / distancea;
			//
			//	context.display->display3DLabel(QString("C:") + QString::number(acos(cosA)*57.296, 'f', 3), CCVector3(m_pointData[2].x, m_pointData[2].y, m_pointData[2].z), &color, titleFont);
			//	context.display->display3DLabel(QString("A:") + QString::number(acos(cosB)*57.296, 'f', 3), CCVector3(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z), &color, titleFont);
			//	context.display->display3DLabel(QString("B:") + QString::number(acos(cosC)*57.296, 'f', 3), CCVector3(m_pointData[1].x, m_pointData[1].y, m_pointData[1].z), &color, titleFont);
			//}
		}
	}
	else if (m_measureMode == MeasureLine_AERA)
	{
		if (pointNum >= 2)
		{
			//画点
			glFunc->glPushAttrib(GL_POINT_BIT);
			glFunc->glPointSize(5.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glEnable(GL_POINT_SMOOTH);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_POINTS);
			for (int i = 0; i < pointNum - 1; i++)
			{
				glFunc->glVertex3f(m_pointData[i].x, m_pointData[i].y, m_pointData[i].z);
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_POINT_BIT

			//画线
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(2.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_LINES);
			glFunc->glVertex3f(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z);

			for (int ii = 1; ii < pointNum - 1; ii++)
			{
				glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
				if (ii != (pointNum - 2))
				{
					glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
				}
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_LINE_BIT

		}
	}
	else if (m_measureMode == MeasureLine_WITHOUTENDPOINTS)
	{
        if (pointNum >= 2)
        {
            //画线
            glFunc->glPushAttrib(GL_LINE_BIT);
            glFunc->glLineWidth(2.0f);
            ccGL::Color4v(glFunc, color.rgba);
            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glBegin(GL_LINES);
            glFunc->glVertex3f(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z);

            for (int ii = 1; ii < pointNum; ii++)
            {
                glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
                if (ii != (pointNum - 1))
                {
                    glFunc->glVertex3f(m_pointData[ii].x, m_pointData[ii].y, m_pointData[ii].z);
                }
            }
            glFunc->glEnd();
            glFunc->glPopAttrib(); //GL_LINE_BIT

            ccGLCameraParameters camera;
            glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
            glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
            glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());
            m_areaPoint2dList.clear();
            for (size_t i = 0; i < pointNum; i++)
            {
                //project the point in 2D
                CCVector3 P3D = m_pointData[i].toFloat();
                CCVector3d P2D;
                camera.project(P3D, P2D);
                m_areaPoint2dList.push_back(P2D);
            }
        }
    }
	if (pushName)
	{
		glFunc->glPopName();
	}
}


void ccMeasureLine::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		glFunc->glPushName(getUniqueIDForDisplay());
	}
	ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 255);
	ccGLCameraParameters camera;
	int pointNum = m_areaPoint2dList.size();
    int pointNum3d = m_pointData.size();
	float halfW = context.glW / 2.0f;
	float halfH = context.glH / 2.0f;
    if (m_measureMode == MeasureLine_HEIGHT)
    {
        if (pointNum == 2)
        {
            glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
            //画点
            glFunc->glPushAttrib(GL_POINT_BIT);
            glFunc->glPointSize(5.0f);
            ccGL::Color4v(glFunc, color.rgba);
            glFunc->glEnable(GL_POINT_SMOOTH);
            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glBegin(GL_POINTS);
            glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
            glFunc->glEnd();
            glFunc->glPopAttrib(); //GL_POINT_BIT

            //画线
            glFunc->glPushAttrib(GL_LINE_BIT);
            glFunc->glLineWidth(2.0f);
            ccGL::Color4v(glFunc, color.rgba);
            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glBegin(GL_LINES);
            glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
            glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
            glFunc->glEnd();
            glFunc->glPopAttrib(); //GL_LINE_BIT
            if (m_isShowLabel)
            {
                QString displayTxt = QString(u8"△") + "Z:" + QString::number(std::abs(m_pointData[1].z - m_pointData[0].z), 'f', 3);
                QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
                font.setBold(true);
                context.display->displayText(displayTxt,
                    static_cast<int>((m_areaPoint2dList[0].x+ m_areaPoint2dList[1].x)/2.0) + context.labelMarkerTextShift_pix,
                    static_cast<int>((m_areaPoint2dList[0].y + m_areaPoint2dList[1].y) / 2.0) + context.labelMarkerTextShift_pix,
                    ccGenericGLDisplay::ALIGN_DEFAULT,
                    context.labelOpacity / 100.0f,
                    &color,
                    &font);
            }

            glFunc->glPopAttrib();

        }
    }
    else if (m_measureMode == MeasureLine_ANGLE)
    {
        if (pointNum >= 2)
        {

            //画线
            glFunc->glPushAttrib(GL_LINE_BIT);
            glFunc->glLineWidth(2.0f);
            ccGL::Color4v(glFunc, color.rgba);
            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glBegin(GL_LINES);
            glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
            glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
            if (pointNum == 3)
            {
                glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
                glFunc->glVertex2d(m_areaPoint2dList[2].x - halfW, m_areaPoint2dList[2].y - halfH);
                //glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
                //glFunc->glVertex2d(m_areaPoint2dList[2].x - halfW, m_areaPoint2dList[2].y - halfH);
            }
            glFunc->glEnd();
            glFunc->glPopAttrib(); //GL_LINE_BIT

            if (m_isShowLabel && pointNum == 3)
            {
                QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
                font.setBold(true);
                double distancea = std::sqrt((m_pointData[1].x - m_pointData[0].x) * (m_pointData[1].x - m_pointData[0].x) + (m_pointData[1].y - m_pointData[0].y) * (m_pointData[1].y - m_pointData[0].y) + (m_pointData[1].z - m_pointData[0].z) * (m_pointData[1].z - m_pointData[0].z));
                double distanceb = std::sqrt((m_pointData[1].x - m_pointData[2].x) * (m_pointData[1].x - m_pointData[2].x) + (m_pointData[1].y - m_pointData[2].y) * (m_pointData[1].y - m_pointData[2].y) + (m_pointData[1].z - m_pointData[2].z) * (m_pointData[1].z - m_pointData[2].z));
                double distancec = std::sqrt((m_pointData[2].x - m_pointData[0].x) * (m_pointData[2].x - m_pointData[0].x) + (m_pointData[2].y - m_pointData[0].y) * (m_pointData[2].y - m_pointData[0].y) + (m_pointData[2].z - m_pointData[0].z) * (m_pointData[2].z - m_pointData[0].z));
                double cosA = (distancec *distancec + distanceb * distanceb - distancea * distancea) / 2.0 / distanceb / distancec;
                double cosB = (distancec *distancec + distancea * distancea - distanceb * distanceb) / 2.0 / distancea / distancec;
                double cosC = (distancea *distancea + distanceb * distanceb - distancec * distancec) / 2.0 / distanceb / distancea;

                //context.display->display3DLabel(QString("C:") + QString::number(acos(cosA)*57.296, 'f', 3), CCVector3(m_pointData[2].x, m_pointData[2].y, m_pointData[2].z), &color, titleFont);
                //context.display->display3DLabel(QString("A:") + QString::number(acos(cosB)*57.296, 'f', 3), CCVector3(m_pointData[0].x, m_pointData[0].y, m_pointData[0].z), &color, titleFont);
                //context.display->display3DLabel(QString("B:") + QString::number(acos(cosC)*57.296, 'f', 3), CCVector3(m_pointData[1].x, m_pointData[1].y, m_pointData[1].z), &color, titleFont);
                //context.display->displayText(QString("C:") + QString::number(acos(cosA)*57.296, 'f', 3),
                //    static_cast<int>(m_areaPoint2dList[2].x) + context.labelMarkerTextShift_pix,
                //    static_cast<int>(m_areaPoint2dList[2].y) + context.labelMarkerTextShift_pix,
                //    ccGenericGLDisplay::ALIGN_DEFAULT,
                //    context.labelOpacity / 100.0f,
                //    &color,
                //    &font);
                //context.display->displayText(QString("A:") + QString::number(acos(cosB)*57.296, 'f', 3),
                //    static_cast<int>(m_areaPoint2dList[0].x) + context.labelMarkerTextShift_pix,
                //    static_cast<int>(m_areaPoint2dList[0].y) + context.labelMarkerTextShift_pix,
                //    ccGenericGLDisplay::ALIGN_DEFAULT,
                //    context.labelOpacity / 100.0f,
                //    &color,
                //    &font);
                context.display->displayText(QString("A:") + QString::number(acos(cosC)*57.296, 'f', 3),
                    static_cast<int>(m_areaPoint2dList[1].x) + context.labelMarkerTextShift_pix,
                    static_cast<int>(m_areaPoint2dList[1].y) + context.labelMarkerTextShift_pix,
                    ccGenericGLDisplay::ALIGN_DEFAULT,
                    context.labelOpacity / 100.0f,
                    &color,
                    &font);
            
            
            
            }
        }
    }
    else if (m_measureMode == MeasureLine_AERA)
	{
		if (pointNum > 2)
		{

			//画线
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(2.0f);
			ccGL::Color4v(glFunc, color.rgba);
			glFunc->glDisable(GL_DEPTH_TEST);
			glFunc->glBegin(GL_LINES);
			glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
			glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
			if (pointNum > 2)
			{
				glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
				glFunc->glVertex2d(m_areaPoint2dList[2].x - halfW, m_areaPoint2dList[2].y - halfH);
			}
			glFunc->glEnd();
			glFunc->glPopAttrib(); //GL_LINE_BIT
            if (pointNum >= 3 && !m_areaStr.isEmpty())
            {
                QFont titleFont(context.display->getTextDisplayFont());
                context.display->displayText(m_areaStr,
                    static_cast<int>(m_areaPoint2dList[2].x) + context.labelMarkerTextShift_pix,
                    static_cast<int>(m_areaPoint2dList[2].y) + context.labelMarkerTextShift_pix,
                    ccGenericGLDisplay::ALIGN_DEFAULT,
                    context.labelOpacity / 100.0f,
                    &color,
                    &titleFont);
            }
		}
	}
	else if (m_measureMode == MeasureLine_DISTANCE && m_areaPoint2dList.size()>1)
	{
        if (pointNum == 2)
        {
            glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);

            //画线
            glFunc->glPushAttrib(GL_LINE_BIT);
            glFunc->glLineWidth(2.0f);
            ccGL::Color4v(glFunc, color.rgba);
            glFunc->glDisable(GL_DEPTH_TEST);
            glFunc->glBegin(GL_LINES);
            glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);
            glFunc->glVertex2d(m_areaPoint2dList[1].x - halfW, m_areaPoint2dList[1].y - halfH);
            glFunc->glEnd();
            glFunc->glPopAttrib(); //GL_LINE_BIT


            glFunc->glPopAttrib();

        }
		if (m_isShowLabel)
		{
			QFont titleFont(context.display->getTextDisplayFont());
			CCVector3 displayPos((m_areaPoint2dList[pointNum - 1].x + m_areaPoint2dList[pointNum - 2].x) / 2.0, (m_areaPoint2dList[pointNum - 1].y + m_areaPoint2dList[pointNum - 2].y) / 2.0, (m_areaPoint2dList[pointNum - 1].z + m_areaPoint2dList[pointNum - 2].z) / 2.0);
			double dx = m_pointData[pointNum3d - 1].x - m_pointData[pointNum3d - 2].x;
			double dy = m_pointData[pointNum3d - 1].y - m_pointData[pointNum3d - 2].y;
			double dz = m_pointData[pointNum3d - 1].z - m_pointData[pointNum3d - 2].z;
			double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
			QString displayTxt = QString("Distance:") + QString::number(distance, 'f', 3);
			QFont bodyFont = context.display->getLabelDisplayFont();
			QFontMetrics bodyFontMetrics(bodyFont);
			int rowHeight = bodyFontMetrics.height();
			if (m_is3dPointMode)
			{
				context.display->displayText(displayTxt,
					displayPos.x,
					displayPos.y, ccGenericGLDisplay::ALIGN_DEFAULT, 0, &color, &bodyFont);
				displayTxt = QString(u8"△") + "X:" + QString::number(std::abs(dx), 'f', 3);
				context.display->displayText(displayTxt,
					displayPos.x,
					displayPos.y- rowHeight, ccGenericGLDisplay::ALIGN_DEFAULT, 0, &color, &bodyFont);
				displayTxt = QString(u8"△") + "Y:" + QString::number(std::abs(dy), 'f', 3);
				context.display->displayText(displayTxt,
					displayPos.x,
					displayPos.y - rowHeight*2, ccGenericGLDisplay::ALIGN_DEFAULT, 0, &color, &bodyFont);
				displayTxt = QString(u8"△") + "Z:" + QString::number(std::abs(dz), 'f', 3);
				context.display->displayText(displayTxt,
					displayPos.x,
					displayPos.y- rowHeight*3, ccGenericGLDisplay::ALIGN_DEFAULT, 0, &color, &bodyFont);
			}
			else
			{
				double distance2d = std::sqrt(dx * dx + dy * dy);
				displayTxt = QString("Distance:") + QString::number(distance2d, 'f', 3);
				context.display->displayText(displayTxt,
					displayPos.x,
					displayPos.y, ccGenericGLDisplay::ALIGN_DEFAULT, 0, &color, &bodyFont);
			}
			//context.display->display3DLabel(displayTxt, displayPos, &color, titleFont);
		}
	}
	if (pushName)
	{
		glFunc->glPopName();
	}
}


void ccMeasureLine::setLabelVisiable(bool isShow)
{
	m_isShowLabel = isShow;
}

void ccMeasureLine::setLastCurrentPos(const CCVector3d &point)
{
	if (m_pointData.size()> 0)
	{
		m_pointData[m_pointData.size() - 1] = point;
	}
}