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
#include "ccSectionAnalysisLine.h"
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
#include "FJStyleManager.h"

//根据三角形顶点计算点B角度
double getTriangleVertexAngle(QPointF pointA, QPointF pointB, QPointF pointC)
{
	double x1 = pointA.x();
	double y1 = pointA.y();
	double x2 = pointB.x();
	double y2 = pointB.y();
	double x3 = pointC.x();
	double y3 = pointC.y();
	double cosNum = ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) - (x1 - x3)*(x1 - x3) - (y1 - y3)*(y1 - y3)) / 2.0 / std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) / std::sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3));
	return acos(cosNum);
}

//计算两条线段是否相交
bool isTwoLineIntersect(QPointF a, QPointF b, QPointF c, QPointF d)
{
	double minnum = 1e-15;
	double area_abc = (a.x() - c.x()) * (b.y() - c.y()) - (a.y() - c.y()) * (b.x() - c.x());
	double area_abd = (a.x() - d.x()) * (b.y() - d.y()) - (a.y() - d.y()) * (b.x() - d.x());
	if (area_abc * area_abd >= -minnum)
	{
		return false;
	}
	double area_cda = (c.x() - a.x()) * (d.y() - a.y()) - (c.y() - a.y()) * (d.x() - a.x());
	double area_cdb = area_cda + area_abc - area_abd;
	if (area_cda * area_cdb >= -minnum)
	{
		return false;
	}
	return true;
}

//点是否在线段ab范围内
bool isInRect(QPointF a, QPointF point, QPointF b,double range)
{
	double x1 = a.x();
	double y1 = a.y();
	double x2 = b.x();
	double y2 = b.y();
	double dx = x2 - x1;
	double dy = y2 - y1;
	QVector<QPointF> vpf;
	QPointF pointStartUp(x1 + dy / std::sqrt(dx*dx + dy * dy)*range, y1 - dx / std::sqrt(dx*dx + dy * dy)*range);
	QPointF pointStartDown(x1 - dy / std::sqrt(dx*dx + dy * dy)*range, y1 + dx / std::sqrt(dx*dx + dy * dy)*range);
	{
		vpf.append(pointStartDown);
		vpf.append(pointStartUp);
	}

	QPointF point1(x2 - dy / std::sqrt(dx*dx + dy * dy)*range, y2 + dx / std::sqrt(dx*dx + dy * dy)*range);
	QPointF point2(x2 + dy / std::sqrt(dx*dx + dy * dy)*range, y2 - dx / std::sqrt(dx*dx + dy * dy)*range);
	if (isTwoLineIntersect(vpf[0], point1, vpf[1], point2))
	{
		vpf.append(point1);
		vpf.append(point2);
	}
	else
	{
		vpf.append(point2);
		vpf.append(point1);
	}
	QPolygonF polygon(vpf);
	if (polygon.containsPoint(point, Qt::WindingFill))
	{
		return true;
	}
	return false;
}

ccSectionAnalysisLine::ccSectionAnalysisLine(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
{
	lockVisibility(false);
	setEnabled(true);
}


QString ccSectionAnalysisLine::getName() const
{
	return m_name;
}

bool ccSectionAnalysisLine::acceptClick(int x, int y, Qt::MouseButton button)
{
	return false;
}

bool ccSectionAnalysisLine::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	return false;
}


void ccSectionAnalysisLine::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.
}


bool ccSectionAnalysisLine::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;
	return true;
}

bool ccSectionAnalysisLine::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;
	return true;
}


void ccSectionAnalysisLine::drawMeOnly(CC_DRAW_CONTEXT& context)
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

void ccSectionAnalysisLine::setPickPoint(const QPointF & startPoint, const QPointF & endPoint)
{
	m_startPoint = startPoint;
	m_endPoint = endPoint;
}

void ccSectionAnalysisLine::getPickPoint(QPointF & startPoint, QPointF & endPoint)
{
	startPoint = m_startPoint;
	endPoint = m_endPoint;
}

bool ccSectionAnalysisLine::getIsPickPoint()
{
	return m_ispickPoint;
}

void ccSectionAnalysisLine::setIsPickPoint(bool ispick)
{
	m_ispickPoint = ispick;
}


double ccSectionAnalysisLine::getThickness()
{
	return m_thickness;
}

void ccSectionAnalysisLine::setThickness(const double &  value)
{
	m_thickness = value;
}

CCVector3d ccSectionAnalysisLine::getOffset()
{
	return m_offset;
}

void ccSectionAnalysisLine::setOffset(const CCVector3d &  value)
{
	m_offset = value;
}


void ccSectionAnalysisLine::drawMeOnly3D(CC_DRAW_CONTEXT& context)
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
		//not particularly fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}
	std::vector<CCVector3d> CurrentPointData = getCurrentOffsetPointData();
	std::vector <std::vector<QPointF>>  data;
	getCurrentPointDataBoxVertex(data);
	if (CurrentPointData.size() > 1)
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(2.0f);
		//glFunc->Color4v(ccColor::Rgba(m_color.red(), m_color.green(), m_color.blue(), m_color.alpha()));
		ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 100);
		ccGL::Color4v(glFunc, color.rgba);
		glFunc->glBegin(GL_LINES);
		glFunc->glEnable(GL_DEPTH_TEST);
		int curPointSize = CurrentPointData.size();
		glFunc->glVertex3f(CurrentPointData[0].x, CurrentPointData[0].y, CurrentPointData[0].z);

		for (int ii = 1; ii < curPointSize; ii++)
		{
			glFunc->glVertex3f(CurrentPointData[ii].x, CurrentPointData[ii].y, CurrentPointData[ii].z);
			if (ii != (curPointSize - 1))
			{
				glFunc->glVertex3f(CurrentPointData[ii].x, CurrentPointData[ii].y, CurrentPointData[ii].z);
			}
		}
		glFunc->glEnd();

		glFunc->glPopAttrib(); //GL_LINE_BIT
	}



	if (CurrentPointData.size()>1 && m_isSelected)
	{
		//glFunc->glEnable(GL_DEPTH_TEST);
		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glFunc->glEnable(GL_BLEND);
		glFunc->glColor4ub(255, 200, 4, 80);

		int lineNum = data.size();
		for (int i = 0;i < lineNum;i++)
		{
			std::vector<QPointF> linePoint = data[i];
			if (linePoint.size() == 4)
			{
				if (m_ProjectionMode == LOOKDOWN)
				{
					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[1].x(), linePoint[1].y(), m_heightMin);
					glFunc->glVertex3f(linePoint[1].x(), linePoint[1].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[2].x(), linePoint[2].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[2].x(), linePoint[2].y(), m_heightMin);
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].x(), linePoint[0].y(), m_heightMin);
					glFunc->glVertex3f(linePoint[0].x(), linePoint[0].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[3].x(), linePoint[3].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[3].x(), linePoint[3].y(), m_heightMin);
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].x(), linePoint[0].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[1].x(), linePoint[1].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[2].x(), linePoint[2].y(), m_heightMax);
					glFunc->glVertex3f(linePoint[3].x(), linePoint[3].y(), m_heightMax);
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].x(), linePoint[0].y(), m_heightMin);
					glFunc->glVertex3f(linePoint[1].x(), linePoint[1].y(), m_heightMin);
					glFunc->glVertex3f(linePoint[2].x(), linePoint[2].y(), m_heightMin);
					glFunc->glVertex3f(linePoint[3].x(), linePoint[3].y(), m_heightMin);
					glFunc->glEnd();
				}
				else
				{
					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[1].y(), m_heightMin, linePoint[1].x());
					glFunc->glVertex3f(linePoint[1].y(), m_heightMax, linePoint[1].x());
					glFunc->glVertex3f(linePoint[2].y(), m_heightMax, linePoint[2].x());
					glFunc->glVertex3f(linePoint[2].y(), m_heightMin, linePoint[2].x());
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].y(), m_heightMin, linePoint[0].x());
					glFunc->glVertex3f(linePoint[0].y(), m_heightMax, linePoint[0].x());
					glFunc->glVertex3f(linePoint[3].y(), m_heightMax, linePoint[3].x());
					glFunc->glVertex3f(linePoint[3].y(), m_heightMin, linePoint[3].x());
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].y(), m_heightMin, linePoint[0].x());
					glFunc->glVertex3f(linePoint[1].y(), m_heightMin, linePoint[1].x());
					glFunc->glVertex3f(linePoint[2].y(), m_heightMin, linePoint[2].x());
					glFunc->glVertex3f(linePoint[3].y(), m_heightMin, linePoint[3].x());
					glFunc->glEnd();

					glFunc->glBegin(GL_QUADS);
					glFunc->glVertex3f(linePoint[0].y(), m_heightMax, linePoint[0].x());
					glFunc->glVertex3f(linePoint[1].y(), m_heightMax, linePoint[1].x());
					glFunc->glVertex3f(linePoint[2].y(), m_heightMax, linePoint[2].x());
					glFunc->glVertex3f(linePoint[3].y(), m_heightMax, linePoint[3].x());
					glFunc->glEnd();
				}
			}
		}

		glFunc->glPopAttrib();
	}

	if (CurrentPointData.size() > 0 && m_isSelected)
	{
		ccColor::Rgba color(255, 255, 255, 255);
		QFont titleFont(context.display->getTextDisplayFont());

		context.display->display3DLabel(QString("From"),
			CCVector3(CurrentPointData[0].x, CurrentPointData[0].y, CurrentPointData[0].z),
			&color,
			titleFont);
		if (CurrentPointData.size() > 1)
		{
			context.display->display3DLabel(QString("To"),
				CCVector3(CurrentPointData[CurrentPointData.size()-1].x, CurrentPointData[CurrentPointData.size() - 1].y, CurrentPointData[CurrentPointData.size() - 1].z),
				&color,
				titleFont);
		}
	}

	if (pushName)
	{
		glFunc->glPopName();
	}




}


void ccSectionAnalysisLine::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
    QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
    if (glFunc == nullptr)
    {
        assert(false);
        return;
    }
    if (!m_isShowMousePos || m_mousePoint.size() != 2)
    {
        return;
    }
    bool pushName = MACRO_DrawEntityNames(context);
    if (pushName)
    {
        glFunc->glPushName(getUniqueIDForDisplay());
    }
    if (m_isShowMousePos && m_mousePoint.size() == 2)
    {
        float halfW = context.glW / 2.0f;
        float halfH = context.glH / 2.0f;
        ccColor::Rgba color(m_color.red(), m_color.green(), m_color.blue(), 100);
        glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
        //画线
        glFunc->glPushAttrib(GL_LINE_BIT);
        glFunc->glLineWidth(2.0f);
        ccGL::Color4v(glFunc, color.rgba);
        glFunc->glDisable(GL_DEPTH_TEST);
        glFunc->glBegin(GL_LINES);
        glFunc->glVertex2d(m_mousePoint[0].x - halfW, m_mousePoint[0].y - halfH);
        glFunc->glVertex2d(m_mousePoint[1].x - halfW, m_mousePoint[1].y - halfH);
        glFunc->glEnd();
        glFunc->glPopAttrib(); //GL_LINE_BIT
        glFunc->glPopAttrib();
    }
    if (pushName)
    {
        glFunc->glPopName();
    }
}

void ccSectionAnalysisLine::setCurrentData(const std::vector<CCVector3d> & currentPointData)
{
	m_CurrentPointData = currentPointData;
}

void ccSectionAnalysisLine::getCurrentData(std::vector<CCVector3d> & currentPointData)
{
	currentPointData.clear();
	currentPointData = m_CurrentPointData;
}

void ccSectionAnalysisLine::setRangeParam(const double &  heightMin, const double &  heightMax)
{
	m_heightMin = heightMin;
	m_heightMax = heightMax;
}

void ccSectionAnalysisLine::getCurrentPointDataBoxVertex(std::vector <std::vector<QPointF>> & data)
{
	double thickness = m_thickness / 2.0;
	data.clear();
	std::vector<CCVector3d> offsetedData = getCurrentOffsetPointData();
	int pointNum = offsetedData.size();
	if (pointNum>1)
	{
		if (m_ProjectionMode == LOOKDOWN)
		{
			double x1 = offsetedData[0].x;
			double y1 = offsetedData[0].y;
			double x2 = offsetedData[1].x;
			double y2 = offsetedData[1].y;
			double x3 = 0;
			double y3 = 0;
			double dx = x2 - x1;
			double dy = y2 - y1;
			std::vector<QPointF> pointList;
			QPointF pointStartUp(x1 + dy / std::sqrt(dx*dx + dy * dy)*thickness, y1 - dx / std::sqrt(dx*dx + dy * dy)*thickness);
			QPointF pointStartDown(x1 - dy / std::sqrt(dx*dx + dy * dy)*thickness, y1 + dx / std::sqrt(dx*dx + dy * dy)*thickness);
			{
				pointList.push_back(pointStartDown);
				pointList.push_back(pointStartUp);
			}
			for (int i = 1; i <= pointNum - 1; i++)
			{
				if (i == pointNum - 1)
				{
					x1 = offsetedData[i - 1].x;
					y1 = offsetedData[i - 1].y;
					x2 = offsetedData[i].x;
					y2 = offsetedData[i].y;
					dx = x2 - x1;
					dy = y2 - y1;
					QPointF point1(x2 - dy / std::sqrt(dx*dx + dy * dy)*thickness, y2 + dx / std::sqrt(dx*dx + dy * dy)*thickness);
					QPointF point2(x2 + dy / std::sqrt(dx*dx + dy * dy)*thickness, y2 - dx / std::sqrt(dx*dx + dy * dy)*thickness);
					if (isTwoLineIntersect(pointList[0], point1, pointList[1], point2))
					{
						pointList.push_back(point1);
						pointList.push_back(point2);
					}
					else
					{
						pointList.push_back(point2);
						pointList.push_back(point1);
					}
					data.push_back(pointList);
				}
				else
				{
					x1 = offsetedData[i - 1].x;
					y1 = offsetedData[i - 1].y;
					x2 = offsetedData[i].x;
					y2 = offsetedData[i].y;
					x3 = offsetedData[i + 1].x;
					y3 = offsetedData[i + 1].y;
					dx = x2 - x1;
					dy = y2 - y1;
					double cosNum = ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) - (x1 - x3)*(x1 - x3) - (y1 - y3)*(y1 - y3)) / 2.0 / std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) / std::sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3));
					double thn = acos(cosNum) / 2;

					double vectorX = (x1 - x2) / std::sqrt(dx*dx + dy * dy)*(thickness / sin(thn));//|R| * cosA
					double vectorY = (y1 - y2) / std::sqrt(dx*dx + dy * dy)*(thickness / sin(thn));//|R| * sinA


					QPointF rotatePointA = QPointF(vectorX*cos(thn) - vectorY * sin(thn) + x2, vectorY*cos(thn) + vectorX * sin(thn) + y2);
					QPointF rotatePointB = QPointF(vectorX*cos(-thn) - vectorY * sin(-thn) + x2, vectorY*cos(-thn) + vectorX * sin(-thn) + y2);

					if (getTriangleVertexAngle(QPointF(x1, y1), QPointF(x2, y2), rotatePointA) < (thn * 2) && getTriangleVertexAngle(QPointF(x3, y3), QPointF(x2, y2), rotatePointA) < (thn * 2))
					{
						rotatePointB = rotatePointA;//rotatePointB为真实值
					}
					rotatePointA = QPointF(x2 - (rotatePointB.x() - x2), y2 - (rotatePointB.y() - y2));


					double thnn = thn / 2.0 - 3.14157 / 2.0 + atan(abs(dx) / std::sqrt(dx*dx + dy * dy));

					if (isTwoLineIntersect(pointList[0], rotatePointA, pointList[1], rotatePointB))
					{
						pointList.push_back(rotatePointA);
						pointList.push_back(rotatePointB);
					}
					else
					{
						pointList.push_back(rotatePointB);
						pointList.push_back(rotatePointA);
					}
					data.push_back(pointList);
					pointList.clear();
					pointList.push_back(rotatePointA);
					pointList.push_back(rotatePointB);
				}
			}
		}
		else
		{
			double x1 = offsetedData[0].z;
			double y1 = offsetedData[0].x;
			double x2 = offsetedData[1].z;
			double y2 = offsetedData[1].x;
			double x3 = 0;
			double y3 = 0;
			double dx = x2 - x1;
			double dy = y2 - y1;
			std::vector<QPointF> pointList;
			QPointF pointStartUp(x1 + dy / std::sqrt(dx*dx + dy * dy)*thickness, y1 - dx / std::sqrt(dx*dx + dy * dy)*thickness);
			QPointF pointStartDown(x1 - dy / std::sqrt(dx*dx + dy * dy)*thickness, y1 + dx / std::sqrt(dx*dx + dy * dy)*thickness);
			{
				pointList.push_back(pointStartDown);
				pointList.push_back(pointStartUp);
			}
			for (int i = 1; i <= pointNum - 1; i++)
			{
				if (i == pointNum - 1)
				{
					x1 = offsetedData[i - 1].z;
					y1 = offsetedData[i - 1].x;
					x2 = offsetedData[i].z;
					y2 = offsetedData[i].x;
					dx = x2 - x1;
					dy = y2 - y1;
					QPointF point1(x2 - dy / std::sqrt(dx*dx + dy * dy)*thickness, y2 + dx / std::sqrt(dx*dx + dy * dy)*thickness);
					QPointF point2(x2 + dy / std::sqrt(dx*dx + dy * dy)*thickness, y2 - dx / std::sqrt(dx*dx + dy * dy)*thickness);
					if (isTwoLineIntersect(pointList[0], point1, pointList[1], point2))
					{
						pointList.push_back(point1);
						pointList.push_back(point2);
					}
					else
					{
						pointList.push_back(point2);
						pointList.push_back(point1);
					}
					data.push_back(pointList);
				}
				else
				{
					x1 = offsetedData[i - 1].z;
					y1 = offsetedData[i - 1].x;
					x2 = offsetedData[i].z;
					y2 = offsetedData[i].x;
					x3 = offsetedData[i + 1].z;
					y3 = offsetedData[i + 1].x;
					dx = x2 - x1;
					dy = y2 - y1;
					double cosNum = ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) - (x1 - x3)*(x1 - x3) - (y1 - y3)*(y1 - y3)) / 2.0 / std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) / std::sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3));
					double thn = acos(cosNum) / 2;

					double vectorX = (x1 - x2) / std::sqrt(dx*dx + dy * dy)*(thickness / sin(thn));//|R| * cosA
					double vectorY = (y1 - y2) / std::sqrt(dx*dx + dy * dy)*(thickness / sin(thn));//|R| * sinA


					QPointF rotatePointA = QPointF(vectorX*cos(thn) - vectorY * sin(thn) + x2, vectorY*cos(thn) + vectorX * sin(thn) + y2);
					QPointF rotatePointB = QPointF(vectorX*cos(-thn) - vectorY * sin(-thn) + x2, vectorY*cos(-thn) + vectorX * sin(-thn) + y2);

					if (getTriangleVertexAngle(QPointF(x1, y1), QPointF(x2, y2), rotatePointA) < (thn * 2) && getTriangleVertexAngle(QPointF(x3, y3), QPointF(x2, y2), rotatePointA) < (thn * 2))
					{
						rotatePointB = rotatePointA;//rotatePointB为真实值
					}
					rotatePointA = QPointF(x2 - (rotatePointB.x() - x2), y2 - (rotatePointB.y() - y2));


					double thnn = thn / 2.0 - 3.14157 / 2.0 + atan(abs(dx) / std::sqrt(dx*dx + dy * dy));

					if (isTwoLineIntersect(pointList[0], rotatePointA, pointList[1], rotatePointB))
					{
						pointList.push_back(rotatePointA);
						pointList.push_back(rotatePointB);
					}
					else
					{
						pointList.push_back(rotatePointB);
						pointList.push_back(rotatePointA);
					}
					data.push_back(pointList);
					pointList.clear();
					pointList.push_back(rotatePointA);
					pointList.push_back(rotatePointB);
				}
			}
		}






	}
}


bool ccSectionAnalysisLine::pointPicking(const QPointF& clickPos,
	const ccGLCameraParameters& camera) 
{
	if (!m_isSelected)
	{
		return false;
	}
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;
	{
		std::vector <std::vector<QPointF>>  data;
		getCurrentPointDataBoxVertex(data);
		int lineNum = data.size();
		for (int i = 0; i < lineNum; i++)
		{
			std::vector<QPointF> linePoint = data[i];
			if (linePoint.size() == 4)
			{
				if (m_ProjectionMode == LOOKDOWN)
				{
					CCVector3d Q3D0(linePoint[0].x(), linePoint[0].y(), m_heightMin);
					CCVector3d Q3D1(linePoint[1].x(), linePoint[1].y(), m_heightMin);
					CCVector3d Q3D2(linePoint[2].x(), linePoint[2].y(), m_heightMin);
					CCVector3d Q3D3(linePoint[3].x(), linePoint[3].y(), m_heightMin);

					CCVector3d Q3U0(linePoint[0].x(), linePoint[0].y(), m_heightMax);
					CCVector3d Q3U1(linePoint[1].x(), linePoint[1].y(), m_heightMax);
					CCVector3d Q3U2(linePoint[2].x(), linePoint[2].y(), m_heightMax);
					CCVector3d Q3U3(linePoint[3].x(), linePoint[3].y(), m_heightMax);

					CCVector3d Q2D;
					camera.project(Q3D0, Q2D);
					QPointF Q3D0To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D1, Q2D);
					QPointF Q3D1To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D2, Q2D);
					QPointF Q3D2To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D3, Q2D);
					QPointF Q3D3To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U0, Q2D);
					QPointF Q3U0To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U1, Q2D);
					QPointF Q3U1To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U2, Q2D);
					QPointF Q3U2To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U3, Q2D);
					QPointF Q3U3To2D(Q2D.x - half_w, Q2D.y - half_h);
					if (isInRect(Q3D0To2D, clickPos, Q3D3To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = false;
						return true;
					}

					if (isInRect(Q3D1To2D, clickPos, Q3D2To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = false;
						return true;
					}

					if (isInRect(Q3U0To2D, clickPos, Q3U3To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = true;
						return true;
					}

					if (isInRect(Q3U1To2D, clickPos, Q3U2To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = true;
						return true;
					}
				}
				else
				{
					CCVector3d Q3D0(linePoint[0].y(), m_heightMin, linePoint[0].x());
					CCVector3d Q3D1(linePoint[1].y(), m_heightMin, linePoint[1].x());
					CCVector3d Q3D2(linePoint[2].y(), m_heightMin, linePoint[2].x());
					CCVector3d Q3D3(linePoint[3].y(), m_heightMin, linePoint[3].x());

					CCVector3d Q3U0(linePoint[0].y(), m_heightMax, linePoint[0].x());
					CCVector3d Q3U1(linePoint[1].y(), m_heightMax, linePoint[1].x());
					CCVector3d Q3U2(linePoint[2].y(), m_heightMax, linePoint[2].x());
					CCVector3d Q3U3(linePoint[3].y(), m_heightMax, linePoint[3].x());

					CCVector3d Q2D;
					camera.project(Q3D0, Q2D);
					QPointF Q3D0To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D1, Q2D);
					QPointF Q3D1To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D2, Q2D);
					QPointF Q3D2To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3D3, Q2D);
					QPointF Q3D3To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U0, Q2D);
					QPointF Q3U0To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U1, Q2D);
					QPointF Q3U1To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U2, Q2D);
					QPointF Q3U2To2D(Q2D.x - half_w, Q2D.y - half_h);

					camera.project(Q3U3, Q2D);
					QPointF Q3U3To2D(Q2D.x - half_w, Q2D.y - half_h);
					if (isInRect(Q3D0To2D, clickPos, Q3D3To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = false;
						return true;
					}

					if (isInRect(Q3D1To2D, clickPos, Q3D2To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = false;
						return true;
					}

					if (isInRect(Q3U0To2D, clickPos, Q3U3To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = true;
						return true;
					}

					if (isInRect(Q3U1To2D, clickPos, Q3U2To2D, 5))
					{
						m_movePart = i;
						m_moveUpPart = true;
						return true;
					}
				}
			}
		}

	}

	return false;
}

void ccSectionAnalysisLine::changePosition(QPointF clickpos, const ccGLCameraParameters& camera)
{
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;
	double height = m_moveUpPart ? m_heightMax : m_heightMin;
	if (m_movePart < m_CurrentPointData.size()-1)
	{
		std::vector<CCVector3d> offsetedData = getCurrentOffsetPointData();
		if (m_ProjectionMode == LOOKDOWN)
		{
			CCVector3d Q3D0(offsetedData[m_movePart].x, offsetedData[m_movePart].y, height);
			CCVector3d Q2D;
			camera.project(Q3D0, Q2D);

			CCVector3d Q3D;
			CCVector3d p2D(clickpos.x() + half_w, clickpos.y() + half_h, 0);
			camera.unprojectz(p2D, Q3D, Q2D.z);

			QPointF pointA = QPointF(offsetedData[m_movePart].x, offsetedData[m_movePart].y);
			QPointF pointB = QPointF(offsetedData[m_movePart + 1].x, offsetedData[m_movePart + 1].y);
			QPointF pointC = QPointF(Q3D.x, Q3D.y);
			double area = (pointA.x() * pointB.y() - pointA.x() * pointC.y() + pointB.x() * pointC.y() - pointB.x() * pointA.y() + pointC.x() * pointA.y() - pointC.x() * pointB.y());
			double dx = pointA.x() - pointB.x();
			double dy = pointA.y() - pointB.y();
			m_thickness = std::abs(area / std::sqrt(dx*dx + dy * dy))*2.0;
			FJStyleManager::Instance()->updateSectionThickness(getName(),m_thickness);
		}
		else
		{
			//CCVector3d Q3D0(offsetedData[m_movePart].x, height, offsetedData[m_movePart].z);
			//CCVector3d Q2D;
			//camera.project(Q3D0, Q2D);

			CCVector3d Q3D;
			CCVector3d p2D(clickpos.x() + half_w, clickpos.y() + half_h, 0);
			camera.unprojecty(p2D, Q3D, height);

			QPointF pointA = QPointF(offsetedData[m_movePart].z, offsetedData[m_movePart].x);
			QPointF pointB = QPointF(offsetedData[m_movePart + 1].z, offsetedData[m_movePart + 1].x);
			QPointF pointC = QPointF(Q3D.z, Q3D.x);
			double area = (pointA.x() * pointB.y() - pointA.x() * pointC.y() + pointB.x() * pointC.y() - pointB.x() * pointA.y() + pointC.x() * pointA.y() - pointC.x() * pointB.y());
			double dx = pointA.x() - pointB.x();
			double dy = pointA.y() - pointB.y();
			m_thickness = std::abs(area / std::sqrt(dx*dx + dy * dy))*2.0;
			FJStyleManager::Instance()->updateSectionThickness(getName(), m_thickness);
		}

	}
}

void ccSectionAnalysisLine::setMousePos(const std::vector<CCVector3d> & mousePoint)
{
	m_mousePoint = mousePoint;
}

void ccSectionAnalysisLine::setMousePosVisiable(bool isShow)
{
	m_isShowMousePos = isShow;
}

bool ccSectionAnalysisLine::getMousePosVisiable()
{
	return m_isShowMousePos;
}

void ccSectionAnalysisLine::setProjectionMode(ProjectionDirection projectionMode)
{
	m_ProjectionMode = projectionMode;
}

std::vector<CCVector3d> ccSectionAnalysisLine::getCurrentOffsetPointData()
{
	std::vector<CCVector3d> data;
	int PointNum = m_CurrentPointData.size();
	for (int i = 0; i < PointNum; i++)
	{
		if (m_ProjectionMode == LOOKDOWN)
		{
			CCVector3d newPointData(m_CurrentPointData[i].x + m_offset.x, m_CurrentPointData[i].y + m_offset.y, m_CurrentPointData[i].z + m_offset.z);
			data.push_back(newPointData);
		}
		else
		{
            CCVector3d newPointData(m_CurrentPointData[i].x + m_offset.x, m_CurrentPointData[i].y + m_offset.y, m_CurrentPointData[i].z + m_offset.z);
			data.push_back(newPointData);
		}
	}
	return data;
}

void ccSectionAnalysisLine::setIsSelected(bool isselect)
{
	m_isSelected = isselect;
}

bool ccSectionAnalysisLine::getIsSelected()
{
	return m_isSelected;
}

ProjectionDirection ccSectionAnalysisLine::getProjectionMode()
{
	return m_ProjectionMode;
}

bool ccSectionAnalysisLine::getIsMeasureOpen()
{
	return m_isMeasureOpen;
}

void ccSectionAnalysisLine::setIsMeasureOpen(bool ispick)
{
	m_isMeasureOpen = ispick;
}