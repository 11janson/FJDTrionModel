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

#include "ccBBox.h"

void ccBBox::draw(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) const
{
	if (!m_valid)
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;
	if (m_isNeedDrawMat)
	{
		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPushMatrix();
		glFunc->glMultMatrixf(m_drawmat.data());
	}

	glFunc->glLineWidth(width_);

#if 0
	CCVector3 bboxDiag = getDiagVec();
	CCVector3 bboxCenter = getCenter();


	ccColor::Rgb faceColor(255, 0, 0);
	ccGL::Color3v(glFunc, faceColor.rgb);

	glFunc->glBegin(GL_QUADS);
	ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0, static_cast<double>(-bboxDiag.z/2));//union =ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0, static_cast<double>(-bboxDiag.z/2));
	ccGL::Vertex3(glFunc, static_cast<double>(bboxDiag.x / 2.0f), 0.0, static_cast<double>(bboxDiag.z/2));
	ccGL::Vertex3(glFunc, static_cast<double>(-bboxDiag.x / 2.0f), 0.0, static_cast<double>(bboxDiag.z/2));
	glFunc->glEnd();

	//width = (m_bbMax.x, m_bbMin.y, m_bbMin.z) - (m_bbMax.x, m_bbMax.y, m_bbMin.z)
	double width = sqrt(pow((m_bbMax.x - m_bbMax.x), 2) + pow((m_bbMin.y - m_bbMax.y), 2) + pow((m_bbMin.z - m_bbMin.z), 2));
	//length = (m_bbMax.x, m_bbMax.y, m_bbMin.z) - (m_bbMin.x, m_bbMax.y, m_bbMin.z)
	double length = sqrt(pow((m_bbMax.x - m_bbMin.x), 2) + pow((m_bbMax.y - m_bbMax.y), 2) + pow((m_bbMin.z - m_bbMin.z), 2));
	//height = (m_bbMin.x, m_bbMin.y, m_bbMin.z) - (m_bbMin.x, m_bbMin.y, m_bbMax.z)
	double height = sqrt(pow((m_bbMin.x - m_bbMin.x), 2) + pow((m_bbMin.y - m_bbMin.y), 2) + pow((m_bbMin.z - m_bbMax.z), 2));
#endif

	ccGL::Color3v(glFunc, col.rgb);
	glFunc->glEnable(GL_LINE_SMOOTH);
	glFunc->glEnable(GL_BLEND);
	glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glFunc->glHint(GL_LINE_SMOOTH, GL_NICEST);

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3v(glFunc, m_bbMin.u);//union =ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMin.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3v(glFunc, m_bbMax.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMax.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINES);
	ccGL::Vertex3v(glFunc, m_bbMin.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMin.y, m_bbMax.z);
	ccGL::Vertex3(glFunc, m_bbMax.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3v(glFunc, m_bbMax.u);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMin.z);
	ccGL::Vertex3(glFunc, m_bbMin.x, m_bbMax.y, m_bbMax.z);
	glFunc->glEnd();

    glFunc->glDisable(GL_BLEND);
    glFunc->glDisable(GL_LINE_SMOOTH);
	if (m_isNeedDrawMat)
	{
		glFunc->glPopMatrix();
	}
}

const ccBBox ccBBox::operator * (const ccGLMatrix& mat)
{
	ccBBox rotatedBox;

	if (m_valid)
	{
		rotatedBox.add(mat * m_bbMin);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMin.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMin.z));
		rotatedBox.add(mat * m_bbMax);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMax.z));
	}

	return rotatedBox;
}

const ccBBox ccBBox::operator * (const ccGLMatrixd& mat)
{
	ccBBox rotatedBox;

	if (m_valid)
	{
		rotatedBox.add(mat * m_bbMin);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMin.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMin.z));
		rotatedBox.add(mat * m_bbMax);
		rotatedBox.add(mat * CCVector3(m_bbMin.x,m_bbMax.y,m_bbMax.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMax.y,m_bbMin.z));
		rotatedBox.add(mat * CCVector3(m_bbMax.x,m_bbMin.y,m_bbMax.z));
	}

	return rotatedBox;
}
