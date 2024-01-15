//##########################################################################
//œ‘ æ∞¸Œß∫–
//janson.yang 2022.6.6
//##########################################################################

//Always first
#include "ccIncludeGL.h"

#include "ccShowBox.h"

//system
#include <cassert>

ccShowBox::ccShowBox(QString name)
	: ccHObject(name)
{
	setEnabled(true);
	setBBoxWidthColor(3.0, ccColor::red);
}

ccShowBox::~ccShowBox()
{
	
}

void ccShowBox::setBox(const ccBBox& box)
{
	m_box = box;
	m_box.setValidity(true);
}

void ccShowBox::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	if (!m_box.isValid())
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;


	//if (m_showBox)
	{
		//m_box.draw(m_selected ? context.bbDefaultCol : ccColor::magenta);
		m_box.draw(context, box_color);
	}
	
	if (!m_selected)
	{
		//nothing to show
		return;
	}

	//standard case: list names pushing (1st level)
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	//draw the interactors
	{


		if (pushName) //2nd level = sub-item
		{
			glFunc->glPushName(0); //fake ID, will be replaced by the arrows one if any
		}


		if (pushName)
		{
			glFunc->glPopName();
		}
	}

	if (pushName)
	{
		glFunc->glPopName();
	}
}