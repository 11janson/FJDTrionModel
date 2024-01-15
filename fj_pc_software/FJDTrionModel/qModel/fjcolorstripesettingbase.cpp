#include "fjcolorstripesettingbase.h"

FJColorStripeSettingBase::FJColorStripeSettingBase(QWidget *parent) :
	CS::Widgets::FramelessDialog(parent)
{

}


FJColorStripeSettingBase::~FJColorStripeSettingBase()
{
}

void FJColorStripeSettingBase::setParam(ccHObject * ent, ccGLWindow * Win)
{
	m_object = ent;
	m_Win = Win;
	init(ent, Win);
}