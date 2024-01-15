#ifndef __FJCOLORSTRIPESETTINGBASE_H__
#define __FJCOLORSTRIPESETTINGBASE_H__

#include <QDialog>
#include "ccHObject.h"
#include "ccGLWindow.h"
#include "framelessdialog.h"
#include "fjpointcloudutil.h"

namespace Ui {
class FJColorStripeSettingBase;
}

class FJColorStripeSettingBase : public CS::Widgets::FramelessDialog
{
    Q_OBJECT

public:
    explicit FJColorStripeSettingBase(QWidget *parent = nullptr);
    ~FJColorStripeSettingBase();

	void setParam(ccHObject * ent, ccGLWindow * Win);
	virtual void init(ccHObject * ent, ccGLWindow * Win) {}
protected:
	ccHObject * m_object;
	ccGLWindow * m_Win;
};

#endif // FJCOLORSTRIPESETTINGSCLASSFICATION_H
