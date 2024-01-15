#ifndef __FJCOLORSTRIPESETTINGSMISSINGCOLORING_H__
#define __FJCOLORSTRIPESETTINGSMISSINGCOLORING_H__

#include <QDialog>
#include "ccHObject.h"
#include "ccGLWindow.h"
#include "framelessdialog.h"
#include "fjcolorstripesettingbase.h"
namespace Ui {
class FJColorStripeSettingsMissingColoring;
}

class FJColorStripeSettingsMissingColoring : public FJColorStripeSettingBase
{
    Q_OBJECT

public:
    explicit FJColorStripeSettingsMissingColoring(QWidget *parent = nullptr);
    ~FJColorStripeSettingsMissingColoring();

	virtual void init(ccHObject * ent, ccGLWindow * Win);
public slots:
	void refreashShow();

    void resetStyle();
private:
    Ui::FJColorStripeSettingsMissingColoring *ui;
    bool m_isShowHidePoint = true;
};

#endif // __FJCOLORSTRIPESETTINGSMISSINGCOLORING_H__
