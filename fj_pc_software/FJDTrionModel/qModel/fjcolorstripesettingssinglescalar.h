#ifndef FJCOLORSTRIPESETTINGSSINGLESCALAR_H
#define FJCOLORSTRIPESETTINGSSINGLESCALAR_H

#include <QDialog>
#include "ccHObject.h"
#include "ccGLWindow.h"
#include "framelessdialog.h"
#include "ccColorScaleEditorWidget.h"
#include <QSharedPointer>
#include "fjcolorstripesettingbase.h"
namespace Ui {
class FJColorStripeSettingsSingleScalar;
}

class FJColorStripeSettingsSingleScalar : public FJColorStripeSettingBase
{
    Q_OBJECT

public:
    explicit FJColorStripeSettingsSingleScalar(QWidget *parent = nullptr);
    ~FJColorStripeSettingsSingleScalar();
	virtual void init(ccHObject * ent, ccGLWindow * Win);
public slots:
	void refreashShow(int value);
	void colorScaleSelected(int value);

private:
    Ui::FJColorStripeSettingsSingleScalar *ui;
	double m_startNum;
	double m_stopNum;
	double m_sfMin;
	double m_sfMax;
	int m_scaleIndex;
	ColorBarWidget * m_dlg;
	QSharedPointer<ColorScaleElementSliders> m_slider;
};

#endif // FJCOLORSTRIPESETTINGSSINGLESCALAR_H
