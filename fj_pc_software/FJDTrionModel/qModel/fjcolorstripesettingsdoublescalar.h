#ifndef FJCOLORSTRIPESETTINGSDOUBLESCALAR_H
#define FJCOLORSTRIPESETTINGSDOUBLESCALAR_H

#include <QDialog>
#include "ccHObject.h"
#include "ccGLWindow.h"

#include "ccColorScaleEditorWidget.h"
#include <QSharedPointer>
#include "fjcolorstripesettingbase.h"
namespace Ui {
class FJColorStripeSettingsDoubleScalar;
}

class FJColorStripeSettingsDoubleScalar : public FJColorStripeSettingBase
{
    Q_OBJECT

public:
    explicit FJColorStripeSettingsDoubleScalar(QWidget *parent = nullptr);
    ~FJColorStripeSettingsDoubleScalar();
	virtual void init(ccHObject * ent, ccGLWindow * Win);
public slots:
	void refreashShow(int value);
	void colorScaleSelected(int value);

private:
    Ui::FJColorStripeSettingsDoubleScalar *ui;
	double m_startNumOne;
	double m_stopNumOne;
	double m_startNumTwo;
	double m_stopNumTwo;
	double m_sfMinOne;
	double m_sfMaxOne;
	double m_sfMinTwo;
	double m_sfMaxTwo;
	int m_scaleIndex;
	QString m_scaleName1 = "unknown";
	QString m_scaleName2 = "unknown";
	ColorBarWidget * m_dlg;
	QSharedPointer<ColorScaleElementSliders> m_slider;
};

#endif // FJCOLORSTRIPESETTINGSDOUBLESCALAR_H
