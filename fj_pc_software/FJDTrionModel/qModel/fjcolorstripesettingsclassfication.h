#ifndef FJCOLORSTRIPESETTINGSCLASSFICATION_H
#define FJCOLORSTRIPESETTINGSCLASSFICATION_H

#include <QDialog>
#include "ccHObject.h"
#include "ccGLWindow.h"

#include "fjpointcloudutil.h"
#include "fjcolorstripesettingbase.h"

namespace Ui {
class FJColorStripeSettingsClassfication;
}

class FJColorStripeSettingsClassfication : public FJColorStripeSettingBase
{
    Q_OBJECT

public:
    explicit FJColorStripeSettingsClassfication(QWidget *parent = nullptr);
    ~FJColorStripeSettingsClassfication();

	virtual void init(ccHObject * ent, ccGLWindow * Win);
public slots:
	void refreashClassficationShow();
private:
    Ui::FJColorStripeSettingsClassfication *ui;
	std::vector<ClassficationData> m_data;
};

#endif // FJCOLORSTRIPESETTINGSCLASSFICATION_H
