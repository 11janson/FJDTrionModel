#include "ui_fjcolorstripesettingsmissingcoloring.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include "mainwindow.h"
#include "fjdoubleslider.h"
#include "fjcolorstripesettingsmissingcoloring.h"
FJColorStripeSettingsMissingColoring::FJColorStripeSettingsMissingColoring(QWidget *parent) :
	FJColorStripeSettingBase(parent),
    ui(new Ui::FJColorStripeSettingsMissingColoring)
{
    ui->setupUi(this->getContentHolder());
	setWindowTitle(QCoreApplication::translate("MainWindow", "Scalar Settings", nullptr));
	GetCancelButton()->setVisible(false);
	GetApplyButton()->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
	GetApplyButton()->setVisible(true);
    QButtonGroup * radioButtonGroup = new QButtonGroup(this);
    radioButtonGroup->addButton(ui->radioButton);
    radioButtonGroup->addButton(ui->radioButton_2);
    radioButtonGroup->setExclusive(true);
	connect(GetApplyButton(), &QAbstractButton::clicked, this, &FJColorStripeSettingsMissingColoring::resetStyle);
}

FJColorStripeSettingsMissingColoring::~FJColorStripeSettingsMissingColoring()
{
    delete ui;
}

void FJColorStripeSettingsMissingColoring::init(ccHObject * ent, ccGLWindow * Win)
{
	if (m_object)
	{
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
        if (cloud && cloud->colorsShown() && cloud->getScalarFieldIndexByName("HasColor") >= 0)
        {
            int sfindex = cloud->getScalarFieldIndexByName("HasColor");
            CCCoreLib::ScalarField* classifSF = cloud->getScalarField(sfindex);
            int pointsize = classifSF->size();
            for (int i = 0; i < pointsize; i++)
            {
                if (classifSF->getValue(i) == 0)
                {
                    ccColor::Rgba curpointcolor = cloud->getPointColor(i);
                    m_isShowHidePoint = (curpointcolor.a > 0);
                    break;
                }
            }

        }
        ui->radioButton->setChecked(!m_isShowHidePoint);
        connect(ui->radioButton, &QRadioButton::toggled, this, &FJColorStripeSettingsMissingColoring::refreashShow);
	}
}

void FJColorStripeSettingsMissingColoring::refreashShow()
{
    int alpha = 0;
    if (!ui->radioButton->isChecked())
    {
        alpha = 255;
    }
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
    if (cloud && cloud->colorsShown() && cloud->getScalarFieldIndexByName("HasColor") >= 0)
    {
        int sfindex = cloud->getScalarFieldIndexByName("HasColor");
        CCCoreLib::ScalarField* classifSF = cloud->getScalarField(sfindex);
        int pointsize = classifSF->size();
        for (int i = 0; i < pointsize; i++)
        {
            if (classifSF->getValue(i) == 0)
            {
                ccColor::Rgba curpointcolor = cloud->getPointColor(i);
                curpointcolor.a = alpha;
                cloud->setPointColor(i, curpointcolor);
            }
        }
        cloud->scaleFieldColorChange();
        if (MainWindow::TheInstance()->GetActiveGLWindow())
        {
            MainWindow::TheInstance()->GetActiveGLWindow()->redraw();
        }
    }
}

void FJColorStripeSettingsMissingColoring::resetStyle()
{
    ui->radioButton->blockSignals(true);
    ui->radioButton_2->blockSignals(true);
    ui->radioButton->setChecked(!m_isShowHidePoint);  
    ui->radioButton_2->setChecked(m_isShowHidePoint);
    ui->radioButton->blockSignals(false);
    ui->radioButton_2->blockSignals(false);
    refreashShow();
}