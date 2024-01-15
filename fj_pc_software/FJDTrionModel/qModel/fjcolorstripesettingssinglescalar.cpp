#include "fjcolorstripesettingssinglescalar.h"
#include "ui_fjcolorstripesettingssinglescalar.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "fjdoubleslider.h"
#include "ccScalarField.h"
#include <QDoubleSpinBox>
#include <QPushButton>
#include "ccColorScalesManager.h"
#include "mainwindow.h"
FJColorStripeSettingsSingleScalar::FJColorStripeSettingsSingleScalar(QWidget *parent) :
    FJColorStripeSettingBase(parent),
    ui(new Ui::FJColorStripeSettingsSingleScalar)
{
    ui->setupUi(this->getContentHolder());
    setWindowTitle(QCoreApplication::translate("MainWindow", "Scalar Settings", nullptr));
    GetCancelButton()->setVisible(false);
    GetApplyButton()->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
    GetApplyButton()->setVisible(true);
    connect(GetApplyButton(), &QAbstractButton::clicked, [=]() {
        ui->horizontalSlider->setSpan(m_startNum, m_stopNum);
        ui->comboBox->setCurrentIndex(m_scaleIndex);
        colorScaleSelected(m_scaleIndex);
        refreashShow(0);
    });
    connect(ui->spinBox_min, qOverload<double>(&QDoubleSpinBox::valueChanged), [=](double value) {
        ui->horizontalSlider->setLowerValue(value);
    });
    connect(ui->spinBox_max, qOverload<double>(&QDoubleSpinBox::valueChanged), [=](double value) {
        ui->horizontalSlider->setUpperValue(value);
    });
    connect(ui->horizontalSlider, qOverload<double>(&FJDoubleSlider::lowerValueChanged), this, &FJColorStripeSettingsSingleScalar::refreashShow);
    connect(ui->horizontalSlider, qOverload<double>(&FJDoubleSlider::upperValueChanged), this, &FJColorStripeSettingsSingleScalar::refreashShow);
    m_slider.reset(new ColorScaleElementSliders());
    m_dlg = new ColorBarWidget(m_slider, this);
    QHBoxLayout * layout = new QHBoxLayout;
    layout->setMargin(0);
    layout->setSpacing(0);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_dlg);
    ui->widget_bar->setLayout(layout);
}

FJColorStripeSettingsSingleScalar::~FJColorStripeSettingsSingleScalar()
{
    delete ui;
}

void FJColorStripeSettingsSingleScalar::init(ccHObject * ent, ccGLWindow * Win)
{
    QMap<QString, QString> scales;
    for (ccColorScalesManager::ScalesMap::const_iterator it = ccColorScalesManager::GetUniqueInstance()->map().constBegin(); it != ccColorScalesManager::GetUniqueInstance()->map().constEnd(); ++it)
    {
        scales.insert((*it)->getName(), (*it)->getUuid());
    }
    if (m_object)
    {
        ui->label->setText(tr("Displays scalar"));
        ui->label_range->setText(tr("Saturation setting"));
        //if (ent->getCurrentCombinationMode() == ALTITUDE)
        //{
        //	ui->label_range->setText(tr("Elevation range"));
        //}
        //else if (ent->getCurrentCombinationMode() == GPSTIME)
        //{
        //	ui->label_range->setText(tr("Time range"));
        //}
        //else if (ent->getCurrentCombinationMode() == USERDATA)
        //{
        //	ui->label_range->setText(tr("User data range"));
        //	ui->frame->setVisible(false);
        //}
        //else if (ent->getCurrentCombinationMode() == INTENSITY)
        //{
        //	ui->label_range->setText(tr("Intensity range"));
        //}
        //else if (ent->getCurrentCombinationMode() == PCVMODE)
        //{
        //	ui->label_range->setText(tr("PCV range"));
        //}
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
        if (cloud)
        {
            ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
            if (sf)
            {
                m_startNum = (sf->saturationRange()).start();
                m_stopNum = (sf->saturationRange()).stop();
                ui->spinBox_min->blockSignals(true);
                ui->spinBox_max->blockSignals(true);
                ui->horizontalSlider->blockSignals(true);
                m_sfMin = (sf->saturationRange()).min();
                m_sfMax = (sf->saturationRange()).max();
                ui->spinBox_min->setRange((sf->saturationRange()).min(), (sf->saturationRange()).max());
                ui->spinBox_max->setRange((sf->saturationRange()).min(), (sf->saturationRange()).max());
                ui->spinBox_min->setValue(m_startNum);
                ui->spinBox_max->setValue(m_stopNum);
                ui->horizontalSlider->setQxtSpanRange((sf->saturationRange()).min(), (sf->saturationRange()).max());
                ui->horizontalSlider->setLowerValue(m_startNum);
                ui->horizontalSlider->setUpperValue(m_stopNum);
                ui->spinBox_min->blockSignals(false);
                ui->spinBox_max->blockSignals(false);
                ui->horizontalSlider->blockSignals(false);

                int scaleNum = 0;
                for (QMap<QString, QString>::const_iterator scale = scales.constBegin(); scale != scales.constEnd(); ++scale)
                {
                    ui->comboBox->addItem(scale.key(), scale.value());
                    QSharedPointer<ccColorScale> colorScale = ccColorScalesManager::GetUniqueInstance()->getScale(scale.value());
                    if (sf->getColorScale() == colorScale)
                    {
                        m_scaleIndex = scaleNum;
                    }
                    scaleNum++;
                }
            }
        }
    }

    ui->comboBox->setCurrentIndex(m_scaleIndex);
    colorScaleSelected(m_scaleIndex);
    connect(ui->comboBox, qOverload<int>(&QComboBox::activated), this, &FJColorStripeSettingsSingleScalar::colorScaleSelected);
}

void FJColorStripeSettingsSingleScalar::refreashShow(int value)
{
    double startNum = ui->horizontalSlider->lowerValue();
    double stopNum = ui->horizontalSlider->upperValue();
    ui->spinBox_min->blockSignals(true);
    ui->spinBox_max->blockSignals(true);
    ui->spinBox_min->setRange(m_sfMin, stopNum);
    ui->spinBox_max->setRange(startNum, m_sfMax);
    ui->spinBox_min->setValue(startNum);
    ui->spinBox_max->setValue(stopNum);
    ui->spinBox_min->blockSignals(false);
    ui->spinBox_max->blockSignals(false);
    if (m_object)
    {
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
        if (cloud)
        {
            ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
            if (sf)
            {
                sf->setSaturationStart(ui->spinBox_min->value());
                sf->setSaturationStop(ui->spinBox_max->value());
                //if (m_Win)
                //{
                //	m_Win->redraw(false);
                //}
            }
            cloud->scaleFieldColorChange();
        }
    }
    MainWindow::TheInstance()->GetActiveGLWindow()->redraw();
}

void FJColorStripeSettingsSingleScalar::colorScaleSelected(int pos)
{
    if (!m_object)
    {
        return;
    }

    if (pos < 0)
    {
        assert(false);
        return;
    }
    QString UUID = ui->comboBox->itemData(pos).toString();
    QSharedPointer<ccColorScale> colorScale = ccColorScalesManager::GetUniqueInstance()->getScale(UUID);
    if (!colorScale)
    {
        return;
    }

    //get current SF
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
    assert(cloud);
    ccScalarField* sf = cloud ? static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField()) : nullptr;
    if (sf && sf->getColorScale() != colorScale)
    {
        sf->setColorScale(colorScale);
        cloud->scaleFieldColorChange();
        if (m_Win)
        {
            m_Win->redraw(false);
        }
    }
    if (colorScale)
    {
        m_slider->clear();
        for (int i = 0; i < colorScale->stepCount(); ++i)
        {
            double relativePos = colorScale->step(i).getRelativePos();
            const QColor& color = colorScale->step(i).getColor();
            ColorScaleElementSlider* slider = new ColorScaleElementSlider(relativePos, color, this);
            slider->setVisible(false);
            m_slider->addSlider(slider);
        }
        m_dlg->update();
    }
    MainWindow::TheInstance()->GetActiveGLWindow()->redraw();
}