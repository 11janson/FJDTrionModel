#include "fjcolorstripesettingsdoublescalar.h"
#include "ui_fjcolorstripesettingsdoublescalar.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "fjdoubleslider.h"
#include "ccScalarField.h"
#include <QDoubleSpinBox>
#include <QPushButton>
#include "ccColorScalesManager.h"
#include <QSharedPointer>
#include "mainwindow.h"
FJColorStripeSettingsDoubleScalar::FJColorStripeSettingsDoubleScalar(QWidget *parent) :
	FJColorStripeSettingBase(parent),
    ui(new Ui::FJColorStripeSettingsDoubleScalar)
{
    ui->setupUi(this->getContentHolder());
	setWindowTitle(QCoreApplication::translate("MainWindow", "Scalar Settings", nullptr));
	GetCancelButton()->setVisible(false);
	GetApplyButton()->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
	GetApplyButton()->setVisible(true);
	connect(GetApplyButton(), &QAbstractButton::clicked, [=]() {
		ui->horizontalSlider->setSpan(m_startNumOne, m_stopNumOne);
		ui->horizontalSlider_second->setSpan(m_startNumTwo, m_stopNumTwo);
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
	connect(ui->spinBox_minsecond, qOverload<double>(&QDoubleSpinBox::valueChanged), [=](double value) {
		ui->horizontalSlider_second->setLowerValue(value);
	});
	connect(ui->spinBox_maxsecond, qOverload<double>(&QDoubleSpinBox::valueChanged), [=](double value) {
		ui->horizontalSlider_second->setUpperValue(value);
	});
	connect(ui->horizontalSlider, qOverload<double>(&FJDoubleSlider::lowerValueChanged), this, &FJColorStripeSettingsDoubleScalar::refreashShow);
	connect(ui->horizontalSlider, qOverload<double>(&FJDoubleSlider::upperValueChanged), this, &FJColorStripeSettingsDoubleScalar::refreashShow);
	connect(ui->horizontalSlider_second, qOverload<double>(&FJDoubleSlider::lowerValueChanged), this, &FJColorStripeSettingsDoubleScalar::refreashShow);
	connect(ui->horizontalSlider_second, qOverload<double>(&FJDoubleSlider::upperValueChanged), this, &FJColorStripeSettingsDoubleScalar::refreashShow);
	m_slider.reset(new ColorScaleElementSliders());
	m_dlg = new ColorBarWidget(m_slider, this);
	QHBoxLayout * layout = new QHBoxLayout;
	layout->setMargin(0);
	layout->setSpacing(0);
	layout->setContentsMargins(0,0,0,0);
	layout->addWidget(m_dlg);
	ui->widget_bar->setLayout(layout);
}

FJColorStripeSettingsDoubleScalar::~FJColorStripeSettingsDoubleScalar()
{
    delete ui;
}

void FJColorStripeSettingsDoubleScalar::init(ccHObject * ent, ccGLWindow * Win)
{
	QMap<QString, QString> scales;
	for (ccColorScalesManager::ScalesMap::const_iterator it = ccColorScalesManager::GetUniqueInstance()->map().constBegin(); it != ccColorScalesManager::GetUniqueInstance()->map().constEnd(); ++it)
	{
		scales.insert((*it)->getName(), (*it)->getUuid());
	}
	m_scaleName1 = "unknown";
	m_scaleName2 = "unknown";
	if (m_object)
	{
		ui->label->setText(tr("Displays scalar"));
        if (ent->getCurrentCombinationMode() == ALTITUDEANDINTENSITY)
		{
			m_scaleName1 = "intensity";
			m_scaleName2 = "coord. z";
			ui->label_rangeone->setText(tr("Intensity range"));
			ui->label_rangetwo->setText(tr("Elevation range"));
		}
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
		if (cloud)
		{
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare(m_scaleName1, Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf)
					{
						m_startNumOne = (sf->displayRange()).start();
						m_stopNumOne = (sf->displayRange()).stop();
						ui->spinBox_min->blockSignals(true);
						ui->spinBox_max->blockSignals(true);
						ui->horizontalSlider->blockSignals(true);
						m_sfMinOne = (sf->displayRange()).min();
						m_sfMaxOne = (sf->displayRange()).max();
						ui->spinBox_min->setRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->spinBox_max->setRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->spinBox_min->setValue(m_startNumOne);
						ui->spinBox_max->setValue(m_stopNumOne);
						ui->horizontalSlider->setQxtSpanRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->horizontalSlider->setLowerValue(m_startNumOne);
						ui->horizontalSlider->setUpperValue(m_stopNumOne);
						ui->spinBox_min->blockSignals(false);
						ui->spinBox_max->blockSignals(false);
						ui->horizontalSlider->blockSignals(false);
					}
				}
				if (sfName.compare(m_scaleName2, Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf)
					{

						m_startNumTwo = (sf->displayRange()).start();
						m_stopNumTwo = (sf->displayRange()).stop();
						ui->spinBox_minsecond->blockSignals(true);
						ui->spinBox_maxsecond->blockSignals(true);
						ui->horizontalSlider_second->blockSignals(true);
						m_sfMinTwo = (sf->displayRange()).min();
						m_sfMaxTwo = (sf->displayRange()).max();
						ui->spinBox_minsecond->setRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->spinBox_maxsecond->setRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->spinBox_minsecond->setValue(m_startNumTwo);
						ui->spinBox_maxsecond->setValue(m_stopNumTwo);
						ui->horizontalSlider_second->setQxtSpanRange((sf->displayRange()).min(), (sf->displayRange()).max());
						ui->horizontalSlider_second->setLowerValue(m_startNumTwo);
						ui->horizontalSlider_second->setUpperValue(m_stopNumTwo);
						ui->spinBox_minsecond->blockSignals(false);
						ui->spinBox_maxsecond->blockSignals(false);
						ui->horizontalSlider_second->blockSignals(false);
					}
				}
			}
			ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
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
	connect(ui->comboBox, qOverload<int>(&QComboBox::activated), this, &FJColorStripeSettingsDoubleScalar::colorScaleSelected);
}

void FJColorStripeSettingsDoubleScalar::refreashShow(int value)
{
	double startNum = ui->horizontalSlider->lowerValue();
	double stopNum = ui->horizontalSlider->upperValue();
	ui->spinBox_min->blockSignals(true);
	ui->spinBox_max->blockSignals(true);
	ui->spinBox_min->setRange(m_sfMinOne, stopNum);
	ui->spinBox_max->setRange(startNum, m_sfMaxOne);
	ui->spinBox_min->setValue(startNum);
	ui->spinBox_max->setValue(stopNum);
	ui->spinBox_min->blockSignals(false);
	ui->spinBox_max->blockSignals(false);

	double startNumsecond = ui->horizontalSlider_second->lowerValue();
	double stopNumsecond = ui->horizontalSlider_second->upperValue();
	ui->spinBox_minsecond->blockSignals(true);
	ui->spinBox_maxsecond->blockSignals(true);
	ui->spinBox_minsecond->setRange(m_sfMinTwo, stopNumsecond);
	ui->spinBox_maxsecond->setRange(startNumsecond, m_sfMaxTwo);
	ui->spinBox_minsecond->setValue(startNumsecond);
	ui->spinBox_maxsecond->setValue(stopNumsecond);
	ui->spinBox_minsecond->blockSignals(false);
	ui->spinBox_maxsecond->blockSignals(false);
	if (m_object)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
		if (cloud)
		{
			ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
				sf->setMinDisplayed(startNum + startNumsecond);
				sf->setMaxDisplayed(stopNum + stopNumsecond);
				//if (m_Win)
				//{
				//	m_Win->redraw(false);
				//}
			}
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare(m_scaleName1, Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf1 = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf1)
					{
						sf1->setMinDisplayed(ui->spinBox_min->value());
						sf1->setMaxDisplayed(ui->spinBox_max->value());
					}
				}
				if (sfName.compare(m_scaleName2, Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf2 = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf2)
					{
						sf2->setMinDisplayed(ui->spinBox_minsecond->value());
						sf2->setMaxDisplayed(ui->spinBox_maxsecond->value());
					}
				}
			}
            cloud->scaleFieldColorChange();
		}
	}
	MainWindow::TheInstance()->GetActiveGLWindow()->redraw();
}

void FJColorStripeSettingsDoubleScalar::colorScaleSelected(int pos)
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
        cloud->scaleFieldColorChange();
		sf->setColorScale(colorScale);
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

