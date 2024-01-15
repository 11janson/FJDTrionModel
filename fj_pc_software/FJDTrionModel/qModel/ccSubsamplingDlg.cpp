//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccSubsamplingDlg.h"
#include "ui_subsamplingDlg.h"

//CCCoreLib
#include <CloudSamplingTools.h>
#include <ScalarField.h>

//qCC_db
#include <ccGenericPointCloud.h>

//Exponent of the 'log' scale used for 'SPACE' interval
static const double SPACE_RANGE_EXPONENT = 0.05;

ccSubsamplingDlg::ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_maxPointCount(maxPointCount)
	, m_maxRadius(maxCloudRadius)
	, m_sfModEnabled(false)
	, m_sfMin(0)
	, m_sfMax(0)
	, m_ui( new Ui::SubsamplingDialog )
{
	m_ui->setupUi(this->getContentHolder());
	setWindowTitle(QCoreApplication::translate("ccSubsamplingDlg", "Cloud Subsampling", nullptr));
	m_ui->samplingMethod->addItem( tr( "Density-based" ) );
	m_ui->samplingMethod->addItem( tr( "Distance-based" ) );
	m_ui->samplingMethod->addItem( tr( "Spatial structure" ) );
	connect(m_ui->samplingValue,  qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccSubsamplingDlg::samplingRateChanged);
	connect(m_ui->samplingMethod, qOverload<int>(&QComboBox::currentIndexChanged),		  this, &ccSubsamplingDlg::changeSamplingMethod);

	m_ui->samplingMethod->setCurrentIndex(1);
	sliderMoved(1.0);
	m_ui->sfGroupBox->setVisible(false);

}

ccSubsamplingDlg::~ccSubsamplingDlg()
{
	delete m_ui;
}

CCCoreLib::ReferenceCloud* ccSubsamplingDlg::getSampledCloud(ccGenericPointCloud* cloud, CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!cloud || cloud->size() == 0)
	{
		ccLog::Warning("[ccSubsamplingDlg::getSampledCloud] Invalid input cloud!");
		return nullptr;
	}

	switch (m_ui->samplingMethod->currentIndex())
	{
	case RANDOM:
		{
			assert(m_ui->samplingValue->value() >= 0);
			unsigned count = static_cast<unsigned>(m_ui->samplingValue->value() / 100.0 * double(m_maxPointCount));
			return CCCoreLib::CloudSamplingTools::subsampleCloudRandomly(	cloud,
																		count,
																		progressCb);
		}
		break;

	case SPACE:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				PointCoordinateType minDist = static_cast<PointCoordinateType>(m_ui->samplingValue->value());
				CCCoreLib::CloudSamplingTools::SFModulationParams modParams;
				modParams.enabled = m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked();
				if (modParams.enabled)
				{
					double deltaSF = static_cast<double>(m_sfMax) - m_sfMin;
					assert(deltaSF >= 0);
					if ( CCCoreLib::GreaterThanEpsilon( deltaSF ) )
					{
						double sfMinSpacing = m_ui->minSFSpacingDoubleSpinBox->value();
						double sfMaxSpacing = m_ui->maxSFSpacingDoubleSpinBox->value();
						modParams.a = (sfMaxSpacing - sfMinSpacing) / deltaSF;
						modParams.b = sfMinSpacing - modParams.a * m_sfMin;
					}
					else
					{
						modParams.a = 0.0;
						modParams.b = m_sfMin;
					}
				}
				return CCCoreLib::CloudSamplingTools::resampleCloudSpatially(	cloud, 
																			minDist,
																			modParams,
																			octree.data(),
																			progressCb);
			}
			else
			{
				ccLog::Warning(QString("[ccSubsamplingDlg::getSampledCloud] Failed to compute octree for cloud '%1'").arg(cloud->getName()));
			}
		}
		break;

	case OCTREE:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
				octree = cloud->computeOctree(progressCb);
			if (octree)
			{
				assert(m_ui->samplingValue->value() >= 0);
				unsigned char level = static_cast<unsigned char>(m_ui->samplingValue->value());
				return CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																					level,
																					CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																					progressCb,
																					octree.data());
			}
			else
			{
				ccLog::Warning(QString("[ccSubsamplingDlg::getSampledCloud] Failed to compute octree for cloud '%1'").arg(cloud->getName()));
			}
		}
		break;
	}

	//something went wrong!
	return nullptr;
}

void ccSubsamplingDlg::updateLabels()
{
	switch(m_ui->samplingMethod->currentIndex())
	{
	case RANDOM:
		m_ui->valueLabel->setText( tr( "Sampling percentage" )+"(%)" );
		break;
	case SPACE:
		m_ui->valueLabel->setText( tr( "Minimum point distance" ) );
		break;
	case OCTREE:
		m_ui->valueLabel->setText( tr( "Level of segmentation" ) );
		break;
	default:
		break;
	}
}

void ccSubsamplingDlg::sliderMoved(int sliderPos)
{
	double rate = sliderPos;
	if (m_ui->samplingMethod->currentIndex() == SPACE)
	{
		rate = pow(rate, SPACE_RANGE_EXPONENT);
		rate = 1.0 - rate;
	}

	double valueRange = static_cast<double>(m_ui->samplingValue->maximum()-m_ui->samplingValue->minimum());
	m_ui->samplingValue->setValue(m_ui->samplingValue->minimum() + rate * valueRange);
}

void ccSubsamplingDlg::samplingRateChanged(double value)
{
	double valueRange = static_cast<double>(m_ui->samplingValue->maximum()-m_ui->samplingValue->minimum());
	double rate = static_cast<double>(value-m_ui->samplingValue->minimum())/valueRange;

	CC_SUBSAMPLING_METHOD method = static_cast<CC_SUBSAMPLING_METHOD>(m_ui->samplingMethod->currentIndex());
	if (method == SPACE)
	{
		rate = 1.0 - rate;
		rate = pow(rate, 1.0/SPACE_RANGE_EXPONENT);

		if (m_sfModEnabled && !m_ui->sfGroupBox->isChecked())
		{
			m_ui->minSFSpacingDoubleSpinBox->setValue(value);
			m_ui->maxSFSpacingDoubleSpinBox->setValue(value);
		}
	}

}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
	m_ui->sfGroupBox->setEnabled(false);

	//update the labels
	m_ui->samplingValue->blockSignals(true);
	switch(index)
	{
	case RANDOM:
		{
			m_ui->samplingValue->setDecimals(2);
			m_ui->samplingValue->setMinimum(0.00);
			m_ui->samplingValue->setMaximum(100.00);
			m_ui->samplingValue->setSingleStep(1);
			m_ui->samplingValue->setEnabled(true);
            emit signalSendStatusBarInfo(QApplication::translate("importantinformationtips", "The smaller the sampling percentage, the fewer reserved points.", nullptr));
		}
		break;
	case SPACE:
		{
			m_ui->samplingValue->setDecimals(4);
			m_ui->samplingValue->setMinimum(0.0);
			m_ui->samplingValue->setMaximum(m_maxRadius);
			double step = m_maxRadius / 1000.0;
			m_ui->samplingValue->setSingleStep(step);
			m_ui->minSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->minSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->maxSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->maxSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->sfGroupBox->setEnabled(m_sfModEnabled);
			m_ui->samplingValue->setDisabled(m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked());
            emit signalSendStatusBarInfo(QApplication::translate("importantinformationtips", "The larger the minimum space,the fewer the number of points.", nullptr));
		}
		break;
	case OCTREE:
		{
			m_ui->samplingValue->setDecimals(0);
			m_ui->samplingValue->setMinimum(1);
			m_ui->samplingValue->setMaximum(static_cast<double>(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL));
			m_ui->samplingValue->setSingleStep(1);
			m_ui->samplingValue->setEnabled(true);
            emit signalSendStatusBarInfo(QApplication::translate("importantinformationtips", "The smaller the subdivision level, the fewer reserved points.", nullptr));
		}
		break;
	default:
		break;
	}
	m_ui->samplingValue->blockSignals(false);

	updateLabels();
	sliderMoved(1.0);
}

void ccSubsamplingDlg::enableSFModulation(ScalarType sfMin, ScalarType sfMax)
{
	m_sfModEnabled = CCCoreLib::ScalarField::ValidValue(sfMin) && CCCoreLib::ScalarField::ValidValue(sfMax);
	if (!m_sfModEnabled)
	{
		ccLog::Warning("[ccSubsamplingDlg::enableSFModulation] Invalid input SF values");
		return;
	}

	m_sfMin = sfMin;
	m_sfMax = sfMax;

	m_ui->sfGroupBox->setEnabled(m_ui->samplingMethod->currentIndex() == SPACE);
	m_ui->minSFlabel->setText(QString::number(sfMin));
	m_ui->maxSFlabel->setText(QString::number(sfMax));
}
