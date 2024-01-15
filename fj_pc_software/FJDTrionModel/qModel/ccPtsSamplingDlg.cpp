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

#include "ccPtsSamplingDlg.h"
#include <QButtonGroup>
ccPtsSamplingDlg::ccPtsSamplingDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, Ui::PointsSamplingDialog()
{
	setupUi(this->getContentHolder());
	QButtonGroup *bg = new QButtonGroup(this);
	bg->addButton(dRadioButton);
	bg->addButton(pnRadioButton);
    connect(pnRadioButton, &QRadioButton::toggled, this, &ccPtsSamplingDlg::slotRadioButtonClicked);
}

void ccPtsSamplingDlg::slotRadioButtonClicked()
{
    if (pnRadioButton->isChecked())
    {
        emit signalSendStatusBarInfo(QApplication::translate("importantinformationtips", "The smaller the sampling percentage, the fewer retained meshes.", nullptr));
        label->setText(tr("Sampling percentage"));
        pnSpinBox->setSuffix("%");
        pnSpinBox->setDecimals(2);
        pnSpinBox->setRange(1, 100.000000);
        pnSpinBox->setSingleStep(1.0);
        pnSpinBox->setValue(80.0);
        update();
    }
    else
    {
        emit signalSendStatusBarInfo(QApplication::translate("importantinformationtips", "The smaller the grid separation rate, the less grid retention.", nullptr));
        label->setText(tr("Grid resolution"));
        pnSpinBox->setSuffix("");
        pnSpinBox->setDecimals(6);
        pnSpinBox->setRange(0.01, 1.000000);
        pnSpinBox->setSingleStep(0.01);
        pnSpinBox->setValue(0.1);
        update();
    }
}

bool ccPtsSamplingDlg::useDensity() const
{
	return dRadioButton->isChecked();
}


double ccPtsSamplingDlg::getDensityValue() const
{
	return pnSpinBox->value();
}


double ccPtsSamplingDlg::getPointsNumber() const
{
	return pnSpinBox->value() / 100.0;
}

