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

#include "ccColorGradientDlg.h"

//common
#include <ccQtHelpers.h>
#include <framelessdialog.h>
//Qt
#include <QColorDialog>
#include <QButtonGroup>
//system
#include <assert.h>

//persistent parameters
static QColor s_firstColor(QColor(0,175,233));
static QColor s_secondColor(QColor(245, 213, 0));
static ccColorGradientDlg::GradientType s_lastType(ccColorGradientDlg::Default);
static double s_lastFreq = 5.0;
static int s_lastDimIndex = 2;

ccColorGradientDlg::ccColorGradientDlg(QWidget* parent)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, Ui::ColorGradientDialog()
{
	setupUi(this->getContentHolder());
	QButtonGroup *bg = new QButtonGroup(this);
	bg->addButton(defaultRampRadioButton);
	bg->addButton(customRampRadioButton);
	bg->addButton(bandingRadioButton);
	connect(firstColorButton, &QAbstractButton::clicked, this, &ccColorGradientDlg::changeFirstColor);
	connect(secondColorButton, &QAbstractButton::clicked, this, &ccColorGradientDlg::changeSecondColor);

	connect(defaultRampRadioButton, &QRadioButton::toggled, this, &ccColorGradientDlg::onQRadioButtontoggledslot);
	connect(customRampRadioButton, &QRadioButton::toggled, this, &ccColorGradientDlg::onQRadioButtontoggledslot);
	connect(bandingRadioButton, &QRadioButton::toggled, this, &ccColorGradientDlg::onQRadioButtontoggledslot);

	//restore previous parameters
	ccQtHelpers::SetButtonColor(secondColorButton, s_secondColor);
	ccQtHelpers::SetButtonColor(firstColorButton, s_firstColor);
	setType(s_lastType);
	bandingFreqSpinBox->setValue(s_lastFreq);
	directionComboBox->setCurrentIndex(s_lastDimIndex);
	setFixedSize(650,450);
}

void ccColorGradientDlg::onQRadioButtontoggledslot(bool checked)
{
	QString objname = qobject_cast<QRadioButton*>(sender())->objectName();
	if (objname == "defaultRampRadioButton")
	{
		firstColorButton->setDisabled(true);
		secondColorButton->setDisabled(true);
		bandingFreqSpinBox->setDisabled(true);
	}
	else if (objname == "customRampRadioButton")
	{
		firstColorButton->setEnabled(true);
		secondColorButton->setEnabled(true);
		bandingFreqSpinBox->setDisabled(true);
	}
	else
	{
		firstColorButton->setDisabled(true);
		secondColorButton->setDisabled(true);
		bandingFreqSpinBox->setEnabled(true);
	}
}

unsigned char ccColorGradientDlg::getDimension() const
{
	s_lastDimIndex = directionComboBox->currentIndex();
	return static_cast<unsigned char>(s_lastDimIndex);
}

void ccColorGradientDlg::setType(ccColorGradientDlg::GradientType type)
{
	switch(type)
	{
	case Default:
		defaultRampRadioButton->setChecked(true);
		firstColorButton->setDisabled(true);
		secondColorButton->setDisabled(true);
		bandingFreqSpinBox->setDisabled(true);
		break;
	case TwoColors:
		customRampRadioButton->setChecked(true);
		firstColorButton->setEnabled(true);
		secondColorButton->setEnabled(true);
		bandingFreqSpinBox->setDisabled(true);
		break;
	case Banding:
		bandingRadioButton->setChecked(true);
		firstColorButton->setDisabled(true);
		secondColorButton->setDisabled(true);
		bandingFreqSpinBox->setEnabled(true);
		break;
	default:
		assert(false);
	}
}

ccColorGradientDlg::GradientType ccColorGradientDlg::getType() const
{
	//ugly hack: we use 's_lastType' here as the type is only requested
	//when the dialog is accepted
	if (customRampRadioButton->isChecked())
		s_lastType = TwoColors;
	else if (bandingRadioButton->isChecked())
		s_lastType = Banding;
	else
		s_lastType = Default;

	return s_lastType;
}

void ccColorGradientDlg::getColors(QColor& first, QColor& second) const
{
	assert(customRampRadioButton->isChecked());
	first = s_firstColor;
	second = s_secondColor;
}

double ccColorGradientDlg::getBandingFrequency() const
{
	//ugly hack: we use 's_lastFreq' here as the frequency is only requested
	//when the dialog is accepted
	s_lastFreq = bandingFreqSpinBox->value();
	return s_lastFreq;
}

void ccColorGradientDlg::changeFirstColor()
{
	CS::Widgets::FramelessDialog outdlg(this);
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
	QColorDialog colordlg(s_firstColor, this);
	connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
	colordlg.setOptions(QColorDialog::NoButtons);
	outdlg.SetContentHolder(&colordlg);
	if (!outdlg.exec())
	{
		return;
	}
	QColor newCol = colordlg.currentColor();
	if (newCol.isValid())
	{
		s_firstColor = newCol;
		ccQtHelpers::SetButtonColor(firstColorButton, s_firstColor);
	}
}

void ccColorGradientDlg::changeSecondColor()
{
	CS::Widgets::FramelessDialog outdlg(this);
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
	QColorDialog colordlg(s_secondColor, this);
	connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
	colordlg.setOptions(QColorDialog::NoButtons);
	outdlg.SetContentHolder(&colordlg);
	if (!outdlg.exec())
	{
		return;
	}
	QColor newCol = colordlg.currentColor();
	if (newCol.isValid())
	{
		s_secondColor = newCol;
		ccQtHelpers::SetButtonColor(secondColorButton, s_secondColor);
	}
}
