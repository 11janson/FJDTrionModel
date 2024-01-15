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

#include "ccPreferencesettingsDlg.h"
#include "ui_PreferencesettingsDlg.h"
#include <QColorDialog>
#include <ccQtHelpers.h>
#include <ccGuiParameters.h>
#include "model/modelchangedlocker.h"
#include "cloudcompareutils/publicutils.h"
#include "model/projectmodel.h"
#include "mainwindow.h"

using namespace Utils;
ccPreferencesettingsDlg::ccPreferencesettingsDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::PreferencesettingsDlg)
{
	m_ui->setupUi(this->getContentHolder());
	initSetting();
	setApplyButtonVisible(true);
	connect(GetApplyButton(), &QPushButton::clicked, this, &ccPreferencesettingsDlg::initParamSetting);
	GetApplyButton()->setText(tr("Reset"));
    setFixedSize(746, 573);
}

ccPreferencesettingsDlg::~ccPreferencesettingsDlg()
{
	delete m_ui;
}

void ccPreferencesettingsDlg::initParamSetting(bool checked)
{
	QColor LabelColor = QColor(255, 200, 4, 1);
	QColor MarkerColor = QColor(255, 71,70,255);
	SetLabelColor(LabelColor);
	SetMarkerColor(MarkerColor);
	Settransparency(25);
	SetLabelfontsize(16);
	SetMarkersize(8);
	SetPrompttextsize(16);
	Setdrawroundpoints(false);
	SetDisplaycrossmark(true);
	Setlength(0);
	Setdiameter(0);
	Setangle(0);
	Setarea(0);
	Setvolume(0);
	Setdecimal(3);
	SetShowunitlabels(false);
	setRenderBrightnessValue(10);
	settRenderContrastValue(25);
	setSettingPointSize(1);
	update();
}

void ccPreferencesettingsDlg::initSetting()
{
	m_ui->stackedWidget->setCurrentIndex(0);
	connect(m_ui->pushButton_unit, &QPushButton::clicked, this, &ccPreferencesettingsDlg::shiftWindow);
	connect(m_ui->pushButton_other, &QPushButton::clicked, this, &ccPreferencesettingsDlg::shiftWindow);
	connect(m_ui->pushButton_classfication, &QPushButton::clicked, this, &ccPreferencesettingsDlg::shiftWindow);
	connect(m_ui->toolButton_add, &QToolButton::clicked, this, &ccPreferencesettingsDlg::addClassfication);

	connect(m_ui->toolButton_labelcolor, &QAbstractButton::clicked, this, &ccPreferencesettingsDlg::changeFirstColor);
	connect(m_ui->toolButton_markercolor, &QAbstractButton::clicked, this, &ccPreferencesettingsDlg::changeSecondColor);
	//QStringList lengthlist = (QStringList() << tr("米") << tr("分米") << tr("厘米") << tr("毫米") << tr("公里") << tr("英里") << tr("英尺") << tr("英寸"));
	QStringList lengthlist = (QStringList() << u8"m" << u8"dm" << "cm" << "mm" << "km" << "mile" << "ft" << "in");
	m_ui->comboBox_length->addItems(lengthlist);
	//QStringList diameterlist = (QStringList() << tr("米") << tr("分米") << tr("厘米") << tr("毫米") << tr("公里") << tr("英里") << tr("英尺") << tr("英寸"));
	QStringList diameterlist = (QStringList() << u8"m" << u8"dm" << "cm" << "mm" << "km" << "mile" << "ft" << "in");
	m_ui->comboBox_diameter->addItems(diameterlist);
	//QStringList anglelist = (QStringList() << tr("度") << tr("分") << tr("秒"));

	QStringList anglelist = (QStringList() << u8"°" << u8"′" << u8"″");
	m_ui->comboBox_angle->addItems(anglelist);
	//QStringList arealist = (QStringList() << tr("平方米") << tr("平方分米") << tr("平方厘米") << tr("平方毫米") << tr("平方英尺") << tr("平方英寸"));
	QStringList arealist = (QStringList() << u8"m²" << u8"dm²" << u8"cm²" << u8"mm²" << u8"ft²" << u8"in²");
	m_ui->comboBox_area->addItems(arealist);
	//QStringList volumelist = (QStringList() << tr("立方米") << tr("立方分米") << tr("立方厘米") << tr("立方毫米") << tr("立方英尺") << tr("立方英寸"));
	QStringList volumelist = (QStringList() << u8"m³" << u8"dm³" << u8"cm³" << u8"mm³" << u8"ft³" << u8"in³");
	m_ui->comboBox_volume->addItems(volumelist);
	QStringList decimallist = (QStringList() <<"0" <<"1" << "2" << "3" << "4" << "5" << "6" << "7" << "8");
	m_ui->comboBox_decimal->addItems(decimallist);
	m_selectedstyle = "background-color:#585858;color: #ffffff;border-left:3px solid #FFC804;border-right:0px solid #FFC804;border-top:0px solid #FFC804;border-bottom:0px solid #FFC804;font-size:14px;border-radius: 0px;text-align:left;padding-left:21px;";
	m_disselectedstyle = "background-color:transparent;color: #ffffff;border:0px solid #838383;font-size:14px;border-radius: 0px;text-align:left;padding-left:24px;";
	m_ui->pushButton_unit->setStyleSheet(m_selectedstyle);
	m_ui->pushButton_other->setStyleSheet(m_disselectedstyle);
	m_ui->pushButton_classfication->setStyleSheet(m_disselectedstyle);
	m_ui->toolButton_add->setIconPixmap("addpointdatanormal", "addpointdataclicked", "addpointdatadisabled");



	//[!].对比度 亮度, 默认值
	const ccGui::ParamStruct m_parameters = ccGui::Parameters();
	m_ui->horizontalSliderbrightness->setValue(m_parameters.m_nBrightness);
	m_ui->horizontalSliderContrast->setValue(m_parameters.m_nContrast);
	m_ui->spinBoxbrightness->setValue(m_parameters.m_nBrightness);
	m_ui->spinBoxContrast->setValue(m_parameters.m_nContrast);

	connect(m_ui->horizontalSliderbrightness, &QSlider::valueChanged, m_ui->spinBoxbrightness, [&](int) {
		int nValue = m_ui->horizontalSliderbrightness->value();
		m_ui->spinBoxbrightness->setValue(nValue);
	});
	connect(m_ui->horizontalSliderContrast, &QSlider::valueChanged, m_ui->spinBoxContrast, [&](int){
		int nValue = m_ui->horizontalSliderContrast->value();
		m_ui->spinBoxContrast->setValue(nValue);
	});
	connect(m_ui->spinBoxbrightness, qOverload<int>(&QSpinBox::valueChanged), this, &ccPreferencesettingsDlg::slotRenderBrightnessContrastChanged);
	connect(m_ui->spinBoxContrast, qOverload<int>(&QSpinBox::valueChanged), this, &ccPreferencesettingsDlg::slotRenderBrightnessContrastChanged);
}

void ccPreferencesettingsDlg::addClassfication()
{
}

void ccPreferencesettingsDlg::slotRenderBrightnessContrastChanged(void)
{
	//[!].线锁住控件
	ModelChangedLocker<QSlider> locker1(m_ui->horizontalSliderbrightness);
	ModelChangedLocker<QSlider> locker2(m_ui->horizontalSliderContrast);

	//[!].获取当前值
	int nBrightness = m_ui->spinBoxbrightness->value();
	int nContrast = m_ui->spinBoxContrast->value();
	m_ui->horizontalSliderbrightness->setValue(nBrightness);
	m_ui->horizontalSliderContrast->setValue(nContrast);
	updateRgbContrastBrightness(nBrightness, nContrast);
}

void ccPreferencesettingsDlg::shiftWindow(bool checked)
{
	QString objname = dynamic_cast<QPushButton*>(sender())->objectName();
	if (objname == "pushButton_unit")
	{
		m_ui->stackedWidget->setCurrentIndex(0);
		m_ui->pushButton_unit->setStyleSheet(m_selectedstyle);
		m_ui->pushButton_other->setStyleSheet(m_disselectedstyle);
		m_ui->pushButton_classfication->setStyleSheet(m_disselectedstyle);
	}
	else if(objname == "pushButton_other")
	{
		m_ui->stackedWidget->setCurrentIndex(2);
		m_ui->pushButton_unit->setStyleSheet(m_disselectedstyle);
		m_ui->pushButton_other->setStyleSheet(m_selectedstyle);
		m_ui->pushButton_classfication->setStyleSheet(m_disselectedstyle);
	}
	else
	{
		m_ui->stackedWidget->setCurrentIndex(3);
		m_ui->pushButton_unit->setStyleSheet(m_disselectedstyle);
		m_ui->pushButton_other->setStyleSheet(m_disselectedstyle);
		m_ui->pushButton_classfication->setStyleSheet(m_selectedstyle);
	}
	update();
}

void ccPreferencesettingsDlg::changeFirstColor(bool checked)
{
	//QColor newCol = QColorDialog::getColor(m_LabelColor, this);

	CS::Widgets::FramelessDialog outdlg(this);
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
	QColorDialog colordlg(m_LabelColor, this);
	connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
	colordlg.setOptions(QColorDialog::NoButtons);
	outdlg.SetContentHolder(&colordlg);
	if (!outdlg.exec())
	{
		return ;
	}
	QColor newCol = colordlg.currentColor();






	if (newCol.isValid())
	{
		m_LabelColor = newCol;
		ccQtHelpers::SetButtonColor(m_ui->toolButton_labelcolor, m_LabelColor);
	}
}

void ccPreferencesettingsDlg::changeSecondColor(bool checked)
{
	//QColor newCol = QColorDialog::getColor(m_MarkerColor, this);

	CS::Widgets::FramelessDialog outdlg(this);
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr));
	QColorDialog colordlg(m_MarkerColor, this);
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
		m_MarkerColor = newCol;
		ccQtHelpers::SetButtonColor(m_ui->toolButton_markercolor, m_MarkerColor);
	}
}


int ccPreferencesettingsDlg::Getlength()
{
	return m_ui->comboBox_length->currentIndex();
}

void ccPreferencesettingsDlg::Setlength(int num)
{
	m_ui->comboBox_length->setCurrentIndex(num);
}

int ccPreferencesettingsDlg::Getdiameter()
{
	return m_ui->comboBox_diameter->currentIndex();
}

void ccPreferencesettingsDlg::Setdiameter(int num)
{
	m_ui->comboBox_diameter->setCurrentIndex(num);
}


int ccPreferencesettingsDlg::Getangle()
{
	return m_ui->comboBox_angle->currentIndex();
}

void ccPreferencesettingsDlg::Setangle(int num)
{
	m_ui->comboBox_angle->setCurrentIndex(num);
}


int ccPreferencesettingsDlg::Getarea()
{
	return m_ui->comboBox_area->currentIndex();
}

void ccPreferencesettingsDlg::Setarea(int num)
{
	m_ui->comboBox_area->setCurrentIndex(num);
}


int ccPreferencesettingsDlg::Getvolume()
{
	return m_ui->comboBox_volume->currentIndex();
}

void ccPreferencesettingsDlg::Setvolume(int num)
{
	m_ui->comboBox_volume->setCurrentIndex(num);
}


int ccPreferencesettingsDlg::Getdecimal()
{
	return m_ui->comboBox_decimal->currentIndex();
}

void ccPreferencesettingsDlg::Setdecimal(int num)
{
	m_ui->comboBox_decimal->setCurrentIndex(num);
}


bool ccPreferencesettingsDlg::GetShowunitlabels()
{
	return m_ui->checkBox->isChecked();
}

void ccPreferencesettingsDlg::SetShowunitlabels(bool ischecked)
{
	m_ui->checkBox->setChecked(ischecked);
}


int ccPreferencesettingsDlg::Gettransparency()
{
	return m_ui->spinBox->value();
}

void ccPreferencesettingsDlg::Settransparency(int num)
{
	m_ui->spinBox->setValue(num);
}


int ccPreferencesettingsDlg::GetLabelfontsize()
{
	return m_ui->spinBox_2->value();
}

void ccPreferencesettingsDlg::SetLabelfontsize(int num)
{
	m_ui->spinBox_2->setValue(num);
}


int ccPreferencesettingsDlg::GetMarkersize()
{
	return m_ui->spinBox_3->value();
}

void ccPreferencesettingsDlg::SetMarkersize(int num)
{
	m_ui->spinBox_3->setValue(num);
}


QColor ccPreferencesettingsDlg::GetLabelColor()
{
	return m_LabelColor;
}

void ccPreferencesettingsDlg::SetLabelColor(QColor color)
{
	m_LabelColor = color;
	ccQtHelpers::SetButtonColor(m_ui->toolButton_labelcolor, m_LabelColor);
}


QColor ccPreferencesettingsDlg::GetMarkerColor()
{
	return m_MarkerColor;
}

void ccPreferencesettingsDlg::SetMarkerColor(QColor color)
{
	m_MarkerColor = color;
	ccQtHelpers::SetButtonColor(m_ui->toolButton_markercolor, m_MarkerColor);
}


int ccPreferencesettingsDlg::GetPrompttextsize()
{
	return m_ui->spinBox_4->value();
}

void ccPreferencesettingsDlg::SetPrompttextsize(int num)
{
	m_ui->spinBox_4->setValue(num);
}


bool ccPreferencesettingsDlg::Getdrawroundpoints()
{
	if (m_ui->radioButton->isChecked())
	{
		return false;
	}
	else
	{
		return true;
	}

}

void ccPreferencesettingsDlg::Setdrawroundpoints(bool isdrawround)
{
	if (isdrawround)
	{
		m_ui->radioButton_2->setChecked(true);
	}
	else
	{
		m_ui->radioButton->setChecked(true);
	}

}



bool ccPreferencesettingsDlg::GetDisplaycrossmark()
{
	return m_ui->checkBox_2->isChecked();
}

void ccPreferencesettingsDlg::SetDisplaycrossmark(bool ischecked)
{
	m_ui->checkBox_2->setChecked(ischecked);
}

int ccPreferencesettingsDlg::getSettingPointSize()
{
    return m_ui->pointSizeComboBox->currentIndex();
}

bool ccPreferencesettingsDlg::setSettingPointSize(int index)
{
    if (index >= 0 && index < 17) {
        m_ui->pointSizeComboBox->setCurrentIndex(index);
        return true;
    }
    return false;
}

void ccPreferencesettingsDlg::setRenderBrightnessValue(int nValue)
{
	int nBright = m_ui->spinBoxbrightness->value();
	if (nBright == nValue){
		return;
	}
	m_ui->spinBoxbrightness->setValue(nValue);
}

int ccPreferencesettingsDlg::getRenderBrightnessValue(void)
{
	return m_ui->spinBoxbrightness->value();
}

void ccPreferencesettingsDlg::settRenderContrastValue(int nValue)
{
	int nContrast = m_ui->horizontalSliderContrast->value();
	if (nContrast == nValue){
		return;
	}
	m_ui->horizontalSliderContrast->setValue(nValue);
}

int ccPreferencesettingsDlg::getRenderContrastValue(void)
{
	return m_ui->horizontalSliderContrast->value();
}

void ccPreferencesettingsDlg::resetContrastBrightness(const int &iBrightness, const int &iContrast)
{
	//[!].获取当前值
	int nBrightness = m_ui->spinBoxbrightness->value();
	int nContrast = m_ui->spinBoxContrast->value();

	if (iBrightness == nBrightness && iContrast == nContrast){
		return;
	}

	//[!].线锁住控件
	ModelChangedLocker<QSlider> locker1(m_ui->horizontalSliderbrightness);
	ModelChangedLocker<QSlider> locker2(m_ui->horizontalSliderContrast);
	ModelChangedLocker<QSpinBox> locker3(m_ui->spinBoxbrightness);
	ModelChangedLocker<QSpinBox> locker4(m_ui->spinBoxContrast);
	m_ui->horizontalSliderbrightness->setValue(nBrightness);
	m_ui->horizontalSliderContrast->setValue(nContrast);
	m_ui->spinBoxbrightness->setValue(iBrightness);
	m_ui->spinBoxContrast->setValue(nContrast);
	updateRgbContrastBrightness(iBrightness, iContrast);
}

void ccPreferencesettingsDlg::updateRgbContrastBrightness(int iBrightness, int iContrast)
{
	PublicUtils::PublicUtils::updateRGBColorBrightContrastValue(iBrightness, iContrast);
	//QTimer::singleShot(50, this, [=]() {
		MainWindow::TheInstance()->slotRedrawAllWindows();
		QCoreApplication::processEvents();
	//});
}
