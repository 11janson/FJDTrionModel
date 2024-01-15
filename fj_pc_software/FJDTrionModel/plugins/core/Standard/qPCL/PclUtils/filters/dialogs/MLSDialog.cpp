//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "MLSDialog.h"


//Qt
#include <QVariant>
#include <QButtonGroup>
#include <QRadioButton>
#include <QSettings>
#include "cswidgets/framelessfiledialog.h"
#include "qstandardpaths.h"
#include "cloudcompareutils/icore.h"
#include "cswidgets/framelessmessagebox.h"
#include <QTimer>
MLSDialog::MLSDialog(QWidget* parent)
	: CS::Widgets::FramelessDialog(parent)
	, Ui::MLSDialog()
{
	setupUi(this->getContentHolder());
	QButtonGroup *bg = new QButtonGroup(this);
	updateCombo();
	bg->addButton(radioButtonnone);
	bg->addButton(radioButtonSample);
	bg->addButton(radioButtonRandom);
	bg->addButton(radioButtonVoxel);
	deactivateAllMethods();
	connect(radioButtonnone, SIGNAL(toggled(bool)), this, SLOT(activateMenu(bool)));
	connect(radioButtonSample, SIGNAL(toggled(bool)), this, SLOT(activateMenu(bool)));
	connect(radioButtonRandom, SIGNAL(toggled(bool)), this, SLOT(activateMenu(bool)));
	connect(radioButtonVoxel, SIGNAL(toggled(bool)), this, SLOT(activateMenu(bool)));
    connect(checkBox, &QCheckBox::clicked, this, &MLSDialog::slotCheckBox);
    connect(pushButton, &QPushButton::clicked, this, &MLSDialog::slotPushButtonBrowes);
    QPushButton *pOkButton = new QPushButton(this);
    this->setOkButton(pOkButton);
    connect(pOkButton, &QPushButton::clicked, this, &MLSDialog::slotOkButtonClicked, Qt::UniqueConnection);
    lineEdit->setReadOnly(true);
    pushButton->setEnabled(false);
    lineEdit->setEnabled(false);
}

void MLSDialog::updateCombo()
{
	//upsampling_method->clear();
	//upsampling_method->addItem(tr("None"), QVariant(MLSParameters::NONE));
	//upsampling_method->addItem(tr("Sample Local Plane"), QVariant(MLSParameters::SAMPLE_LOCAL_PLANE));
	//upsampling_method->addItem(tr("Random Uniform Density"), QVariant(MLSParameters::RANDOM_UNIFORM_DENSITY));
	//upsampling_method->addItem(tr("Voxel Grid Dilation"), QVariant(MLSParameters::VOXEL_GRID_DILATION));
}

void MLSDialog::activateMenu(bool ischecked)
{
	QString name = dynamic_cast<QRadioButton*>(QObject::sender())->objectName();
	deactivateAllMethods();
	if (name == "radioButtonSample")
	{
		upsampling_radius->setEnabled(true);
		upsampling_step_size->setEnabled(true);
		m_method = MLSParameters::SAMPLE_LOCAL_PLANE;
	}
	else if (name == "radioButtonRandom")
	{
		step_point_density->setEnabled(true);
		m_method = MLSParameters::RANDOM_UNIFORM_DENSITY;
	}
	else if (name == "radioButtonVoxel")
	{
		dilation_voxel_size->setEnabled(true);
		dilation_iterations->setEnabled(true);
		m_method = MLSParameters::VOXEL_GRID_DILATION;
	}
	else
	{
		deactivateAllMethods();
		m_method = MLSParameters::NONE;
	}
	//upsampling_radius->setEnabled(true);
	//upsampling_step_size->setEnabled(true);

	//step_point_density->setEnabled(true);

	//dilation_voxel_size->setEnabled(true);
	//dilation_iterations->setEnabled(true);
}


void MLSDialog::deactivateAllMethods()
{
	upsampling_radius->setEnabled(false);
	upsampling_step_size->setEnabled(false);

	step_point_density->setEnabled(false);

	dilation_voxel_size->setEnabled(false);
	dilation_iterations->setEnabled(false);
}


MLSParameters::UpsamplingMethod MLSDialog::GetCurrentMethod()
{
	return m_method;
}

void MLSDialog::slotCheckBox()
{
    if (checkBox->isChecked()) {
        lineEdit->setEnabled(true);
        pushButton->setEnabled(true);
        if (!m_fileSavePath.isEmpty()) {
            lineEdit->setText(m_fileSavePath);
        }
    }
    else
    {
        lineEdit->setEnabled(false);
        pushButton->setEnabled(false);
        lineEdit->clear();
    }
}

void MLSDialog::slotPushButtonBrowes()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("SORFilterDlgPath");
    QString currentPath = settings.value("SORFilterDlgPath", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    if (!lineEdit->text().isEmpty()) {
        currentPath = lineEdit->text();
    }
    QString SaveName = CS::Widgets::FramelessFileDialog::getExistingDirectory(this, tr("Save file"), currentPath, QFileDialog::ShowDirsOnly);
    if (SaveName.isEmpty()) {
        settings.group();
        return;
    }
    settings.setValue("SORFilterDlgPath", SaveName);
    settings.endGroup();
    lineEdit->setText(SaveName);
    m_fileSavePath = SaveName;
}

void MLSDialog::slotOkButtonClicked()
{
    if (!checkBox->isChecked()) {
        accept();
        return;
    }
    QDir dir(lineEdit->text());
    if (lineEdit->isEnabled() && lineEdit->text().isEmpty())
    {
        lineEdit->setStyleSheet("QLineEdit{padding-left:12px;}QLineEdit{border-radius: 2px;border: 1px solid #FF4746;} ");
        QTimer::singleShot(1500, this, [=]() {
            if (lineEdit) {
                lineEdit->setStyleSheet("QLineEdit{padding-left:12px;border: 1px solid #989898;border - radius: 2px;}");
            }
        });
        return;
    }
    QString fileStr = dir.absoluteFilePath("Modeltemplkjh.txt");
    QFile file(fileStr);
    bool bol = false;
    if (file.open(QIODevice::ReadWrite)) {
        bol = true;
        file.close();
    }
    if (!bol || !QFile::remove(fileStr)) {
        CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Selecting a path cannot save data,Please reselect the path."));
        return;
    }

    accept();
}

void MLSDialog::setFileSavePath(QString path)
{
    m_fileSavePath = path;
}

QString MLSDialog::getSaveFilePath()
{
    if (checkBox->isChecked()) {
        return m_fileSavePath;
    }
    else {
        return QString();
    }
}