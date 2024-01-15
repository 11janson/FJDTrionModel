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

#include "ccSORFilterDlg.h"
#include "ui_sorFilterDlg.h"
#include <QSettings>
#include "cswidgets/framelessfiledialog.h"
#include "qstandardpaths.h"
#include "cloudcompareutils/icore.h"
#include "cswidgets/framelessmessagebox.h"
#include <QTimer>
ccSORFilterDlg::ccSORFilterDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::SorFilterDialog )
{
	m_ui->setupUi(this->getContentHolder());
    m_ui->lineEdit->setReadOnly(true);
    m_ui->lineEdit->setEnabled(false);
    m_ui->pushButton->setEnabled(false);
    connect(m_ui->checkBox, &QCheckBox::clicked, this, &ccSORFilterDlg::slotQCheckBoxClicked,Qt::UniqueConnection);
    connect(m_ui->pushButton, &QPushButton::clicked, this, &ccSORFilterDlg::slotQPushbuttonClicked, Qt::UniqueConnection); 
    disconnect(this->GetOKButton(), &QAbstractButton::clicked, this, &QDialog::accept);
    QPushButton *pOkButton = new QPushButton(this);
    this->setOkButton(pOkButton);
    connect(pOkButton, &QPushButton::clicked, this, &ccSORFilterDlg::slotOkButtonClicked, Qt::UniqueConnection);
}

ccSORFilterDlg::~ccSORFilterDlg()
{
	delete m_ui;
}

int ccSORFilterDlg::KNN() const
{
	return m_ui->knnSpinBox->value();
}

void ccSORFilterDlg::setKNN(int knn)
{
	m_ui->knnSpinBox->setValue( knn );
}

void ccSORFilterDlg::setMaxKNN(int maxKNN)
{
	m_ui->knnSpinBox->setMaximum(maxKNN);
}

void ccSORFilterDlg::setMinKNN(int minKNN)
{
	m_ui->knnSpinBox->setMinimum(minKNN);
}

double ccSORFilterDlg::nSigma() const
{
	return m_ui->nSigmaDoubleSpinBox->value();
}

void ccSORFilterDlg::setNSigma(double nSigma)
{
	m_ui->nSigmaDoubleSpinBox->setValue( nSigma );
}

void ccSORFilterDlg::slotQCheckBoxClicked()
{
    if (m_ui->checkBox->isChecked()) {
        m_ui->lineEdit->setEnabled(true);
        m_ui->pushButton->setEnabled(true);
        if (!m_fileSavePath.isEmpty()){
            m_ui->lineEdit->setText(m_fileSavePath);
        }
    }
    else
    {
        m_ui->lineEdit->setEnabled(false);
        m_ui->pushButton->setEnabled(false);
        m_ui->lineEdit->clear();
    }
}

void ccSORFilterDlg::slotQPushbuttonClicked()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("SORFilterDlgPath");
    QString currentPath = settings.value("SORFilterDlgPath", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    if (!m_ui->lineEdit->text().isEmpty()) {
        currentPath = m_ui->lineEdit->text();
    }
    QString SaveName = CS::Widgets::FramelessFileDialog::getExistingDirectory(this, tr("Save file"), currentPath, QFileDialog::ShowDirsOnly);
    if (SaveName.isEmpty()) {
        settings.group();
        return;
    }
    settings.setValue("SORFilterDlgPath", SaveName);
    settings.endGroup();
    m_ui->lineEdit->setText(SaveName);
    m_fileSavePath = SaveName;
}

void ccSORFilterDlg::slotOkButtonClicked()
{
    if (!m_ui->checkBox->isChecked()) {
        accept();
        return;
    }
    QDir dir(m_ui->lineEdit->text());
    if (m_ui->lineEdit->isEnabled() && m_ui->lineEdit->text().isEmpty())
    {
        m_ui->lineEdit->setStyleSheet("QLineEdit{padding-left:12px;}QLineEdit{border-radius: 2px;border: 1px solid #FF4746;} ");
       
        QTimer::singleShot(1500, this, [=]() {
            if (m_ui->lineEdit) {
                m_ui->lineEdit->setStyleSheet("QLineEdit{padding-left:12px;border: 1px solid #989898;border - radius: 2px;}");
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

void ccSORFilterDlg::setFileSavePath(QString path)
{
    m_fileSavePath = path;
}

QString ccSORFilterDlg::getSaveFilePath()
{
    if (m_ui->checkBox->isChecked()) {
        return m_fileSavePath;
    }
    else {
        return QString();
    }
}
