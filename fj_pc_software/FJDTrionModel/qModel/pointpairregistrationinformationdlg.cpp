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

#include "pointpairregistrationinformationdlg.h"
#include "ui_pointpairregistrationinformationdlg.h"
#include "QSettings"
#include "cloudcompareutils\icore.h"
#include "QStandardPaths"
#include "framelessfiledialog.h"
#include "QTextStream"
#include "framelessmessagebox.h"


ccPointPairRegistrationInformationDlg::ccPointPairRegistrationInformationDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::PointPairRegistrationInformationDlg)
{
	m_ui->setupUi(this->getContentHolder());
    getCancelButton()->setVisible(false);
    getApplyButton()->setVisible(true);
    getApplyButton()->setText(tr("Save"));
    connect(getApplyButton(), &QAbstractButton::clicked, this, &ccPointPairRegistrationInformationDlg::slotUIButtonSave);
}

void ccPointPairRegistrationInformationDlg::setSaveButtonVisiable(bool isshow)
{
    getApplyButton()->setVisible(isshow);
}

ccPointPairRegistrationInformationDlg::~ccPointPairRegistrationInformationDlg()
{
	delete m_ui;
}

void ccPointPairRegistrationInformationDlg::setTopMessage(QString message)
{
	m_ui->label_toptxt->setText(message);
}

void ccPointPairRegistrationInformationDlg::setMidMessage(QString message)
{
	m_ui->label_mintxt->setText(message);
}

void ccPointPairRegistrationInformationDlg::setBottomMessage(QString message)
{
	m_ui->label_bottomtxt->setText(message);
}

void ccPointPairRegistrationInformationDlg::slotUIButtonSave()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("ccPointPairRegistrationInformationDlg");
    QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this, tr("Select output file"), currentPath, "*.txt");
    if (outputFilename.isEmpty())
    {
        settings.endGroup();
        return;
    }
    settings.setValue("filedoc", QFileInfo(outputFilename).absolutePath());
    settings.endGroup();
    if (!outputFilename.endsWith(".txt", Qt::CaseInsensitive))
    {
        outputFilename += ".txt";
    }
    QFile file(outputFilename);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    QTextStream out(&file);
    out.setCodec("UTF-8");
    out << tr("Transformation matrix:") << "\n";

    out << m_transMatstr << "\n";
    out << tr("Scale factor:") << "\n";
    out << m_ui->label_bottomtxt->text() << "\n";

    file.close();
    CS::Widgets::FramelessMessageBox::information(this, tr("Information"), tr("Save successfully."));
}

void ccPointPairRegistrationInformationDlg::setTransMatstr(QString message)
{
    m_transMatstr = message;
}