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

#include "fjinformationshowdlg.h"
#include "ui_fjinformationshowdlg.h"


FjInformationShowDlg::FjInformationShowDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::FjInformationShowDlg)
{
	m_ui->setupUi(this->getContentHolder());
    getCancelButton()->setVisible(false);
    getApplyButton()->setVisible(false);
}

FjInformationShowDlg::~FjInformationShowDlg()
{
	delete m_ui;
}

void FjInformationShowDlg::setTopMessage(QString leftmes, QString message)
{
    m_ui->label->setText(leftmes);
	m_ui->label_mintxt->setText(message);
}

