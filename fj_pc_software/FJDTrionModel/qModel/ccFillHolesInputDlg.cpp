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

#include "ccFillHolesInputDlg.h"
#include "ui_FillHolesInputDlg.h"


ccFillHolesInputDlg::ccFillHolesInputDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::FillHolesInputDlg)
{
	m_ui->setupUi(this->getContentHolder());
	setFixedSize(440, 309);
}

ccFillHolesInputDlg::~ccFillHolesInputDlg()
{
	delete m_ui;
}

double ccFillHolesInputDlg::max_hole_diam()
{
    return m_ui->doubleSpinBox->value();
}
int ccFillHolesInputDlg::max_num_hole_edges()
{
    return m_ui->spinBox->value();
}
