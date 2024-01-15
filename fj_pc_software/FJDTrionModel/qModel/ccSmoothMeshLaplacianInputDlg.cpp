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

#include "ccSmoothMeshLaplacianInputDlg.h"
#include "ui_SmoothMeshLaplacianInputDlg.h"


ccSmoothMeshLaplacianInputDlg::ccSmoothMeshLaplacianInputDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::SmoothMeshLaplacianInputDlg)
{
	m_ui->setupUi(this->getContentHolder());
}

ccSmoothMeshLaplacianInputDlg::~ccSmoothMeshLaplacianInputDlg()
{
	delete m_ui;
}


double ccSmoothMeshLaplacianInputDlg::GetSmoothingcoefficient() const
{
	int index = m_ui->comboBox->currentIndex();
    if (index == 0)
    {
        return 0.2;
    }
    else if (index == 1)
    {
        return 0.5;
    }
    else
    {
        return 0.8;
    }
}

