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

#include "ccFillHoleDlg.h"

#include <DgmOctree.h>

ccFillHoleDlg::ccFillHoleDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, Ui::FillHoleDialog()
{
	setupUi(this->getContentHolder());
}

double ccFillHoleDlg::max_hole_diam()
{
    return spinBox->value();
}
int ccFillHoleDlg::max_num_hole_edges()
{
    return 100;
}




