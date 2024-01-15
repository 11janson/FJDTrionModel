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

#include "ccTriangleNetDlg.h"

#include <DgmOctree.h>

ccTriangleDlg::ccTriangleDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, Ui::TriangleDialog()
{
	setupUi(this->getContentHolder());
    layout()->setContentsMargins(24, 14, 24, 16);
    doubleSpinBox->setRange(1.0, 100.0);
}

surfaceMeshPara ccTriangleDlg::getSurfaceMeshPara()
{
    surfaceMeshPara para;

    para.methodType = comboBox->currentIndex();
    para.subFactor = doubleSpinBox->value();

    return para;
}
