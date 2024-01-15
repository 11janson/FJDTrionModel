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

#ifndef CC_CC_LABELING_DLG_HEADER
#define CC_CC_LABELING_DLG_HEADER

#include <ui_labelingDlg.h>
#include <framelessdialog.h>
//! Dialog to define connected components labelinng parameters
class ccLabelingDlg : public CS::Widgets::FramelessDialog, public Ui::LabelingDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccLabelingDlg(QWidget* parent = nullptr);

	//! Returns octree level (defines grid step)
	int getOctreeLevel();

	//! Returns min number of points per extracted CC
	int getMinPointsNb();

	//Set clustering point threshold
	void setMaxPointsNb(int maxPointsNb);
	void setMinPointsNb(int minPointsNb);

	//! Specifies whether each extracted CC should get a random color
	bool randomColors();
};

#endif
