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

#ifndef CC_CC_FILLHOLE_DLG_HEADER
#define CC_CC_FILLHOLE_DLG_HEADER

#include <ui_fillholeDlg.h>
#include <framelessdialog.h>
//! Dialog to define connected components labelinng parameters
class ccFillHoleDlg : public CS::Widgets::FramelessDialog, public Ui::FillHoleDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccFillHoleDlg(QWidget* parent = nullptr);

    double max_hole_diam();
    int max_num_hole_edges();
};

#endif
