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

#ifndef CC_CC_VOLDISPLAY_DLG_HEADER
#define CC_CC_VOLDISPLAY_DLG_HEADER

#include <ui_volumeDisplayDlg.h>
#include"libs/cloudcomparewidgets/metahublframelessdialog.h"
//! Dialog to define connected components labelinng parameters
class ccVolDisplayDlg : public CS::MetahubWidgets::MetahubFramelessDialog, public Ui::VolumeDisplayDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccVolDisplayDlg(QWidget* parent = nullptr);


	void setFileName(QString name);
	void setCalcData(QString strData);
    void setLabelInfoVisble(bool state);
    void setAreaData(QString strData);
};

#endif
