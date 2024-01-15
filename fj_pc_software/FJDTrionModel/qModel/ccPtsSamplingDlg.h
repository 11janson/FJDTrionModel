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

#ifndef CC_POINTS_SAMPLING_DLG_HEADER
#define CC_POINTS_SAMPLING_DLG_HEADER

#include <ui_ptsSamplingDlg.h>
#include "cloudcomparewidgets/metahublframelessdialog.h"
//! Dialog: points sampling on a mesh
class ccPtsSamplingDlg : public CS::MetahubWidgets::MetahubFramelessDialog, public Ui::PointsSamplingDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPtsSamplingDlg(QWidget* parent = nullptr);


	bool useDensity() const;
	double getDensityValue() const;
    double getPointsNumber() const;

public slots:
    void slotRadioButtonClicked();
signals:
    void signalSendStatusBarInfo(QString);

};

#endif //CC_POINTS_SAMPLING_DLG_HEADER
