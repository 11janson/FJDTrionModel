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

#ifndef __POINTPAIRREGISTRATIONINFORMATIONDLG_H__
#define __POINTPAIRREGISTRATIONINFORMATIONDLG_H__

#include <QDialog>
#include "cloudcomparewidgets\metahublframelessdialog.h"
namespace Ui {
	class PointPairRegistrationInformationDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class ccPointPairRegistrationInformationDlg : public CS::MetahubWidgets::MetahubFramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPointPairRegistrationInformationDlg(QWidget* parent = nullptr);
	
	~ccPointPairRegistrationInformationDlg();
	
	void setTopMessage(QString message);

	void setMidMessage(QString message);

	void setBottomMessage(QString message);

    void setTransMatstr(QString message);

    void setSaveButtonVisiable(bool isshow);

public slots:
    void slotUIButtonSave();
	
private:
	Ui::PointPairRegistrationInformationDlg* m_ui;

    QString m_transMatstr;
};

#endif //CC_SOR_FILTER_DLG_HEADER
