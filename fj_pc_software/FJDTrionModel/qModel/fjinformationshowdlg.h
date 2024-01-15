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

#ifndef __FJINFORMATIONSHOWDLG_H__
#define __FJINFORMATIONSHOWDLG_H__

#include <QDialog>
#include "cloudcomparewidgets\metahublframelessdialog.h"
namespace Ui {
	class FjInformationShowDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class FjInformationShowDlg : public CS::MetahubWidgets::MetahubFramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit FjInformationShowDlg(QWidget* parent = nullptr);
	
	~FjInformationShowDlg();
	
	void setTopMessage(QString leftmes,QString message);
	
private:
	Ui::FjInformationShowDlg* m_ui;

    QString m_transMatstr;
};

#endif //__FJINFORMATIONSHOWDLG_H__
