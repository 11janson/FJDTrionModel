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

#ifndef CC_TRIANGULATEINPUTDLG
#define CC_TRIANGULATEINPUTDLG

#include <QDialog>
#include "cloudcomparewidgets/metahublframelessdialog.h"
#include "PointProjectionTools.h"
namespace Ui {
	class TriangulateInputDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class ccTriangulateInputDlg : public CS::MetahubWidgets::MetahubFramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccTriangulateInputDlg(QWidget* parent = nullptr);
	
	~ccTriangulateInputDlg();
	double GetInputNum();

    CCCoreLib::TRIANGULATION_TYPES getTriangulateType();

public slots:
    void slotRadioButtonClicked();
private:
	Ui::TriangulateInputDlg* m_ui;
};

#endif //CC_SOR_FILTER_DLG_HEADER
