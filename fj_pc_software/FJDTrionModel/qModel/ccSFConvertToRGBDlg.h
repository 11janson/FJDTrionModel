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

#ifndef CCSFCONVERTTORGBDLG
#define CCSFCONVERTTORGBDLG

#include <QDialog>
#include <framelessdialog.h>
namespace Ui {
	class SFConvertToRGBDlg;
}

class ccSFConvertToRGBDlg : public CS::Widgets::FramelessDialog
{
	
	Q_OBJECT
public:
	enum SFCONVERTTORGBDLGSTATE { Rejected, Accepted, Canceled };
	//! Default constructor
	explicit ccSFConvertToRGBDlg(QWidget* parent = nullptr);
	
	~ccSFConvertToRGBDlg();
	/**
    * @brief 获取按钮选择结果
    */
	SFCONVERTTORGBDLGSTATE getResultStale();
	
private:
	Ui::SFConvertToRGBDlg* m_ui;
	SFCONVERTTORGBDLGSTATE m_state = Canceled;
};

#endif //CC_SOR_FILTER_DLG_HEADER
