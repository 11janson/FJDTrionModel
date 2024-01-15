//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #

#ifndef __TRIONABOUTDLG_H__
#define __TRIONABOUTDLG_H__

#include <QDialog>
#include <QNetworkReply>
#include "framelessdialog.h"

namespace Ui {
	class TrionAboutDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class TrionAboutDlg : public CS::Widgets::FramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit TrionAboutDlg(QWidget* parent = nullptr);
	
	~TrionAboutDlg();

	
public slots:
	void checkVersion();
	void requestRemoteVersionFinished(QNetworkReply *reply);
private:
	Ui::TrionAboutDlg* m_ui;
};

#endif //CC_SOR_FILTER_DLG_HEADER
