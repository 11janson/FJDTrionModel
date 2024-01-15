//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef MLSDIALOG_H
#define MLSDIALOG_H

#include <ui_MLSDialog.h>
#include "../MLSSmoothingUpsampling.h"
//Qt
#include <QDialog>
#include <framelessdialog.h>

class MLSDialog : public CS::Widgets::FramelessDialog, public Ui::MLSDialog
{
	Q_OBJECT

public:
	explicit MLSDialog(QWidget* parent = nullptr);
	MLSParameters::UpsamplingMethod GetCurrentMethod();

    void slotCheckBox();

    void slotPushButtonBrowes();

    void setFileSavePath(QString);

    void slotOkButtonClicked();

    QString getSaveFilePath();
public slots:
	void activateMenu(bool ischecked);
protected:
	void deactivateAllMethods();
	void updateCombo();
	MLSParameters::UpsamplingMethod  m_method = MLSParameters::NONE;
private:
    QString m_fileSavePath;
};

#endif // MLSDIALOG_H
