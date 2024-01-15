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

#include "ccVolDisplayDlg.h"

#include <DgmOctree.h>

ccVolDisplayDlg::ccVolDisplayDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, Ui::VolumeDisplayDlg()
{
	setupUi(this->getContentHolder());

    label_info->setVisible(false);
}


void ccVolDisplayDlg::setFileName(QString name)
{
    label_Name->setToolTip(name);

    QFontMetrics fontWidth(label_Name->font());//得到每个字符的宽度
    QString elideNote = fontWidth.elidedText(name, Qt::ElideRight, 240);//最大宽度150像素

    label_Name->setText(elideNote);//显示省略好的字符串
}

void ccVolDisplayDlg::setCalcData(QString strData)
{
    label_value->setText(strData);
}

void ccVolDisplayDlg::setLabelInfoVisble(bool state)
{
    label_info->setVisible(state);
    if (state)
    {
        setFixedHeight(305);
    }
    else
    {
        setFixedHeight(245);
    }
}

void ccVolDisplayDlg::setAreaData(QString strData)
{
    label_3->setText(strData);
}

