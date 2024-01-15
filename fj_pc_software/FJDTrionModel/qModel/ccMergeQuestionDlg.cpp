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

#include "ccMergeQuestionDlg.h"
#include "ui_MergeQuestionDlg.h"


ccMergeQuestionDlg::ccMergeQuestionDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::MergeQuestionDlg)
{
	m_ui->setupUi(this->getContentHolder());
}

ccMergeQuestionDlg::~ccMergeQuestionDlg()
{
	delete m_ui;
}


void ccMergeQuestionDlg::SetCloudNum(int num)
{
	m_ui->label_num->setText(QString::number(num));
}
