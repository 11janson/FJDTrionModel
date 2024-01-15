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

#include "ccSFConvertToRGBDlg.h"
#include "ui_SFConvertToRGBDlg.h"
#include <QPushButton>

ccSFConvertToRGBDlg::ccSFConvertToRGBDlg(QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::SFConvertToRGBDlg)
{
	m_ui->setupUi(this->getContentHolder());
	this->bottomWidget()->setVisible(false);
	connect(m_ui->pushButton_ok, &QPushButton::clicked, [this]() {
		m_state = ccSFConvertToRGBDlg::SFCONVERTTORGBDLGSTATE::Accepted; 
		accept(); });
	connect(m_ui->pushButton_no, &QPushButton::clicked, [this]() {
		m_state = ccSFConvertToRGBDlg::SFCONVERTTORGBDLGSTATE::Rejected;
		accept(); });
	connect(m_ui->pushButton_cancel, &QPushButton::clicked, [this]() {
		m_state = ccSFConvertToRGBDlg::SFCONVERTTORGBDLGSTATE::Canceled;
		accept(); });
}

ccSFConvertToRGBDlg::~ccSFConvertToRGBDlg()
{
	delete m_ui;
}


ccSFConvertToRGBDlg::SFCONVERTTORGBDLGSTATE ccSFConvertToRGBDlg::getResultStale()
{
	return m_state;
}

