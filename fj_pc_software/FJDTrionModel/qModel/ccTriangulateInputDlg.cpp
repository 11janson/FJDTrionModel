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

#include "ccTriangulateInputDlg.h"
#include "ui_TriangulateInputDlg.h"
#include <QButtonGroup>
#include <QRadioButton>
ccTriangulateInputDlg::ccTriangulateInputDlg(QWidget* parent/*=nullptr*/)
	: CS::MetahubWidgets::MetahubFramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::TriangulateInputDlg )
{
	m_ui->setupUi(this->getContentHolder());
    QButtonGroup * radioButtonGroup = new QButtonGroup(this);
    radioButtonGroup->addButton(m_ui->radioButton);
    radioButtonGroup->addButton(m_ui->radioButton_2);
    connect(m_ui->radioButton, &QRadioButton::toggled, this, &ccTriangulateInputDlg::slotRadioButtonClicked);
    m_ui->label_3->setText(tr("Tip: Perform a hole filling operation on holes that are less than the hole filling threshold."));
}

ccTriangulateInputDlg::~ccTriangulateInputDlg()
{
	delete m_ui;
}

void ccTriangulateInputDlg::slotRadioButtonClicked()
{
    if (m_ui->radioButton->isChecked())
    {
        m_ui->label->setText(tr("Filling threshold"));
        m_ui->doubleSpinBox->setSuffix("(m)");
        m_ui->doubleSpinBox->setDecimals(8);
        m_ui->doubleSpinBox->setRange(0,1000000000.000000);
        m_ui->doubleSpinBox->setSingleStep(1.0);
        m_ui->doubleSpinBox->setValue(0.0);
        m_ui->label_3->setText(tr("Tip: Perform a hole filling operation on holes that are less than the hole filling threshold."));
    }
    else
    {
        m_ui->label->setText(tr("Sampling interval"));
        m_ui->doubleSpinBox->setSuffix("");
        m_ui->doubleSpinBox->setDecimals(2);
        m_ui->doubleSpinBox->setRange(1, 1000.000000);
        m_ui->doubleSpinBox->setSingleStep(1.0);
        m_ui->doubleSpinBox->setValue(10.0);
        m_ui->label_3->setText("");
    }
}

double ccTriangulateInputDlg::GetInputNum()
{
	return m_ui->doubleSpinBox->value();
}


CCCoreLib::TRIANGULATION_TYPES ccTriangulateInputDlg::getTriangulateType()
{
    if (m_ui->radioButton->isChecked())
    {
        return CCCoreLib::DELAUNAY_2D_BEST_LS_PLANE;
    }
    else
    {
        return CCCoreLib::SURFACE_MESH;
    }
}