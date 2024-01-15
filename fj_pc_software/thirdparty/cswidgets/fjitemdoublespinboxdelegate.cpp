#include <QDoubleSpinBox>
#include "fjitemdoublespinboxdelegate.h"
fjItemDoubleSpinboxDelegate::fjItemDoubleSpinboxDelegate(QObject *parent):QItemDelegate(parent)
{
    m_dMin = -999999999.9999999;
    m_dMax = 999999999.9999999;
}
fjItemDoubleSpinboxDelegate::~fjItemDoubleSpinboxDelegate()
{

}
QWidget * fjItemDoubleSpinboxDelegate::createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
	editor->setButtonSymbols(QAbstractSpinBox::NoButtons);
	editor->setDecimals(3);
    editor->setMinimum(m_dMin);
    editor->setMaximum(m_dMax);
	editor->setSingleStep(0);
	return editor;
}
void fjItemDoubleSpinboxDelegate::setEditorData(QWidget * editor, const QModelIndex & index) const
{
    bool istextempty = index.model()->data(index, Qt::EditRole).toString().isEmpty();
	double value = index.model()->data(index, Qt::EditRole).toDouble();
	QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
    if (istextempty && m_enableEmpty)
    {
        spinBox->clear();
    }
    else
    {
        spinBox->setValue(value);
    }
}
void fjItemDoubleSpinboxDelegate::setModelData(QWidget * editor, QAbstractItemModel * model, const QModelIndex & index) const
{
	QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
    bool istextempty = spinBox->text().isEmpty();
	spinBox->interpretText();
	double value = spinBox->value();
	QString valueStr = QString::number(value,'f', 3);
    if (istextempty && m_enableEmpty)
    {
        valueStr = "";
    }
	model->setData(index, valueStr, Qt::EditRole);
}
void fjItemDoubleSpinboxDelegate::updateEditorGeometry(QWidget * editor, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	editor->setGeometry(option.rect);
}

void fjItemDoubleSpinboxDelegate::setMaxValue(double value)
{
    m_dMax = value;
}

void fjItemDoubleSpinboxDelegate::setMinValue(double value)
{
    m_dMin = value;
}