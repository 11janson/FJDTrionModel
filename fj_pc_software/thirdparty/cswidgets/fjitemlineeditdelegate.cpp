#include <QDoubleSpinBox>
#include "fjitemlineeditdelegate.h"
FJItemLineEditDelegate::FJItemLineEditDelegate(const QRegExp& regExp,QObject *parent):QItemDelegate(parent)
{
	m_regExp = regExp;
}
FJItemLineEditDelegate::~FJItemLineEditDelegate()
{

}
QWidget * FJItemLineEditDelegate::createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const
{

	Q_UNUSED(option);
	Q_UNUSED(index);
	QLineEdit* editor = new QLineEdit(parent);
	editor->setValidator(new QRegExpValidator(m_regExp, parent));
	return editor;
}
void FJItemLineEditDelegate::setEditorData(QWidget * editor, const QModelIndex & index) const
{
	QString text = index.model()->data(index, Qt::EditRole).toString();
	QLineEdit* lineEdit = qobject_cast <QLineEdit*>(editor);
	lineEdit->setText(text);
}
void FJItemLineEditDelegate::setModelData(QWidget * editor, QAbstractItemModel * model, const QModelIndex & index) const
{
	QLineEdit* lineEdit = qobject_cast<QLineEdit*>(editor);
	QString text = lineEdit->text();
	model->setData(index, text, Qt::EditRole);
}
void FJItemLineEditDelegate::updateEditorGeometry(QWidget * editor, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	editor->setGeometry(option.rect);
}