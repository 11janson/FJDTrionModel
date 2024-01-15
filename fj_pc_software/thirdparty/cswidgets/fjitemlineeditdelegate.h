#ifndef __FJITEMLINEEDITDELEGATE_H__
#define __FJITEMLINEEDITDELEGATE_H__
#include "cswidgets_global.h"
#include <QItemDelegate>
#include <QLineEdit>
#include <QRegExp>
class CSWIDGETS_EXPORT FJItemLineEditDelegate : public QItemDelegate
{
//	代理类
public:
	FJItemLineEditDelegate(const QRegExp& regExp, QObject *parent = 0);
	~FJItemLineEditDelegate();
	QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const;
	void setEditorData(QWidget * editor, const QModelIndex & index) const;
	void setModelData(QWidget * editor, QAbstractItemModel * model, const QModelIndex & index) const;
	void updateEditorGeometry(QWidget * editor, const QStyleOptionViewItem & option, const QModelIndex & index) const;


private:
	QRegExp m_regExp;
};
#endif
