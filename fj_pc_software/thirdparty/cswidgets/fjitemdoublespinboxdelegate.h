#ifndef __FJITEMCOMBOXDELEGATE_H__
#define __FJITEMCOMBOXDELEGATE_H__
#include "cswidgets_global.h"
#include <QItemDelegate>

class CSWIDGETS_EXPORT fjItemDoubleSpinboxDelegate : public QItemDelegate
{
//	代理类
public:
	fjItemDoubleSpinboxDelegate(QObject *parent = 0);
	~fjItemDoubleSpinboxDelegate();
	QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const;
	void setEditorData(QWidget * editor, const QModelIndex & index) const;
	void setModelData(QWidget * editor, QAbstractItemModel * model, const QModelIndex & index) const;
	void updateEditorGeometry(QWidget * editor, const QStyleOptionViewItem & option, const QModelIndex & index) const;
	
    void setMaxValue(double value);
    void setMinValue(double value);
    void setEnableEmptyStr(bool enable) { m_enableEmpty = enable; }
private:
    double m_dMin;
    double m_dMax;
    bool m_enableEmpty = false;
};
#endif
