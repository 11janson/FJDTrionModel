#include "EditDoubleSpinBox.h"
#include <QMouseEvent>

using namespace CS::Widgets;
EditDoubleSpinBox::EditDoubleSpinBox(QWidget *parent) 
    : QDoubleSpinBox(parent)
{
  
}

void EditDoubleSpinBox::getSpinBoxChild()
{
    m_pLineEdit = this->findChild<QLineEdit*>();
    m_pLineEdit->installEventFilter(this); //声明机制的存在
}

void EditDoubleSpinBox::setPropertyValue(QString strValue)
{
    m_strValue = strValue;
}

//重写过滤器
bool EditDoubleSpinBox::eventFilter(QObject *obj, QEvent *e)
{
    if (e->type() == QEvent::MouseButtonPress) {
        QMouseEvent *me = (QMouseEvent*)e;
        if (me->button() == Qt::LeftButton) {
            if (obj == m_pLineEdit) {
                emit signalSpinBoxClicked(m_strValue);
            }
        }
    }
    return QObject::eventFilter(obj, e);
}
