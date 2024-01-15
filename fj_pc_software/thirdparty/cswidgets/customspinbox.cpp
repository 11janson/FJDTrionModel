#include "customspinbox.h"

CustomSpinBox::CustomSpinBox(QWidget *parent) : QDoubleSpinBox(parent)
{}

void CustomSpinBox::focusOutEvent(QFocusEvent *event)
{
    // 当spinbox失去焦点时，如果内容为空，则将其数值保存为空
    //if (text().isEmpty())
    //{
    //    //setValue(0);
    //    clear();
    //}

    QDoubleSpinBox::focusOutEvent(event);
}

void CustomSpinBox::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
    {
        //clear();  
        QDoubleSpinBox::keyPressEvent(event);
        if (text().isEmpty())
        {
            setValue(0);
            clear();
        }
        emit textChanged("");
    }
    else if (event->modifiers() == Qt::ControlModifier && event->key() == Qt::Key_V) {
        QDoubleSpinBox::keyPressEvent(event);
        emit textChanged("");
    }
    else
    {
        QDoubleSpinBox::keyPressEvent(event);
    }
}