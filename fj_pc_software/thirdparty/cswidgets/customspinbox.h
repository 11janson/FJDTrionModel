#ifndef __CUSTOMSPINBOX_H__
#define __CUSTOMSPINBOX_H__

#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QKeyEvent>
#include "cswidgets_global.h"
class CSWIDGETS_EXPORT CustomSpinBox : public QDoubleSpinBox
{
public:
    CustomSpinBox(QWidget *parent = nullptr);

protected:
    void focusOutEvent(QFocusEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;
};

#endif // __CUSTOMSPINBOX_H__
