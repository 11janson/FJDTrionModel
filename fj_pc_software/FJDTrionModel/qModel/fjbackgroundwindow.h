#ifndef FJBACKGROUNDWINDOW_H
#define FJBACKGROUNDWINDOW_H

#include <QWidget>

class FJBackgroundWindow : public QWidget
{
    Q_OBJECT

public:
    FJBackgroundWindow(QWidget *parent = nullptr);
    ~FJBackgroundWindow();

protected:
	virtual void paintEvent(QPaintEvent *event);
};
#endif // FJBACKGROUNDWINDOW_H
