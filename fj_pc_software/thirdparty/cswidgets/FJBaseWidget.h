#ifndef WIDGET_H
#define WIDGET_H
#include <QDialog>
#include <QWidget>
#include "cswidgets_global.h"
class CSWIDGETS_EXPORT FJBaseWidget : public QDialog
{
    Q_OBJECT   

public:
	explicit FJBaseWidget(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~FJBaseWidget();
	/**
	*@brief 初始化界面样式，继承该类需要先调用父类此接口，将颜色、图标等样式的设置写入此接口，切换皮肤后会根据继承关系逐步调用此接口
	*/
	virtual void InitFJStyle()=0;
public slots:
    void InitStyleslot();
};
#endif // WIDGET_H
