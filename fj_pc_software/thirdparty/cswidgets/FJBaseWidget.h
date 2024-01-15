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
	*@brief ��ʼ��������ʽ���̳и�����Ҫ�ȵ��ø���˽ӿڣ�����ɫ��ͼ�����ʽ������д��˽ӿڣ��л�Ƥ�������ݼ̳й�ϵ�𲽵��ô˽ӿ�
	*/
	virtual void InitFJStyle()=0;
public slots:
    void InitStyleslot();
};
#endif // WIDGET_H
