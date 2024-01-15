#ifndef __METAHUBTOOLBUTTON_H__
#define __METAHUBTOOLBUTTON_H__

#include <QWidget>
#include <QToolButton>
#include <QPixmap>
#include <QEvent>
#include "cswidgets_global.h"
class CSWIDGETS_EXPORT MetahubToolButton : public QToolButton
{
    Q_OBJECT

public:
    MetahubToolButton(QWidget *parent = nullptr);
    ~MetahubToolButton();

	/**
	* @brief ����QToolButton��ͬ���״̬ͼ��
	* @param ����״̬ͼ�꣬��껬��״̬ͼ�꣬ʧЧͼ��
	* @return 
    */
	void setIconPixmap(QString normalIcon, QString clickedIcon, QString disabledIcon);

protected:
	virtual void enterEvent(QEvent *e);
	virtual void leaveEvent(QEvent *e);
private:
	QString m_normalIcon;
	QString m_clickedIcon;
	QString m_disabledIcon;
};
#endif // METAHUBTOOLBUTTON_H
