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
	* @brief 设置QToolButton不同鼠标状态图标
	* @param 正常状态图标，鼠标滑过状态图标，失效图标
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
