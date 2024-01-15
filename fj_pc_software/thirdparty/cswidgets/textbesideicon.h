#pragma once
#include "cswidgets_global.h"
#include <QFrame>
#include <QLabel>
#include <QToolButton>
#include <QPushButton>
#include <QIcon>
#include <QVariant>
#include <QHBoxLayout>

namespace CS {
namespace Widgets {
   class CSWIDGETS_EXPORT TextBesideIcon :public QToolButton
{
    Q_OBJECT
public:
	enum MouseState { Normal, Pressed, Hover };
public:
    TextBesideIcon(QWidget *parent = 0);
    ~TextBesideIcon();
    void initWidget();
	void setLeftMargin(int margin);
	void setIcon(const QIcon &icon);
    void setIconSize(const QSize &size);
    void setText(const QString &text);
	QString text() { return m_pText->text(); }
	/* void setFixedSize(const QSize &size);
	 void setStyleSheet(QString style);
	 void setProperty(const char *name, const QVariant &value);*/
    void setWidgetCheckable(bool bChecked);
	void setIcons(const QVector<QPair<MouseState, QIcon> > icons);
	QVector<QPair<MouseState, QIcon>>  icons() { return m_icons; }
	QToolButton* iconToolbutton() { return m_pIcon; }
	QLabel*      textLabel()  {return m_pText; }

   
signals:
    void signal_buttonClicked();


private:
    QLabel                   *m_pText;
    QToolButton              *m_pIcon;
	QVector<QPair<MouseState, QIcon>>             m_icons;
	bool        m_isChecked = false;
	QSpacerItem              *m_pHoriSpacer;
  
};

}
}

