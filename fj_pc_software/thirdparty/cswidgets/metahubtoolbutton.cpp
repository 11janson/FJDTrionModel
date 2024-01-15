#include "metahubtoolbutton.h"
#include "FJStyleManager.h"

MetahubToolButton::MetahubToolButton(QWidget *parent)
    : QToolButton(parent)
{
}

MetahubToolButton::~MetahubToolButton()
{
}


void MetahubToolButton::setIconPixmap(QString normalIcon, QString clickedIcon, QString disabledIcon)
{
	m_normalIcon = normalIcon;
	m_clickedIcon = clickedIcon;
	m_disabledIcon = disabledIcon;
	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_normalIcon + ".png");
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_clickedIcon + ".png"), QIcon::Active, QIcon::Off);
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_disabledIcon + ".png"), QIcon::Disabled, QIcon::Off);
	setIcon(pIcon);
}


void MetahubToolButton::enterEvent(QEvent *e)
{
	QToolButton::enterEvent(e);
	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_clickedIcon + ".png");
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_clickedIcon + ".png"), QIcon::Active, QIcon::Off);
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_disabledIcon + ".png"), QIcon::Disabled, QIcon::Off);
	setIcon(pIcon);
}

void MetahubToolButton::leaveEvent(QEvent *e)
{
	QToolButton::leaveEvent(e);
	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_normalIcon + ".png");
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_clickedIcon + ".png"), QIcon::Active, QIcon::Off);
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + m_disabledIcon + ".png"), QIcon::Disabled, QIcon::Off);
	setIcon(pIcon);
}
