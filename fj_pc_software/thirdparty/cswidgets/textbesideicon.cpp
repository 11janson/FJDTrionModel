#include "textbesideicon.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QFontMetrics>

using namespace CS;
using namespace Widgets;

const int DEFAULT_MIN_WIDTH = 32;

TextBesideIcon::TextBesideIcon(QWidget *parent)
    :QToolButton(parent)
    ,m_pText(nullptr)
    ,m_pIcon(nullptr)
	,m_pHoriSpacer(nullptr)
{
    initWidget();
}

TextBesideIcon::~TextBesideIcon()
{
    if (m_pIcon)
    {
        this->layout()->removeWidget(m_pIcon);
        delete m_pIcon;
        m_pIcon = nullptr;
    }

    if (m_pText)
    {
        this->layout()->removeWidget(m_pText);
        delete m_pText;
        m_pText = nullptr;
    }
}

void TextBesideIcon::initWidget()
{
    QHBoxLayout *pHorLayout = new QHBoxLayout;
    pHorLayout->setMargin(0);
    pHorLayout->setSpacing(10);
	m_pHoriSpacer = new QSpacerItem(30, height(), QSizePolicy::Fixed, QSizePolicy::Minimum);
    //pHorLayout->addItem(m_pHoriSpacer);
	pHorLayout->addStretch();
    m_pIcon = new QToolButton(this);
    m_pIcon->setContentsMargins(0, 0, 0, 0);
    m_pIcon->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    pHorLayout->addWidget(m_pIcon,Qt::AlignLeft);
	m_pIcon->setFixedSize(QSize(36, 36));
    m_pText = new QLabel(this);
    m_pText->setAlignment(Qt::AlignLeft|Qt::AlignVCenter);
    m_pText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    pHorLayout->addWidget(m_pText);
    pHorLayout->addStretch();

    setLayout(pHorLayout);

    connect(m_pIcon, &QToolButton::clicked, this, &TextBesideIcon::click);
}
void TextBesideIcon::setLeftMargin(int margin)
{
	m_pHoriSpacer->changeSize(margin, height());
}
void TextBesideIcon::setIcon(const QIcon &icon)
{
    m_pIcon->setIcon(icon);
}

void TextBesideIcon::setIconSize(const QSize &size)
{
    m_pIcon->setIconSize(size);
    m_pIcon->setFixedSize(size);
}

void TextBesideIcon::setText(const QString &text)
{
    m_pText->setText(text);
}

//void TextBesideIcon::setFixedSize(const QSize &size)
//{
//    this->setFixedSize(size);
//}
//
//void TextBesideIcon::setStyleSheet(QString style)
//{
//    //this->setStyleSheet(style);
//}
//
//void TextBesideIcon::setProperty(const char *name, const QVariant &value)
//{
//    this->setProperty(name, value);
//}

void TextBesideIcon::setWidgetCheckable(bool bChecked)
{
    this->setCheckable(bChecked);
    m_pIcon->setCheckable(bChecked);
}

void TextBesideIcon::setIcons(const QVector<QPair<MouseState, QIcon> > icons)
{
	m_icons = icons;
}


