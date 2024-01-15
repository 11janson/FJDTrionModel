#include "SARibbonSeparatorWidget.h"
#include <QStylePainter>
#include <QPainter>
#include <QDebug>

SARibbonSeparatorWidget::SARibbonSeparatorWidget(int height, QWidget *parent)
    : QWidget(parent)
    , m_topMargins(4)
    , m_bottomMargins(4)
{
    setFixedSize(6, height);
}


SARibbonSeparatorWidget::SARibbonSeparatorWidget(QWidget *parent)
    : QWidget(parent)
    , m_topMargins(4)
    , m_bottomMargins(4)
{
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    setFixedWidth(21);
}


QSize SARibbonSeparatorWidget::sizeHint() const
{
    return (QSize(21, height()));
}


/**
 * @brief 设置分割线的上下距离
 * @param top 上边界，默认为4
 * @param bottom 下边界，默认为4
 */
void SARibbonSeparatorWidget::setTopBottomMargins(int top, int bottom)
{
    m_topMargins = top;
    m_bottomMargins = bottom;
}


void SARibbonSeparatorWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);

    painter.setPen(palette().window().color().darker(114));
    int x1 = rect().left()+17;
	//修改分割线上下间距为固定12
    painter.drawLine(QPoint(x1, rect().top() + 12), QPoint(x1, rect().bottom() - 12));
}
