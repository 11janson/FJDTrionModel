#include "frameline.h"
#include <QHBoxLayout>
#include <QSplitter>

using namespace CS;
using namespace CS::Widgets;

FrameLine::FrameLine(QObject *parent)
    : QObject(parent)
{

}

FrameLine::~FrameLine()
{

}

QFrame* FrameLine::createHorizontalLine(
        const QString &strLinePropertyName,
        QWidget*parent,
        const quint32 &lineWidth,
        const quint32 &marginTB,
        const quint32 &marginLR)
{
    QHBoxLayout*  pHSeparatorLayout = new QHBoxLayout();
    pHSeparatorLayout->setSpacing(0);
    pHSeparatorLayout->setContentsMargins(marginLR, marginTB, marginLR, marginTB);
    QFrame *pHSeparator = new QFrame(parent);
    pHSeparator->setFixedHeight(lineWidth);
    //    pHSeparator->setFrameShape(QFrame::HLine);
    pHSeparator->setFrameShadow(QFrame::Plain);
    pHSeparator->setLineWidth(lineWidth);
    pHSeparator->setProperty(strLinePropertyName.toStdString().c_str(), true);
    pHSeparator->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pHSeparatorLayout->addWidget(pHSeparator);
    return pHSeparator;
}

QFrame* FrameLine::createVerticalLine(
        const QString &strLinePropertyName,
        QWidget*parent,
        const quint32 &lineWidth,
        const quint32 &marginLR,
        const quint32 &marginTB)
{
    QVBoxLayout*  pHSeparatorLayout = new QVBoxLayout();
    pHSeparatorLayout->setSpacing(0);
    pHSeparatorLayout->setContentsMargins(marginLR, marginTB, marginLR, marginTB);
    QFrame *pHSeparator = new QFrame(parent);
    pHSeparator->setFixedWidth(lineWidth);
    pHSeparator->setFrameShape(QFrame::VLine);
    pHSeparator->setFrameShadow(QFrame::Plain);
    pHSeparator->setLineWidth(lineWidth);
    pHSeparator->setProperty(strLinePropertyName.toStdString().c_str(), true);
    pHSeparator->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    pHSeparatorLayout->addWidget(pHSeparator);
    return pHSeparator;
}


