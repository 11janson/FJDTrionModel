﻿#include "flowlayout.h"

#include <algorithm>
#include <QWidget>

using namespace CS;
using namespace CS::Widgets;
FlowLayout::FlowLayout(QWidget *parent, int margin, int hSpacing, int vSpacing)
	: QLayout(parent),
	x_hSpace(hSpacing),
	x_vSpace(vSpacing)
{
	setContentsMargins(margin, margin, margin, margin);
}

FlowLayout::FlowLayout(int margin, int hSpacing, int vSpacing)
	: x_hSpace(hSpacing),
	x_vSpace(vSpacing)
{
	setContentsMargins(margin, margin, margin, margin);
}

FlowLayout::~FlowLayout()
{
	QLayoutItem *item;
	while ((item = takeAt(0)))
		delete item;
}

void FlowLayout::addItem(QLayoutItem *item)
{
	x_itemList.append(item);
}

int FlowLayout::horizontalSpacing() const
{
	if (x_hSpace >= 0) 
	{
		return x_hSpace;
	}
	else 
	{
		return smartSpacing(QStyle::PM_LayoutHorizontalSpacing);
	}
}

int FlowLayout::verticalSpacing() const
{
	if (x_vSpace >= 0) {
		return x_vSpace;
	}
	else {
		return smartSpacing(QStyle::PM_LayoutVerticalSpacing);
	}
}

int FlowLayout::count() const
{
	return x_itemList.size();
}

QLayoutItem *FlowLayout::itemAt(int index) const
{
	return x_itemList.value(index);
}

QLayoutItem *FlowLayout::takeAt(int index)
{
	if (index >= 0 && index < x_itemList.size())
		return x_itemList.takeAt(index);
	else
		return 0;
}

Qt::Orientations FlowLayout::expandingDirections() const
{
	return 0;
}

bool FlowLayout::hasHeightForWidth() const
{
	return true;
}

int FlowLayout::heightForWidth(int width) const
{
	int height = doLayout(QRect(0, 0, width, 0), true);
	return height;
}

void FlowLayout::setGeometry(const QRect &rect)
{
	QLayout::setGeometry(rect);
	doLayout(rect, false);
}

QSize FlowLayout::sizeHint() const
{
	return minimumSize();
}

QSize FlowLayout::minimumSize() const
{
	QSize size;
	QLayoutItem *item;
	foreach(item, x_itemList)
		size = size.expandedTo(item->minimumSize());

	size += QSize(2 * margin(), 2 * margin());
	return size;
}

int FlowLayout::doLayout(const QRect &rect, bool testOnly) const
{
	int left, top, right, bottom;
	getContentsMargins(&left, &top, &right, &bottom);
	QRect effectiveRect = rect.adjusted(+left, +top, -right, -bottom);
	int x = effectiveRect.x();
	int y = effectiveRect.y();
	int lineHeight = 0;

	QLayoutItem *item;
	foreach(item, x_itemList) {
		QWidget *wid = item->widget();
		int spaceX = horizontalSpacing();
		if (spaceX == -1)
			spaceX = wid->style()->layoutSpacing(
			QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Horizontal);
		int spaceY = verticalSpacing();
		if (spaceY == -1)
			spaceY = wid->style()->layoutSpacing(
			QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Vertical);

		int nextX = x + item->sizeHint().width() + spaceX;
		if (nextX - spaceX > effectiveRect.right() && lineHeight > 0) {
			x = effectiveRect.x();
			y = y + lineHeight + spaceY;
			nextX = x + item->sizeHint().width() + spaceX;
			lineHeight = 0;
		}

		if (!testOnly)
			item->setGeometry(QRect(QPoint(x, y), item->sizeHint()));

		x = nextX;
		lineHeight = qMax(lineHeight, item->sizeHint().height());
	}
	return y + lineHeight - rect.y() + bottom;
}

int FlowLayout::smartSpacing(QStyle::PixelMetric pm) const
{
	QObject *parent = this->parent();
	if (!parent) {
		return -1;
	}
	else if (parent->isWidgetType()) {
		QWidget *pw = static_cast<QWidget *>(parent);
		return pw->style()->pixelMetric(pm, 0, pw);
	}
	else {
		return static_cast<QLayout *>(parent)->spacing();
	}
}
