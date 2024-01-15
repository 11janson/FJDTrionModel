#include "checkboxheaderview.h"
#include "FJStyleManager.h"

CheckBoxHeaderView::CheckBoxHeaderView(int checkColumnIndex, QPoint topLeft, QSize size, Qt::Orientation orientation, QWidget *parent) : QHeaderView(orientation, parent)
{
    checkColIndex = checkColumnIndex;
    this->topLeft = topLeft;
    boxSize = size;
    isChecked = true;
}

void CheckBoxHeaderView::setCheckState(bool state)
{
    isChecked = state;
}


void CheckBoxHeaderView::paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
{
    painter->save();
    QHeaderView::paintSection(painter, rect, logicalIndex);
    painter->restore();
    if (logicalIndex == checkColIndex)
    {
        QPixmap pixmapon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/selected.png");
        QPixmap pixmapoff(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/unselected.png");
        QRect targetRect(topLeft.x(), topLeft.y(), boxSize.width(), boxSize.height());  
        if (isChecked)
        {
            painter->drawPixmap(targetRect, pixmapon);
        }
        else
        {
            painter->drawPixmap(targetRect, pixmapoff);
        }
        return;
        QStyleOptionButton option;
        option.rect = QRect(topLeft.x(), topLeft.y(), boxSize.width(), boxSize.height());
        if (isChecked)
        {
            option.state = QStyle::State_On;
            option.icon = QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/selected.png");
        }
        else
        {
            option.state = QStyle::State_Off;
            option.icon = QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/unselected.png");
        }
        //加入复选框，设置样式
        QCheckBox *check = new QCheckBox();
        QString checkstyle = "QCheckBox{color: #ffffff;}\n"
            "QCheckBox::indicator:checked {image: url(theme/qssimage/selected.png);}\n"
            "QCheckBox::indicator:unchecked {image: url(theme/qssimage/unselected.png);}";
        //QString sheet = QString("QCheckBox::indicator {width: %1px;  height: %2px;}").arg(boxSize.width()).arg(boxSize.height());
        //check->setStyleSheet(checkstyle);
        this->style()->drawControl(QStyle::CE_CheckBox, &option, painter, check);
    }
}

void CheckBoxHeaderView::mousePressEvent(QMouseEvent *event)
{
    if (visualIndexAt(event->pos().x()) == checkColIndex)
    {
        isChecked = !isChecked;
        this->updateSection(checkColIndex);
        emit signalCheckStateChanged(isChecked);
    }
    //继承后此信号必须手动发送，否则无法响应
    emit QHeaderView::sectionClicked(visualIndexAt(event->pos().x()));
    QHeaderView::mousePressEvent(event);
}
