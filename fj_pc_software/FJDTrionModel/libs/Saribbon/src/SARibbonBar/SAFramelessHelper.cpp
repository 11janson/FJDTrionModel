﻿#include "SAFramelessHelper.h"
#include <QRect>
#include <QRubberBand>
#include <QMouseEvent>
#include <QHoverEvent>
#include <QApplication>
#include <QScreen>
#include <QDebug>
#include <QDesktopWidget>
#include <QStyle>
#include "SARibbonMainWindow.h"
#include <QTimer>

class WidgetData;

/*****
* FramelessHelperPrivate
* 存储界面对应的数据集合，以及是否可移动、可缩放属性
*****/
class FramelessHelperPrivate 
{
public:
    FramelessHelperPrivate(SAFramelessHelper *ptr) {
        m_dptr = ptr;
    }
    QHash<QWidget *, WidgetData *> m_widgetDataHash;
    bool m_bWidgetMovable        : true;
    bool m_bWidgetResizable      : true;
    bool m_bRubberBandOnResize   : true;
    bool m_bRubberBandOnMove     : true;
    friend class SAFramelessHelper;
    SAFramelessHelper *m_dptr      = nullptr;

};

/*****
* CursorPosCalculator
* 计算鼠标是否位于左、上、右、下、左上角、左下角、右上角、右下角
*****/
class CursorPosCalculator
{
public:
    explicit CursorPosCalculator();
    void reset();
    void recalculate(const QPoint& globalMousePos, const QRect& frameRect);

public:
    bool m_bOnEdges              : true;
    bool m_bOnLeftEdge           : true;
    bool m_bOnRightEdge          : true;
    bool m_bOnTopEdge            : true;
    bool m_bOnBottomEdge         : true;
    bool m_bOnTopLeftEdge        : true;
    bool m_bOnBottomLeftEdge     : true;
    bool m_bOnTopRightEdge       : true;
    bool m_bOnBottomRightEdge    : true;

    static int m_nBorderWidth;
    static int m_nTitleHeight;
};

int CursorPosCalculator::m_nBorderWidth = 5;
int CursorPosCalculator::m_nTitleHeight = 30;

/***** CursorPosCalculator *****/
CursorPosCalculator::CursorPosCalculator()
{
    reset();
}


void CursorPosCalculator::reset()
{
    m_bOnEdges = false;
    m_bOnLeftEdge = false;
    m_bOnRightEdge = false;
    m_bOnTopEdge = false;
    m_bOnBottomEdge = false;
    m_bOnTopLeftEdge = false;
    m_bOnBottomLeftEdge = false;
    m_bOnTopRightEdge = false;
    m_bOnBottomRightEdge = false;
}


void CursorPosCalculator::recalculate(const QPoint& gMousePos, const QRect& frameRect)
{
    int globalMouseX = gMousePos.x();
    int globalMouseY = gMousePos.y();

    int frameX = frameRect.x();
    int frameY = frameRect.y();

    int frameWidth = frameRect.width();
    int frameHeight = frameRect.height();

    m_bOnLeftEdge = (globalMouseX >= frameX &&
        globalMouseX <= frameX + m_nBorderWidth);


    m_bOnRightEdge = (globalMouseX >= frameX + frameWidth - m_nBorderWidth &&
        globalMouseX <= frameX + frameWidth);

    m_bOnTopEdge = (globalMouseY >= frameY &&
        globalMouseY <= frameY + m_nBorderWidth);

    m_bOnBottomEdge = (globalMouseY >= frameY + frameHeight - m_nBorderWidth &&
        globalMouseY <= frameY + frameHeight);

    m_bOnTopLeftEdge = m_bOnTopEdge && m_bOnLeftEdge;
    m_bOnBottomLeftEdge = m_bOnBottomEdge && m_bOnLeftEdge;
    m_bOnTopRightEdge = m_bOnTopEdge && m_bOnRightEdge;
    m_bOnBottomRightEdge = m_bOnBottomEdge && m_bOnRightEdge;

    m_bOnEdges = m_bOnLeftEdge || m_bOnRightEdge || m_bOnTopEdge || m_bOnBottomEdge;
}


/*****
* WidgetData
* 更新鼠标样式、移动窗体、缩放窗体
*****/
class WidgetData : public QObject
{
    Q_OBJECT
public:
    explicit WidgetData(FramelessHelperPrivate *d, QWidget *pTopLevelWidget);
    ~WidgetData();
    QWidget *widget();

    // 处理鼠标事件-划过、按下、释放、移动
    bool handleWidgetEvent(QEvent *event);

    // 更新橡皮筋状态
    void updateRubberBandStatus();

    uint getWigetHeight();
    //[!]记录当前在哪个屏幕
    int  currentWindowIndex = 0;
private:
    // 更新鼠标样式
    void updateCursorShape(const QPoint& gMousePos);

    // 重置窗体大小
    void resizeWidget(const QPoint& gMousePos);

    // 移动窗体
    void moveWidget(const QPoint& gMousePos);

    // 处理鼠标按下
    bool handleMousePressEvent(QMouseEvent *event);

    // 处理鼠标释放
    bool handleMouseReleaseEvent(QMouseEvent *event);

    // 处理鼠标移动
    bool handleMouseMoveEvent(QMouseEvent *event);

    // 处理鼠标离开
    bool handleLeaveEvent(QEvent *event);

    // 处理鼠标进入
    bool handleHoverMoveEvent(QHoverEvent *event);

    //处理鼠标双击事件
    bool handleDoubleClickedMouseEvent(QMouseEvent *event);

    //[!]处理窗口状态发生变化事件
    bool handWindowStateChange(QEvent *event);
private slots:
    void slotScreenChange();

    void slotPrimaryScreenChanged();
private:
    FramelessHelperPrivate *d;
    QRubberBand *m_pRubberBand;
    QWidget *m_pWidget;
    QPoint m_ptDragPos;
    CursorPosCalculator m_pressedMousePos;
    CursorPosCalculator m_moveMousePos;
    bool m_bLeftButtonPressed;
    bool m_bCursorShapeChanged;
    bool m_bLeftButtonTitlePressed;
    Qt::WindowFlags m_windowFlags;
    //[!]用来记录当前窗口的高度，用高度判断是否最大化了
    int currentWidgetHeight = -1;
    //[!]记录最小化前窗口的属性
    QRect m_Rect;
};

/***** WidgetData *****/
WidgetData::WidgetData(FramelessHelperPrivate *_d, QWidget *pTopLevelWidget)
{
    d = _d;
    m_pWidget = pTopLevelWidget;
    m_bLeftButtonPressed = false;
    m_bCursorShapeChanged = false;
    m_bLeftButtonTitlePressed = false;
    m_pRubberBand = NULL;

    m_windowFlags = m_pWidget->windowFlags();
    m_pWidget->setMouseTracking(true);
    m_pWidget->setAttribute(Qt::WA_Hover, true);

    updateRubberBandStatus();
    QObject::connect(qApp, &QGuiApplication::screenRemoved, this, &WidgetData::slotScreenChange, Qt::UniqueConnection);
    QList<QScreen *> screens = QGuiApplication::screens();
    for (int i = 0; i < screens.size(); i++) {
        QScreen *screen = screens.at(i);
        connect(screen, &QScreen::geometryChanged, this, &WidgetData::slotPrimaryScreenChanged,Qt::UniqueConnection);
    }


}


WidgetData::~WidgetData()
{
    m_pWidget->setMouseTracking(false);
    m_pWidget->setWindowFlags(m_windowFlags);
    m_pWidget->setAttribute(Qt::WA_Hover, false);

    delete m_pRubberBand;
    m_pRubberBand = NULL;
}
void WidgetData::slotPrimaryScreenChanged()
{
    if (m_pWidget)
    {
        m_pWidget->setWindowState(Qt::WindowMaximized);
        m_pWidget->adjustSize();

    }
}

QWidget *WidgetData::widget()
{
    return (m_pWidget);
}

bool WidgetData::handleWidgetEvent(QEvent *event)
{
    switch (event->type())
    {
    case QEvent::MouseButtonPress:
        return (handleMousePressEvent(static_cast<QMouseEvent *>(event)));

    case QEvent::MouseButtonRelease:
        return (handleMouseReleaseEvent(static_cast<QMouseEvent *>(event)));

    case QEvent::MouseMove:
        return (handleMouseMoveEvent(static_cast<QMouseEvent *>(event)));

    case QEvent::Leave:
        return (handleLeaveEvent(static_cast<QMouseEvent *>(event)));

    case QEvent::HoverMove:
        return (handleHoverMoveEvent(static_cast<QHoverEvent *>(event)));

    case QEvent::MouseButtonDblClick:
        return (handleDoubleClickedMouseEvent(static_cast<QMouseEvent *>(event)));
    case  QEvent::WindowStateChange:
        return (handWindowStateChange(static_cast<QEvent *>(event)));
    default:
        break;
    }
    return (false);
}


void WidgetData::updateRubberBandStatus()
{
    if (d->m_bRubberBandOnMove || d->m_bRubberBandOnResize) {
        if (NULL == m_pRubberBand) {
            m_pRubberBand = new QRubberBand(QRubberBand::Rectangle);
        }
    }else {
        delete m_pRubberBand;
        m_pRubberBand = NULL;
    }
}


void WidgetData::updateCursorShape(const QPoint& gMousePos)
{
    if (m_pWidget->height() == currentWidgetHeight) {
        if (m_bCursorShapeChanged) {
            m_pWidget->unsetCursor();
        }
        return;
    }

    m_moveMousePos.recalculate(gMousePos, m_pWidget->frameGeometry());

    if (m_moveMousePos.m_bOnTopLeftEdge || m_moveMousePos.m_bOnBottomRightEdge) {
        m_pWidget->setCursor(Qt::SizeFDiagCursor);
        m_bCursorShapeChanged = true;
    }else if (m_moveMousePos.m_bOnTopRightEdge || m_moveMousePos.m_bOnBottomLeftEdge) {
        m_pWidget->setCursor(Qt::SizeBDiagCursor);
        m_bCursorShapeChanged = true;
    }else if (m_moveMousePos.m_bOnLeftEdge || m_moveMousePos.m_bOnRightEdge) {
        m_pWidget->setCursor(Qt::SizeHorCursor);
        m_bCursorShapeChanged = true;
    }else if (m_moveMousePos.m_bOnTopEdge || m_moveMousePos.m_bOnBottomEdge) {
        m_pWidget->setCursor(Qt::SizeVerCursor);
        m_bCursorShapeChanged = true;
    }else {
        if (m_bCursorShapeChanged) {
            m_pWidget->unsetCursor();
            m_bCursorShapeChanged = false;
        }
    }
}


void WidgetData::resizeWidget(const QPoint& gMousePos)
{
    QRect origRect;

    if (d->m_bRubberBandOnResize) {
        origRect = m_pRubberBand->frameGeometry();
    }else{
        origRect = m_pWidget->frameGeometry();
    }

    int left = origRect.left();
    int top = origRect.top();
    int right = origRect.right();
    int bottom = origRect.bottom();

    origRect.getCoords(&left, &top, &right, &bottom);

    int minWidth = m_pWidget->minimumWidth();
    int minHeight = m_pWidget->minimumHeight();

    if (m_pressedMousePos.m_bOnTopLeftEdge) {
        left = gMousePos.x();
        top = gMousePos.y();
    }else if (m_pressedMousePos.m_bOnBottomLeftEdge) {
        left = gMousePos.x();
        bottom = gMousePos.y();
    }else if (m_pressedMousePos.m_bOnTopRightEdge) {
        right = gMousePos.x();
        top = gMousePos.y();
    }else if (m_pressedMousePos.m_bOnBottomRightEdge) {
        right = gMousePos.x();
        bottom = gMousePos.y();
    }else if (m_pressedMousePos.m_bOnLeftEdge) {
        left = gMousePos.x();
    }else if (m_pressedMousePos.m_bOnRightEdge) {
        right = gMousePos.x();
    }else if (m_pressedMousePos.m_bOnTopEdge) {
        top = gMousePos.y();
    }else if (m_pressedMousePos.m_bOnBottomEdge) {
        bottom = gMousePos.y();
    }

    QRect newRect(QPoint(left, top), QPoint(right, bottom));

    if (newRect.isValid()) {
        if (minWidth > newRect.width()) {
            if (left != origRect.left()) {
                newRect.setLeft(origRect.left());
            }else{
                newRect.setRight(origRect.right());
            }
        }
        if (minHeight > newRect.height()) {
            if (top != origRect.top()) {
                newRect.setTop(origRect.top());
            }else{
                newRect.setBottom(origRect.bottom());
            }
        }

        if (d->m_bRubberBandOnResize) {
            m_pRubberBand->setGeometry(newRect);
        }else {
            m_pWidget->setGeometry(newRect);
        }
    }
}


void WidgetData::moveWidget(const QPoint& gMousePos)
{
    if (d->m_bRubberBandOnMove) {
        m_pRubberBand->move(gMousePos - m_ptDragPos);
    }else {
        m_pWidget->move(gMousePos - m_ptDragPos);
    }
}


bool WidgetData::handleMousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_bLeftButtonPressed = true;
        m_bLeftButtonTitlePressed = event->pos().y() < m_moveMousePos.m_nTitleHeight;

        QRect frameRect = m_pWidget->frameGeometry();
        m_pressedMousePos.recalculate(event->globalPos(), frameRect);

        m_ptDragPos = event->globalPos() - frameRect.topLeft();

        if (m_pressedMousePos.m_bOnEdges) {
            if (m_pWidget->height() == currentWidgetHeight) {
                //窗口在最大化状态时，点击边界不做任何处理
                return (false);
            }
            if (d->m_bRubberBandOnResize) {
                m_pRubberBand->setGeometry(frameRect);
                m_pRubberBand->show();
                return (true);
            }
        }else if (d->m_bRubberBandOnMove && m_bLeftButtonTitlePressed) {
            if (m_pWidget->height() == currentWidgetHeight) {
                //窗口在最大化状态时，点击标题栏不做任何处理
                return (false);
            }
            m_pRubberBand->setGeometry(frameRect);
            m_pRubberBand->show();
            return (true);
        }
    }
    return (false);
}


bool WidgetData::handleMouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_bLeftButtonPressed = false;
        m_bLeftButtonTitlePressed = false;
        m_pressedMousePos.reset();
        if (m_pRubberBand && m_pRubberBand->isVisible()) {
            m_pRubberBand->hide();
            m_pWidget->setGeometry(m_pRubberBand->geometry());
            return (true);
        }
    }
    return (false);
}


bool WidgetData::handleMouseMoveEvent(QMouseEvent *event)
{
    if (m_bLeftButtonPressed) {
        if (d->m_bWidgetResizable && m_pressedMousePos.m_bOnEdges) {
            if (m_pWidget->height() == currentWidgetHeight) {
                //窗口在最大化状态时，点击边界不做任何处理
                return (false);
            }
            resizeWidget(event->globalPos());
            return (true);
        }else if (d->m_bWidgetMovable && m_bLeftButtonTitlePressed) {
            if (m_pWidget->height() == currentWidgetHeight) {
                //[!]移动窗口的时候，设置窗口为Normal大小
                QRect normalGeometry = m_pWidget->normalGeometry();
                int screenIndex = QApplication::desktop()->screenNumber(m_pWidget);
                if (screenIndex < 0 || screenIndex >= QGuiApplication::screens().size()) {
                    qDebug() << "Error: Invalid screen index: " << screenIndex;
                    return false;
                }
                QScreen *screen = QGuiApplication::screens().at(screenIndex);
                QRect availableGeometry = screen->availableGeometry();
                int width = availableGeometry.width() * 0.75;
                int height = availableGeometry.height() * 0.75;
                int x = (availableGeometry.width() - width) / 2 + availableGeometry.x();
                int y = (availableGeometry.height() - height) / 2 + availableGeometry.y();
                m_pWidget->setGeometry(x, y, width, height);
                m_Rect = m_pWidget->geometry();
                m_pWidget->setWindowState(Qt::WindowNoState);
                emit d->m_dptr->signalSetWindowGroupButtonState(true);
                //先求出窗口到鼠标的相对位置
                QPoint p = event->globalPos();
                p.ry() -= 10;
                p.rx() -= (normalGeometry.width() / 2);
                m_pWidget->move(p);
                //这时要重置m_ptDragPos
                m_ptDragPos = QPoint(normalGeometry.width()/2, 10);
                return (true);
            }
            moveWidget(event->globalPos());
            m_Rect = m_pWidget->geometry();
            return (true);
        }
        return (false);
    }else if (d->m_bWidgetResizable) {
        updateCursorShape(event->globalPos());
    }
    return (false);
}


bool WidgetData::handleLeaveEvent(QEvent *event)
{
    Q_UNUSED(event)
    if (!m_bLeftButtonPressed) {
        m_pWidget->unsetCursor();
        return (true);
    }
    return (false);
}


bool WidgetData::handleHoverMoveEvent(QHoverEvent *event)
{
    if (d->m_bWidgetResizable) {
        updateCursorShape(m_pWidget->mapToGlobal(event->pos()));
    }
    return (false);
}


bool WidgetData::handleDoubleClickedMouseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (m_pWidget) {
            SARibbonMainWindow *mainwindow = qobject_cast<SARibbonMainWindow *>(m_pWidget);
            if (mainwindow) {
                if (mainwindow->windowButtonFlags() & Qt::WindowMaximizeButtonHint) {
                    bool titlePressed = event->pos().y() < m_moveMousePos.m_nTitleHeight;
                    if (titlePressed) {
                        int screenIndex = QApplication::desktop()->screenNumber(m_pWidget);
                        if (screenIndex < 0 || screenIndex >= QGuiApplication::screens().size()) {
                            qDebug() << "Error: Invalid screen index: " << screenIndex;
                            return false;
                        }
                        QScreen *screen = QGuiApplication::screens().at(screenIndex);
                        QRect availableGeometry = screen->availableGeometry();
                        if (m_pWidget->height() == availableGeometry.height()){
                            int width = availableGeometry.width() * 0.75;
                            int height = availableGeometry.height() * 0.75;
                            int x = (availableGeometry.width() - width) / 2 + availableGeometry.x();
                            int y = (availableGeometry.height() - height) / 2 + availableGeometry.y();
                            m_pWidget->setGeometry(x, y, width, height);
                            emit d->m_dptr->signalSetWindowGroupButtonState(true);
                        }else {
                            m_pWidget->setGeometry(availableGeometry);
                            emit d->m_dptr->signalSetWindowGroupButtonState(false);

                        }
                        return (true);
                    }
                }
            }
        }
    }
    return (false);
}
uint WidgetData::getWigetHeight()
{
    return uint(currentWidgetHeight);
}
void WidgetData::slotScreenChange()
{

    QTimer::singleShot(1000, this, [=]() {
        QList<QScreen *> connected_screens = QGuiApplication::screens();
        if (connected_screens.size() < 1) {
            qDebug() << "not find screen";
            return;
        }
        QRect rect;
        for (int i = 0; i < connected_screens.size(); i++) {
            rect = connected_screens.at(i)->availableGeometry();
        }
        int width = m_Rect.width();
        int heigt = m_Rect.height();
        m_pWidget->move(rect.x(), rect.y());
        m_Rect.setX(m_pWidget->geometry().x());
        m_Rect.setY(m_pWidget->geometry().y());
        m_Rect.setWidth(width);
        m_Rect.setHeight(heigt);
        m_pWidget->setWindowState(Qt::WindowMinimized);
        m_pWidget->adjustSize();
    });

}
//[!]修改设置最大化时策略，修改主窗口的窗口状态
bool WidgetData::handWindowStateChange(QEvent *event)
{
    if ((m_pWidget->windowState() & Qt::WindowMaximized) || (m_pWidget->windowState() & Qt::WindowFullScreen))
    {
        int screenIndex = QApplication::desktop()->screenNumber(m_pWidget); 
        if (screenIndex < 0 || screenIndex >= QGuiApplication::screens().size()) {
            qDebug() << "Error: Invalid screen index: " << screenIndex;
            return false;
        }
        QScreen *screen = QGuiApplication::screens().at(screenIndex);
        QRect availableGeometry = screen->availableGeometry();
        availableGeometry.setTop(availableGeometry.top() - 1);
        QSize screenSize = availableGeometry.size();
        screenSize.setHeight(availableGeometry.height() -1);
        m_pWidget->setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, screenSize, availableGeometry));
        currentWidgetHeight = screenSize.height();
        m_Rect = m_pWidget->geometry();
        emit d->m_dptr->signalSetWindowGroupButtonState(false);
        return true;
    }
    else if (m_pWidget->windowState() & Qt::WindowMinimized)
    {
        emit d->m_dptr->signalSetWindowGroupButtonState(true);
        m_Rect = m_pWidget->geometry();
        return false;
    }
    else
    {
        if(!m_Rect.isNull())
        m_pWidget->setGeometry(m_Rect);
        emit d->m_dptr->signalSetWindowGroupButtonState(true);
        return true;
    }

}

SAFramelessHelper::SAFramelessHelper(QObject *parent)
    : QObject(parent),
    d(new FramelessHelperPrivate(this))
{
    d->m_bWidgetMovable = true;
    d->m_bWidgetResizable = true;
    d->m_bRubberBandOnResize = false;
    d->m_bRubberBandOnMove = false;
    if (parent) {
        QWidget *w = qobject_cast<QWidget *>(parent);
        if (w) {
            w->setWindowFlags(w->windowFlags()|Qt::FramelessWindowHint);
            setWidgetMovable(true);         //设置窗体可移动
            setWidgetResizable(true);       //设置窗体可缩放
            setRubberBandOnMove(false);     //设置橡皮筋效果-可移动
            setRubberBandOnResize(true);    //设置橡皮筋效果-可缩放
            activateOn(w);                  //激活当前窗体
        }
    }
}


SAFramelessHelper::~SAFramelessHelper()
{
    QList<QWidget *> keys = d->m_widgetDataHash.keys();
    int size = keys.size();

    for (int i = 0; i < size; ++i)
    {
        delete d->m_widgetDataHash.take(keys[i]);
    }

    delete d;
}


bool SAFramelessHelper::eventFilter(QObject *obj, QEvent *event)
{
    switch (event->type())
    {
    case QEvent::MouseMove:
    case QEvent::HoverMove:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::Leave:
    {
        WidgetData *data = d->m_widgetDataHash.value(static_cast<QWidget *>(obj));
        if (data) {
            return (data->handleWidgetEvent(event));
        }
        break;
    }
    case  QEvent::WindowStateChange:
    {
        WidgetData *data = d->m_widgetDataHash.value(static_cast<QWidget *>(obj));
        if (data) {
            return (data->handleWidgetEvent(event));
        }
        break;
    }

    default:
        break;
    }
    return (QObject::eventFilter(obj, event));
}


void SAFramelessHelper::activateOn(QWidget *topLevelWidget)
{
    if (!d->m_widgetDataHash.contains(topLevelWidget)) {
        WidgetData *data = new WidgetData(d, topLevelWidget);
        d->m_widgetDataHash.insert(topLevelWidget, data);

        topLevelWidget->installEventFilter(this);
    }
}


void SAFramelessHelper::removeFrom(QWidget *topLevelWidget)
{
    WidgetData *data = d->m_widgetDataHash.take(topLevelWidget);

    if (data) {
        topLevelWidget->removeEventFilter(this);
        delete data;
    }
}


void SAFramelessHelper::setRubberBandOnMove(bool movable)
{
    d->m_bRubberBandOnMove = movable;
    QList<WidgetData *> list = d->m_widgetDataHash.values();

    foreach(WidgetData *data, list)
    {
        data->updateRubberBandStatus();
    }
}


void SAFramelessHelper::setWidgetMovable(bool movable)
{
    d->m_bWidgetMovable = movable;
}


void SAFramelessHelper::setWidgetResizable(bool resizable)
{
    d->m_bWidgetResizable = resizable;
}


void SAFramelessHelper::setRubberBandOnResize(bool resizable)
{
    d->m_bRubberBandOnResize = resizable;
    QList<WidgetData *> list = d->m_widgetDataHash.values();

    foreach(WidgetData *data, list)
    {
        data->updateRubberBandStatus();
    }
}


void SAFramelessHelper::setBorderWidth(int width)
{
    if (width > 0) {
        CursorPosCalculator::m_nBorderWidth = width;
    }
}

void SAFramelessHelper::setTitleHeight(int height)
{
    if (height > 0) {
        CursorPosCalculator::m_nTitleHeight = height;
    }
}


bool SAFramelessHelper::widgetMovable()
{
    return (d->m_bWidgetMovable);
}


bool SAFramelessHelper::widgetResizable()
{
    return (d->m_bWidgetResizable);
}


bool SAFramelessHelper::rubberBandOnMove()
{
    return (d->m_bRubberBandOnMove);
}


bool SAFramelessHelper::rubberBandOnResisze()
{
    return (d->m_bRubberBandOnResize);
}


uint SAFramelessHelper::borderWidth()
{
    return (CursorPosCalculator::m_nBorderWidth);
}


uint SAFramelessHelper::titleHeight()
{
    return (CursorPosCalculator::m_nTitleHeight);
}

#include "SAFramelessHelper.moc"