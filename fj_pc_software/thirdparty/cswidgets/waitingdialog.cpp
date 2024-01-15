#include "waitingdialog.h"
using namespace CS::Widgets;
#include "csutils/qtcassert.h"
using namespace Utils;
#include "frameline.h"
#include <QEvent>
#include <QPainter>
#include <QPixmap>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDebug>
#include <QtMath>
#include <mutex>
#include <QApplication>

static std::once_flag    SingletonFlag;
WaitingDialog    *WaitingDialog::m_pInstance = nullptr;//类外初始化-单例使用

QColor WaitingDialog::DEFAULT_BORDER_COLOR = QColor(0x22, 0x21, 0x26);// 默认边框颜色
QColor WaitingDialog::DEFAULT_SHADOW_COLOR = Qt::black;
QColor WaitingDialog::DEFAULT_BACKGROUND = QColor(0x38, 0x3a, 0x3f);

WaitingDialog* WaitingDialog::instance(QWidget *parent, Qt::WindowFlags f)
{
    std::call_once(SingletonFlag, [&parent, &f]()
    {
        m_pInstance = new WaitingDialog(parent, f);
    });
    return m_pInstance;
}

void WaitingDialog::deleteInstance()
{
    if (m_pInstance != nullptr)
    {
        delete m_pInstance;
        m_pInstance = nullptr;
    }
}

WaitingDialog::WaitingDialog(QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f)
    , m_rotation(0)
{
    setWindowFlags(Qt::FramelessWindowHint | windowFlags());
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_NoSystemBackground);
    setProperty("isWaitingDialog", true);
    setModal(true);
    resize(270, 160);
    m_timer.setSingleShot(false);
    //connect(&m_timer, &QTimer::timeout, this, &WaitingDialog::step);
    setIndicatorSize(WaitingDialog::Large);
    QVBoxLayout *pMainLayout = new QVBoxLayout;
    pMainLayout->setSpacing(12);

    pMainLayout->setContentsMargins(15, 20, 15, 20);
	m_pMovie = new QMovie(":/utils/images/wait.gif");
	m_pMovie->setSpeed(120);
	m_pMovie->setScaledSize(QSize(100, 100));
	m_pWaitMoveLabel = new QLabel();
	m_pWaitMoveLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pWaitMoveLabel->setFixedSize(QSize(100, 100));
	pMainLayout->addWidget(m_pWaitMoveLabel, 0, Qt::AlignCenter);
	m_pWaitMoveLabel->setMovie(m_pMovie);
	
    pMainLayout->addStretch();
    m_pInformativeLabel = new QLabel(this);
    m_pInformativeLabel->setAlignment(Qt::AlignCenter);
    m_pInformativeLabel->setWordWrap(true);

	pMainLayout->addWidget(m_pInformativeLabel);
    setLayout(pMainLayout);
}
WaitingDialog::~WaitingDialog()
{
    hideWaitingDialog();
}
static QString imageFileNameForIndicatorSize(WaitingDialog::IndicatorSize size)
{
    switch (size) {
    case WaitingDialog::Large:
        return QLatin1String(":/utils/images/progressindicator_big.png");
    case WaitingDialog::Medium:
        return QLatin1String(":/utils/images/progressindicator_medium.png");
    case WaitingDialog::Small:
    default:
        return QLatin1String(":/utils/images/progressindicator_small.png");
    }
}

void WaitingDialog::setIndicatorSize(WaitingDialog::IndicatorSize size)
{
    m_size = size;
    m_rotationStep = size == Small ? 45 : 30;
    m_timer.setInterval(size == Small ? 100 : 80);
    m_pixmap.load(imageFileNameForIndicatorSize(size));
    updateGeometry();
    update();
}

WaitingDialog::IndicatorSize WaitingDialog::indicatorSize() const
{
    return m_size;
}
QSize WaitingDialog::sizeHint() const
{
    return m_pixmap.size() / m_pixmap.devicePixelRatio();
}

void WaitingDialog::attachToWidget(QWidget *parent)
{
    if (parentWidget())
        parentWidget()->removeEventFilter(this);

    setParent(parent);
    resizeToParent();

    if (parent)
    {
        parent->installEventFilter(this);

    }

    raise();
}

void WaitingDialog::showWaitingDialog(const QString & text, TooltipLevel level)
{
    QColor textColor = Qt::white;
    if (WaitingDialog::Warning == level)
    {
        textColor = Qt::yellow;
    }
    else if (WaitingDialog::Error == level)
    {
        textColor = Qt::red;
    }
	m_pMovie->start();
    QPalette palette = m_pInformativeLabel->palette();
    palette.setColor(QPalette::Foreground, textColor);
    m_pInformativeLabel->setPalette(palette);
    m_pInformativeLabel->setText(QString::null);
    m_pInformativeLabel->update();

    if (!m_bIsSetParent)
        setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);

    if (!text.isEmpty())
    {
        setInformativeText(text);
    }
	
	if (isHidden())
		show();
}

void WaitingDialog::hideWaitingDialog()
{
	m_pMovie->stop();
    m_pInformativeLabel->setText("");
    m_pInformativeLabel->update();
    if (!isHidden())
        hide();
}

void WaitingDialog::hide()
{
    QDialog::hide();
}

void WaitingDialog::paintEvent(QPaintEvent *)
{
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    path.addRect(10, 10, this->width() - 20, this->height() - 20);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillPath(path, QBrush(QColor(45, 45, 48)));

    QColor color(0, 0, 0, 35);
    for (int i = 0; i < 10; i++)
    {
        QPainterPath path;
        path.setFillRule(Qt::WindingFill);
        path.addRect(10 - i, 10 - i, this->width() - (10 - i) * 2, height() - (10 - i) * 2);
        color.setAlpha(150 - qSqrt(i) * 50);
        painter.setPen(color);
        painter.drawPath(path);
    }


	//QPainter painter(this);
	//painter.setRenderHint(QPainter::Antialiasing, true);
	//QColor usedColor(98, 177, 255);

	//painter.setPen(QPen(usedColor, 5));
	//QRectF rectangle(4, 4, this->width() - 8, this->height() - 8);
	//int startAngle = 90 * 16;     //起始角度
	//int spanAngle = m_currentValue * 16;   //跨越度数

	//painter.drawArc(rectangle, startAngle, -spanAngle);


   /* QPainter p(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.setBrush(QBrush(QColor(45, 45, 48)));
    int height = rect().height();
    QPoint translate(rect().width() / 2, height / 2);
    QTransform t;
    t.translate(translate.x(), translate.y());
    t.rotate(m_rotation);
    t.translate(-translate.x(), -translate.y());
    p.setTransform(t);
    QSize pixmapUserSize(m_pixmap.size() / m_pixmap.devicePixelRatio());
    p.drawPixmap(QPoint((rect().width() - pixmapUserSize.width()) / 2,
        (height - pixmapUserSize.height()) / 2),
        m_pixmap);*/
}

void WaitingDialog::showEvent(QShowEvent *)
{
    m_timer.start();
    moveToCenter();
    raise();
    activateWindow();
}

void WaitingDialog::hideEvent(QHideEvent *e)
{
    m_timer.stop();
}

bool WaitingDialog::eventFilter(QObject *obj, QEvent *ev)
{
    if (obj == parent() && ev->type() == QEvent::Resize) {
        resizeToParent();
    }

    if (obj != this) {

        return QWidget::eventFilter(obj, ev);
    }

    QEvent::Type type = ev->type();
    switch (type)
    {
    case QEvent::KeyPress:
    {
        int key_type = static_cast<QKeyEvent*>(ev)->key();
        if (key_type == Qt::Key_Alt)

        m_bAltKeyPressed = true;
        break;
    }
    case QEvent::KeyRelease:
    {
 
        int key_type = static_cast<QKeyEvent*>(ev)->key();
        if (key_type == Qt::Key_Alt)
            m_bAltKeyPressed = false;
        break;
    }
    case QEvent::Close:
    {
     
        ev->ignore();
        return true;

    }
    break;
    default:
        break;
    }

    return QWidget::eventFilter(obj, ev);
}

void WaitingDialog::keyPressEvent(QKeyEvent *event)
{
    qDebug() << Q_FUNC_INFO << "keyPress :" << event->key();
    switch (event->key())
    {
    case Qt::Key_Escape:
        break;

    case Qt::Key_Alt:
        m_bAltKeyPressed = true;
        break;
    default:
        QDialog::keyPressEvent(event);
    }
}

void WaitingDialog::keyReleaseEvent(QKeyEvent *event)
{
    qDebug() << Q_FUNC_INFO << "keyRelease :" << event->key();
    switch (event->key())
    {
    case Qt::Key_Escape:
        break;

    case Qt::Key_Alt:
        m_bAltKeyPressed = false;
        break;
    default:
        QDialog::keyReleaseEvent(event);
    }

}
void WaitingDialog::step()
{
    m_rotation = (m_rotation + m_rotationStep + 360) % 360;
    update();
}

void WaitingDialog::resizeToParent()
{
    QTC_ASSERT(parentWidget(), return);
    moveToCenter();
}

void WaitingDialog::moveToCenter()
{
    if (!this->parentWidget())
    {
        qDebug() << Q_FUNC_INFO << "parentWidget is null!";
        return;
    }

    QPoint pt = QPoint(this->parentWidget()->geometry().x() +
        this->parentWidget()->width() / 2 -
        this->width() / 2,
        this->parentWidget()->geometry().y() +
        this->parentWidget()->height() / 2 -
        this->height() / 2);

    QPoint thisPoint;
    if (m_bIsSetParent)
    {
        thisPoint = this->parentWidget()->mapFromParent(pt);
    }
    else
    {
        thisPoint = pt;
    }
    setGeometry(QRect(thisPoint.x(), thisPoint.y(), this->width(), this->height()));
}

void WaitingDialog::setInformativeText(const QString &text, TooltipLevel level)
{
    QColor textColor = Qt::white;
    if (WaitingDialog::Warning == level)
    {
        textColor = Qt::yellow;
    }
    else if (WaitingDialog::Error == level)
    {
        textColor = Qt::red;
    }

    QPalette palette = m_pInformativeLabel->palette();
    palette.setColor(QPalette::Foreground, textColor);
    m_pInformativeLabel->setPalette(palette);

    m_pInformativeLabel->setText(text);
    m_pInformativeLabel->update();
}

void WaitingDialog::setButtonText(const QString &text)
{
  
}

void WaitingDialog::setParentChanged(bool bChanged)
{
    m_bIsSetParent = bChanged;
}

QString WaitingDialog::informativeText() const
{
    return m_pInformativeLabel->text();
}

void WaitingDialog::closeEvent(QCloseEvent * event)
{
    qDebug() << Q_FUNC_INFO << "m_bAltKeyPressed " << m_bAltKeyPressed;
    event->ignore();
    return;
}



void WaitingDialog::changeEvent(QEvent * event)
{
    if (event->type() != QEvent::WindowStateChange) return;

    if (this->windowState() == Qt::WindowNoState) {
    }
    else if (this->windowState() == Qt::WindowMinimized) {
    }
    return;
}

