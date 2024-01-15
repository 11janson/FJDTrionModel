#include "imagepreview.h"
#include <QMenu>
#include <QContextMenuEvent>
#include <QStyleOption>
#include <QPainter>
#include <QFileDialog>
#include <QToolButton>
#include <QHBoxLayout>

#include "cloudcompareutils/icore.h"
#include "cswidgets/titlebar.h"


using namespace CS;
using namespace CS::MetahubWidgets;
using namespace CS::Core;

using namespace CS::Widgets;
using namespace CS::MetahubWidgets;


QColor m_defaultBackgroundColor = qRgb(40, 42, 43);
QColor m_defaultBorderColor = qRgb(56, 56, 56);
QColor m_defaultSpecialBackgroungColor = qRgb(34, 33, 38);
QColor m_defaultNormalBackgroungColor = qRgb(56, 58, 63);

ImageView::ImageView(QWidget *parent
    , Qt::WindowFlags f)
    :MetahubFramelessDialog(parent)
{
    setAttribute(Qt::WA_DeleteOnClose, true);
    bottomWidget()->setVisible(false);
  

    QString strStyleSheet = CS::Core::ICore::themeStyleSheet();
    QFile styleSheet(strStyleSheet);
    if (styleSheet.open(QIODevice::ReadOnly)) {
        getTitleBar()->GetButtonByType(TitleBar::PredefinedButtonType::CloseButton)->setStyleSheet(styleSheet.readAll());
    }

    this->setMouseTracking(true);
    QHBoxLayout *pMainHLayout = new QHBoxLayout();
    pMainHLayout->setContentsMargins(20, 0, 20, 0);
    m_pNextButton = new QPushButton();
    m_pNextButton->setStyleSheet("QPushButton{background-color:transparent;border:none}");
    m_pNextButton->setFixedSize(QSize(100, 100));
    m_pNextButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPreviousButton = new QPushButton();
    m_pPreviousButton->setFixedSize(QSize(100, 100));
    m_pPreviousButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPreviousButton->setStyleSheet("QPushButton{background-color:transparent;border:none}");
    pMainHLayout->addWidget(m_pPreviousButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
    pMainHLayout->addStretch();
    pMainHLayout->addWidget(m_pNextButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
    getContentHolder()->setLayout(pMainHLayout);

    setAttribute(Qt::WA_TranslucentBackground);
    //[!].禁止窗口移动
    getFramelessHelper()->setWidgetMovable(false);

    connect(m_pNextButton, &QPushButton::clicked,
        this, &ImageView::slotNextButton);

    connect(m_pPreviousButton, &QPushButton::clicked,
        this, &ImageView::slotPreviousButton);


    //[!].设置获取背景图片
    m_pNextButton->setIconSize(QSize(60, 60));
    m_pNextButton->setIcon(Core::ICore::resourceThemeImage("next.png"));

    m_pPreviousButton->setIconSize(QSize(60, 60));
    m_pPreviousButton->setIcon(Core::ICore::resourceThemeImage("before.png"));

   
    //[!].设置关闭按钮属性
    TitleBar    *pTitleBar;
    pTitleBar = this->getTitleBar();
    pTitleBar->setFixedHeight(70);
    pTitleBar->setStyleSheet("background-color:transparent;");
    pTitleBar->GetButtonByType(4)->setFixedSize(48, 48);
    pTitleBar->GetButtonByType(4)->setIconSize(QSize(48, 48));
    pTitleBar->GetButtonByType(4)->setStyleSheet("background-color:transparent;border:none");
    pTitleBar->GetButtonByType(4)->setIcon(Core::ICore::resourceThemeImage("roundclosebutton.png"));
    setAttribute(Qt::WA_TranslucentBackground);
    this->getTilteBarLayout()->setContentsMargins(0, 32, 45, 0);
    

    return;
}

ImageView::~ImageView()
{

}



void CS::MetahubWidgets::ImageView::loadImage(const QImage & image)
{
    m_Image = image;
}

void CS::MetahubWidgets::ImageView::setImageList(const std::vector<QImage> listImage)
{
    m_listImageList = listImage;
}


void CS::MetahubWidgets::ImageView::setActivateIndex(const int &index)
{
    m_nAcitveIndex = index;
}

void CS::MetahubWidgets::ImageView::slotNextButton(void)
{
    m_nAcitveIndex++;
    updateUIFromModel();
}
void CS::MetahubWidgets::ImageView::slotPreviousButton(void)
{
    m_nAcitveIndex--;
    updateUIFromModel();
}

void CS::MetahubWidgets::ImageView::updateUIFromModel(void)
{
    m_pNextButton->setEnabled(true);
    m_pPreviousButton->setEnabled(true);
    if (m_listImageList.empty()){
      
        return;
    }

    if (m_nAcitveIndex >= m_listImageList.size()) {
        m_nAcitveIndex = m_listImageList.size();
        m_pNextButton->setEnabled(false);
    }else if (m_nAcitveIndex <= 0)
    {
        m_pPreviousButton->setEnabled(false);
        m_nAcitveIndex = 1;
    }
    
    m_Image = m_listImageList.at(m_nAcitveIndex - 1);
    update();
    return;
}

void ImageView::paintEvent(QPaintEvent *event)
{
	// 绘制样式
	QStyleOption opt;
	opt.init(this);
	QPainter painter(this);
	style()->drawPrimitive(QStyle::PE_Widget, &opt, &painter, this);


    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    path.addRect(1, 1, this->width() - 2, this->height() - 2);

    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillPath(path, QBrush(QColor(m_defaultBackgroundColor.red(),
        m_defaultBackgroundColor.green(),
        m_defaultBackgroundColor.blue(), 150)));
    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setColor(QColor(m_defaultBorderColor.red(),
        m_defaultBorderColor.green(),
        m_defaultBorderColor.blue()));

    painter.setPen(pen);
    painter.drawRect(0, 0, this->width(), this->height());
    painter.restore();


	if (m_Image.isNull())
		return QWidget::paintEvent(event);

	// 根据窗口计算应该显示的图片的大小
	int width = qMin(m_Image.width(), (this->width()/3 )* 2);
	int height = width * 1.0 / (m_Image.width() * 1.0 / m_Image.height());
	height = qMin(height, this->height());
	width = height * 1.0 * (m_Image.width() * 1.0 / m_Image.height());

	// 平移
	painter.translate(this->width() / 2 + m_XPtInterval, this->height() / 2 + m_YPtInterval);
    
	// 缩放
	painter.scale(m_ZoomValue, m_ZoomValue);

	// 绘制图像
	QRect picRect(-width / 2 + 1, -height / 2 + 1, width - 2, height -2);
	painter.drawImage(picRect, m_Image);
}

void ImageView::wheelEvent(QWheelEvent *event)
{
	int value = event->delta();
	if (value > 0)
		onZoomInImage();
	else
		onZoomOutImage();

	this->update();
}

void ImageView::mousePressEvent(QMouseEvent *event)
{
	m_OldPos = event->pos();
	m_Pressed = true;
}

void ImageView::mouseMoveEvent(QMouseEvent *event)
{
	if (!m_Pressed)
		return QWidget::mouseMoveEvent(event);

	this->setCursor(Qt::SizeAllCursor);
	QPoint pos = event->pos();
	int xPtInterval = pos.x() - m_OldPos.x();
	int yPtInterval = pos.y() - m_OldPos.y();
    int x = this->width() / 2;
    int y = this->height() / 2;
    if (-m_XPtInterval > (this->width() / 2) || m_XPtInterval > (this->width() / 2))
    {
        if (m_XPtInterval < 0)
        {
            m_XPtInterval = -x;
        }
        else
        {
            m_XPtInterval = x;
        }
       return;
    }
     if(-m_YPtInterval > (this->height() / 2) || m_YPtInterval > (this->height() / 2))
    {
         if (m_YPtInterval < 0)
         {
             m_YPtInterval = -y;
         }
         else 
         {
             m_YPtInterval = y;
         }
        return;
    }
    m_XPtInterval += xPtInterval;
    m_YPtInterval += yPtInterval;
	m_OldPos = pos;
	this->update();
}

void ImageView::mouseReleaseEvent(QMouseEvent *event)
{
	m_Pressed = false;
	this->setCursor(Qt::ArrowCursor);
}

void ImageView::onLoadImage(void)
{
	

	
}

void ImageView::onZoomInImage(void)
{
    if (m_ZoomValue >= 2)
    {
        return;
    }
	m_ZoomValue += 0.2;
	this->update();
}

void ImageView::onZoomOutImage(void)
{
	m_ZoomValue -= 0.2;
    if (m_ZoomValue <= 0.2)
    {
        m_ZoomValue += 0.2;
        return;
    }
	this->update();
}

void ImageView::onPresetImage(void)
{
	m_ZoomValue = 1.0;
	m_XPtInterval = 0;
	m_YPtInterval = 0;
	this->update();
}

