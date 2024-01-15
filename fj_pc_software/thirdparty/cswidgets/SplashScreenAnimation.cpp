#include "SplashScreenAnimation.h"
#include <QDesktopWidget>
#include <QApplication>
#include <QPainter>
#include <QPixmap>
#include <QtMath>

using namespace CS;
using namespace CS::Widgets;



SplashScreenAnimation *CS::Widgets::SplashScreenAnimation::m_instance = nullptr;
namespace CS {
	namespace Widgets {
		class SplashScreenAnimationPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit SplashScreenAnimationPrivate(SplashScreenAnimation *ptr);
			virtual ~SplashScreenAnimationPrivate();

		private:
			/**
			*@brief 创建UI
			*/
			void createWidget(void);
			/**
			*@brief 创建连接
			*/
			void createConnect(void);


		private:
			friend class CS::Widgets::SplashScreenAnimation;
			CS::Widgets::SplashScreenAnimation *m_pQptr = nullptr;

		};
		
	}
}

SplashScreenAnimationPrivate::SplashScreenAnimationPrivate(SplashScreenAnimation *ptr) 
{
	m_pQptr = ptr;
}

SplashScreenAnimationPrivate::~SplashScreenAnimationPrivate()
{
}

void CS::Widgets::SplashScreenAnimationPrivate::createWidget(void)
{
	m_pQptr->setWindowFlags(Qt::FramelessWindowHint | m_pQptr->windowFlags());
	m_pQptr->setFixedSize(QSize(603, 379));

}


void CS::Widgets::SplashScreenAnimationPrivate::createConnect(void)
{
}


SplashScreenAnimation::SplashScreenAnimation(QPixmap pixmap)
	: QSplashScreen(pixmap)
	,m_pDptr(new SplashScreenAnimationPrivate(this))

{

	m_pDptr->createWidget();
	m_pDptr->createConnect();

	/*m_timer.setInterval(50);
	m_nCount = 0;
	connect(&m_timer, &QTimer::timeout, this, [=] {
		if (m_nCount > 30) {
			m_nCount = 0;
		}
		QString str;
		for (int i = 0; i < m_nCount; i++) {
			str += "....";
		}
		m_strDisplay = m_strInfo + str;
		m_nCount++;
		repaint();
		QApplication::processEvents();

	});*/
	//m_timer.start();


}

CS::Widgets::SplashScreenAnimation::~SplashScreenAnimation()
{
}

SplashScreenAnimation * CS::Widgets::SplashScreenAnimation::instance(void)
{
	if (!m_instance) {
		QPixmap pixmap;
		m_instance = new SplashScreenAnimation(pixmap);
		
	}
	return m_instance;
}

void CS::Widgets::SplashScreenAnimation::setBackgroundPixmap(QPixmap pixmap)
{
	m_Pixmap = pixmap;
}

void CS::Widgets::SplashScreenAnimation::setBackgroundPixmap(const QString & strPath)
{
	m_Pixmap.load(strPath);
	return;
}

void CS::Widgets::SplashScreenAnimation::showMessageInformation(const QString & strMessage)
{
	m_strInfo = tr("%1, Please wait...").arg(strMessage); 
	repaint();
}


void CS::Widgets::SplashScreenAnimation::showEvent(QShowEvent *event)
{
	QSplashScreen::showEvent(event);
	QDesktopWidget* desktop = QApplication::desktop(); 
	move((desktop->width() - this->width()) / 2, (desktop->height() - this->height()) / 2);
	return;
}

void CS::Widgets::SplashScreenAnimation::paintEvent(QPaintEvent *event)
{
	QPainterPath path;
	path.setFillRule(Qt::WindingFill);
	path.addRect(1, 1, this->width() - 2, this->height() - 2);
	QColor border = qRgb(0, 0, 0);
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.fillPath(path, QBrush(QColor(border.red(),
		border.green(),
		border.blue())));
	painter.save();
	QPen pen;
	pen.setWidth(2);
	pen.setColor(QColor(0,
		0,
		0));

	painter.setPen(pen);
	painter.drawRect(0, 0, this->width(), this->height());
	painter.restore();

	//[!].话背景
	QPainter p(this);
	p.drawPixmap(1, 1, this->width() - 2, this->height() - 2, m_Pixmap);

	//[!].写信息
	QPainter text(this);
	QFont font("Arial", 10); 
	text.setFont(font);
	QRectF rect(1, this->height() - 20, this->width(), 20);
	text.setPen(QColor(Qt::green));
	text.drawText(rect, Qt::AlignLeft | Qt::AlignBottom, m_strInfo);
	return;

}

#include "SplashScreenAnimation.moc"
