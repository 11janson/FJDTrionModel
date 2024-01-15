#include "fjsectionanalysisrenderwidget.h"
#include <QPainter>
#include <QPen>
#include <QColor>
#include "ccGuiParameters.h"
FJSectionAnalysisRenderWidget::FJSectionAnalysisRenderWidget(QWidget *parent)
    : QWidget(parent)
{
	setMouseTracking(true);
    m_image = QImage(800, 607, QImage::Format_RGB32);
    m_image.fill(qRgb(63, 63, 63));
}

FJSectionAnalysisRenderWidget::~FJSectionAnalysisRenderWidget()
{
}

void FJSectionAnalysisRenderWidget::setData(SectionAnalysisData data)
{
	m_data = data;
	initPoisition();
    m_isneedupdateimage = true;
	update();
}

void FJSectionAnalysisRenderWidget::paintEvent(QPaintEvent *event)
{
	QWidget::paintEvent(event);
    std::vector<QColor> colorData{ QColor(255,0,0,255),QColor(0,255,0,255) ,QColor(0,0,255,255) ,QColor(255,255,0,255) ,QColor(0,255,255,255),QColor(255,0,255,255),QColor(192,192,192,255) ,QColor(128,128,128,255) ,QColor(128,0,0,255) ,QColor(128,128,0,255),QColor(0,128,0,255),QColor(128,0,128,255) ,QColor(0,128,128,255) ,QColor(0,0,128,255) };
    if (m_isneedupdateimage)
    {
		bool isautopointsize = false;
		int pointszie = 1;
		for (auto curPointCloud : m_selectedObjectList)
		{
			if (curPointCloud)
			{
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(curPointCloud);
				if (cloud)
				{
					if (cloud->getPointSizeAdaptively())
					{
						isautopointsize = true;
					}
					pointszie = cloud->getPointSize();
				}
			}
		}
		if (pointszie < 1)
		{
			pointszie = 1;
		}
		pointszie++;
        int curwidth = std::max(1.0, std::abs(m_rightBottom.x() - m_leftTop.x()));
        int curheight = std::max(1.0, std::abs(m_rightBottom.y() - m_leftTop.y()));
        m_image = m_image.scaled(curwidth, curheight);
        m_image.fill(qRgb(63, 63, 63));
        QPainter painterimg(&m_image);
        painterimg.setRenderHint(QPainter::HighQualityAntialiasing, true);
        //painterimg.setRenderHint(QPainter::SmoothPixmapTransform, true);
        int currentSectionNum = 0;
        for (auto curPointdata : m_data.Pointdata)
        {
            if (m_showPointColor)
            {
                QPen curpen;
				if (isautopointsize)
				{
					int ppointsize = curwidth / width() /5 + 2;
					if (ppointsize <= 1)
					{
						ppointsize = 2;
					}
					curpen.setWidth(ppointsize);
				}
				else
				{
					curpen.setWidth(pointszie);
				}

                if (currentSectionNum < m_data.PointCloudVisiable.size() && m_data.PointCloudVisiable[currentSectionNum])
                {
                    for (auto curPoint : curPointdata.second)
                    {
                        curpen.setColor(curPoint.first);
                        painterimg.setPen(curpen);
                        QPointF point = getImageScreenPoint(curPoint.second);
                        painterimg.drawPoint(point);
                    }
                }
                currentSectionNum++;
            }
            else
            {
                int colorindex = currentSectionNum % 14;
                QPen curpen(colorData[colorindex]);
				if (isautopointsize)
				{
					int ppointsize = curwidth / width() / 5 +2;
					if (ppointsize <= 1)
					{
						ppointsize = 2;
					}
					curpen.setWidth(ppointsize);
				}
				else
				{
					curpen.setWidth(pointszie);
				}
                painterimg.setPen(curpen);
                if (currentSectionNum < m_data.PointCloudVisiable.size() && m_data.PointCloudVisiable[currentSectionNum])
                {
                    for (auto curPoint : curPointdata.second)
                    {
                        QPointF point = getImageScreenPoint(curPoint.second);
                        painterimg.drawPoint(point);
                    }
                }
                currentSectionNum++;
            }

        }
        m_isneedupdateimage = false;
    }
    QPainter  painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    //painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    if (m_data.Pointdata.size() != 0)
    {
        QRectF rect(m_leftTop.x(), m_leftTop.y(), std::abs(m_rightBottom.x() - m_leftTop.x()), std::abs(m_rightBottom.y() - m_leftTop.y()));
        //m_image = m_image.scaled(std::abs(m_rightBottom.x() - m_leftTop.x()), std::abs(m_rightBottom.y() - m_leftTop.y()));
        painter.drawImage(rect, m_image);
    }
    int currentSectionNum = 0;
    for (auto curPointdata : m_data.Pointdata)
    {
        if (!m_showPointColor)
        {
            int colorindex = currentSectionNum % 14;
            QPen curpen(colorData[colorindex]);
            curpen.setWidth(6);
            painter.setPen(curpen);
            painter.drawEllipse(QPointF(width()*0.66666 -4, 20 + currentSectionNum * 20), 3, 3);
            currentSectionNum++;
        }
    }
	QPen pen(m_data.color);
	pen.setWidth(2);
	pen.setColor(QColor(255, 200, 4));
	painter.setPen(pen);
	if (m_mode != NONEMODE && isMeasureOpen)
	{
		//emit updateMessage(QString("Section:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180) + QString("Length:") + QString::number(abs(m_pointStop.x() - m_pointStart.x())) + QString("Height:") + QString::number(event->y()) + QString::number(abs(m_pointStop.y() - m_pointStart.y())));
		if ((m_pointStop.x()>= m_pointStart.x()) && (m_pointStop.y() > m_pointStart.y()) || (m_pointStop.x() <= m_pointStart.x()) && (m_pointStop.y() < m_pointStart.y()))
		{
			painter.drawLine(getScreenPoint(m_pointStart), getScreenPoint(m_pointStop));
			painter.drawLine(getScreenPoint(m_pointStart), getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())));
			painter.drawLine(getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())), getScreenPoint(m_pointStop));
			QFontMetrics fm(font());
			QRect rect = fm.boundingRect(QString("S:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / abs(m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3));
			double offsetX = 0;
			double offsetY = 0;
			if (m_pointStop.x() >= m_pointStart.x())
			{
				offsetX = -rect.width();
			}
			if (m_pointStop.x() < m_pointStart.x())
			{
				offsetY = 20;
			}
			painter.drawText(QPointF((getScreenPoint(m_pointStart).x() + getScreenPoint(m_pointStop).x()) / 2 + offsetX, (getScreenPoint(m_pointStart).y() + getScreenPoint(m_pointStop).y()) / 2+ offsetY), QString("S:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3));
			offsetX = 0;
			offsetY = 0;
			if (m_pointStop.y() > m_pointStart.y())
			{
				offsetY = 22;
			}
			painter.drawText(QPointF((getScreenPoint(m_pointStart).x() + getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())).x()) / 2 , (getScreenPoint(m_pointStart).y() + getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())).y()) / 2 + offsetY), QString("L:") + QString::number(abs(m_pointStop.x() - m_pointStart.x()), 'f', 3));

			rect = fm.boundingRect(QString("H:") + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
			offsetX = 0;
			offsetY = 0;
			if (m_pointStop.x() < m_pointStart.x())
			{
				offsetX = -rect.width();
			}
			if (m_pointStop.x() == m_pointStart.x() && m_pointStop.y() < m_pointStart.y())
			{
				offsetY = 22;
			}
			if (m_pointStop.x() == m_pointStart.x() && m_pointStop.y() > m_pointStart.y())
			{
				offsetY = -22;
			}
			painter.drawText(QPointF((getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())).x() + getScreenPoint(m_pointStop).x()) / 2 + offsetX, (getScreenPoint(QPointF(m_pointStop.x(), m_pointStart.y())).y() + getScreenPoint(m_pointStop).y()) / 2+ offsetY), QString("H:") + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
		}
		else
		{
			painter.drawLine(getScreenPoint(m_pointStart), getScreenPoint(m_pointStop));
			painter.drawLine(getScreenPoint(m_pointStart), getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())));
			painter.drawLine(getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())), getScreenPoint(m_pointStop));
			QFontMetrics fm(font());
			QRect rect = fm.boundingRect(QString("S:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / abs(m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3));
			double offset = 0;
			double offsetY = 0;
			if (m_pointStop.x() < m_pointStart.x())
			{
				offset = -rect.width();
				offsetY = 22;
			}
			painter.drawText(QPointF((getScreenPoint(m_pointStart).x() + getScreenPoint(m_pointStop).x()) / 2 + offset, (getScreenPoint(m_pointStart).y() + getScreenPoint(m_pointStop).y()) / 2+ offsetY), QString("S:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3));
			rect = fm.boundingRect(QString("H:") + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
			offset = 0;
			offsetY = 0;
			if (m_pointStop.x() > m_pointStart.x())
			{
				offset = -rect.width();
			}
			if (m_pointStop.x() < m_pointStart.x())
			{
				offsetY = 22;
			}

			painter.drawText(QPointF((getScreenPoint(m_pointStart).x() + getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())).x()) / 2 + offset, (getScreenPoint(m_pointStart).y() + getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())).y()) / 2+ offsetY), QString("H:") + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));

			offset = 0;
			offsetY = 0;
			if (m_pointStop.y() < m_pointStart.y())
			{
				offsetY = 22;
			}
			if (m_pointStop.y() == m_pointStart.y() && m_pointStop.x() > m_pointStart.x())
			{
				offsetY = 22;
			}
			painter.drawText(QPointF((getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())).x() + getScreenPoint(m_pointStop).x()) / 2, (getScreenPoint(QPointF(m_pointStart.x(), m_pointStop.y())).y() + getScreenPoint(m_pointStop).y()) / 2 + offsetY), QString("L:") + QString::number(abs(m_pointStop.x() - m_pointStart.x()), 'f', 3));
		}
		
	}
	QPen pen2(Qt::DashLine);
	QVector<qreal> dashes;
	dashes << 20 << 5 ;
	pen2.setDashPattern(dashes);
	pen2.setColor(QColor(75,75,75));
	pen2.setWidth(1);
	painter.setPen(pen2);
	for (int i = 0;i<6;i++)
	{
		painter.drawLine(QPointF(0,height()/7*(i+1)), QPointF(width(), height() / 7 * (i + 1)));
	}
	for (int i = -1; i < 8; i++)
	{
		if (i != -1)
		{
			painter.drawLine(QPointF(width() / 9 * (i + 1), 0), QPointF(width() / 9 * (i + 1), height()));
		}
	}

	QPen pen3;
	pen3.setColor(QColor(177, 177, 177));
	pen3.setWidth(1);
	painter.setPen(pen3);
	for (int i = 0; i < 6; i++)
	{
		if (i != 6 && m_data.Pointdata.size() != 0)
		{
			painter.drawText(QPointF(10, height() / 7 * (i + 1) -5), QString::number(getCloudPoint(QPointF(0, height() / 7.0 * (i + 1))).y(), 'f', 3));
		}
	}
	for (int i = -1; i < 8; i++)
	{
		if (m_data.Pointdata.size() != 0)
		{
			painter.drawText(QPointF(width() / 9 * (i + 1), height() - 15), QString::number(getCloudPoint(QPointF(width() / 9.0 * (i + 1), 0)).x(), 'f', 3));
		}
	}

}

void FJSectionAnalysisRenderWidget::mousePressEvent(QMouseEvent* event)
{
	QWidget::mousePressEvent(event);
	if (m_data.Pointdata.size() == 0)
	{
		return;
	}
	if (event->buttons() & Qt::RightButton)
	{
		m_lastRightMouseClickPoint = QPointF(event->x(), event->y());
		m_leftTopOld = m_leftTop;
		m_rightBottomOld = m_rightBottom;
	}
	if ((event->buttons() & Qt::LeftButton) && isMeasureOpen)
	{
		if (m_mode == NONEMODE || m_mode == SUCCESSMODE)
		{
			m_pointStart = getCloudPoint(QPointF(event->x(), event->y()));
			m_pointStop = getCloudPoint(QPointF(event->x(), event->y()));
			m_mode = RUNNINGMODE;
			update();
		}
		else if (m_mode == RUNNINGMODE)
		{
			QPointF pointStop = getCloudPoint(QPointF(event->x(), event->y()));
			if (pointStop != m_pointStart)
			{
				m_pointStop = getCloudPoint(QPointF(event->x(), event->y()));
				m_mode = SUCCESSMODE;
				emit updateMessage(QString(tr("Slope:")) + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3) + "; " + QString(tr("Length:")) + QString::number(abs(m_pointStop.x() - m_pointStart.x()), 'f', 3) + "; " + QString(tr("Height:")) + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
				emit finshedPickPoint(m_pointStart, m_pointStop);
				update();
			}
		}
	}
}

void FJSectionAnalysisRenderWidget::mouseMoveEvent(QMouseEvent* event)
{
	QWidget::mouseMoveEvent(event);
	if (m_data.Pointdata.size() == 0)
	{
		return;
	}
	QPointF point = getCloudPoint(QPointF(event->x(), event->y()));
	if (!isMeasureOpen)
	{
		emit updateMessage(QString("X:") + QString::number(point.x(),'f', 3) + "; " + QString("Y:") + QString::number(point.y(), 'f', 3));
	}
	if (event->buttons() & Qt::RightButton)
	{
		QPointF dmoved = QPointF(event->x() - m_lastRightMouseClickPoint.x(),event->y() - m_lastRightMouseClickPoint.y());
		m_leftTop = dmoved + m_leftTopOld;
		m_rightBottom = dmoved + m_rightBottomOld;
		update();
	}
	else
	{
		if (m_mode == RUNNINGMODE && isMeasureOpen)
		{
			m_pointStop = getCloudPoint(QPointF(event->x(), event->y())); 
			//emit updateMessage(QString("Section:") + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3) + QString("Length:") + QString::number(abs(m_pointStop.x() - m_pointStart.x()), 'f', 3) + QString("Height:") + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
			update();
		}
	}

}

void FJSectionAnalysisRenderWidget::setPickPoint(QPointF start, QPointF end)
{
	m_pointStart = start;
	m_pointStop = end;
	m_mode = SUCCESSMODE;
}

void FJSectionAnalysisRenderWidget::resizeEvent(QResizeEvent *event)
{
	QWidget::resizeEvent(event);
	double sizeModifierx = double(event->size().width()) / double(event->oldSize().width());
	double sizeModifiery = double(event->size().height()) / double(event->oldSize().height());
	m_leftTop = QPointF((m_leftTop.x() - double(event->oldSize().width()) / 2.0)*sizeModifierx + double(event->size().width()) / 2.0, (m_leftTop.y() - double(event->oldSize().height()) / 2.0)*sizeModifiery + double(event->size().height()) / 2.0);
	m_rightBottom = QPointF((m_rightBottom.x() - double(event->oldSize().width()) / 2.0)*sizeModifierx + double(event->size().width()) / 2.0, (m_rightBottom.y() - double(event->oldSize().height()) / 2.0)*sizeModifiery + double(event->size().height()) / 2.0);
    m_isneedupdateimage = true;
    update();
}


void FJSectionAnalysisRenderWidget::wheelEvent(QWheelEvent* event)
{
	QWidget::wheelEvent(event);
	if (m_data.Pointdata.size() == 0)
	{
		return;
	}
	int curwidth = std::max(1.0, std::abs(m_rightBottom.x() - m_leftTop.x()));
	int curheight = std::max(1.0, std::abs(m_rightBottom.y() - m_leftTop.y()));
	if (event->delta() > 0 && (curwidth > 50000 || curheight > 50000))
	{
		return;
	}
	double sizeModifier = (event->delta() < 0 ? 0.9 : 1.1);
	QPointF leftTop = QPointF(0,0);
	QPointF rightBottom = QPointF(width(),height());
	double dx = abs(getCloudPoint(leftTop).x() - getCloudPoint(rightBottom).x())*sizeModifier;
	double dy = abs(getCloudPoint(leftTop).y() - getCloudPoint(rightBottom).y())*sizeModifier;
	if ((dx < 0.025*10.0  || dy < 0.025*8) && event->delta() > 0)
	{
		return;
	}
	if ((dx > 10000 * 8 || dy > 10000 * 6) && event->delta() < 0)
	{
		return;
	}

	m_leftTop = QPointF((m_leftTop.x()- event->x())*sizeModifier+ event->x(), (m_leftTop.y() - event->y())*sizeModifier + event->y());
	m_rightBottom = QPointF((m_rightBottom.x() - event->x())*sizeModifier + event->x(), (m_rightBottom.y() - event->y())*sizeModifier + event->y());
    m_isneedupdateimage = true;
	update();
}


void FJSectionAnalysisRenderWidget::mouseReleaseEvent(QMouseEvent* event)
{
	QWidget::mouseReleaseEvent(event);
	m_lastRightMouseClickPoint = QPointF(-1, -1);
}

QPointF FJSectionAnalysisRenderWidget::getCloudPoint(QPointF point)
{
	//return QPointF(point.x()/width()*m_data.xStop, point.y() / height()*(m_data.yStop- m_data.yStart)+ m_data.yStart);
	return QPointF((point.x() - m_leftTop.x()) / (m_rightBottom.x()- m_leftTop.x())*m_data.xStop, (m_rightBottom.y() - point.y()) / (m_rightBottom.y() - m_leftTop.y())*(m_data.yStop - m_data.yStart) + m_data.yStart);
}

QPointF FJSectionAnalysisRenderWidget::getScreenPoint(QPointF point)
{
	//return QPointF(point.x() * width()/m_data.xStop, (point.y() - m_data.yStart) * height()/(m_data.yStop - m_data.yStart));
	return QPointF(point.x() * (m_rightBottom.x() - m_leftTop.x()) / m_data.xStop + m_leftTop.x(), m_rightBottom.y() -(point.y() - m_data.yStart) * (m_rightBottom.y() - m_leftTop.y()) / (m_data.yStop - m_data.yStart));
}

QPointF FJSectionAnalysisRenderWidget::getImageScreenPoint(QPointF point)
{
    return QPointF(point.x() * (m_rightBottom.x() - m_leftTop.x()) / m_data.xStop, (m_rightBottom.y() - m_leftTop.y()) - (point.y() - m_data.yStart) * (m_rightBottom.y() - m_leftTop.y()) / (m_data.yStop - m_data.yStart));
}

void FJSectionAnalysisRenderWidget::startMeasure(bool isopen)
{
	isMeasureOpen = isopen;
	if (m_mode == SUCCESSMODE && isMeasureOpen)
	{
		emit updateMessage(QString(tr("Slope:")) + QString::number(atan((abs(m_pointStop.y() - m_pointStart.y())) / (m_pointStop.x() - m_pointStart.x())) / 3.14157 * 180, 'f', 3) +"; "+ QString(tr("Length:")) + QString::number(abs(m_pointStop.x() - m_pointStart.x()), 'f', 3) + "; " + QString(tr("Height:")) + QString::number(abs(m_pointStop.y() - m_pointStart.y()), 'f', 3));
	}
	else if (isMeasureOpen)
	{
		emit updateMessage(QString(tr("Slope:")) + QString::number(0, 'f', 0) + "; " + QString(tr("Length:")) + QString::number(0, 'f', 0) + "; " + QString(tr("Height:")) + QString::number(0, 'f', 0));
	}
	update();
}

void FJSectionAnalysisRenderWidget::initPoisition()
{
	if (m_data.xStop == 0)
	{
		return;
	}
	if (m_data.yStop == m_data.yStart)
	{
		m_data.yStop += 0.1;
		m_data.yStart -= 0.1;
	}
	//double radio = double(width()) / double(height());
	double dy = m_data.yStop - m_data.yStart;
	double radio = dy / m_data.xStop;
	if (double(width()) * radio < height())
	{
		m_leftTop = QPointF(0, (double(height()) - double(width()) * radio)/2);
		m_rightBottom = QPointF(width(), height() - (double(height()) - double(width()) * radio)/2);
	}
	else
	{
		m_leftTop = QPointF((double(width()) - double(height()) / radio) / 2, 0);
		m_rightBottom = QPointF(width() - (double(width()) - double(height()) / radio) / 2, height());
	}


	//m_leftTop = QPointF(0, 0);
	//m_rightBottom = QPointF(width(), height());
}

void FJSectionAnalysisRenderWidget::setRenderMode(SectionAnalysisRenderMode mode)
{
	m_mode = mode;
	update();
}

SectionAnalysisRenderMode FJSectionAnalysisRenderWidget::getRenderMode()
{
	return m_mode;
}

void FJSectionAnalysisRenderWidget::setPointCloudVisiable(std::vector<bool> visiablelist)
{
    m_data.PointCloudVisiable = visiablelist;
    m_isneedupdateimage = true;
    update();
}

void FJSectionAnalysisRenderWidget::setShowPointColor(bool isshow)
{ 
    m_showPointColor = isshow; 
    m_isneedupdateimage = true;
    update(); 
}

void FJSectionAnalysisRenderWidget::updatePointSize()
{
    m_isneedupdateimage = true;
    update();
}

void  FJSectionAnalysisRenderWidget::setCurrentObjects(ccHObject::Container objs)
{
	m_selectedObjectList = objs;
}