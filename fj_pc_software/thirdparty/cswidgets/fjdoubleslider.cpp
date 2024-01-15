#include "fjdoubleslider.h"
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QRect>
#include "FJStyleManager.h"
FJDoubleSlider::FJDoubleSlider(QWidget *parent)
    : QWidget(parent)
{

}

FJDoubleSlider::~FJDoubleSlider()
{
}


void FJDoubleSlider::mousePressEvent(QMouseEvent* event)
{
    QWidget::mousePressEvent(event);
    if(std::abs(event->y() - height()/2)>5)
    {
        return;
    }
	if (event->buttons() & Qt::LeftButton)
	{
		double LowerPos = (m_Lower - m_Min) / (m_Max - m_Min)*(width() - 10) + 5;
		double UpperPos = (m_Upper - m_Min) / (m_Max - m_Min)*(width() - 10) + 5;
		if (std::abs(event->x() - LowerPos) < 5)
		{
			m_Handle = LowerHandle;
		}
		else if (std::abs(event->x() - UpperPos) < 5)
		{
			m_Handle = UpperHandle;
		}
		else
		{
			m_Handle = NoHandle;
		}
	}

}

void FJDoubleSlider::mouseMoveEvent(QMouseEvent* event)
{
    QWidget::mouseMoveEvent(event);
	if (event->buttons() & Qt::LeftButton)
	{
		if (event->x() < 5 || event->x() > (width()-5) || event->y() < 0 || event->y() > height())
		{
			return;
		}
		if (m_Handle == LowerHandle || m_Handle == UpperHandle)
		{
			if (m_Handle == LowerHandle)
			{
				m_Lower = double(event->x() - 5) / double(width() - 10) * double(m_Max - m_Min) + m_Min;
				if (m_Lower > m_Upper)
				{
					emit upperValueChanged(m_Lower);
				}
				else
				{
					emit lowerValueChanged(m_Lower);
				}
			}
			if (m_Handle == UpperHandle)
			{
				m_Upper = double(event->x() - 5) / double(width() - 10) * double(m_Max - m_Min) + m_Min;
				if (m_Lower > m_Upper)
				{
					emit lowerValueChanged(m_Upper);
				}
				else
				{
					emit upperValueChanged(m_Upper);
				}
			}
			update();
		}
	}
}

void FJDoubleSlider::mouseReleaseEvent(QMouseEvent* event)
{
    QWidget::mouseReleaseEvent(event);
    m_Handle = NoHandle;
}

void FJDoubleSlider::paintEvent(QPaintEvent* event)
{
	updatePos();
    QWidget::paintEvent(event);
    QPainter painter(this);
    QPen pen(QColor(255, 200, 4));
    pen.setWidth(1);
    painter.setPen(pen);
    double minPos = std::min(m_LowerPos,m_UpperPos);
    double maxPos = std::max(m_LowerPos,m_UpperPos);
    QRect rect(minPos,height()/2-2,maxPos-minPos ,4);
    painter.setBrush(QBrush(QColor(255,200,4)));
    painter.drawRect(rect);

    QPen pen2(QColor(130, 130, 130));
    pen2.setWidth(1);
    painter.setPen(pen2);
    painter.setBrush(QBrush(QColor(130, 130,130)));
    painter.drawRect(QRect(0, rect.top(), rect.left(), rect.height()));
    painter.drawRect(QRect(rect.right(), rect.top(), width(), rect.height()));
	painter.drawPixmap(rect.left() - 5, rect.center().y() - 5, QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/movebutton.png"));
	painter.drawPixmap(rect.right() - 8, rect.center().y() - 5, QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/movebutton.png"));
}

void FJDoubleSlider::setLowerValue(double lower)
{
    m_Lower = std::max(lower, m_Min);
    update();
	if (m_Lower > m_Upper)
	{
		emit upperValueChanged(m_Lower);
	}
	else
	{
		emit lowerValueChanged(m_Lower);
	}
}

void FJDoubleSlider::setUpperValue(double upper)
{
	m_Upper = std::min(upper,m_Max);
    update();
	if (m_Lower > m_Upper)
	{
		emit lowerValueChanged(m_Upper);
	}
	else
	{
		emit upperValueChanged(m_Upper);
	}
}

void FJDoubleSlider::setSpan(double lower, double upper)
{
    m_Lower = lower;
    m_Upper = upper;
    update();
	if (m_Lower > m_Upper)
	{
		emit lowerValueChanged(m_Upper);
		emit upperValueChanged(m_Lower);
	}
	else
	{
		emit lowerValueChanged(m_Lower);
		emit upperValueChanged(m_Upper);
	}
}

void FJDoubleSlider::updatePos()
{
    m_LowerPos = (m_Lower - m_Min)/(m_Max - m_Min)*(width() - 10) + 5;
    m_UpperPos = (m_Upper - m_Min)/(m_Max - m_Min)*(width() - 10) + 5;
}

double FJDoubleSlider::lowerValue()
{ 
	double lower = std::min(m_Lower, m_Upper);
	return lower;
}

double FJDoubleSlider::upperValue()
{ 
	double upper = std::max(m_Lower, m_Upper);
	return upper;
}