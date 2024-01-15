//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccOverlayDialog.h"

//qCC_glWindow
#include <ccGLWindow.h>

//qCC_db
#include <ccLog.h>

//Qt
#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QPainter>
//system
#include <cassert>
#include "ccGuiParameters.h"
#include <QScreen>
#include <QGuiApplication>
ccOverlayDialog::ccOverlayDialog(QWidget* parent/*=nullptr*/, Qt::WindowFlags flags/*=Qt::FramelessWindowHint | Qt::Tool*/)
	: FJBaseWidget(parent, flags)
	, m_associatedWin(nullptr)
	, m_processing(false)
{
	m_parent = parent;
	setAttribute(Qt::WA_NoMousePropagation);
}

ccOverlayDialog::~ccOverlayDialog()
{
	onLinkedWindowDeletion();
}

void ccOverlayDialog::InitFJStyle()
{

}

void ccOverlayDialog::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Escape:
		break;
	default:
		QDialog::keyPressEvent(event);
	}


}

void ccOverlayDialog::resizeEvent(QResizeEvent *event)
{
    QDialog::resizeEvent(event);
}

void ccOverlayDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    this->setFixedSize(this->size());
}

void ccOverlayDialog::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, true);
	QPen pen;
	pen.setWidth(1);
	pen.setColor(QColor(112, 112, 112));
	painter.setPen(pen);
	painter.drawRect(0, 0, this->width() - 0, this->height() - 0);
	painter.restore();

}

void ccOverlayDialog::mousePressEvent(QMouseEvent *event)
{
	m_isPressed = true;
	QPoint point1 = this->frameGeometry().topLeft();
	QPoint point2 = m_parent->frameGeometry().topLeft();
	m_startMovePos = event->globalPos() - this->frameGeometry().topLeft() + m_parent->frameGeometry().topLeft();

	m_startPos = this->mapFromGlobal(QCursor::pos());//开始拖拽时点击控件的什么位置
	if (nullptr != m_parent)
	{
		m_nOffLeft = m_startPos.rx();
		m_nOffRight = m_parent->width() - (this->width());
		m_nOffTop = m_startPos.ry();
		m_nOffBottom = m_parent->height() - (this->height());
	}
}

void ccOverlayDialog::mouseMoveEvent(QMouseEvent *event)
{
	if (m_isPressed && nullptr != m_parent)
	{
		QPoint pos = event->globalPos() - m_startMovePos;
		int rx = pos.rx();
		int ry = pos.ry();
		QPoint tmpPos;
		//if (rx < 0)//left矫正，防止子控件被拖出左侧
		//{
		//	rx = 0;
		//}
		//if (ry < 0)//top矫正，防止子控件被拖出上测
		//{
		//	ry = 0;
		//}
		//if (rx > m_nOffRight)//right矫正，防止子控件被拖出右侧
		//{
		//	rx = m_nOffRight;
		//}
		//if (ry > m_nOffBottom)//bottom矫正，防止子控件被拖出下侧
		//{
		//	ry = m_nOffBottom;
		//}

		tmpPos = QPoint(rx, ry);
		move(m_parent->mapToGlobal(tmpPos));//移动子控件
	}

}

void ccOverlayDialog::mouseReleaseEvent(QMouseEvent *event)
{
	m_isPressed = false;
}

bool ccOverlayDialog::linkWith(ccGLWindow* win)
{
	if (m_processing)
	{
		ccLog::Warning("[ccOverlayDialog] Can't change associated window while running/displayed!");
		return false;
	}

	//same dialog? nothing to do
	if (m_associatedWin == win)
	{
		return true;
	}

	if (m_associatedWin)
	{
		//we automatically detach the former dialog
		{
			QWidgetList topWidgets = QApplication::topLevelWidgets();
			foreach(QWidget* widget, topWidgets)
			{
				widget->removeEventFilter(this);
			}
		}
		m_associatedWin->disconnect(this);
		m_associatedWin = nullptr;
	}

	m_associatedWin = win;
	if (m_associatedWin)
	{
		QWidgetList topWidgets = QApplication::topLevelWidgets();
		foreach(QWidget* widget, topWidgets)
		{
			widget->installEventFilter(this);
		}
		connect(m_associatedWin, &QObject::destroyed, this, &ccOverlayDialog::onLinkedWindowDeletion);
	}

	return true;
}

void ccOverlayDialog::onLinkedWindowDeletion(QObject* object/*=nullptr*/)
{
	if (m_processing)
		stop(false);

	linkWith(nullptr);
}

bool ccOverlayDialog::start()
{
	if (m_processing)
		return false;

	m_processing = true;

	//auto-show
	show();
	ccGui::ParamStruct param = ccGui::Parameters();
	param.m_openShiftMeasure = false;
	ccGui::Set(param);
	return true;
}

void ccOverlayDialog::stop(bool accepted)
{
	m_processing = false;

	//auto-hide
    if (!m_bControlDialogNotClose)
    {
        hide();
    }
    m_bControlDialogNotClose = false;
	linkWith(nullptr);
	ccGui::ParamStruct param = ccGui::Parameters();
	param.m_openShiftMeasure = true;
	ccGui::Set(param);
	emit processFinished(accepted);
}

void ccOverlayDialog::reject()
{
	QDialog::reject();

	stop(false);
}

void ccOverlayDialog::addOverriddenShortcut(Qt::Key key)
{
	m_overriddenKeys.push_back(key);
}

void ccOverlayDialog::removeOverriddenShortcut(Qt::Key key)
{
    m_overriddenKeys.removeAll(key);
}

bool ccOverlayDialog::eventFilter(QObject *obj, QEvent *e)
{
	if (e->type() == QEvent::KeyPress)
	{
		QKeyEvent* keyEvent = static_cast<QKeyEvent*>(e);
		if (m_overriddenKeys.contains(keyEvent->key()))
		{
            if (keyEvent->key() == Qt::Key_Z && keyEvent->modifiers() == Qt::ControlModifier) {
                emit shortcutDoubleBondTriggered(keyEvent->key(), keyEvent->modifiers());
                return true;
            }

			emit shortcutTriggered(keyEvent->key());
			return true;
		}

		return QDialog::eventFilter(obj, e);
	}
	else
	{
		if (e->type() == QEvent::Show)
		{
			emit shown();
		}
		
		// standard event processing
		return QDialog::eventFilter(obj, e);
	}
}
