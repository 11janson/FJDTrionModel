#pragma once
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

#include "CCPluginAPI.h"

//Qt
#include <QDialog>
#include <QList>
#include <FJBaseWidget.h>
#include <QShowEvent>
#include "qevent.h"
class ccGLWindow;

//! Generic overlay dialog interface
class CCPLUGIN_LIB_API ccOverlayDialog : public FJBaseWidget
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccOverlayDialog(QWidget* parent = nullptr, Qt::WindowFlags flags = Qt::FramelessWindowHint | Qt::Tool);

	//! Destructor
	~ccOverlayDialog() override;


	virtual void InitFJStyle();
	//! Links the overlay dialog with a MDI window
	/** Warning: link can't be modified while dialog is displayed/process is running!
		\return success
	**/
	virtual bool linkWith(ccGLWindow* win);

	//! Starts process
	/** \return success
	**/
	virtual bool start();

	//! Stops process/dialog
	/** Automatically emits the 'processFinished' signal (with input state as argument).
		\param accepted process/dialog result
	**/
	virtual void stop(bool accepted);

	//reimplemented from QDialog
	void reject() override;

	//! Adds a keyboard shortcut (single key) that will be overridden from the associated window
	/** When an overridden key is pressed, the shortcutTriggered(int) signal is emitted.
	**/
	void addOverriddenShortcut(Qt::Key key);

    void removeOverriddenShortcut(Qt::Key key);

	//! Returns whether the tool is currently started or not
	bool started() const { return m_processing; }

signals:

	//! Signal emitted when process is finished
	/** \param accepted specifies how the process finished (accepted or not)
	**/
	void processFinished(bool accepted);

	//! Signal emitted when an overridden key shortcut is pressed
	/** See ccOverlayDialog::addOverriddenShortcut
	**/
	void shortcutTriggered(int key);

    void shortcutDoubleBondTriggered(int key, Qt::KeyboardModifiers);

	//! Signal emitted when a 'show' event is detected
	void shown();

protected:

	//! Slot called when the linked window is deleted (calls 'onClose')
	virtual void onLinkedWindowDeletion(QObject* object = nullptr);

	virtual void mousePressEvent(QMouseEvent *event);

	virtual void mouseMoveEvent(QMouseEvent *event);

	virtual void mouseReleaseEvent(QMouseEvent *event);

	virtual void paintEvent(QPaintEvent *event);

	virtual void keyPressEvent(QKeyEvent *event);

    virtual void resizeEvent(QResizeEvent *event);

    virtual void showEvent(QShowEvent *event);
protected:

	//inherited from QObject
	bool eventFilter(QObject *obj, QEvent *e) override;

	//! Associated (MDI) window
	ccGLWindow* m_associatedWin;

	//! Running/processing state
	bool m_processing;

	//! Overridden keys
	QList<int> m_overriddenKeys;

	bool m_isPressed = false;//是否鼠标按下的标识
	QPoint m_startPos;//开始拖拽时鼠标点击控件的相对位置
	QPoint m_startMovePos;

	int m_nOffLeft = 0;
	int m_nOffRight = 0;
	int m_nOffTop = 0;
	int m_nOffBottom = 0;
	QWidget *m_parent = nullptr;//保存父控件的指针

    public:
        //[!]控制需要刷新数据时，窗口不关闭（解决窗口闪现问题）
        bool m_bControlDialogNotClose = false;

};
