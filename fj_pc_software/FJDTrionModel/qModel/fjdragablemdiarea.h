#ifndef FJDRAGABLEMDIAREA_H
#define FJDRAGABLEMDIAREA_H

#include <QWidget>
#include <QMdiSubWindow>
#include <QMouseEvent>
#include <QSplitter>
class FJDragableMdiArea : public QWidget
{
    Q_OBJECT

public:
	enum MDIAREASHOWTYPE { SHOWONLY3D, SHOWONLY2D,SHOWALL};
    FJDragableMdiArea(QWidget *parent = nullptr);
    ~FJDragableMdiArea();
    QMdiSubWindow * addMethubSubWindow(QWidget *widget, Qt::WindowFlags windowFlags = Qt::WindowFlags(),bool is3D = true);

	/**
	 * @brief 设置当前显示模式
	 * @param
	 * @return
	*/
	void setShowType(FJDragableMdiArea::MDIAREASHOWTYPE type);

	/**
	 * @brief 获取2D获取当前显示模式
	 * @param
	 * @return
	*/
	FJDragableMdiArea::MDIAREASHOWTYPE getShowType();

	/**
	 * @brief 获取2D子窗口
	 * @param
	 * @return
	*/
	QMdiSubWindow * get2dWindow();

	/**
	 * @brief 获取3D子窗口
	 * @param
	 * @return
	*/
	QMdiSubWindow * get3dWindow();

	/**
	 * @brief 刷新界面显示
	 * @param 
	 * @return 
    */
	void refreashWindow();

	/**
	 * @brief 获取子窗口
	 * @param
	 * @return
	*/
	QList<QMdiSubWindow*> subWindowList();

	/**
	 * @brief 设置水平还是垂直
	 * @param
	 * @return
	*/
	void setDirection(Qt::Orientation orientation);

	/**
	 * @brief 设置3D窗口在前还是在后
	 * @param
	 * @return
	*/
	void setWindowOrder(bool is3dfront);

	void setActiveSubWindow(QMdiSubWindow *window);

	QMdiSubWindow * activeSubWindow() const;

	void closeAllSubWindows();
protected:

    virtual void resizeEvent(QResizeEvent *event);
	virtual void paintEvent(QPaintEvent *event);

signals:
	void subWindowActivated(QMdiSubWindow *window);

public slots:

private:
    QMdiSubWindow * m_3dWindow = nullptr;
    QMdiSubWindow * m_2dWindow= nullptr;
	bool m_is3DWindowActive = true;
    int m_seperateHeight = -1;
    bool m_isDragState = false;
    double m_ratio = 0.6;
	MDIAREASHOWTYPE m_showType = SHOWONLY3D;
	QSplitter                       *m_pEditCenterSpliterWidget = nullptr;
};
#endif // FJDRAGABLEMDIAREA_H
