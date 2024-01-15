#ifndef FJSECTIONANALYSISRENDERWIDGET_H
#define FJSECTIONANALYSISRENDERWIDGET_H

#include <QWidget>
#include <QColor>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QImage>
#include "ccHObjectCaster.h"
#include <ccPointCloud.h>
struct SectionAnalysisData
{

	QColor color = QColor(Qt::red);
	double xStart = 0;
	double yStart = 0;
	double xStop = 0;
	double yStop = 0;
	double thickness = 0.1;
	std::multimap<QString, std::vector<std::pair<QColor,QPointF>>> Pointdata;
    std::vector<bool> PointCloudVisiable;
};

enum SectionAnalysisRenderMode
{
	SUCCESSMODE,
	NONEMODE,
	RUNNINGMODE
};

class FJSectionAnalysisRenderWidget : public QWidget
{
    Q_OBJECT

public:
    FJSectionAnalysisRenderWidget(QWidget *parent = nullptr);
    ~FJSectionAnalysisRenderWidget();

	//设置剖面数据
	void setData(SectionAnalysisData data);

	//根据屏幕点获取剖面坐标
	QPointF getCloudPoint(QPointF point);

	//根据剖面坐标获取屏幕坐标
	QPointF getScreenPoint(QPointF point);

    //根据剖面坐标获取图片坐标
    QPointF getImageScreenPoint(QPointF point);

	//开始测量
	void startMeasure(bool isopen);

	//初始化坐标
	void initPoisition();

	//设置剖面选取的两个点
	void setPickPoint(QPointF start, QPointF end);

	void setRenderMode(SectionAnalysisRenderMode mode);

	SectionAnalysisRenderMode getRenderMode();

    //设置点云是否显示
    void setPointCloudVisiable(std::vector<bool> visiablelist);

    //设置是否显示点云颜色
    void setShowPointColor(bool isshow);

    //更新点大小
    void updatePointSize();

	//设置当前选中点云
	void setCurrentObjects(ccHObject::Container objs);

signals:
	void updateMessage(QString str);
	void finshedPickPoint(QPointF start,QPointF end);

protected:
	virtual void paintEvent(QPaintEvent *event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void wheelEvent(QWheelEvent* event);
	virtual void resizeEvent(QResizeEvent *event);
private:
	QPointF m_pointStart;
	QPointF m_pointStop;
	SectionAnalysisData m_data;
	SectionAnalysisRenderMode m_mode = NONEMODE;
	QPointF m_leftTopOld = QPointF(0, 0);
	QPointF m_rightBottomOld = QPointF(0, 0);
	QPointF m_leftTop = QPointF(0, 0);
	QPointF m_rightBottom = QPointF(0, 0);
	QPointF m_lastLeftMouseClickPoint = QPointF(-1,-1);
	QPointF m_lastRightMouseClickPoint = QPointF(-1, -1);
	bool isMeasureOpen = false;
    bool m_showPointColor = false;

    QImage m_image;
    bool m_isneedupdateimage = false;

	ccHObject::Container  m_selectedObjectList;

};
#endif // FJSECTIONANALYSISRENDERWIDGET_H
