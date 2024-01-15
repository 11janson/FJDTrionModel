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

#ifndef CC_POINT_PROPERTIES_DIALOG_HEADER
#define CC_POINT_PROPERTIES_DIALOG_HEADER

#include "ccPointPickingGenericInterface.h"

#include "cc2DLabel.h"

//Local
#include <ui_pointPropertiesDlg.h>
#include "ccmeasureline.h"

class cc2DLabel;
class cc2DViewportLabel;
class ccHObject;
class ccClipBox;

//! Dialog for simple point picking (information, distance, etc.)
class ccPointPropertiesDlg : public ccPointPickingGenericInterface, public Ui::PointPropertiesDlg
{
	Q_OBJECT

public:

	enum CloudTypeMode
	{
		POINT2DMODE,
		POINT3DMODE,
	};


	enum MeasureMode
	{
		MEASURE_POINT = 0,
		MEASURE_DISTANCE = 1,
		MEASURE_ANGLE = 2,
		MEASURE_HEIGHT = 3,
		MEASURE_AERA = 4,
		MEASURE_VOLUME = 5,
	};

	//! Default constructor
	explicit ccPointPropertiesDlg(ccPickingHub* pickingHub, QWidget* parent);
	//! Default destructor
	virtual ~ccPointPropertiesDlg();

	//inherited from ccPointPickingGenericInterface
	virtual bool start() override;
	virtual void stop(bool state) override;
	virtual bool linkWith(ccGLWindow* win) override;

	//设置是2D还是3D
	void setCurrentCloudTypeMode(CloudTypeMode mode);

	//设置当前测量模式
	void setCurrentMeasureMode(MeasureMode mode) { m_measureMode = mode; }

	//设置当前点云
	void setCurrentObj(ccHObject::Container object) {m_selectedEntities = object; m_object = m_selectedEntities[0];}

public slots:

	//鼠标移动槽函数
	void slotMouseMovePos(CCVector3d pos, QPointF mousePos);

	//单点测量
	void activatePointPropertiesDisplay(); 

	//距离测量
	void activateDistanceDisplay();

	//角度测量
	void activateAngleDisplay();

	//高度测量
	void activeHeightDisplay();

	//面积测量
	void activeAeraDisplay();

	//体积测量
	void activeVolumeDisplay();

	//右键结束测量
	void closePolyLine(int, int);


	//ESC取消
	void onShortcutTriggered(int key);

    //更新视图
    void updatePolylinePoint(int x, int y);

protected:

	//获取距离测量值
	double getDistanceMeasureResult();

	//获取面积测量值
	double getAreaMeasureResult(std::vector<CCVector3d> pointList);

	//根据屏幕坐标获取三维坐标
	CCVector3d getPositionByScreenPos(QPointF point);

	//根据屏幕坐标获取面积测量时通过中心点面的三维点
	CCVector3d getPositionByScreenPosThroughCenter(QPointF point);

	//撤销
	void initializeState();

	//保存测量结果
	void exportCurrentLabel();

	//初始化样式
	void InitStyle();

	//修改按钮样式
	void createAction(QToolButton * button,const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName);

	//获取面积测量方向向量
	void getDirectionVector();

	//获取鼠标位置处当前方向的点
	CCVector3d getCurrentDirectionMousePos(CCVector3d point);

signals:

	//! Signal emitted when a new label is created
	void newLabel(ccHObject*);

protected:

	//inherited from ccPointPickingGenericInterface
	void processPickedPoint(const PickedItem& picked) override;

	//! Associated 3D label
	cc2DLabel* m_label;

	ccClipBox* m_clipBox;

	CloudTypeMode m_pointMode = POINT3DMODE;  //2d还是3d

	MeasureMode m_measureMode = MEASURE_POINT;  //当前测量模式

	ccHObject* m_object = nullptr;   //当前标签父节点

	ccHObject::Container m_selectedEntities; //选中测量的点云

	ccMeasureLine * m_measureLine = nullptr;   //测量过程中的线

	CCVector3d m_directionVector;  //面积测量方向向量
};

#endif
