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

#ifndef CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER
#define CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QSet>

//GUI
#include <ui_graphicalSegmentationDlg.h>
#include "ccPickingHub.h"
#include "ccPickingListener.h"
#include "ccshowline.h"
#include "ccshowPoints.h"

class ccPolyline;
class ccPointCloud;
class ccGLWindow;
class ccMainAppInterface;

//! Graphical segmentation mechanism (with polyline)
class ccGraphicalSegmentationTool : public ccOverlayDialog,public ccPickingListener, public Ui::GraphicalSegmentationDlg
{
	Q_OBJECT

public:
	enum CurrentSegmentMode {
		RECTANGULAR = 0,
		POLYLINEMODE,
		NOPICKINGMODE
	};
	enum Cuttingmode {
		POINTCLOUDCUTTING = 0,
		TRACKCUTTING
	};

	//! Default constructor
	explicit ccGraphicalSegmentationTool(QWidget* parent,ccPickingHub* pickingHub);
	//! Destructor
	virtual ~ccGraphicalSegmentationTool();

signals:
	/**
	*@brief 拾取模式发生变化
	*/
	void signalSegmentationModeStatusChanged(void);

	/**
	*@brief 拾取状态完成
	*/
	void signalSegmentationModeFinish(void);
public:





	virtual void InitFJStyle();
	//! Adds an entity (and/or its children) to the 'to be segmented' pool
	/** Warning: some entities may be rejected if they are
		locked, or can't be segmented this way.
		\return whether entity has been added to the pool or not
	**/
	bool addEntity(ccHObject* anObject, bool silent = false);
	
	//! Returns the number of entites currently in the the 'to be segmented' pool
	unsigned getNumberOfValidEntities() const;

	//! Get a pointer to the polyline that has been segmented
	const ccPolyline* getPolyLine() const { return m_segmentationPoly; }

	//! Returns the active 'to be segmented' set
	QSet<ccHObject*>& entities() { return m_toSegment; }
	//! Returns the active 'to be segmented' set (const version)
	const QSet<ccHObject*>& entities() const { return m_toSegment; }

	//inherited from ccOverlayDialog
	virtual bool linkWith(ccGLWindow* win) override;
	virtual bool start() override;
	virtual void stop(bool accepted) override;
    virtual void updateSegmentationPickStatus(void);

	//! Returns whether hidden parts should be delete after segmentation
	bool deleteHiddenParts() const { return m_deleteHiddenParts; }

	//! Remove entities from the 'to be segmented' pool
	/** \warning 'unallocateVisibilityArray' will be called on all point clouds
		prior to be removed from the pool.
	**/
	void removeAllEntities(bool unallocateVisibilityArrays);

	//! Apply segmentation and update the database (helper)
	bool applySegmentation(ccMainAppInterface* app, ccHObject::Container& newEntities,ccHObject::Container& enterEntities);

	//设置点云
	void setPointCloud(ccHObject* obj);

	//设置轨迹
	void setTrackPointCloud(ccHObject* obj);

	//设置普通切割模式还是轨迹切割模式
	void setCuttingMode(Cuttingmode cuttingMode);

	//选中点云
	void onItemPicked(const PickedItem& pi) override;

	//清除两点选择显示
	void clearTwoPointSelectCache();

    //更新高亮点云点大小
	void updateHeightLightPointSize(bool state, int index);
	//[!]Ctrl+z
    void shortCutkeysCtrlZFunction();

	/**
	*@执行取消拾取状态
	*/
	void exeCancelSegmentModeStatus(void);
	/**
	*@brief 设置是否支持做个拾取框选路径
	*@param 使能
	*/
	void setMultipleSegmentationEnable(bool bEnable);

	/**
	*@brief 获取裁切盒集合
	*/
	ccHObject* getPolyContainer(void);

	/**
	*@brief 获取当前截取的模式
	*/
	CurrentSegmentMode getCuurentSegmentMode(void) { return m_segmentMode; }

protected:

	//选择两点切割模式
	void onTwoPointCutting();

	//设置点云是否可见
	void onPointCloudVisiable();

	void segmentIn();
	void segmentOut();
	void segment(bool keepPointsInside, ScalarType classificationValue = CCCoreLib::NAN_VALUE);
	void reset();
	void resetOnce();//修改撤回按钮可撤回选择的多边形
	void apply();
	void applyAndDelete();
	void cancel();
	inline void addPointToPolyline(int x, int y) { return addPointToPolylineExt(x, y, false); }
	void addPointToPolylineExt(int x, int y, bool allowClicksOutside);
	void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void closeRectangle();
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void pauseSegmentationMode(bool);
    void setClassificationValue();
	void doSetPolylineSelection();
	void doSetRectangularSelection();
	void doActionUseExistingPolyline();
	void doExportSegmentationPolyline();

	void removeLastStep();//取消上一步动作

	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

    //! To capture overridden shortcuts (pause button, etc.)
    void onShortcutDoubleBondTriggered(int, Qt::KeyboardModifiers);

	//! Prepare entity before removal
	void prepareEntityForRemoval(ccHObject* entity, bool unallocateVisibilityArrays);

	//! Whether to allow or not to exort the current segmentation polyline
	void allowPolylineExport(bool state);

	void initcurrentStyle();

	//缩放平移时更新框线位置
	void updatePolylinePoint(int x,int y);

	void resetSegmentOnce();//撤销一次分割操作

	//添加选取时间段
	void addSelectTimeStep(double startTime,double stopTime);

	//是否在选中时间段内
	bool isInSelectTimeStepRange(double time);

    

protected:
	//! Set of entities to be segmented
	QSet<ccHObject*> m_toSegment;

	//! Whether something has changed or not (for proper 'cancel')
	bool m_somethingHasChanged;
	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		RECTANGLE		= 2,
		//...			= 4,
		//...			= 8,
		//...			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Current process state
	unsigned m_state;

	//! Segmentation polyline
	ccPolyline* m_segmentationPoly;
	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

	ccHObject* polyContainer;//存储多个裁剪框

	//! Selection mode
	bool m_rectangularSelection;

	//! Whether to delete hidden parts after segmentation
	bool m_deleteHiddenParts;

	CurrentSegmentMode m_segmentMode = NOPICKINGMODE;

	std::vector<CCVector3d> m_3DPointData; //m_segmentationPoly的真实坐标

	bool m_isMousePressMoveValid = true;

	std::vector<std::map<int, std::vector<unsigned char>>> m_cashSegmentPoints;

	ccHObject* m_pointCloud = nullptr; // 按轨迹切割的点云

	double m_pointCloudSFShift = 0.0;  // 按轨迹切割的点云时间标量偏移值

	ccHObject* m_trackPointCloud = nullptr; // 按轨迹切割的轨迹

	double m_trackPointCloudSFShift = 0.0;  // 按轨迹切割的轨迹时间标量偏移值

	Cuttingmode m_cuttingMode = POINTCLOUDCUTTING; //当前是按轨迹切割还是普通切割

	int m_firstPointIndex = -1;  //两点选择第一个点index

	int m_secondPointIndex = -1;  //两点选择第二个点index
	
	bool m_isTwoPointCuttingMode = false;  //是否为两点选择模式

	ccPickingHub* m_pickingHub; //监听类

	ccShowPoints* m_selectedPoints;  //当前选择框

	ccShowLine * m_twoPointLine;     //两点选择工具连线

	bool m_Invertselection = false; //是否为反选模式

	std::map<double, double> m_startToStopTime;  //两点选择模式选择的各个起止时间段

	bool	m_bMultipleSegmentationEnable = true;		 ///<是否之前过个裁切框

};

#endif //CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER
