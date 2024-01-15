//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
#ifndef __FJMANUALCLASSIFICATIONDLG_H__
#define __FJMANUALCLASSIFICATIONDLG_H__

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QSet>
#include <QCheckBox>
#include <QGridLayout>
//GUI
#include <ui_fjmanualclassificationdlg.h>
#include "ccGenericPointCloud.h"
#include "metahubtoolbutton.h"

class ccPolyline;
class ccPointCloud;
class ccGLWindow;
class ccMainAppInterface;

class FJManualClassificationDlg : public ccOverlayDialog, public Ui::FJManualClassificationDlg
{
	Q_OBJECT

public:
	enum CurrentSegmentMode {
		RECTANGULAR = 0,
		POLYLINEMODE,
		LINEUPMODE,
		LINEDOWNMODE,
		LINELASSOMODE,
		NOPICKINGMODE
	};

	explicit FJManualClassificationDlg(QWidget* parent);

	virtual ~FJManualClassificationDlg();

	//重置界面样式
	virtual void InitFJStyle();

	//添加待分类点云
	bool addEntity(ccHObject* anObject, bool silent = false);
	
	//返回待分类点云数量
	unsigned getNumberOfValidEntities() const;

	//绑定显示窗口
	virtual bool linkWith(ccGLWindow* win) override;

	//开始手动分类功能
	virtual bool start() override;

	//关闭手动分类功能
	virtual void stop(bool accepted) override;

	//删除所有点云
	void removeAllEntities(bool unallocateVisibilityArrays);

	//清除所有类别
	void clearAllClassficationType();

	//初始化下拉菜单类别内容
	void initAllClassficationType();

    //更新高亮点云点大小
	void updateHeightLightPointSize(bool state, float size);
protected:

	//初始化界面样式
	void initcurrentStyle();

	//缩放平移时更新框线位置
	void updatePolylinePoint(int x, int y);

	//重置界面
	void reset();

	//关闭界面
	void cancel();

	//快捷键操作
	void onShortcutTriggered(int);

	//开启多边形选择
	void doSetPolylineSelection();

	//开启矩形选择
	void doSetRectangularSelection();

	//开启线上选择
	void doSetPolylineUpSelection();

	//开启线下选择
	void doSetPolylineDownSelection();

	//开启套索选择
	void doSetPolylineLassoSelection();

	//设置是否为选择模式
	void pauseSegmentationMode(bool);

	//取消上一步动作
	void removeLastStep();

	//设置当前类别种类
	void setCurrentClassficationType(const std::set<double> & selectedClassfications);

	//计算选中点云类别
	void calculateManualClassification();

	//左键单击释放槽函数
	inline void addPointToPolyline(int x, int y) { return addPointToPolylineExt(x, y, false); }

	//左键单击释放
	void addPointToPolylineExt(int x, int y, bool allowClicksOutside);

	//获取当前选取方式
	unsigned getCurrentSelectedMode();

	//关闭套索选择
	void closeLasso();

	//关闭多边形选择
	void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal

	//关闭矩形选择
	void closeRectangle();

	//鼠标移动
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);

	//修改按钮图标
	void setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix);

	//关闭按钮选中状态
	void closeButtonState();
public slots:

	//选择全部类别
	void selectedAllClassfication();

	//应用类别分类
	void applyClassfication();

	//删除所有选中点云
	void clearALL();

	//类别勾选槽函数
	void checkboxStateChanged(int);

	//更新类别颜色
	void updateTypeColor();

	//修改类别颜色
	void changeClassficationColor();
    //[!]撤销
    void onShortcutDoubleBondTriggered(int, Qt::KeyboardModifiers);

protected:
	//! Set of entities to be segmented
	QSet<ccHObject*> m_toSegment;

	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		RECTANGLE		= 2,
		POLYLINEUP      = 4,
		POLYLINEDOWN    = 8,
		POLYLINELASSO   = 16,
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

	CurrentSegmentMode m_segmentMode = NOPICKINGMODE;

	std::vector<CCVector3d> m_3DPointData; //m_segmentationPoly的真实坐标

	bool m_isMousePressMoveValid = true;

	ccPointCloud* m_selectedPoints;  //当前选择框

	std::vector<QCheckBox *> m_checkBoxs; //当前类别勾选框

	std::vector<double> m_currentSelectType; //当前类别

	QGridLayout *m_layout; //类别布局框

	ccGenericPointCloud::VisibilityTableType m_cashTable;//缓存选中点云index

	bool m_Invertselection = false; //是否为反选模式
};

#endif //CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER
