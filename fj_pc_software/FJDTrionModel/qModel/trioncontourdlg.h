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

#ifndef __TRIONCONTOURDLG_H__
#define __TRIONCONTOURDLG_H__

#include <QDialog>
#include "ccGenericPointCloud.h"
#include "ccPolyline.h"
#include "ccGLWindow.h"
#include "ccRasterGrid.h"
#include "ccPointCloud.h"
#include "framelessdialog.h"
namespace Ui {
	class TrionContourDlg;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class TrionContourDlg : public CS::Widgets::FramelessDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit TrionContourDlg(ccGenericPointCloud* cloud, ccGLWindow* glWindow, QWidget* parent = nullptr);
	
	~TrionContourDlg();

signals:
    //关闭程序
    void processFinshed(bool state);

public slots:

	//参数改变修改预览按钮禁用状态
	void slotParamUpdate();

	//首曲线颜色选择
	void slotTntermediateContourColorSelected();

	//计曲线颜色选择
	void slotCountingCurveColorSelected();

	//预览
	void generateContours();

	//保存
	void exportContourLines();

	//清除
	void removeContourLines();

    //显示详细参数设置列表
    void slotDetailSetting();

    //等高线颜色来源选择
    void slotColorFromChanged();

    //恢复默认设置
    void slotRestore();

    //最大高程最小高程改变
    void slotMaxOrMinHeightChanged();

protected:
    virtual void closeEvent(QCloseEvent *event);
private:

	//! Adds a new contour line
	void addNewContour(ccPolyline* poly, double height, unsigned subIndex,int index);

	//计算网格
	void updateGridAndDisplay();

	//! Updates the 2D display zoom
	void update2DDisplayZoom(ccBBox& box);

	//! Updates the grid
	bool updateGrid(bool interpolateSF = false);

	//! Returns the grid size
	bool getGridSize(unsigned& width, unsigned& height) const;


	//! Converts the grid to a cloud with scalar field(s)
	ccPointCloud* convertGridToCloud(const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
		bool interpolateSF,
		bool interpolateColors,
		bool copyHillshadeSF,
		const QString& activeSFName,
		bool exportToOriginalCS) const;

	//! Returns strategy for empty cell filling (extended version)
	ccRasterGrid::EmptyCellFillOption getFillEmptyCellsStrategyExt(double& emptyCellsHeight,
		double& minHeight,
		double& maxHeight) const;

	//! Shortcut to ccRasterGrid::convertToCloud
	ccPointCloud* convertGridToCloudBase(const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
		bool interpolateSF,
		bool interpolateColors,
		bool resampleInputCloudXY,
		bool resampleInputCloudZ, //only considered if resampleInputCloudXY is true!
		ccGenericPointCloud* inputCloud,
		bool fillEmptyCells,
		double emptyCellsHeight,
		bool exportToOriginalCS) const;


    //恢复初始设置
    void initDefaultSetting();
	
private:
	Ui::TrionContourDlg* m_ui;

	ccGenericPointCloud* m_cloud;//原点云

	QColor m_TntermediateContourColor = QColor(255,0,0);  //首曲线颜色

	QColor m_CountingCurveColor = QColor(0,0,255);  //计曲线颜色

	std::vector<ccPolyline*> m_contourLines;  //等高线

	ccGLWindow* m_glWindow;//3D窗口

	ccRasterGrid m_grid;  //光栅网格

	ccPointCloud* m_rasterCloud; //光栅点云
};

#endif //CC_SOR_FILTER_DLG_HEADER
