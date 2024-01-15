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

#ifndef CC_VOLUME_CALC_TOOL_HEADER
#define CC_VOLUME_CALC_TOOL_HEADER
#include <vector>
//Local
#include "cc2.5DimEditor.h"
#include "ccPickingListener.h"
//Qt
#include <QDialog>
#include "cloudcomparewidgets/metahublframelessdialog.h"
#include "CCGeom.h"
#include "ccPointCloud.h"
#include "ccshowrect.h"

class ccGenericPointCloud;
class ccPolyline;
class ccHObject;
class ccCuboids;
class ccSurface;
class ccPlane;
class ccPoint;
class ccPickingHub;
namespace Ui {
	class VolumeCalcDialog;
}

//! Volume calculation tool (dialog)
class ccVolumeCalcTool : public QWidget, public cc2Point5DimEditor, public ccPickingListener
{
	Q_OBJECT

public:
	//! Default constructor
	ccVolumeCalcTool(ccPickingHub* pickingHub, ccGenericPointCloud* cloud1, ccGenericPointCloud* cloud2, QWidget* parent = nullptr);

	//! Destructor
	~ccVolumeCalcTool();
	virtual void slotUIButtonOk(void);
	//Inherited from cc2Point5DimEditor
	virtual double getGridStep() const override;
	virtual unsigned char getProjectionDimension() const override;
	virtual ccRasterGrid::ProjectionType getTypeOfProjection() const override;

	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

	//! Adds a point to the 'align' set
	bool addAlignedPoint(CCVector3d& Pin);

	//! Report info
	struct ReportInfo
	{
		ReportInfo()
			: volume(0)
			, addedVolume(0)
			, removedVolume(0)
			, surface(0)
			, matchingPrecent(0)
			, ceilNonMatchingPercent(0)
			, groundNonMatchingPercent(0)
			, averageNeighborsPerCell(0)
		{}

		QString toText(int precision = 6) const;

		double volume;
		double addedVolume;
		double removedVolume;
        double surface;
        double addedSurface;
        double removedSurface;
		float matchingPrecent;
		float ceilNonMatchingPercent;
		float groundNonMatchingPercent;
		double averageNeighborsPerCell;
	};

	//! Static accessor
	static bool ComputeVolume(	ccRasterGrid& grid,
								ccGenericPointCloud* ground,
								ccGenericPointCloud* ceil,
								const ccBBox& gridBox,
								unsigned char vertDim,
								double gridStep,
								unsigned gridWidth,
								unsigned gridHeight,
								ccRasterGrid::ProjectionType projectionType,
								ccRasterGrid::EmptyCellFillOption groundEmptyCellFillStrategy,
								double groundMaxEdgeLength,
								ccRasterGrid::EmptyCellFillOption ceilEmptyCellFillStrategy,
								double ceilMaxEdgeLength,
								ccVolumeCalcTool::ReportInfo& reportInfo,
								double groundHeight,
								double ceilHeight,
								QWidget* parentWidget = nullptr);

    static bool ComputeVolume2(ccRasterGrid& grid,
        std::vector<float> PlanePara,
        ccGenericPointCloud* ceil,
        const ccBBox& gridBox,
        unsigned char vertDim,
        double gridStep,
        unsigned gridWidth,
        unsigned gridHeight,
        ccRasterGrid::ProjectionType projectionType,
        ccRasterGrid::EmptyCellFillOption ceilEmptyCellFillStrategy,
        double ceilMaxEdgeLength,
        ccVolumeCalcTool::ReportInfo& reportInfo,
        double ceilHeight,
        QWidget* parentWidget = nullptr);
	//! Converts a (volume) grid to a point cloud
	static ccPointCloud* ConvertGridToCloud(	ccRasterGrid& grid,
												const ccBBox& gridBox,
												unsigned char vertDim,
												bool exportToOriginalCS);

public slots:
    void slotExportWordFileToDesktop();

	//[!]文件导出完成提示框
	void slotExportFileExit();

    void slotEdgeLengthChanged();
	
protected:

	void adjustItemWidth(QComboBox * combox);
	//! Accepts the dialog and save settings
	void saveSettingsAndAccept();

	//! Save persistent settings and 'accept' dialog
	void saveSettings();

	//! Called when the projection direction changes
	void projectionDirChanged(int);

	//! Called when the SF projection type changes
	void sfProjectionTypeChanged(int);

	//Inherited from cc2Point5DimEditor
	virtual bool showGridBoxEditor() override;

	//! Called when the (ground) empty cell filling strategy changes
	void groundFillEmptyCellStrategyChanged(int);
	//! Called when the (ceil) empty cell filling strategy changes
	void ceilFillEmptyCellStrategyChanged(int);

	//! Called when the an option of the grid generation has changed
	void gridOptionChanged();

	//! Updates the gid info
	void updateGridInfo();

	//! Update the grid and the 2D display
	void updateGridAndDisplay();

	void oncreatebtn();

	void onclosebtn();

    void onresetbtn();
	//! Ground source changed
	void groundSourceChanged(int index);
	//! Ceil source changed
	void ceilSourceChanged(int);

	//! Exports info to clipboard
	void exportToClipboard() const;

	//! Exports the grid as a point cloud
	void exportGridAsCloud();

	//! Sets the displayed number precision
	void setDisplayedNumberPrecision(int);

	//! Returns the ground cloud (or constant height)
	std::pair<ccGenericPointCloud*, double> getGroundCloud() const;

	//! Returns the ceil cloud (or constant height)
	std::pair<ccGenericPointCloud*, double> getCeilCloud() const;

    int link3dMainViewer();
    void redrawConsultPlane();//绘制参考平面

protected: //standard methods

	//Inherited from cc2Point5DimEditor
	virtual void gridIsUpToDate(bool state) override;

	//! Load persistent settings
	void loadSettings();

	//! Updates the grid
	bool updateGrid(std::pair<ccGenericPointCloud*, double> ground, std::pair<ccGenericPointCloud*, double> ceil,ccBBox box);

    bool updateGrid2(std::vector<float> Para, std::pair<ccGenericPointCloud*, double> ceil, ccBBox box);
	//! Converts the grid to a point cloud
    ccPointCloud* convertGridToCloud(bool exportToOriginalCS) const;
    ccCuboids* convertGridToBoxes() const;

	//! Outputs the report
	void outputReport(const ReportInfo& info);

	//清除自定义平面已选点
	void clearCustomParam();
protected: //members

	//! First associated cloud
	ccGenericPointCloud* m_cloud1;
	//! Second associated cloud
	ccGenericPointCloud* m_cloud2;

	//! Last report
	/** Only valid if clipboardPushButton is enabled
	**/
	ReportInfo m_lastReport;

	//小数点位数
	int m_precision = 3;
	double m_ceilMaxEdgeLength = 0;
	double m_groundMaxEdgeLength = 0;
	Ui::VolumeCalcDialog* m_ui;
	ccPickingHub* m_pickingHub;//选点监听

private:
    ccCuboids* m_pBoxContainer;//立方体网格
    ccSurface* m_pSurface;//参考平面
	ccShowRect * m_RectSurface; // 任意平面参考平面
    //[!]获取doc中的所有数据
    QMap<QString, QString>map;
    //[!]保存所有书签
    QStringList mlist;

	ccPointCloud m_pickingPoints;//自定义平面选点
signals:
	void signalExportFileExit();
};

#endif //CC_VOLUME_CALC_TOOL_HEADER
