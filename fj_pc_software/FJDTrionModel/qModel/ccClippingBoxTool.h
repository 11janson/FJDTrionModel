﻿//##########################################################################
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

#ifndef CC_CLIPPING_BOX_TOOL_HEADER
#define CC_CLIPPING_BOX_TOOL_HEADER

//common
#include <ccOverlayDialog.h>

//local
#include "ccEnvelopeExtractor.h"

#include <ui_clippingBoxDlg.h>

//qCC_db
#include <ccGLUtils.h>

//system
#include <vector>
#include<QDoubleSpinBox>
class ccGenericPointCloud;
class ccGenericMesh;
class ccProgressDialog;
class ccGLWindow;
class ccHObject;
class ccClipBox;
class ccPolyline;
class ccBBox;

//! Dialog for managing a clipping box
class ccClippingBoxTool : public ccOverlayDialog, public Ui::ClippingBoxDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccClippingBoxTool(QWidget* parent);
	//! Default destructor
	virtual ~ccClippingBoxTool();

	//inherited from ccOverlayDialog
	virtual bool linkWith(ccGLWindow* win) override;
	virtual bool start() override;
	virtual void stop(bool state) override;

	//将裁剪框添加到选中点云
	void addClippingBoxToPointCloud();

	//清楚点云中的裁剪框
	void clearClippingBoxFromPointCloud();
	//! Returns box
	ccClipBox* box() const { return m_clipBox; }

	//! Adds an entity
	/** \return success, if the entity is eligible for clipping
	**/
	bool addAssociatedEntity(ccHObject* anObject);

	//! Returns the current number of associated entities
	unsigned getNumberOfAssociatedEntity() const;

	//! Extract slices and optionally envelopes from various clouds and/or clouds
	/** \param clouds input clouds (may be empty if meshes are defined)
		\param meshes input meshes (may be empty if clouds are defined)
		\param clipBox clipping box
		\param singleSliceMode if true, a single cut is made (the process is not repeated) and only the envelope is extracted (not the slice)
		\param processDimensions If singleSliceMode is true: the dimension normal to the slice should be true (and the others false). Otherwise: the dimensions along which to repeat the cuting process should be true.
		\param outputSlices output slices (if successful)
		\param extractEnvelopes whether to extract envelopes or not
		\param maxEdgeLength max envelope edge length (the smaller, the tighter the envelope will be)
		\param outputEnvelopes output envelopes (if successful)
		\param extractLevelSet whether to extract the level set or not
		\param levelSetGridStep the step of the grid from which the level set will be extraced
		\param levelSet level set (contour) lines (if any)
		\param gap optional gap between each slice
		\param multiPass multi-pass envelope extraction
		\param splitEnvelopes whether to split the envelope(s) when the segment can't be smaller than the specified 'maxEdgeLength'
		\param projectOnBestFitPlane to project the points on the slice best fitting plane (otherwise the plane normal to the 
		\param visualDebugMode displays a 'debugging' window during the envelope extraction process
		\param generateRandomColors randomly colors the extracted slices
		\param progressDialog optional progress dialog
	**/
	static bool ExtractSlicesAndContours
		(
		const std::vector<ccGenericPointCloud*>& clouds,
		const std::vector<ccGenericMesh*>& meshes,
		ccClipBox& clipBox,
		bool singleSliceMode,
		bool processDimensions[3],
		std::vector<ccHObject*>& outputSlices,

		bool extractEnvelopes,
		PointCoordinateType maxEdgeLength,
		ccEnvelopeExtractor::EnvelopeType envelopeType,
		std::vector<ccPolyline*>& outputEnvelopes,

		bool extractLevelSet,
		double levelSetGridStep,
		int levelSetMinVertCount,
		std::vector<ccPolyline*>& levelSet,

		PointCoordinateType gap = 0,
		bool multiPass = false,
		bool splitEnvelopes = false,
		bool projectOnBestFitPlane = false,
		bool visualDebugMode = false,
		bool generateRandomColors = false,
		ccProgressDialog* progressDialog = 0);

    void reset();

public slots:
	void closeDialog();
protected:

	void toggleInteractors(bool);
	void toggleBox(bool);

	void editBox();
	void restoreLastBox();
	void extractContour();
	void removeLastContour();
	void exportSlice();
	void exportMultSlices();

	void onBoxModified(const ccBBox* box);

	void thicknessChanged(double);

	inline void shiftXMinus() { shiftBox(0, true);  }
	inline void shiftXPlus()  { shiftBox(0, false); }
	inline void shiftYMinus() { shiftBox(1, true);  }
	inline void shiftYPlus()  { shiftBox(1, false); }
	inline void shiftZMinus() { shiftBox(2, true);  }
	inline void shiftZPlus()  { shiftBox(2, false); }

	void setFrontView();
	void setBottomView();
	void setTopView();
	void setBackView();
	void setLeftView();
	void setRightView();


    void slotSectionIn();
    void slotSectionOut();
    void slotCreate();



protected:

	//! Extracts slices and/or contours
	void extractSlicesAndContours(bool singleSliceMode);

	//! Shift box
	void shiftBox(unsigned char dim, bool minus);

	//! Sets predefined view
	void setView(CC_VIEW_ORIENTATION orientation);

	//! Clipping box
	ccClipBox* m_clipBox;
    private:
        void setUIstyleSheet(QToolButton*,QString);
        //[!]组合是否开启
        bool m_bCurrentCombination = false;
    public:
        //[!]提供外部刷新数据
        bool  refreshData(ccHObject::Container);
        void setCombineOpen();
        //[!]当组合关闭的时候，是否可以使用裁剪按钮
        bool m_bCloseCombinationCropButton = false;
        //[!]组合接口 true为组合状态
        void setCurrentButtonStatus(bool);
        //[!]记录点云状态是否被改变
        bool m_bPointCloudStateChanges = false;

        //[!]点击裁剪后按钮状态
        void setSectionButtonStatus(bool);
        //[!]命名时判断点云外部还是点云内部
        bool disPlayPointName = true;
        //[!]存储初始点云
        ccHObject::Container m_selectedEntities;
        //[!]暂存预览的点云
        std::map<int, ccHObject *> m_pContainerPointObject;

    public slots:
        void slotSetResetToolButtonEnabel(bool);
        void slotThicknesstoolbutton();
        void slotSetCloseDoubleBox();
    signals:
        void signalSendDoubleSpinBoxValue(CCVector3);
		void updateSegmentState(bool isopen);

};

#endif //CC_CLIPPING_BOX_TOOL_HEADER
