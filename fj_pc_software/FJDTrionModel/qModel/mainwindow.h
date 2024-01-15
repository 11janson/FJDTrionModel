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

#ifndef CC_MAIN_WINDOW_HEADER
#define CC_MAIN_WINDOW_HEADER

//Qt
#include <QMainWindow>
#include<QTimer>
#include<QDatetime>
#include<QLabel>
//Local
#include "ccEntityAction.h"
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"
#include "fjdragablemdiarea.h"
//CCCoreLib
#include <AutoSegmentationTools.h>

//Ribbon 2022.3.31 janson
#include "SARibbonMainWindow.h"
#include "icenterwidget.h"
#include"metahubpdfdialog.h"

#include "model/planecutting.h"
#include <ccGenericGLDisplay.h>
#include <ccGenericPointCloud.h>
#include <QScrollArea>
#include "ccPointAccuracyVerificationDlg.h"



class QAction;
class QMdiArea;
class QMdiSubWindow;
class QToolBar;
class QToolButton;
class QLineEdit;
class QCheckBox;

class cc3DMouseManager;
class ccCameraParamEditDlg;
class ccClippingBoxTool;
class ccComparisonDlg;
class ccDBRoot;
class ccDrawableObject;
class ccGamepadManager;
class ccGLWindow;
class ccGraphicalSegmentationTool;
class ccGraphicalTransformationTool;
class ccHObject;
class ccOverlayDialog;
class ccPluginUIManager;
class ccPointListPickingDlg;
class ccPointPairRegistrationDlg;
class ccPointPropertiesDlg;
class ccPrimitiveFactoryDlg;
class ccRecentFiles;
class ccSectionExtractionTool;
class ccStdPluginInterface;
class ccTracePolylineTool;
class FjVolumeCalcDlg;
class FJSectionAnalysisDlg;
struct dbTreeSelectionInfo;

//Ribbon
class SARibbonCategory;
class SARibbonContextCategory;
class SARibbonCustomizeWidget;
class SARibbonActionsManager;
class SARibbonQuickAccessBar;
class SARibbonButtonGroupWidget;
class QTextEdit;

class ccPointPickingPosition;
class ccVolumeMeasureTool;
class FJManualClassificationDlg;
class ccSurface;
class PointSizedialog;
class TrionImageFusionDlg;
namespace Ui {
	class MainWindow;
} 

//! Main window
class MainWindow : public SARibbonMainWindow, public ccMainAppInterface, public ccPickingListener
{
	Q_OBJECT

protected:
	//! Default constructor
	//MainWindow() {};
	MainWindow(QWidget* parent = nullptr);

	//! Default desctructor
	~MainWindow() override;
signals:

    //[!]设置单个点云的大小
	//[!]参数说明 cloudAutoState 参数为false时代表 点云自适应大小属性关闭
	
    void signalUpdateEnterCloudSize(bool cloudAutoState, int index);
    /**
    *@brief 位置发生改变
    */
    void signalPosChanged(void);

    void showErrorStr(QString str);

    //[!]判断开始输出与默认输出
    void signalsSendStatusBar(bool);

    void sigSelectItemChanged();

    //[!]队列刷新属性表的UI
    void signalQueueRefreshPropertiesView();

    //[!]与插件传递cloud来源点云
    void signalDBTreeCloudFromPath(QString);

public:

	/**
	*@brief 设置此节点以及父节点是否勾选
	*/
	void setSelfAndParentObjectEnabled(ccHObject* self, bool isenabled);

    //[!]设置单个点云的大小
    void setEnterCloudSize(int index);
    //[!]设置全局的点大小
    void setGlobalPointSize(int index);
    /**
    * @brief 后台保存las文件
    * @param 会默认保存到初始打开文件的位置
    * @return 保存成功返回true
    */
    bool threadSaveLasFile(ccHObject *,QString file, bool state = false);
    //[!]获取所有文件打开时候的路径
    const QStringList getIsOpenFilesPath();

    QString getDBTreeCloudFilePath(ccHObject::Container);
	/**
	* @brief ����ѡ�е������������Ʋ���ʾ
	* @param
	* @return
	*/
	void deselectOtherObject();

	/**
	* @brief ��ʾ���еĵ���
	* @param
	* @return
	*/
	void selectAllObject();

	/**
	* @brief 取消除选择文件可见
	* @param 子节点是否可见
	* @return
	*/
	void deselectAllOtherObject(bool isChildrenNoodVisiable);

	//! Returns the unique instance of this object
	static MainWindow* TheInstance();

    /**
    Melone PCD加载大坐标点云配置文件赋初初值
    */
    void initPCDShiftValue();

	//! Static shortcut to MainWindow::getActiveGLWindow
	static ccGLWindow* GetActiveGLWindow();

	//! Returns a given GL sub-window (determined by its title)
	/** \param title window title
	**/
	static ccGLWindow* GetGLWindow(const QString& title);

	//! Returns all GL sub-windows
	/** \param[in,out] glWindows vector to store all sub-windows
	**/
	static void GetGLWindows(std::vector<ccGLWindow*>& glWindows);

	//! Static shortcut to MainWindow::refreshAll
	static void RefreshAllGLWindow(bool only2D = false);

	//! Static shortcut to MainWindow::updateUI
	static void UpdateUI();

	//! Deletes current main window instance
	static void DestroyInstance();

    //! 根据缩放比例设置点大小
    int getPointSizeByZoomRatio();


    //! set BTree Select Object
    void setDBTreeSelectObject(ccHObject* pObject);

	//! set planecutting running
	void setPlanecuttingStatus(bool running);

    //! delete mark DBTree marked item
    void deleteDBTreeSelectObject(ccHObject* pObject);

    //! free mark DBTree marked item
    void freeDBTreeSelectObject();

	//! set Left DBTree Enabled
	void setLeftDBTreeEnabled(bool enable);

	//! set View Button Enabled
	void setViewButtonDisabled(bool enable,bool onlyviewbutton = false);

	//! hide Properties dlg
	void hidePropertiesDlg();

	//! show Properties dlg
	void showPropertiesDlg(ccHObject* obj);

	//! Expands tree at a given node
	void expandElement(ccHObject* obj, bool state);

    //[!].判断节点是否展开
    bool getIsExpandElement(ccHObject* object);

	//! show 2d window
	void show2dWindow();

	//! show 3d window
	void show3dWindow();

    //! change To 2d View
    void changeTo2dView();

    void bindView(ccGLWindow* win);

    void setViewerActionState(bool state);

    void setViewerActionIcon(QString icon = "3d", QString text = "Viewer3D");

    //！get globle pointsize
    int getGloblePointSize();

	//! reset view
	void setGlobalZoom() override;

    //切换正视/透视投影
    void exchangePerspectiveView();

    //启用/禁用图像融合功能
    void setImageFusionEnable(bool enable);

    bool getStateUseOrthographic() { return m_stateUseOrthographic; }

    //! pObj add child pointcloud node
    void doActionAddChildObj(ccHObject* pObj);

    //! export pointcloud to .tiff file
    void doActionExportTiff();

	//! Returns active GL sub-window (if any)
	ccGLWindow* getActiveGLWindow() override;
	
	//! Returns MDI area subwindow corresponding to a given 3D view
	QMdiSubWindow* getMDISubWindow(ccGLWindow* win);

	//! Returns a given views
	ccGLWindow* getGLWindow(int index) const;

	//! Returns the number of 3D views
	int getGLWindowCount() const;


	//! Tries to load several files (and then pushes them into main DB)
	/** \param filenames list of all filenames
		\param fileFilter selected file filter (i.e. type)
		\param destWin destination window (0 = active one)
	**/
	virtual void addToDB( const QStringList& filenames,
						  QString fileFilter = QString(),
						  ccGLWindow* destWin = nullptr );
	
	//inherited from ccMainAppInterface
	void addToDB( ccHObject* obj,
				  bool updateZoom = false,
				  bool autoExpandDBTree = true,
				  bool checkDimensions = false,
				  bool autoRedraw = true ) override;
	
	void registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos) override;
	void unregisterOverlayDialog(ccOverlayDialog* dlg) override;
	void updateOverlayDialogsPlacement() override;
	void removeFromDB(ccHObject* obj, bool autoDelete = true) override;
	void setSelectedInDB(ccHObject* obj, bool selected) override;
	void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;
	void forceConsoleDisplay() override;
	ccHObject* dbRootObject() override;
	inline  QMainWindow* getMainWindow() override { return this; }
	ccHObject* loadFile(QString filename, bool silent) override;
	inline  const ccHObject::Container& getSelectedEntities() const override { return m_selectedEntities; }
	void createGLWindow(ccGLWindow*& window, QWidget*& widget) const override;
	void destroyGLWindow(ccGLWindow*) const override;
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;
	ccColorScalesManager* getColorScalesManager() override;
	void spawnHistogramDialog(	const std::vector<unsigned>& histoValues,
								double minVal, double maxVal,
								QString title, QString xAxisLabel) override;
	ccPickingHub* pickingHub() override { return m_pickingHub; }
	ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject* obj) override;
	void putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context) override;
	
	void putObjectIntoDBTreeByIndex(ccHObject* obj, const ccHObjectContext& context, const int & index) override;
	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;
	
	//! Returns real 'dbRoot' object
	virtual ccDBRoot* db();

	//! Adds the "Edit Plane" action to the given menu.
	/**
	 * This is the only MainWindow UI action used externally (by ccDBRoot).
	**/
	void  addEditPlaneAction( QMenu &menu ) const;
	
	//! Sets up the UI (menus and toolbars) based on loaded plugins
	void initPlugins();

	//! Updates the 'Properties' view
	void updatePropertiesView();

	/**
	 * @brief �޸�qdockwidget��ɫ�߿�
	 * @param dock���ںͱ���
	 * @return 
   */
	void changeDockWidgetTitle(QDockWidget * dockwidget,QString title);

    /**
    *@获取顶部导航
    */
    SARibbonBar* getMainTopNavigation(void) { return ribbon; };

	/**
	*@设置是否使用平行投影并锁定于解锁
	*/
	void setUseOrthographic(bool isuse);

    /**
    *@brief 获取Action对象
    */
    QAction* getMainWindowAction(const QString &strObjectName);

    /**
    *@brief 裁剪盒重置
    */
    void resetClippingBoxTool();

    /**
    *@brief 更新关闭全部按钮状态
    */
    void updateCloseAllButtonState();

public slots:
    //[!]状态栏
	void updateStatusbarInformation(QString positionstr);

	//[!]状态栏(预留类型)
	void updateStatusbarInformationType(QString positionstr,CS::Model::StatusMessageType);

	//坐标转换与点云配准预览槽函数
	void onPairRegistrationSignalPreview();

	//坐标转换与点云配准重置槽函数
	void onPairRegistrationSignalReset();

	/**
	*@brief ����ƽ���и��
	*/
	void slotShowPlaneCut();

    /**
    *@brief ע��Aciton
    */
    void slotActionNotes();

    void slotChangeView();

    /**
    *@brief 更新激活码注册Category
    */
    void slotUpdateRegisterCoderCategory(void);

	/**
  *@brief �������еĴ���
  */
	void slotRedrawAllWindows(void);

    void deactivatePointPickingMode(bool);

public://���ܷ�������
     /**
     *@brief ����ʰȡģʽ
     */
    void setPickMode(bool bEnable);

    /**
    *@brief 设置主导航是否能进行切换
    */
    void setMainWindowNavigationEnable(bool bEnable);

    /**
    *更新树选择
    */
    void updateUIWithSelection();

    //退出应用时更新菜单和项目树禁用状态，不刷新属性表
    void slotExitFunction(bool);

    //更新组合渲染菜单按钮
    void updateCombineColorMenuState();

	/**
	*@brief 
	*/
	void setCurrentActiveWindows(FJDragableMdiArea::MDIAREASHOWTYPE typle);
private:
	/**
	*@brief ¥����ȡ
	*/
	void doActionFloorExtraction();

	/**
	*@brief ��ѯ���ܹ�״̬
	*/
	void slotTimerCheckDog(void);

	/**
	*@brief ��Ȩ������ѯ
	*/
	void slotLicensingAciton(void);

	/**
     *@brief ��ʾʱ��
     */
	void slotShowCurrentTime(void);

	/**
	 *@brief 显示鼠标位置坐标
	 */
	void slotShowCurrentPos(CCVector3d);
   
   /**
    *@brief ��ǰ�����ע�ͷ����ı�
    */
    void slotActivatedNoteChanged(void);

	/**
	*@brief ����ת��
	*/
	void slotCoordinateConversion(void);

    /**
    *@brief ����ת��
    */
    void slotHeightFittingAction(void);

	/**
	*@brief EDL��ť
	*/
	void slotEDLControl(void);

	/**
	*@brief ��ɫ������
	*/
	void slotRibbonStripSetting(void);

	/**
	*@brief ����ƴ��
	*/
	void slotAccuracyVerification(void);

	/**
	*@brief 剖面分析
	*/
	void sloSectionAnalysisation(void);
    /**
    *@brief 密度测量
    */
    void slotmDensityMeasurementAciton(void);

    /**
    *@brief 列队处理
    */
    void slotQueueupForProcessing(void);

    /**
    *@brief 图像融合
    */
    void slotImageFusion(void);

	/**
	*@brief 等高线
	*/
	void slotContourAction(void);

    /**
    *@brief 动态物剔除
    */
    void slotDynamicObjectRemovalAction(void);

	/**
	*@brief 手动分类
	*/
	void slotManualClassificationDlg(void);

	/**
	*@brief 按类别提取
	*/
	void slotCategoryExtraction(void);

	/**
	*@brief 精细化类别提取按钮 longxin.miao 2023/9/21 针对树冠树木提取后分类操作
	*/
	void slotRefineCategoryExtraction(void);

	/**
	*@brief 按轨迹裁剪
	*/
	void slotTrackClipping(void);

	/**
	*@brief 真彩色
	*/
	void slotTrionPointcloudColoring(void);

	/**
	*@brief 相机参数对准
	*/
	void slotCameraPointAlignment(void);

    /**
    *@brief RTK数据解析
    */
    void slotRtkDataAnalysisAction(void);

	/**
	*@brief 手动分类关闭
	*/
	void deactivateManualClassification(bool);

	/**
	*@brief 倾斜拉直
	*/
	void slotAccuracyActiveZaxisLevel(void);
	
    /**
    *@biref ˢ�¼���Ĵ���
    */
    void slotRedrawActiveWindow(void);

    /**
    *@biref ˢ��2d����
    */
    void slotRedraw2DWindow(void);

  
    /**
    *@brief ����DB����model������
    */
    void slotRenewalDBTreeModel(void);
    /**
     *@ˢ�½���
     */
    void slotUpdateWindow(void);
    /**
    *@brief ��ǰRibbonTab�ı�
    */
    void slotCurrentRibbonTabChanged(int index);

	/**
	*@brief 更新平行投影或者透视投影方式
	*/
	void updatePerspectiveStateSlot(void);

    /**
    *@brief 界面错误信息提示
    */
    void signalErrorMessageSlot(QString str);

	/**
	*@brief ccHObject选中
	*/
	void measureLabelSelectedSlot(int uuid);

    /**
    *@brief 点云删除
    */
    void slotpointcloudRemoved(int uuid);

	/**
	*@brief 点云生成正射影像
	*/
	void slotUIGenerateOrthoimageChanged(void);

    /**
    *@brief 项目树最小化按钮
    */
    void onDockWidgetIconLeftBtn();

    /**
    *@brief 项目树最大化按钮
    */
    void onDockWidgetIconRightBtn();
    //[!]刷新在GLDBroot上的节点的所有点大小的状态
    void updateGlWindowCloudPointSize();

public slots:
	void SetViewLock();
	
private:
	/**
	*@brief 设置其他文件节点隐藏
	*/
	void setOtherFileObjectEnabled(ccHObject * self, bool isenabled);

	/**
	*@brief �����������ӽڵ��Ƿ�ѡ
	*/
	void setSelfAndSubObjectEnabled(ccHObject * self,bool isenabled);


	//���ð�ť��ʾ������ɫ
	void SetQActionShowNormal();

	//����ͼƬ·����ȡ���QIcon
	QIcon GetIconByPicName(const QString & path1, const QString & path2, const QString & path3);

	//! Creates a new 3D GL sub-window
	ccGLWindow* new3DView( bool allowEntitySelection, bool is3D = true);

	//! Zooms in (current 3D view)
	void zoomIn();
	//! Zooms out (current 3D view)
	void zoomOut();

	//! Displays 'help' dialog
	void doActionShowHelpDialog();
	//! Displays file open dialog
	void doActionLoadFile();
	//! Displays file save dialog
	void doActionSaveFile();
	//! Displays the Global Shift settings dialog
	void doActionGlobalShiftSeetings();
	//! Toggles the 'show Qt warnings in Console' option
	void doEnableQtWarnings(bool);

	//! Clones currently selected entities
	void doActionClone();

	void doActionInDoor();

    //2022.9.16__Aric.tang
    /**
    *@brief ���̱߳�������
    */
    void slotShowError(QString str);
	void doActionOutDoor();
	//void onDockablePropertiesChanged(Qt::DockWidgetAreas allowedAreas);
    //aric.tang_2022.10.14
    //tree_one by octree
    void doActionTerrain();
    void doActionTrees();

	//! Updates entities display target when a gl sub-window is deleted
	/** \param glWindow the window that is going to be delete
	**/
	void prepareWindowDeletion(QObject* glWindow);

	//! Slot called when the exclusive fullscreen mode is toggled on a window
	void onExclusiveFullScreenToggled(bool);

	//inherited from ccMainAppInterface
	void freezeUI(bool state) override;
	void redrawAll(bool only2D = false) override;
	void refreshAll(bool only2D = false) override;
	void enableAll() override;
	void disableAll() override;
	void disableAllBut(ccGLWindow* win) override;
	void updateUI() override;
	
	virtual void toggleActiveWindowStereoVision(bool);
	void toggleActiveWindowCenteredPerspective() override;
	void toggleActiveWindowCustomLight() override;
	void toggleActiveWindowSunLight() override;
	void toggleActiveWindowViewerBasedPerspective() override;
	void zoomOnSelectedEntities() override;
	
	void increasePointSize() override;
	void decreasePointSize() override;
	
	void toggleLockRotationAxis();
	void doActionEnableBubbleViewMode();
	void setPivotAlwaysOn();
	void setPivotRotationOnly();
	void setPivotOff();
	void toggleActiveWindowAutoPickRotCenter(bool);
	void toggleActiveWindowShowCursorCoords(bool);

	//! Handles new label
	void handleNewLabel(ccHObject*);

	void setActiveSubWindow(QWidget* window);
	void showDisplayOptions();
	void showSelectedEntitiesHistogram();
	void testFrameRate();
	void toggleFullScreen(bool state);
	void toggleVisualDebugTraces();
	void toggleExclusiveFullScreen(bool state);
	void update3DViewsMenu();
	void updateMenus();
	void on3DViewActivated(QMdiSubWindow*);
	void addToDBAuto(const QStringList& filenames);
	void echoMouseWheelRotate(float);
	void echoBaseViewMatRotation(const ccGLMatrixd& rotMat);
	void echoCameraPosChanged(const CCVector3d&);
	void echoPivotPointChanged(const CCVector3d&);

	void doActionRenderToFile();

	//menu action
	void doActionSetUniqueColor();
	void doActionColorize();
	void doActionRGBToGreyScale();
	void doActionSetColor(bool colorize);
	void doActionSetColorGradient();
	void doActionInterpolateColors();
	void doActionChangeColorLevels();
	void doActionEnhanceRGBWithIntensities();
	void doActionColorFromScalars();

	void doActionSFGaussianFilter();
	void doActionSFBilateralFilter();
	void doActionSFConvertToRGB();
	void doActionSFConvertToRandomRGB();
	void doActionRenameSF();
	void doActionOpenColorScalesManager();
	void doActionAddIdField();
    void doActionSplitCloudUsingSF();
	void doActionSetSFAsCoord();
	void doActionInterpolateScalarFields();

	void doComputeGeometricFeature();
	void doActionSFGradient();
	void doRemoveDuplicatePoints();
	void doSphericalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doCylindricalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doActionFitPlane();
	void doActionFitSphere();
	void doActionFitFacet();
	void doActionFitQuadric();
	void doShowPrimitiveFactory();

	void doActionComputeNormals();
	void doActionInvertNormals();
	void doActionConvertNormalsToHSV();
	void doActionConvertNormalsToDipDir();
	void doActionComputeOctree();
	void doActionComputeKdTree();
	void doActionApplyTransformation();
	void doActionMerge();
	void doActionRegister();
	void doAction4pcsRegister(); //Aurelien BEY le 13/11/2008
	void doActionSubsample(); //Aurelien BEY le 4/12/2008
	void doActionStatisticalTest();
	void doActionSamplePointsOnMesh();
	void doActionSamplePointsOnPolyline();
	void doActionSmoohPolyline();
	void doActionConvertTextureToColor();
	void doActionLabelConnectedComponents();
	void doActionComputeStatParams();
	void doActionFilterByValue();
	
	// Picking operations
	void enablePickingOperation(ccGLWindow* win, QString message);
	void cancelPreviousPickingOperation(bool aborted);

	// For rotation center picking
	void doPickRotationCenter();
	// For leveling
	void doLevel();

	void InitMainWindowStyle();
	
	void doActionCreatePlane();
	void doActionEditPlane();
	void doActionFlipPlane();
	void doActionComparePlanes();

	void doActionDeleteScanGrids();
	void doActionSmoothMeshSF();
	void doActionEnhanceMeshSF();
	void doActionAddConstantSF();
	void doActionAddClassificationSF();
	void doActionScalarFieldArithmetic();
	void doActionScalarFieldFromColor();
	void doActionOrientNormalsFM();
	void doActionOrientNormalsMST();
	void doActionResampleWithOctree();
	void doActionComputeMeshAA();
	void doActionComputeMeshLS();
	void doActionMeshScanGrids();
	void doActionComputeDistanceMap();
	void doActionComputeDistToBestFitQuadric3D();
	void doActionMeasureMeshSurface();
	void doActionMeasureMeshVolume();
	void doActionFlagMeshVertices();
	void doActionSmoothMeshLaplacian();
	void doActionSubdivideMesh();
	void doActionFlipMeshTriangles();
	void doActionComputeCPS();
	void doActionShowWaveDialog();
	void doActionCompressFWFData();
	void doActionKMeans();
	void doActionFrontPropagation();
	void doActionApplyScale();
	void doActionEditGlobalShiftAndScale();
	void doActionMatchBBCenters();
	void doActionMatchScales();
	void doActionSORFilter();
	void doActionFilterNoise();
	void doActionUnroll();
	void doActionCreateGBLSensor();
	void doActionCreateCameraSensor();
	void doActionModifySensor();
	void doActionProjectUncertainty();
	void doActionCheckPointsInsideFrustum();
	void doActionComputeDistancesFromSensor();
	void doActionComputeScatteringAngles();
	void doActionSetViewFromSensor();
	void doActionShowDepthBuffer();
	void doActionExportDepthBuffer();
	void doActionComputePointsVisibility();
	void doActionRasterize();
	void doCompute2HalfDimVolume();
	void doConvertPolylinesToMesh();
	void doMeshTwoPolylines();
	void doActionExportCoordToSF();
	void doActionExportNormalToSF();
	void doComputeBestFitBB();
	void doActionCrop();

	void doActionEditCamera();
	void doActionAdjustZoom();
	void doActionSaveViewportAsCamera();
	void doActionResetGUIElementsPos();
	void doActionResetAllVBOs();

	//Shaders & plugins
	void doActionLoadShader();
	void doActionDeleteShader();
	void dotractChangeTimer();
	void doActionFindBiggestInnerRectangle();
	void doActionAbout();
	bool findSub2dObject(ccHObject * self, bool & isFind);

	/**
	 * @brief ��ǿ����ɫ
	 * @param
	 * @return
	*/
	void doActionColorByIntensity();

	/**
	 * @brief �����ĵ���ť
	 * @param 
	 * @return 
    */
	void doActionhelpDocument();
	//Clipping box
	void activateClippingBoxMode();
	void deactivateClippingBoxMode(bool);

	//Graphical transformation
	void activateTranslateRotateMode();
	void deactivateTranslateRotateMode(bool);

	//Graphical segmentation
	void activateSegmentationMode();
	void deactivateSegmentationMode(bool);

	//Polyline tracing
	void activateTracePolylineMode();
	void deactivateTracePolylineMode(bool);

	//Section extraction
	void activateSectionExtractionMode();
	void deactivateSectionExtractionMode(bool);

	//Entities comparison
	void doActionCloudCloudDist();
	void doActionCloudMeshDist();
	void doActionCloudPrimitiveDist();
	void deactivateComparisonMode(int);

	//Point picking mechanism
	void activatePointPickingMode();

	//Point list picking mechanism
	void activatePointListPickingMode();
	void deactivatePointListPickingMode(bool);
	void doActionpreferencesetting();


	//Point-pair registration mechanism
	void activateRegisterPointPairTool();
	void deactivateRegisterPointPairTool(bool);


	//Current active scalar field
	void doActionToggleActiveSFColorScale();
	void doActionShowActiveSFPrevious();
	void doActionShowActiveSFNext();

	//! Removes all entities currently loaded in the DB tree
	void closeAll();

	//! Batch export some info from a set of selected clouds
	void doActionExportCloudInfo();
	//! Batch export some info from a set of selected planes
	void doActionExportPlaneInfo();

	//! Generates a matrix with the best (registration) RMS for all possible couple among the selected entities
	void doActionComputeBestICPRmsMatrix();

	//! Creates a cloud with the (bounding-box) centers of all selected entities
	void doActionCreateCloudFromEntCenters();

	//! Creates a cloud with a single point
	void createSinglePointCloud();
	//! Creates a cloud from the clipboard (ASCII) data
	void createPointCloudFromClipboard();

	inline void doActionMoveBBCenterToOrigin()    { doActionFastRegistration(MoveBBCenterToOrigin); }
	inline void doActionMoveBBMinCornerToOrigin() { doActionFastRegistration(MoveBBMinCornerToOrigin); }
	inline void doActionMoveBBMaxCornerToOrigin() { doActionFastRegistration(MoveBBMaxCornerToOrigin); }

	//! Restores position and state of all GUI elements
	void restoreGUIElementsPos();

	void deactivatePointPickingPositionMode(bool);
	//void activeVolumeMeasureMode();
	//void deactivateMeasureVolumeMode(bool);

	void updateSegmentStateSlot(bool isopen);

private:
	//! Shortcut: asks the user to select one cloud
	/** \param defaultCloudEntity a cloud to select by default (optional)
		\param inviteMessage invite message (default is something like 'Please select an entity:') (optional)
		\return the selected cloud (or null if the user cancelled the operation)
	**/
	ccPointCloud* askUserToSelectACloud(ccHObject* defaultCloudEntity = nullptr, QString inviteMessage = QString());

	enum FastRegistrationMode
	{
		MoveBBCenterToOrigin,
		MoveBBMinCornerToOrigin,
		MoveBBMaxCornerToOrigin
	};

	void doActionFastRegistration(FastRegistrationMode mode);

	void toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY property );
	void clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY property );
	
	void setView( CC_VIEW_ORIENTATION view ) override;
	
	//! Apply transformation to the selected entities
	void applyTransformation(const ccGLMatrixd& transMat);

	//! Creates point clouds from multiple 'components'
	void createComponentsClouds(ccGenericPointCloud* cloud,
								CCCoreLib::ReferenceCloudContainer& components,
								unsigned minPointPerComponent,
								bool randomColors,
								bool selectComponents,
								bool sortBysize = true);

	//! Saves position and state of all GUI elements
	void saveGUIElementsPos();

	void setOrthoView(ccGLWindow* win);
	void setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw = true);
	void setViewerPerspectiveView(ccGLWindow* win);
	
	void showEvent(QShowEvent* event) override;
	void closeEvent(QCloseEvent* event) override;
	void moveEvent(QMoveEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	bool eventFilter(QObject *obj, QEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;

	virtual void paintEvent(QPaintEvent *event);

	
	//! Makes the window including an entity zoom on it (helper)
	void zoomOn(ccHObject* object);

	//! Active SF action fork
	/** - action=0 : toggle SF color scale
		- action=1 : activate previous SF
		- action=2 : activate next SF
		\param action action id
	**/
	void doApplyActiveSFAction(int action);

	//! Mesh computation fork
	/** \param type triangulation type
	**/
	void doActionComputeMesh(CCCoreLib::TRIANGULATION_TYPES type);

    void doActionFillHoles();//补洞

	//! Computes the orientation of an entity
	/** Either fit a plane or a 'facet' (2D polygon)
	**/
	void doComputePlaneOrientation(bool fitFacet);

	//! Sets up any input devices (3D mouse, gamepad) and adds their menus
	void setupInputDevices();
	//! Stops input and destroys any input device handling
	void destroyInputDevices();

	//! Connects all QT actions to slots
	void connectActions();

	//! Enables menu entires based on the current selection
	void enableUIItems(dbTreeSelectionInfo& selInfo);

	//! Updates the view mode pop-menu based for a given window (or an absence of!)
	void updateViewModePopUpMenu(ccGLWindow* win);

	//! Updates the pivot visibility pop-menu based for a given window (or an absence of!)
	void updatePivotVisibilityPopUpMenu(ccGLWindow* win);

	//! Checks whether stereo mode can be stopped (if necessary) or not
	bool checkStereoMode(ccGLWindow* win);

	//! Adds a single value SF to the active point cloud
	void addConstantSF(ccPointCloud* cloud, QString sfName, bool integerValue);

	//Ribbon janson.yang

	void initGraphicInterface();
	/**
	 * @brief �޸�QAction�����������ʧЧ����״̬ͼ��
	 * @param 
	*/
	void setActionIcon(QAction * action,const QString& normalPix,const QString& clickedPix, const QString& disabledPix);
    /**
     * @brief �޸�QAction�����������ʧЧ����״̬ͼ�� ���̶���С��
     * @param
     */
    void setActionIcon(QAction * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix, QSize& size);


private:
	//设置菜单按钮是否可用
	void setMenuButtonEnable(QString ribbonObjName, QString pannelObjName, QString buttonObjName, bool isenable);
	void createRibbonMenu();
	void createCategoryEdit(SARibbonCategory* page);
	void createCategoryView(SARibbonCategory* page);
	void createCategoryStart(SARibbonCategory* page);
	void createCategoryDrafting(SARibbonCategory* page);

	void createCategoryAlgorithm(SARibbonCategory* page);
	QAction* createAction(const QString& text, const QString& iconurl, const QString& objName);
	QAction* createAction(const QString& text, const QString& iconurl);
	QAction* createAction(const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName);
	void resetActionIcon(QAction* act, const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName);

	void createQuickAcdessBar(SARibbonQuickAccessBar* quickAccessBar);
	void createRightButtonGroup(SARibbonButtonGroupWidget* rightBar);

	void initActionsMenus();//��ʼ���Զ����action��menu

    void createRenderViewers(); //����3d��2d��Ⱦ����

    // z轴倾斜自动拉平 janson
    void toggleActiveZaxisLevel();
    // 1.树冠树干分类 2.整体植被提取 longxin.miao
    void toggleCallCanupoProcessClassify(void);
	void toggleCallCanupoProcessClassify_AutoSeg(void);
    // 单木新增测试 longxin.miao 2023/10/9
    void toggleCallIndividualTreeAddition(void);
private:
	//�޸ĵײ�״̬������
	void reviseStatusBarAttributes();
    //����Ƕ��ͨ��Dockwidget
    void createEmbeddedDockWidget();
private: 
    //[!]保存所有已打开文件的路径
    QStringList m_IsOpenFilesPath;
	//members
	QTimer m_TimerReadDog;
	//! Main UI
	Ui::MainWindow* m_UI;
	
    ccHObject* m_pLastObject;

    ccGLMatrixd m_Matd;
	//! DB tree
	ccDBRoot* m_ccRoot;

	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! Recent files menu
	ccRecentFiles* m_recentFiles;
	
	//! 3D mouse
	cc3DMouseManager* m_3DMouseManager;

	//! Gamepad handler
	ccGamepadManager* m_gamepadManager;

	//! View mode pop-up menu button
	QToolButton* m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton* m_pivotVisibilityPopupButton;

	QToolButton* m_2D3DPopupButton;

	//! Flag: first time the window is made visible
	bool m_firstShow;

	//! Point picking hub
	ccPickingHub* m_pickingHub;

	/******************************/
	/***        MDI AREA        ***/
	/******************************/

	FJDragableMdiArea * m_mdiArea;
	/**
	 * @brief ���������ⲿ����
	*/
	QWidget * m_outmdiArea;

	/**
	 * @brief ����������ߴ���
	*/
	QScrollArea * m_leftContainWidget;

	//! CloudCompare MDI area overlay dialogs
	struct ccMDIDialogs
	{
		ccOverlayDialog* dialog;
		Qt::Corner position;

		//! Constructor with dialog and position
		ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
			: dialog(dlg)
			, position(pos)
		{}
	};

	//! Repositions an MDI dialog at its right position
	void repositionOverlayDialog(ccMDIDialogs& mdiDlg);

	//! Registered MDI area 'overlay' dialogs
	std::vector<ccMDIDialogs> m_mdiDialogs;

	/*** dialogs ***/
	//! Camera params dialog
	ccCameraParamEditDlg* m_cpeDlg;
	//! Graphical segmentation dialog
	ccGraphicalSegmentationTool* m_gsTool;
	//! 手动分类界面
	FJManualClassificationDlg* m_manualClassificationTool;
	//! Polyline tracing tool
	ccTracePolylineTool * m_tplTool;
	//! Section extraction dialog
	ccSectionExtractionTool* m_seTool;
	//! Graphical transformation dialog
	ccGraphicalTransformationTool* m_transTool;
	//! Clipping box dialog
	ccClippingBoxTool* m_clipTool;
	//! Cloud comparison dialog
	ccComparisonDlg* m_compDlg;
	//! Point properties mode dialog
	ccPointPropertiesDlg* m_ppDlg;
	//! Point list picking
	ccPointListPickingDlg* m_plpDlg;
	//! Point-pair registration
	ccPointPairRegistrationDlg* m_pprDlg;
	//! Primitive factory dialog
	ccPrimitiveFactoryDlg* m_pfDlg;
	

    PointSizedialog * m_psWidget;
	//! ������֤����
	ccPointAccuracyVerificationDlg* m_pavDlg;
	//janson.yang
	ccPointPickingPosition* m_ppp;
	//ccVolumeMeasureTool* m_volumeTool;

	/*** plugins ***/
	//! Manages plugins - menus, toolbars, and the about dialog
	ccPluginUIManager	*m_pluginUIManager;

	//Ribbon janson.yang
	SARibbonBar* ribbon;
	//�༭
	////��Ŀ
	QAction* coor_transform_action_;//����ת��

	QAction* colorByIntensityAction;//��ǿ����ɫ

	////����
	QAction* outRoomClass_action_;//�������
	QAction* inRoomClass_action_;//���ڷ���
	QAction* floorextractionClass_action_;//¥����ȡ
    QAction *actionPlaneCut; //ƽ���и�
    QAction *actionNotes; //�ʼ�
    //aric.tang_2022.10.14__trees
    QAction *forestTerrainAndVege_action_;
    QAction *forestRestAndTrees_action_;
    //���ƽ�ͼ
    QAction *actionPointMap; //ƽ���и�
	//�ӽǲ˵�
	QMenu* viewEye_;
	//����ɫ�˵�
	QMenu* m_pointcolormenu;

	QAction *m_actionComputeSurfaceMesh; 	//表面三角网
	QAction *m_actionAccuracyActiveAxisZ;	//倾斜拉直
    QAction *m_actionFillHoles; 	//补洞

	//�㷨

	//QMenu*   menuLanguage_;
	
	QLineEdit* searchLineEidt_;//������

	//�û�����
	QAction* about_action;

	//�û�����
	QAction* m_helpDocument_action;

	//ccԭ�е�menu
	QMenuBar* menubar;
	QMenu *menuFile;
	QMenu *menuDisplay;
	QMenu *menuToolbars;
	QMenu *menuLights;
	QMenu *menuActiveScalarField;
	QMenu *menuLanguage;
	QMenu *menuHelp;
	QMenu *menuEdit;
	QMenu *menuScalarFields;
	QMenu *menuColors;
	QMenu *menuNormals;
	QMenu *menuOrientNormals;
	QMenu *menuConvert_to;
	QMenu *menuOctree;
	QMenu *menuMesh;
	QMenu *menuMeshScalarField;
	QMenu *menuSensors;
	QMenu *menuGroundBasedLidar;
	QMenu *menuCameraSensor;
	QMenu *menuToggle;
	QMenu *menuWaveform;
	QMenu *menuPlane;
	QMenu *menuGrid;
	QMenu *menuPolyline;
	QMenu *menuCloud;
	QMenu *menu3DViews;
	QMenu *menuTools;
	QMenu *menuSegmentation;
	QMenu *menuProjection;
	QMenu *menuStatistics;
	QMenu *menuDistances;
	QMenu *menuRegistration;
	QMenu *menuOther;
	QMenu *menuSandBox;
	QMenu *menuFit;
	QMenu *menuClean;
	QMenu *menuVolume;
	QMenu *menuBatchExport;

	int m_leftTreeWidgetWidth = 200;//左侧项目树窗口宽度
    QWidget* m_titleWidgettree;
    QToolButton * m_treeMinButton;
    QAction * m_actionCompute2HalfDimVolume;//两期对比
    QAction * m_DynamicObjectRemovalAction;//动态物剔除
	QAction * m_ContourAction;//等高线按钮
    QAction * m_QueueupForProcessingAction;//列队处理
    QAction * m_ImageFusionAction;//图像融合
	QString m_DirName;
	QAction * mlsaction;
	QAction * pcvaction;
	QAction	* m_LicensingAciton;
    QAction * csfAction;
	QAction * m_EDLAction; //EDL��ť
	QAction * m_ribbonStripAction; //ɫ��������
	QAction * m_coordinateConversionAciton;
	QAction * m_AccuracyverificationAciton;
	QAction * m_SectionAnalysisAciton;

    QAction * m_pHeightFittingAction = nullptr;
	
    QAction *m_pViewerAction; 
	QAction *m_PerspectiveAction; //切换正式投影和透视投影按钮
	QAction *m_pCreatePointCloudOrthphotoAction = nullptr;     ///< 点云生成正射影像


	QTimer m_tractChangeTimer;
private:
	bool m_stateUseOrthographic; //记录旧的投影方式
	bool m_isClippingBoxOpen = false; // 裁剪盒功能是否打开
	bool m_isClippingBoxCanCombination = false; // 裁剪盒功能是否可以组合
	QLabel *m_pShowLockTime = nullptr;
	QLabel *m_pLockTime = nullptr;
	QLabel *m_pCurrentTime = nullptr;
	QLabel *m_pShowCurrentTime = nullptr;
	QTimer *m_pTime = nullptr;
	QDateTime time;

    QLabel *m_pShowOffsetPos = nullptr;
    QLabel *m_pShowPointNumberPos = nullptr;
	QLabel *m_pShowCurrentPos = nullptr;

	//ͨ��dockwidget����
	CS::Widgets::ICenterWidget *m_pDockCenterWidget = nullptr;
	//MainWindow����Dockwidget
	QDockWidget *m_pDockWidgetPublic = nullptr;
	CS::Widgets::pdfDialog* m_pPdfoutDlg;

	//planecuting ����״̬
	bool m_PlaneStatus;
	bool m_isPerspectiveChangeLock = false;

	//[!]保存当前窗口的属性，在resizEvent中刷新。首次启动时，获取属性
	QRect m_rect;
	std::vector<CCVector3d> m_timeToTrackVec; //差值后的轨迹点
	std::vector<CCVector3d> m_trackDirectionVec; //差值后的轨迹点方向
	int m_currentIndex = 0;//当前轨迹点序号
};

#endif
