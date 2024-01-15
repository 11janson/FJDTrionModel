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


#include "mainwindow.h"
#include <QScreen>
#include <ShlObj_core.h>


//CCCoreLib Includes
#include <CloudSamplingTools.h>
#include <Delaunay2dMesh.h>
#include <Jacobi.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <ccGuiParameters.h>

//for tests
#include <ChamferDistanceTransform.h>
#include <SaitoSquaredDistanceTransform.h>

//qCC_db
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <ccCameraSensor.h>
#include <ccColorScalesManager.h>
#include <ccCylinder.h>
#include <ccFacet.h>
#include <ccFileUtils.h>
#include <ccGBLSensor.h>
#include <ccImage.h>
#include <ccKdTree.h>
#include <ccPlane.h>
#include <ccProgressDialog.h>
#include <ccQuadric.h>
#include <ccSphere.h>
#include <ccSubMesh.h>
#include <ccClipBox.h>


//qCC_io
#include <ccShiftAndScaleCloudDlg.h>
#include <BinFilter.h>
#include <AsciiFilter.h>
#include <DepthMapFileFilter.h>

//QCC_glWindow
#include <ccGLWindow.h>
#include <ccRenderingTools.h>


//ccqfc
#include <framelessfiledialog.h>
#include <framelessmessagebox.h>
#include "cswidgets/waitingdialog.h"


//local includes
#include "ccConsole.h"
#include "ccEntityAction.h"
#include "ccHistogramWindow.h"
#include "ccInnerRect2DFinder.h"

//common
#include <ccPickingHub.h>
//common dialogs
#include <ccCameraParamEditDlg.h>
#include <ccDisplayOptionsDlg.h>
#include <ccPickOneElementDlg.h>
#include <ccStereoModeDlg.h>

//dialogs
#include "ccAboutDialog.h"
#include "ccAdjustZoomDlg.h"
#include "ccAlignDlg.h"
#include "ccApplication.h"
#include "ccApplyTransformationDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccBoundingBoxEditorDlg.h"
#include "ccCamSensorProjectionDlg.h"
#include "ccClippingBoxTool.h"
#include "ccColorFromScalarDlg.h"
#include "ccColorScaleEditorDlg.h"
#include "ccComparisonDlg.h"
#include "ccPrimitiveDistanceDlg.h"
#include "ccFilterByValueDlg.h"
#include "ccGBLSensorProjectionDlg.h"
#include "ccGeomFeaturesDlg.h"
#include "ccGraphicalSegmentationTool.h"
#include "ccGraphicalTransformationTool.h"
#include "ccItemSelectionDlg.h"
#include "ccLabelingDlg.h"
#include "ccMatchScalesDlg.h"
#include "ccNoiseFilterDlg.h"
#include "ccOrderChoiceDlg.h"
#include "ccPlaneEditDlg.h"
#include "ccPointListPickingDlg.h"
#include "ccPointPairRegistrationDlg.h"
#include "ccPointPropertiesDlg.h"
#include "ccPrimitiveFactoryDlg.h"
#include "ccPtsSamplingDlg.h"
#include "ccRasterizeTool.h"
#include "ccRegistrationDlg.h"
#include "ccRenderToFileDlg.h"
#include "ccScaleDlg.h"
#include "ccSectionExtractionTool.h"
#include "ccSensorComputeDistancesDlg.h"
#include "ccSensorComputeScatteringAnglesDlg.h"
#include "ccSORFilterDlg.h"
#include "ccSubsamplingDlg.h"
#include "ccTracePolylineTool.h"
#include "ccTranslationManager.h"
#include "ccUnrollDlg.h"
#include "ccWaveformDialog.h"
#include "ccEntitySelectionDlg.h"
#include "ccSmoothPolylineDlg.h"
#include "ccSmoothMeshLaplacianInputDlg.h"
#include "ccMergeQuestionDlg.h"
#include "C2DFlatDrawProxy.h"
#include "ccSurfaceMeshInputDlg.h"
#include "ccFillHolesInputDlg.h"
#include "ccTriangleNetDlg.h"
#include "ccFillHoleDlg.h"
#include "ccVolDisplayDlg.h"
#include "cc2DLabel.h"




//other
#include "ccCropTool.h"
#include "ccGLPluginInterface.h"
#include "ccPersistentSettings.h"
#include "ccRecentFiles.h"
#include "ccRegistrationTools.h"
#include "ccUtils.h"
#include "db_tree/ccDBRoot.h"
#include "pluginManager/ccPluginUIManager.h"
#include "ccPreferencesettingsDlg.h"
#include "outdoorwindowdialog.h"
#include "indoorwindowdialog.h"
#include "floorextractiondialog.h"
#include "cloudcomparewidgets/metahubframelesswaitdialog.h"
#include "cloudcomparewidgets/metahubabout.h"
#include "cloudcomparewidgets/metahubmessbox.h"
#include "cloudcomparewidgets/metahubnotedialog.h"

#include"quazip/JlCompress.h"

#include "libs/cloudcompareutils/exportdatatofile.h"
#include "cloudcompareutils/publicutils.h"
#include "createpointcloudorthoimagedialog.h"

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include "cc3DMouseManager.h"
#endif

//Gamepads
#ifdef CC_GAMEPAD_SUPPORT
#include "ccGamepadManager.h"
#endif

#ifdef CC_CORE_LIB_USES_TBB
#include <tbb/tbb_stddef.h>
#endif

//Qt
#include <QClipboard>
#include <QtConcurrent>
#include <QFuture>
#include <QHBoxLayout>


//Qt UI files
#include <ui_distanceMapDlg.h>
#include <ui_globalShiftSettingsDlg.h>
#include <ui_mainWindow.h>

//System
#include <iostream>
#include <random>


//Ribbon
#include "SARibbonBar.h"
#include "SARibbonTabBar.h"
#include "SARibbonCategory.h"
#include "SARibbonPannel.h"
#include "SARibbonToolButton.h"
#include "SARibbonMenu.h"
#include "SARibbonComboBox.h"
#include "SARibbonLineEdit.h"
#include "SARibbonGallery.h"
#include "SARibbonCheckBox.h"
#include "SARibbonQuickAccessBar.h"
#include "SARibbonButtonGroupWidget.h"
#include "SARibbonApplicationButton.h"
#include "SARibbonCustomizeWidget.h"
#include "SARibbonElementManager.h"
#include "SARibbonCustomizeDialog.h"
#include "SAFramelessHelper.h"
#include "ccTriangulateInputDlg.h"
#include "PointSizedialog.h"
//janson.yang
//#include "ccPointPickingPosition.h"
//#include "ccVolumeMeasureTool.h"

//pcl
#include <cc2sm.h>

using namespace std;

#include "modelcontrol/cchobjectcontrol.h"
#include "FJStyleManager.h"
#include "QPdfWidget"
#include "ccScalarField.h"
#include <algorithm>
#include "measuremeshvolumedlg.h"
#include "ccBBox.h"
#include "ccGLMatrix.h"
#include "trionaboutdlg.h"
#include"importantinformationtips.h"
#include "trioncontourdlg.h"
#include "pointclouddatainputdlg.h"
//global static pointer (as there should only be one instance of MainWindow!) 
static MainWindow* s_instance = nullptr;
using namespace Utils;
//default file filter separator
static const QString s_fileFilterSeparator(";;");

#define EPSILON 0.00000001

enum PickingOperation {
	NO_PICKING_OPERATION,
	PICKING_ROTATION_CENTER,
	PICKING_LEVEL_POINTS,
};
static ccGLWindow* s_pickingWindow = nullptr;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = nullptr;
static ccHObject* s_levelEntity = nullptr;

MainWindow::MainWindow(QWidget* parent)
    : SARibbonMainWindow(parent, true)
    , m_UI(new Ui::MainWindow)
    , m_ccRoot(nullptr)
    , m_uiFrozen(false)
    , m_recentFiles(new ccRecentFiles(this))
    , m_3DMouseManager(nullptr)
    , m_gamepadManager(nullptr)
    , m_viewModePopupButton(nullptr)
    , m_pivotVisibilityPopupButton(nullptr)
    , m_firstShow(true)
    , m_pickingHub(nullptr)
    , m_cpeDlg(nullptr)
    , m_gsTool(nullptr)
    , m_manualClassificationTool(nullptr)
    , m_tplTool(nullptr)
    , m_seTool(nullptr)
    , m_transTool(nullptr)
    , m_clipTool(nullptr)
    , m_compDlg(nullptr)
    , m_ppDlg(nullptr)
    , m_plpDlg(nullptr)
    , m_pprDlg(nullptr)
    , m_pavDlg(nullptr)
    , m_pfDlg(nullptr)
    , m_ppp(nullptr)
    , ribbon(nullptr)
    , mlsaction(nullptr)
    , pcvaction(nullptr)
    , m_PerspectiveAction(nullptr)
    , m_pLastObject(nullptr)
    , m_PlaneStatus(false)
{
	m_UI->setupUi(this);
	//addbywangwenyu
	m_DirName = QCoreApplication::applicationDirPath();
	InitMainWindowStyle();
    //[!]设置全局默认显示文件打开保存对话框
    qApp->setProperty("LoadOrSaveFileDialogDisplay", true);
	//janson.yang
	initActionsMenus();

	//setWindowTitle(QStringLiteral("CloudCompare v") + ccApp->versionLongStr(false));
	qRegisterMetaType<CCVector3d>("CCVector3d");
	qRegisterMetaType<CCVector3d>("CCVector3d&");

	m_pluginUIManager = new ccPluginUIManager(this, this);
	ccTranslationManager::Get().populateMenu(menuLanguage, ccApp->translationPath());

#ifdef Q_OS_MAC
	m_UI->actionAbout->setMenuRole(QAction::AboutRole);
	m_UI->actionAboutPlugins->setMenuRole(QAction::ApplicationSpecificRole);

	m_UI->actionFullScreen->setText(tr("Enter Full Screen"));
	m_UI->actionFullScreen->setShortcut(QKeySequence(Qt::CTRL + Qt::META + Qt::Key_F));
#endif

	// Set up dynamic menus
	menuFile->insertMenu(m_UI->actionSave, m_recentFiles->menu());
	CS::Widgets::WaitingDialog::instance(this);
	//此处添加状态栏
	reviseStatusBarAttributes();
    //此处增加通用嵌入DockWidget
    createEmbeddedDockWidget();
    //[!]关键信息显示载入
    connect(QMainWindow::statusBar(), &QStatusBar::messageChanged, this, &MainWindow::updateStatusbarInformation);
    //[!]载入刷新属性表
    connect(this, &MainWindow::signalQueueRefreshPropertiesView, this, &MainWindow::updateUI, Qt::QueuedConnection);
    //[!]获取屏幕属性
    m_rect = QApplication::desktop()->availableGeometry();
    //[!].初始化model
    CS::Model::ProjectModel::instance();
    CS::Model::ProjectModel::instance()->setMainWindow(this);
	//[!]链接Model状态栏信号到主框架
	connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalStatusMessage, this, &MainWindow::updateStatusbarInformationType);
    //[!]链接第三方插件，传递cloud来源路径
    connect(this,&MainWindow::signalDBTreeCloudFromPath, CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::updateDBTreeCloudFromPath);
    //[!]保存树上文件到本地
    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalSaveCloudToLocal,this , &MainWindow::threadSaveLasFile);

	//ccConsole::Init(m_UI->consoleWidget, this, this);
	ccConsole::TheInstance()->setAutoRefresh(false);//显示console
	//此处添加日志输出路径addbycarl.wang
	ccConsole::TheInstance()->setLogFile(m_DirName + "/theme/FJDTrionMetahub.log");

	m_UI->actionEnableQtWarnings->setChecked(ccConsole::QtMessagesEnabled());

	//advanced widgets not handled by QDesigner
	{

		{
            QSize size(24,24);
            
			setActionIcon(m_UI->actionGlobalZoom,"actionGlobalZoom","actionGlobalZoomClicked","actionGlobalZoom", size);
			setActionIcon(m_UI->actionViewlock, "actionViewlock", "actionViewHover", "actionViewlock", size);
			setActionIcon(m_UI->actionPickRotationCenter, "actionPickRotationCenter", "actionPickRotationCenterClicked", "actionPickRotationCenter", size);
			setActionIcon(m_UI->actionSetViewTop, "actionSetViewTop", "actionSetViewTopClicked", "actionSetViewTopDisable", size);
			setActionIcon(m_UI->actionSetViewFront, "actionSetViewFront", "actionSetViewFrontClicked", "actionSetViewFrontDisable", size);
			setActionIcon(m_UI->actionSetViewLeft, "actionSetViewLeft", "actionSetViewLeftClicked", "actionSetViewLeftDisable", size);
			setActionIcon(m_UI->actionSetViewBack, "actionSetViewBack", "actionSetViewBackClicked", "actionSetViewBackDisable", size);
			setActionIcon(m_UI->actionSetViewRight, "actionSetViewRight", "actionSetViewRightClicked", "actionSetViewRightDisable", size);
			setActionIcon(m_UI->actionSetViewBottom, "actionSetViewBottom", "actionSetViewBottomClicked", "actionSetViewBottomDisable", size);
			setActionIcon(m_UI->actionSetViewIso1, "actionSetViewIso1", "actionSetViewIso1Clicked", "actionSetViewIso1Disable", size);
			setActionIcon(m_UI->actionSetViewIso2, "actionSetViewIso2", "actionSetViewIso2Clicked", "actionSetViewIso2Disable", size);
			setActionIcon(m_UI->actionSegment, "segmenticon", "segmenticonselect", "segmenticondisable", size);
			setActionIcon(m_UI->actionPointPicking, "measureicon", "measureiconselect", "measureicondisable", size);

            m_UI->actionGlobalZoom->setObjectName("actionGlobalZoom");
            m_UI->actionViewlock->setObjectName("actionViewlock");
            m_UI->actionSetViewTop->setObjectName("actionViewTop");
            m_UI->actionSetViewFront->setObjectName("actionViewFront");
            m_UI->actionSetViewLeft->setObjectName("actionViewLeft");
            m_UI->actionSetViewBack->setObjectName("actionViewBack");
            m_UI->actionSetViewRight->setObjectName("actionViewRight");
            m_UI->actionSetViewBottom->setObjectName("actionViewBottom");
            m_UI->actionSetViewIso1->setObjectName("actionViewIso1");
            m_UI->actionSetViewIso2->setObjectName("actionViewIso2");
            m_UI->actionSegment->setObjectName("actionSegment");
            m_UI->actionPointPicking->setObjectName("actionPointPicking");
			//投影切换按钮
			m_PerspectiveAction = createAction(QCoreApplication::translate("MainWindow", "Perspective projection", nullptr), "", "PerspectiveAction");
			setActionIcon(m_PerspectiveAction,"perspectiveIcon","perspectiveIcon","perspectiveIcon");
			QObject::connect(m_PerspectiveAction, &QAction::triggered, [this]() {
				toggleActiveWindowCenteredPerspective();
			});

			//3d窗口
			QString action_text = QCoreApplication::translate("MainWindow", "Viewer3D", nullptr);
			QString action_icon_path = QString::fromUtf8("");
			QString objectName = "viewerAction";
			m_pViewerAction = createAction(action_text, action_icon_path, objectName);
			setActionIcon(m_pViewerAction, "3d", "3d", "3d");
            QObject::connect(m_pViewerAction, &QAction::triggered, this, &MainWindow::slotChangeView);
            m_UI->toolBarView->insertAction(m_UI->actionViewlock, m_pViewerAction);
            m_UI->toolBarView->insertAction(m_UI->actionViewlock, m_PerspectiveAction);

			////2d窗口
			//action_text = QCoreApplication::translate("MainWindow", "Viewer2D", nullptr);
			//action_icon_path = QString::fromUtf8("");
			//objectName = "viewer2dAction";
			//m_pViewer2dAction = createAction(action_text, action_icon_path, objectName);
			//setActionIcon(m_pViewer2dAction, "viewer2dAction", "viewer2dAction", "viewer2dAction");

			//QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dAction.png");
			//pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dActionClicked.png"), QIcon::Active, QIcon::Off);
			//pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dAction.png"), QIcon::Disabled, QIcon::Off);
			//m_2D3DPopupButton = new QToolButton();
			//m_2D3DPopupButton->setIcon(pIcon);
			//QMenu* viewmenu = new QMenu(m_2D3DPopupButton);
			//viewmenu->addAction(m_pViewer3dAction);
			//viewmenu->addAction(m_pViewer2dAction);

			//m_2D3DPopupButton->setMenu(viewmenu);
			//m_2D3DPopupButton->setPopupMode(QToolButton::InstantPopup);
			//m_2D3DPopupButton->setToolTip(QCoreApplication::translate("MainWindow", "Viewer3D", nullptr));
			//m_2D3DPopupButton->setStatusTip(m_2D3DPopupButton->toolTip());
			//m_UI->toolBarView->insertWidget(m_UI->actionViewlock, m_2D3DPopupButton);
			
			
			m_UI->toolBarView->removeAction(m_UI->actionDisplayOptions);
			m_UI->toolBarView->removeAction(m_UI->actionEditCamera);
			m_UI->toolBarView->removeAction(m_UI->actionAutoPickRotationCenter);
			m_UI->toolBarView->removeAction(m_UI->actionLevel);
			m_UI->toolBarView->removeAction(m_UI->actionZoomAndCenter);
			m_UI->toolBarView->removeAction(m_UI->actionEnableStereo);
			m_UI->toolBarView->removeAction(m_UI->actionPickRotationCenter);
            m_UI->toolBarView->addSeparator();
			m_UI->toolBarView->addAction(m_UI->actionPointPicking);
			m_UI->toolBarView->addAction(m_UI->actionSegment);
		}
		//view mode pop-up menu
		{
			m_viewModePopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_viewModePopupButton);
			menu->addAction(m_UI->actionSetOrthoView);
			menu->addAction(m_UI->actionSetCenteredPerspectiveView);
			menu->addAction(m_UI->actionSetViewerPerspectiveView);

			m_viewModePopupButton->setMenu(menu);
			m_viewModePopupButton->setPopupMode(QToolButton::InstantPopup);
			m_viewModePopupButton->setToolTip(tr("Set current view mode"));
			m_viewModePopupButton->setStatusTip(m_viewModePopupButton->toolTip());
			//m_UI->toolBarView->insertWidget(m_UI->actionZoomAndCenter, m_viewModePopupButton);
			m_viewModePopupButton->setEnabled(true);
		}

		//pivot center pop-up menu
		{
			m_pivotVisibilityPopupButton = new QToolButton();
			QMenu* menu = new QMenu(m_pivotVisibilityPopupButton);
			menu->addAction(m_UI->actionSetPivotAlwaysOn);
			menu->addAction(m_UI->actionSetPivotRotationOnly);
			menu->addAction(m_UI->actionSetPivotOff);

			m_pivotVisibilityPopupButton->setMenu(menu);
			m_pivotVisibilityPopupButton->setPopupMode(QToolButton::InstantPopup);
			m_pivotVisibilityPopupButton->setToolTip(tr("Set pivot visibility"));
			m_pivotVisibilityPopupButton->setStatusTip(m_pivotVisibilityPopupButton->toolTip());
			//m_UI->toolBarView->insertWidget(m_UI->actionZoomAndCenter, m_pivotVisibilityPopupButton);
			m_pivotVisibilityPopupButton->setEnabled(false);
		}
	}
	//tabifyDockWidget(DockableDBTree,DockableProperties);

	//db-tree
	{
		m_ccRoot = new ccDBRoot(m_UI->dbTreeView, nullptr, this);
		m_UI->dbTreeView->setFocusPolicy(Qt::NoFocus);
        //m_UI->propertiesTreeView->setVisible(false);
		//m_UI->propertiesTreeView->setFocusPolicy(Qt::NoFocus);

		connect(m_ccRoot, &ccDBRoot::selectionChanged, this, &MainWindow::updateUIWithSelection, Qt::QueuedConnection);
		connect(m_ccRoot, &ccDBRoot::signalSaveFile, this, &MainWindow::doActionSaveFile);
		connect(m_ccRoot, &ccDBRoot::dbIsEmpty, this, [=]() { updateUIWithSelection(); updateMenus(); }, Qt::QueuedConnection); //we don't call updateUI because there's no need to update the properties dialog
		connect(m_ccRoot, &ccDBRoot::dbIsNotEmptyAnymore, this, [=]() { updateUIWithSelection(); updateMenus(); }, Qt::QueuedConnection); //we don't call updateUI because there's no need to update the properties dialog
	}

	//MDI Area
	{
		m_outmdiArea = new QWidget(this);
		QHBoxLayout * layout = new QHBoxLayout;
		layout->setMargin(0);
		layout->setSpacing(0);
		m_leftContainWidget = new QScrollArea(this);
		m_leftContainWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        m_leftContainWidget->setStyleSheet("QWidget{background: rgb(60, 60, 60);}");
        m_leftContainWidget->setObjectName("MainWindowsLeftContain");
		layout->addWidget(m_leftContainWidget);
		m_outmdiArea->installEventFilter(this);
		m_mdiArea = new FJDragableMdiArea(this);
		layout->addWidget(m_mdiArea);
        QWidget *pRightContainWidget = new QWidget();
        pRightContainWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        pRightContainWidget->setObjectName("MainWindowRightWidght");
        pRightContainWidget->setStyleSheet("QWidget{background: rgb(30, 30, 30);}");
        layout->addWidget(pRightContainWidget);
        pRightContainWidget->setVisible(false);
		m_outmdiArea->setStyleSheet("QWidget{background: rgb(72, 72, 72);}");
		m_outmdiArea->setLayout(layout);
        m_mdiArea->setObjectName("MainWindowsMdiArea");
		setCentralWidget(m_outmdiArea);
		connect(m_mdiArea, &FJDragableMdiArea::subWindowActivated, this, &MainWindow::updateMenus);
		connect(m_mdiArea, &FJDragableMdiArea::subWindowActivated, this, &MainWindow::on3DViewActivated);
		m_mdiArea->installEventFilter(this);
		m_leftContainWidget->setVisible(false);
        QApplication::instance()->setProperty("MainWindowsEnable", true);
	}

    //2022.9.16__Aric.tang
    connect(this, &MainWindow::showErrorStr, this, &MainWindow::slotShowError);

	//picking hub
	{
		m_pickingHub = new ccPickingHub(this, this);
		connect(m_mdiArea, &FJDragableMdiArea::subWindowActivated, m_pickingHub, &ccPickingHub::onActiveWindowChanged);
	}


	connectActions();
	m_TimerReadDog.setInterval(3 * 60 * 1000);

	//janson.yang
	createRibbonMenu();

    createRenderViewers();


	setupInputDevices();

	freezeUI(false);

	updateUI();

	setContextMenuPolicy(Qt::NoContextMenu);//取消右键菜单

    //Melone.yang
    initPCDShiftValue();

#ifdef CC_CORE_LIB_USES_TBB
	ccConsole::Print(QStringLiteral("[TBB] Using Intel's Threading Building Blocks %1.%2")
		.arg(QString::number(TBB_VERSION_MAJOR), QString::number(TBB_VERSION_MINOR)));
#endif

	ccConsole::Print(tr("FJD Trion Metahub started!"));

	//changeDockWidgetTitle(m_UI->DockableProperties, QCoreApplication::translate("MainWindow", "Properties", nullptr));
    changeDockWidgetTitle(m_pDockWidgetPublic, QCoreApplication::translate("MainWindowUI", "Slice plane", nullptr));
	//默认为居中透视效果carl,wang
	toggleActiveWindowCenteredPerspective();
	connect(FJStyleManager::Instance(), &FJStyleManager::updatePerspectiveStateSignal, this, &MainWindow::updatePerspectiveStateSlot);
    connect(FJStyleManager::Instance(), &FJStyleManager::signalErrorMessage, this, &MainWindow::signalErrorMessageSlot);
	//setMaximumSize(QGuiApplication::primaryScreen()->availableGeometry().size());
	//点云选中
	connect(FJStyleManager::Instance(), &FJStyleManager::measureLabelSelected, this, &MainWindow::measureLabelSelectedSlot);

    //添加项目树放大缩小按钮
    {
        m_UI->toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/dockwidgeticonright.png"));
        connect(m_UI->toolButton, &QToolButton::clicked, this, &MainWindow::onDockWidgetIconRightBtn);
        m_UI->toolButton->setVisible(false);
        m_titleWidgettree = new QWidget(m_UI->DockableDBTree);
        QHBoxLayout* psLayouttree = new QHBoxLayout;
        psLayouttree->setSpacing(0);
        psLayouttree->setContentsMargins(16, 6, 8, 5);
        QLabel * docktitlelabeltree = new QLabel(m_titleWidgettree);
        docktitlelabeltree->setText(QCoreApplication::translate("MainWindow", "Objects", nullptr));
        docktitlelabeltree->setObjectName("nobordertransparent");
        //docktitlelabeltree->setFixedHeight(21);
        psLayouttree->addWidget(docktitlelabeltree);
        psLayouttree->addStretch();
        m_treeMinButton = new QToolButton(m_titleWidgettree);
        connect(m_treeMinButton, &QToolButton::clicked, this, &MainWindow::onDockWidgetIconLeftBtn);
        m_treeMinButton->setFixedSize(16, 16);
        m_treeMinButton->setStyleSheet("background-color: transparent;border: 0px solid #1c1c1c; ");
        m_treeMinButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/dockwidgeticonleft.png"));
        psLayouttree->addWidget(m_treeMinButton);
        m_titleWidgettree->setLayout(psLayouttree);
        m_titleWidgettree->setObjectName("dockwidgettitle");
        m_titleWidgettree->setFixedHeight(33);
        QWidget* pTitleWidgettree = m_UI->DockableDBTree->titleBarWidget();
        m_UI->DockableDBTree->setTitleBarWidget(m_titleWidgettree);
        delete pTitleWidgettree;
    }

	connect(&m_tractChangeTimer, &QTimer::timeout, this, &MainWindow::dotractChangeTimer);
}

void MainWindow::dotractChangeTimer()
{
	if (m_currentIndex < (m_timeToTrackVec.size()-1))
	{
		ccGLWindow* cur3dwin = getActiveGLWindow();
		if (cur3dwin)
		{
			CCVector3d P= m_timeToTrackVec[m_currentIndex];

			cur3dwin->setCameraPos(P);
			cur3dwin->setPivotPoint(P, false, false);

			double targetWidth = 10;

			double focalDistance = targetWidth / cur3dwin->getViewportParameters().computeDistanceToWidthRatio();
			CCVector3d diff = m_trackDirectionVec[m_currentIndex];
			//diff.z = 0;
			diff = diff / diff.norm();
			CCVector3d v = diff* focalDistance;
			//cur3dwin->moveCamera(v);
			//cur3dwin->rotateBaseViewMat(rotMat);
			//cur3dwin->setFov(5);
			cur3dwin->setCustomView(diff, CCVector3d(0,0,1), true);

		}
		m_currentIndex++;
		//m_tractChangeTimer.stop();
	}
}

void MainWindow::signalErrorMessageSlot(QString str)
{
    if (str == "Not enough chash")
    {
        QTimer::singleShot(1000, this, [=]() {
            CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Your rendering device is out of memory, please try closing some programs or deleting irrelevant data."));
        });
 
    }
	else if (str == "largecloud")
	{
		QCoreApplication::processEvents();
	}
}

void MainWindow::onDockWidgetIconLeftBtn()
{
	m_leftTreeWidgetWidth = m_UI->dbTreeView->width();
    QWidget* titleWidgettree = new QWidget(m_UI->DockableDBTree);
    titleWidgettree->setFixedHeight(1);
    m_UI->DockableDBTree->setTitleBarWidget(titleWidgettree);
    m_UI->DockableDBTree->titleBarWidget()->setFixedHeight(1);
    m_UI->dbTreeView->setVisible(false);
    m_UI->toolButton->setVisible(true);
    //m_UI->DockableDBTree->setFixedWidth(18);
	m_UI->DockableDBTree->setMinimumWidth(18);
	m_UI->DockableDBTree->setMaximumWidth(18);
}

void MainWindow::onDockWidgetIconRightBtn()
{
    QWidget* pTitleWidgettree = m_UI->DockableDBTree->titleBarWidget();
    m_UI->DockableDBTree->setTitleBarWidget(m_titleWidgettree);
    delete pTitleWidgettree;
    m_UI->dbTreeView->setVisible(true);
    m_UI->toolButton->setVisible(false);
	m_UI->DockableDBTree->setMaximumWidth(9999);
	m_UI->DockableDBTree->setMinimumWidth(150);
	QList<QDockWidget*> temp_docklist;
	temp_docklist << m_UI->DockableDBTree;
	QList<int> temp_sizelist;
	temp_sizelist << m_leftTreeWidgetWidth;
	resizeDocks(temp_docklist, temp_sizelist,Qt::Horizontal);
}

void MainWindow::changeDockWidgetTitle(QDockWidget * dockwidget, QString title)
{
	QWidget* TitleWidgettree = new QWidget(dockwidget);
	QHBoxLayout* psLayouttree = new QHBoxLayout;
	psLayouttree->setSpacing(0);
    psLayouttree->setContentsMargins(16,0,13,0);
	QLabel * docktitlelabeltree = new QLabel(dockwidget);
	docktitlelabeltree->setText(title);
	docktitlelabeltree->setObjectName("nobordertransparent");
    docktitlelabeltree->setFixedHeight(33);
	psLayouttree->addWidget(docktitlelabeltree);
	psLayouttree->addStretch();
	QLabel* TitleWidgetlabeltree = new QLabel(dockwidget);
	TitleWidgetlabeltree->setPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/dockwidgeticonsmall.png"));
	TitleWidgetlabeltree->setObjectName("nobordertransparent");
	psLayouttree->addWidget(TitleWidgetlabeltree);
	TitleWidgettree->setLayout(psLayouttree);
	TitleWidgettree->setObjectName("dockwidgettitle");
	TitleWidgettree->setFixedHeight(33);
	QWidget* pTitleWidgettree = dockwidget->titleBarWidget();
	dockwidget->setTitleBarWidget(TitleWidgettree);
	delete pTitleWidgettree;
}

void MainWindow::setPickMode(bool bEnable)
{
    ccGui::ParamStruct m_parameters = ccGui::Parameters();
    m_parameters.m_bNotePickGLEnable = bEnable;
    m_parameters.singleClickPicking = true;
    if (!m_parameters.m_bNotePickGLEnable) {

        QApplication::restoreOverrideCursor();
    }

    ccGLWindow* pActiveWidget = getActiveGLWindow();
    if (pActiveWidget) {
        ccGLWindow::PICKING_MODE mode = m_parameters.m_bNotePickGLEnable ? ccGLWindow::PICKING_MODE::NOTE_PICKING : ccGLWindow::PICKING_MODE::NO_PICKING;
        pActiveWidget->setPickingMode(mode);
    }
    ccGui::Set(m_parameters);
}

void MainWindow::InitMainWindowStyle()
{
	//QString dirname = FJStyleManager::Instance()->Getcurrentthemepath() + "themestyle.qss";
	QString dirname = m_DirName + "/theme/themestyle.qss";
	QFile styleSheet(dirname);
	if (styleSheet.open(QIODevice::ReadOnly)) {
		this->setStyleSheet(styleSheet.readAll());
	}
}

MainWindow::~MainWindow()
{
    qDebug() << "~MainWindow() is enter";
	disconnect();
	destroyInputDevices();

	cancelPreviousPickingOperation(false); //just in case

	assert(m_ccRoot && m_mdiArea);
	m_ccRoot->disconnect();
	m_mdiArea->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = nullptr;

	//remove all entities from 3D views before quitting to avoid any side-effect
	//(this won't be done automatically since we've just reset m_ccRoot)
	ccRoot->getRootEntity()->setDisplay_recursive(nullptr);
	for (int i = 0; i < getGLWindowCount(); ++i)
	{
		getGLWindow(i)->setSceneDB(nullptr);
	}
	m_cpeDlg = nullptr;
	m_gsTool = nullptr;
	m_manualClassificationTool = nullptr;
	m_seTool = nullptr;
	m_transTool = nullptr;
	m_clipTool = nullptr;
	m_compDlg = nullptr;
	m_ppDlg = nullptr;
	m_plpDlg = nullptr;
	m_pprDlg = nullptr;
	m_pavDlg = nullptr;
	m_pfDlg = nullptr;
	m_ppp = nullptr;


    qDebug() << "~MainWindow() is while";
	//release all 'overlay' dialogs
	while (!m_mdiDialogs.empty())
	{
		ccMDIDialogs mdiDialog = m_mdiDialogs.back();
		m_mdiDialogs.pop_back();

		mdiDialog.dialog->disconnect();
		mdiDialog.dialog->stop(false);
		mdiDialog.dialog->setParent(nullptr);
		delete mdiDialog.dialog;
	}
    qDebug() << "~MainWindow() is release";
	//m_mdiDialogs.clear();
	m_mdiArea->closeAllSubWindows();

	if (ccRoot)
	{
		delete ccRoot;
		ccRoot = nullptr;
	}

	delete m_UI;
	m_UI = nullptr;

	ccConsole::ReleaseInstance(false); //if we flush the console, it will try to display the console window while we are destroying everything!

	//存储cc原有的menubar，暂时无用 janson.yang
	if (menubar != nullptr)
	{
		delete menubar;
		menubar = nullptr;
	}
	
    m_pDockCenterWidget->disconnect();
    delete m_pDockCenterWidget;
    m_pDockCenterWidget = nullptr;

    qDebug() << "~MainWindow() is end";
}

void MainWindow::initPlugins()
{
	m_pluginUIManager->init();

	//m_pluginUIManager->pluginMenu()->menuAction()->setVisible(false);//隐藏plugins菜单栏janson.yang

	// Set up dynamic tool bars
	addToolBar(Qt::RightToolBarArea, m_pluginUIManager->glFiltersToolbar());
	addToolBar(Qt::RightToolBarArea, m_pluginUIManager->mainPluginToolbar());

	for (QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars())
	{
		addToolBar(Qt::TopToolBarArea, toolbar);
	}

	// Set up dynamic menus
	//m_UI->menubar->insertMenu( m_UI->menu3DViews->menuAction(), m_pluginUIManager->pluginMenu() );
	menuDisplay->insertMenu( /*m_UI->*/menuActiveScalarField->menuAction(), m_pluginUIManager->shaderAndFilterMenu());

	menuToolbars->addAction(m_pluginUIManager->actionShowMainPluginToolbar());
	menuToolbars->addAction(m_pluginUIManager->actionShowGLFilterToolbar());



	mlsaction = m_pluginUIManager->GetActionByName("MLS");
	pcvaction = m_pluginUIManager->GetActionByName("PCV / ShadeVis");
    
	QAction* testBroomAction = m_pluginUIManager->GetActionByName("CEA Virtual Broom");
	QAction* testAnimationAction = m_pluginUIManager->GetActionByName("Animation");
	
	//QAction* test0Action = m_pluginUIManager->GetActionByName("Classify");

//	csfAction = m_pluginUIManager->GetActionByName("CSF Filter");


	//重置相关图标
	QString action_text;
	QString action_icon_path;
	QString objectName;

	action_text = QCoreApplication::translate("MainWindow", "MLS", nullptr);
	action_icon_path = QString(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/mlsicon.png");
	objectName = "actionMLSreconstruction";
	resetActionIcon(mlsaction, action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/mlsiconselect.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/mlsicondisable.png", objectName);
	mlsaction->setToolTip(QCoreApplication::translate("MainWindow", "Smooth using MLS, optionally upsample", nullptr));

	action_text = QCoreApplication::translate("MainWindow", "Light enhancement", nullptr);
	action_icon_path = QString(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pcvicon.png");
	objectName = "actionPCVreconstruction";
    setActionIcon(pcvaction, "pcvicon", "pcviconClick", "pcvicondisable");
	pcvaction->setText(action_text);
	//resetActionIcon(pcvaction, action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pcvicon.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pcvicondisable.png", objectName);
    pcvaction->setToolTip(QCoreApplication::translate("MainWindow", "Improve point cloud intensity visualization", nullptr));

	//加载plugins到ribbon
	if (ribbon)
	{
		if (ribbon->categoryByObjectName("Start"))
		{
			if (ribbon->categoryByObjectName("Start")->pannelByObjectName("PointProcess"))
			{
				//ribbon->categoryByObjectName("Start")->pannelByObjectName("PointProcess")->addLargeAction(testBroomAction);
				//[!]隐藏MLS
				//ribbon->categoryByObjectName("Start")->pannelByObjectName("PointProcess")->addLargeAction(mlsaction);
//				ribbon->categoryByObjectName("edit")->pannelByObjectName("Points")->addLargeAction(csfAction);
				//ribbon->categoryByObjectName("Start")->pannelByObjectName("PointProcess")->addLargeAction(testAnimationAction);
				//ribbon->categoryByObjectName("Start")->pannelByObjectName("PointProcess")->addLargeAction(m_UI->actionSaveViewportAsObject);
				
			}


		}
        if (ribbon->categoryByObjectName("view"))
        {
            if (ribbon->categoryByObjectName("view")->pannelByObjectName("Appearance"))
            {
				pcvaction->setEnabled(true);
                ribbon->categoryByObjectName("view")->pannelByObjectName("Appearance")->addLargeAction(pcvaction);
            }
        }
	}

}

void MainWindow::doEnableQtWarnings(bool state)
{
	ccConsole::EnableQtMessages(state);
}

void MainWindow::increasePointSize()
{
	//active window?
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPointSize(win->getViewportParameters().defaultPointSize + 1);
		win->redraw();
	}
}

void MainWindow::decreasePointSize()
{
	//active window?
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPointSize(win->getViewportParameters().defaultPointSize - 1);
		win->redraw();
	}
}

void MainWindow::setupInputDevices()
{
#ifdef CC_3DXWARE_SUPPORT
	m_3DMouseManager = new cc3DMouseManager(this, this);
	m_UI->menuFile->insertMenu(m_UI->actionCloseAll, m_3DMouseManager->menu());
#endif

#ifdef CC_GAMEPAD_SUPPORT
	m_gamepadManager = new ccGamepadManager(this, this);
	menuFile->insertMenu(m_UI->actionCloseAll, m_gamepadManager->menu());//janson.yang
#endif

#if defined(CC_3DXWARE_SUPPORT) || defined(CC_GAMEPAD_SUPPORT)
	menuFile->insertSeparator(m_UI->actionCloseAll);//janson.yang
#endif
}

void MainWindow::destroyInputDevices()
{
#ifdef CC_GAMEPAD_SUPPORT
	delete m_gamepadManager;
	m_gamepadManager = nullptr;
#endif

#ifdef CC_3DXWARE_SUPPORT
	delete m_3DMouseManager;
	m_3DMouseManager = nullptr;
#endif
}

void MainWindow::connectActions()
{
	assert(m_ccRoot);
	assert(m_mdiArea);

	//Keyboard shortcuts

	//'A': toggles selected items activation
	connect(m_UI->actionToggleActivation, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::ACTIVE);
	});

	//'V': toggles selected items visibility
	connect(m_UI->actionToggleVisibility, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::VISIBLE);
	});

	//'N': toggles selected items normals visibility
	connect(m_UI->actionToggleNormals, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::NORMALS);
	});

	//'C': toggles selected items colors visibility
	connect(m_UI->actionToggleColors, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::COLOR);
	});

	//'S': toggles selected items SF visibility
	connect(m_UI->actionToggleSF, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::SCALAR_FIELD);
	});

	//'D': toggles selected items '3D name' visibility
	connect(m_UI->actionToggleShowName, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::NAME);
	});

	//'M': toggles selected items materials/textures visibility
	connect(m_UI->actionToggleMaterials, &QAction::triggered, this, [=]() {
		toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY::MATERIAL);
	});

	//TODO... but not ready yet ;)
	m_UI->actionLoadShader->setVisible(false);
	m_UI->actionDeleteShader->setVisible(false);
	m_UI->actionKMeans->setVisible(false);
	m_UI->actionFrontPropagation->setVisible(false);

	///*** MAIN MENU ***/

	//"File" menu
	connect(m_UI->actionOpen, &QAction::triggered, this, &MainWindow::doActionLoadFile);
	connect(m_UI->actionSave, &QAction::triggered, this, &MainWindow::doActionSaveFile);
	connect(m_UI->actionGlobalShiftSettings, &QAction::triggered, this, &MainWindow::doActionGlobalShiftSeetings);
	connect(m_UI->actionPrimitiveFactory, &QAction::triggered, this, &MainWindow::doShowPrimitiveFactory);
	connect(m_UI->actionCloseAll, &QAction::triggered, this, &MainWindow::closeAll);
	connect(m_UI->actionQuit, &QAction::triggered, this, &QWidget::close);

	//"Edit > Colors" menu
	connect(m_UI->actionSetUniqueColor, &QAction::triggered, this, &MainWindow::doActionSetUniqueColor);
	connect(m_UI->actionSetColorGradient, &QAction::triggered, this, &MainWindow::doActionSetColorGradient);
	connect(m_UI->actionChangeColorLevels, &QAction::triggered, this, &MainWindow::doActionChangeColorLevels);
	connect(m_UI->actionColorize, &QAction::triggered, this, &MainWindow::doActionColorize);
	connect(m_UI->actionRGBToGreyScale, &QAction::triggered, this, &MainWindow::doActionRGBToGreyScale);
	connect(m_UI->actionInterpolateColors, &QAction::triggered, this, &MainWindow::doActionInterpolateColors);
	connect(m_UI->actionEnhanceRGBWithIntensities, &QAction::triggered, this, &MainWindow::doActionEnhanceRGBWithIntensities);
	connect(m_UI->actionColorFromScalarField, &QAction::triggered, this, &MainWindow::doActionColorFromScalars);
	connect(m_UI->actionClearColor, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY::COLORS);
	});

	//"Edit > Normals" menu
	connect(m_UI->actionComputeNormals, &QAction::triggered, this, &MainWindow::doActionComputeNormals);
	connect(m_UI->actionInvertNormals, &QAction::triggered, this, &MainWindow::doActionInvertNormals);
	connect(m_UI->actionConvertNormalToHSV, &QAction::triggered, this, &MainWindow::doActionConvertNormalsToHSV);
	connect(m_UI->actionConvertNormalToDipDir, &QAction::triggered, this, &MainWindow::doActionConvertNormalsToDipDir);
	connect(m_UI->actionExportNormalToSF, &QAction::triggered, this, &MainWindow::doActionExportNormalToSF);
	connect(m_UI->actionOrientNormalsMST, &QAction::triggered, this, &MainWindow::doActionOrientNormalsMST);
	connect(m_UI->actionOrientNormalsFM, &QAction::triggered, this, &MainWindow::doActionOrientNormalsFM);
	connect(m_UI->actionClearNormals, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY::NORMALS);
	});

	//"Edit > Octree" menu
	connect(m_UI->actionComputeOctree, &QAction::triggered, this, &MainWindow::doActionComputeOctree);
	connect(m_UI->actionResampleWithOctree, &QAction::triggered, this, &MainWindow::doActionResampleWithOctree);

	//"Edit > Grid" menu
	connect(m_UI->actionDeleteScanGrid, &QAction::triggered, this, &MainWindow::doActionDeleteScanGrids);

	//"Edit > Cloud" menu
	connect(m_UI->actionCreateSinglePointCloud, &QAction::triggered, this, &MainWindow::createSinglePointCloud);
	connect(m_UI->actionPasteCloudFromClipboard, &QAction::triggered, this, &MainWindow::createPointCloudFromClipboard);
	//the 'Paste from clipboard' tool depends on the clipboard state
	{
		const QClipboard* clipboard = QApplication::clipboard();
		assert(clipboard);
		m_UI->actionPasteCloudFromClipboard->setEnabled(clipboard->mimeData()->hasText());
		connect(clipboard, &QClipboard::dataChanged, [&]() { m_UI->actionPasteCloudFromClipboard->setEnabled(clipboard->mimeData()->hasText()); });
	}


	//"Edit > Mesh" menu
	connect(m_UI->actionComputeMeshAA, &QAction::triggered, this, &MainWindow::doActionComputeMeshAA);
	connect(m_UI->actionComputeMeshLS, &QAction::triggered, this, &MainWindow::doActionComputeMeshLS);
	connect(m_UI->actionMeshTwoPolylines, &QAction::triggered, this, &MainWindow::doMeshTwoPolylines);
	connect(m_UI->actionMeshScanGrids, &QAction::triggered, this, &MainWindow::doActionMeshScanGrids);
	connect(m_UI->actionConvertTextureToColor, &QAction::triggered, this, &MainWindow::doActionConvertTextureToColor);
	connect(m_UI->actionSamplePointsOnMesh, &QAction::triggered, this, &MainWindow::doActionSamplePointsOnMesh);
	connect(m_UI->actionSmoothMeshLaplacian, &QAction::triggered, this, &MainWindow::doActionSmoothMeshLaplacian);
	connect(m_UI->actionSubdivideMesh, &QAction::triggered, this, &MainWindow::doActionSubdivideMesh);
	connect(m_UI->actionFlipMeshTriangles, &QAction::triggered, this, &MainWindow::doActionFlipMeshTriangles);
	connect(m_UI->actionMeasureMeshSurface, &QAction::triggered, this, &MainWindow::doActionMeasureMeshSurface);
	connect(m_UI->actionFlagMeshVertices, &QAction::triggered, this, &MainWindow::doActionFlagMeshVertices);
	//"Edit > Mesh > Scalar Field" menu
	connect(m_UI->actionSmoothMeshSF, &QAction::triggered, this, &MainWindow::doActionSmoothMeshSF);
	connect(m_UI->actionEnhanceMeshSF, &QAction::triggered, this, &MainWindow::doActionEnhanceMeshSF);
	//"Edit > Polyline" menu
	connect(m_UI->actionSamplePointsOnPolyline, &QAction::triggered, this, &MainWindow::doActionSamplePointsOnPolyline);
	connect(m_UI->actionSmoothPolyline, &QAction::triggered, this, &MainWindow::doActionSmoohPolyline);

	//"Edit > Plane" menu
	connect(m_UI->actionCreatePlane, &QAction::triggered, this, &MainWindow::doActionCreatePlane);
	connect(m_UI->actionEditPlane, &QAction::triggered, this, &MainWindow::doActionEditPlane);
	connect(m_UI->actionFlipPlane, &QAction::triggered, this, &MainWindow::doActionFlipPlane);
	connect(m_UI->actionComparePlanes, &QAction::triggered, this, &MainWindow::doActionComparePlanes);
	//"Edit > Sensor > Ground-Based lidar" menu
	connect(m_UI->actionShowDepthBuffer, &QAction::triggered, this, &MainWindow::doActionShowDepthBuffer);
	connect(m_UI->actionExportDepthBuffer, &QAction::triggered, this, &MainWindow::doActionExportDepthBuffer);
	connect(m_UI->actionComputePointsVisibility, &QAction::triggered, this, &MainWindow::doActionComputePointsVisibility);
	//"Edit > Sensor" menu
	connect(m_UI->actionCreateGBLSensor, &QAction::triggered, this, &MainWindow::doActionCreateGBLSensor);
	connect(m_UI->actionCreateCameraSensor, &QAction::triggered, this, &MainWindow::doActionCreateCameraSensor);
	connect(m_UI->actionModifySensor, &QAction::triggered, this, &MainWindow::doActionModifySensor);
	connect(m_UI->actionProjectUncertainty, &QAction::triggered, this, &MainWindow::doActionProjectUncertainty);
	connect(m_UI->actionCheckPointsInsideFrustum, &QAction::triggered, this, &MainWindow::doActionCheckPointsInsideFrustum);
	connect(m_UI->actionComputeDistancesFromSensor, &QAction::triggered, this, &MainWindow::doActionComputeDistancesFromSensor);
	connect(m_UI->actionComputeScatteringAngles, &QAction::triggered, this, &MainWindow::doActionComputeScatteringAngles);
	connect(m_UI->actionViewFromSensor, &QAction::triggered, this, &MainWindow::doActionSetViewFromSensor);
	//"Edit > Scalar fields" menu
	connect(m_UI->actionShowHistogram, &QAction::triggered, this, &MainWindow::showSelectedEntitiesHistogram);
	connect(m_UI->actionComputeStatParams, &QAction::triggered, this, &MainWindow::doActionComputeStatParams);
	connect(m_UI->actionSFGradient, &QAction::triggered, this, &MainWindow::doActionSFGradient);
	connect(m_UI->actionGaussianFilter, &QAction::triggered, this, &MainWindow::doActionSFGaussianFilter);
	connect(m_UI->actionBilateralFilter, &QAction::triggered, this, &MainWindow::doActionSFBilateralFilter);
	connect(m_UI->actionFilterByValue, &QAction::triggered, this, &MainWindow::doActionFilterByValue);
	connect(m_UI->actionAddConstantSF, &QAction::triggered, this, &MainWindow::doActionAddConstantSF);
	connect(m_UI->actionAddClassificationSF, &QAction::triggered, this, &MainWindow::doActionAddClassificationSF);
	connect(m_UI->actionScalarFieldArithmetic, &QAction::triggered, this, &MainWindow::doActionScalarFieldArithmetic);
	connect(m_UI->actionScalarFieldFromColor, &QAction::triggered, this, &MainWindow::doActionScalarFieldFromColor);
	connect(m_UI->actionConvertToRGB, &QAction::triggered, this, &MainWindow::doActionSFConvertToRGB);
	connect(m_UI->actionConvertToRandomRGB, &QAction::triggered, this, &MainWindow::doActionSFConvertToRandomRGB);
	connect(m_UI->actionRenameSF, &QAction::triggered, this, &MainWindow::doActionRenameSF);
	connect(m_UI->actionOpenColorScalesManager, &QAction::triggered, this, &MainWindow::doActionOpenColorScalesManager);
	connect(m_UI->actionAddIdField, &QAction::triggered, this, &MainWindow::doActionAddIdField);
	connect(m_UI->actionSplitCloudUsingSF, &QAction::triggered, this, &MainWindow::doActionSplitCloudUsingSF);
	connect(m_UI->actionSetSFAsCoord, &QAction::triggered, this, &MainWindow::doActionSetSFAsCoord);
	connect(m_UI->actionInterpolateSFs, &QAction::triggered, this, &MainWindow::doActionInterpolateScalarFields);
	connect(m_UI->actionDeleteScalarField, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY::CURRENT_SCALAR_FIELD);
	});
	connect(m_UI->actionDeleteAllSF, &QAction::triggered, this, [=]() {
		clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY::ALL_SCALAR_FIELDS);
	});

	//"Edit > Waveform" menu
	connect(m_UI->actionShowWaveDialog, &QAction::triggered, this, &MainWindow::doActionShowWaveDialog);
	connect(m_UI->actionCompressFWFData, &QAction::triggered, this, &MainWindow::doActionCompressFWFData);
	//"Edit" menu
	connect(m_UI->actionClone, &QAction::triggered, this, &MainWindow::doActionClone);
	connect(m_UI->actionMerge, &QAction::triggered, this, &MainWindow::doActionMerge);
	connect(m_UI->actionApplyTransformation, &QAction::triggered, this, &MainWindow::doActionApplyTransformation);
	connect(m_UI->actionApplyScale, &QAction::triggered, this, &MainWindow::doActionApplyScale);
	connect(m_UI->actionTranslateRotate, &QAction::triggered, this, &MainWindow::activateTranslateRotateMode);
	connect(m_UI->actionSegment, &QAction::triggered, this, &MainWindow::activateSegmentationMode);
	connect(m_UI->actionTracePolyline, &QAction::triggered, this, &MainWindow::activateTracePolylineMode);

	connect(m_UI->actionCrop, &QAction::triggered, this, &MainWindow::doActionCrop);
	connect(m_UI->actionEditGlobalShiftAndScale, &QAction::triggered, this, &MainWindow::doActionEditGlobalShiftAndScale);
	connect(m_UI->actionSubsample, &QAction::triggered, this, &MainWindow::doActionSubsample);
	connect(m_UI->actionDelete, &QAction::triggered, m_ccRoot, &ccDBRoot::deleteSelectedEntities);

	//"Tools > Clean" menu
	connect(m_UI->actionSORFilter, &QAction::triggered, this, &MainWindow::doActionSORFilter);
	connect(m_UI->actionNoiseFilter, &QAction::triggered, this, &MainWindow::doActionFilterNoise);

	//"Tools > Projection" menu
	connect(m_UI->actionUnroll, &QAction::triggered, this, &MainWindow::doActionUnroll);
	connect(m_UI->actionRasterize, &QAction::triggered, this, &MainWindow::doActionRasterize);
	connect(m_UI->actionConvertPolylinesToMesh, &QAction::triggered, this, &MainWindow::doConvertPolylinesToMesh);
	//connect(m_UI->actionCreateSurfaceBetweenTwoPolylines, &QAction::triggered, this, &MainWindow::doMeshTwoPolylines); //DGM: already connected to actionMeshTwoPolylines
	connect(m_UI->actionExportCoordToSF, &QAction::triggered, this, &MainWindow::doActionExportCoordToSF);

	//"Tools > Registration" menu
	connect(m_UI->actionMatchBBCenters, &QAction::triggered, this, &MainWindow::doActionMatchBBCenters);
	connect(m_UI->actionMatchScales, &QAction::triggered, this, &MainWindow::doActionMatchScales);
	connect(m_UI->actionRegister, &QAction::triggered, this, &MainWindow::doActionRegister);
	connect(m_UI->actionPointPairsAlign, &QAction::triggered, this, &MainWindow::activateRegisterPointPairTool);
	connect(m_UI->actionBBCenterToOrigin, &QAction::triggered, this, &MainWindow::doActionMoveBBCenterToOrigin);
	connect(m_UI->actionBBMinCornerToOrigin, &QAction::triggered, this, &MainWindow::doActionMoveBBMinCornerToOrigin);
	connect(m_UI->actionBBMaxCornerToOrigin, &QAction::triggered, this, &MainWindow::doActionMoveBBMaxCornerToOrigin);
	//"Tools > Distances" menu
	connect(m_UI->actionCloudCloudDist, &QAction::triggered, this, &MainWindow::doActionCloudCloudDist);
	connect(m_UI->actionCloudMeshDist, &QAction::triggered, this, &MainWindow::doActionCloudMeshDist);
	connect(m_UI->actionCloudPrimitiveDist, &QAction::triggered, this, &MainWindow::doActionCloudPrimitiveDist);
	connect(m_UI->actionCPS, &QAction::triggered, this, &MainWindow::doActionComputeCPS);
	//"Tools > Volume" menu
	//"Tools > Statistics" menu
	connect(m_UI->actionComputeStatParams2, &QAction::triggered, this, &MainWindow::doActionComputeStatParams); //duplicated action --> we can't use the same otherwise we get an ugly console warning on Linux :(
	connect(m_UI->actionStatisticalTest, &QAction::triggered, this, &MainWindow::doActionStatisticalTest);
	//"Tools > Segmentation" menu
	connect(m_UI->actionLabelConnectedComponents, &QAction::triggered, this, &MainWindow::doActionLabelConnectedComponents);
	connect(m_UI->actionKMeans, &QAction::triggered, this, &MainWindow::doActionKMeans);
	connect(m_UI->actionFrontPropagation, &QAction::triggered, this, &MainWindow::doActionFrontPropagation);
	connect(m_UI->actionCrossSection, &QAction::triggered, this, &MainWindow::activateClippingBoxMode);
	connect(m_UI->actionExtractSections, &QAction::triggered, this, &MainWindow::activateSectionExtractionMode);
	//"Tools > Fit" menu
	connect(m_UI->actionFitPlane, &QAction::triggered, this, &MainWindow::doActionFitPlane);
	connect(m_UI->actionFitSphere, &QAction::triggered, this, &MainWindow::doActionFitSphere);
	connect(m_UI->actionFitFacet, &QAction::triggered, this, &MainWindow::doActionFitFacet);
	connect(m_UI->actionFitQuadric, &QAction::triggered, this, &MainWindow::doActionFitQuadric);
	//"Tools > Batch export" menu
	connect(m_UI->actionExportCloudInfo, &QAction::triggered, this, &MainWindow::doActionExportCloudInfo);
	connect(m_UI->actionExportPlaneInfo, &QAction::triggered, this, &MainWindow::doActionExportPlaneInfo);
	//"Tools > Other" menu
	connect(m_UI->actionComputeGeometricFeature, &QAction::triggered, this, &MainWindow::doComputeGeometricFeature);
	connect(m_UI->actionRemoveDuplicatePoints, &QAction::triggered, this, &MainWindow::doRemoveDuplicatePoints);
	//"Tools"
	connect(m_UI->actionLevel, &QAction::triggered, this, &MainWindow::doLevel);
	connect(m_UI->actionPointListPicking, &QAction::triggered, this, &MainWindow::activatePointListPickingMode);
	connect(m_UI->actionPointPicking, &QAction::triggered, this, &MainWindow::activatePointPickingMode);

	//"Tools > Sand box (research)" menu
	connect(m_UI->actionComputeKdTree, &QAction::triggered, this, &MainWindow::doActionComputeKdTree);
	connect(m_UI->actionDistanceMap, &QAction::triggered, this, &MainWindow::doActionComputeDistanceMap);
	connect(m_UI->actionDistanceToBestFitQuadric3D, &QAction::triggered, this, &MainWindow::doActionComputeDistToBestFitQuadric3D);
	connect(m_UI->actionComputeBestFitBB, &QAction::triggered, this, &MainWindow::doComputeBestFitBB);
	connect(m_UI->actionAlign, &QAction::triggered, this, &MainWindow::doAction4pcsRegister); //Aurelien BEY le 13/11/2008
	connect(m_UI->actionSNETest, &QAction::triggered, this, &MainWindow::doSphericalNeighbourhoodExtractionTest);
	connect(m_UI->actionCNETest, &QAction::triggered, this, &MainWindow::doCylindricalNeighbourhoodExtractionTest);
	connect(m_UI->actionFindBiggestInnerRectangle, &QAction::triggered, this, &MainWindow::doActionFindBiggestInnerRectangle);
	connect(m_UI->actionCreateCloudFromEntCenters, &QAction::triggered, this, &MainWindow::doActionCreateCloudFromEntCenters);
	connect(m_UI->actionComputeBestICPRmsMatrix, &QAction::triggered, this, &MainWindow::doActionComputeBestICPRmsMatrix);

	//"Display" menu
	connect(m_UI->actionFullScreen, &QAction::toggled, this, &MainWindow::toggleFullScreen);
	connect(m_UI->actionExclusiveFullScreen, &QAction::toggled, this, &MainWindow::toggleExclusiveFullScreen);
	connect(m_UI->actionRefresh, &QAction::triggered, this, &MainWindow::refreshAll);

    //QShortcut* shortcut1 = new QShortcut(QKeySequence("F8"), this);
    //shortcut1->setContext(Qt::ShortcutContext::ApplicationShortcut);
	//connect(shortcut1, &QShortcut::activated, this, &MainWindow::testFrameRate);
	connect(m_UI->actionToggleCenteredPerspective, &QAction::triggered, this, &MainWindow::toggleActiveWindowCenteredPerspective);
	connect(m_UI->actionToggleViewerBasedPerspective, &QAction::triggered, this, &MainWindow::toggleActiveWindowViewerBasedPerspective);
	connect(m_UI->actionShowCursor3DCoordinates, &QAction::toggled, this, &MainWindow::toggleActiveWindowShowCursorCoords);
	connect(m_UI->actionLockRotationAxis, &QAction::triggered, this, &MainWindow::toggleLockRotationAxis);
	connect(m_UI->actionEnterBubbleViewMode, &QAction::triggered, this, &MainWindow::doActionEnableBubbleViewMode);
	connect(m_UI->actionEditCamera, &QAction::triggered, this, &MainWindow::doActionEditCamera);
	connect(m_UI->actionAdjustZoom, &QAction::triggered, this, &MainWindow::doActionAdjustZoom);
	connect(m_UI->actionSaveViewportAsObject, &QAction::triggered, this, &MainWindow::doActionSaveViewportAsCamera);

	//"Display > Lights & Materials" menu
	connect(m_UI->actionDisplayOptions, &QAction::triggered, this, &MainWindow::showDisplayOptions);
	connect(m_UI->actionToggleSunLight, &QAction::triggered, this, &MainWindow::toggleActiveWindowSunLight);
	connect(m_UI->actionToggleCustomLight, &QAction::triggered, this, &MainWindow::toggleActiveWindowCustomLight);
	connect(m_UI->actionRenderToFile, &QAction::triggered, this, &MainWindow::doActionRenderToFile);
	//"Display > Shaders & filters" menu
	connect(m_UI->actionLoadShader, &QAction::triggered, this, &MainWindow::doActionLoadShader);
	connect(m_UI->actionDeleteShader, &QAction::triggered, this, &MainWindow::doActionDeleteShader);

	//"Display > Active SF" menu
	connect(m_UI->actionToggleActiveSFColorScale, &QAction::triggered, this, &MainWindow::doActionToggleActiveSFColorScale);
	connect(m_UI->actionShowActiveSFPrevious, &QAction::triggered, this, &MainWindow::doActionShowActiveSFPrevious);
	connect(m_UI->actionShowActiveSFNext, &QAction::triggered, this, &MainWindow::doActionShowActiveSFNext);

	//"Display" menu
	connect(m_UI->actionResetGUIElementsPos, &QAction::triggered, this, &MainWindow::doActionResetGUIElementsPos);
	connect(m_UI->actionResetAllVBOs, &QAction::triggered, this, &MainWindow::doActionResetAllVBOs);

	//"3D Views" menu
	connect(menu3DViews, &QMenu::aboutToShow, this, &MainWindow::update3DViewsMenu);
	//connect(m_UI->actionNew3DView, &QAction::triggered, this, &MainWindow::new3DView);
	connect(m_UI->actionZoomIn, &QAction::triggered, this, &MainWindow::zoomIn);
	connect(m_UI->actionZoomOut, &QAction::triggered, this, &MainWindow::zoomOut);
	//connect(m_UI->actionClose3DView, &QAction::triggered, m_mdiArea, &QMdiArea::closeActiveSubWindow);
	//connect(m_UI->actionCloseAll3DViews, &QAction::triggered, m_mdiArea, &QMdiArea::closeAllSubWindows);
	//connect(m_UI->actionTile3DViews, &QAction::triggered, m_mdiArea, &QMdiArea::tileSubWindows);
	//connect(m_UI->actionCascade3DViews, &QAction::triggered, m_mdiArea, &QMdiArea::cascadeSubWindows);
	//connect(m_UI->actionNext3DView, &QAction::triggered, m_mdiArea, &QMdiArea::activateNextSubWindow);
	//connect(m_UI->actionPrevious3DView, &QAction::triggered, m_mdiArea, &QMdiArea::activatePreviousSubWindow);

	//"About" menu entry
	connect(m_UI->actionHelp, &QAction::triggered, this, &MainWindow::doActionShowHelpDialog);
	connect(m_UI->actionAboutPlugins, &QAction::triggered, m_pluginUIManager, &ccPluginUIManager::showAboutDialog);
	connect(m_UI->actionEnableQtWarnings, &QAction::toggled, this, &MainWindow::doEnableQtWarnings);

	connect(m_UI->actionAbout, &QAction::triggered, this, [this]() {
		ccAboutDialog* aboutDialog = new ccAboutDialog(this);
		aboutDialog->exec();
	});

	/*** Toolbars ***/

	//View toolbar
	connect(m_UI->actionGlobalZoom, &QAction::triggered, this, &MainWindow::setGlobalZoom);
	connect(m_UI->actionViewlock, &QAction::triggered, this, &MainWindow::SetViewLock);
	connect(m_UI->actionPickRotationCenter, &QAction::triggered, this, &MainWindow::doPickRotationCenter);
	connect(m_UI->actionZoomAndCenter, &QAction::triggered, this, &MainWindow::zoomOnSelectedEntities);
	connect(m_UI->actionSetPivotAlwaysOn, &QAction::triggered, this, &MainWindow::setPivotAlwaysOn);
	connect(m_UI->actionSetPivotRotationOnly, &QAction::triggered, this, &MainWindow::setPivotRotationOnly);
	connect(m_UI->actionSetPivotOff, &QAction::triggered, this, &MainWindow::setPivotOff);

	connect(m_UI->actionSetOrthoView, &QAction::triggered, this, [this]() {
		setOrthoView(getActiveGLWindow());
	});
	connect(m_UI->actionSetCenteredPerspectiveView, &QAction::triggered, this, [this]() {
		setCenteredPerspectiveView(getActiveGLWindow());
	});
	connect(m_UI->actionSetViewerPerspectiveView, &QAction::triggered, this, [this]() {
		setViewerPerspectiveView(getActiveGLWindow());
	});

	connect(m_UI->actionEnableStereo, &QAction::toggled, this, &MainWindow::toggleActiveWindowStereoVision);
	connect(m_UI->actionAutoPickRotationCenter, &QAction::toggled, this, &MainWindow::toggleActiveWindowAutoPickRotCenter);

	connect(m_UI->actionSetViewTop, &QAction::triggered, this, [=]() { setView(CC_TOP_VIEW); });
	connect(m_UI->actionSetViewBottom, &QAction::triggered, this, [=]() { setView(CC_BOTTOM_VIEW); });
	connect(m_UI->actionSetViewFront, &QAction::triggered, this, [=]() { setView(CC_FRONT_VIEW); });
	connect(m_UI->actionSetViewBack, &QAction::triggered, this, [=]() { setView(CC_BACK_VIEW); });
	connect(m_UI->actionSetViewLeft, &QAction::triggered, this, [=]() { setView(CC_LEFT_VIEW); });
	connect(m_UI->actionSetViewRight, &QAction::triggered, this, [=]() { setView(CC_RIGHT_VIEW); });
	connect(m_UI->actionSetViewIso1, &QAction::triggered, this, [=]() { setView(CC_ISO_VIEW_1); });
	connect(m_UI->actionSetViewIso2, &QAction::triggered, this, [=]() { setView(CC_ISO_VIEW_2); });

	//hidden
	connect(m_UI->actionEnableVisualDebugTraces, &QAction::triggered, this, &MainWindow::toggleVisualDebugTraces);

	connect(&m_TimerReadDog, &QTimer::timeout, this, &MainWindow::slotTimerCheckDog);

    //平面切割
    connect(actionPlaneCut, &QAction::triggered, this, &MainWindow::slotShowPlaneCut);

	
	//坐标转换
	connect(m_coordinateConversionAciton, &QAction::triggered, this, &MainWindow::slotCoordinateConversion);

	//EDL按钮
	connect(m_EDLAction, &QAction::triggered, this, &MainWindow::slotEDLControl);


	//等高线
	connect(m_ContourAction, &QAction::triggered, this, &MainWindow::slotContourAction);


	//倾斜拉平
	connect(m_actionAccuracyActiveAxisZ, &QAction::triggered, this, &MainWindow::slotAccuracyActiveZaxisLevel);
	

    //[!].启动或编辑注释
    connect(actionNotes, &QAction::triggered, this, &MainWindow::slotActionNotes);

    //[!].拾取发生改变
    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalActivatedNoteChanged,
        this, &MainWindow::slotActivatedNoteChanged);

    //[!].主框架部分
    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalRedrawActiveWindow,
        this, &MainWindow::slotRedrawActiveWindow);

    //[!].主框架部分
    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalRedraw2DWindow,
        this, &MainWindow::slotRedraw2DWindow);

    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalRedrawAllWindows,
        this, &MainWindow::slotRedrawAllWindows, Qt::QueuedConnection);

    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalRenewalDBTreeModel,
        this, &MainWindow::slotRenewalDBTreeModel);


    //[!].激活更新UI软件模块授权
    connect(CS::Model::ProjectModel::instance(), &CS::Model::ProjectModel::signalRegisterActivateUpdateUIModel,
        this, &MainWindow::slotUpdateRegisterCoderCategory);

	//[!].点云生成正射影像
	connect(m_pCreatePointCloudOrthphotoAction, &QAction::triggered, this, &MainWindow::slotUIGenerateOrthoimageChanged);

}

void MainWindow::doActionColorize()
{
	doActionSetColor(true);
}

void MainWindow::doActionSetUniqueColor()
{
	doActionSetColor(false);
}

void MainWindow::doActionSetColor(bool colorize)
{
	if (!ccEntityAction::setColor(m_selectedEntities, colorize, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionRGBToGreyScale()
{
	if (!ccEntityAction::rgbToGreyScale(m_selectedEntities))
		return;

	refreshAll();
}

void MainWindow::doActionSetColorGradient()
{
	if (!ccEntityAction::setColorGradient(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionChangeColorLevels()
{
	ccEntityAction::changeColorLevels(m_selectedEntities, this);
}

void MainWindow::doActionInterpolateColors()
{
	if (!ccEntityAction::interpolateColors(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionInterpolateScalarFields()
{
	if (!ccEntityAction::interpolateSFs(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionEnhanceRGBWithIntensities()
{
	if (!ccEntityAction::enhanceRGBWithIntensities(m_selectedEntities, this))
		return;

	refreshAll();
}

void MainWindow::doActionColorFromScalars()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			//create color from scalar dialogue
			ccColorFromScalarDlg* cfsDlg = new ccColorFromScalarDlg(this, cloud);
			cfsDlg->setAttribute(Qt::WA_DeleteOnClose, true);
			cfsDlg->show();
		}
	}
}

void MainWindow::doActionInvertNormals()
{
	if (!ccEntityAction::invertNormals(m_selectedEntities))
		return;

	refreshAll();
}

void MainWindow::doActionConvertNormalsToDipDir()
{
	if (!ccEntityAction::convertNormalsTo(m_selectedEntities,
		ccEntityAction::NORMAL_CONVERSION_DEST::DIP_DIR_SFS))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionConvertNormalsToHSV()
{
	if (!ccEntityAction::convertNormalsTo(m_selectedEntities,
		ccEntityAction::NORMAL_CONVERSION_DEST::HSV_COLORS))
	{
		return;
	}

	refreshAll();
	updateUI();
}

static double s_kdTreeMaxErrorPerCell = 0.1;
void MainWindow::doActionComputeKdTree()
{
	ccGenericPointCloud* cloud = nullptr;

	if (haveOneSelection())
	{
		ccHObject* ent = m_selectedEntities.front();
		bool lockedVertices;
		cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
		if (lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
			return;
		}
	}

	if (!cloud)
	{
		ccLog::Error(tr("Selected one and only one point cloud or mesh!"));
		return;
	}

	bool ok;
	s_kdTreeMaxErrorPerCell = QInputDialog::getDouble(this, tr("Compute Kd-tree"), tr("Max error per leaf cell:"), s_kdTreeMaxErrorPerCell, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(true, this);

	//computation
	QElapsedTimer eTimer;
	eTimer.start();
	ccKdTree* kdtree = new ccKdTree(cloud);

	if (kdtree->build(s_kdTreeMaxErrorPerCell, CCCoreLib::DistanceComputationTools::MAX_DIST_95_PERCENT, 4, 1000, &pDlg))
	{
		qint64 elapsedTime_ms = eTimer.elapsed();

		ccConsole::Print("[doActionComputeKdTree] Timing: %2.3f s", elapsedTime_ms / 1.0e3);
		cloud->setEnabled(true); //for mesh vertices!
		cloud->addChild(kdtree);
		kdtree->setDisplay(cloud->getDisplay());
		kdtree->setVisible(true);
		kdtree->prepareDisplayForRefresh();
#ifdef QT_DEBUG
		kdtree->convertCellIndexToSF();
#else
		kdtree->convertCellIndexToRandomColor();
#endif

		addToDB(kdtree);

		refreshAll();
		updateUI();
	}
	else
	{
		ccLog::Error(tr("An error occurred"));
		delete kdtree;
		kdtree = nullptr;
	}
}

void MainWindow::doActionComputeOctree()
{
	if (!ccEntityAction::computeOctree(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionResampleWithOctree()
{
	bool ok;
	int pointCount = QInputDialog::getInt(this, tr("Resample with octree"), tr("Points (approx.)"), 1000000, 1, INT_MAX, 100000, &ok);
	if (!ok)
		return;

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	assert(pointCount > 0);
	unsigned aimedPoints = static_cast<unsigned>(pointCount);

	bool errors = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccPointCloud* cloud = nullptr;

		/*if (ent->isKindOf(CC_TYPES::MESH)) //TODO
			cloud = ccHObjectCaster::ToGenericMesh(ent)->getAssociatedCloud();
		else */
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			cloud = static_cast<ccPointCloud*>(entity);
		}

		if (cloud)
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
			{
				octree = cloud->computeOctree(&pDlg);
				if (!octree)
				{
					ccConsole::Error(tr("Could not compute octree for cloud '%1'").arg(cloud->getName()));
					continue;
				}
			}

			cloud->setEnabled(false);
			QElapsedTimer eTimer;
			eTimer.start();
			CCCoreLib::GenericIndexedCloud* result = CCCoreLib::CloudSamplingTools::resampleCloudWithOctree
			(
				cloud,
				aimedPoints,
				CCCoreLib::CloudSamplingTools::CELL_GRAVITY_CENTER,
				&pDlg,
				octree.data()
			);

			if (result)
			{
				ccConsole::Print("[ResampleWithOctree] Timing: %3.2f s.", eTimer.elapsed() / 1.0e3);
				ccPointCloud* newCloud = ccPointCloud::From(result, cloud);

				delete result;
				result = nullptr;

				if (newCloud)
				{
					addToDB(newCloud);
					newCloud->setDisplay(cloud->getDisplay());
					newCloud->prepareDisplayForRefresh();
				}
				else
				{
					errors = true;
				}
			}
		}
	}

	if (errors)
		ccLog::Error(tr("[ResampleWithOctree] Errors occurred during the process, result may be incomplete"));

	refreshAll();
}

void MainWindow::doActionApplyTransformation()
{
	ccApplyTransformationDlg dlg(this);
	if (!dlg.exec())
		return;

	ccGLMatrixd transMat = dlg.getTransformation();
	applyTransformation(transMat);
}

void MainWindow::applyTransformation(const ccGLMatrixd& mat)
{
	//if the transformation is partly converted to global shift/scale
	bool updateGlobalShiftAndScale = false;
	double scaleChange = 1.0;
	CCVector3d shiftChange(0, 0, 0);
	ccGLMatrixd transMat = mat;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = getSelectedEntities();

	//special case: the selected entity is a group
	//if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	//{
	//	ccHObject* ent = selectedEntities.front();
	//	m_selectedEntities.clear();
	//	for (unsigned i=0; i<ent->getChildrenNumber(); ++i)
	//	{
	//		selectedEntities.push_back(ent->getChild(i));
	//	}
	//}

	bool firstCloud = true;

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		//we don't test primitives (it's always ok while the 'vertices lock' test would fail)
		if (!entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			//specific test for locked vertices
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
			if (cloud)
			{
				if (lockedVertices)
				{
					ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
					continue;
				}

				if (firstCloud)
				{
					//test if the translated cloud was already "too big"
					//(in which case we won't bother the user about the fact
					//that the transformed cloud will be too big...)
					ccBBox localBBox = entity->getOwnBB();
					CCVector3d Pl = localBBox.minCorner();
					double Dl = localBBox.getDiagNormd();

					//the cloud was alright
					if (!ccGlobalShiftManager::NeedShift(Pl)
						&& !ccGlobalShiftManager::NeedRescale(Dl))
					{
						//test if the translated cloud is not "too big" (in local coordinate space)
						ccBBox rotatedBox = entity->getOwnBB() * transMat;
						double Dl2 = rotatedBox.getDiagNorm();
						CCVector3d Pl2 = rotatedBox.getCenter();

						bool needShift = ccGlobalShiftManager::NeedShift(Pl2);
						bool needRescale = ccGlobalShiftManager::NeedRescale(Dl2);

						if (needShift || needRescale)
						{
							//existing shift information
							CCVector3d globalShift = cloud->getGlobalShift();
							double globalScale = cloud->getGlobalScale();

							//we compute the transformation matrix in the global coordinate space
							ccGLMatrixd globalTransMat = transMat;
							globalTransMat.scaleRotation(1.0 / globalScale);
							globalTransMat.setTranslation(globalTransMat.getTranslationAsVec3D() - globalShift);
							//and we apply it to the cloud bounding-box
							ccBBox rotatedBox = cloud->getOwnBB() * globalTransMat;
							double Dg = rotatedBox.getDiagNorm();
							CCVector3d Pg = rotatedBox.getCenter();

							//ask the user the right values!
							ccShiftAndScaleCloudDlg sasDlg(Pl2, Dl2, Pg, Dg, this);
							sasDlg.showApplyAllButton(false);
							sasDlg.showTitle(true);
							sasDlg.setKeepGlobalPos(true);
							sasDlg.showKeepGlobalPosCheckbox(false); //we don't want the user to mess with this!
							sasDlg.showPreserveShiftOnSave(true);

							//add "original" entry
							int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Original"), globalShift, globalScale));
							//sasDlg.setCurrentProfile(index);
							//add "suggested" entry
							CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
							double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
							index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Suggested"), suggestedShift, suggestedScale));
							sasDlg.setCurrentProfile(index);
							//add "last" entries (if any)
							sasDlg.addShiftInfo(ccGlobalShiftManager::GetLast());
							//changebycarl取消弹出窗口
							if (true)
							{
								//store the shift for next time!
								double newScale = sasDlg.getScale();
								CCVector3d newShift = sasDlg.getShift();
								ccGlobalShiftManager::StoreShift(newShift, newScale);

								//get the relative modification to existing global shift/scale info
								assert(cloud->getGlobalScale() != 0);
								scaleChange = newScale / cloud->getGlobalScale();
								shiftChange = newShift - cloud->getGlobalShift();

								updateGlobalShiftAndScale = (scaleChange != 1.0 || shiftChange.norm2() != 0);

								//update transformation matrix accordingly
								if (updateGlobalShiftAndScale)
								{
									transMat.scaleRotation(scaleChange);
									transMat.setTranslation(transMat.getTranslationAsVec3D() + newScale * shiftChange);
								}
							}
							else if (sasDlg.cancelled())
							{
								ccLog::Warning(tr("[ApplyTransformation] Process cancelled by user"));
								return;
							}
						}
					}

					firstCloud = false;
				}

				if (updateGlobalShiftAndScale)
				{
					//apply translation as global shift
					cloud->setGlobalShift(cloud->getGlobalShift() + shiftChange);
					cloud->setGlobalScale(cloud->getGlobalScale() * scaleChange);
					const CCVector3d& T = cloud->getGlobalShift();
					double scale = cloud->getGlobalScale();
					ccLog::Warning(tr("[ApplyTransformation] Cloud '%1' global shift/scale information has been updated: shift = (%2,%3,%4) / scale = %5").arg(cloud->getName()).arg(T.x).arg(T.y).arg(T.z).arg(scale));
				}
			}
		}

		//we temporarily detach entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyRigidTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->setGLTransformation(ccGLMatrix(transMat.data()));
		//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
		entity->applyGLTransformation_recursive();
		entity->prepareDisplayForRefresh_recursive();
		putObjectBackIntoDBTree(entity, objContext);
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	ccLog::Print(tr("[ApplyTransformation] Applied transformation matrix:"));
	ccLog::Print(transMat.toString(12, ' ')); //full precision
	ccLog::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	refreshAll();
}

typedef std::pair<ccHObject*, ccGenericPointCloud*> EntityCloudAssociation;
void MainWindow::doActionApplyScale()
{
	ccScaleDlg dlg(this);
	if (!dlg.exec())
		return;
	dlg.saveState();

	//save values for next time
	CCVector3d scales = dlg.getScales();
	bool keepInPlace = dlg.keepInPlace();
	bool rescaleGlobalShift = dlg.rescaleGlobalShift();

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//first check that all coordinates are kept 'small'
	std::vector< EntityCloudAssociation > candidates;
	{
		bool testBigCoordinates = true;
		//size_t processNum = 0;

		for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
		{
			bool lockedVertices;
			//try to get the underlying cloud (or the vertices set for a mesh)
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
			//otherwise we can look if the selected entity is a polyline
			if (!cloud && entity->isA(CC_TYPES::POLY_LINE))
			{
				cloud = dynamic_cast<ccGenericPointCloud*>(static_cast<ccPolyline*>(entity)->getAssociatedCloud());
				if (!cloud || cloud->isAncestorOf(entity))
					lockedVertices = true;
			}
			if (!cloud || !cloud->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccLog::Warning(tr("[Apply scale] Entity '%1' can't be scaled this way").arg(entity->getName()));
				continue;
			}
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
				//++processNum;
				continue;
			}

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
				C = cloud->getOwnBB().getCenter();

			//we must check that the resulting cloud coordinates are not too big
			if (testBigCoordinates)
			{
				ccBBox bbox = cloud->getOwnBB();
				CCVector3 bbMin = bbox.minCorner();
				CCVector3 bbMax = bbox.maxCorner();

				double maxx = std::max(std::abs(bbMin.x), std::abs(bbMax.x));
				double maxy = std::max(std::abs(bbMin.y), std::abs(bbMax.y));
				double maxz = std::max(std::abs(bbMin.z), std::abs(bbMax.z));

				const double maxCoord = ccGlobalShiftManager::MaxCoordinateAbsValue();
				bool oldCoordsWereTooBig = (maxx > maxCoord
					|| maxy > maxCoord
					|| maxz > maxCoord);

				if (!oldCoordsWereTooBig)
				{
					maxx = std::max(std::abs((bbMin.x - C.x) * scales.x + C.x), std::abs((bbMax.x - C.x) * scales.x + C.x));
					maxy = std::max(std::abs((bbMin.y - C.y) * scales.y + C.y), std::abs((bbMax.y - C.y) * scales.y + C.y));
					maxz = std::max(std::abs((bbMin.z - C.z) * scales.z + C.z), std::abs((bbMax.z - C.z) * scales.z + C.z));

					bool newCoordsAreTooBig = (maxx > maxCoord
						|| maxy > maxCoord
						|| maxz > maxCoord);

					if (newCoordsAreTooBig)
					{
						if (CS::Widgets::FramelessMessageBox::question(
							this,
							tr("Big coordinates"),
							tr("Resutling coordinates will be too big (original precision may be lost!). Proceed anyway?"),
							QMessageBox::Yes,
							QMessageBox::No) == QMessageBox::Yes)
						{
							//ok, we won't test anymore and proceed
							testBigCoordinates = false;
						}
						else
						{
							//we stop the process
							return;
						}
					}
				}
			}

			assert(cloud);
			candidates.emplace_back(entity, cloud);
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning(tr("[Apply scale] No eligible entities (point clouds or meshes) were selected!"));
		return;
	}

	//now do the real scaling work
	{
		for (auto &candidate : candidates)
		{
			ccHObject* ent = candidate.first;
			ccGenericPointCloud* cloud = candidate.second;

			CCVector3 C(0, 0, 0);
			if (keepInPlace)
			{
				C = cloud->getOwnBB().getCenter();
			}

			//we temporarily detach entity, as it may undergo
			//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::scale
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);

			cloud->scale(static_cast<PointCoordinateType>(scales.x),
				static_cast<PointCoordinateType>(scales.y),
				static_cast<PointCoordinateType>(scales.z),
				C);

			putObjectBackIntoDBTree(cloud, objContext);
			cloud->prepareDisplayForRefresh_recursive();

			//don't forget the 'global shift'!
			//DGM: but not the global scale!
			if (rescaleGlobalShift)
			{
				const CCVector3d& shift = cloud->getGlobalShift();
				cloud->setGlobalShift(CCVector3d(shift.x*scales.x,
					shift.y*scales.y,
					shift.z*scales.z));
			}

			ent->prepareDisplayForRefresh_recursive();
		}
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	if (!keepInPlace)
		zoomOnSelectedEntities();

	refreshAll();
	updateUI();
}

void MainWindow::doActionEditGlobalShiftAndScale()
{
	//get the global shift/scale info and bounding box of all selected clouds
	std::vector< std::pair<ccShiftedObject*, ccHObject*> > shiftedEntities;
	CCVector3d Pl(0, 0, 0);
	double Dl = 1.0;
	CCVector3d Pg(0, 0, 0);
	double Dg = 1.0;
	//shift and scale (if unique)
	CCVector3d shift(0, 0, 0);
	double scale = 1.0;
	{
		bool uniqueShift = true;
		bool uniqueScale = true;
		ccBBox localBB;
		//sadly we don't have a double-typed bounding box class yet ;)
		CCVector3d globalBBmin(0, 0, 0);
		CCVector3d globalBBmax(0, 0, 0);

		for (ccHObject *entity : getSelectedEntities())
		{
			bool lockedVertices;
			ccShiftedObject* shifted = ccHObjectCaster::ToShifted(entity, &lockedVertices);
			if (!shifted)
			{
				continue;
			}
			//for (unlocked) entities only
			if (lockedVertices)
			{
				//get the vertices
				assert(entity->isKindOf(CC_TYPES::MESH));
				ccGenericPointCloud* vertices = static_cast<ccGenericMesh*>(entity)->getAssociatedCloud();
				if (!vertices || !entity->isAncestorOf(vertices))
				{
					ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
					continue;
				}
				entity = vertices;
			}

			CCVector3 Al = entity->getOwnBB().minCorner();
			CCVector3 Bl = entity->getOwnBB().maxCorner();
			CCVector3d Ag = shifted->toGlobal3d<PointCoordinateType>(Al);
			CCVector3d Bg = shifted->toGlobal3d<PointCoordinateType>(Bl);

			//update local BB
			localBB.add(Al);
			localBB.add(Bl);

			//update global BB
			if (shiftedEntities.empty())
			{
				globalBBmin = Ag;
				globalBBmax = Bg;
				shift = shifted->getGlobalShift();
				uniqueScale = shifted->getGlobalScale();
			}
			else
			{
				globalBBmin = CCVector3d(std::min(globalBBmin.x, Ag.x),
					std::min(globalBBmin.y, Ag.y),
					std::min(globalBBmin.z, Ag.z));
				globalBBmax = CCVector3d(std::max(globalBBmax.x, Bg.x),
					std::max(globalBBmax.y, Bg.y),
					std::max(globalBBmax.z, Bg.z));

				if (uniqueShift)
				{
					uniqueShift = CCCoreLib::LessThanEpsilon((shifted->getGlobalShift() - shift).norm());
				}
				if (uniqueScale)
				{
					uniqueScale = CCCoreLib::LessThanEpsilon(std::abs(shifted->getGlobalScale() - scale));
				}
			}

			shiftedEntities.emplace_back(shifted, entity);
		}

		Pg = globalBBmin;
		Dg = (globalBBmax - globalBBmin).norm();

		Pl = localBB.minCorner();
		Dl = (localBB.maxCorner() - localBB.minCorner()).normd();

		if (!uniqueShift)
			shift = Pl - Pg;
		if (!uniqueScale)
			scale = Dg / Dl;
	}

	if (shiftedEntities.empty())
	{
		return;
	}

	ccShiftAndScaleCloudDlg sasDlg(Pl, Dl, Pg, Dg, this);
	sasDlg.showApplyAllButton(shiftedEntities.size() > 1);
	sasDlg.showApplyButton(shiftedEntities.size() == 1);
	sasDlg.showNoButton(false);
	sasDlg.setShiftFieldsPrecision(6);
	//add "original" entry
	int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo(tr("Original"), shift, scale));
	sasDlg.setCurrentProfile(index);
	//add "last" entries (if any)
	sasDlg.addShiftInfo(ccGlobalShiftManager::GetLast());

	if (!sasDlg.exec())
		return;

	shift = sasDlg.getShift();
	scale = sasDlg.getScale();
	bool preserveGlobalPos = sasDlg.keepGlobalPos();

	ccLog::Print(tr("[Global Shift/Scale] New shift: (%1, %2, %3)").arg(shift.x).arg(shift.y).arg(shift.z));
	ccLog::Print(tr("[Global Shift/Scale] New scale: %1").arg(scale));

	//apply new shift
	{
		for (auto &entity : shiftedEntities)
		{
			ccShiftedObject* shifted = entity.first;
			ccHObject* ent = entity.second;
			if (preserveGlobalPos)
			{
				//to preserve the global position of the cloud, we may have to translate and/or rescale the cloud
				CCVector3d Ql = ent->getOwnBB().minCorner();
				CCVector3d Qg = shifted->toGlobal3d(Ql);
				CCVector3d Ql2 = Qg * scale + shift;
				CCVector3d T = Ql2 - Ql;

				assert(shifted->getGlobalScale() > 0);
				double scaleCoef = scale / shifted->getGlobalScale();

				if (CCCoreLib::GreaterThanEpsilon(T.norm())
					|| CCCoreLib::GreaterThanEpsilon(std::abs(scaleCoef - 1.0)))
				{
					ccGLMatrix transMat;
					transMat.toIdentity();
					transMat.scaleRotation(static_cast<float>(scaleCoef));
					transMat.setTranslation(T);

					//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
					ent->applyGLTransformation_recursive(&transMat);
					ent->prepareDisplayForRefresh_recursive();

					ccLog::Warning(tr("[Global Shift/Scale] To preserve its original position, the entity '%1' has been translated of (%2 ; %3 ; %4) and rescaled of a factor %5")
						.arg(ent->getName())
						.arg(T.x)
						.arg(T.y)
						.arg(T.z)
						.arg(scaleCoef));
				}
			}
			shifted->setGlobalShift(shift);
			shifted->setGlobalScale(scale);
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doComputeBestFitBB()
{
	if (CS::Widgets::FramelessMessageBox::warning(this,
		tr("This method is for test purpose only"),
		tr("Cloud(s) are going to be rotated while still displayed in their previous position! Proceed?"),
		QMessageBox::Yes | QMessageBox::No,
		QMessageBox::No) != QMessageBox::Yes)
	{
		return;
	}

	//backup selected entities as removeObjectTemporarilyFromDBTree can modify them
	ccHObject::Container selectedEntities = getSelectedEntities();

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
		{
			CCCoreLib::Neighbourhood Yk(cloud);

			CCCoreLib::SquareMatrixd covMat = Yk.computeCovarianceMatrix();
			if (covMat.isValid())
			{
				CCCoreLib::SquareMatrixd eigVectors;
				std::vector<double> eigValues;
				if (CCCoreLib::Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
				{
					CCCoreLib::Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues);

					ccGLMatrix trans;
					GLfloat* rotMat = trans.data();
					for (unsigned j = 0; j < 3; ++j)
					{
						double u[3];
						CCCoreLib::Jacobi<double>::GetEigenVector(eigVectors, j, u);
						CCVector3 v(static_cast<PointCoordinateType>(u[0]),
							static_cast<PointCoordinateType>(u[1]),
							static_cast<PointCoordinateType>(u[2]));
						v.normalize();
						rotMat[j * 4] = static_cast<float>(v.x);
						rotMat[j * 4 + 1] = static_cast<float>(v.y);
						rotMat[j * 4 + 2] = static_cast<float>(v.z);
					}

					const CCVector3* G = Yk.getGravityCenter();
					assert(G);
					trans.shiftRotationCenter(*G);

					cloud->setGLTransformation(trans);
					trans.invert();

					//we temporarily detach entity, as it may undergo
					//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
					ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(cloud);
					static_cast<ccPointCloud*>(cloud)->applyRigidTransformation(trans);
					putObjectBackIntoDBTree(cloud, objContext);

					entity->prepareDisplayForRefresh_recursive();
				}
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionFlagMeshVertices()
{
	bool errors = false;
	bool success = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
			ccPointCloud* vertices = ccHObjectCaster::ToPointCloud(mesh ? mesh->getAssociatedCloud() : nullptr);
			if (mesh && vertices)
			{
				//prepare a new scalar field
				int sfIdx = vertices->getScalarFieldIndexByName(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
				if (sfIdx < 0)
				{
					sfIdx = vertices->addScalarField(CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME);
					if (sfIdx < 0)
					{
						ccConsole::Warning(tr("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
						errors = true;
						continue;
					}
				}
				CCCoreLib::ScalarField* flags = vertices->getScalarField(sfIdx);

				CCCoreLib::MeshSamplingTools::EdgeConnectivityStats stats;
				if (CCCoreLib::MeshSamplingTools::flagMeshVerticesByType(mesh, flags, &stats))
				{
					vertices->setCurrentDisplayedScalarField(sfIdx);
					ccScalarField* sf = vertices->getCurrentDisplayedScalarField();
					if (sf)
					{
						sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::VERTEX_QUALITY));
						//sf->setColorRampSteps(3); //ugly :(
					}
					vertices->showSF(true);
					mesh->showSF(true);
					mesh->prepareDisplayForRefresh_recursive();
					success = true;

					//display stats in the Console as well
					ccConsole::Print(tr("[Mesh Quality] Mesh '%1' edges: %2 total (normal: %3 / on hole borders: %4 / non-manifold: %5)").arg(entity->getName()).arg(stats.edgesCount).arg(stats.edgesSharedByTwo).arg(stats.edgesNotShared).arg(stats.edgesSharedByMore));
				}
				else
				{
					vertices->deleteScalarField(sfIdx);
					sfIdx = -1;
					ccConsole::Warning(tr("Not enough memory to flag the vertices of mesh '%1'!").arg(mesh->getName()));
					errors = true;
				}
			}
			else
			{
				assert(false);
			}
		}
	}

	refreshAll();
	updateUI();

	if (success)
	{
		//display reminder
		forceConsoleDisplay();
		ccConsole::Print(tr("[Mesh Quality] SF flags: %1 (NORMAL) / %2 (BORDER) / (%3) NON-MANIFOLD").arg(CCCoreLib::MeshSamplingTools::VERTEX_NORMAL).arg(CCCoreLib::MeshSamplingTools::VERTEX_BORDER).arg(CCCoreLib::MeshSamplingTools::VERTEX_NON_MANIFOLD));
	}

	if (errors)
	{
		ccConsole::Error(tr("Error(s) occurred! Check the console..."));
	}
}


void MainWindow::doActionMeasureMeshSurface()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
			if (mesh)
			{
				double S = CCCoreLib::MeshSamplingTools::computeMeshArea(mesh);
				//we force the console to display itself
				forceConsoleDisplay();
				ccConsole::Print(tr("[Mesh Surface] Mesh '%1': S=%2 (square units)").arg(entity->getName()).arg(S));
				if (mesh->size())
				{
					ccConsole::Print(tr("[Mesh Surface] Average triangle surface: %1 (square units)").arg(S / double(mesh->size())));
				}
			}
			else
			{
				assert(false);
			}
		}
	}
}

void MainWindow::doActionComputeDistancesFromSensor()
{
	//we support more than just one sensor in selection
	if (!haveSelection())
	{
		ccConsole::Error(tr("Select at least one sensor"));
		return;
	}

	//start dialog
	ccSensorComputeDistancesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccSensor* sensor = ccHObjectCaster::ToSensor(entity);
		assert(sensor);
		if (!sensor)
			continue; //skip this entity

		//get associated cloud
		ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
		ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
		if (!cloud)
		{
			return;
		}

		//sensor center
		CCVector3 sensorCenter;
		if (!sensor->getActiveAbsoluteCenter(sensorCenter))
			return;

		//squared required?
		bool squared = cdDlg.computeSquaredDistances();

		//set up a new scalar field
		const char* defaultRangesSFname = squared ? CC_DEFAULT_SQUARED_RANGES_SF_NAME : CC_DEFAULT_RANGES_SF_NAME;
		int sfIdx = cloud->getScalarFieldIndexByName(defaultRangesSFname);
		if (sfIdx < 0)
		{
			sfIdx = cloud->addScalarField(defaultRangesSFname);
			if (sfIdx < 0)
			{
				ccConsole::Error(tr("Not enough memory!"));
				return;
			}
		}
		CCCoreLib::ScalarField* distances = cloud->getScalarField(sfIdx);

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			ScalarType s = static_cast<ScalarType>(squared ? (*P - sensorCenter).norm2() : (*P - sensorCenter).norm());
			distances->setValue(i, s);
		}

		distances->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh_recursive();
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeScatteringAngles()
{
	//there should be only one sensor in current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one GBL sensor!"));
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities.front());
	assert(sensor);

	//sensor center
	CCVector3 sensorCenter;
	if (!sensor->getActiveAbsoluteCenter(sensorCenter))
		return;

	//get associated cloud
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* cloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
	if (!cloud)
	{
		return;
	}
	if (!cloud->hasNormals())
	{
		ccConsole::Error(tr("The cloud must have normals!"));
		return;
	}

	ccSensorComputeScatteringAnglesDlg cdDlg(this);
	if (!cdDlg.exec())
		return;

	bool toDegreeFlag = cdDlg.anglesInDegrees();

	//prepare a new scalar field
	const char* defaultScatAnglesSFname = toDegreeFlag ? CC_DEFAULT_DEG_SCATTERING_ANGLES_SF_NAME : CC_DEFAULT_RAD_SCATTERING_ANGLES_SF_NAME;
	int sfIdx = cloud->getScalarFieldIndexByName(defaultScatAnglesSFname);
	if (sfIdx < 0)
	{
		sfIdx = cloud->addScalarField(defaultScatAnglesSFname);
		if (sfIdx < 0)
		{
			ccConsole::Error(tr("Not enough memory!"));
			return;
		}
	}
	CCCoreLib::ScalarField* angles = cloud->getScalarField(sfIdx);

	//perform computations
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		//the point position
		const CCVector3* P = cloud->getPoint(i);

		//build the ray
		CCVector3 ray = *P - sensorCenter;
		ray.normalize();

		//get the current normal
		CCVector3 normal(cloud->getPointNormal(i));
		//normal.normalize(); //should already be the case!

		//compute the angle
		PointCoordinateType cosTheta = ray.dot(normal);
		ScalarType theta = std::acos(std::min(std::abs(cosTheta), 1.0f));

		if (toDegreeFlag)
		{
			theta = CCCoreLib::RadiansToDegrees(theta);
		}

		angles->setValue(i, theta);
	}

	angles->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);
	cloud->prepareDisplayForRefresh_recursive();

	refreshAll();
	updateUI();
}

void MainWindow::doActionSetViewFromSensor()
{
	//there should be only one sensor in current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error(tr("Select one and only one sensor!"));
		return;
	}

	ccSensor* sensor = ccHObjectCaster::ToSensor(m_selectedEntities.front());
	assert(sensor);

	//try to find the associated window
	ccGenericGLDisplay* win = sensor->getDisplay();
	if (!win)
	{
		//get associated cloud
		ccPointCloud * cloud = ccHObjectCaster::ToPointCloud(sensor->getParent());
		if (cloud)
			win = cloud->getDisplay();
	}

	if (sensor->applyViewport(win))
	{
		ccConsole::Print(tr("[DoActionSetViewFromSensor] Viewport applied"));
	}
}

void MainWindow::doActionCreateGBLSensor()
{
	ccGBLSensorProjectionDlg spDlg(this);
	if (!spDlg.exec())
		return;

	//We create the corresponding sensor for each input cloud (in a perfect world, there should be only one ;)
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

			//we create a new sensor
			ccGBLSensor* sensor = new ccGBLSensor();

			//we init its parameters with the dialog
			spDlg.updateGBLSensor(sensor);

			//we compute projection
			if (sensor->computeAutoParameters(cloud))
			{
				cloud->addChild(sensor);

				//we try to guess the sensor relative size (dirty)
				ccBBox bb = cloud->getOwnBB();
				double diag = bb.getDiagNorm();
				if (diag < 1.0)
					sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e-3));
				else if (diag > 10000.0)
					sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e3));

				//we display depth buffer
				int errorCode;
				if (sensor->computeDepthBuffer(cloud, errorCode))
				{
					ccRenderingTools::ShowDepthBuffer(sensor, this);
				}
				else
				{
					ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
				}

				////DGM: test
				//{
				//	//add positions
				//	const unsigned count = 1000;
				//	const PointCoordinateType R = 100;
				//	const PointCoordinateType dh = 100;
				//	for (unsigned i=0; i<1000; ++i)
				//	{
				//		float angle = (float)i/(float)count * 6 * M_PI;
				//		float X = R * cos(angle);
				//		float Y = R * sin(angle);
				//		float Z = (float)i/(float)count * dh;

				//		ccIndexedTransformation trans;
				//		trans.initFromParameters(-angle,CCVector3(0,0,1),CCVector3(X,Y,Z));
				//		sensor->addPosition(trans,i);
				//	}
				//}

				//set position
				//ccIndexedTransformation trans;
				//sensor->addPosition(trans,0);

				ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
				if (win)
				{
					sensor->setDisplay_recursive(win);
					sensor->setVisible(true);
					ccBBox box = cloud->getOwnBB();
					win->updateConstellationCenterAndZoom(&box);
				}

				addToDB(sensor);
			}
			else
			{
				ccLog::Error(tr("Failed to create sensor"));
				delete sensor;
				sensor = nullptr;
			}
		}
	}

	updateUI();
}

void MainWindow::doActionCreateCameraSensor()
{
	//we create the camera sensor
	ccCameraSensor* sensor = new ccCameraSensor();

	ccHObject* ent = nullptr;
	if (haveSelection())
	{
		assert(haveOneSelection());
		ent = m_selectedEntities.front();
	}

	//we try to guess the sensor relative size (dirty)
	if (ent && ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
		ccBBox bb = cloud->getOwnBB();
		double diag = bb.getDiagNorm();
		if (diag < 1.0)
			sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e-3));
		else if (diag > 10000.0)
			sensor->setGraphicScale(static_cast<PointCoordinateType>(1.0e3));

		//set position
		ccIndexedTransformation trans;
		sensor->addPosition(trans, 0);
	}

	ccCamSensorProjectionDlg spDlg(this);
	//spDlg.initWithCamSensor(sensor); //DGM: we'd better leave the default parameters of the dialog!
	if (!spDlg.exec())
	{
		delete sensor;
		return;
	}
	spDlg.updateCamSensor(sensor);

	ccGLWindow* win = nullptr;
	if (ent)
	{
		ent->addChild(sensor);
		win = static_cast<ccGLWindow*>(ent->getDisplay());
	}
	else
	{
		win = getActiveGLWindow();
	}

	if (win)
	{
		sensor->setDisplay(win);
		sensor->setVisible(true);
		if (ent)
		{
			ccBBox box = ent->getOwnBB();
			win->updateConstellationCenterAndZoom(&box);
		}
	}

	addToDB(sensor);

	updateUI();
}

void MainWindow::doActionModifySensor()
{
	//there should be only one point cloud with sensor in current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::SENSOR))
	{
		ccConsole::Error(tr("Select one and only one sensor!"));
		return;
	}

	ccSensor* sensor = static_cast<ccSensor*>(m_selectedEntities.front());

	//Ground based laser sensors
	if (sensor->isA(CC_TYPES::GBL_SENSOR))
	{
		ccGBLSensor* gbl = static_cast<ccGBLSensor*>(sensor);

		ccGBLSensorProjectionDlg spDlg(this);
		spDlg.initWithGBLSensor(gbl);

		if (!spDlg.exec())
			return;

		//we update its parameters
		spDlg.updateGBLSensor(gbl);

		//we re-project the associated cloud (if any)
		if (gbl->getParent() && gbl->getParent()->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(gbl->getParent());

			int errorCode;
			if (gbl->computeDepthBuffer(cloud, errorCode))
			{
				//we display depth buffer
				ccRenderingTools::ShowDepthBuffer(gbl, this);
			}
			else
			{
				ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
			}
		}
		else
		{
			//ccConsole::Warning(tr("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
		}
	}
	//Camera sensors
	else if (sensor->isA(CC_TYPES::CAMERA_SENSOR))
	{
		ccCameraSensor* cam = static_cast<ccCameraSensor*>(sensor);

		ccCamSensorProjectionDlg spDlg(this);
		spDlg.initWithCamSensor(cam);

		if (!spDlg.exec())
			return;

		//we update its parameters
		spDlg.updateCamSensor(cam);
	}
	else
	{
		ccConsole::Error(tr("Can't modify this kind of sensor!"));
		return;
	}

	if (sensor->isVisible() && sensor->isEnabled())
	{
		sensor->prepareDisplayForRefresh();
		refreshAll();
	}

	updateUI();
}

void MainWindow::doActionProjectUncertainty()
{
	//there should only be one sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one camera (projective) sensor!"));
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities.front());
	if (!sensor)
	{
		assert(false);
		return;
	}

	const ccCameraSensor::LensDistortionParameters::Shared& distParams = sensor->getDistortionParameters();
	if (!distParams || distParams->getModel() != ccCameraSensor::BROWN_DISTORTION)
	{
		ccLog::Error(tr("Sensor has no associated uncertainty model! (Brown, etc.)"));
		return;
	}

	//we need a cloud to project the uncertainty on!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud on which to project the uncertainty:"));
	if (!pointCloud)
	{
		return;
	}

	CCCoreLib::ReferenceCloud points(pointCloud);
	if (!points.reserve(pointCloud->size()))
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}
	points.addPointIndex(0, pointCloud->size());

	// compute uncertainty
	std::vector< Vector3Tpl<ScalarType> > accuracy;
	if (!sensor->computeUncertainty(&points, accuracy/*, false*/))
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	/////////////
	// SIGMA D //
	/////////////
	const char dimChar[3] = { 'x','y','z' };
	for (unsigned d = 0; d < 3; ++d)
	{
		// add scalar field
		QString sfName = tr("[%1] Uncertainty (%2)").arg(sensor->getName()).arg(dimChar[d]);
		int sfIdx = pointCloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccLog::Error(tr("An error occurred! (see console)"));
			return;
		}

		// fill scalar field
		CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i = 0; i < count; i++)
				sf->setValue(i, accuracy[i].u[d]);
			sf->computeMinAndMax();
		}
	}

	/////////////////
	// SIGMA TOTAL //
	/////////////////

	// add scalar field
	{
		QString sfName = tr("[%1] Uncertainty (3D)").arg(sensor->getName());
		int sfIdx = pointCloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = pointCloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccLog::Error(tr("An error occurred! (see console)"));
			return;
		}

		// fill scalar field
		CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
		assert(sf);
		if (sf)
		{
			unsigned count = static_cast<unsigned>(accuracy.size());
			assert(count == pointCloud->size());
			for (unsigned i = 0; i < count; i++)
				sf->setValue(i, accuracy[i].norm());
			sf->computeMinAndMax();
		}

		pointCloud->showSF(true);
		pointCloud->setCurrentDisplayedScalarField(sfIdx);
		pointCloud->prepareDisplayForRefresh();
	}

	refreshAll();
}

void MainWindow::doActionCheckPointsInsideFrustum()
{
	//there should be only one camera sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::CAMERA_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one camera sensor!"));
		return;
	}

	ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(m_selectedEntities.front());
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud to filter:"));
	if (!pointCloud)
	{
		return;
	}

	//comupte/get the point cloud's octree
	ccOctree::Shared octree = pointCloud->getOctree();
	if (!octree)
	{
		octree = pointCloud->computeOctree();
		if (!octree)
		{
			ccConsole::Error(tr("Failed to compute the octree!"));
			return;
		}
	}
	assert(octree);

	// filter octree then project the points
	std::vector<unsigned> inCameraFrustum;
	if (!octree->intersectWithFrustum(sensor, inCameraFrustum))
	{
		ccConsole::Error(tr("Failed to intersect sensor frustum with octree!"));
	}
	else
	{
		// scalar field
		const char sfName[] = "Frustum visibility";
		int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);

		if (inCameraFrustum.empty())
		{
			ccConsole::Error(tr("No point fell inside the frustum!"));
			if (sfIdx >= 0)
				pointCloud->deleteScalarField(sfIdx);
		}
		else
		{
			if (sfIdx < 0)
				sfIdx = pointCloud->addScalarField(sfName);
			if (sfIdx < 0)
			{
				ccLog::Error(tr("Failed to allocate memory for output scalar field!"));
				return;
			}

			CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
			assert(sf);
			if (sf)
			{
				sf->fill(0);

				const ScalarType c_insideValue = static_cast<ScalarType>(1);

				for (unsigned index : inCameraFrustum)
				{
					sf->setValue(index, c_insideValue);
				}

				sf->computeMinAndMax();
				pointCloud->setCurrentDisplayedScalarField(sfIdx);
				pointCloud->showSF(true);

				pointCloud->redrawDisplay();
			}
		}
	}

	updateUI();
}

void MainWindow::doActionShowDepthBuffer()
{
	if (!haveSelection())
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = static_cast<ccGBLSensor*>(m_selectedEntities.front());
			if (sensor->getDepthBuffer().zBuff.empty())
			{
				//look for depending cloud
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity->getParent());
				if (cloud)
				{
					//force depth buffer computation
					int errorCode;
					if (!sensor->computeDepthBuffer(cloud, errorCode))
					{
						ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
					}
				}
				else
				{
					ccConsole::Error(tr("Internal error: sensor ('%1') parent is not a point cloud!").arg(sensor->getName()));
					return;
				}
			}

			ccRenderingTools::ShowDepthBuffer(sensor, this);
		}
	}
}

void MainWindow::doActionExportDepthBuffer()
{
	if (!haveSelection())
		return;

	//persistent settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString filename = CS::Widgets::FramelessFileDialog::getSaveFileName(this, tr("Select output file"), currentPath, DepthMapFileFilter::GetFileFilter());
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(filename).absolutePath());
	settings.endGroup();

	ccHObject* toSave = nullptr;
	bool multEntities = false;
	if (haveOneSelection())
	{
		toSave = m_selectedEntities.front();
	}
	else
	{
		toSave = new ccHObject("Temp Group");

		for (ccHObject *entity : getSelectedEntities())
		{
			toSave->addChild(entity, ccHObject::DP_NONE);
		}
		multEntities = true;
	}

	DepthMapFileFilter::SaveParameters parameters;
	{
		parameters.alwaysDisplaySaveDialog = true;
	}
	CC_FILE_ERROR result = DepthMapFileFilter().saveToFile(toSave, filename, parameters);

	if (result != CC_FERR_NO_ERROR)
	{
		FileIOFilter::DisplayErrorMessage(result, tr("saving"), filename);
	}
	else
	{
		ccLog::Print(tr("[I/O] File '%1' saved successfully").arg(filename));
	}

	if (multEntities)
	{
		delete toSave;
		toSave = nullptr;
	}
}

void MainWindow::doActionComputePointsVisibility()
{
	//there should be only one camera sensor in the current selection!
	if (!haveOneSelection() || !m_selectedEntities.front()->isKindOf(CC_TYPES::GBL_SENSOR))
	{
		ccConsole::Error(tr("Select one and only one GBL/TLS sensor!"));
		return;
	}

	ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(m_selectedEntities.front());
	if (!sensor)
		return;

	//we need a cloud to filter!
	ccHObject* defaultCloud = sensor->getParent() && sensor->getParent()->isA(CC_TYPES::POINT_CLOUD) ? sensor->getParent() : nullptr;
	ccPointCloud* pointCloud = askUserToSelectACloud(defaultCloud, tr("Select a cloud to filter:"));
	if (!pointCloud)
	{
		return;
	}

	if (sensor->getDepthBuffer().zBuff.empty())
	{
		if (defaultCloud)
		{
			//the sensor has no depth buffer, we'll ask the user if he wants to compute it first
			if (CS::Widgets::FramelessMessageBox::warning(this,
				tr("Depth buffer"),
				tr("Sensor has no depth buffer: do you want to compute it now?"),
				QMessageBox::Yes | QMessageBox::No,
				QMessageBox::Yes) == QMessageBox::No)
			{
				//we can stop then...
				return;
			}

			int errorCode;
			if (sensor->computeDepthBuffer(static_cast<ccPointCloud*>(defaultCloud), errorCode))
			{
				ccRenderingTools::ShowDepthBuffer(sensor, this);
			}
			else
			{
				ccConsole::Error(ccGBLSensor::GetErrorString(errorCode));
				return;
			}
		}
		else
		{
			ccConsole::Error(tr("Sensor has no depth buffer (and no associated cloud?)"));
			return;
		}
	}

	// scalar field
	const char sfName[] = "Sensor visibility";
	int sfIdx = pointCloud->getScalarFieldIndexByName(sfName);
	if (sfIdx < 0)
		sfIdx = pointCloud->addScalarField(sfName);
	if (sfIdx < 0)
	{
		ccLog::Error(tr("Failed to allocate memory for output scalar field!"));
		return;
	}

	CCCoreLib::ScalarField* sf = pointCloud->getScalarField(sfIdx);
	assert(sf);
	if (sf)
	{
		sf->fill(0);

		//progress bar
		ccProgressDialog pdlg(true);
		CCCoreLib::NormalizedProgress nprogress(&pdlg, pointCloud->size());
		pdlg.setMethodTitle(tr("Compute visibility"));
		pdlg.setInfo(tr("Points: %L1").arg(pointCloud->size()));
		pdlg.start();
		QApplication::processEvents();

		for (unsigned i = 0; i < pointCloud->size(); i++)
		{
			const CCVector3* P = pointCloud->getPoint(i);
			unsigned char visibility = sensor->checkVisibility(*P);
			ScalarType visValue = static_cast<ScalarType>(visibility);

			sf->setValue(i, visValue);

			if (!nprogress.oneStep())
			{
				//cancelled by user
				pointCloud->deleteScalarField(sfIdx);
				sf = nullptr;
				break;
			}
		}

		if (sf)
		{
			sf->computeMinAndMax();
			pointCloud->setCurrentDisplayedScalarField(sfIdx);
			pointCloud->showSF(true);

			ccConsole::Print(tr("Visibility computed for cloud '%1'").arg(pointCloud->getName()));
			ccConsole::Print(tr("\tVisible = %1").arg(CCCoreLib::POINT_VISIBLE));
			ccConsole::Print(tr("\tHidden = %1").arg(CCCoreLib::POINT_HIDDEN));
			ccConsole::Print(tr("\tOut of range = %1").arg(CCCoreLib::POINT_OUT_OF_RANGE));
			ccConsole::Print(tr("\tOut of fov = %1").arg(CCCoreLib::POINT_OUT_OF_FOV));
		}
		pointCloud->redrawDisplay();
	}

	updateUI();
}

void MainWindow::doActionConvertTextureToColor()
{
	if (!ccEntityAction::convertTextureToColor(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSamplePointsOnMesh()
{
    InformationPrompt set(QApplication::translate("importantinformationtips", "The smaller the sampling percentage, the fewer retained meshes.", nullptr));
	static double s_ptsSamplingCount = 1000000;
	static double s_ptsSamplingDensity = 10.0;
	static bool s_useDensity = false;

	ccPtsSamplingDlg dlg(this);
    connect(&dlg, &ccPtsSamplingDlg::signalSendStatusBarInfo, this, &MainWindow::updateStatusbarInformation, Qt::UniqueConnection);
	dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Point Sampling on Mesh", nullptr));
	//restore last parameters
	if (!dlg.exec())
		return;

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	bool withNormals = true;
	bool withRGB = true;
	bool withTexture = true;
	s_useDensity = dlg.useDensity();
	assert(dlg.getPointsNumber() >= 0);
	double ptsSamplingCount = dlg.getPointsNumber();
	s_ptsSamplingDensity = 1.0 / dlg.getDensityValue();//采样密度为网格分辨率的倒数

	bool errors = false;
	int errortype = 0;
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::MESH))
			continue;

		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);
		assert(mesh);
        if (mesh->getAssociatedCloud())
        {
            s_ptsSamplingCount = ptsSamplingCount * mesh->getAssociatedCloud()->size();
        }
		ccPointCloud* cloud = mesh->samplePoints(s_useDensity,
			s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
			withNormals,
			withRGB,
			withTexture,
			&pDlg,&errortype);

		if (cloud)
		{
			addToDB(cloud);
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error(tr("[doActionSamplePointsOnMesh] Errors occurred during the process! Result may be incomplete!"));
		if (errortype == 1)
		{
			CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("No point was generated (sampling density is too low)?"));
		}
		else if(errortype == 2)
		{
			CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Not enough memory!"));
		}

	}

	refreshAll();
}

void MainWindow::doActionSamplePointsOnPolyline()
{
	static double s_ptsSamplingCount = 1000;
	static double s_ptsSamplingDensity = 10.0;
	static bool s_useDensity = false;

	ccPtsSamplingDlg dlg(this);
	//restore last parameters
//	dlg.optionsFrame->setVisible(false);
	if (!dlg.exec())
		return;

	assert(dlg.getPointsNumber() >= 0);
	s_ptsSamplingCount = dlg.getPointsNumber();
	s_ptsSamplingDensity = dlg.getDensityValue();
	s_useDensity = dlg.useDensity();

	bool errors = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::POLY_LINE))
			continue;

		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccPointCloud* cloud = poly->samplePoints(s_useDensity,
			s_useDensity ? s_ptsSamplingDensity : s_ptsSamplingCount,
			true);

		if (cloud)
		{
			addToDB(cloud);
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error(tr("[DoActionSamplePointsOnPolyline] Errors occurred during the process! Result may be incomplete!"));
	}

	refreshAll();
}

void MainWindow::doActionSmoohPolyline()
{
	static int s_iterationCount = 5;
	static double s_ratio = 0.25;

	ccSmoothPolylineDialog dlg(this);
	//restore last parameters
	dlg.setIerationCount(s_iterationCount);
	dlg.setRatio(s_ratio);
	if (!dlg.exec())
		return;

	s_iterationCount = dlg.getIerationCount();
	s_ratio = dlg.getRatio();

	bool errors = false;

	ccHObject::Container selectedEntities = getSelectedEntities();
	m_ccRoot->unselectAllEntities();

	for (ccHObject *entity : selectedEntities)
	{
		if (!entity->isKindOf(CC_TYPES::POLY_LINE))
			continue;

		ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
		assert(poly);

		ccPolyline* smoothPoly = poly->smoothChaikin(s_ratio, static_cast<unsigned>(s_iterationCount));
		if (smoothPoly)
		{
			if (poly->getParent())
			{
				poly->getParent()->addChild(smoothPoly);
			}
			poly->setEnabled(false);
			addToDB(smoothPoly);

			m_ccRoot->selectEntity(smoothPoly, true);
		}
		else
		{
			errors = true;
		}
	}

	if (errors)
	{
		ccLog::Error(tr("[DoActionSmoohPolyline] Errors occurred during the process! Result may be incomplete!"));
	}

	refreshAll();
}

void MainWindow::doRemoveDuplicatePoints()
{
	if (!haveSelection())
		return;

	bool first = true;

	//persistent setting(s)
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::DuplicatePointsGroup());
	double minDistanceBetweenPoints = settings.value(ccPS::DuplicatePointsMinDist(), 1.0e-12).toDouble();

	bool ok;
	minDistanceBetweenPoints = QInputDialog::getDouble(this, tr("Remove duplicate points"), tr("Min. Space between points:"), minDistanceBetweenPoints, 0, 1.0e8, 12, &ok);
	if (!ok)
		return;

	//save parameter
	settings.setValue(ccPS::DuplicatePointsMinDist(), minDistanceBetweenPoints);

	static const char DEFAULT_DUPLICATE_TEMP_SF_NAME[] = "DuplicateFlags";

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectAllEntities' and 'selectEntity' will change the set of currently selected entities!

	for (ccHObject *entity : selectedEntities)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			//create temporary SF for 'duplicate flags'
			int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_DUPLICATE_TEMP_SF_NAME);
			if (sfIdx < 0)
				sfIdx = cloud->addScalarField(DEFAULT_DUPLICATE_TEMP_SF_NAME);
			if (sfIdx >= 0)
				cloud->setCurrentScalarField(sfIdx);
			else
			{
				ccConsole::Error(tr("Couldn't create temporary scalar field! Not enough memory?"));
				break;
			}

			ccOctree::Shared octree = cloud->getOctree();

			CCCoreLib::GeometricalAnalysisTools::ErrorCode result = CCCoreLib::GeometricalAnalysisTools::FlagDuplicatePoints(cloud,
				minDistanceBetweenPoints,
				&pDlg,
				octree.data());

			if (result == CCCoreLib::GeometricalAnalysisTools::NoError)
			{
				//count the number of duplicate points!
				CCCoreLib::ScalarField* flagSF = cloud->getScalarField(sfIdx);
				unsigned duplicateCount = 0;
				assert(flagSF);
				if (flagSF)
				{
					for (unsigned j = 0; j < flagSF->currentSize(); ++j)
					{
						if (flagSF->getValue(j) != 0)
						{
							++duplicateCount;
						}
					}
				}

				if (duplicateCount == 0)
				{
					ccConsole::Print(tr("Cloud '%1' has no duplicate points").arg(cloud->getName()));
				}
				else
				{
					ccConsole::Warning(tr("Cloud '%1' has %2 duplicate point(s)").arg(cloud->getName()).arg(duplicateCount));

					ccPointCloud* filteredCloud = cloud->filterPointsByScalarValue(0, 0);
					if (filteredCloud)
					{
						int sfIdx2 = filteredCloud->getScalarFieldIndexByName(DEFAULT_DUPLICATE_TEMP_SF_NAME);
						assert(sfIdx2 >= 0);
						filteredCloud->deleteScalarField(sfIdx2);
						filteredCloud->setName(QString("%1.clean").arg(cloud->getName()));
						filteredCloud->setDisplay(cloud->getDisplay());
						filteredCloud->prepareDisplayForRefresh();
						addToDB(filteredCloud);
						if (first)
						{
							m_ccRoot->unselectAllEntities();
							first = false;
						}
						cloud->setEnabled(false);
						m_ccRoot->selectEntity(filteredCloud, true);
					}
				}
			}
			else
			{
				ccConsole::Error(tr("An error occurred! (Not enough memory?)"));
			}

			cloud->deleteScalarField(sfIdx);
		}
	}

	if (!first)
		ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));

	refreshAll();
}

void MainWindow::doActionFilterByValue()
{
	typedef std::pair<ccHObject*, ccPointCloud*> EntityAndVerticesType;
	std::vector<EntityAndVerticesType> toFilter;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			//la methode est activee sur le champ scalaire affiche
			CCCoreLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				toFilter.emplace_back(entity, pc);
			}
			else
			{
				ccConsole::Warning(tr("Entity [%1] has no active scalar field!").arg(entity->getName()));
			}
		}
	}

	if (toFilter.empty())
		return;

	double minVald = 0.0;
	double maxVald = 1.0;

	//compute min and max "displayed" scalar values of currently selected
	//entities (the ones with an active scalar field only!)
	{
		for (size_t i = 0; i < toFilter.size(); ++i)
		{
			ccScalarField* sf = toFilter[i].second->getCurrentDisplayedScalarField();
			assert(sf);

			if (i == 0)
			{
				minVald = static_cast<double>(sf->displayRange().start());
				maxVald = static_cast<double>(sf->displayRange().stop());
			}
			else
			{
				if (minVald > static_cast<double>(sf->displayRange().start()))
					minVald = static_cast<double>(sf->displayRange().start());
				if (maxVald < static_cast<double>(sf->displayRange().stop()))
					maxVald = static_cast<double>(sf->displayRange().stop());
			}
		}
	}

	ccFilterByValueDlg dlg(minVald, maxVald, -1.0e9, 1.0e9, this);
	if (!dlg.exec())
		return;

	ccFilterByValueDlg::Mode mode = dlg.mode();
	assert(mode != ccFilterByValueDlg::CANCEL);

	ScalarType minVal = static_cast<ScalarType>(dlg.minDoubleSpinBox->value());
	ScalarType maxVal = static_cast<ScalarType>(dlg.maxDoubleSpinBox->value());

	ccHObject::Container results;
	{
		for (auto &item : toFilter)
		{
			ccHObject* ent = item.first;
			ccPointCloud* pc = item.second;
			//CCCoreLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			//assert(sf);

			//we set as output (OUT) the currently displayed scalar field
			int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			assert(outSfIdx >= 0);
			pc->setCurrentOutScalarField(outSfIdx);
			//pc->setCurrentScalarField(outSfIdx);

			ccHObject* resultInside = nullptr;
			ccHObject* resultOutside = nullptr;
			if (ent->isKindOf(CC_TYPES::MESH))
			{
				pc->hidePointsByScalarValue(minVal, maxVal);
				if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
					resultInside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false);
				else if (ent->isA(CC_TYPES::SUB_MESH))
					resultInside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);

				if (mode == ccFilterByValueDlg::SPLIT)
				{
					pc->invertVisibilityArray();
					if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
						resultOutside = ccHObjectCaster::ToMesh(ent)->createNewMeshFromSelection(false);
					else if (ent->isA(CC_TYPES::SUB_MESH))
						resultOutside = ccHObjectCaster::ToSubMesh(ent)->createNewSubMeshFromSelection(false);
				}

				pc->unallocateVisibilityArray();
			}
			else if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				//pc->hidePointsByScalarValue(minVal,maxVal);
				//result = ccHObjectCaster::ToGenericPointCloud(ent)->hidePointsByScalarValue(false);
				//pc->unallocateVisibilityArray();

				//shortcut, as we know here that the point cloud is a "ccPointCloud"
				resultInside = pc->filterPointsByScalarValue(minVal, maxVal, false);

				if (mode == ccFilterByValueDlg::SPLIT)
				{
					resultOutside = pc->filterPointsByScalarValue(minVal, maxVal, true);
				}
			}

			if (resultInside)
			{
				ent->setEnabled(false);
				resultInside->setDisplay(ent->getDisplay());
				resultInside->prepareDisplayForRefresh();
				addToDB(resultInside);

				results.push_back(resultInside);
			}
			if (resultOutside)
			{
				ent->setEnabled(false);
				resultOutside->setDisplay(ent->getDisplay());
				resultOutside->prepareDisplayForRefresh();
				resultOutside->setName(resultOutside->getName() + ".outside");
				addToDB(resultOutside);

				results.push_back(resultOutside);
			}
			//*/
		}
	}

	if (!results.empty())
	{
		ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
		if (m_ccRoot)
		{
			m_ccRoot->selectEntities(results);
		}
	}

	refreshAll();
}

void MainWindow::doActionSFConvertToRandomRGB()
{
	if (!ccEntityAction::sfConvertToRandomRGB(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFConvertToRGB()
{
	if (!ccEntityAction::sfConvertToRGB(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionToggleActiveSFColorScale()
{
	doApplyActiveSFAction(0);
}

void MainWindow::doActionShowActiveSFPrevious()
{
	doApplyActiveSFAction(1);
}

void MainWindow::doActionShowActiveSFNext()
{
	doApplyActiveSFAction(2);
}

void MainWindow::doApplyActiveSFAction(int action)
{
	if (!haveOneSelection())
	{
		if (haveSelection())
		{
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		}
		return;
	}
	ccHObject* ent = m_selectedEntities.front();

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;
	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		//see ccPropertiesTreeDelegate::fillWithMesh
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
		return;
	}

	assert(cloud);
	int sfIdx = cloud->getCurrentDisplayedScalarFieldIndex();
	switch (action)
	{
	case 0: //Toggle SF color scale
		if (sfIdx >= 0)
		{
			cloud->showSFColorsScale(!cloud->sfColorScaleShown());
			cloud->prepareDisplayForRefresh();
		}
		else
			ccConsole::Warning(tr("No active scalar field on entity '%1'").arg(ent->getName()));
		break;
	case 1: //Activate previous SF
		if (sfIdx >= 0)
		{
			cloud->setCurrentDisplayedScalarField(sfIdx - 1);
			cloud->prepareDisplayForRefresh();
		}
		break;
	case 2: //Activate next SF
		if (sfIdx + 1 < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			cloud->setCurrentDisplayedScalarField(sfIdx + 1);
			cloud->prepareDisplayForRefresh();
		}
		break;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionRenameSF()
{
	if (!ccEntityAction::sfRename(m_selectedEntities, this))
		return;

	updateUI();
}

void MainWindow::doActionOpenColorScalesManager()
{
	ccColorScaleEditorDialog cseDlg(ccColorScalesManager::GetUniqueInstance(), this, ccColorScale::Shared(nullptr), this);

	if (cseDlg.exec())
	{
		//save current scale manager state to persistent settings
		ccColorScalesManager::GetUniqueInstance()->toPersistentSettings();
	}

	updateUI();
}

void MainWindow::doActionAddIdField()
{
	if (!ccEntityAction::sfAddIdField(m_selectedEntities))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSplitCloudUsingSF()
{
	if (!ccEntityAction::sfSplitCloud(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFGaussianFilter()
{
	if (!ccEntityAction::sfGaussianFilter(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFBilateralFilter()
{
	if (!ccEntityAction::sfBilateralFilter(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSmoothMeshSF()
{
	if (!ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::SMOOTH_MESH_SF, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionEnhanceMeshSF()
{
	if (!ccEntityAction::processMeshSF(m_selectedEntities, ccMesh::ENHANCE_MESH_SF, this))
		return;

	refreshAll();
	updateUI();
}

static double s_subdivideMaxArea = 1.0;
void MainWindow::doActionSubdivideMesh()
{
	bool ok;
	s_subdivideMaxArea = QInputDialog::getDouble(this, tr("Subdivide mesh"), tr("Max area per triangle:"), s_subdivideMaxArea, 1e-6, 1e6, 8, &ok);
	if (!ok)
		return;

	//ccProgressDialog pDlg(true, this);
	//pDlg.setAutoClose(false);
	bool warningIssued = false;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			//single mesh?
			if (entity->isA(CC_TYPES::MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(entity);

				ccMesh* subdividedMesh = nullptr;
				try
				{
					subdividedMesh = mesh->subdivide(static_cast<PointCoordinateType>(s_subdivideMaxArea));
				}
				catch (...)
				{
					ccLog::Error(tr("[Subdivide] An error occurred while trying to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}

				if (subdividedMesh)
				{
					subdividedMesh->setName(QString("%1.subdivided(S<%2)").arg(mesh->getName()).arg(s_subdivideMaxArea));
					subdividedMesh->setDisplay(mesh->getDisplay());
					mesh->redrawDisplay();
					mesh->setEnabled(false);
					addToDB(subdividedMesh);
				}
				else
				{
					ccConsole::Warning(tr("[Subdivide] Failed to subdivide mesh '%1' (not enough memory?)").arg(mesh->getName()));
				}
			}
			else if (!warningIssued)
			{
				ccLog::Warning(tr("[Subdivide] Works only on real meshes!"));
				warningIssued = true;
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionFlipMeshTriangles()
{
	bool warningIssued = false;
	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			//single mesh?
			if (entity->isA(CC_TYPES::MESH))
			{
				ccMesh* mesh = static_cast<ccMesh*>(entity);
				mesh->flipTriangles();
				mesh->prepareDisplayForRefresh();
			}
			else if (!warningIssued)
			{
				ccLog::Warning(tr("[Flip triangles] Works only on real meshes!"));
				warningIssued = true;
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionSmoothMeshLaplacian()
{
    InformationPrompt set(QApplication::translate("importantinformationtips", "The higher the smoothness level, the smoother the grid.", nullptr));
	static unsigned	s_laplacianSmooth_nbIter = 20;
	static double	s_laplacianSmooth_factor = 0.2;
	ccSmoothMeshLaplacianInputDlg dlg(this);
	dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Mesh Smoothing", nullptr));
	if (dlg.exec() != QDialog::Accepted)
	{
		return;
	}
    s_laplacianSmooth_nbIter = 20;
	s_laplacianSmooth_factor = dlg.GetSmoothingcoefficient();


	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isA(CC_TYPES::MESH) || entity->isA(CC_TYPES::PRIMITIVE)) //FIXME: can we really do this with primitives?
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(entity);

			if (mesh->laplacianSmooth(s_laplacianSmooth_nbIter,
				static_cast<PointCoordinateType>(s_laplacianSmooth_factor),
				&pDlg))
			{
				mesh->prepareDisplayForRefresh_recursive();
			}
			else
			{
				ccConsole::Warning(tr("Failed to apply Laplacian smoothing to mesh '%1'").arg(mesh->getName()));
			}
		}
	}

	refreshAll();
    //[!]队列刷新
    emit signalQueueRefreshPropertiesView();
}

// helper for doActionMerge
void AddToRemoveList(ccHObject* toRemove, ccHObject::Container& toBeRemovedList)
{
	// is a parent or sibling already in the "toBeRemoved" list?
	size_t count = toBeRemovedList.size();
	for (size_t j = 0; j < count;)
	{
		if (toBeRemovedList[j]->isAncestorOf(toRemove))
		{
			// nothing to do, we already have an ancestor
			return;
		}
		else if (toRemove->isAncestorOf(toBeRemovedList[j]))
		{
			// we don't need to keep the children
			toBeRemovedList[j] = toBeRemovedList.back();
			toBeRemovedList.pop_back();
			count--;
		}
		else
		{
			// forward
			++j;
		}
	}

	toBeRemovedList.push_back(toRemove);
}

void MainWindow::doActionMerge()
{
    //let's look for clouds or meshes (warning: we don't mix them)
    std::vector<ccPointCloud*> clouds;
    std::vector<ccMesh*> meshes;

    try
    {
        for (ccHObject *entity : getSelectedEntities())
        {
            if (!entity)
                continue;

            if (entity->isA(CC_TYPES::POINT_CLOUD))
            {
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
                clouds.push_back(cloud);

                // check whether this cloud is an ancestor of the first cloud in the selection
                if (clouds.size() > 1)
                {
                    if (clouds.back()->isAncestorOf(clouds.front()))
                    {
                        // this way we are sure that the first cloud is not below any other cloud
                        std::swap(clouds.front(), clouds.back());
                    }
                }
            }
            else if (entity->isKindOf(CC_TYPES::MESH))
            {
                ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
                //this is a purely theoretical test for now!
                if (mesh && mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
                {
                    meshes.push_back(mesh);
                }
                else
                {
                    qDebug() << tr("Only meshes with standard vertices are handled for now! Can't merge entity '%1'...").arg(entity->getName());
                }
            }
            else
            {
                qDebug() << tr("Entity '%1' is neither a cloud nor a mesh, can't merge it!").arg(entity->getName());
            }
        }
    }
    catch (const std::bad_alloc&)
    {
        qDebug() << tr("Not enough memory!");
        return;
    }

    if (clouds.empty() && meshes.empty())
    {
        qDebug() << tr("Select only clouds or meshes!");
        return;
    }
    if (!clouds.empty() && !meshes.empty())
    {
        qDebug() << tr("Can't mix point clouds and meshes!");
    }

    //[!]1.先弹框提示
    if (!clouds.empty()) {
        ccMergeQuestionDlg dlg(this);
        dlg.SetCloudNum(clouds.size());
        dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Point cloud merging", nullptr));
        if (!dlg.exec())
            return;
    }
    else if (!meshes.empty()) {
        ccMergeQuestionDlg dlg(this);
        dlg.SetCloudNum(meshes.size());
        dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Point cloud merging", nullptr));
        if (!dlg.exec())
            return;

    }
    else {//[!].没有任何数据退出
        return;
    }

    //[!]1.线程函数
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance(this)->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));

    //merge clouds?
    if (!clouds.empty())
    {
        //we deselect all selected entities (as most of them are going to disappear)
        if (m_ccRoot)
        {
            m_ccRoot->unselectAllEntities();
            //assert(!haveSelection());
            //m_selectedEntities.clear();
        }

        //we will remove the useless clouds/meshes later
        ccHObject::Container toBeRemoved;

        ccPointCloud* firstCloud = nullptr;
        ccHObjectContext firstCloudContext;

        //whether to generate the 'original cloud index' scalar field or not
        CCCoreLib::ScalarField* ocIndexSF = nullptr;
        size_t cloudIndex = 0;
        qDebug() << "doActionMerge start - 2";

        //[!].第一份点云数据
        firstCloud = clouds.at(0);
        firstCloudContext = removeObjectTemporarilyFromDBTree(firstCloud);

        QFuture<void> fu = QtConcurrent::run([&]() {
            int sfIdx = firstCloud->getScalarFieldIndexByName(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
            if (sfIdx < 0)
            {
                sfIdx = firstCloud->addScalarField(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
            }
            if (sfIdx < 0)
            {
                qDebug() << tr("Couldn't allocate a new scalar field for storing the original cloud index! Try to free some memory ...");
                return;
            }
            else
            {
                ocIndexSF = firstCloud->getScalarField(sfIdx);
                ocIndexSF->fill(0);
                firstCloud->setCurrentDisplayedScalarField(sfIdx);
            }


            for (size_t i = 1; i < clouds.size(); ++i)
            {
                ccPointCloud* pc = clouds[i];
                if (!pc) {
                    continue;
                }
                unsigned countBefore = firstCloud->size();
                unsigned countAdded = pc->size();
                *firstCloud += pc;

                //success?
                if (firstCloud->size() == countBefore + countAdded)
                {
                    firstCloud->prepareDisplayForRefresh_recursive();

                    ccHObject* toRemove = nullptr;
                    //if the entity to remove is inside a group with a unique child, we can remove the group as well
                    ccHObject* parent = pc->getParent();
                    if (parent && parent->isA(CC_TYPES::HIERARCHY_OBJECT) && parent->getChildrenNumber() == 1 && parent != firstCloudContext.parent)
                        toRemove = parent;
                    else
                        toRemove = pc;

                    AddToRemoveList(toRemove, toBeRemoved);
                    QCoreApplication::processEvents();
                    QThread::msleep(20);


                    if (ocIndexSF)
                    {
                        ScalarType index = static_cast<ScalarType>(++cloudIndex);
                        for (unsigned i = 0; i < countAdded; ++i)
                        {
                            ocIndexSF->setValue(countBefore + i, index);
                        }
                    }
                }
                else
                {
                    qDebug() << tr("Fusion failed! (not enough memory?)");
                    break;
                }
                pc = nullptr;
            }

            qDebug() << "doActionMerge start - 3";

            if (ocIndexSF)
            {
                ocIndexSF->computeMinAndMax();
                firstCloud->showSF(true);
            }
            qDebug() << "doActionMerge start - 4";

            //something to remove?
            for (ccHObject* toRemove : toBeRemoved)
            {
                if (firstCloud->isAncestorOf(toRemove))
                {
                    // we cannot call 'removeElement' on a child of the first cloud, as it's temporarily detached from the DB tree!
                    if (toRemove->getParent())
                        toRemove->getParent()->removeChild(toRemove);
                    else
                        delete toRemove;
                }
                else
                {
                    m_ccRoot->removeElement(toRemove);
                }
            }
            toBeRemoved.clear();
            qDebug() << "doActionMerge start - 5";
        });

        while (!fu.isFinished()) {
            QThread::msleep(20);
            WaitingDialog::MetahublFramelessWaitingdialog::instance(this)->jumpToNextFrame();

        }

        //eventually we can put back the first cloud in DB
        if (firstCloud)
        {
            firstCloud->setCurrentCombinationMode(OTHER);
            putObjectBackIntoDBTree(firstCloud, firstCloudContext);
            if (m_ccRoot)
                m_ccRoot->selectEntity(firstCloud);
            CS::Model::ProjectModel::instance()->clearSelectEntitys();
            CS::Model::ProjectModel::instance()->appendSelectEntity(firstCloud);
            firstCloud->deleteVertexArrays();
            setSelfAndParentObjectEnabled(firstCloud,true);
        }
        qDebug() << "doActionMerge start - 6";

    }
    //merge meshes?
    else if (!meshes.empty())
    {
        bool createSubMeshes = true;
        //createSubMeshes = (CS::Widgets::FramelessMessageBox::question(this, tr("Create sub-meshes"), tr("Do you want to create sub-mesh entities corresponding to each source mesh? (requires more memory)"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);

        //meshes are merged
        ccPointCloud* baseVertices = new ccPointCloud("vertices");
        ccMesh* baseMesh = new ccMesh(baseVertices);
        baseMesh->setName("Merged mesh");
        baseMesh->addChild(baseVertices);
        baseVertices->setEnabled(false);

        QFuture<void> fu = QtConcurrent::run([&]() {
            for (ccMesh *mesh : meshes)
            {
                //if (mesh->isA(CC_TYPES::PRIMITIVE))
                //{
                //	mesh = mesh->ccMesh::cloneMesh(); //we want a clone of the mesh part, not the primitive!
                //}

                if (!baseMesh->merge(mesh, createSubMeshes))
                {
                    qDebug() << tr("Fusion failed! (not enough memory?)");
                    break;
                }
            }

        });

        while (!fu.isFinished()) {
            QThread::msleep(20);
            QCoreApplication::processEvents();

        }

        baseMesh->setDisplay_recursive(meshes.front()->getDisplay());
        baseMesh->setVisible(true);
        addToDB(baseMesh);
        if (m_ccRoot) {
            m_ccRoot->selectEntity(baseMesh);
            CS::Model::ProjectModel::instance()->clearSelectEntitys();
            CS::Model::ProjectModel::instance()->appendSelectEntity(baseMesh);
            setSelfAndParentObjectEnabled(baseMesh, true);
        }
    }

    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    refreshAll();
    setGlobalZoom();
    updateUI();
    return;
}

void MainWindow::zoomOn(ccHObject* object)
{
	ccGLWindow* win = static_cast<ccGLWindow*>(object->getDisplay());
	if (win)
	{
		ccBBox box = object->getDisplayBB_recursive(false, win);
		win->updateConstellationCenterAndZoom(&box);
	}
}

void MainWindow::doActionRegister()
{
	if (m_selectedEntities.size() != 2
		|| (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities.front()->isKindOf(CC_TYPES::MESH))
		|| (!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD) && !m_selectedEntities[1]->isKindOf(CC_TYPES::MESH)))
	{
		ccConsole::Error(tr("Select 2 point clouds or meshes!"));
		return;
	}

	ccHObject* data = static_cast<ccHObject*>(m_selectedEntities.front());
	ccHObject* model = static_cast<ccHObject*>(m_selectedEntities[1]);
	if (data->isKindOf(CC_TYPES::MESH) && model->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		//by default, prefer the mesh as the reference
		std::swap(data, model);
	}

	ccRegistrationDlg rDlg(data, model, this);
	if (!rDlg.exec())
		return;

	//model and data order may have changed!
	model = rDlg.getModelEntity();
	data = rDlg.getDataEntity();

	double minRMSDecrease = rDlg.getMinRMSDecrease();
	if (std::isnan(minRMSDecrease))
	{
		ccLog::Error(tr("Invalid minimum RMS decrease value"));
		return;
	}
	if (minRMSDecrease < ccRegistrationDlg::GetAbsoluteMinRMSDecrease())
	{
		minRMSDecrease = ccRegistrationDlg::GetAbsoluteMinRMSDecrease();
		ccLog::Error(tr("Minimum RMS decrease value is too small.\n%1 will be used instead (numerical accuracy limit).").arg(minRMSDecrease, 0, 'E', 1));
		rDlg.setMinRMSDecrease(minRMSDecrease);
	}

	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	{
		parameters.convType = rDlg.getConvergenceMethod();
		parameters.minRMSDecrease = minRMSDecrease;
		parameters.nbMaxIterations = rDlg.getMaxIterationCount();
		parameters.adjustScale = rDlg.adjustScale();
		parameters.filterOutFarthestPoints = rDlg.removeFarthestPoints();
		parameters.samplingLimit = rDlg.randomSamplingLimit();
		parameters.finalOverlapRatio = rDlg.getFinalOverlap() / 100.0;
		parameters.transformationFilters = rDlg.getTransformationFilters();
		parameters.maxThreadCount = rDlg.getMaxThreadCount();
		parameters.useC2MSignedDistances = rDlg.useC2MSignedDistances();
		parameters.normalsMatching = rDlg.normalsMatchingOption();
	}
	bool useDataSFAsWeights = rDlg.useDataSFAsWeights();
	bool useModelSFAsWeights = rDlg.useModelSFAsWeights();

	//semi-persistent storage (for next call)
	rDlg.saveParameters();

	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	unsigned finalPointCount = 0;

	if (ccRegistrationTools::ICP(data,
		model,
		transMat,
		finalScale,
		finalError,
		finalPointCount,
		parameters,
		useDataSFAsWeights,
		useModelSFAsWeights,
		this))
	{
		QString rmsString = tr("Final RMS*: %1 (computed on %2 points)").arg(finalError).arg(finalPointCount);
		QString rmsDisclaimerString = tr("(* RMS is potentially weighted, depending on the selected options)");
		ccLog::Print(QString("[Register] ") + rmsString);
		ccLog::Print(QString("[Register] ") + rmsDisclaimerString);

		QStringList summary;
		summary << rmsString;
		summary << rmsDisclaimerString;
		summary << "----------------";

		//transformation matrix
		{
			summary << "Transformation matrix";
			summary << transMat.toString(3, '\t'); //low precision, just for display
			summary << "----------------";

			ccLog::Print(tr("[Register] Applied transformation matrix:"));
			ccLog::Print(transMat.toString(12, ' ')); //full precision
			ccLog::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));
		}

		if (parameters.adjustScale)
		{
			QString scaleString = tr("Scale: %1 (already integrated in above matrix!)").arg(finalScale);
			ccLog::Warning(QString("[Register] ") + scaleString);
			summary << scaleString;
		}
		else
		{
			ccLog::Print(tr("[Register] Scale: fixed (1.0)"));
			summary << tr("Scale: fixed (1.0)");
		}

		//overlap
		summary << "----------------";
		QString overlapString = tr("Theoretical overlap: %1%").arg(static_cast<int>(parameters.finalOverlapRatio * 100));
		ccLog::Print(QString("[Register] ") + overlapString);
		summary << overlapString;

		summary << "----------------";
		summary << tr("This report has been output to Console (F8)");

		//cloud to move
		ccGenericPointCloud* pc = nullptr;

		if (data->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			pc = ccHObjectCaster::ToGenericPointCloud(data);
		}
		else if (data->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(data);
			pc = mesh->getAssociatedCloud();

			//warning: point cloud is locked!
			if (pc->isLocked())
			{
				pc = nullptr;
				//we ask the user about cloning the 'data' mesh
				QMessageBox::StandardButton result = CS::Widgets::FramelessMessageBox::question(this,
					tr("Registration"),
					tr("Data mesh vertices are locked (they may be shared with other meshes): Do you wish to clone this mesh to apply transformation?"),
					QMessageBox::Ok | QMessageBox::Cancel,
					QMessageBox::Ok);

				//continue process?
				if (result == QMessageBox::Ok)
				{
					ccGenericMesh* newMesh = nullptr;
					if (mesh->isA(CC_TYPES::MESH))
						newMesh = static_cast<ccMesh*>(mesh)->cloneMesh();
					else
					{
						//FIXME TODO
						ccLog::Error(tr("Doesn't work on sub-meshes yet!"));
					}

					if (newMesh)
					{
						newMesh->setDisplay(data->getDisplay());
						addToDB(newMesh);
						data = newMesh;
						pc = newMesh->getAssociatedCloud();
					}
					else
					{
						ccLog::Error(tr("Failed to clone 'data' mesh! (not enough memory?)"));
					}
				}
			}
		}

		//if we managed to get a point cloud to move!
		if (pc)
		{
			//we temporarily detach cloud, as it may undergo
			//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::applyRigidTransformation
			ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(pc);
			pc->applyRigidTransformation(transMat);
			putObjectBackIntoDBTree(pc, objContext);

			//don't forget to update mesh bounding box also!
			if (data->isKindOf(CC_TYPES::MESH))
				ccHObjectCaster::ToGenericMesh(data)->refreshBB();

			//don't forget global shift
			ccGenericPointCloud* refPc = ccHObjectCaster::ToGenericPointCloud(model);
			if (refPc)
			{
				if (refPc->isShifted())
				{
					const CCVector3d& Pshift = refPc->getGlobalShift();
					const double& scale = refPc->getGlobalScale();
					pc->setGlobalShift(Pshift);
					pc->setGlobalScale(scale);
					ccLog::Warning(tr("[ICP] Aligned entity global shift has been updated to match the reference: (%1,%2,%3) [x%4]").arg(Pshift.x).arg(Pshift.y).arg(Pshift.z).arg(scale));
				}
				else if (pc->isShifted()) //we'll ask the user first before dropping the shift information on the aligned cloud
				{
					if (CS::Widgets::FramelessMessageBox::question(this, tr("Drop shift information?"), tr("Aligned entity is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
					{
						pc->setGlobalShift(0, 0, 0);
						pc->setGlobalScale(1.0);
						ccLog::Warning(tr("[ICP] Aligned entity global shift has been reset to match the reference!"));
					}
				}
			}

			data->prepareDisplayForRefresh_recursive();
			data->setName(data->getName() + QString(".registered"));
			zoomOn(data);
		}

		//pop-up summary
		CS::Widgets::FramelessMessageBox::information(this, tr("Registration info"), summary.join("\n"));
		forceConsoleDisplay();
	}

	refreshAll();
	updateUI();
}

//Aurelien BEY le 13/11/2008 : ajout de la fonction permettant de traiter la fonctionnalite de recalage grossier
void MainWindow::doAction4pcsRegister()
{
	if (CS::Widgets::FramelessMessageBox::warning(this,
		tr("Work in progress"),
		tr("This method is still under development: are you sure you want to use it? (a crash may likely happen)"),
		QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
		return;

	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccGenericPointCloud* model = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities.front());
	ccGenericPointCloud* data = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[1]);

	ccAlignDlg aDlg(model, data);
	if (!aDlg.exec())
		return;

	// model = aDlg.getModelObject();
	data = aDlg.getDataObject();

	//Take the correct number of points among the clouds
	CCCoreLib::ReferenceCloud *subModel = aDlg.getSampledModel();
	CCCoreLib::ReferenceCloud *subData = aDlg.getSampledData();

	unsigned nbMaxCandidates = aDlg.isNumberOfCandidatesLimited() ? aDlg.getMaxNumberOfCandidates() : 0;

	ccProgressDialog pDlg(true, this);

	CCCoreLib::PointProjectionTools::Transformation transform;
	if (CCCoreLib::FPCSRegistrationTools::RegisterClouds(subModel,
		subData,
		transform,
		static_cast<ScalarType>(aDlg.getDelta()),
		static_cast<ScalarType>(aDlg.getDelta() / 2),
		static_cast<PointCoordinateType>(aDlg.getOverlap()),
		aDlg.getNbTries(),
		5000,
		&pDlg,
		nbMaxCandidates))
	{
		//output resulting transformation matrix
		{
			ccGLMatrix transMat = FromCCLibMatrix<double, float>(transform.R, transform.T);
			forceConsoleDisplay();
			ccConsole::Print(tr("[Align] Resulting matrix:"));
			ccConsole::Print(transMat.toString(12, ' ')); //full precision
			ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));
		}

		ccPointCloud *newDataCloud = data->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(data)->cloneThis() : ccPointCloud::From(data, data);

		if (data->getParent())
			data->getParent()->addChild(newDataCloud);
		newDataCloud->setName(data->getName() + QString(".registered"));
		transform.apply(*newDataCloud);
		newDataCloud->invalidateBoundingBox(); //invalidate bb
		newDataCloud->setDisplay(data->getDisplay());
		newDataCloud->prepareDisplayForRefresh();
		zoomOn(newDataCloud);
		addToDB(newDataCloud);

		data->setEnabled(false);
		data->prepareDisplayForRefresh_recursive();
	}
	else
	{
		ccConsole::Warning(tr("[Align] Registration failed!"));
	}

	if (subModel)
		delete subModel;
	if (subData)
		delete subData;

	refreshAll();
	updateUI();
}

void MainWindow::doActionSubsample()
{
	//find candidates
	std::vector<ccPointCloud*> clouds;
	unsigned maxPointCount = 0;
	double maxCloudRadius = 0;
	ScalarType sfMin = CCCoreLib::NAN_VALUE;
	ScalarType sfMax = CCCoreLib::NAN_VALUE;
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
				clouds.push_back(cloud);

				maxPointCount = std::max<unsigned>(maxPointCount, cloud->size());
				maxCloudRadius = std::max<double>(maxCloudRadius, cloud->getOwnBB().getDiagNorm());

				//we also look for the min and max sf values
				ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
				if (sf)
				{
					if (!ccScalarField::ValidValue(sfMin) || sfMin > sf->getMin())
						sfMin = sf->getMin();
					if (!ccScalarField::ValidValue(sfMax) || sfMax < sf->getMax())
						sfMax = sf->getMax();
				}
			}
		}
	}

	if (clouds.empty())
	{
		ccConsole::Error(tr("Select at least one point cloud!"));
		return;
	}

	//Display dialog



    updateStatusbarInformation(QApplication::translate("importantinformationtips", "The larger the minimum space,the fewer the number of points.", nullptr));
	ccSubsamplingDlg sDlg(maxPointCount, maxCloudRadius, this);
    connect(&sDlg, &ccSubsamplingDlg::signalSendStatusBarInfo, this, &MainWindow::updateStatusbarInformation, Qt::UniqueConnection);
	bool hasValidSF = ccScalarField::ValidValue(sfMin) && ccScalarField::ValidValue(sfMax);
	if (hasValidSF)
		sDlg.enableSFModulation(sfMin, sfMax);
	//framesDlg.bottomLayout()->setVisiable(false);;
	//framesDlg.exec();
    bool outInfo = sDlg.exec();
    updateStatusbarInformation("EXIT");

	if (!outInfo)
		return;

	//process clouds
	ccHObject::Container resultingClouds;
	{
		ccProgressDialog pDlg(false, this);
        pDlg.setInfo(tr("Processing and analyzing point cloud data, please wait..."));
        pDlg.setMethodTitle(tr("Processing"));
		pDlg.setAutoReset(false);
		pDlg.setValue(1);
        pDlg.start();
        int i = 1;
        QFuture<void> fu = QtConcurrent::run([&]() {
            bool errors = false;
            for (size_t i = 0; i < clouds.size(); ++i)
            {
                ccPointCloud* cloud = clouds[i];
                CCCoreLib::ReferenceCloud *sampledCloud = sDlg.getSampledCloud(cloud, nullptr);
                if (!sampledCloud)
                {
                    ccConsole::Warning(tr("[Subsampling] Failed to subsample cloud '%1'!").arg(cloud->getName()));
                    errors = true;
                    continue;
                }

                int warnings = 0;
                ccPointCloud *newPointCloud = cloud->partialClone(sampledCloud, &warnings);

                delete sampledCloud;
                sampledCloud = nullptr;

                if (newPointCloud)
                {
                    newPointCloud->setName(cloud->getName() + QString(".subsampled"));
                    newPointCloud->copyGlobalShiftAndScale(*cloud);
                    newPointCloud->setDisplay(cloud->getDisplay());

                    resultingClouds.push_back(newPointCloud);

                    if (warnings)
                    {
                        ccLog::Warning(tr("[Subsampling] Not enough memory: colors, normals or scalar fields may be missing!"));
                        errors = true;
                    }
                }
                else
                {
                    ccLog::Error(tr("Not enough memory!"));
                    break;
                }
            }

            if (errors)
            {
                ccLog::Error(tr("Errors occurred (see console)"));
            }
        }); 
        
        while (!fu.isFinished()) {
            QThread::msleep(500);
            if (i < 98){
                pDlg.setValue(++i);
            }
            QCoreApplication::processEvents();
        }
        pDlg.update(100);
		QThread::msleep(500);
		pDlg.stop();
	}

    if (m_ccRoot) {
        if (clouds.size() != resultingClouds.size())
        {
            qDebug() << "error.   clouds.size():" << clouds.size() << "resultingClouds.size():" << resultingClouds.size();
            return;
        }
        for (int i = 0; i < clouds.size(); ++i)
        {
            if (clouds[i]->getParent())
            {
                clouds[i]->getParent()->addChild(resultingClouds[i]);
                clouds[i]->setEnabled(false);
            }
            addToDB(resultingClouds[i]);
            resultingClouds[i]->prepareDisplayForRefresh();
            QCoreApplication::processEvents();
        }

        m_ccRoot->selectEntities(resultingClouds);
    }

    ccGui::ParamStruct parameters = ccGui::Parameters();
    if (parameters.m_is2DView) {
        setViewButtonDisabled(true);
    }
    else
    {
        setViewButtonDisabled(false);
    }

	refreshAll();
	updateUI();
    MainWindow::TheInstance()->setGlobalZoom();
}

void MainWindow::doActionStatisticalTest()
{
	if (!ccEntityAction::statisticalTest(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeStatParams()
{
	ccEntityAction::computeStatParams(m_selectedEntities, this);
}

struct ComponentIndexAndSize
{
	unsigned index;
	unsigned size;

	ComponentIndexAndSize(unsigned i, unsigned s) : index(i), size(s) {}

	static bool DescendingCompOperator(const ComponentIndexAndSize& a, const ComponentIndexAndSize& b)
	{
		return a.size > b.size;
	}
};

void MainWindow::createComponentsClouds(ccGenericPointCloud* cloud,
	CCCoreLib::ReferenceCloudContainer& components,
	unsigned minPointsPerComponent,
	bool randomColors,
	bool selectComponents,
	bool sortBysize/*=true*/)
{
    if (!cloud || components.empty())
        return;


    //[!]1.线程函数
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));
    QObject end;
    connect(&end, &QObject::destroyed, this, [=]() {
        WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    });

    //[!].排序

   
       


    //std::vector<ComponentIndexAndSize> sortedIndexes;
    //std::vector<ComponentIndexAndSize>* _sortedIndexes = nullptr;
    if (sortBysize)
    {
        //[!].根据大小进行排序 从大到小
        std::sort(components.begin(), components.end(), [](CCCoreLib::ReferenceCloud* ptr1,
            CCCoreLib::ReferenceCloud* ptr2) {
            return ptr1->size() > ptr2->size();
        });
       
    }

    //we create "real" point clouds for all input components
    {
        ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : nullptr;
        //we create a new group to store all CCs
        ccHObject* ccGroup = new ccHObject(cloud->getName() + QString(" [Clus]"));
        QMutex sMutex;
        auto function = [&](CCCoreLib::ReferenceCloud* pComponent) {
            //if it has enough points
            if (pComponent->size() >= minPointsPerComponent)
            {
                //we create a new entity
                ccPointCloud* compCloud = (pc ? pc->partialClone(pComponent) : ccPointCloud::From(pComponent));
                if (compCloud)
                {
                    //shall we colorize it with random color?
                    if (randomColors)
                    {
                        ccColor::Rgb col = ccColor::Generator::Random();
                        compCloud->setColor(col);
                        compCloud->showColors(true);
                        compCloud->showSF(false);
                    }

                    //'shift on load' information
                    if (pc)
                    {
                        compCloud->copyGlobalShiftAndScale(*pc);
                    }

                    {
                        QMutexLocker lock(&sMutex);
                        compCloud->setVisible(true);
                        compCloud->setName(QString("Clu_%1").arg(ccGroup->getChildrenNumber()));
                        compCloud->setSelected(true);
                        ccGroup->addChild(compCloud);
                    }

                }
                else
                {
                    ccConsole::Warning(tr("[CreateComponentsClouds] Failed to create component #%1! (not enough memory)").arg(ccGroup->getChildrenNumber() + 1));
                }
            }
            delete pComponent;
            pComponent = nullptr;
        };


        int maxThreadCount = QThread::idealThreadCount();
        QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
        QFuture<void> fu = QtConcurrent::map(components, function);
        while (!fu.isFinished()) {
            QThread::msleep(20);
            QCoreApplication::processEvents();

        }

        components.clear();
        if (ccGroup->getChildrenNumber() == 0)
        {
            CS::Widgets::FramelessMessageBox::critical(s_instance, tr("Tips"), tr("Cannot generate clusters, please confirm the parameters and try again."));
            delete ccGroup;
            ccGroup = nullptr;
        }
        else
        {
            ccGroup->setDisplay(cloud->getDisplay());
            ccHObject* parent = cloud->getParent();
            parent->addChild(ccGroup);
            addToDB(ccGroup);

            ccConsole::Print(tr("[CreateComponentsClouds] %1 component(s) were created from cloud '%2'").arg(ccGroup->getChildrenNumber()).arg(cloud->getName()));
        }
        cloud->prepareDisplayForRefresh();
        if (ccGroup)
        {
            cloud->setEnabled(false);
            ccConsole::Warning(tr("[CreateComponentsClouds] Original cloud has been automatically hidden"));
        }
    }
}

void MainWindow::doActionLabelConnectedComponents()
{
	//keep only the point clouds!
	std::vector<ccGenericPointCloud*> clouds;
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				clouds.push_back(ccHObjectCaster::ToGenericPointCloud(entity));
		}
	}

	size_t count = clouds.size();
	if (count == 0)
		return;
    InformationPrompt set(QApplication::translate("importantinformationtips", "The smaller Octree level,the more obvious the clustering.The fewer the number of points,the more point cloud clusters.", nullptr));
	ccLabelingDlg dlg(this);

	//It is meaningless if the clustering threshold is too large or too small
	dlg.setMaxPointsNb(9999);
	dlg.setMinPointsNb(10);

	dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Cluster", nullptr));
	if (count == 1)
		dlg.octreeLevelSpinBox->setCloud(clouds.front());
	if (!dlg.exec())
		return;

	int octreeLevel = dlg.getOctreeLevel();
	unsigned minComponentSize = static_cast<unsigned>(std::max(0, dlg.getMinPointsNb()));
	bool randColors = dlg.randomColors();

	ccProgressDialog pDlg(false, this);
	pDlg.setAutoClose(false);

	//we unselect all entities as we are going to automatically select the created components
	//(otherwise the user won't perceive the change!)
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	for (ccGenericPointCloud *cloud : clouds)
	{
		if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

			ccOctree::Shared theOctree = cloud->getOctree();
			if (!theOctree)
			{
				ccProgressDialog pOctreeDlg(true, this);
				theOctree = cloud->computeOctree(&pOctreeDlg);
				if (!theOctree)
				{
					ccConsole::Error(tr("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
					break;
				}
			}

			//we create/activate CCs label scalar field
			int sfIdx = pc->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			if (sfIdx < 0)
			{
				sfIdx = pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			}
			if (sfIdx < 0)
			{
				ccConsole::Error(tr("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ..."));
				break;
			}
			pc->setCurrentScalarField(sfIdx);

			//we try to label all CCs
			CCCoreLib::ReferenceCloudContainer components;
			int componentCount = CCCoreLib::AutoSegmentationTools::labelConnectedComponents(cloud,
				static_cast<unsigned char>(octreeLevel),
				false,
				&pDlg,
				theOctree.data());

			if (componentCount >= 0)
			{
				//if successful, we extract each CC (stored in "components")

				//safety test
				int realComponentCount = 0;
				{
					for (size_t i = 0; i < components.size(); ++i)
					{
						if (components[i]->size() >= minComponentSize)
						{
							++realComponentCount;
						}
					}
				}

				if (realComponentCount > 500)
				{
					//too many components
					if (CS::Widgets::FramelessMessageBox::warning(this, tr("Many components"), tr("Do you really expect up to %1 components?\n(this may take a lot of time to process and display)").arg(realComponentCount), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
					{
						//cancel
						pc->deleteScalarField(sfIdx);
						if (pc->getNumberOfScalarFields() != 0)
						{
							pc->setCurrentDisplayedScalarField(static_cast<int>(pc->getNumberOfScalarFields()) - 1);
						}
						else
						{
							pc->showSF(false);
						}
						pc->prepareDisplayForRefresh();
						continue;
					}
				}

				pc->getCurrentInScalarField()->computeMinAndMax();
				if (!CCCoreLib::AutoSegmentationTools::extractConnectedComponents(cloud, components))
				{
					ccConsole::Warning(tr("[DoActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
				}
			}
			else
			{
				ccConsole::Warning(tr("[DoActionLabelConnectedComponents] Something went wrong while extracting CCs from cloud %1...").arg(cloud->getName()));
			}

			//we delete the CCs label scalar field (we don't need it anymore)
			pc->deleteScalarField(sfIdx);
			sfIdx = -1;

			//we create "real" point clouds for all CCs
			if (!components.empty())
			{
				createComponentsClouds(cloud, components, minComponentSize, randColors, true);
			}
		}
	}
    MainWindow::TheInstance()->setGlobalZoom();
	refreshAll();
	updateUI();
}

void MainWindow::doActionSetSFAsCoord()
{
	if (!ccEntityAction::sfSetAsCoord(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionExportCoordToSF()
{
	if (!ccEntityAction::exportCoordToSF(m_selectedEntities, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionExportNormalToSF()
{
	if (!ccEntityAction::exportNormalToSF(m_selectedEntities, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::doMeshTwoPolylines()
{
	if (m_selectedEntities.size() != 2)
		return;

	ccPolyline* p1 = ccHObjectCaster::ToPolyline(m_selectedEntities.front());
	ccPolyline* p2 = ccHObjectCaster::ToPolyline(m_selectedEntities[1]);
	if (!p1 || !p2)
	{
		ccConsole::Error(tr("Select 2 and only 2 polylines"));
		return;
	}

	//Ask the user how the 2D projection should be computed
	bool useViewingDir = false;
	CCVector3 viewingDir(0, 0, 0);
	if (p1->getDisplay())
	{
		useViewingDir = (CS::Widgets::FramelessMessageBox::question(this, tr("Projection method"), tr("Use best fit plane (yes) or the current viewing direction (no)"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No);
		if (useViewingDir)
		{
			viewingDir = -p1->getDisplay()->getViewportParameters().getViewDir().toPC();
		}
	}

	ccMesh* mesh = ccMesh::TriangulateTwoPolylines(p1, p2, useViewingDir ? &viewingDir : nullptr);
	if (mesh)
	{
		addToDB(mesh);
		if (mesh->computePerVertexNormals())
		{
			mesh->showNormals(true);
		}
		else
		{
			ccLog::Warning(tr("[Mesh two polylines] Failed to compute normals!"));
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}
	}
	else
	{
		ccLog::Error(tr("Failed to create mesh (see Console)"));
		forceConsoleDisplay();
	}
}

void MainWindow::doConvertPolylinesToMesh()
{
	if (!haveSelection())
		return;

	std::vector<ccPolyline*> polylines;
	try
	{
		if (haveOneSelection() && m_selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			ccHObject* obj = m_selectedEntities.back();
			for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
			{
				if (obj->getChild(i)->isA(CC_TYPES::POLY_LINE))
					polylines.push_back(static_cast<ccPolyline*>(obj->getChild(i)));
			}
		}
		else
		{
			for (ccHObject *entity : getSelectedEntities())
			{
				if (entity->isA(CC_TYPES::POLY_LINE))
				{
					polylines.push_back(static_cast<ccPolyline*>(entity));
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	if (polylines.empty())
	{
		ccConsole::Error(tr("Select a group of polylines or multiple polylines (contour plot)!"));
		return;
	}

	ccPickOneElementDlg poeDlg(tr("Projection dimension"), tr("Contour plot to mesh"), this);
	poeDlg.addElement("X");
	poeDlg.addElement("Y");
	poeDlg.addElement("Z");
	poeDlg.setDefaultIndex(2);
	if (!poeDlg.exec())
		return;

	int dim = poeDlg.getSelectedIndex();
	assert(dim >= 0 && dim < 3);

	const unsigned char Z = static_cast<unsigned char>(dim);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	//number of segments
	unsigned segmentCount = 0;
	unsigned vertexCount = 0;
	{
		for (ccPolyline *poly : polylines)
		{
			if (poly)
			{
				//count the total number of vertices and segments
				vertexCount += poly->size();
				segmentCount += poly->segmentCount();
			}
		}
	}

	if (segmentCount < 2)
	{
		//not enough points/segments
		ccLog::Error(tr("Not enough segments!"));
		return;
	}

	//we assume we link with CGAL now (if not the call to Delaunay2dMesh::buildMesh will fail anyway)
	std::vector<CCVector2> points2D;
	std::vector<int> segments2D;
	try
	{
		points2D.reserve(vertexCount);
		segments2D.reserve(segmentCount * 2);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	//fill arrays
	{
		for (ccPolyline *poly : polylines)
		{
			if (poly == nullptr)
				continue;

			unsigned vertCount = poly->size();
			int vertIndex0 = static_cast<int>(points2D.size());
			bool closed = poly->isClosed();
			for (unsigned v = 0; v < vertCount; ++v)
			{
				const CCVector3* P = poly->getPoint(v);
				int vertIndex = static_cast<int>(points2D.size());
				points2D.push_back(CCVector2(P->u[X], P->u[Y]));

				if (v + 1 < vertCount)
				{
					segments2D.push_back(vertIndex);
					segments2D.push_back(vertIndex + 1);
				}
				else if (closed)
				{
					segments2D.push_back(vertIndex);
					segments2D.push_back(vertIndex0);
				}
			}
		}
		assert(points2D.size() == vertexCount);
		assert(segments2D.size() == segmentCount * 2);
	}

	CCCoreLib::Delaunay2dMesh* delaunayMesh = new CCCoreLib::Delaunay2dMesh;
	std::string errorStr;
	if (!delaunayMesh->buildMesh(points2D, segments2D, errorStr))
	{
		ccLog::Error(tr("Third party library error: %1").arg(QString::fromStdString(errorStr)));
		delete delaunayMesh;
		return;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertexCount))
	{
		//not enough memory
		ccLog::Error(tr("Not enough memory!"));
		delete vertices;
		delete delaunayMesh;
		return;
	}

	//fill vertices cloud
	{
		for (ccPolyline *poly : polylines)
		{
			unsigned vertCount = poly->size();
			for (unsigned v = 0; v < vertCount; ++v)
			{
				const CCVector3* P = poly->getPoint(v);
				vertices->addPoint(*P);
			}
		}
		delaunayMesh->linkMeshWith(vertices, false);
	}

#ifdef QT_DEBUG
	//Test delaunay output
	{
		unsigned vertCount = vertices->size();
		for (unsigned i = 0; i < delaunayMesh->size(); ++i)
		{
			const CCCoreLib::VerticesIndexes* tsi = delaunayMesh->getTriangleVertIndexes(i);
			assert(tsi->i1 < vertCount && tsi->i2 < vertCount && tsi->i3 < vertCount);
		}
	}
#endif

	ccMesh* mesh = new ccMesh(delaunayMesh, vertices);
	if (mesh->size() != delaunayMesh->size())
	{
		//not enough memory (error will be issued later)
		delete mesh;
		mesh = nullptr;
	}

	//don't need this anymore
	delete delaunayMesh;
	delaunayMesh = nullptr;

	if (mesh)
	{
		mesh->addChild(vertices);
		mesh->setVisible(true);
		vertices->setEnabled(false);
		addToDB(mesh);
		if (mesh->computePerVertexNormals())
		{
			mesh->showNormals(true);
		}
		else
		{
			ccLog::Warning(tr("[Contour plot to mesh] Failed to compute normals!"));
		}

		if (mesh->getDisplay())
		{
			mesh->getDisplay()->redraw();
		}

		//global shift & scale (we copy it from the first polyline by default)
		mesh->copyGlobalShiftAndScale(*polylines.front());
	}
	else
	{
		ccLog::Error(tr("Not enough memory!"));
		delete vertices;
		vertices = nullptr;
	}
}


void MainWindow::doActionRasterize()
{
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select only one point cloud!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();
	if (!ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select a point cloud!"));
		return;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	ccRasterizeTool rasterizeTool(cloud, this);
	rasterizeTool.exec();
}

void MainWindow::doActionExportTiff()
{
    if (!haveOneSelection())
    {
        ccConsole::Error(tr("Select only one mesh!"));
        return;
    }

    ccHObject* ent = m_selectedEntities.front();
    if (!ent || !ent->isKindOf(CC_TYPES::MESH))
    {
        ccConsole::Error(tr("Select a MESH!"));
        return;
    }
    ccMesh* pMesh = dynamic_cast<ccMesh*>(ent);
    if (!pMesh) {
        return;
    }

    QString strDefaultPath = CS::Model::ProjectModel::instance()->property("defalutpath").toString();
    QDir defaultPathDir(strDefaultPath);
    if (strDefaultPath.isEmpty() || !defaultPathDir.exists()) {
        strDefaultPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    }

    //[!].选择文件保存的路径

    QString selectFilter;
    QString sFileName = CS::Widgets::FramelessFileDialog::getSaveFileName(this, tr("Save File"), strDefaultPath + "//image" + ".tif", "*.tif;;*.tiff" , &selectFilter);
    if (sFileName.isEmpty()) {
        return;
    }

    QFileInfo file(sFileName);
    strDefaultPath = file.absoluteDir().path();
    CS::Model::ProjectModel::instance()->setProperty("defalutpath", strDefaultPath);


    //[!].获取选择的文件后缀类型
    QFileInfo fi;
    fi = QFileInfo(sFileName);
    sFileName = fi.path() + QString("/") + fi.fileName();

    QImage image = pMesh->createMeshImage();
    image.save(sFileName);
    return;
}


void MainWindow::doActionDeleteScanGrids()
{
	//look for clouds with scan grids
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		assert(cloud);

		if (cloud->gridCount() > 0)
		{
			cloud->removeGrids();
		}
	}
	refreshAll();
	updateUI();
}


void MainWindow::doActionMeshScanGrids()
{
	//ask the user for the min angle (inside triangles)
	static double s_meshMinTriangleAngle_deg = 1.0;
	{
		bool ok = true;
		double minAngle_deg = QInputDialog::getDouble(this, tr("Triangulate"), tr("Min triangle angle (in degrees)"), s_meshMinTriangleAngle_deg, 0, 90.0, 3, &ok);
		if (!ok)
			return;
		s_meshMinTriangleAngle_deg = minAngle_deg;
	}

	//look for clouds with scan grids
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		assert(cloud);

		for (size_t i = 0; i < cloud->gridCount(); ++i)
		{
			ccPointCloud::Grid::Shared grid = cloud->grid(i);
			if (!grid)
			{
				assert(false);
				continue;
			}

			ccMesh* gridMesh = cloud->triangulateGrid(*grid, s_meshMinTriangleAngle_deg);
			if (gridMesh)
			{
				cloud->addChild(gridMesh);
				gridMesh->setDisplay(cloud->getDisplay());
				addToDB(gridMesh, false, true, false, false);
				gridMesh->prepareDisplayForRefresh();
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionComputeMeshAA()
{
	doActionComputeMesh(CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED);
}

void MainWindow::doActionComputeMeshLS()
{
    //InformationPrompt set(QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr));
	doActionComputeMesh(CCCoreLib::DELAUNAY_2D_BEST_LS_PLANE);
}

void MainWindow::doActionComputeMesh(CCCoreLib::TRIANGULATION_TYPES type)
{
    deselectAllOtherObject(false);
    refreshAll();
    updateUI();
	//ask the user for the max edge length
	static double s_meshMaxEdgeLength = 0.0;
    surfaceMeshPara surfacemeshpara;
    QString strPrompt;
    int meshType = -1;
    ccTriangulateInputDlg newdlg(this);
    newdlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Triangular Mesh", nullptr));
    if (newdlg.exec() != QDialog::Accepted)
    {
        return;
    }
    CCCoreLib::TRIANGULATION_TYPES selectedtype = newdlg.getTriangulateType();
	if (CCCoreLib::SURFACE_MESH != selectedtype)
	{
		double maxEdgeLength = newdlg.GetInputNum();

		s_meshMaxEdgeLength = maxEdgeLength;
        strPrompt = QCoreApplication::translate("MainWindow", "Creating a mesh...", nullptr);
        meshType = 0;
	}
    else
    {
        surfacemeshpara.methodType = 0;
        surfacemeshpara.subFactor = newdlg.GetInputNum();
        strPrompt = QCoreApplication::translate("MainWindow", "Constructing the mesh...", nullptr);
        meshType = 1;
    }
    QString workPath = QDir::currentPath();
    QString tempPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::TempLocation));
    QDir::setCurrent(tempPath);
    QString tempFile = "./out_cgalSurfaceMesh.off";

    //[!].显示等待框
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(strPrompt);

    bool errors = false;
    //ccMesh* mesh = nullptr;
    ccPointCloud* cloud = nullptr;
    std::vector<ccMesh*> lstMesh;
    //[!].并发线程执行，避免处理过程中界面卡死
    QFuture<void> f1 = QtConcurrent::run([&]() {

        //select candidates
        ccHObject::Container clouds;
        ccMesh* mesh = nullptr;
        bool hadNormals = false;
        {
            for (ccHObject *entity : getSelectedEntities())
            {
                if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
                {
                    clouds.push_back(entity);
                    if (entity->isA(CC_TYPES::POINT_CLOUD))
                    {
                        hadNormals |= static_cast<ccPointCloud*>(entity)->hasNormals();
                    }
                }
            }
        }

        //if the cloud(s) already had normals, ask the use if wants to update them or keep them as is (can look strange!)
        bool updateNormals = false;
        if (hadNormals)
        {
            updateNormals = true;
        }

        for (size_t i = 0; i < clouds.size(); ++i)
        {
            ccHObject* ent = clouds[i];
            assert(ent->isKindOf(CC_TYPES::POINT_CLOUD));

            //compute mesh
            cloud = ccHObjectCaster::ToPointCloud(ent);
            QString objName = cloud->getName();
            if (CCCoreLib::SURFACE_MESH == selectedtype && QDir(tempPath).exists())
            {
                //计算原始点云平均间隔
                float resSolution = cc2smReader(cloud).computeCloudResolution();
                resSolution *= surfacemeshpara.subFactor; //下采样间隔

                //均匀采样
                ccOctree::Shared octree = cloud->getOctree();
                if (!octree)
                octree = cloud->computeOctree(nullptr, false);

                CCCoreLib::CloudSamplingTools::SFModulationParams modParams;
                modParams.enabled = false;

                CCCoreLib::ReferenceCloud *sampledCloud = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(cloud,
                    resSolution,
                    modParams);

                if (!sampledCloud)
                {
                    ccConsole::Warning(tr("[Subsampling] Failed to subsample cloud '%1'!").arg(cloud->getName()));
                    errors = true;
                    return;
                }

                ccPointCloud *newPointCloud = cloud->partialClone(sampledCloud);

                delete sampledCloud;
                sampledCloud = nullptr;

                bool res = ccMesh::SufaceMesh(newPointCloud,
                    surfacemeshpara.methodType,
                    resSolution,
                    tempFile.toStdString(),
                    updateNormals,
                    static_cast<PointCoordinateType>(s_meshMaxEdgeLength),
                    2);

                delete newPointCloud;
                newPointCloud = nullptr;

                if (res && QFileInfo::exists(tempFile))
                {
                    //to use the same 'global shift' for multiple files
                    CCVector3d loadCoordinatesShift(0, 0, 0);
                    bool loadCoordinatesTransEnabled = false;

                    FileIOFilter::LoadParameters parameters;
                    {
                        parameters.alwaysDisplayLoadDialog = true;
                        parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
                        parameters._coordinatesShift = &loadCoordinatesShift;
                        parameters._coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
                        //parameters.parentWidget = this;
                        parameters.parentWidget = nullptr;

                    }

                    CC_FILE_ERROR result = CC_FERR_NO_ERROR;
                    ccHObject* newGroup = FileIOFilter::LoadFromFile(tempFile, parameters, result);

                    if (newGroup->getChildrenNumber() > 0)
                    {
                        //convert raw mesh to ccMesh
                        mesh = ccHObjectCaster::ToMesh(newGroup->getChild(0));

                        if (!mesh)
                        {
                            ccLog::Warning("[ccMesh::Triangulate] An error occurred while computing mesh! (not enough memory?)");
                            errors = true;
                            return;
                        }

                        mesh->setName(cloud->getName() + QString(".mesh"));
                        mesh->setDisplay(cloud->getDisplay());
                        bool cloudHadNormals = cloud->hasNormals();
                        //compute per-vertex normals if necessary
                        if (!cloudHadNormals || updateNormals)
                        {
                            mesh->computeNormals(true);
                        }
                        mesh->showNormals(false);
                        mesh->copyGlobalShiftAndScale(*cloud);
                    }
                        QFile::remove(tempFile);
                }
            }
            else
            {
                mesh = ccMesh::Triangulate(cloud,
                    selectedtype,
                    updateNormals,
                    static_cast<PointCoordinateType>(s_meshMaxEdgeLength),
                    2 //XY plane by default
                );
            }

            if (mesh)
            {
                if (CCCoreLib::SURFACE_MESH == selectedtype)
                {
                    mesh->setName(objName + ".Surface.mesh");
                }
                else
                {
                    mesh->setName(objName + ".TIN.mesh");
                    mesh->setMetaData("TIN", true);
                }
                QDir::setCurrent(workPath);

                mesh->showNormals(true);//不显示法线
                cloud->addChild(mesh);
				cloud->showNormals(false);
                cloud->prepareDisplayForRefresh_recursive();
                addToDB(mesh, true, false, true, true);

                lstMesh.push_back(mesh);
            }
            else
            {
                errors = true;
            }
        }

        if (errors)
        {
            ccConsole::Error(tr("Error(s) occurred! See the Console messages"));
        }

        


    });

    while (!f1.isFinished()) {
        QThread::msleep(10);
        QCoreApplication::processEvents();
    }
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    QDir::setCurrent(workPath);
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        return;
    }
    foreach(ccMesh* mesh, lstMesh)
    {
        if (mesh && mesh->getParent())
        {
            mesh->setMetaData("MeshType", QVariant(meshType));
            setSelectedInDB(mesh->getParent(), true);
            win->redraw();
            updateUI();
            setSelectedInDB(mesh, true);
        }
    }
    
    if (errors)
    {
        //表面三角网构建失败
        CS::Widgets::FramelessMessageBox::warning(this,
            QCoreApplication::translate("MainWindow", "Error", nullptr),
            QCoreApplication::translate("MainWindow", "Failed to construct the mesh. Check the data and try again.", nullptr));
    }

    win->redraw();
    updateUI();
    setGlobalZoom();
}

void MainWindow::doActionFitQuadric()
{
	bool errors = false;

	//for all selected entities
	for (ccHObject *entity : getSelectedEntities())
	{
		//look for clouds
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);

			double rms = 0.0;
			ccQuadric* quadric = ccQuadric::Fit(cloud, &rms);
			if (quadric)
			{
				cloud->addChild(quadric);
				quadric->setName(QString("Quadric (%1)").arg(cloud->getName()));
				quadric->setDisplay(cloud->getDisplay());
				quadric->prepareDisplayForRefresh();
				quadric->copyGlobalShiftAndScale(*cloud);
				addToDB(quadric);

				ccConsole::Print(tr("[DoActionFitQuadric] Quadric local coordinate system:"));
				ccConsole::Print(quadric->getTransformation().toString(12, ' ')); //full precision
				ccConsole::Print(tr("[DoActionFitQuadric] Quadric equation (in local coordinate system): ") + quadric->getEquationString());
				ccConsole::Print(QString("[DoActionFitQuadric] RMS: %1").arg(rms));

#if 0
				//test: project the input cloud on the quadric
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* newCloud = static_cast<ccPointCloud*>(cloud)->cloneThis();
					if (newCloud)
					{
						const PointCoordinateType* eq = quadric->getEquationCoefs();
						const Tuple3ub& dims = quadric->getEquationDims();

						const unsigned char dX = dims.x;
						const unsigned char dY = dims.y;
						const unsigned char dZ = dims.z;

						const ccGLMatrix& trans = quadric->getTransformation();
						ccGLMatrix invTrans = trans.inverse();
						for (unsigned i = 0; i < newCloud->size(); ++i)
						{
							CCVector3* P = const_cast<CCVector3*>(newCloud->getPoint(i));
							CCVector3 Q = invTrans * (*P);
							Q.u[dZ] = eq[0] + eq[1] * Q.u[dX] + eq[2] * Q.u[dY] + eq[3] * Q.u[dX] * Q.u[dX] + eq[4] * Q.u[dX] * Q.u[dY] + eq[5] * Q.u[dY] * Q.u[dY];
							*P = trans * Q;
						}
						newCloud->invalidateBoundingBox();
						newCloud->setName(newCloud->getName() + ".projection_on_quadric");
						addToDB(newCloud);
					}
				}
#endif
			}
			else
			{
				ccConsole::Warning(tr("Failed to compute quadric on cloud '%1'").arg(cloud->getName()));
				errors = true;
			}
		}
	}

	if (errors)
	{
		ccConsole::Error(tr("Error(s) occurred: see console"));
	}

	refreshAll();
}

void MainWindow::doActionComputeDistanceMap()
{
	static unsigned steps = 128;
	static double margin = 0.0;
	static bool filterRange = false;
	static double range[2] = { 0.0, 1.0 };

	//show dialog
	{
		QDialog dialog(this);
		Ui_DistanceMapDialog ui;
		ui.setupUi(&dialog);

		ui.stepsSpinBox->setValue(static_cast<int>(steps));
		ui.marginDoubleSpinBox->setValue(margin);
		ui.rangeCheckBox->setChecked(filterRange);
		ui.minDistDoubleSpinBox->setValue(range[0]);
		ui.maxDistDoubleSpinBox->setValue(range[1]);

		if (!dialog.exec())
		{
			return;
		}

		steps = static_cast<unsigned>(ui.stepsSpinBox->value());
		margin = ui.marginDoubleSpinBox->value();
		filterRange = ui.rangeCheckBox->isChecked();
		range[0] = ui.minDistDoubleSpinBox->value();
		range[1] = ui.maxDistDoubleSpinBox->value();
	}

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity->isKindOf(CC_TYPES::MESH) && !entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			//non handled entity type
			continue;
		}

		//CCCoreLib::ChamferDistanceTransform cdt;
		CCCoreLib::SaitoSquaredDistanceTransform cdt;
		if (!cdt.initGrid(Tuple3ui(steps, steps, steps)))
		{
			//not enough memory
			ccLog::Error(tr("Not enough memory!"));
			return;
		}

		ccBBox box = entity->getOwnBB();
		PointCoordinateType largestDim = box.getMaxBoxDim() + static_cast<PointCoordinateType>(margin);
		CCVector3 minCorner = box.getCenter() - CCVector3(1, 1, 1) * (largestDim / 2);
		PointCoordinateType cellDim = largestDim / steps + std::numeric_limits<PointCoordinateType>::epsilon(); //to avoid rounding issues when projecting triangles or points inside the grid

		bool result = false;
		if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(entity);
			result = cdt.initDT(mesh, cellDim, minCorner, &pDlg);
		}
		else
		{
			ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
			result = cdt.initDT(cloud, cellDim, minCorner, &pDlg);
		}

		if (!result)
		{
			ccLog::Error(tr("Not enough memory!"));
			return;
		}

		//cdt.propagateDistance(CHAMFER_345, &pDlg);
		cdt.propagateDistance(&pDlg);

		//convert the grid to a cloud
		ccPointCloud* gridCloud = new ccPointCloud(entity->getName() + QString(".distance_grid(%1)").arg(steps));
		{
			unsigned pointCount = steps * steps*steps;
			if (!gridCloud->reserve(pointCount))
			{
				ccLog::Error(tr("Not enough memory!"));
				delete gridCloud;
				return;
			}

			ccScalarField* sf = new ccScalarField("DT values");
			if (!sf->reserveSafe(pointCount))
			{
				ccLog::Error(tr("Not enough memory!"));
				delete gridCloud;
				sf->release();
				return;
			}

			for (unsigned i = 0; i < steps; ++i)
			{
				for (unsigned j = 0; j < steps; ++j)
				{
					for (unsigned k = 0; k < steps; ++k)
					{
						ScalarType d = std::sqrt(static_cast<ScalarType>(cdt.getValue(i, j, k))) * cellDim;

						if (!filterRange || (d >= range[0] && d <= range[1]))
						{
							gridCloud->addPoint(minCorner + CCVector3(i + 0.5, j + 0.5, k + 0.5) * cellDim);
							sf->addElement(d);
						}
					}
				}
			}

			sf->computeMinAndMax();
			int sfIdx = gridCloud->addScalarField(sf);

			if (gridCloud->size() == 0)
			{
				ccLog::Warning(tr("[DistanceMap] Cloud '%1': no point falls inside the specified range").arg(entity->getName()));
				delete gridCloud;
				gridCloud = nullptr;
			}
			else
			{
				gridCloud->setCurrentDisplayedScalarField(sfIdx);
				gridCloud->showSF(true);
				gridCloud->setDisplay(entity->getDisplay());
				gridCloud->shrinkToFit();
				entity->prepareDisplayForRefresh();
				addToDB(gridCloud);
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeDistToBestFitQuadric3D()
{
	bool ok = true;
	int steps = QInputDialog::getInt(this, tr("Distance to best fit quadric (3D)"), tr("Steps (per dim.)"), 50, 10, 10000, 10, &ok);
	if (!ok)
		return;

	for (ccHObject *entity : getSelectedEntities())
	{
		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
			CCCoreLib::Neighbourhood Yk(cloud);

			double Q[10];
			if (Yk.compute3DQuadric(Q))
			{
				const double& a = Q[0];
				const double& b = Q[1];
				const double& c = Q[2];
				const double& e = Q[3];
				const double& f = Q[4];
				const double& g = Q[5];
				const double& l = Q[6];
				const double& m = Q[7];
				const double& n = Q[8];
				const double& d = Q[9];

				//gravity center
				const CCVector3* G = Yk.getGravityCenter();
				if (!G)
				{
					ccConsole::Warning(tr("Failed to get the center of gravity of cloud '%1'!").arg(cloud->getName()));
					continue;
				}

				const ccBBox bbox = cloud->getOwnBB();
				PointCoordinateType maxDim = bbox.getMaxBoxDim();
				CCVector3 C = bbox.getCenter();

				//Sample points on a cube and compute for each of them the distance to the Quadric
				ccPointCloud* newCloud = new ccPointCloud();
				if (!newCloud->reserve(steps*steps*steps))
				{
					ccConsole::Error(tr("Not enough memory!"));
				}

				const char defaultSFName[] = "Dist. to 3D quadric";
				int sfIdx = newCloud->getScalarFieldIndexByName(defaultSFName);
				if (sfIdx < 0)
					sfIdx = newCloud->addScalarField(defaultSFName);
				if (sfIdx < 0)
				{
					ccConsole::Error(tr("Couldn't allocate a new scalar field for computing distances! Try to free some memory ..."));
					delete newCloud;
					continue;
				}

				ccScalarField* sf = static_cast<ccScalarField*>(newCloud->getScalarField(sfIdx));
				assert(sf);

				for (int x = 0; x < steps; ++x)
				{
					CCVector3 P;
					P.x = C.x + maxDim * (static_cast<PointCoordinateType>(x) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
					for (int y = 0; y < steps; ++y)
					{
						P.y = C.y + maxDim * (static_cast<PointCoordinateType>(y) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
						for (int z = 0; z < steps; ++z)
						{
							P.z = C.z + maxDim * (static_cast<PointCoordinateType>(z) / static_cast<PointCoordinateType>(steps - 1) - CCCoreLib::PC_ONE / 2);
							newCloud->addPoint(P);

							//compute distance to quadric
							CCVector3 Pc = P - *G;
							ScalarType dist = static_cast<ScalarType>(a*Pc.x*Pc.x + b * Pc.y*Pc.y + c * Pc.z*Pc.z
								+ e * Pc.x*Pc.y + f * Pc.y*Pc.z + g * Pc.x*Pc.z
								+ l * Pc.x + m * Pc.y + n * Pc.z + d);

							sf->addElement(dist);
						}
					}
				}

				if (sf)
				{
					sf->computeMinAndMax();
					newCloud->setCurrentDisplayedScalarField(sfIdx);
					newCloud->showSF(true);
				}
				newCloud->setName(tr("Distance map to 3D quadric"));
				newCloud->setDisplay(cloud->getDisplay());
				newCloud->prepareDisplayForRefresh();

				addToDB(newCloud);
			}
			else
			{
				ccConsole::Warning(tr("Failed to compute 3D quadric on cloud '%1'").arg(cloud->getName()));
			}
		}
	}

	refreshAll();
}

void MainWindow::doActionComputeCPS()
{
	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccOrderChoiceDlg dlg(m_selectedEntities.front(), tr("Compared"),
		m_selectedEntities[1], tr("Reference"),
		this);
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* srcCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	if (!compCloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
	{
		ccConsole::Error(tr("Compared cloud must be a real point cloud!"));
		return;
	}
	ccPointCloud* cmpPC = static_cast<ccPointCloud*>(compCloud);

	static const char DEFAULT_CPS_TEMP_SF_NAME[] = "CPS temporary";
	int sfIdx = cmpPC->getScalarFieldIndexByName(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cmpPC->addScalarField(DEFAULT_CPS_TEMP_SF_NAME);
	if (sfIdx < 0)
	{
		ccConsole::Error(tr("Couldn't allocate a new scalar field for computing distances! Try to free some memory ..."));
		return;
	}
	cmpPC->setCurrentScalarField(sfIdx);
	if (!cmpPC->enableScalarField())
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}
	//cmpPC->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueToNaN); //now done by default by computeCloud2CloudDistances

	CCCoreLib::ReferenceCloud CPSet(srcCloud);
	ccProgressDialog pDlg(true, this);
	CCCoreLib::DistanceComputationTools::Cloud2CloudDistancesComputationParams params;
	params.CPSet = &CPSet;
	int result = CCCoreLib::DistanceComputationTools::computeCloud2CloudDistances(compCloud, srcCloud, params, &pDlg);
	cmpPC->deleteScalarField(sfIdx);

	if (result >= 0)
	{
		ccPointCloud* newCloud = nullptr;
		//if the source cloud is a "true" cloud, the extracted CPS
		//will also get its attributes
		newCloud = srcCloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(srcCloud)->partialClone(&CPSet) : ccPointCloud::From(&CPSet, srcCloud);

		newCloud->setName(QString("[%1]->CPSet(%2)").arg(srcCloud->getName(), compCloud->getName()));
		newCloud->setDisplay(compCloud->getDisplay());
		newCloud->prepareDisplayForRefresh();
		addToDB(newCloud);

		//we hide the source cloud (for a clearer display)
		srcCloud->setEnabled(false);
		srcCloud->prepareDisplayForRefresh();
	}

	refreshAll();
}

void MainWindow::doActionComputeNormals()
{
	if (!ccEntityAction::computeNormals(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsMST()
{
	if (!ccEntityAction::orientNormalsMST(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionOrientNormalsFM()
{
	if (!ccEntityAction::orientNormalsFM(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

static int s_innerRectDim = 2;
void MainWindow::doActionFindBiggestInnerRectangle()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities.front() : nullptr;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select one point cloud!"));
		return;
	}

	bool ok;
	int dim = QInputDialog::getInt(this, tr("Dimension"), tr("Orthogonal dim (X=0 / Y=1 / Z=2)"), s_innerRectDim, 0, 2, 1, &ok);
	if (!ok)
		return;
	s_innerRectDim = dim;

	ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
	ccBox* box = ccInnerRect2DFinder().process(cloud, static_cast<unsigned char>(dim));

	if (box)
	{
		cloud->addChild(box);
		box->setVisible(true);
		box->setDisplay(cloud->getDisplay());
		box->setDisplay(cloud->getDisplay());
		addToDB(box);
	}

	updateUI();
}

void MainWindow::doActionFastRegistration(FastRegistrationMode mode)
{
	//we need at least 1 entity
	if (m_selectedEntities.empty())
		return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	for (ccHObject *entity : selectedEntities)
	{
		ccBBox box = entity->getBB_recursive();

		CCVector3 T; //translation

		switch (mode)
		{
		case MoveBBCenterToOrigin:
			T = -box.getCenter();
			break;
		case MoveBBMinCornerToOrigin:
			T = -box.minCorner();
			break;
		case MoveBBMaxCornerToOrigin:
			T = -box.maxCorner();
			break;
		default:
			assert(false);
			return;
		}

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans.setTranslation(T);

		forceConsoleDisplay();
		ccConsole::Print(tr("[Synchronize] Transformation matrix (%1):").arg(entity->getName()));
		ccConsole::Print(glTrans.toString(12, ' ')); //full precision
		ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

		//we temporarily detach entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(entity, objContext);

		entity->prepareDisplayForRefresh_recursive();
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	zoomOnSelectedEntities();

	updateUI();
}

void MainWindow::doActionMatchBBCenters()
{
	//we need at least 2 entities
	if (m_selectedEntities.size() < 2)
		return;

	//we must backup 'm_selectedEntities' as removeObjectTemporarilyFromDBTree can modify it!
	ccHObject::Container selectedEntities = m_selectedEntities;

	//by default, we take the first entity as reference
	//TODO: maybe the user would like to select the reference himself ;)
	ccHObject* refEnt = selectedEntities[0];
	CCVector3 refCenter = refEnt->getBB_recursive().getCenter();

	for (ccHObject *entity : selectedEntities) //warning, getSelectedEntites may change during this loop!
	{
		CCVector3 center = entity->getBB_recursive().getCenter();

		CCVector3 T = refCenter - center;

		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;

		forceConsoleDisplay();
		ccConsole::Print(tr("[Synchronize] Transformation matrix (%1 --> %2):").arg(entity->getName(), selectedEntities[0]->getName()));
		ccConsole::Print(glTrans.toString(12, ' ')); //full precision
		ccConsole::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

		//we temporarily detach entity, as it may undergo
		//'severe' modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
		entity->applyGLTransformation_recursive(&glTrans);
		putObjectBackIntoDBTree(entity, objContext);

		entity->prepareDisplayForRefresh_recursive();
	}

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	zoomOnSelectedEntities();

	updateUI();
}

//semi-persistent parameters
static ccLibAlgorithms::ScaleMatchingAlgorithm s_msAlgorithm = ccLibAlgorithms::PCA_MAX_DIM;
static double s_msRmsDiff = 1.0e-5;
static int s_msFinalOverlap = 100;

void MainWindow::doActionMatchScales()
{
	//we need at least 2 entities
	if (m_selectedEntities.size() < 2)
		return;

	//we must select the point clouds and meshes
	ccHObject::Container selectedEntities;
	try
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD)
				|| entity->isKindOf(CC_TYPES::MESH))
			{
				selectedEntities.push_back(entity);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccConsole::Error(tr("Not enough memory!"));
		return;
	}

	ccMatchScalesDlg msDlg(selectedEntities, 0, this);
	msDlg.setSelectedAlgorithm(s_msAlgorithm);
	msDlg.rmsDifferenceLineEdit->setText(QString::number(s_msRmsDiff, 'e', 1));
	msDlg.overlapSpinBox->setValue(s_msFinalOverlap);

	if (!msDlg.exec())
		return;

	//save semi-persistent parameters
	s_msAlgorithm = msDlg.getSelectedAlgorithm();
	if (s_msAlgorithm == ccLibAlgorithms::ICP_SCALE)
	{
		s_msRmsDiff = msDlg.rmsDifferenceLineEdit->text().toDouble();
		s_msFinalOverlap = msDlg.overlapSpinBox->value();
	}

	ccLibAlgorithms::ApplyScaleMatchingAlgorithm(s_msAlgorithm,
		selectedEntities,
		s_msRmsDiff,
		s_msFinalOverlap,
		msDlg.getSelectedIndex(),
		this);

	//reselect previously selected entities!
	if (m_ccRoot)
		m_ccRoot->selectEntities(selectedEntities);

	refreshAll();
	updateUI();
}

void MainWindow::doActionSORFilter()
{
    InformationPrompt set(QApplication::translate("importantinformationtips", "The greater the deletion intensity, the fewer points are retained.", nullptr));
    QApplication::processEvents(QEventLoop::AllEvents, 100);
    deselectAllOtherObject(false);
    ccGLWindow* win = getActiveGLWindow();
    if (win ) {
        win->update();
    }
    update();
    ccHObject::Container selectedEntities = getSelectedEntities();
    ccSORFilterDlg sorDlg(this);
	sorDlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Delete outliers", nullptr));
	//set semi-persistent/dynamic parameters
	static int s_sorFilterKnn = 6;
	static double s_sorFilterNSigma = 1.0;
	sorDlg.setKNN(s_sorFilterKnn);
	sorDlg.setNSigma(s_sorFilterNSigma);
    sorDlg.setFileSavePath(this->getDBTreeCloudFilePath(getSelectedEntities()));
   
	if (!sorDlg.exec())
		return;
    QString path = sorDlg.getSaveFilePath();
    if (!path.isEmpty())
    {
        QDir dirs(path);
        path = dirs.path();
    }
	//update semi-persistent/dynamic parameters
	s_sorFilterKnn = sorDlg.KNN();
	s_sorFilterNSigma = sorDlg.nSigma();

	ccProgressDialog pDlg(false, this);
    pDlg.setAutoReset(false);
	pDlg.setAutoClose(true);
	pDlg.setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    pDlg.setWindowTitle(tr("Discrete point filtering"));
    pDlg.setInfo(tr("Filtering discrete points, please wait..."));
	bool firstCloud = true;

	for (ccHObject *entity : selectedEntities)
	{
        m_ccRoot->unselectEntity(entity);
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (cloud && lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
			continue;
		}
		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::sorFilter(cloud,
			s_sorFilterKnn,
			s_sorFilterNSigma,
			nullptr,
			&pDlg);

		if (selection && cloud)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(tr("[DoActionSORFilter] No points were removed from cloud '%1'").arg(cloud->getName()));
			}
			else
			{
				ccPointCloud* cleanCloud = cloud->partialClone(selection);
				if (cleanCloud)
				{
                    QString cloudName = cloud->getName() + QString(".clean1");
                    ccHObject * parentCloud = static_cast<ccHObject*>(cloud)->getParent();
                    int count = 2;
                    while (1) {
                        bool found = false;
                        for (int i = 0; i < parentCloud->getChildrenNumber(); i++) {
                            if (parentCloud->getChild(i)->getName() == cloudName) {
                                cloudName.chop(1);
                                cloudName += QString::number(count);
                                count++;
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            break;
                        }
                    }
                    cleanCloud->setName(cloudName);
                    if (!path.isEmpty()) {
                       
                        pDlg.setInfo(tr("Processing"));
                        pDlg.setInfo(tr("Saving, please wait"));
                        pDlg.setValue(95);
                        Sleep(500);
                        if (!m_ccRoot && cloud->getParent()) {
                            return;
                        }
                        QDir saveFileNames(path);
                        QString FilePaths = saveFileNames.filePath(cleanCloud->getName());
                        if (!threadSaveLasFile(cleanCloud, FilePaths)) {
                            qDebug() << "save SORLas file error." << path + cleanCloud->getName();
                        }
                    }
                    pDlg.setValue(100);
                    Sleep(500);
                    pDlg.stop();

					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud);
                    
					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(tr("[DoActionSORFilter] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			if (cloud != nullptr)
			{
				ccConsole::Warning(tr("[DoActionSORFilter] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
			}
			else
			{
				ccConsole::Warning(tr("[DoActionSORFilter] Trying to apply the noise filter to null cloud"));
			}
		}
	}

	refreshAll();
	updateUI();
    MainWindow::TheInstance()->setGlobalZoom();
}

void MainWindow::doActionFilterNoise()
{
	PointCoordinateType kernelRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);

	ccNoiseFilterDlg noiseDlg(this);

	//set semi-persistent/dynamic parameters
	static bool s_noiseFilterUseKnn = false;
	static int s_noiseFilterKnn = 6;
	static bool s_noiseFilterUseAbsError = false;
	static double s_noiseFilterAbsError = 1.0;
	static double s_noiseFilterNSigma = 1.0;
	static bool s_noiseFilterRemoveIsolatedPoints = false;
	noiseDlg.radiusDoubleSpinBox->setValue(kernelRadius);
	noiseDlg.knnSpinBox->setValue(s_noiseFilterKnn);
	noiseDlg.nSigmaDoubleSpinBox->setValue(s_noiseFilterNSigma);
	noiseDlg.absErrorDoubleSpinBox->setValue(s_noiseFilterAbsError);
	noiseDlg.removeIsolatedPointsCheckBox->setChecked(s_noiseFilterRemoveIsolatedPoints);
	if (s_noiseFilterUseAbsError)
		noiseDlg.absErrorRadioButton->setChecked(true);
	else
		noiseDlg.relativeRadioButton->setChecked(true);
	if (s_noiseFilterUseKnn)
		noiseDlg.knnRadioButton->setChecked(true);
	else
		noiseDlg.radiusRadioButton->setChecked(true);

	if (!noiseDlg.exec())
		return;

	//update semi-persistent/dynamic parameters
	kernelRadius = static_cast<PointCoordinateType>(noiseDlg.radiusDoubleSpinBox->value());
	s_noiseFilterUseKnn = noiseDlg.knnRadioButton->isChecked();
	s_noiseFilterKnn = noiseDlg.knnSpinBox->value();
	s_noiseFilterUseAbsError = noiseDlg.absErrorRadioButton->isChecked();
	s_noiseFilterNSigma = noiseDlg.nSigmaDoubleSpinBox->value();
	s_noiseFilterAbsError = noiseDlg.absErrorDoubleSpinBox->value();
	s_noiseFilterRemoveIsolatedPoints = noiseDlg.removeIsolatedPointsCheckBox->isChecked();

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	bool firstCloud = true;

	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: and 'selectEntity' will change the set of currently selected entities!

	for (ccHObject *entity : selectedEntities)
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (cloud && lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), haveOneSelection());
			continue;
		}

		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::noiseFilter(cloud,
			kernelRadius,
			s_noiseFilterNSigma,
			s_noiseFilterRemoveIsolatedPoints,
			s_noiseFilterUseKnn,
			s_noiseFilterKnn,
			s_noiseFilterUseAbsError,
			s_noiseFilterAbsError,
			nullptr,
			&pDlg);

		if (selection && cloud)
		{
			if (selection->size() == cloud->size())
			{
				ccLog::Warning(tr("[DoActionFilterNoise] No points were removed from cloud '%1'").arg(cloud->getName()));
			}
			else
			{
				ccPointCloud* cleanCloud = cloud->partialClone(selection);
				if (cleanCloud)
				{
					cleanCloud->setName(cloud->getName() + QString(".clean"));
					cleanCloud->setDisplay(cloud->getDisplay());
					if (cloud->getParent())
						cloud->getParent()->addChild(cleanCloud);
					addToDB(cleanCloud);

					cloud->setEnabled(false);
					if (firstCloud)
					{
						ccConsole::Warning(tr("Previously selected entities (sources) have been hidden!"));
						firstCloud = false;
						m_ccRoot->selectEntity(cleanCloud, true);
					}
				}
				else
				{
					ccConsole::Warning(tr("[DoActionFilterNoise] Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
				}
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			if (cloud != nullptr)
			{
				ccConsole::Warning(tr("[DoActionFilterNoise] Failed to apply the noise filter to cloud '%1'! (not enough memory?)").arg(cloud->getName()));
			}
			else
			{
				ccConsole::Warning(tr("[DoActionFilterNoise] Trying to apply the noise filter to null cloud"));
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doActionUnroll()
{
	//there should be only one point cloud or one mesh!
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select one and only one entity!"));
		return;
	}

	//if selected entity is a mesh, the method will be applied to its vertices
	bool lockedVertices;
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities.front(), &lockedVertices);
	if (lockedVertices)
	{
		ccUtils::DisplayLockedVerticesWarning(m_selectedEntities.front()->getName(), true);
		return;
	}

	//for "real" point clouds only
	if (!cloud || !cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Method can't be applied on locked vertices or virtual point clouds!"));
		return;
	}
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	ccUnrollDlg unrollDlg(this);
	unrollDlg.fromPersistentSettings();
	if (!unrollDlg.exec())
		return;
	unrollDlg.toPersistentSettings();

	ccPointCloud::UnrollMode mode = unrollDlg.getType();
	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
	unsigned char dim = static_cast<unsigned char>(unrollDlg.getAxisDimension());
	bool exportDeviationSF = unrollDlg.exportDeviationSF();
	CCVector3 center = unrollDlg.getAxisPosition();

	//let's rock unroll ;)
	ccProgressDialog pDlg(true, this);

	double startAngle_deg = 0.0;
	double stopAngle_deg = 360.0;
	unrollDlg.getAngleRange(startAngle_deg, stopAngle_deg);
	if (startAngle_deg >= stopAngle_deg)
	{
		CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Invalid angular range"));
		return;
	}

	ccPointCloud* output = nullptr;
	switch (mode)
	{
	case ccPointCloud::CYLINDER:
	{
		ccPointCloud::UnrollCylinderParams params;
		params.radius = radius;
		params.axisDim = dim;
		if (unrollDlg.isAxisPositionAuto())
		{
			center = pc->getOwnBB().getCenter();
		}
		params.center = center;
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, &pDlg);
	}
	break;

	case ccPointCloud::CONE:
	case ccPointCloud::STRAIGHTENED_CONE:
	case ccPointCloud::STRAIGHTENED_CONE2:
	{
		ccPointCloud::UnrollConeParams params;
		params.radius = (mode == ccPointCloud::CONE ? 0 : radius);
		params.apex = center;
		params.coneAngle_deg = unrollDlg.getConeHalfAngle();
		params.axisDim = dim;
		output = pc->unroll(mode, &params, exportDeviationSF, startAngle_deg, stopAngle_deg, &pDlg);
	}
	break;

	default:
		assert(false);
		break;
	}

	if (output)
	{
		if (m_selectedEntities.front()->isA(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities.front());
			mesh->setEnabled(false);
			ccConsole::Warning(tr("[Unroll] Original mesh has been automatically hidden"));
			ccMesh* outputMesh = mesh->cloneMesh(output);
			outputMesh->addChild(output);
			addToDB(outputMesh, true, true, false, true);
			outputMesh->setEnabled(true);
			outputMesh->setVisible(true);
		}
		else
		{
			pc->setEnabled(false);
			ccConsole::Warning(tr("[Unroll] Original cloud has been automatically hidden"));
			if (pc->getParent())
			{
				pc->getParent()->addChild(output);
			}
			addToDB(output, true, true, false, true);
		}
		updateUI();
	}
}

ccGLWindow* MainWindow::getActiveGLWindow()
{
	if (!m_mdiArea)
	{
		return nullptr;
	}

	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		return GLWindowFromWidget(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return GLWindowFromWidget(subWindowList[0]->widget());
		}
	}

	return nullptr;
}

QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindow* win)
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	for (int i = 0; i < subWindowList.size(); ++i)
	{
		if (GLWindowFromWidget(subWindowList[i]->widget()) == win)
			return subWindowList[i];
	}

	//not found!
	return nullptr;
}

ccGLWindow* MainWindow::getGLWindow(int index) const
{
	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (index >= 0 && index < subWindowList.size())
	{
		ccGLWindow* win = GLWindowFromWidget(subWindowList[index]->widget());
		assert(win);
		return win;
	}
	else
	{
		assert(false);
		return nullptr;
	}
}

int MainWindow::getGLWindowCount() const
{
	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
}

void MainWindow::zoomIn()
{
	ccGLWindow* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(15.0f);
	}
}

void MainWindow::initPCDShiftValue()
{
    QSettings shiftsettings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    shiftsettings.beginGroup("EntityPointShift");
    shiftsettings.setValue("x", 0);
    shiftsettings.setValue("y", 0);
    shiftsettings.setValue("z", 0);
    shiftsettings.setValue("scale", 1);
    shiftsettings.endGroup();
}

void MainWindow::zoomOut()
{
	ccGLWindow* win = MainWindow::getActiveGLWindow();
	if (win)
	{
		//we simulate a real wheel event
		win->onWheelEvent(-15.0f);
	}
}

ccGLWindow* MainWindow::new3DView(bool allowEntitySelection, bool is3D)
{
	assert(m_ccRoot && m_mdiArea);

	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;

	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error(tr("Failed to create the 3D view"));
		assert(false);
		return nullptr;
	}

	//restore options
	{
		QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
		bool autoPickRotationCenter = settings.value(ccPS::AutoPickRotationCenter(), true).toBool();
		//view3D->setAutoPickPivotAtCenter(autoPickRotationCenter);
	}

	//viewWidget->setMinimumSize(400, 300);
	m_mdiArea->addMethubSubWindow(viewWidget, Qt::FramelessWindowHint, is3D);//无边框的3d view窗口 janson.yang
	//m_mdiArea->addSubWindow(viewWidget);
	
	if (allowEntitySelection)
	{
		connect(view3D, &ccGLWindow::entitySelectionChanged, this, [=](ccHObject *entity) {
			m_ccRoot->selectEntity(entity);
		});

		connect(view3D, &ccGLWindow::entitiesSelectionChanged, this, [=](std::unordered_set<int> entities) {
			m_ccRoot->selectEntities(entities);
		});
	}

	//'echo' mode
	connect(view3D, &ccGLWindow::UpdatePosition, this, &MainWindow::updateStatusbarInformation);
	connect(view3D, &ccGLWindow::mouseWheelRotated, this, &MainWindow::echoMouseWheelRotate);
	connect(view3D, &ccGLWindow::viewMatRotated, this, &MainWindow::echoBaseViewMatRotation);
	connect(view3D, &ccGLWindow::cameraPosChanged, this, &MainWindow::echoCameraPosChanged);
	connect(view3D, &ccGLWindow::pivotPointChanged, this, &MainWindow::echoPivotPointChanged);
	connect(view3D, &ccGLWindow::sigTransScreenPosTo3DPos, this, &MainWindow::slotShowCurrentPos);


	connect(view3D, &QObject::destroyed, this, &MainWindow::prepareWindowDeletion);
	connect(view3D, &ccGLWindow::filesDropped, this, &MainWindow::addToDBAuto, Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	connect(view3D, &ccGLWindow::newLabel, this, &MainWindow::handleNewLabel);
	connect(view3D, &ccGLWindow::exclusiveFullScreenToggled, this, &MainWindow::onExclusiveFullScreenToggled);
    connect(FJStyleManager::Instance(), &FJStyleManager::signalDeleteVertexArrays, view3D, &ccGLWindow::deleteVertexArrays);
    connect(FJStyleManager::Instance(), &FJStyleManager::signalDeleteBuffers, view3D, &ccGLWindow::deleteBuffers);
	connect(view3D, &ccGLWindow::signalCloudWillAddDBRoot, this, &MainWindow::updateGlWindowCloudPointSize);

	if (m_pickingHub)
	{
		//we must notify the picking hub as well if the window is destroyed
		connect(view3D, &QObject::destroyed, m_pickingHub, &ccPickingHub::onActiveWindowDeleted);
	}

	view3D->setSceneDB(m_ccRoot->getRootEntity());
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	m_ccRoot->updatePropertiesView();

	QMainWindow::statusBar()->showMessage(tr("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();

    view3D->setPointSize(ccGui::Parameters().m_globalpointsize);
    qDebug() << "cuurrent display glWindow:" << view3D->windowTitle() << "set globaPointSize" << ccGui::Parameters().m_globalpointsize;

	return view3D;
}

void MainWindow::prepareWindowDeletion(QObject* glWindow)
{
	if (!m_ccRoot)
		return;

	//we assume only ccGLWindow can be connected to this slot!
	ccGLWindow* win = qobject_cast<ccGLWindow*>(glWindow);

	m_ccRoot->hidePropertiesView();
	m_ccRoot->getRootEntity()->removeFromDisplay_recursive(win);
	m_ccRoot->updatePropertiesView();
}

static bool s_autoSaveGuiElementPos = true;
void MainWindow::doActionResetGUIElementsPos()
{
	//show the user it will be maximized
	showMaximized();

	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.remove(ccPS::MainWinGeom());
	settings.remove(ccPS::MainWinState());

	CS::Widgets::FramelessMessageBox::information(this,
		tr("Restart"),
		tr("To finish the process, you'll have to close and restart CloudCompare"));

	//to avoid saving them right away!
	s_autoSaveGuiElementPos = false;
}

void MainWindow::doActionResetAllVBOs()
{
	ccHObject::Container clouds;
	m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);

	size_t releasedSize = 0;
	for (ccHObject* entity : clouds)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			releasedSize += cloud->vboSize();
			cloud->releaseVBOs();
		}
	}

	if (releasedSize != 0)
	{
		ccLog::Print(tr("All VBOs have been released (%1 Mb)").arg(releasedSize / static_cast<double>(1 << 20), 0, 'f', 2));
		if (ccGui::Parameters().useVBOs)
		{
			ccLog::Warning(tr("You might want to disable the 'use VBOs' option in the Display Settings to keep the GPU memory empty"));
		}
	}
	else
	{
		ccLog::Print(tr("No VBO allocated"));
	}
}

void MainWindow::restoreGUIElementsPos()
{
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);

	QString dirname = QCoreApplication::applicationDirPath();
	dirname += "/theme/dockconfig.ini";
	QFile file(dirname);
	if (file.open(QIODevice::ReadOnly))
	{
		QDataStream in(&file);
		QVariant previousState;
		QByteArray state;
		in >> state;
		restoreState(state);
		file.close();
	}
    this->setWindowState(Qt::WindowMaximized);

}

void MainWindow::showEvent(QShowEvent* event)
{
	if (m_firstShow)
	{
		restoreGUIElementsPos();

		m_firstShow = false;
        //[!].开启加密狗
        m_TimerReadDog.start();

        //[!].更新模块授权
        slotUpdateRegisterCoderCategory();
	}
	QMainWindow::showEvent(event);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "close is enter";
	//是否再次弹出提示关闭按钮，配置文件 janson.yang 2022.5.19
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);

	settings.beginGroup("isPropmt");
	bool isNotPromtBoxvalue = settings.value("isNotPropmtCloseBox",false).toBool();

	//If we don't have anything displayed, then just close...
	if (m_ccRoot && (m_ccRoot->getRootEntity()->getChildrenNumber() == 0) || isNotPromtBoxvalue)
	{
		event->accept();
	}
	else	// ...otherwise confirm
	{
		CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question,
			tr("Quit"),
			tr("Are you sure you want to quit?"),
			QMessageBox::Cancel | QMessageBox::Ok,
			this);
		//添加复选框 janson.yang 2022.5.19
		QCheckBox  *dontPrompt = new QCheckBox(QCoreApplication::translate("MainWindow", "Never ask again", nullptr), this);
		message_box.setCheckBox(dontPrompt);

		if (message_box.exec() == QMessageBox::Ok)
		{
			if (dontPrompt->isChecked())
				settings.setValue("isNotPropmtCloseBox", true);
			else
				settings.setValue("isNotPropmtCloseBox", false);
			delete dontPrompt;
			dontPrompt = nullptr;
			event->accept();
		}
		else
		{
			event->ignore();
		}
	}

	settings.endGroup();

	//[!].关闭加密时钟
	m_TimerReadDog.stop();

    if (event->isAccepted())
    {
        if (qApp->property("document").toBool())
        {
            delete m_pPdfoutDlg;
            m_pPdfoutDlg = nullptr;
        }
    }
	if (s_autoSaveGuiElementPos)
	{
		saveGUIElementsPos();
	}

    qDebug() << "close is end！";
}

void MainWindow::saveGUIElementsPos()
{
	//save the state as settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.setValue(ccPS::MainWinGeom(), saveGeometry());
	//dobywangwenyu
	//settings.setValue(ccPS::MainWinState(), saveState());
	QString dirname = QCoreApplication::applicationDirPath();
	dirname += "/theme/dockconfig.ini";
	QFile file(dirname);
	if (file.open(QIODevice::WriteOnly))
	{
		QDataStream out(&file);
		out << this->saveState();
		file.close();
	}
}

void MainWindow::moveEvent(QMoveEvent* event)
{
	QMainWindow::moveEvent(event);
	
	updateOverlayDialogsPlacement();
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
	updateOverlayDialogsPlacement();
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	QMainWindow::paintEvent(event);
	Q_FOREACH(QTabBar* tab, this->findChildren<QTabBar *>())
	{
		tab->setDrawBase(false);
    }  
}

void MainWindow::registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos)
{
	//check for existence
	for (ccMDIDialogs& mdi : m_mdiDialogs)
	{
		if (mdi.dialog == dlg)
		{
			//we only update its position in this case
			mdi.position = pos;
			repositionOverlayDialog(mdi);
			return;
		}
	}

	//otherwise we add it to DB
	m_mdiDialogs.push_back(ccMDIDialogs(dlg, pos));

	//automatically update the dialog placement when its shown
	//connect(dlg, &ccOverlayDialog::shown, this, [=]()
	//{
	//	//check for existence
	//	for (ccMDIDialogs& mdi : m_mdiDialogs)
	//	{
	//		if (mdi.dialog == dlg)
	//		{
	//			repositionOverlayDialog(mdi);
	//			break;
	//		}
	//	}
	//});

	repositionOverlayDialog(m_mdiDialogs.back());
}

void MainWindow::unregisterOverlayDialog(ccOverlayDialog* dialog)
{
	for (std::vector<ccMDIDialogs>::iterator it = m_mdiDialogs.begin(); it != m_mdiDialogs.end(); ++it)
	{
		if (it->dialog == dialog)
		{
			m_mdiDialogs.erase(it);
			break;
		}
	}
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
	switch (event->type())
	{
	case QEvent::Resize:
	case QEvent::Move:
		updateOverlayDialogsPlacement();
		break;
    case QEvent::KeyPress:
    case QEvent::KeyRelease: {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
        if (keyEvent->key() == Qt::Key_Delete) {
            return true;
        }
        break;
    }
	default:
		//nothing to do
		break;
	}


	// standard event processing
	//return QObject::eventFilter(obj, event);

	//这个过滤是为了把ribbonBar上的动作传递到mainwindow，再传递到frameless，
	//由于ribbonbar会遮挡刁frameless的区域，导致frameless无法捕获这些消息
	//2022.4.18 janson.yang 
	return SARibbonMainWindow::eventFilter(obj, event);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Escape:
	{
		if (s_pickingWindow != nullptr)
		{
			cancelPreviousPickingOperation(true);
		}
		break;
	}

	default:
		QMainWindow::keyPressEvent(event);
	}
}

void MainWindow::updateOverlayDialogsPlacement()
{
	for (ccMDIDialogs& mdiDlg : m_mdiDialogs)
	{
		repositionOverlayDialog(mdiDlg);
	}
    emit signalPosChanged();
}

void MainWindow::repositionOverlayDialog(ccMDIDialogs& mdiDlg)
{
    if (!mdiDlg.dialog || !mdiDlg.dialog->isVisible() || !m_mdiArea)
        return;

	int dx = 0;
	int dy = 0;
	static const int margin = 5;
	switch (mdiDlg.position)
	{
	case Qt::TopLeftCorner:
		dx = margin;
		dy = margin;
		break;
	case Qt::TopRightCorner:
		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
		dy = margin;
		break;
	case Qt::BottomLeftCorner:
		dx = margin;
		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
		break;
	case Qt::BottomRightCorner:
		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
		break;
	}

	//show();
	mdiDlg.dialog->move(m_mdiArea->mapToGlobal(QPoint(dx, dy)));
	mdiDlg.dialog->raise();
}

void MainWindow::toggleVisualDebugTraces()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleDebugTrace();
		win->redraw(false, false);
	}
}

void MainWindow::toggleFullScreen(bool state)
{
	if (state)
		showFullScreen();
	else
		showNormal();

#ifdef Q_OS_MAC
	if (state)
	{
		m_UI->actionFullScreen->setText(tr("Exit Full Screen"));
	}
	else
	{
		m_UI->actionFullScreen->setText(tr("Enter Full Screen"));
	}
#endif
}

void MainWindow::toggleExclusiveFullScreen(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleExclusiveFullScreen(state);
	}
}

void MainWindow::doActionShowHelpDialog()
{
	CS::Widgets::FramelessMessageBox messageBox;
	messageBox.setTextFormat(Qt::RichText);
	messageBox.setWindowTitle("Documentation");
	messageBox.setText("Please look at the <a href='http://www.cloudcompare.org/doc/wiki'>wiki</a>");
	messageBox.setStandardButtons(QMessageBox::Ok);
	messageBox.exec();
}

void MainWindow::freezeUI(bool state)
{
	////freeze standard plugins
    if (!m_UI)
    {
        return;
    }
	m_UI->toolBarMainTools->setDisabled(state);
	m_UI->toolBarSFTools->setDisabled(state);

	m_pluginUIManager->mainPluginToolbar()->setDisabled(state);

	//freeze plugin toolbars
	for (QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars())
	{
		toolbar->setDisabled(state);
	}

	m_UI->DockableDBTree->setDisabled(state);
	//m_UI->menubar->setDisabled(state);

	if (state)
	{
		menuEdit->setDisabled(true);
		menuTools->setDisabled(true);

		if (ribbon)
		{
			ribbon->applicationButton()->setDisabled(true);
			if (ribbon->categoryByObjectName("edit"))
			{
				ribbon->categoryByObjectName("edit")->setDisabled(true);
			}
			if (ribbon->categoryByObjectName("Start"))
			{
				ribbon->categoryByObjectName("Start")->setDisabled(true);
			}
			if (ribbon->categoryByObjectName("Paint"))
			{
				ribbon->categoryByObjectName("Paint")->setDisabled(true);
			}
            if (ribbon->categoryByObjectName("Forestry"))
            {
                ribbon->categoryByObjectName("Forestry")->setDisabled(true);
            }
		}
		setMenuButtonEnable("view","pannelview", "CrosssectionBtn",false);
	}
	else
	{
		updateMenus();
		
		if (ribbon)
		{
			ribbon->applicationButton()->setDisabled(false);
			if (ribbon->categoryByObjectName("edit"))
			{
				ribbon->categoryByObjectName("edit")->setDisabled(false);
			}
			if (ribbon->categoryByObjectName("Start"))
			{
				ribbon->categoryByObjectName("Start")->setDisabled(false);
			}
			if (ribbon->categoryByObjectName("Paint"))
			{
				ribbon->categoryByObjectName("Paint")->setDisabled(false);
			}
            if (ribbon->categoryByObjectName("Forestry"))
            {
                ribbon->categoryByObjectName("Forestry")->setDisabled(false);
            }
			setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
		}
	}

	m_UI->actionPointPicking->setDisabled(state);
	m_UI->actionSegment->setDisabled(state);
	m_uiFrozen = state;
}

void MainWindow::activateRegisterPointPairTool()
{

    if (!haveSelection())
    {
        ccConsole::Error(tr("Select at least one entity (point cloud or mesh)!"));
        return;
    }
    m_pViewerAction->setEnabled(false);
    changeTo2dView();

    ccHObject::Container alignedEntities;
    ccHObject::Container refEntities;
    try
    {
        ccHObject::Container entities;
        entities.reserve(m_selectedEntities.size());

        for (ccHObject* entity : m_selectedEntities)
        {
            //for now, we only handle clouds or meshes
            if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
            {
                entities.push_back(entity);
            }
        }

        if (entities.empty())
        {
            ccConsole::Error("Select at least two entity (point cloud or mesh)!");
            return;
        }
        else if (entities.size() == 1)
        {
            ccConsole::Error("Select at least two entity (point cloud or mesh)!");
            return;
        }
        else if (entities.size() == 2)
        {
            alignedEntities.push_back(entities[0]);
            refEntities.push_back(entities[1]);
        }
    }
    catch (const std::bad_alloc&)
    {
        ccLog::Error(tr("Not enough memory"));
        return;
    }

    if (alignedEntities.empty())
    {
        ccLog::Error(tr("No aligned entity selected"));
        return;
    }
    deselectAllOtherObject(true);

    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        ccLog::Error(tr("[PointPairRegistration] Failed to create dedicated 3D view!"));
        return;
    }

    //we disable all windows
    disableAllBut(win);

    if (!m_pprDlg)
    {
        m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this, this);
        connect(m_pprDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::showMessage, this, &MainWindow::updateStatusbarInformation);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::signalPreview, this, &MainWindow::onPairRegistrationSignalPreview, Qt::UniqueConnection);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::signalReset, this, &MainWindow::onPairRegistrationSignalReset, Qt::UniqueConnection);
        connect(m_pprDlg, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
        registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
    }
    m_pprDlg->setCoordinateConversionStyle(false);

    if (!m_pprDlg->init(win, alignedEntities, &refEntities))
    {
        deactivateRegisterPointPairTool(false);
    }

    freezeUI(true);
    setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
    if (!m_pprDlg->start())
    {
        deactivateRegisterPointPairTool(false);
    }
    else
    {
        m_isClippingBoxCanCombination = true;
        updateOverlayDialogsPlacement();
    }
    updateStatusbarInformation(QApplication::translate("importantinformationtips", "Target identification should be carried out in intensity display mode.", nullptr));
}

void MainWindow::deactivateRegisterPointPairTool(bool state)
{
    m_pViewerAction->setEnabled(true);

	m_isClippingBoxCanCombination = false;
	if (m_pprDlg)
	{
		m_pprDlg->clear();
		setGlobalZoom();
	}

	if (m_pavDlg)
		m_pavDlg->clear();

	if (!m_isClippingBoxOpen)
	{
        //we enable all GL windows
        qDebug() << "SectionAnalysisDlgclose30";
        enableAll();
        qDebug() << "SectionAnalysisDlgclose31";
        freezeUI(false);
        qDebug() << "SectionAnalysisDlgclose32";
        updateUI();
        qDebug() << "SectionAnalysisDlgclose33";
        m_UI->dbTreeView->update();
	}
	else
	{
		if (m_clipTool)
		{
			m_clipTool->setCurrentButtonStatus(false);
			m_clipTool->closeDialog();
		}
	}
}

void MainWindow::slotExitFunction(bool)
{
    enableAll();
    freezeUI(false);
    updateUIWithSelection();
    updateMenus();
}

void MainWindow::activateSectionExtractionMode()
{
	if (!haveSelection())
		return;

	if (!m_seTool)
	{
		m_seTool = new ccSectionExtractionTool(this);
		connect(m_seTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSectionExtractionMode);
        connect(m_seTool, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
	}

	//add clouds
	ccGLWindow* firstDisplay = nullptr;
	{
		unsigned validCount = 0;
		for (ccHObject *entity : getSelectedEntities())
		{
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(entity)))
				{
					if (!firstDisplay && entity->getDisplay())
					{
						firstDisplay = static_cast<ccGLWindow*>(entity->getDisplay());
					}

					++validCount;
				}
			}
		}

		if (validCount == 0)
		{
			ccConsole::Error(tr("No cloud in selection!"));
			return;
		}
	}

	//deselect all entities
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	ccGLWindow* win = new3DView(false);
	if (!win)
	{
		ccLog::Error(tr("[SectionExtraction] Failed to create dedicated 3D view!"));
		return;
	}

	if (firstDisplay && firstDisplay->getGlFilter())
	{
		win->setGlFilter(firstDisplay->getGlFilter()->clone());
	}
	m_seTool->linkWith(win);

	freezeUI(true);
	m_UI->toolBarView->setDisabled(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_seTool->start())
		deactivateSectionExtractionMode(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateSectionExtractionMode(bool state)
{
	if (m_seTool)
		m_seTool->removeAllEntities();
    m_pViewerAction->setEnabled(true);

	//we enable all GL windows
	enableAll();

	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
	if (!subWindowList.isEmpty())
		subWindowList[0]->showMaximized();

	freezeUI(false);
	m_UI->toolBarView->setDisabled(false);

	updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->redraw();
}



void MainWindow::activateSegmentationMode()
{
	if (!haveSelection())
		return;
    m_pViewerAction->setEnabled(false);
    changeTo2dView();
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
        return;

	setUseOrthographic(true);
	deselectAllOtherObject(true);
	if (!m_gsTool)
	{
		m_gsTool = new ccGraphicalSegmentationTool(this, m_pickingHub);
		connect(m_gsTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSegmentationMode);

		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
	}
	m_gsTool->setCuttingMode(ccGraphicalSegmentationTool::POINTCLOUDCUTTING);
	m_gsTool->linkWith(win);

	for (ccHObject *entity : getSelectedEntities())
	{
		m_gsTool->addEntity(entity);
	}

	if (m_gsTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error(tr("No segmentable entity in active window!"));
		return;
	}

	freezeUI(true);
	if (!m_isClippingBoxOpen)
	{
		setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
	}
	m_UI->toolBarView->setDisabled(false);

	//we disable all other windows
	disableAllBut(win);

	if (!m_gsTool->start())
	{
		deactivateSegmentationMode(false);
	}
	else
	{
		m_isClippingBoxCanCombination = true;
		updateOverlayDialogsPlacement();
		if (m_isClippingBoxOpen)
		{
			m_clipTool->setCurrentButtonStatus(true);
			m_clipTool->setCombineOpen();
		}
	}
	//setGlobalZoom();
}


void MainWindow::deactivateSegmentationMode(bool state)
{
    if (!m_gsTool)
    {
        assert(false);
        return;
    }

    ccHObject::Container EnterObj;
    ccHObject::Container result;
	bool deleteHiddenParts = false;

    //shall we apply segmentation?
    if (state)
    {

        m_gsTool->applySegmentation(this, result, EnterObj);

		if (m_ccRoot)
        {
            if (result.size() > 0)
            {
                m_ccRoot->selectEntities(result);
                m_selectedEntities = result;
            }
            else if(EnterObj.size() > 0)
            {
                 m_ccRoot->selectEntities(EnterObj);
                 m_selectedEntities = EnterObj;
            }
            else
            {
                if (m_isClippingBoxOpen)
                {
                    m_ccRoot->deleteSelectedEntities();
                    m_selectedEntities.clear();
                    m_clipTool->closeDialog();
                    m_isClippingBoxOpen = false;
                }
            }
        }
	}
	else
	{
		m_gsTool->removeAllEntities(true);
	}
	m_isClippingBoxCanCombination = false;
	//we enable all GL windows
	if (!m_isClippingBoxOpen)
	{
        m_pViewerAction->setEnabled(true);

		enableAll();

        freezeUI(false);

        updateUI();
    }
    else
    {
        ccBBox extents;
        ccGLMatrix transformation;

        if (m_clipTool)
        {
            if (state)
            {
                m_clipTool->m_bControlDialogNotClose = true;
                m_clipTool->closeDialog();
                activateClippingBoxMode();
            }
            m_clipTool->m_bCloseCombinationCropButton = true;
            m_clipTool->setCurrentButtonStatus(false);
        }
		m_UI->actionSegment->setEnabled(true);
	}
	ccGLWindow* win = getActiveGLWindow();
	setUseOrthographic(false);
	if (win)
	{
		win->redraw();
	}

}

void MainWindow::activateTracePolylineMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_tplTool)
	{
		m_tplTool = new ccTracePolylineTool(m_pickingHub, this);
		connect(m_tplTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTracePolylineMode);
        connect(m_tplTool, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
		registerOverlayDialog(m_tplTool, Qt::TopRightCorner);
	}

	m_tplTool->linkWith(win);

	freezeUI(true);
	m_UI->toolBarView->setDisabled(false);

	//we disable all other windows
	disableAllBut(win);

	if (!m_tplTool->start())
		deactivateTracePolylineMode(false);
	else
		updateOverlayDialogsPlacement();
}

void MainWindow::deactivateTracePolylineMode(bool)
{
    m_pViewerAction->setEnabled(true);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();

	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
}

void MainWindow::activatePointListPickingMode()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	//there should be only one point cloud in current selection!
	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select one and only one entity!"));
		return;
	}

	ccHObject* entity = m_selectedEntities.front();
	if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
	{
		ccConsole::Error(tr("Select a cloud or a mesh"));
		return;
	}

	if (!entity->isVisible() || !entity->isEnabled())
	{
		ccConsole::Error(tr("Entity must be visible!"));
		return;
	}

	if (!m_plpDlg)
	{
		m_plpDlg = new ccPointListPickingDlg(m_pickingHub, this);
		connect(m_plpDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivatePointListPickingMode);
        connect(m_plpDlg, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });

		registerOverlayDialog(m_plpDlg, Qt::TopRightCorner);
	}

	//DGM: we must update marker size spin box value (as it may have changed by the user with the "display dialog")
	m_plpDlg->markerSizeSpinBox->setValue(win->getDisplayParameters().labelMarkerSize);

	m_plpDlg->linkWith(win);
	m_plpDlg->linkWithEntity(entity);

	freezeUI(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_plpDlg->start())
		deactivatePointListPickingMode(false);
	else
		updateOverlayDialogsPlacement();
}


void MainWindow::deactivatePointListPickingMode(bool state)
{
	if (m_plpDlg)
	{
		m_plpDlg->linkWithEntity(nullptr);
	}
    m_pViewerAction->setEnabled(true);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::activatePointPickingMode()
{
	if (m_selectedEntities.size()== 0)
		return;
    m_pViewerAction->setEnabled(false);
    changeTo2dView();
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        return;
    }

	int point2dnum = 0;
	int point3dnum = 0;
	for (auto curselectedobj : m_selectedEntities)
	{
		if (curselectedobj->getMetaData("is2dSurface").toBool())
		{
			point2dnum++;
		}
		else
		{
			point3dnum++;
		}
	}
	if (point2dnum > 0 && point3dnum > 0)
	{
		CS::Widgets::FramelessMessageBox::critical(s_instance, tr("Error"), tr("2D point cloud cannot be measured with 3D point cloud!"));
		return;
	}
	int pointSize = m_selectedEntities.size();
	if (pointSize > 1)
	{
		ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0]);
		if (cloud1)
		{
			CCVector3d Pshift = cloud1->getGlobalShift();
			for (int i = 1; i <= pointSize - 1; i++)
			{
				ccGenericPointCloud* cloud2 = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[i]);
				if (cloud2)
				{
					CCVector3d Pshift2 = cloud2->getGlobalShift();
					if ((Pshift2 - Pshift).norm() > 1)
					{
						CS::Widgets::FramelessMessageBox::critical(this, tr("Prompt"), tr("Because the selection point cloud data offset values are different, the measurement cannot be performed!"));
						return;
					}
				}
			}
		}
	}
    //setUseOrthographic(true);
	deselectAllOtherObject(true);

	if (!m_ppDlg)
	{
		m_ppDlg = new ccPointPropertiesDlg(m_pickingHub, this);
		connect(m_ppDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivatePointPickingMode);
		connect(m_ppDlg, &ccPointPropertiesDlg::newLabel, this, &MainWindow::handleNewLabel);	
        connect(m_ppDlg, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });

		registerOverlayDialog(m_ppDlg, Qt::TopRightCorner);
	}
	if (m_selectedEntities[0]->getMetaData("is2dSurface").toBool())
	{
		m_ppDlg->setCurrentCloudTypeMode(ccPointPropertiesDlg::POINT2DMODE);
	}
	else
	{
		m_ppDlg->setCurrentCloudTypeMode(ccPointPropertiesDlg::POINT3DMODE);
	}
	m_ppDlg->setCurrentObj(m_selectedEntities);
	m_ppDlg->linkWith(win);

	freezeUI(true);

	//we disable all other windows
	disableAllBut(win);

	if (!m_ppDlg->start())
		deactivatePointPickingMode(false);
	else
		updateOverlayDialogsPlacement();



}

void MainWindow::deactivatePointPickingMode(bool state)
{
    m_pViewerAction->setEnabled(true);

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
	//setUseOrthographic(false);
}

void MainWindow::deactivatePointPickingPositionMode(bool)
{
	/*if (m_ppp)
	{
		m_ppp->linkWithEntity(nullptr);
	}*/

	//we enable all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::activateClippingBoxMode()
{
	if (!haveSelection())
	{
		return;
	}
    m_pViewerAction->setEnabled(false);
    changeTo2dView();

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}

	if (!m_clipTool)
	{
		m_clipTool = new ccClippingBoxTool(this);
		connect(m_clipTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateClippingBoxMode);
		connect(m_clipTool, &ccClippingBoxTool::updateSegmentState, this, &MainWindow::updateSegmentStateSlot);
        connect(m_clipTool, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
        win->update();
	}
    m_clipTool->m_selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectEntity' will change the set of currently selected entities!

	if (m_isClippingBoxCanCombination)
	{
		ccGui::ParamStruct param = ccGui::Parameters();
		param.m_openShiftMeasure = false;
		ccGui::Set(param);
		m_clipTool->setCurrentButtonStatus(true);
	}
    if (!m_isClippingBoxCanCombination)
    {
        MainWindow::TheInstance()->deselectAllOtherObject(false);
    }
   
	m_clipTool->linkWith(win);



     //[!]打开裁剪盒时关闭当前选择点云包围盒显示
    for (int i = 0; i < m_clipTool->m_selectedEntities.size(); i++)
    {
        if (m_clipTool->m_selectedEntities.at(i)->isKindOf(CC_TYPES::MESH))
        {
            if (m_clipTool->m_selectedEntities.at(i)->getParent())
            {
                m_clipTool->m_selectedEntities.at(i)->getParent()->m_displayBoundingBox = false;
                m_clipTool->m_selectedEntities.at(i)->getParent()->setVisible(false);
            }
        }
        m_clipTool->m_selectedEntities.at(i)->m_displayBoundingBox = false;
        m_clipTool->addAssociatedEntity(m_clipTool->m_selectedEntities.at(i));
    }

    if (m_clipTool->getNumberOfAssociatedEntity() == 0)
    {
        m_clipTool->close();
        return;
    }

    if (m_pprDlg)
    {
        m_pprDlg->m_bCurrentModelCombination = true;
    }
	if (m_clipTool->start())
	{
        if (MainWindow::TheInstance()->getActiveGLWindow())
        {
            MainWindow::TheInstance()->getActiveGLWindow()->zoomGlobal(true);
        }
		registerOverlayDialog(m_clipTool, Qt::TopLeftCorner);
		freezeUI(true);
		updateOverlayDialogsPlacement();
		//deactivate all other GL windows
		disableAllBut(win);
		m_isClippingBoxOpen = true;
        m_clipTool->slotSetResetToolButtonEnabel(false);
        MainWindow::TheInstance()->setGlobalZoom();
        updateSegmentStateSlot(true);
	}
	else
	{
        CS::Widgets::FramelessMessageBox::warning(this, "Error!", "Unexpected error!");
	}
	//[!]非组合情况下开启shift测量
	if (!m_isClippingBoxCanCombination)
	{
		ccGui::ParamStruct param = ccGui::Parameters();
		param.m_openShiftMeasure = true;
		ccGui::Set(param);
	}
	m_clipTool->adjustSize();
}

void MainWindow::updateSegmentStateSlot(bool isopen)
{
	if (!m_isClippingBoxCanCombination)
	{
		m_UI->actionSegment->setEnabled(isopen);
	}	
}

void MainWindow::deactivateClippingBoxMode(bool state)
{
	//[!]关闭裁剪盒时显示包围盒
	for (int i = 0; i < getSelectedEntities().size(); i++)
	{
		if (getSelectedEntities().at(i)->getParent())
		{
			ccHObject *pParent = getSelectedEntities().at(i)->getParent();
			for (int j = 0; j < pParent->getChildrenNumber(); j++)
			{
				if (pParent->getChild(j)->isKindOf(CC_TYPES::MESH))
				{
					pParent->m_displayBoundingBox = true;
                    pParent->setVisible(true);
				}
				pParent->getChild(j)->m_displayBoundingBox = true;
			}
		}
	}
    if (m_pprDlg)
    {
        m_pprDlg->m_bCurrentModelCombination = false;
        m_pprDlg->setPointPairRegistrationCurrentButtonstate();
    }
	m_isClippingBoxOpen = false;
	if (m_isClippingBoxCanCombination)
	{
		setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
	}
	else
	{
        m_pViewerAction->setEnabled(true);

		//we reactivate all GL windows
		enableAll();

		freezeUI(false);
		updateUI();
	}
	m_clipTool->setCurrentButtonStatus(false);
}

//void MainWindow::deactivateMeasureVolumeMode(bool state)
//{
//	enableAll();
//
//	//freezeUI(false);
//
//	//updateUI();
//}

//void MainWindow::activeVolumeMeasureMode()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		return;
//	}
//
//	if (m_ccRoot)
//	{
//		//m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
//		if (!m_ccRoot->hasChildren())
//			return;
//	}
//
//	if (!m_volumeTool)
//	{
//		m_volumeTool = new ccVolumeMeasureTool(this);
//		connect(m_volumeTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateMeasureVolumeMode);
//	}
//
//	if (!m_volumeTool->linkWith(win))
//		return;
//
//	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectEntity' will change the set of currently selected entities!
//	for (ccHObject *entity : selectedEntities)
//	{
//		if (m_volumeTool->addAssociatedEntity(entity))
//		{
//			//automatically deselect the entity (to avoid seeing its bounding box ;)
//			//m_ccRoot->unselectEntity(entity);
//			entity->setSelected(false);
//		}
//	}
//
//	if (m_volumeTool->getNumberOfAssociatedEntity() == 0)
//	{
//		//m_volumeTool->close();
//		m_volumeTool->stop(true);
//		return;
//	}
//
//	if (m_volumeTool->start())
//	{
//		registerOverlayDialog(m_volumeTool, Qt::TopRightCorner);//改行不能注释，原因：如果没有点击关闭测量按钮，直接关闭软件，会引起ccVolumeMeasureTool::stop函数崩溃 janson.yang2022.6.8
//		freezeUI(true);
//		updateOverlayDialogsPlacement();
//		//deactivate all other GL windows
//		disableAllBut(win);
//
//		if (ribbon)
//		{
//			ribbon->applicationButton()->setEnabled(false);
//			ribbon->setCategorysEnabled(false, "edit");
//			ribbon->setCategorysEnabled(true, "edit", "measure");
//		}
//	}
//	else
//	{
//		ccConsole::Error(tr("Unexpected error!")); //indeed...
//	}
//
//
//}

void MainWindow::activateTranslateRotateMode()
{
	if (!haveSelection())
		return;
    m_pViewerAction->setEnabled(false);
    changeTo2dView();

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	if (!m_transTool)
		m_transTool = new ccGraphicalTransformationTool(this);
	assert(m_transTool->getNumberOfValidEntities() == 0);
	m_transTool->linkWith(win);

	bool rejectedEntities = false;
	for (ccHObject *entity : getSelectedEntities())
	{
		entity->setEnabled(true);
		if (!m_transTool->addEntity(entity))
			rejectedEntities = true;

		if (entity->getParent())
		{
			while (entity->getParent()->getName() != "DB Tree")
			{
				entity = entity->getParent();
				if (!entity->getParent())
				{
					break;
				}
				entity->setEnabled(true);
			}

		}

	}

	if (m_transTool->getNumberOfValidEntities() == 0)
	{
		ccConsole::Error(tr("No entity eligible for manual transformation! (see console)"));
		return;
	}
	else if (rejectedEntities)
	{
		ccConsole::Error(tr("Some entities were ignored! (see console)"));
	}
	//try to activate "moving mode" in current GL window
	if (m_transTool->start())
	{
		connect(m_transTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTranslateRotateMode);
        connect(m_transTool, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
        m_transTool->show();
		registerOverlayDialog(m_transTool, Qt::TopRightCorner);
		freezeUI(true);
		updateOverlayDialogsPlacement();
		//deactivate all other GL windows
		disableAllBut(win);
	}
	else
	{
		ccConsole::Error(tr("Unexpected error!")); //indeed...
	}

}

void MainWindow::deactivateTranslateRotateMode(bool state)
{
	if (m_transTool)
	{
		//reselect previously selected entities!
		if (state && m_ccRoot)
		{
			const ccHObject& transformedSet = m_transTool->getValidEntities();
			try
			{
				ccHObject::Container transformedEntities;
				transformedEntities.resize(transformedSet.getChildrenNumber());
				for (unsigned i = 0; i < transformedSet.getChildrenNumber(); ++i)
				{
					transformedEntities[i] = transformedSet.getChild(i);
				}
				m_ccRoot->selectEntities(transformedEntities);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory (nothing to do)
			}
		}
		//m_transTool->close();

		m_transTool->deleteBottomDialog();
	}
    m_pViewerAction->setEnabled(true);

	//we reactivate all GL windows
	enableAll();

	freezeUI(false);

	updateUI();
}

void MainWindow::testFrameRate()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->startFrameRateTest();
}

void MainWindow::showDisplayOptions()
{
    //ccDisplayOptionsDlg displayOptionsDlg(this);
    //displayOptionsDlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Display Options", nullptr));
    //connect(&displayOptionsDlg, &ccDisplayOptionsDlg::aspectHasChanged, this, [=]() { redrawAll();	});

    //displayOptionsDlg.exec();

    //disconnect(&displayOptionsDlg);
	CS::Widgets::FramelessDialog outdlg(this);
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
	outdlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Background Color", nullptr));
	ccColor::Rgb currentcolor = ccGui::Parameters().backgroundCol;
	QColorDialog colordlg(QColor(currentcolor.r, currentcolor.g, currentcolor.b), this);
	connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
	colordlg.setOptions(QColorDialog::NoButtons);
	outdlg.SetContentHolder(&colordlg);
	if (outdlg.exec())
	{
		QColor newCol = colordlg.currentColor();
		if (!newCol.isValid())
			return;
		ccGui::ParamStruct m_parameters = ccGui::Parameters();
		m_parameters.backgroundCol = ccColor::FromQColor(newCol);
		m_parameters.drawBackgroundGradient = false;
		ccGui::Set(m_parameters);
		m_parameters.toPersistentSettings();
		redrawAll();
	}

}

void MainWindow::doActionpreferencesetting()
{
    //[!]获取到全局记录的点大小
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("OpenGL");
    int pointSize = settings.value("m_globalpointsize",1).toInt();
    settings.endGroup();

	ccGui::ParamStruct m_parameters = ccGui::Parameters();
	QColor LabelColor = QColor(m_parameters.labelBackgroundCol.r, m_parameters.labelBackgroundCol.g, m_parameters.labelBackgroundCol.b);
	QColor MarkerColor = QColor(m_parameters.labelMarkerCol.r, m_parameters.labelMarkerCol.g, m_parameters.labelMarkerCol.b);
	int transparency = m_parameters.labelOpacity;
	int Labelfontsize = m_parameters.labelFontSize;
	int Markersize = m_parameters.labelMarkerSize;
	int defaultfontsize = m_parameters.defaultFontSize;
	bool isdrawround = m_parameters.drawRoundedPoints;
	bool isdrawmiddlecross = m_parameters.displayCross;


	ccPreferencesettingsDlg dlg(this);
	dlg.setWindowTitle(QCoreApplication::translate("MainWindow", "Settings", nullptr));
	dlg.SetLabelColor(LabelColor);
	dlg.SetMarkerColor(MarkerColor);
	dlg.Settransparency(transparency);
	dlg.SetLabelfontsize(Labelfontsize);
	dlg.SetMarkersize(Markersize);
	dlg.SetPrompttextsize(defaultfontsize);
	dlg.Setdrawroundpoints(isdrawround);
	dlg.SetDisplaycrossmark(isdrawmiddlecross);
	dlg.Setlength(static_cast<int>(m_parameters.lengthunit));
	dlg.Setdiameter(static_cast<int>(m_parameters.diameterunit));
	dlg.Setangle(static_cast<int>(m_parameters.angleunit));
	dlg.Setarea(static_cast<int>(m_parameters.areaunit));
	dlg.Setvolume(static_cast<int>(m_parameters.volumnunit));
	dlg.Setdecimal(m_parameters.displayedNumPrecision);
	dlg.SetShowunitlabels(m_parameters.isshowUnitlabel);
    m_parameters.m_globalpointsize = pointSize;
    dlg.setSettingPointSize(m_parameters.m_globalpointsize);
	dlg.resetContrastBrightness(m_parameters.m_nBrightness, m_parameters.m_nContrast);
	if (!dlg.exec()){
		//[!].如果取消，回复之前的保存的值
		dlg.resetContrastBrightness(m_parameters.m_nBrightness, m_parameters.m_nContrast);
		return;
	}
	m_parameters.labelBackgroundCol = ccColor::FromQColora(dlg.GetLabelColor());
	m_parameters.labelMarkerCol = ccColor::FromQColora(dlg.GetMarkerColor());
	m_parameters.labelOpacity = dlg.Gettransparency();
	m_parameters.labelFontSize = dlg.GetLabelfontsize();
	m_parameters.labelMarkerSize = dlg.GetMarkersize();
	m_parameters.defaultFontSize = dlg.GetPrompttextsize();
	m_parameters.drawRoundedPoints = dlg.Getdrawroundpoints();
	m_parameters.displayCross = dlg.GetDisplaycrossmark();
	m_parameters.lengthunit = static_cast<ccGui::ParamStruct::Lengthparametertype>(dlg.Getlength());
	m_parameters.diameterunit = static_cast<ccGui::ParamStruct::Lengthparametertype>(dlg.Getdiameter());
	m_parameters.angleunit = static_cast<ccGui::ParamStruct::Angleparametertype>(dlg.Getangle());
	m_parameters.areaunit = static_cast<ccGui::ParamStruct::Areaparametertype>(dlg.Getarea());
	m_parameters.volumnunit = static_cast<ccGui::ParamStruct::Volumnparametertype>(dlg.Getvolume());
	m_parameters.displayedNumPrecision = dlg.Getdecimal();
	m_parameters.isshowUnitlabel = dlg.GetShowunitlabels();
	m_parameters.m_nBrightness = dlg.getRenderBrightnessValue();
	m_parameters.m_nContrast = dlg.getRenderContrastValue();
    m_parameters.m_globalpointsize = dlg.getSettingPointSize();
    ccGui::Set(m_parameters);
    m_parameters.toPersistentSettings();
    setGlobalPointSize(m_parameters.m_globalpointsize);
	redrawAll();
}

void MainWindow::doActionRenderToFile()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	ccRenderToFileDlg rtfDlg(win->glWidth(), win->glHeight(), this);

	if (rtfDlg.exec())
	{
		QApplication::processEvents();
		win->renderToFile(rtfDlg.getFilename(), rtfDlg.getZoom(), rtfDlg.dontScalePoints(), rtfDlg.renderOverlayItems());
	}
}

void MainWindow::doActionEditCamera()
{
	//current active MDI area
	QMdiSubWindow* qWin = m_mdiArea->activeSubWindow();
	if (!qWin)
		return;

	if (!m_cpeDlg)
	{
		m_cpeDlg = new ccCameraParamEditDlg(qWin, m_pickingHub);
		//m_cpeDlg->makeFrameless(); //does not work on linux

		connect(m_mdiArea, &FJDragableMdiArea::subWindowActivated,
			m_cpeDlg, qOverload<QMdiSubWindow*>(&ccCameraParamEditDlg::linkWith));

		registerOverlayDialog(m_cpeDlg, Qt::BottomLeftCorner);
	}

	m_cpeDlg->linkWith(qWin);
	m_cpeDlg->start();

	updateOverlayDialogsPlacement();
}

void MainWindow::doActionAdjustZoom()
{
	//current active MDI area
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	const ccViewportParameters& params = win->getViewportParameters();
	if (params.perspectiveView)
	{
		ccConsole::Error(tr("Orthographic mode only!"));
		return;
	}

	ccAdjustZoomDlg azDlg(win, this);

	if (!azDlg.exec())
		return;

	//apply new focal
	double focalDist = azDlg.getFocalDistance();
	win->setFocalDistance(focalDist);
	win->redraw();
}

static unsigned s_viewportIndex = 0;
void MainWindow::doActionSaveViewportAsCamera()
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
		return;

	cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport #%1").arg(++s_viewportIndex));
	viewportObject->setParameters(win->getViewportParameters());
	viewportObject->setDisplay(win);

	addToDB(viewportObject);
}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindow* win = nullptr;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject *entity = m_selectedEntities[i];

		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindow*>(entity->getDisplay());
		}

		if (win)
		{
			if (entity->getDisplay() == win)
			{
				tempGroup.addChild(entity, ccHObject::DP_NONE);
			}
			else if (entity->getDisplay() != nullptr)
			{
				ccLog::Error(tr("All selected entities must be displayed in the same 3D view!"));
				return;
			}
		}
	}

	if (tempGroup.getChildrenNumber() != 0)
	{
		ccBBox box = tempGroup.getDisplayBB_recursive(false, win);
		if (!box.isValid())
		{
			ccLog::Warning(tr("Selected entities have no valid bounding-box!"));
		}
		else
		{
			if (win != nullptr)
			{
				win->updateConstellationCenterAndZoom(&box);
			}
		}
	}

	refreshAll();
}

void MainWindow::exchangePerspectiveView()
{
    toggleActiveWindowCenteredPerspective();
}

void MainWindow::setGlobalZoom()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		if (win->windowTitle() == "2D View")
		{
			emit win->signalsZoomGloba2DviewWindow(true);
		}
		win->zoomGlobal(true);
	}
		
}

void MainWindow::SetViewLock()
{
	//if (m_selectedEntities.size() == 1)
	//{
	//	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	//	if (pc)
	//	{
	//		int sfIdx = 0;
	//		if (pc)
	//		{
	//			sfIdx = pc->getScalarFieldIndexByName("gpstime");
	//			if (sfIdx < 0)
	//			{
	//				return;
	//			}
	//			m_timeToTrackVec.clear();
	//			m_trackDirectionVec.clear();
	//			m_currentIndex = 0;
	//			CCCoreLib::ScalarField* classifSF = pc->getScalarField(sfIdx);
	//			int cloudSize = pc->size();
	//			std::map<double, CCVector3> trackpointmap;
	//			for (int i = 0; i < cloudSize; ++i)
	//			{
	//				const CCVector3* P = pc->getPoint(i);
	//				trackpointmap[classifSF->getValue(i)] = *P;
	//			}
	//			double starttime = -999999999;
	//			CCVector3 firstpos(-999999999, -999999999, -999999999);
	//			std::vector<CCVector3> trackVec;
	//			for (auto curdata : trackpointmap)
	//			{
	//				CCVector3 posdiff = curdata.second - firstpos;
	//				double diff = curdata.first - starttime;
	//				if (diff > 10 && posdiff.norm() > 2)
	//				{
	//					trackVec.push_back(curdata.second);
	//					starttime = curdata.first;
	//					firstpos = curdata.second;
	//				}					
	//			}
	//			int numPoints = 19;
	//			for (int i = 0;i < (trackVec.size()-2);i++)
	//			{
	//				CCVector3 startPos = trackVec[i];
	//				CCVector3 endPos = trackVec[i+1];
	//				CCVector3d startDir = (endPos - startPos).toDouble();
	//				CCVector3d endDir = (trackVec[i + 2] - endPos).toDouble();
	//				for (int i = 0; i < numPoints; ++i) {
	//					double t = static_cast<double>(i) / (numPoints - 1);

	//					// 线性插值位置
	//					CCVector3 interpolatedPosition = startPos * (1.0 - t) + endPos * t;

	//					// Slerp 插值方向
	//					startDir.normalize();
	//					endDir.normalize();
	//					double dotProduct = CCVector3d::vdotd(startDir.u, endDir.u);
	//					double theta = std::acos(dotProduct) * t;
	//					CCVector3d relativeVec = (endDir - startDir * dotProduct);
	//					relativeVec.normalize();
	//					CCVector3d interpolatedDirection = startDir * std::cos(theta) + relativeVec * std::sin(theta);
	//					m_timeToTrackVec.push_back(interpolatedPosition);
	//					m_trackDirectionVec.push_back(interpolatedDirection);
	//				}
	//			}
	//			m_tractChangeTimer.start(10);
	//		}
	//	}
	//}
	//else
	//{
	//	m_tractChangeTimer.stop();
	//}
	//return;



	static bool checked = false;
	checked = !checked;
	if (checked)
	{
		setActionIcon(m_UI->actionViewlock, "actionViewlockClicked", "actionViewlockClicked", "actionViewlockClicked");
		m_UI->actionViewlock->setText(QCoreApplication::translate("MainWindow", "Unlock the view", nullptr));
		m_UI->actionViewlock->setToolTip(QCoreApplication::translate("MainWindow", "Unlock the view", nullptr));
	}
	else
	{
		setActionIcon(m_UI->actionViewlock, "actionViewlock", "actionViewHover", "actionViewlock");
		m_UI->actionViewlock->setText(QCoreApplication::translate("MainWindow", "Lock the view", nullptr));
		m_UI->actionViewlock->setToolTip(QCoreApplication::translate("MainWindow", "Lock the view", nullptr));
	}
	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	if (!windows.isEmpty())
	{
		for (QMdiSubWindow *window : windows)
		{
			ccGLWindow *child = GLWindowFromWidget(window->widget());
			if (child)
			{
				child->SetViewLock(checked);
			}
		}
	}
}

void MainWindow::setPivotAlwaysOn()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindow::PIVOT_ALWAYS_SHOW);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotAlwaysOn->icon());
	}
}

void MainWindow::setPivotRotationOnly()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindow::PIVOT_SHOW_ON_MOVE);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotRotationOnly->icon());
	}
}

void MainWindow::setPivotOff()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setPivotVisibility(ccGLWindow::PIVOT_HIDE);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotOff->icon());
	}
}

void MainWindow::setOrthoView(ccGLWindow* win)
{
	if (win)
	{
		if (!checkStereoMode(win))
		{
			return;
		}
		win->setPerspectiveState(false, true);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetOrthoView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw/*=true*/)
{
	if (win)
	{
		win->setPerspectiveState(true, true);
		if (autoRedraw)
			win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetCenteredPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(true);
	}
}

void MainWindow::setViewerPerspectiveView(ccGLWindow* win)
{
	if (win)
	{
		win->setPerspectiveState(true, false);
		win->redraw();

		//update pop-up menu 'top' icon
		if (m_viewModePopupButton)
			m_viewModePopupButton->setIcon(m_UI->actionSetViewerPerspectiveView->icon());
		if (m_pivotVisibilityPopupButton)
			m_pivotVisibilityPopupButton->setEnabled(false);
	}
}

void MainWindow::enablePickingOperation(ccGLWindow* win, QString message)
{
	if (!win)
	{
		assert(false);
		return;
	}

	assert(m_pickingHub);
	if (!m_pickingHub->addListener(this))
	{
		ccLog::Error(tr("Can't start the picking mechanism (another tool is already using it)"));
		return;
	}

	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
	//if (m_pprDlg)
	//	m_pprDlg->pause(true);

	s_pickingWindow = win;
	win->displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, true, 24 * 3600);
	win->redraw(true, false);

	freezeUI(true);
}

void MainWindow::cancelPreviousPickingOperation(bool aborted)
{
	if (!s_pickingWindow)
		return;

	switch (s_currentPickingOperation)
	{
	case PICKING_ROTATION_CENTER:
		//nothing to do
		break;
	case PICKING_LEVEL_POINTS:
		if (s_levelMarkersCloud)
		{
			s_pickingWindow->removeFromOwnDB(s_levelMarkersCloud);
			delete s_levelMarkersCloud;
			s_levelMarkersCloud = nullptr;
		}
		break;
	default:
		assert(false);
		break;
	}

	if (aborted)
	{
		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
//		s_pickingWindow->displayNewMessage(tr("Picking operation aborted"), ccGLWindow::LOWER_LEFT_MESSAGE);
	}
	s_pickingWindow->redraw(false);

	//specific case: we allow the 'point-pair based alignment' tool to process the picked point!
	if (m_pprDlg)
		m_pprDlg->pause(false);

	if (m_pavDlg)
		m_pavDlg->pause(false);

	freezeUI(false);

	m_pickingHub->removeListener(this);

	s_pickingWindow = nullptr;
	s_currentPickingOperation = NO_PICKING_OPERATION;
	//setActionIcon(m_UI->actionPickRotationCenter, "actionPickRotationCenter", "actionPickRotationCenterClicked", "actionPickRotationCenter");
	ccGui::ParamStruct m_parameters = ccGui::Parameters();
	m_parameters.m_bShowCapacityPoint = false;
	ccGui::Set(m_parameters);
	//m_UI->actionPickRotationCenter->setText(QCoreApplication::translate("MainWindow", "Pick rotation center", nullptr));
	//m_UI->actionPickRotationCenter->setToolTip(QCoreApplication::translate("MainWindow", "Pick rotation center", nullptr));

}

void MainWindow::onItemPicked(const PickedItem& pi)
{
	if (!s_pickingWindow || !m_pickingHub)
	{
		return;
	}

	if (!pi.entity)
	{
		return;
	}

	if (m_pickingHub->activeWindow() != s_pickingWindow)
	{
		ccLog::Warning(tr("The point was picked in the wrong window"));
		return;
	}

	CCVector3 pickedPoint = pi.P3D;
	switch (s_currentPickingOperation)
	{
	case PICKING_LEVEL_POINTS:
	{
		//we only accept points picked on the right entity!
		//if (obj != s_levelEntity)
		//{
		//	ccLog::Warning(tr("[Level] Only points picked on '%1' are considered!").arg(s_levelEntity->getName()));
		//	return;
		//}

		if (!s_levelMarkersCloud)
		{
			assert(false);
			cancelPreviousPickingOperation(true);
		}

		for (unsigned i = 0; i < s_levelMarkersCloud->size(); ++i)
		{
			const CCVector3* P = s_levelMarkersCloud->getPoint(i);
			if ((pickedPoint - *P).norm() < 1.0e-6)
			{
				ccLog::Warning(tr("[Level] Point is too close from the others!"));
				return;
			}
		}

		//add the corresponding marker
		s_levelMarkersCloud->addPoint(pickedPoint);
		unsigned markerCount = s_levelMarkersCloud->size();
		cc2DLabel* label = new cc2DLabel();
		label->addPickedPoint(s_levelMarkersCloud, markerCount - 1);
		label->setName(QString("P#%1").arg(markerCount));
		label->setDisplayedIn2D(false);
		label->setDisplay(s_pickingWindow);
		label->setVisible(true);
		s_levelMarkersCloud->addChild(label);
		s_pickingWindow->redraw();

		if (markerCount == 3)
		{
			//we have enough points!
			const CCVector3* A = s_levelMarkersCloud->getPoint(0);
			const CCVector3* B = s_levelMarkersCloud->getPoint(1);
			const CCVector3* C = s_levelMarkersCloud->getPoint(2);
			CCVector3 X = *B - *A;
			CCVector3 Y = *C - *A;
			CCVector3 Z = X.cross(Y);
			//we choose 'Z' so that it points 'upward' relatively to the camera (assuming the user will be looking from the top)
			CCVector3d viewDir = s_pickingWindow->getViewportParameters().getViewDir();
			if (Z.toDouble().dot(viewDir) > 0)
			{
				Z = -Z;
			}
			Y = Z.cross(X);
			X.normalize();
			Y.normalize();
			Z.normalize();

			ccGLMatrixd trans;
			double* mat = trans.data();
			mat[0] = X.x; mat[4] = X.y; mat[8] = X.z; mat[12] = 0;
			mat[1] = Y.x; mat[5] = Y.y; mat[9] = Y.z; mat[13] = 0;
			mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0;
			mat[3] = 0; mat[7] = 0; mat[11] = 0; mat[15] = 1;

			CCVector3d T = -A->toDouble();
			trans.apply(T);
			T += *A;
			trans.setTranslation(T);

			assert(haveOneSelection() && m_selectedEntities.front() == s_levelEntity);
			applyTransformation(trans);

			//clear message
			s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
			s_pickingWindow->setView(CC_TOP_VIEW);
		}
		else
		{
			//we need more points!
			return;
		}
	}
	//we use the next 'case' entry (PICKING_ROTATION_CENTER) to redefine the rotation center as well!
	assert(s_levelMarkersCloud && s_levelMarkersCloud->size() != 0);
	pickedPoint = *s_levelMarkersCloud->getPoint(0);
	//break;

	case PICKING_ROTATION_CENTER:
	{
		CCVector3d newPivot = pickedPoint;
		//specific case: transformation tool is enabled
		if (m_transTool && m_transTool->started())
		{
			m_transTool->setRotationCenter(newPivot);
			const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
			s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
			s_pickingWindow->displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center for interactive transformation")
				.arg(pickedPoint.x, 0, 'f', precision)
				.arg(pickedPoint.y, 0, 'f', precision)
				.arg(pickedPoint.z, 0, 'f', precision),
				ccGLWindow::LOWER_LEFT_MESSAGE, true);
		}
		else
		{
			const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
			if (!params.perspectiveView || params.objectCenteredView)
			{
				//apply current GL transformation (if any)
				pi.entity->getGLTransformation().apply(newPivot);
				s_pickingWindow->setPivotPoint(newPivot, true, true);
			}
		}
		//s_pickingWindow->redraw(); //already called by 'cancelPreviousPickingOperation' (see below)
	}
	break;

	default:
		assert(false);
		break;
	}

	cancelPreviousPickingOperation(false);
}

void MainWindow::doLevel()
{
	//picking operation already in progress
	if (s_pickingWindow)
	{
		if (s_currentPickingOperation == PICKING_LEVEL_POINTS)
		{
			cancelPreviousPickingOperation(true);
		}
		else
		{
			ccConsole::Error(tr("Stop the other picking operation first!"));
		}
		return;
	}

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccConsole::Error(tr("No active 3D view!"));
		return;
	}

	if (!haveOneSelection())
	{
		ccConsole::Error(tr("Select an entity!"));
		return;
	}

	//create markers cloud
	assert(!s_levelMarkersCloud);
	{
		s_levelMarkersCloud = new ccPointCloud("Level points");
		if (!s_levelMarkersCloud->reserve(3))
		{
			ccConsole::Error(tr("Not enough memory!"));
			return;
		}
		win->addToOwnDB(s_levelMarkersCloud);
	}

	s_levelEntity = m_selectedEntities.front();
	s_levelLabels.clear();
	s_currentPickingOperation = PICKING_LEVEL_POINTS;

	enablePickingOperation(win, tr("Pick three points on the floor plane (click the Level button or press Escape to cancel)"));
}

void MainWindow::doPickRotationCenter()
{
	ccGLWindow* win = qobject_cast<ccGLWindow*>(m_mdiArea->get3dWindow()->widget());
	//ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccConsole::Error(tr("No active 3D view!"));
		return;
	}
	static bool isPickRotationState = false;
	isPickRotationState = !isPickRotationState;
	if (isPickRotationState)
	{
		bool objectCentered = true;
		bool perspectiveEnabled = win->getPerspectiveState(objectCentered);
		if (perspectiveEnabled && !objectCentered)
		{
			ccLog::Error(tr("Perspective mode is viewer-centered: can't use a point as rotation center!"));
			return;
		}
		ccGLWindow* pActiveWidget = getActiveGLWindow();
		if (pActiveWidget)
		{
			if (pActiveWidget->getPickingMode() == ccGLWindow::PICKING_MODE::NOTE_PICKING)
			{
				actionNotes->trigger();
			}
		}
		s_currentPickingOperation = PICKING_ROTATION_CENTER;
		enablePickingOperation(win, QApplication::translate("MainWindowUI", "Pick a point to be used as rotation center (click on icon again to cancel)"));
		setActionIcon(m_UI->actionPickRotationCenter, "actionPickRotationCenterClicked", "actionPickRotationCenterClicked", "actionPickRotationCenterClicked");
		ccGui::ParamStruct m_parameters = ccGui::Parameters();
		m_parameters.m_bShowCapacityPoint = true;
		ccGui::Set(m_parameters);
		m_UI->actionPickRotationCenter->setText(QCoreApplication::translate("MainWindow", "Auto-pick rotation center", nullptr));
		m_UI->actionPickRotationCenter->setToolTip(QCoreApplication::translate("MainWindow", "Auto-pick rotation center", nullptr));
		win->setAutoPickPivotAtCenter(false);
	}
	else
	{
		if (s_pickingWindow)
		{
			if (s_currentPickingOperation == PICKING_ROTATION_CENTER)
			{
				cancelPreviousPickingOperation(true);
				ccGui::ParamStruct m_parameters = ccGui::Parameters();
				m_parameters.m_bShowCapacityPoint = false;
				ccGui::Set(m_parameters);
			}
			else
			{
				ccConsole::Error(tr("Stop the other picking operation first!"));
			}
		}
		setActionIcon(m_UI->actionPickRotationCenter, "actionPickRotationCenter", "actionPickRotationCenterClicked", "actionPickRotationCenter");
		m_UI->actionPickRotationCenter->setText(QCoreApplication::translate("MainWindow", "Pick rotation center", nullptr));
		m_UI->actionPickRotationCenter->setToolTip(QCoreApplication::translate("MainWindow", "Pick rotation center", nullptr));
		win->setAutoPickPivotAtCenter(true);
	}

}

ccPointCloud* MainWindow::askUserToSelectACloud(ccHObject* defaultCloudEntity/*=nullptr*/, QString inviteMessage/*=QString()*/)
{
	ccHObject::Container clouds;
	m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	if (clouds.empty())
	{
		ccConsole::Error(tr("No cloud in database!"));
		return nullptr;
	}
	//default selected index
	int selectedIndex = 0;
	if (defaultCloudEntity)
	{
		for (size_t i = 1; i < clouds.size(); ++i)
		{
			if (clouds[i] == defaultCloudEntity)
			{
				selectedIndex = static_cast<int>(i);
				break;
			}
		}
	}
	//ask the user to choose a cloud
	{
		selectedIndex = ccItemSelectionDlg::SelectEntity(clouds, selectedIndex, this, inviteMessage);
		if (selectedIndex < 0)
			return nullptr;
	}

	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < clouds.size());
	return ccHObjectCaster::ToPointCloud(clouds[selectedIndex]);
}

void MainWindow::toggleSelectedEntitiesProperty(ccEntityAction::TOGGLE_PROPERTY property)
{
	if (!ccEntityAction::toggleProperty(m_selectedEntities, property))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::clearSelectedEntitiesProperty(ccEntityAction::CLEAR_PROPERTY property)
{
	if (!ccEntityAction::clearProperty(m_selectedEntities, property, this))
	{
		return;
	}

	refreshAll();
	updateUI();
}

void MainWindow::setView(CC_VIEW_ORIENTATION view)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setView(view);
	}
}

void MainWindow::spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel)
{
	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle(tr("Histogram"));

	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle(title);
		histogram->fromBinArray(histoValues, minVal, maxVal);
		histogram->setAxisLabels(xAxisLabel, tr("Count"));
		histogram->refresh();
	}

	hDlg->show();
}

void MainWindow::showSelectedEntitiesHistogram()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		//for "real" point clouds only
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			//we display the histogram of the current scalar field
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
			if (sf)
			{
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
				hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
				hDlg->setWindowTitle(tr("Histogram [%1]").arg(cloud->getName()));

				ccHistogramWindow* histogram = hDlg->window();
				{
					unsigned numberOfPoints = cloud->size();
					unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
					//we take the 'nearest' multiple of 4
					numberOfClasses &= (~3);
					numberOfClasses = std::max<unsigned>(4, numberOfClasses);
					numberOfClasses = std::min<unsigned>(256, numberOfClasses);

					histogram->setTitle(tr("%1 (%2 values) ").arg(sf->getName()).arg(numberOfPoints));
					bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
					histogram->fromSF(sf, numberOfClasses, true, showNaNValuesInGrey);
					histogram->setAxisLabels(sf->getName(), tr("Count"));
					histogram->refresh();
				}
				hDlg->show();
			}
		}
	}
}

void MainWindow::doActionCrop()
{
	//find candidates
	std::vector<ccHObject*> candidates;
	ccBBox baseBB;
	{
		const ccHObject::Container& selectedEntities = getSelectedEntities();
		for (ccHObject *entity : selectedEntities)
		{
			if (entity->isA(CC_TYPES::POINT_CLOUD)
				|| entity->isKindOf(CC_TYPES::MESH))
			{
				candidates.push_back(entity);
				baseBB += entity->getOwnBB();
			}
		}
	}

	if (candidates.empty())
	{
		ccConsole::Warning(tr("[Crop] No eligible candidate found!"));
		return;
	}

	ccBoundingBoxEditorDlg bbeDlg(false, false, this);
	bbeDlg.setBaseBBox(baseBB, false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Crop");

	if (!bbeDlg.exec())
	{
		//process cancelled by user
		return;
	}

	//deselect all entities
	if (m_ccRoot)
	{
		m_ccRoot->unselectAllEntities();
	}

	//cropping box
	ccBBox box = bbeDlg.getBox();

	//process cloud/meshes
	bool errors = false;
	bool successes = false;
	{
		for (ccHObject *entity : candidates)
		{
			ccHObject* croppedEnt = ccCropTool::Crop(entity, box, true);
			if (croppedEnt)
			{
				croppedEnt->setName(entity->getName() + QString(".cropped"));
				croppedEnt->setDisplay(entity->getDisplay());
				croppedEnt->prepareDisplayForRefresh();
				if (entity->getParent())
					entity->getParent()->addChild(croppedEnt);
				entity->setEnabled(false);
				addToDB(croppedEnt);
				//select output entity
				m_ccRoot->selectEntity(croppedEnt, true);
				successes = true;
			}
			else
			{
				errors = true;
			}
		}
	}

	if (successes)
		ccLog::Warning(tr("[Crop] Selected entities have been hidden"));
	if (errors)
		ccLog::Error(tr("Error(s) occurred! See the Console"));

	refreshAll();
	updateUI();
}

void MainWindow::doActionClone()
{
	ccHObject* lastClone = nullptr;

	for (ccHObject *entity : getSelectedEntities())
	{
		ccHObject* clone = nullptr;

		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clone = ccHObjectCaster::ToGenericPointCloud(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning cloud %1").arg(entity->getName()));
			}
		}
		else if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			clone = static_cast<ccGenericPrimitive*>(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning primitive %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::MESH))
		{
			clone = ccHObjectCaster::ToMesh(entity)->cloneMesh();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning mesh %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::POLY_LINE))
		{
			clone = ccHObjectCaster::ToPolyline(entity)->clone();
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning polyline %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = ccHObjectCaster::ToFacet(entity);
			clone = (facet ? facet->clone() : nullptr);
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning facet %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::CAMERA_SENSOR))
		{
			ccCameraSensor* camera = ccHObjectCaster::ToCameraSensor(entity);
			if (camera)
			{
				ccCameraSensor* cloned = new ccCameraSensor(*camera);
				clone = (cloned ? cloned : nullptr);
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning camera sensor %1").arg(entity->getName()));
			}
		}
		else if (entity->isA(CC_TYPES::GBL_SENSOR))
		{
			ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(entity);
			if (sensor)
			{
				ccGBLSensor* cloned = new ccGBLSensor(*sensor);
				clone = (cloned ? cloned : nullptr);
			}
			if (!clone)
			{
				ccConsole::Error(tr("An error occurred while cloning GBL sensor %1").arg(entity->getName()));
			}
		}
		else
		{
			ccLog::Warning(tr("Entity '%1' can't be cloned (type not supported yet!)").arg(entity->getName()));
		}

		if (clone)
		{
			//copy GL transformation history
			clone->setGLTransformationHistory(entity->getGLTransformationHistory());
			//copy display
			clone->setDisplay(entity->getDisplay());

			addToDB(clone);
			lastClone = clone;
		}
	}

	if (lastClone && m_ccRoot)
	{
		m_ccRoot->selectEntity(lastClone);
	}

	updateUI();
}

void MainWindow::doActionAddConstantSF()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;

	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
		return;
	}

	QString defaultName = "Constant";
	unsigned trys = 1;
	while (cloud->getScalarFieldIndexByName(qPrintable(defaultName)) >= 0 || trys > 99)
	{
		defaultName = tr("Constant #%1").arg(++trys);
	}

	//ask for a name
	bool ok;
	QString sfName = QInputDialog::getText(this, tr("New SF name"), tr("SF name (must be unique)"), QLineEdit::Normal, defaultName, &ok);
	if (!ok)
		return;

	if (ccEntityAction::sfAddConstant(cloud, sfName, false, this))
	{
		updateUI();
		cloud->redrawDisplay();
	}
}

void MainWindow::doActionAddClassificationSF()
{
	if (!haveOneSelection())
	{
		if (haveSelection())
			ccConsole::Error(tr("Select only one cloud or one mesh!"));
		return;
	}

	ccHObject* ent = m_selectedEntities.front();

	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);

	//for "real" point clouds only
	if (!cloud)
		return;

	if (lockedVertices && !ent->isAncestorOf(cloud))
	{
		ccUtils::DisplayLockedVerticesWarning(ent->getName(), true);
		return;
	}

	if (ccEntityAction::sfAddConstant(cloud, "Classification", true, this))
	{
		updateUI();
		cloud->redrawDisplay();
	}
}

void MainWindow::doActionScalarFieldFromColor()
{
	if (!ccEntityAction::sfFromColor(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionScalarFieldArithmetic()
{
	if (!ccEntityAction::sfArithmetic(m_selectedEntities, this))
		return;

	refreshAll();
	updateUI();
}

void MainWindow::doActionFitSphere()
{
	double outliersRatio = 0.5;
	double confidence = 0.99;

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (ccHObject *entity : getSelectedEntities())
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud)
			continue;

		CCVector3 center;
		PointCoordinateType radius;
		double rms;
		if (CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(cloud,
			outliersRatio,
			center,
			radius,
			rms,
			&pDlg,
			confidence) != CCCoreLib::GeometricalAnalysisTools::NoError)
		{
			ccLog::Warning(tr("[Fit sphere] Failed to fit a sphere on cloud '%1'").arg(cloud->getName()));
			continue;
		}

		ccLog::Print(tr("[Fit sphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
			.arg(cloud->getName())
			.arg(center.x)
			.arg(center.y)
			.arg(center.z)
			.arg(radius)
			.arg(rms));

		ccGLMatrix trans;
		trans.setTranslation(center);
		ccSphere* sphere = new ccSphere(radius, &trans, tr("Sphere r=%1 [rms %2]").arg(radius).arg(rms));
		cloud->addChild(sphere);
		//sphere->setDisplay(cloud->getDisplay());
		sphere->prepareDisplayForRefresh();
		sphere->copyGlobalShiftAndScale(*cloud);
		addToDB(sphere, false, false, false);
	}

	refreshAll();
}

void MainWindow::doActionFitPlane()
{
	doComputePlaneOrientation(false);
}

void MainWindow::doActionFitFacet()
{
	doComputePlaneOrientation(true);
}

void MainWindow::doComputePlaneOrientation(bool fitFacet)
{
	if (!haveSelection())
		return;

	double maxEdgeLength = 0.0;
	if (fitFacet)
	{
		bool ok = true;
		static double s_polygonMaxEdgeLength = 0.0;
		maxEdgeLength = QInputDialog::getDouble(this, tr("Fit facet"), tr("Max edge length (0 = no limit)"), s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
		if (!ok)
			return;
		s_polygonMaxEdgeLength = maxEdgeLength;
	}

	ccHObject::Container selectedEntities = getSelectedEntities(); //warning, getSelectedEntites may change during this loop!
	bool firstEntity = true;

	for (ccHObject *entity : selectedEntities)
	{
		ccShiftedObject* shifted = nullptr;
		CCCoreLib::GenericIndexedCloudPersist* cloud = nullptr;

		if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
			cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(poly);
			shifted = poly;
		}
		else
		{
			ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(entity);
			if (gencloud)
			{
				cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(gencloud);
				shifted = gencloud;
			}
		}

		if (cloud)
		{
			double rms = 0.0;
			CCVector3 C;
			CCVector3 N;

			ccHObject* plane = nullptr;
			if (fitFacet)
			{
				ccFacet* facet = ccFacet::Create(cloud, static_cast<PointCoordinateType>(maxEdgeLength));
				if (facet)
				{
					plane = static_cast<ccHObject*>(facet);
					N = facet->getNormal();
					C = facet->getCenter();
					rms = facet->getRMS();

					//manually copy shift & scale info!
					if (shifted)
					{
						ccPolyline* contour = facet->getContour();
						if (contour)
						{
							contour->copyGlobalShiftAndScale(*shifted);
						}

						ccMesh* polygon = facet->getPolygon();
						if (polygon)
						{
							polygon->copyGlobalShiftAndScale(*shifted);
						}

						ccPointCloud* points = facet->getOriginPoints();
						if (points)
						{
							points->copyGlobalShiftAndScale(*shifted);
						}
					}
				}
			}
			else
			{
				ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
				if (pPlane)
				{
					plane = static_cast<ccHObject*>(pPlane);
					N = pPlane->getNormal();
					C = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
					pPlane->enableStippling(true);

					if (shifted)
					{
						pPlane->copyGlobalShiftAndScale(*shifted);
					}
				}
			}

			//as all information appears in Console...
			forceConsoleDisplay();

			if (plane)
			{
				ccConsole::Print(tr("[Orientation] Entity '%1'").arg(entity->getName()));
				ccConsole::Print(tr("\t- plane fitting RMS: %1").arg(rms));

				//We always consider the normal with a positive 'Z' by default!
				if (N.z < 0.0)
					N *= -1.0;
				ccConsole::Print(tr("\t- normal: (%1, %2, %3)").arg(N.x).arg(N.y).arg(N.z));

				//we compute strike & dip by the way
				PointCoordinateType dip = 0.0f;
				PointCoordinateType dipDir = 0.0f;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));

				//hack: output the transformation matrix that would make this normal points towards +Z
				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, CCCoreLib::PC_ONE));
				CCVector3 Gt = C;
				makeZPosMatrix.applyRotation(Gt);
				makeZPosMatrix.setTranslation(C - Gt);
				ccConsole::Print(tr("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:"));
				ccConsole::Print(makeZPosMatrix.toString(12, ' ')); //full precision
				ccConsole::Print(tr("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog"));

				plane->setName(dipAndDipDirStr);
				plane->applyGLTransformation_recursive(); //not yet in DB
				plane->setVisible(true);
				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

				entity->addChild(plane);
				plane->setDisplay(entity->getDisplay());
				plane->prepareDisplayForRefresh_recursive();
				addToDB(plane);

				if (firstEntity)
				{
					m_ccRoot->unselectAllEntities();
					m_ccRoot->selectEntity(plane);
				}
			}
			else
			{
				ccConsole::Warning(tr("Failed to fit a plane/facet on entity '%1'").arg(entity->getName()));
			}
		}
	}

	refreshAll();
	updateUI();
}

void MainWindow::doShowPrimitiveFactory()
{
	if (!m_pfDlg)
		m_pfDlg = new ccPrimitiveFactoryDlg(this);

	m_pfDlg->setModal(false);
	m_pfDlg->setWindowModality(Qt::NonModal);
	m_pfDlg->show();
}

void MainWindow::doComputeGeometricFeature()
{
	static ccLibAlgorithms::GeomCharacteristicSet s_selectedCharacteristics;
	static CCVector3 s_upDir(0, 0, 1);
	static bool s_upDirDefined = false;

	ccGeomFeaturesDlg gfDlg(this);
	double radius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	gfDlg.setRadius(radius);

	// restore semi-persistent parameters
	gfDlg.setSelectedFeatures(s_selectedCharacteristics);
	if (s_upDirDefined)
	{
		gfDlg.setUpDirection(s_upDir);
	}

	if (!gfDlg.exec())
		return;

	radius = gfDlg.getRadius();
	if (!gfDlg.getSelectedFeatures(s_selectedCharacteristics))
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	CCVector3* upDir = gfDlg.getUpDirection();

	// remember semi-persistent parameters
	s_upDirDefined = (upDir != nullptr);
	if (s_upDirDefined)
	{
		s_upDir = *upDir;
	}

	ccLibAlgorithms::ComputeGeomCharacteristics(s_selectedCharacteristics, static_cast<PointCoordinateType>(radius), m_selectedEntities, upDir, this);

	refreshAll();
	updateUI();
}

void MainWindow::doActionSFGradient()
{
	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, m_selectedEntities, this))
		return;
	refreshAll();
	updateUI();
}

void MainWindow::doSphericalNeighbourhoodExtractionTest()
{
	size_t selNum = m_selectedEntities.size();
	if (selNum < 1)
		return;

	//spherical neighborhood extraction radius
	PointCoordinateType sphereRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
	if (sphereRadius < 0)
	{
		ccConsole::Error(tr("Invalid kernel size!"));
		return;
	}

	bool ok;
	double val = QInputDialog::getDouble(this, tr("SNE test"), tr("Radius:"), static_cast<double>(sphereRadius), DBL_MIN, 1.0e9, 8, &ok);
	if (!ok)
		return;
	sphereRadius = static_cast<PointCoordinateType>(val);

	QString sfName = tr("Spherical extraction test (%1)").arg(sphereRadius);

	ccProgressDialog pDlg(true, this);
	pDlg.setAutoClose(false);

	for (size_t i = 0; i < selNum; ++i)
	{
		//we only process clouds
		if (!m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);

		int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIdx < 0)
			sfIdx = cloud->addScalarField(qPrintable(sfName));
		if (sfIdx < 0)
		{
			ccConsole::Error(tr("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(cloud->getName()));
			return;
		}

		ccOctree::Shared octree = cloud->getOctree();
		if (!octree)
		{
			pDlg.reset();
			pDlg.show();
			octree = cloud->computeOctree(&pDlg);
			if (!octree)
			{
				ccConsole::Error(tr("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
				return;
			}
		}

		CCCoreLib::ScalarField* sf = cloud->getScalarField(sfIdx);
		sf->fill(CCCoreLib::NAN_VALUE);
		cloud->setCurrentScalarField(sfIdx);

		QElapsedTimer eTimer;
		eTimer.start();

		size_t extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);

		const unsigned samples = 1000;
		for (unsigned j = 0; j < samples; ++j)
		{
			unsigned randIndex = dist(gen);
			CCCoreLib::DgmOctree::NeighboursSet neighbours;
			octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex), sphereRadius, neighbours, level);
			size_t neihgboursCount = neighbours.size();
			extractedPoints += neihgboursCount;
			for (size_t k = 0; k < neihgboursCount; ++k)
				cloud->setPointScalarValue(neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
		}
		ccConsole::Print(tr("[SNE_TEST] Mean extraction time = %1 ms (radius = %2, mean (neighbours) = %3)").arg(eTimer.elapsed()).arg(sphereRadius).arg(extractedPoints / static_cast<double>(samples), 0, 'f', 1));

		sf->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh();
	}

	refreshAll();
	updateUI();
}

void MainWindow::doCylindricalNeighbourhoodExtractionTest()
{
	bool ok;
	double radius = QInputDialog::getDouble(this, tr("CNE Test"), tr("radius"), 0.02, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	double height = QInputDialog::getDouble(this, tr("CNE Test"), tr("height"), 0.05, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;

	ccPointCloud* cloud = new ccPointCloud(tr("cube"));
	const unsigned ptsCount = 1000000;
	if (!cloud->reserve(ptsCount))
	{
		ccConsole::Error(tr("Not enough memory!"));
		delete cloud;
		return;
	}

	//fill a unit cube with random points
	{
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_real_distribution<double> dist(0, 1);

		for (unsigned i = 0; i < ptsCount; ++i)
		{
			CCVector3 P(dist(gen),
				dist(gen),
				dist(gen));

			cloud->addPoint(P);
		}
	}

	//get/Add scalar field
	static const char DEFAULT_CNE_TEST_TEMP_SF_NAME[] = "CNE test";
	int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_CNE_TEST_TEMP_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField(DEFAULT_CNE_TEST_TEMP_SF_NAME);
	if (sfIdx < 0)
	{
		ccConsole::Error(tr("Not enough memory!"));
		delete cloud;
		return;
	}
	cloud->setCurrentScalarField(sfIdx);

	//reset scalar field
	cloud->getScalarField(sfIdx)->fill(CCCoreLib::NAN_VALUE);

	ccProgressDialog pDlg(true, this);
	ccOctree::Shared octree = cloud->computeOctree(&pDlg);
	if (octree)
	{
		QElapsedTimer subTimer;
		subTimer.start();
		unsigned long long extractedPoints = 0;
		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5*radius)); //2.5 = empirical
		const unsigned samples = 1000;
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_real_distribution<PointCoordinateType> distAngle(0, static_cast<PointCoordinateType>(2 * M_PI));
		std::uniform_int_distribution<unsigned> distIndex(0, ptsCount - 1);

		for (unsigned j = 0; j < samples; ++j)
		{
			//generate random normal vector
			CCVector3 dir(0, 0, 1);
			{
				ccGLMatrix rot;
				rot.initFromParameters(distAngle(gen),
					distAngle(gen),
					distAngle(gen),
					CCVector3(0, 0, 0));
				rot.applyRotation(dir);
			}
			unsigned randIndex = distIndex(gen);

			CCCoreLib::DgmOctree::CylindricalNeighbourhood cn;
			cn.center = *cloud->getPoint(randIndex);
			cn.dir = dir;
			cn.level = level;
			cn.radius = static_cast<PointCoordinateType>(radius);
			cn.maxHalfLength = static_cast<PointCoordinateType>(height / 2);

			octree->getPointsInCylindricalNeighbourhood(cn);
			//octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),radius,neighbours,level);
			size_t neihgboursCount = cn.neighbours.size();
			extractedPoints += static_cast<unsigned long long>(neihgboursCount);
			for (size_t k = 0; k < neihgboursCount; ++k)
			{
				cloud->setPointScalarValue(cn.neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(cn.neighbours[k].squareDistd)));
			}
		}
		ccConsole::Print(tr("[CNE_TEST] Mean extraction time = %1 ms (radius = %2, height = %3, mean (neighbours) = %4))").arg(subTimer.elapsed()).arg(radius).arg(height).arg(static_cast<double>(extractedPoints) / samples, 0, 'f', 1));
	}
	else
	{
		ccConsole::Error(tr("Failed to compute octree!"));
	}

	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
	sf->computeMinAndMax();
	sf->showNaNValuesInGrey(false);
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);

	addToDB(cloud);

	refreshAll();
	updateUI();
}

void MainWindow::doActionCreateCloudFromEntCenters()
{
	size_t selNum = getSelectedEntities().size();

	ccPointCloud* centers = new ccPointCloud(tr("centers"));
	if (!centers->reserve(static_cast<unsigned>(selNum)))
	{
		ccLog::Error(tr("Not enough memory!"));
		delete centers;
		centers = nullptr;
		return;
	}

	//look for clouds
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);

			if (cloud == nullptr)
			{
				continue;
			}

			centers->addPoint(cloud->getOwnBB().getCenter());

			//we display the cloud in the same window as the first (selected) cloud we encounter
			if (!centers->getDisplay())
			{
				centers->setDisplay(cloud->getDisplay());
			}
		}
	}

	if (centers->size() == 0)
	{
		ccLog::Error(tr("No cloud in selection?!"));
		delete centers;
		centers = nullptr;
	}
	else
	{
		centers->resize(centers->size());
		centers->setPointSize(10);
		centers->setVisible(true);
		addToDB(centers);
	}
}

void MainWindow::doActionComputeBestICPRmsMatrix()
{
	//look for clouds
	std::vector<ccPointCloud*> clouds;
	try
	{
		for (ccHObject *entity : getSelectedEntities())
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				clouds.push_back(cloud);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	size_t cloudCount = clouds.size();
	if (cloudCount < 2)
	{
		ccLog::Error(tr("Need at least two clouds!"));
		return;
	}

	//init matrices
	std::vector<double> rmsMatrix;
	std::vector<ccGLMatrix> matrices;
	std::vector< std::pair<double, double> > matrixAngles;
	try
	{
		rmsMatrix.resize(cloudCount*cloudCount, 0);

		//init all possible transformations
		static const double angularStep_deg = 45.0;
		unsigned phiSteps = static_cast<unsigned>(360.0 / angularStep_deg);
		assert(CCCoreLib::LessThanEpsilon(std::abs(360.0 - phiSteps * angularStep_deg)));
		unsigned thetaSteps = static_cast<unsigned>(180.0 / angularStep_deg);
		assert(CCCoreLib::LessThanEpsilon(std::abs(180.0 - thetaSteps * angularStep_deg)));
		unsigned rotCount = phiSteps * (thetaSteps - 1) + 2;
		matrices.reserve(rotCount);
		matrixAngles.reserve(rotCount);

		for (unsigned j = 0; j <= thetaSteps; ++j)
		{
			//we want to cover the full [0-180] interval! ([-90;90] in fact)
			double theta_deg = j * angularStep_deg - 90.0;
			for (unsigned i = 0; i < phiSteps; ++i)
			{
				double phi_deg = i * angularStep_deg;
				ccGLMatrix trans;
				trans.initFromParameters(static_cast<float>(CCCoreLib::DegreesToRadians(phi_deg)),
					static_cast<float>(CCCoreLib::DegreesToRadians(theta_deg)),
					0,
					CCVector3(0, 0, 0));
				matrices.push_back(trans);
				matrixAngles.push_back(std::pair<double, double>(phi_deg, theta_deg));

				//for poles, no need to rotate!
				if (j == 0 || j == thetaSteps)
					break;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory!"));
		return;
	}

	//let's start!
	{
		ccProgressDialog pDlg(true, this);
		pDlg.setMethodTitle(tr("Testing all possible positions"));
		pDlg.setInfo(tr("%1 clouds and %2 positions").arg(cloudCount).arg(matrices.size()));
		CCCoreLib::NormalizedProgress nProgress(&pDlg, static_cast<unsigned>(((cloudCount*(cloudCount - 1)) / 2)*matrices.size()));
		pDlg.start();
		QApplication::processEvents();

		//#define TEST_GENERATION
#ifdef TEST_GENERATION
		ccPointCloud* testSphere = new ccPointCloud();
		testSphere->reserve(matrices.size());
#endif

		for (size_t i = 0; i < cloudCount - 1; ++i)
		{
			ccPointCloud* A = clouds[i];
			A->computeOctree();

			for (size_t j = i + 1; j < cloudCount; ++j)
			{
				ccGLMatrix transBToZero;
				transBToZero.toIdentity();
				transBToZero.setTranslation(-clouds[j]->getOwnBB().getCenter());

				ccGLMatrix transFromZeroToA;
				transFromZeroToA.toIdentity();
				transFromZeroToA.setTranslation(A->getOwnBB().getCenter());

#ifndef TEST_GENERATION
				double minRMS = -1.0;
				int bestMatrixIndex = -1;
				ccPointCloud* bestB = nullptr;
#endif
				for (size_t k = 0; k < matrices.size(); ++k)
				{
					ccPointCloud* B = clouds[j]->cloneThis();
					if (!B)
					{
						ccLog::Error(tr("Not enough memory!"));
						return;
					}

					ccGLMatrix BtoA = transFromZeroToA * matrices[k] * transBToZero;
					B->applyRigidTransformation(BtoA);

#ifndef TEST_GENERATION
					double finalRMS = 0.0;
					unsigned finalPointCount = 0;
					CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
					CCCoreLib::ICPRegistrationTools::ScaledTransformation registerTrans;
					CCCoreLib::ICPRegistrationTools::Parameters params;
					{
						params.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
						params.minRMSDecrease = 1.0e-6;
					}

					result = CCCoreLib::ICPRegistrationTools::Register(A, nullptr, B, params, registerTrans, finalRMS, finalPointCount);

					if (result >= CCCoreLib::ICPRegistrationTools::ICP_ERROR)
					{
						delete B;
						if (bestB)
							delete bestB;
						ccLog::Error(tr("An error occurred while performing ICP!"));
						return;
					}

					if (minRMS < 0 || finalRMS < minRMS)
					{
						minRMS = finalRMS;
						bestMatrixIndex = static_cast<int>(k);
						std::swap(bestB, B);
					}

					if (B)
					{
						delete B;
						B = nullptr;
					}
#else
					addToDB(B);

					//Test sphere
					CCVector3 Y(0, 1, 0);
					matrices[k].apply(Y);
					testSphere->addPoint(Y);
#endif

					if (!nProgress.oneStep())
					{
						//process cancelled by user
						return;
					}
				}

#ifndef TEST_GENERATION
				if (bestMatrixIndex >= 0)
				{
					assert(bestB);
					ccHObject* group = new ccHObject(tr("Best case #%1 / #%2 - RMS = %3").arg(i + 1).arg(j + 1).arg(minRMS));
					group->addChild(bestB);
					group->setDisplay_recursive(A->getDisplay());
					addToDB(group);
					ccLog::Print(tr("[DoActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: min RMS = %3 (phi = %4 / theta = %5 deg.)").arg(i + 1).arg(j + 1).arg(minRMS).arg(matrixAngles[bestMatrixIndex].first).arg(matrixAngles[bestMatrixIndex].second));
				}
				else
				{
					assert(!bestB);
					ccLog::Warning(tr("[DoActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: INVALID").arg(i + 1).arg(j + 1));
				}

				rmsMatrix[i*cloudCount + j] = minRMS;
#else
				addToDB(testSphere);
				i = cloudCount;
				break;
#endif
			}
		}
	}

	//export result as a CSV file
#ifdef TEST_GENERATION
	if (false)
#endif
	{
		//persistent settings
		QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
		settings.beginGroup(ccPS::SaveFile());
		QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

		QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this,
			tr("Select output file"),
			currentPath, "*.csv");
		if (outputFilename.isEmpty())
			return;

		QFile fp(outputFilename);
		if (fp.open(QFile::Text | QFile::WriteOnly))
		{
			QTextStream stream(&fp);
			//header
			{
				stream << "RMS";
				for (ccPointCloud *cloud : clouds)
				{
					stream << ';';
					stream << cloud->getName();
				}
				stream << endl;
			}

			//rows
			for (size_t j = 0; j < cloudCount; ++j)
			{
				stream << clouds[j]->getName();
				stream << ';';
				for (size_t i = 0; i < cloudCount; ++i)
				{
					stream << rmsMatrix[j*cloudCount + i];
					stream << ';';
				}
				stream << endl;
			}

			ccLog::Print(tr("[DoActionComputeBestICPRmsMatrix] Job done"));
		}
		else
		{
			ccLog::Error(tr("Failed to save output file?!"));
		}
	}
}

void MainWindow::doActionExportPlaneInfo()
{
	ccHObject::Container planes;

	const ccHObject::Container& selectedEntities = getSelectedEntities();
	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//a group
		selectedEntities.front()->filterChildren(planes, true, CC_TYPES::PLANE, false);
	}
	else
	{
		for (ccHObject* ent : selectedEntities)
		{
			if (ent->isKindOf(CC_TYPES::PLANE))
			{
				//a single plane
				planes.push_back(static_cast<ccPlane*>(ent));
			}
		}
	}

	if (planes.size() == 0)
	{
		ccLog::Error(tr("No plane in selection"));
		return;
	}

	//persistent settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this,
		tr("Select output file"),
		currentPath, "*.csv");

	if (outputFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	QFile csvFile(outputFilename);
	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
	{
		ccConsole::Error(tr("Failed to open file for writing! (check file permissions)"));
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//write CSV header
	QTextStream csvStream(&csvFile);
	csvStream << "Name;";
	csvStream << "Width;";
	csvStream << "Height;";
	csvStream << "Cx;";
	csvStream << "Cy;";
	csvStream << "Cz;";
	csvStream << "Nx;";
	csvStream << "Ny;";
	csvStream << "Nz;";
	csvStream << "Dip;";
	csvStream << "Dip dir;";
	csvStream << endl;

	QChar separator(';');

	//write one line per plane
	for (ccHObject* ent : planes)
	{
		ccPlane* plane = static_cast<ccPlane*>(ent);

		CCVector3 C = plane->getOwnBB().getCenter();
		CCVector3 N = plane->getNormal();
		PointCoordinateType dip_deg = 0;
		PointCoordinateType dipDir_deg = 0;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip_deg, dipDir_deg);

		csvStream << plane->getName() << separator;		//Name
		csvStream << plane->getXWidth() << separator;	//Width
		csvStream << plane->getYWidth() << separator;	//Height
		csvStream << C.x << separator;					//Cx
		csvStream << C.y << separator;					//Cy
		csvStream << C.z << separator;					//Cz
		csvStream << N.x << separator;					//Nx
		csvStream << N.y << separator;					//Ny
		csvStream << N.z << separator;					//Nz
		csvStream << dip_deg << separator;				//Dip
		csvStream << dipDir_deg << separator;			//Dip direction
		csvStream << endl;
	}

	ccConsole::Print(tr("[I/O] File '%1' successfully saved (%2 plane(s))").arg(outputFilename).arg(planes.size()));
	csvFile.close();
}

void MainWindow::doActionExportCloudInfo()
{
	//look for clouds
	ccHObject::Container clouds;

	const ccHObject::Container& selectedEntities = getSelectedEntities();
	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//a group
		selectedEntities.front()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
	}
	else
	{
		for (ccHObject* entity : selectedEntities)
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				clouds.push_back(cloud);
			}
		}
	}

	if (clouds.empty())
	{
		ccConsole::Error(tr("Select at least one point cloud!"));
		return;
	}

	//persistent settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this,
		tr("Select output file"),
		currentPath, "*.csv");
	if (outputFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}

	QFile csvFile(outputFilename);
	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
	{
		ccConsole::Error(tr("Failed to open file for writing! (check file permissions)"));
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//determine the maximum number of SFs
	unsigned maxSFCount = 0;
	for (ccHObject* entity : clouds)
	{
		maxSFCount = std::max<unsigned>(maxSFCount, static_cast<ccPointCloud*>(entity)->getNumberOfScalarFields());
	}

	//write CSV header
	QTextStream csvStream(&csvFile);
	csvStream << "Name;";
	csvStream << "Points;";
	csvStream << "meanX;";
	csvStream << "meanY;";
	csvStream << "meanZ;";
	{
		for (unsigned i = 0; i < maxSFCount; ++i)
		{
			QString sfIndex = QString("SF#%1").arg(i + 1);
			csvStream << sfIndex << " name;";
			csvStream << sfIndex << " valid values;";
			csvStream << sfIndex << " mean;";
			csvStream << sfIndex << " std.dev.;";
			csvStream << sfIndex << " sum;";
		}
	}
	csvStream << endl;

	//write one line per cloud
	{
		for (ccHObject* entity : clouds)
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);

			CCVector3 G = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
			csvStream << cloud->getName() << ';' /*"Name;"*/;
			csvStream << cloud->size() << ';' /*"Points;"*/;
			csvStream << G.x << ';' /*"meanX;"*/;
			csvStream << G.y << ';' /*"meanY;"*/;
			csvStream << G.z << ';' /*"meanZ;"*/;
			for (unsigned j = 0; j < cloud->getNumberOfScalarFields(); ++j)
			{
				CCCoreLib::ScalarField* sf = cloud->getScalarField(j);
				csvStream << sf->getName() << ';' /*"SF name;"*/;

				unsigned validCount = 0;
				double sfSum = 0.0;
				double sfSum2 = 0.0;
				for (unsigned k = 0; k < sf->currentSize(); ++k)
				{
					const ScalarType& val = sf->getValue(k);
					if (CCCoreLib::ScalarField::ValidValue(val))
					{
						++validCount;
						sfSum += val;
						sfSum2 += val * val;
					}
				}
				csvStream << validCount << ';' /*"SF valid values;"*/;
				double mean = sfSum / validCount;
				csvStream << mean << ';' /*"SF mean;"*/;
				csvStream << sqrt(std::abs(sfSum2 / validCount - mean * mean)) << ';' /*"SF std.dev.;"*/;
				csvStream << sfSum << ';' /*"SF sum;"*/;
			}
			csvStream << endl;
		}
	}

	ccConsole::Print(tr("[I/O] File '%1' successfully saved (%2 cloud(s))").arg(outputFilename).arg(clouds.size()));
	csvFile.close();
}

void MainWindow::doActionCloudCloudDist()
{
	if (getSelectedEntities().size() != 2)
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::POINT_CLOUD) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select 2 point clouds!"));
		return;
	}

	ccOrderChoiceDlg dlg(m_selectedEntities.front(), tr("Compared"),
		m_selectedEntities[1], tr("Reference"),
		this);
	if (!dlg.exec())
		return;

	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
	ccGenericPointCloud* refCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());

	//assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;

	m_compDlg = new ccComparisonDlg(compCloud, refCloud, ccComparisonDlg::CLOUDCLOUD_DIST, this);
	if (!m_compDlg->initDialog())
	{
		ccConsole::Error(tr("Failed to initialize comparison dialog"));
		delete m_compDlg;
		m_compDlg = nullptr;
		return;
	}

	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();
	//cDlg.setModal(false);
	//cDlg.exec();
	freezeUI(true);
}

void MainWindow::doActionCloudMeshDist()
{
	if (getSelectedEntities().size() != 2)
	{
		ccConsole::Error(tr("Select 2 entities!"));
		return;
	}

	bool isMesh[2] = { false,false };
	unsigned meshNum = 0;
	unsigned cloudNum = 0;
	for (unsigned i = 0; i < 2; ++i)
	{
		if (m_selectedEntities[i]->isKindOf(CC_TYPES::MESH))
		{
			++meshNum;
			isMesh[i] = true;
		}
		else if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			++cloudNum;
		}
	}

	if (meshNum == 0)
	{
		ccConsole::Error(tr("Select at least one mesh!"));
		return;
	}
	else if (meshNum + cloudNum < 2)
	{
		ccConsole::Error(tr("Select one mesh and one cloud or two meshes!"));
		return;
	}

	ccHObject* compEnt = nullptr;
	ccGenericMesh* refMesh = nullptr;

	if (meshNum == 1)
	{
		compEnt = m_selectedEntities[isMesh[0] ? 1 : 0];
		refMesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[isMesh[0] ? 0 : 1]);
	}
	else
	{
		ccOrderChoiceDlg dlg(m_selectedEntities.front(), tr("Compared"),
			m_selectedEntities[1], tr("Reference"),
			this);
		if (!dlg.exec())
			return;

		compEnt = dlg.getFirstEntity();
		refMesh = ccHObjectCaster::ToGenericMesh(dlg.getSecondEntity());
	}

	//assert(!m_compDlg);
	if (m_compDlg)
		delete m_compDlg;
	m_compDlg = new ccComparisonDlg(compEnt, refMesh, ccComparisonDlg::CLOUDMESH_DIST, this);
	if (!m_compDlg->initDialog())
	{
		ccConsole::Error(tr("Failed to initialize comparison dialog"));
		delete m_compDlg;
		m_compDlg = nullptr;
		return;
	}

	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
	m_compDlg->show();

	freezeUI(true);
}

void MainWindow::doActionCloudPrimitiveDist()
{
	bool foundPrimitive = false;
	ccHObject::Container clouds;
	ccHObject* refEntity = nullptr;
	CC_CLASS_ENUM entityType = CC_TYPES::OBJECT;
	QString errString = tr("[Compute Primitive Distances] Cloud to %1 failed, error code = %2!");

	for (unsigned i = 0; i < getSelectedEntities().size(); ++i)
	{

		if (m_selectedEntities[i]->isKindOf(CC_TYPES::PRIMITIVE) || m_selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
		{
			if (m_selectedEntities[i]->isA(CC_TYPES::PLANE) ||
				m_selectedEntities[i]->isA(CC_TYPES::SPHERE) ||
				m_selectedEntities[i]->isA(CC_TYPES::CYLINDER) ||
				m_selectedEntities[i]->isA(CC_TYPES::CONE) ||
				m_selectedEntities[i]->isA(CC_TYPES::BOX) ||
				m_selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
			{
				if (foundPrimitive)
				{
					ccConsole::Error(tr("[Compute Primitive Distances] Select only a single Plane/Box/Sphere/Cylinder/Cone/Polyline Primitive"));
					return;
				}
				foundPrimitive = true;
				refEntity = m_selectedEntities[i];
				entityType = refEntity->getClassID();
			}
		}
		else if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			clouds.push_back(m_selectedEntities[i]);
		}
	}

	if (!foundPrimitive)
	{
		ccConsole::Error(tr("[Compute Primitive Distances] Select at least one Plane/Box/Sphere/Cylinder/Cone/Polyline Primitive!"));
		return;
	}
	if (clouds.size() <= 0)
	{
		ccConsole::Error(tr("[Compute Primitive Distances] Select at least one cloud!"));
		return;
	}

	ccPrimitiveDistanceDlg pDD{ this };
	if (refEntity->isA(CC_TYPES::PLANE))
	{
		pDD.treatPlanesAsBoundedCheckBox->setUpdatesEnabled(true);
	}
	bool execute = true;
	if (!refEntity->isA(CC_TYPES::POLY_LINE))
	{
		execute = pDD.exec();
	}
	if (execute)
	{
		bool signedDist = pDD.signedDistances();
		bool flippedNormals = signedDist && pDD.flipNormals();
		bool treatPlanesAsBounded = pDD.treatPlanesAsBounded();
		size_t errorCount = 0;
		for (auto &cloud : clouds)
		{
			ccPointCloud* compEnt = ccHObjectCaster::ToPointCloud(cloud);
			int sfIdx = compEnt->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
			if (sfIdx < 0)
			{
				//we need to create a new scalar field
				sfIdx = compEnt->addScalarField(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
				if (sfIdx < 0)
				{
					ccLog::Warning(tr("[Compute Primitive Distances] [Cloud: %1] Couldn't allocate a new scalar field for computing distances! Try to free some memory ...").arg(compEnt->getName()));
					++errorCount;
					continue;
				}
			}
			compEnt->setCurrentScalarField(sfIdx);
			if (!compEnt->enableScalarField())
			{
				ccLog::Warning(tr("[Compute Primitive Distances] [Cloud: %1] Not enough memory").arg(compEnt->getName()));
				++errorCount;
				continue;
			}
			compEnt->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueToNaN);
			int returnCode;
			switch (entityType)
			{
			case CC_TYPES::SPHERE:
			{
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2SphereEquation(compEnt, refEntity->getOwnBB().getCenter(), static_cast<ccSphere*>(refEntity)->getRadius(), signedDist)))
					ccConsole::Error(errString.arg(tr("Sphere")).arg(returnCode));
				break;
			}
			case CC_TYPES::PLANE:
			{
				ccPlane* plane = static_cast<ccPlane*>(refEntity);
				if (treatPlanesAsBounded)
				{
					CCCoreLib::SquareMatrix rotationTransform(plane->getTransformation().data(), true);
					if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2RectangleEquation(compEnt, plane->getXWidth(), plane->getYWidth(), rotationTransform, plane->getCenter(), signedDist)))
					{
						ccConsole::Warning(errString.arg(tr("Bounded Plane")).arg(returnCode));
						++errorCount;
					}
				}
				else
				{
					if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2PlaneEquation(compEnt, static_cast<ccPlane*>(refEntity)->getEquation(), signedDist)))
					{
						ccConsole::Warning(errString.arg(tr("Infinite Plane")).arg(returnCode));
						++errorCount;
					}
				}
				break;
			}
			case CC_TYPES::CYLINDER:
			{
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2CylinderEquation(compEnt, static_cast<ccCylinder*>(refEntity)->getBottomCenter(), static_cast<ccCylinder*>(refEntity)->getTopCenter(), static_cast<ccCylinder*>(refEntity)->getBottomRadius(), signedDist)))
				{
					ccConsole::Warning(errString.arg(tr("Cylinder")).arg(returnCode));
					++errorCount;
				}
				break;
			}
			case CC_TYPES::CONE:
			{
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2ConeEquation(compEnt, static_cast<ccCone*>(refEntity)->getLargeCenter(), static_cast<ccCone*>(refEntity)->getSmallCenter(), static_cast<ccCone*>(refEntity)->getLargeRadius(), static_cast<ccCone*>(refEntity)->getSmallRadius(), signedDist)))
				{
					ccConsole::Warning(errString.arg(tr("Cone")).arg(returnCode));
					++errorCount;
				}
				break;
			}
			case CC_TYPES::BOX:
			{
				const ccGLMatrix& glTransform = refEntity->getGLTransformationHistory();
				CCCoreLib::SquareMatrix rotationTransform(glTransform.data(), true);
				CCVector3 boxCenter = glTransform.getColumnAsVec3D(3);
				if (!(returnCode = CCCoreLib::DistanceComputationTools::computeCloud2BoxEquation(compEnt, static_cast<ccBox*>(refEntity)->getDimensions(), rotationTransform, boxCenter, signedDist)))
				{
					ccConsole::Warning(errString.arg(tr("Box")).arg(returnCode));
					++errorCount;
				}
				break;
			}
			case CC_TYPES::POLY_LINE:
			{
				signedDist = false;
				flippedNormals = false;
				ccPolyline* line = static_cast<ccPolyline*>(refEntity);
				returnCode = CCCoreLib::DistanceComputationTools::computeCloud2PolylineEquation(compEnt, line);
				if (!returnCode)
				{
					ccConsole::Warning(errString.arg(tr("Polyline")).arg(returnCode));
					++errorCount;
				}
				break;
			}
			default:
			{
				ccConsole::Error(tr("[Compute Primitive Distances] Unsupported primitive type")); //Shouldn't ever reach here...
				break;
			}
			}
			QString sfName;
			sfName.clear();
			sfName = QString(signedDist ? CC_CLOUD2PRIMITIVE_SIGNED_DISTANCES_DEFAULT_SF_NAME : CC_CLOUD2PRIMITIVE_DISTANCES_DEFAULT_SF_NAME);
			if (flippedNormals)
			{
				compEnt->forEach(CCCoreLib::ScalarFieldTools::SetScalarValueInverted);
				sfName += QString("[-]");
			}

			int _sfIdx = compEnt->getScalarFieldIndexByName(qPrintable(sfName));
			if (_sfIdx >= 0)
			{
				compEnt->deleteScalarField(_sfIdx);
				//we update sfIdx because indexes are all messed up after deletion
				sfIdx = compEnt->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
			}
			compEnt->renameScalarField(sfIdx, qPrintable(sfName));

			ccScalarField* sf = static_cast<ccScalarField*>(compEnt->getScalarField(sfIdx));
			if (sf)
			{
				ScalarType mean;
				ScalarType variance;
				sf->computeMinAndMax();
				sf->computeMeanAndVariance(mean, &variance);
				ccLog::Print(tr("[Compute Primitive Distances] [Primitive: %1] [Cloud: %2] [%3] Mean distance = %4 / std deviation = %5")
					.arg(refEntity->getName())
					.arg(compEnt->getName())
					.arg(sfName)
					.arg(mean)
					.arg(sqrt(variance)));
			}
			compEnt->setCurrentDisplayedScalarField(sfIdx);
			compEnt->showSF(sfIdx >= 0);
			compEnt->prepareDisplayForRefresh_recursive();
		}

		if (errorCount != 0)
		{
			ccLog::Error(tr("%1 error(s) occurred: refer to the Console (F8)").arg(errorCount));
		}

		MainWindow::UpdateUI();

		MainWindow::RefreshAllGLWindow(false);
	}
}


void MainWindow::deactivateComparisonMode(int result)
{
	//DGM: a bug apperead with recent changes (from CC or QT?)
	//which prevent us from deleting the dialog right away...
	//(it seems that QT has not yet finished the dialog closing
	//when the 'finished' signal is sent).
	//if(m_compDlg)
	//	delete m_compDlg;
	//m_compDlg = 0;

	//if the comparison is a success, we select only the compared entity
	if (m_compDlg && result == QDialog::Accepted && m_ccRoot)
	{
		ccHObject* compEntity = m_compDlg->getComparedEntity();
		if (compEntity)
		{
			m_ccRoot->selectEntity(compEntity);
		}
	}

	freezeUI(false);

	updateUI();
}

void MainWindow::toggleActiveWindowSunLight()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleSunLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowCustomLight()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->toggleCustomLight();
		win->redraw(false);
	}
}

void MainWindow::toggleActiveWindowAutoPickRotCenter(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setAutoPickPivotAtCenter(state);

		//save the option
		{
			QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
			settings.setValue(ccPS::AutoPickRotationCenter(), state);
		}
	}
}

void MainWindow::toggleActiveWindowShowCursorCoords(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->showCursorCoordinates(state);
	}
}

void MainWindow::toggleActiveWindowStereoVision(bool state)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		bool isActive = win->stereoModeIsEnabled();
		if (isActive == state)
		{
			//nothing to do
			return;
		}

		if (isActive)
		{
			win->disableStereoMode();

			if (win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
				|| win->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
			{
				//disable (exclusive) full screen
				m_UI->actionExclusiveFullScreen->setChecked(false);
			}
		}
		else
		{
			//display a parameters dialog
			ccStereoModeDlg smDlg(this);
			smDlg.setParameters(win->getStereoParams());
			if (!smDlg.exec())
			{
				//cancelled by the user
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
				return;
			}

			ccGLWindow::StereoParams params = smDlg.getParameters();
			ccLog::Warning(QString::number(params.stereoStrength));
#ifndef CC_GL_WINDOW_USE_QWINDOW
			if (!params.isAnaglyph())
			{
				ccLog::Error(tr("This version doesn't handle stereo glasses and headsets.\nUse the 'Stereo' version instead."));
				//activation of the stereo mode failed: cancel selection
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
				return;
			}
#endif

			//force perspective state!
			if (!win->getViewportParameters().perspectiveView)
			{
				setCenteredPerspectiveView(win, false);
			}

			if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION
				|| params.glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
			{
				//force (exclusive) full screen
				m_UI->actionExclusiveFullScreen->setChecked(true);
			}

			if (smDlg.updateFOV())
			{
				//set the right FOV
				double fov_deg = 2 * CCCoreLib::RadiansToDegrees(std::atan(params.screenWidth_mm / (2.0 * params.screenDistance_mm)));
				ccLog::Print(tr("[Stereo] F.O.V. forced to %1 deg.").arg(fov_deg));
				win->setFov(fov_deg);
			}

			if (!win->enableStereoMode(params))
			{
				if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION
					|| params.glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
				{
					//disable (exclusive) full screen
					m_UI->actionExclusiveFullScreen->setChecked(false);
				}

				//activation of the stereo mode failed: cancel selection
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
			}
		}
		win->redraw();
	}
}

bool MainWindow::checkStereoMode(ccGLWindow* win)
{
	assert(win);

	if (win && win->getViewportParameters().perspectiveView && win->stereoModeIsEnabled())
	{
		ccGLWindow::StereoParams params = win->getStereoParams();
		bool wasExclusiveFullScreen = win->exclusiveFullScreen();
		if (wasExclusiveFullScreen)
		{
			win->toggleExclusiveFullScreen(false);
		}
		win->disableStereoMode();

		if (CS::Widgets::FramelessMessageBox::question(this,
			tr("Stereo mode"),
			tr("Stereo-mode only works in perspective mode. Do you want to disable it?"),
			QMessageBox::Yes,
			QMessageBox::No) == QMessageBox::No)
		{
			if (wasExclusiveFullScreen)
			{
				win->toggleExclusiveFullScreen(true);
				win->enableStereoMode(params);
			}
			return false;
		}
		else
		{
			if (win == getActiveGLWindow())
			{
				m_UI->actionEnableStereo->setChecked(false);
			}
			else
			{
				assert(false);
				m_UI->actionEnableStereo->blockSignals(true);
				m_UI->actionEnableStereo->setChecked(false);
				m_UI->actionEnableStereo->blockSignals(false);
			}
		}
	}

	return true;
}

void MainWindow::changeTo2dView()
{
    ccGui::ParamStruct parameters = ccGui::Parameters();
    if (parameters.m_is2DView)
    {
        m_pViewerAction->setEnabled(false);
        slotChangeView();
    }
}

void MainWindow::setViewerActionState(bool state)
{
    m_pViewerAction->setEnabled(state);
}

void MainWindow::setViewerActionIcon(QString icon, QString text)
{
    setActionIcon(m_pViewerAction, icon, icon, icon);
    m_pViewerAction->setText(QCoreApplication::translate("MainWindow", text.toStdString().c_str(), nullptr));
}

void MainWindow::bindView(ccGLWindow* win)
{
    std::vector<ccHObject*> children;
    m_ccRoot->getRootEntity()->filterChildren(children);
    for (ccHObject* pobj : children)
    {
        pobj->prepareDisplayForRefresh_recursive();
        pobj->setDisplay_recursive(win);
    }
    if (win->windowName().compare("2D View") == 0)
    {
        CCVector3d axis = CCVector3d(0, 0, 1);
        win->setView(CC_VIEW_ORIENTATION::CC_TOP_VIEW);
        win->lockRotationAxis(true, axis);
    }

    setActiveSubWindow(win);
    win->zoomGlobal(true);
    win->redraw(true, false);
}

void MainWindow::slotChangeView()
{
    ccGLWindow* viewer_3d = GetGLWindow("3D View");
    ccGLWindow* viewer_2d = GetGLWindow("2D View");
    const ccViewportParameters& params = viewer_3d->getViewportParameters();
    ccGui::ParamStruct parameters = ccGui::Parameters();
    if (parameters.m_is2DView)
    {
        setActionIcon(m_pViewerAction, "3d", "3d", "3d");
        m_pViewerAction->setText(QCoreApplication::translate("MainWindow", "Viewer3D", nullptr));
        parameters.m_is2DView = false;
        //bindView(viewer_3d);
        //show3dWindow();
        viewer_3d->lockRotationAxis(false, CCVector3d(0, 0, 1));
        ccGLMatrixd Matd;
        Matd *= m_Matd;
        viewer_3d->setBaseViewMat(Matd);
        viewer_3d->setShowZAxis(true);
        setViewButtonDisabled(false, true);
        setActionIcon(m_PerspectiveAction, "perspectiveIcon", "perspectiveIcon", "perspectiveDisable");
        m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Perspective projection", nullptr));
        m_PerspectiveAction->setEnabled(true);
        viewer_3d->setPerspectiveState(true, true);
    }
    else
    {
        setActionIcon(m_pViewerAction, "2d", "2d", "2d");
        m_pViewerAction->setText(QCoreApplication::translate("MainWindow", "Viewer2D", nullptr));
        parameters.m_is2DView = true;
        //bindView(viewer_2d);
        //show2dWindow();
        m_Matd = viewer_3d->getBaseViewMat();
        ccViewportParameters params = viewer_3d->getViewportParameters();
        CCVector3d axis = CCVector3d(0, 0, 1);
        viewer_3d->setView(CC_VIEW_ORIENTATION::CC_TOP_VIEW);
        viewer_3d->lockRotationAxis(true, axis);
        viewer_3d->setShowZAxis(false);

        setViewButtonDisabled(true, true);
        setActionIcon(m_PerspectiveAction, "orthographicIcon", "orthographicIcon", "orthographicDisable");
        m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Orthographic projection", nullptr));
        m_PerspectiveAction->setEnabled(false);
        viewer_3d->setPerspectiveState(false, false);
        //m_UI->actionPointPicking->setEnabled(false);
        //m_UI->actionSegment->setEnabled(false);
        //m_UI->actionPointPicking->setDisabled(true);
        //m_UI->actionSegment->setDisabled(true);
    }
    ccGui::Set(parameters);
    if (m_selectedEntities.size() > 0)
    {
        m_UI->actionPointPicking->setEnabled(true);
        m_UI->actionSegment->setEnabled(true);
    }

    redrawAll();
}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
	if (m_isPerspectiveChangeLock)
	{
		return;
	}
	ccGLWindow* win = getActiveGLWindow();
	//ccGLWindow* win = GetGLWindow("3D View");
	if (win)
	{
		if (win->objectName() == "viewer2d")
		{
			return;
		}
		const ccViewportParameters& params = win->getViewportParameters();
		if (params.perspectiveView && params.objectCenteredView && !checkStereoMode(win)) //we need to check this only if we are already in object-centered perspective mode
		{
			return;
		}
		if (params.perspectiveView)
		{
			setActionIcon(m_PerspectiveAction, "orthographicIcon", "orthographicIcon", "orthographicDisable");
			m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Orthographic projection", nullptr));
		}
		else
		{
			setActionIcon(m_PerspectiveAction, "perspectiveIcon", "perspectiveIcon", "perspectiveDisable");
			m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Perspective projection", nullptr));
		}
		win->togglePerspective(true);
		win->redraw(false);
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);
	}
}

void MainWindow::toggleActiveWindowViewerBasedPerspective()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		const ccViewportParameters& params = win->getViewportParameters();
		if (params.perspectiveView && !params.objectCenteredView && !checkStereoMode(win)) //we need to check this only if we are already in viewer-based perspective mode
		{
			return;
		}
		win->togglePerspective(false);
		win->redraw(false);
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);
	}
}

void MainWindow::createSinglePointCloud()
{
	// ask the user to input the point coordinates
	static CCVector3d s_lastPoint(0, 0, 0);
	static size_t s_lastPointIndex = 0;
	ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastPoint.x, s_lastPoint.y, s_lastPoint.z, 4, tr("Point coordinates"), this);
	if (axisDlg.buttonBox->button(QDialogButtonBox::Ok))
		axisDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
	if (!axisDlg.exec())
		return;
	s_lastPoint.x = axisDlg.doubleSpinBox1->value();
	s_lastPoint.y = axisDlg.doubleSpinBox2->value();
	s_lastPoint.z = axisDlg.doubleSpinBox3->value();

	// create the cloud
	ccPointCloud* cloud = new ccPointCloud();
	if (!cloud->reserve(1))
	{
		delete cloud;
		ccLog::Error(tr("Not enough memory!"));
		return;
	}
	cloud->setName(tr("Point #%1").arg(++s_lastPointIndex));
	cloud->addPoint(s_lastPoint.toPC());
	cloud->setPointSize(5);

	// add it to the DB tree
	addToDB(cloud, true, true, true, true);

	// select it
	m_ccRoot->unselectAllEntities();
	setSelectedInDB(cloud, true);
}

void MainWindow::createPointCloudFromClipboard()
{
	const QClipboard* clipboard = QApplication::clipboard();
	assert(clipboard);
	const QMimeData* mimeData = clipboard->mimeData();
	if (!mimeData)
	{
		ccLog::Warning(tr("Clipboard is empty"));
		return;
	}

	if (!mimeData->hasText())
	{
		ccLog::Error("ASCII/text data expected");
		return;
	}

	// try to convert the data to a point cloud
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters.parentWidget = this;
	}

	ccHObject container;
	QByteArray data = mimeData->data("text/plain");
	CC_FILE_ERROR result = AsciiFilter().loadAsciiData(data, tr("Clipboard"), container, parameters);
	if (result != CC_FERR_NO_ERROR)
	{
		FileIOFilter::DisplayErrorMessage(result, tr("loading"), tr("from the clipboard"));
		return;
	}

	// we only expect clouds
	ccHObject::Container clouds;
	if (container.filterChildren(clouds, true, CC_TYPES::POINT_CLOUD) == 0)
	{
		assert(false);
		ccLog::Error(tr("No cloud loaded"));
		return;
	}

	// detach the clouds from the loading container
	for (ccHObject* cloud : clouds)
	{
		if (cloud)
		{
			container.removeDependencyWith(cloud);
		}
	}
	container.removeAllChildren();

	// retrieve or create the group to store the 'clipboard' clouds
	ccHObject* clipboardGroup = nullptr;
	{
		static unsigned s_clipboardGroupID = 0;

		if (s_clipboardGroupID != 0)
		{
			clipboardGroup = dbRootObject()->find(s_clipboardGroupID);
			if (nullptr == clipboardGroup)
			{
				// can't find the previous group
				s_clipboardGroupID = 0;
			}
		}

		if (s_clipboardGroupID == 0)
		{
			clipboardGroup = new ccHObject(tr("Clipboard"));
			s_clipboardGroupID = clipboardGroup->getUniqueID();
			addToDB(clipboardGroup, false, false, false, false);
		}
	}
	assert(clipboardGroup);

	bool normalsDisplayedByDefault = ccOptions::Instance().normalsDisplayedByDefault;
	for (ccHObject* cloud : clouds)
	{
		if (cloud)
		{
			clipboardGroup->addChild(cloud);
			cloud->setName(tr("Cloud #%1").arg(clipboardGroup->getChildrenNumber()));

			if (!normalsDisplayedByDefault)
			{
				// disable the normals on all loaded clouds!
				static_cast<ccGenericPointCloud*>(cloud)->showNormals(false);
			}
		}
	}

	// eventually, we can add the clouds to the DB tree
	for (size_t i = 0; i < clouds.size(); ++i)
	{
		ccHObject* cloud = clouds[i];
		if (cloud)
		{
			bool lastCloud = (i + 1 == clouds.size());
			addToDB(cloud, lastCloud, lastCloud, true, lastCloud);
		}
	}

	QMainWindow::statusBar()->showMessage(tr("%1 cloud(s) loaded from the clipboard").arg(clouds.size()), 2000);
}

void MainWindow::toggleLockRotationAxis()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		bool wasLocked = win->isRotationAxisLocked();
		bool isLocked = !wasLocked;

		static CCVector3d s_lastAxis(0.0, 0.0, 1.0);
		if (isLocked)
		{
			ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastAxis.x, s_lastAxis.y, s_lastAxis.z, 4, tr("Lock rotation axis"), this);
			if (axisDlg.buttonBox->button(QDialogButtonBox::Ok))
				axisDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
			if (!axisDlg.exec())
				return;
			s_lastAxis.x = axisDlg.doubleSpinBox1->value();
			s_lastAxis.y = axisDlg.doubleSpinBox2->value();
			s_lastAxis.z = axisDlg.doubleSpinBox3->value();
		}
		win->lockRotationAxis(isLocked, s_lastAxis);

		m_UI->actionLockRotationAxis->blockSignals(true);
		m_UI->actionLockRotationAxis->setChecked(isLocked);
		m_UI->actionLockRotationAxis->blockSignals(false);

		if (isLocked)
		{
			win->displayNewMessage(tr("[ROTATION LOCKED]"), ccGLWindow::UPPER_CENTER_MESSAGE, false, 24 * 3600, ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		else
		{
			win->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, ccGLWindow::ROTAION_LOCK_MESSAGE);
		}
		win->redraw(true, false);
	}
}

void MainWindow::doActionEnableBubbleViewMode()
{
	//special case: the selected entity is a TLS sensor or a cloud with a TLS sensor
	if (m_ccRoot)
	{
		ccHObject::Container selectedEntities;
		m_ccRoot->getSelectedEntities(selectedEntities);

		if (selectedEntities.size() == 1)
		{
			ccHObject* ent = selectedEntities.front();
			ccGBLSensor* sensor = nullptr;
			if (ent->isA(CC_TYPES::GBL_SENSOR))
			{
				sensor = static_cast<ccGBLSensor*>(ent);
			}
			else if (ent->isA(CC_TYPES::POINT_CLOUD))
			{
				ccHObject::Container sensors;
				ent->filterChildren(sensors, false, CC_TYPES::GBL_SENSOR, true);
				if (sensors.size() >= 1)
				{
					sensor = static_cast<ccGBLSensor*>(sensors.front());
				}
			}

			if (sensor)
			{
				sensor->applyViewport();
				return;
			}
		}
	}

	//otherwise we simply enable the bubble view mode in the active 3D view
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setBubbleViewMode(true);
		win->redraw(false);
	}
}

void MainWindow::doActionDeleteShader()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setShader(nullptr);
	}
}

void MainWindow::removeFromDB(ccHObject* obj, bool autoDelete/*=true*/)
{
	if (!obj)
		return;

	//remove dependency to avoid deleting the object when removing it from DB tree
	if (!autoDelete && obj->getParent())
		obj->getParent()->removeDependencyWith(obj);

	if (m_ccRoot)
		m_ccRoot->removeElement(obj);
}

void MainWindow::setSelectedInDB(ccHObject* obj, bool selected)
{
	if (obj && m_ccRoot)
	{
		if (selected)
			m_ccRoot->selectEntity(obj);
		else
			m_ccRoot->unselectEntity(obj);
	}
}

void MainWindow::addToDB(ccHObject* obj,
	bool updateZoom/*=true*/,
	bool autoExpandDBTree/*=true*/,
	bool checkDimensions/*=true*/,
	bool autoRedraw/*=true*/)
{
	//[!]将点云加到model
    ccPointCloud* transCloud = dynamic_cast<ccPointCloud*>(obj);
    if (transCloud)
    {
        CS::Model::ProjectModel::instance()->addPointClouds(transCloud);
    }

	//let's check that the new entity is not too big nor too far from scene center!
	if (checkDimensions)
	{
		//get entity bounding box
		ccBBox bBox = obj->getBB_recursive();

		CCVector3 center = bBox.getCenter();
		PointCoordinateType diag = bBox.getDiagNorm();

		CCVector3d P = center;
		CCVector3d Pshift(0, 0, 0);
		double scale = 1.0;
		bool preserveCoordinateShift = true;
		//here we must test that coordinates are not too big whatever the case because OpenGL
		//really doesn't like big ones (even if we work with GLdoubles :( ).
		if (ccGlobalShiftManager::Handle(P, diag, ccGlobalShiftManager::DIALOG_IF_NECESSARY, false, Pshift, &preserveCoordinateShift, &scale))
		{
			bool needRescale = (scale != 1.0);
			bool needShift = (Pshift.norm2() > 0);

			if (needRescale || needShift)
			{
				ccGLMatrix mat;
				mat.toIdentity();
				mat.data()[0] = mat.data()[5] = mat.data()[10] = static_cast<float>(scale);
				mat.setTranslation(Pshift);
				obj->applyGLTransformation_recursive(&mat);
				ccConsole::Warning(tr("Entity '%1' has been translated: (%2,%3,%4) and rescaled of a factor %5 [original position will be restored when saving]").arg(obj->getName()).arg(Pshift.x, 0, 'f', 2).arg(Pshift.y, 0, 'f', 2).arg(Pshift.z, 0, 'f', 2).arg(scale, 0, 'f', 6));
			}

			//update 'global shift' and 'global scale' for ALL clouds recursively
			if (preserveCoordinateShift)
			{
				//FIXME: why don't we do that all the time by the way?!
				ccHObject::Container children;
				children.push_back(obj);
				while (!children.empty())
				{
					ccHObject* child = children.back();
					children.pop_back();

					if (child->isKindOf(CC_TYPES::POINT_CLOUD))
					{
						ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(child);
						pc->setGlobalShift(pc->getGlobalShift() + Pshift);
						pc->setGlobalScale(pc->getGlobalScale() * scale);
					}

					for (unsigned i = 0; i < child->getChildrenNumber(); ++i)
					{
						children.push_back(child->getChild(i));
					}
				}
			}
		}
	}
	//add object to DB root
	if (m_ccRoot)
	{
		//force a 'global zoom' if the DB was emtpy!
		if (!m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
		{
			updateZoom = true;
		}
		m_ccRoot->addElement(obj, autoExpandDBTree);
	}
	else
	{
		ccLog::Warning(tr("[MainWindow::addToDB] Internal error: no associated DB?!"));
		assert(false);
	}

	//we can now set destination display (if none already)
    if (!obj)
    {
        return;
    }
	if (!obj->getDisplay())
	{
        ccGLWindow* activeWin =  GetGLWindow("3D View");
		if (!activeWin)
		{
            qDebug() << "[DISPLAY]get 3d view failed ";
            activeWin = getActiveGLWindow();
            if (!activeWin)
            {
                qDebug() << "[DISPLAY]get glwindow error";
                return;
            }
		}
        qDebug() << "[DISPLAY]ADD FILE display success: "<< activeWin->windowTitle();
        obj->setDisplay_recursive(activeWin);
        obj->prepareDisplayForRefresh_recursive();
	}

	//eventually we update the corresponding display
	assert(obj->getDisplay());
	if (updateZoom)
	{
		static_cast<ccGLWindow*>(obj->getDisplay())->zoomGlobal(true); //automatically calls ccGLWindow::redraw
	}
	else if (autoRedraw)
	{
		obj->redrawDisplay();
	}

	//[!]自适应标志<加载>
	{
		bool autoSizeState = static_cast<bool>(ccGui::Parameters().m_globalpointsize);
		ccPointCloud* pCloudParent = dynamic_cast<ccPointCloud*>(obj);
		if (pCloudParent) {
			pCloudParent->setPointSizeAdaptively(!autoSizeState);
			pCloudParent->setPointSize(0);
		}
		std::vector<ccHObject *> childrens;
		obj->filterChildren(childrens, true, CC_TYPES::POINT_CLOUD);
        for (auto it = childrens.begin(); it != childrens.end(); ++it) {
            ccHObject* obj = *it;
			ccPointCloud* pCloud = dynamic_cast<ccPointCloud*>(*it);
            if (pCloud) {
				pCloud->setPointSizeAdaptively(!autoSizeState);
                pCloud->setPointSize(0);
			}
        }
	}
}

void MainWindow::updateCloseAllButtonState()
{
    if (m_ccRoot->getRootEntity()->getChildrenNumber() > 0)
    {
        m_UI->actionCloseAll->setDisabled(false);
    }
    else
    {
        m_UI->actionCloseAll->setDisabled(true);
    }
}

void MainWindow::onExclusiveFullScreenToggled(bool state)
{
	//we simply update the fullscreen action method icon (whatever the window)
	ccGLWindow* win = getActiveGLWindow();

	if (win == nullptr)
		return;

	m_UI->actionExclusiveFullScreen->blockSignals(true);
	m_UI->actionExclusiveFullScreen->setChecked(win ? win->exclusiveFullScreen() : false);
	m_UI->actionExclusiveFullScreen->blockSignals(false);

	if (!state
		&&	win->stereoModeIsEnabled()
		&& (win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
			|| win->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY))
	{
		//auto disable stereo mode as NVidia Vision only works in full screen mode!
		m_UI->actionEnableStereo->setChecked(false);
	}
}

ccHObject* MainWindow::loadFile(QString filename, bool silent)
{
	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = silent ? false : true;
		parameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		parameters.parentWidget = silent ? nullptr : this;
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result);
    if (result != CC_FERR_NO_ERROR && result != CC_FERR_CANCELED_BY_USER && !silent)
    {
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Critical, tr("Error"), QCoreApplication::translate("MainWindow", "Open failed!File type unsupported or data read error.", nullptr), QMessageBox::Ok, this);
        message_box.exec();
    }
	return newGroup;

}

void MainWindow::addToDBAuto(const QStringList& filenames)
{
	ccGLWindow* win = qobject_cast<ccGLWindow*>(QObject::sender());

	addToDB(filenames, QString(), win);
}



void MainWindow::addToDB(const QStringList& filenames,
	QString fileFilter/*=QString()*/,
	ccGLWindow* destWin/*=nullptr*/)
{
	//to use the same 'global shift' for multiple files
	CCVector3d loadCoordinatesShift(0, 0, 0);
	bool loadCoordinatesTransEnabled = false;

	FileIOFilter::LoadParameters parameters;
	{
		parameters.alwaysDisplayLoadDialog = qApp->property("LoadOrSaveFileDialogDisplay").toBool();
		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
		parameters._coordinatesShift = &loadCoordinatesShift;
		parameters._coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
        parameters.parentWidget = qApp->property("LoadOrSaveFileDialogDisplay").toBool() ? this : nullptr;
	}

	bool normalsDisplayedByDefault = ccOptions::Instance().normalsDisplayedByDefault;
	FileIOFilter::ResetSesionCounter();
	QString suffixstr = "*.txt *.asc *.neu *.xyz *.pts *.csv *.ply *.las *.laz";
	for (const QString &filename : filenames)
	{
		if (parameters.parentWidget && suffixstr.contains(QFileInfo(filename).suffix(), Qt::CaseInsensitive))
		{
			parameters.progress = new ccProgressDialog(true, parameters.parentWidget);
		}
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result, fileFilter);
		if (newGroup)
		{
            m_IsOpenFilesPath.append(filename);
			if (!normalsDisplayedByDefault)
			{
				//disable the normals on all loaded clouds!
				ccHObject::Container clouds;
				newGroup->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
			}

			if (destWin)
			{
				newGroup->setDisplay_recursive(destWin);
			}
			addToDB(newGroup, true, true, true);
            QMetaObject::invokeMethod(m_recentFiles, [=]() {
                m_recentFiles->addFilePath(filename);
                refreshAll(false);
                update();
            }, Qt::AutoConnection);
            emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
		}

		if (result == CC_FERR_CANCELED_BY_USER)
		{
			//stop importing the file if the user has cancelled the current process!
			break;
		}
		if (!newGroup)
		{
			qDebug()<<QString("Open Failed11: %1'").arg(int(result));
            QMetaObject::invokeMethod(this, [result]() {
                if (result == CC_FERR_NOT_ENOUGH_MEMORY)
                {
                    CS::Widgets::FramelessMessageBox::critical(s_instance, tr("Error"), tr("Out of memory, failed to open."));
                    return;
                }

                CS::Widgets::FramelessMessageBox::critical(s_instance, tr("Error"), tr("Open failed!File type unsupported or data read error."));
            }, Qt::AutoConnection);
		}
		if (parameters.progress)
		{
			QEventLoop loop;
			connect(GetActiveGLWindow(), &ccGLWindow::signalPainteOnce, &loop, &QEventLoop::quit);
			loop.exec();
			delete parameters.progress;
			parameters.progress = nullptr;
		}
	}

    QMetaObject::invokeMethod(m_ccRoot, [=]() {
        if (m_ccRoot->getRootEntity()->getChildrenNumber() > 0)
        {
            m_UI->actionCloseAll->setDisabled(false);
        }
    }, Qt::AutoConnection);
    qDebug() << "add_file success";
}

void MainWindow::handleNewLabel(ccHObject* entity)
{
	if (entity)
	{
		addToDB(entity);
	}
	else
	{
		assert(false);
	}
}

void MainWindow::forceConsoleDisplay()
{
	//if the console is hidden, we autoamtically display it!
	/*if (m_UI->DockableConsole && m_UI->DockableConsole->isHidden())
	{
		m_UI->DockableConsole->show();
		QApplication::processEvents();
	}*/
}

ccColorScalesManager* MainWindow::getColorScalesManager()
{
	return ccColorScalesManager::GetUniqueInstance();
}

void MainWindow::closeAll()
{
	if (!m_ccRoot)
	{
		return;
	}

	CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question,
		tr("Close All"),
		tr("Are you sure you want to delete all loaded entities?"),
		QMessageBox::Ok | QMessageBox::Cancel,
		this);

	if (message_box.exec() == QMessageBox::Cancel)
	{
		return;
	}

	m_ccRoot->unloadAll();
	m_UI->actionCloseAll->setDisabled(true);
	redrawAll(false);
}

void MainWindow::doActionAddChildObj(ccHObject* pObj)
{
    //persistent settings
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup(ccPS::LoadFile());
    QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
    QString currentOpenDlgFilter = "CAD DXF geometry (*.dxf)";//settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();

    // Add all available file I/O filters (with import capabilities)
    QStringList filterStrings;// = FileIOFilter::ImportFilterList();
    filterStrings.push_back(currentOpenDlgFilter);
    const QString &allFilter = filterStrings.at(0);

    //if (!filterStrings.contains(currentOpenDlgFilter))
    //{
    //    currentOpenDlgFilter = allFilter;
    //}
    if (currentPath.isEmpty())
    {
        currentPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    }
    //file choosing dialog
    QStringList selectedFiles = CS::Widgets::FramelessFileDialog::getOpenFileNames(this, tr("Open file"), currentPath, filterStrings.join(s_fileFilterSeparator), &currentOpenDlgFilter);
    if (selectedFiles.isEmpty())
        return;

    //save last loading parameters
    //currentPath = QFileInfo(selectedFiles[0]).absolutePath();
    //settings.setValue(ccPS::CurrentPath(), currentPath);
    //settings.setValue(ccPS::SelectedInputFilter(), currentOpenDlgFilter);
    //settings.endGroup();

    //if (currentOpenDlgFilter == allFilter)
    //{
    //    currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!
    //}

    //QTime time;
    //time.start();

    //load files
    //for (auto curFileName : selectedFiles)
    //{
    //	QStringList selectedCurFile;
    //	selectedCurFile.push_back(curFileName);
    //	if (curFileName.contains(".pcd") || currentOpenDlgFilter.contains("pcd"))
    //	{
    //		WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));
    //	}
    //	addToDB(selectedCurFile, currentOpenDlgFilter);
    //	if (curFileName.contains(".pcd") || currentOpenDlgFilter.contains("pcd"))
    //	{
    //		WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    //	}
    //}
    if (m_ccRoot->getRootEntity()->getChildrenNumber() > 0)
    {
        m_UI->actionCloseAll->setDisabled(false);
    }
}

void MainWindow::doActionLoadFile()
{
	//persistent settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();
	qApp->setProperty("openfile", 1);
	// Add all available file I/O filters (with import capabilities)
	const QStringList filterStrings = FileIOFilter::ImportFilterList();
	const QString &allFilter = filterStrings.at(0);

	if (!filterStrings.contains(currentOpenDlgFilter))
	{
		currentOpenDlgFilter = allFilter;
	}
	if (currentPath.isEmpty())
	{
		currentPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
	}
	//file choosing dialog
	QStringList selectedFiles = CS::Widgets::FramelessFileDialog::getOpenFileNames(this, tr("Open file"), currentPath, filterStrings.join(s_fileFilterSeparator), &currentOpenDlgFilter);
	if (selectedFiles.isEmpty())
		return;

	//save last loading parameters
	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.setValue(ccPS::SelectedInputFilter(), currentOpenDlgFilter);
	settings.endGroup();

	if (currentOpenDlgFilter == allFilter)
	{
		currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!
	}

	//QTime time;
	//time.start();

	//load files
	//for (auto curFileName : selectedFiles)
	//{
	//	QStringList selectedCurFile;
	//	selectedCurFile.push_back(curFileName);
	//	if (curFileName.contains(".pcd") || currentOpenDlgFilter.contains("pcd"))
	//	{
	//		WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));
	//	}
	//	addToDB(selectedCurFile, currentOpenDlgFilter);
	//	if (curFileName.contains(".pcd") || currentOpenDlgFilter.contains("pcd"))
	//	{
	//		WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
	//	}
	//}
	addToDB(selectedFiles, currentOpenDlgFilter);
	if (m_ccRoot->getRootEntity()->getChildrenNumber() > 0)
	{
		m_UI->actionCloseAll->setDisabled(false);
	}
    std::vector<ccHObject*> children;
    m_ccRoot->getRootEntity()->filterChildren(children,true,CC_TYPES::POINT_CLOUD);
    for(ccHObject* pObj : children)
    {
        ccPointCloud* pCloud = ccHObjectCaster::ToPointCloud(pObj);
        if (pCloud)
        {
            CS::Model::ProjectModel::instance()->addPointClouds(pCloud);
        }
    }
	//ccLog::Print(tr("\tloadFile Time: %1 s").arg(time.elapsed() / 1000.0));
}

//Helper: check for a filename validity
static bool IsValidFileName(QString filename)
{
#ifdef CC_WINDOWS
	QString sPattern("^(?!^(PRN|AUX|CLOCK\\$|NUL|CON|COM\\d|LPT\\d|\\..*)(\\..+)?$)[^\\x00-\\x1f\\\\?*:\\"";|/]+$");
#else
	QString sPattern("^(([a-zA-Z]:|\\\\)\\\\)?(((\\.)|(\\.\\.)|([^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?))\\\\)*[^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?$");
#endif

	return QRegExp(sPattern).exactMatch(filename);
}

void MainWindow::doActionSaveFile()
{
	if (!haveSelection())
		return;

	ccHObject clouds("clouds");
	ccHObject meshes("meshes");
	ccHObject images("images");
	ccHObject polylines("polylines");
	ccHObject other("other");
	ccHObject otherSerializable("serializable");
	ccHObject::Container entitiesToDispatch;
	entitiesToDispatch.insert(entitiesToDispatch.begin(), m_selectedEntities.begin(), m_selectedEntities.end());
	ccHObject entitiesToSave;
	while (!entitiesToDispatch.empty())
	{
		ccHObject* child = entitiesToDispatch.back();
		entitiesToDispatch.pop_back();

		if (child->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned j = 0; j < child->getChildrenNumber(); ++j)
				entitiesToDispatch.push_back(child->getChild(j));
		}
		else
		{
			//we put the entity in the container corresponding to its type
			ccHObject* dest = nullptr;
			if (child->isA(CC_TYPES::POINT_CLOUD))
				dest = &clouds;
			else if (child->isKindOf(CC_TYPES::MESH))
				dest = &meshes;
			else if (child->isKindOf(CC_TYPES::IMAGE))
				dest = &images;
			else if (child->isKindOf(CC_TYPES::POLY_LINE))
				dest = &polylines;
			else if (child->isSerializable())
				dest = &otherSerializable;
			else
				dest = &other;

			assert(dest);

			//we don't want double insertions if the user has clicked both the father and child
			if (!dest->find(child->getUniqueID()))
			{
				dest->addChild(child, ccHObject::DP_NONE);
				entitiesToSave.addChild(child, ccHObject::DP_NONE);
			}
		}
	}

	bool hasCloud = (clouds.getChildrenNumber() != 0);
	bool hasMesh = (meshes.getChildrenNumber() != 0);
	bool hasImages = (images.getChildrenNumber() != 0);
	bool hasPolylines = (polylines.getChildrenNumber() != 0);
	bool hasSerializable = (otherSerializable.getChildrenNumber() != 0);
	bool hasOther = (other.getChildrenNumber() != 0);

	int stdSaveTypes = static_cast<int>(hasCloud)
		+ static_cast<int>(hasMesh)
		+ static_cast<int>(hasImages)
		+ static_cast<int>(hasPolylines)
		+ static_cast<int>(hasSerializable);
	if (stdSaveTypes == 0)
	{
		ccConsole::Error(tr("Can't save selected entity(ies) this way!"));
		return;
	}

	//we set up the right file filters, depending on the selected
	//entities type (cloud, mesh, etc.).
    bool bHasVectors = false;
	QStringList fileFilters;
	{
		for (const FileIOFilter::Shared &filter : FileIOFilter::GetFilters())
		{
			bool atLeastOneExclusive = false;
			//can this filter export one or several clouds?
			bool canExportClouds = true;
			if (hasCloud)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportClouds = (filter->canSave(CC_TYPES::POINT_CLOUD, multiple, isExclusive)
					&& (multiple || clouds.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several meshes?
			bool canExportMeshes = true;
			if (hasMesh)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportMeshes = (filter->canSave(CC_TYPES::MESH, multiple, isExclusive)
					&& (multiple || meshes.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several polylines?
			bool canExportPolylines = true;
			if (hasPolylines)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportPolylines = (filter->canSave(CC_TYPES::POLY_LINE, multiple, isExclusive)
					&& (multiple || polylines.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several images?
			bool canExportImages = true;
			if (hasImages)
			{
				bool isExclusive = true;
				bool multiple = false;
				canExportImages = (filter->canSave(CC_TYPES::IMAGE, multiple, isExclusive)
					&& (multiple || images.getChildrenNumber() == 1));
				atLeastOneExclusive |= isExclusive;
			}

			//can this filter export one or several other serializable entities?
			bool canExportSerializables = true;
			if (hasSerializable)
			{
				//check if all entities have the same type
				{
					CC_CLASS_ENUM firstClassID = otherSerializable.getChild(0)->getUniqueID();
					for (unsigned j = 1; j < otherSerializable.getChildrenNumber(); ++j)
					{
						if (otherSerializable.getChild(j)->getUniqueID() != firstClassID)
						{
							//we add a virtual second 'stdSaveType' so as to properly handle exlusivity
							++stdSaveTypes;
							break;
						}
					}
				}

				for (unsigned j = 0; j < otherSerializable.getChildrenNumber(); ++j)
				{
					ccHObject* child = otherSerializable.getChild(j);
					bool isExclusive = true;
					bool multiple = false;
					canExportSerializables &= (filter->canSave(child->getClassID(), multiple, isExclusive)
						&& (multiple || otherSerializable.getChildrenNumber() == 1));
					atLeastOneExclusive |= isExclusive;
				}
			}

			bool useThisFilter = canExportClouds
				&& canExportMeshes
				&&	canExportImages
				&&	canExportPolylines
				&&	canExportSerializables
				&& (!atLeastOneExclusive || stdSaveTypes == 1);

			if (useThisFilter)
			{
				QStringList ff = filter->getFileFilters(false);
				for (int j = 0; j < ff.size(); ++j)
					fileFilters.append(ff[j]);
			}
		}
	}
    bool bChildHasVectors = false;
    bool bIs2dSurface = false;
    for (unsigned j = 0; j < entitiesToSave.getChildrenNumber(); ++j)
    {
        ccHObject* child = entitiesToSave.getChild(j);
        if (child->getMetaData("is2dSurface").toBool())
        {
            bIs2dSurface = true;
            ccHObject* pNextChild = child->getChild(0);
            if (pNextChild &&pNextChild->getMetaData("graphicsentitydxf").toBool())
            {
                bChildHasVectors = true;
                break;
            }
        }
        if (child->getMetaData("graphicsentitydxf").toBool())
        {
            bHasVectors = true;
            break;
        }
    }

    if (bHasVectors && fileFilters.contains("Model entities(*.bin)"))
    {
        fileFilters.clear();
        fileFilters.append("CAD DXF geometry (*.dxf)");
        //fileFilters.removeAt(fileFilters.indexOf("Model entities(*.bin)"));
    }
    else if (bChildHasVectors)
    {
        int index = fileFilters.indexOf("CAD DXF geometry (*.dxf)");
        fileFilters.removeAt(index);

        index = fileFilters.indexOf("Model entities(*.bin)");
        fileFilters.removeAt(index);
    }
	else
	{
		int index = fileFilters.indexOf("CAD DXF geometry (*.dxf)");
		fileFilters.removeAt(index);
	}
	//persistent settings
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::SaveFile());

    int indexLAS = fileFilters.indexOf("LAS cloud (*.las *.laz)");
    int indexBIN = fileFilters.indexOf("Model entities(*.bin)");	//default filter
	if (indexLAS >= 0 && indexBIN >= 0)
	{
		fileFilters.swap(indexLAS, indexBIN);
	}
	QString selectedFilter;
	if (fileFilters.size() > 0)
	{
		selectedFilter  = fileFilters.first();
	}
	else
	{
		selectedFilter = "Model entities(*.bin)";
	}

		
    if (hasCloud)
    {
        selectedFilter = settings.value(ccPS::SelectedOutputFilterCloud(), selectedFilter).toString();
    }
	else if (hasMesh)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterMesh(), selectedFilter).toString();
	else if (hasImages)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterImage(), selectedFilter).toString();
	else if (hasPolylines)
		selectedFilter = settings.value(ccPS::SelectedOutputFilterPoly(), selectedFilter).toString();

	//default output path (+ filename)
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
	QString fullPathName = currentPath;

	if (haveOneSelection())
	{
		//hierarchy objects have generally as name: 'filename.ext (fullpath)'
		//so we must only take the first part! (otherwise this type of name
		//with a path inside perturbs the QFileDialog a lot ;))
		QString defaultFileName(m_selectedEntities.front()->getName());
		if (m_selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			QStringList parts = defaultFileName.split(' ', QString::SkipEmptyParts);
			if (!parts.empty())
			{
				defaultFileName = parts[0];
			}
		}
		//[!]处理点云轨迹后缀问题
		if (defaultFileName.contains(".trajectory"))
		{
			QString tempName = "";
			QStringList list = defaultFileName.split(".trajectory");
			for (int i = 0; i < list.size(); i++)
			{
				tempName += list.at(i);
				if (i + 1 < list.size())
				{
					tempName += "-trajectory";
				}
			}
			if (!tempName.isEmpty())
			{
				defaultFileName = tempName;
			}
		}

		if (!IsValidFileName(defaultFileName))
		{
			ccLog::Warning(tr("[I/O] First entity's name would make an invalid filename! Can't use it..."));
           // defaultFileName = "project";
		}

		fullPathName += QString("/") + defaultFileName;
	}
    QString suffixLists = fileFilters.join(s_fileFilterSeparator);
    QString fileSuffix = QFileInfo(fullPathName).suffix();
    if (!suffixLists.contains(fileSuffix) || fileSuffix.isEmpty()) {
        foreach(const QString& filter, fileFilters) {
            QRegExp rx("\\(([^\\)]+)\\)");
            if (rx.indexIn(filter) != -1) {
               QStringList suffixes = rx.cap(1).split(' ');
               if (!suffixes.isEmpty()) {
                   QString suffixtemp = suffixes.at(0);
                   fullPathName += suffixtemp.remove("*");
                   break;
               }
            }
        }
    }
    
	//ask the user for the output filename
	QString selectedFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this,
		tr("Save file"),
		fullPathName, suffixLists, &selectedFilter);
	if (selectedFilename.isEmpty())
	{
		//process cancelled by the user
		return;
	}
	//输入保存文件名后缀与选择的后缀不一致时，添加选择的后缀
	if (!QFileInfo(selectedFilename).suffix().isEmpty() && !selectedFilter.contains(("." + QFileInfo(selectedFilename).suffix()))) 
	{
		if (!selectedFilter.isEmpty())
		{
			QStringList list = selectedFilter.split("*");
			if (list.size() > 1)
			{
				QString suffixstr = list.at(1);
				suffixstr = suffixstr.left(suffixstr.size() - 1);
				selectedFilename += QString("%1").arg(suffixstr);
			}
		}
	}

	//保存时重名检测
	{
		QString suffixstr = "";
		QString selectedFilenamestr = "";
		if (!QFileInfo(selectedFilename).suffix().isEmpty() && selectedFilter.contains(QFileInfo(selectedFilename).suffix()))
		{
			selectedFilenamestr = selectedFilename;
		}
		else if (!selectedFilter.isEmpty())
		{
			QStringList list = selectedFilter.split("*");
			if (list.size() > 1)
			{
				suffixstr = list.at(1);
				suffixstr = suffixstr.left(suffixstr.size() - 1);
			}
			selectedFilenamestr = selectedFilename + suffixstr;
		}
		QFileInfo fileInfo(selectedFilenamestr);
		if (fileInfo.exists())
		{
			if (CS::Widgets::FramelessMessageBox::question(this, tr("Overwrite file"), tr("There is a file with the same name, do you want to overwrite it?"), QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok)
			{
                //[!]新增对于同名文件后缀递增处理
                int Namenum = 1;
                QString filename_without_suffix = fileInfo.completeBaseName();
                QString suffix = fileInfo.suffix();
                while (true)
                {
                    QFileInfo fileInfo(selectedFilenamestr);
                    if (fileInfo.exists())
                    {
                        selectedFilenamestr = fileInfo.absolutePath() + "/" + filename_without_suffix +"(" +QString::number(Namenum)+")" +"." + suffix;
                        Namenum++;
                    }
                    else
                    {
                        break;
                    }
                }
                    selectedFilename = selectedFilenamestr;
			}
		}
	}



	//ignored items
	if (hasOther)
	{
		ccConsole::Warning(tr("[I/O] The following selected entities won't be saved:"));
		for (unsigned i = 0; i < other.getChildrenNumber(); ++i)
		{
			ccConsole::Warning(QString("\t- %1s").arg(other.getChild(i)->getName()));
		}
	}

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	FileIOFilter::SaveParameters parameters;
	{
        parameters.alwaysDisplaySaveDialog = qApp->property("LoadOrSaveFileDialogDisplay").toBool();
		parameters.parentWidget = qApp->property("LoadOrSaveFileDialogDisplay").toBool() ? this : nullptr;
	}
	//这些格式保存无进度条，添加等待窗
	std::vector<QString> suffixList{"ply","vtk","dxf","pcd","shp","pov","bin"};
	bool isSelectType = false;
	for (auto curStr: suffixList)
	{
		if (selectedFilter.contains(curStr))
		{
			isSelectType = true;
			break;
		}
	}
	//if (isSelectType)
	//{
 //       WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
	//	 WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));
	//}
	//specific case: BIN format
	if (selectedFilter == BinFilter::GetFileFilter())
	{
		if (haveOneSelection())
		{
			result = FileIOFilter::SaveToFile(m_selectedEntities.front(), selectedFilename, parameters, selectedFilter);
		}
		else
		{
			//we'll regroup all selected entities in a temporary group
			ccHObject tempContainer;
			ConvertToGroup(m_selectedEntities, tempContainer, ccHObject::DP_NONE);
			if (tempContainer.getChildrenNumber())
			{
				result = FileIOFilter::SaveToFile(&tempContainer, selectedFilename, parameters, selectedFilter);
			}
			else
			{
				ccLog::Warning(tr("[I/O] None of the selected entities can be saved this way..."));
				result = CC_FERR_NO_SAVE;
			}
		}
	}
	else if (entitiesToSave.getChildrenNumber() != 0)
	{
		//ignored items
		//if (hasSerializable)
		//{
		//	if (!hasOther)
		//		ccConsole::Warning(tr("[I/O] The following selected entites won't be saved:")); //display this warning only if not already done
		//	for (unsigned i = 0; i < otherSerializable.getChildrenNumber(); ++i)
		//		ccConsole::Warning(tr("\t- %1").arg(otherSerializable.getChild(i)->getName()));
		//}

		//QTime time;
		//time.start();

		result = FileIOFilter::SaveToFile(entitiesToSave.getChildrenNumber() > 1 ? &entitiesToSave : entitiesToSave.getChild(0),
			selectedFilename,
			parameters,
			selectedFilter);

		if (result != CC_FERR_NO_ERROR && result != CC_FERR_CANCELED_BY_USER)
		{
			CS::Widgets::FramelessMessageBox::warning(MainWindow::TheInstance(),
				QApplication::translate("MainWindow", "Error", nullptr),
				QApplication::translate("MainWindow", "Failed to save file.", nullptr));

		}
		//ccLog::Print(tr("\tsaveFile Time: %1 s").arg(time.elapsed() / 1000.0));
	}

	//update default filters
	if (hasCloud)
		settings.setValue(ccPS::SelectedOutputFilterCloud(), selectedFilter);
	if (hasMesh)
		settings.setValue(ccPS::SelectedOutputFilterMesh(), selectedFilter);
	if (hasImages)
		settings.setValue(ccPS::SelectedOutputFilterImage(), selectedFilter);
	if (hasPolylines)
		settings.setValue(ccPS::SelectedOutputFilterPoly(), selectedFilter);
	//if (isSelectType)
	//{
	//	 WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
	//}
	//we update current file path
	currentPath = QFileInfo(selectedFilename).absolutePath();
	settings.setValue(ccPS::CurrentPath(), currentPath);
	settings.endGroup();
}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	if (mdiWin)
	{
		ccGLWindow* myWidget = qobject_cast<ccGLWindow*>(mdiWin->widget());
		if (!myWidget)
		{
			return;
		}
		{
			const ccViewportParameters& params = myWidget->getViewportParameters();
			if (params.perspectiveView)
			{
				setActionIcon(m_PerspectiveAction, "perspectiveIcon", "perspectiveIcon", "perspectiveDisable");
				m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Perspective projection", nullptr));
			}
			else
			{
				setActionIcon(m_PerspectiveAction, "orthographicIcon", "orthographicIcon", "orthographicDisable");
				m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Orthographic projection", nullptr));
			}
			if (myWidget->objectName() == "viewer2d")
			{
				m_PerspectiveAction->setEnabled(false);
			}
			else
			{
				if (!m_isPerspectiveChangeLock)
				{
					m_PerspectiveAction->setEnabled(true);
				}
			}
		}
	}

	ccGLWindow* win = mdiWin ? GLWindowFromWidget(mdiWin->widget()) : nullptr;
	if (win)
	{
		updateViewModePopUpMenu(win);
		updatePivotVisibilityPopUpMenu(win);

		m_UI->actionLockRotationAxis->blockSignals(true);
		m_UI->actionLockRotationAxis->setChecked(win->isRotationAxisLocked());
		m_UI->actionLockRotationAxis->blockSignals(false);

		m_UI->actionEnableStereo->blockSignals(true);
		m_UI->actionEnableStereo->setChecked(win->stereoModeIsEnabled());
		m_UI->actionEnableStereo->blockSignals(false);

		m_UI->actionExclusiveFullScreen->blockSignals(true);
		m_UI->actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
		m_UI->actionExclusiveFullScreen->blockSignals(false);

		m_UI->actionShowCursor3DCoordinates->blockSignals(true);
		m_UI->actionShowCursor3DCoordinates->setChecked(win->cursorCoordinatesShown());
		m_UI->actionShowCursor3DCoordinates->blockSignals(false);

		m_UI->actionAutoPickRotationCenter->blockSignals(true);
		m_UI->actionAutoPickRotationCenter->setChecked(win->autoPickPivotAtCenter());
		m_UI->actionAutoPickRotationCenter->blockSignals(false);
	}

	m_UI->actionLockRotationAxis->setEnabled(win != nullptr);
	m_UI->actionEnableStereo->setEnabled(win != nullptr);
	m_UI->actionExclusiveFullScreen->setEnabled(win != nullptr);
}

void MainWindow::updateViewModePopUpMenu(ccGLWindow* win)
{
	if (!m_viewModePopupButton)
		return;

	//update the view mode pop-up 'top' icon
	if (win)
	{
		bool objectCentered = true;
		bool perspectiveEnabled = win->getPerspectiveState(objectCentered);

		QAction* currentModeAction = nullptr;
		if (!perspectiveEnabled)
		{
			currentModeAction = m_UI->actionSetOrthoView;
		}
		else if (objectCentered)
		{
			currentModeAction = m_UI->actionSetCenteredPerspectiveView;
		}
		else
		{
			currentModeAction = m_UI->actionSetViewerPerspectiveView;
		}

		assert(currentModeAction);
		m_viewModePopupButton->setIcon(currentModeAction->icon());
		m_viewModePopupButton->setEnabled(true);
	}
	else
	{
		m_viewModePopupButton->setIcon(QIcon());
		m_viewModePopupButton->setEnabled(false);
	}
}

void MainWindow::updatePivotVisibilityPopUpMenu(ccGLWindow* win)
{
	if (!m_pivotVisibilityPopupButton)
		return;

	//update the pivot visibility pop-up 'top' icon
	if (win)
	{
		QAction* visibilityAction = nullptr;
		switch (win->getPivotVisibility())
		{
		case ccGLWindow::PIVOT_HIDE:
			visibilityAction = m_UI->actionSetPivotOff;
			break;
		case ccGLWindow::PIVOT_SHOW_ON_MOVE:
			visibilityAction = m_UI->actionSetPivotRotationOnly;
			break;
		case ccGLWindow::PIVOT_ALWAYS_SHOW:
			visibilityAction = m_UI->actionSetPivotAlwaysOn;
			break;
		default:
			assert(false);
		}

		if (visibilityAction)
			m_pivotVisibilityPopupButton->setIcon(visibilityAction->icon());

		//pivot is not available in viewer-based perspective!
		bool objectCentered = true;
		win->getPerspectiveState(objectCentered);
		m_pivotVisibilityPopupButton->setEnabled(objectCentered);
	}
	else
	{
		m_pivotVisibilityPopupButton->setIcon(QIcon());
		m_pivotVisibilityPopupButton->setEnabled(false);
	}
}

void MainWindow::updateMenus()
{
	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		ccGLWindow* myWidget = qobject_cast<ccGLWindow*>(activeSubWindow->widget());
		if (!myWidget)
		{
			return;
		}
		if (m_mdiArea->getShowType() == FJDragableMdiArea::SHOWALL)
		{
			myWidget->setIsNeedDrawSelectRect(true);
			if (myWidget->objectName() == "viewer3d")
			{
				ccGLWindow* nextWidget = qobject_cast<ccGLWindow*>(m_mdiArea->get2dWindow()->widget());
				if (nextWidget)
				{
					nextWidget->setIsNeedDrawSelectRect(false);
				}
			}
			else
			{
				ccGLWindow* nextWidget = qobject_cast<ccGLWindow*>(m_mdiArea->get3dWindow()->widget());
				if (nextWidget)
				{
					nextWidget->setIsNeedDrawSelectRect(false);
				}
			}
		}

		redrawAll();
	}
	ccGLWindow* active3DView = getActiveGLWindow();
	bool hasMdiChild = (active3DView != nullptr);
	int mdiChildCount = getGLWindowCount();
	bool hasLoadedEntities = (m_ccRoot && m_ccRoot->getRootEntity() && m_ccRoot->getRootEntity()->getChildrenNumber() != 0);
	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities() > 0);

    //janson
    bool atLeastOneCloud = (m_ccRoot && m_ccRoot->countSelectedEntities(CC_TYPES::POINT_CLOUD) > 0);
    bool atLeastOneMesh = (m_ccRoot && m_ccRoot->countSelectedEntities(CC_TYPES::MESH) > 0);

	//General Menu
	menuEdit->setEnabled(true/*hasSelectedEntities*/);
	menuTools->setEnabled(true/*hasSelectedEntities*/);

	//3D Views Menu
	m_UI->actionClose3DView->setEnabled(hasMdiChild);
	m_UI->actionCloseAll3DViews->setEnabled(mdiChildCount != 0);
	m_UI->actionTile3DViews->setEnabled(mdiChildCount > 1);
	m_UI->actionCascade3DViews->setEnabled(mdiChildCount > 1);
	m_UI->actionNext3DView->setEnabled(mdiChildCount > 1);
	m_UI->actionPrevious3DView->setEnabled(mdiChildCount > 1);

	//Shaders & Filters display Menu
	bool shadersEnabled = (active3DView ? active3DView->areShadersEnabled() : false);
	m_UI->actionLoadShader->setEnabled(shadersEnabled);
	m_UI->actionDeleteShader->setEnabled(shadersEnabled);

	//View Menu
	m_UI->toolBarView->setEnabled(hasMdiChild);

	//oher actions
	/*m_UI->actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
	m_UI->actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);*/
    //janson
	//左侧工具栏按钮，通过2d3d区别是否可用
    //m_UI->actionSegment->setEnabled(hasMdiChild && (atLeastOneCloud || atLeastOneMesh));
	//m_UI->actionPointPicking->setEnabled(hasMdiChild && hasLoadedEntities);


    //m_UI->actionTranslateRotate->setEnabled(hasMdiChild && (atLeastOneCloud || atLeastOneMesh));

	m_UI->actionTestFrameRate->setEnabled(hasMdiChild);
	m_UI->actionRenderToFile->setEnabled(hasMdiChild);
	m_UI->actionToggleSunLight->setEnabled(hasMdiChild);
	m_UI->actionToggleCustomLight->setEnabled(hasMdiChild);
	m_UI->actionToggleCenteredPerspective->setEnabled(hasMdiChild);
	m_UI->actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);

	//plugins
	m_pluginUIManager->updateMenus();
}

void MainWindow::update3DViewsMenu()
{
	menu3DViews->clear();
	menu3DViews->addAction(m_UI->actionNew3DView);
	menu3DViews->addSeparator();
	menu3DViews->addAction(m_UI->actionZoomIn);
	menu3DViews->addAction(m_UI->actionZoomOut);
	menu3DViews->addSeparator();
	menu3DViews->addAction(m_UI->actionClose3DView);
	menu3DViews->addAction(m_UI->actionCloseAll3DViews);
	menu3DViews->addSeparator();
	menu3DViews->addAction(m_UI->actionTile3DViews);
	menu3DViews->addAction(m_UI->actionCascade3DViews);
	menu3DViews->addSeparator();
	menu3DViews->addAction(m_UI->actionNext3DView);
	menu3DViews->addAction(m_UI->actionPrevious3DView);

	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
	if (!windows.isEmpty())
	{
		//Dynamic Separator
		QAction* separator = new QAction(this);
		separator->setSeparator(true);
		menu3DViews->addAction(separator);

		int i = 0;

		for (QMdiSubWindow *window : windows)
		{
			ccGLWindow *child = GLWindowFromWidget(window->widget());

			QString text = QString("&%1 %2").arg(++i).arg(child->windowTitle());
			QAction *action = menu3DViews->addAction(text);

			action->setCheckable(true);
			action->setChecked(child == getActiveGLWindow());

			connect(action, &QAction::triggered, this, [=]() {
				setActiveSubWindow(window);
			});
		}
	}
}

void MainWindow::setActiveSubWindow(QWidget *window)
{
	if (!window || !m_mdiArea)
		return;
	m_mdiArea->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(window));
}

void MainWindow::redrawAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->redraw(only2D);
	}
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->refresh(only2D);
	}
}

void MainWindow::updateUI()
{
    if (!m_UI)
    {
        return;
    }

	updateUIWithSelection();
	updateMenus();
	updatePropertiesView();
}

void MainWindow::updatePropertiesView()
{
	if (m_ccRoot)
	{
		m_ccRoot->updatePropertiesView();
	}
}

void MainWindow::updateUIWithSelection()
{
	dbTreeSelectionInfo selInfo;

	m_selectedEntities.clear();

	if (m_ccRoot)
	{
		m_ccRoot->getSelectedEntities(m_selectedEntities, CC_TYPES::OBJECT, &selInfo);
		if (qobject_cast<ccDBRoot*>(sender()))
		{
			emit sigSelectItemChanged();
		}
	}

    //更新点数与偏移值
    {
        bool isShowLabel = (m_selectedEntities.size() == 1 && selInfo.cloudCount == 1);
        m_pShowPointNumberPos->setVisible(isShowLabel);
        m_pShowOffsetPos->setVisible(isShowLabel);
        if (isShowLabel)
        {
            ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
            if (cloud)
            {
                m_pShowPointNumberPos->setText(tr("Count") + ": " + QString::number(cloud->size()));
                m_pShowOffsetPos->setText(tr("Offset") + ": " + QString::number(cloud->getGlobalShift().x, 'f', 3) + ", " + QString::number(cloud->getGlobalShift().y, 'f', 3) + ", " + QString::number(cloud->getGlobalShift().z, 'f', 3));
            }
        }
    }


	//enableUIItems(selInfo);
	//addbycarl如果删光了，close all按钮失效
	if (m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
	{
		m_UI->actionCloseAll->setDisabled(true);
	}
    
}

void MainWindow::enableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(true);
	}
}

void MainWindow::disableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(false);
	}
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
	//we disable all other windows
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		if (GLWindowFromWidget(window->widget()) != win)
		{
			window->setEnabled(false);
		}
	}

}

void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
{
    if (!m_UI)
    {
        return;
    }
	if (!m_UI->DockableDBTree->isEnabled())
	{
		return;
	}

	bool dbIsEmpty = (!m_ccRoot || !m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0);
	bool atLeastOneEntity = (selInfo.selCount > 0);
	bool atLeastOneCloud = (selInfo.cloudCount > 0);
	bool atLeastOneMesh = (selInfo.meshCount > 0);
	//bool atLeastOneOctree = (selInfo.octreeCount > 0);
	bool atLeastOneNormal = (selInfo.normalsCount > 0);
	bool atLeastOneColor = (selInfo.colorCount > 0);
	bool atLeastOneSF = (selInfo.sfCount > 0);
	bool atLeastOneGrid = (selInfo.gridCound > 0);

	//bool atLeastOneSensor = (selInfo.sensorCount > 0);
	bool atLeastOneGBLSensor = (selInfo.gblSensorCount > 0);
	bool atLeastOneCameraSensor = (selInfo.cameraSensorCount > 0);
	bool atLeastOnePolyline = (selInfo.polylineCount > 0);
	bool activeWindow = (getActiveGLWindow() != nullptr);
    bool active3dWin = activeWindow ? (getActiveGLWindow()->objectName() == "viewer3d") : false;//当前窗口是否为3d窗口 janson
    //[!]判断2d窗口
    bool activew2dWin = activeWindow ? (getActiveGLWindow()->objectName() == "viewer2d") : false;

	//menuEdit->setEnabled(atLeastOneEntity);
	//menuTools->setEnabled(atLeastOneEntity);

	m_UI->actionTracePolyline->setEnabled(!dbIsEmpty);
	m_UI->actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
	m_UI->actionSave->setEnabled(atLeastOneEntity);
	m_UI->actionClone->setEnabled(atLeastOneEntity);
	m_UI->actionDelete->setEnabled(atLeastOneEntity);
	m_UI->actionExportCoordToSF->setEnabled(atLeastOneEntity);
	m_UI->actionExportNormalToSF->setEnabled(atLeastOneNormal);
	//m_UI->actionSegment->setEnabled(atLeastOneEntity && activeWindow);
	//左侧工具栏按钮，通过2d3d区别是否可用
	//m_UI->actionSegment->setEnabled((atLeastOneCloud | atLeastOneMesh) && activeWindow);//janson
	//m_UI->actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
	m_UI->actionTranslateRotate->setEnabled((atLeastOneCloud | atLeastOneMesh) && active3dWin && (selInfo.cloudCount + selInfo.meshCount) == m_selectedEntities.size());//janson
	m_UI->actionShowDepthBuffer->setEnabled(atLeastOneGBLSensor);
	m_UI->actionExportDepthBuffer->setEnabled(atLeastOneGBLSensor);
	m_UI->actionComputePointsVisibility->setEnabled(atLeastOneGBLSensor);
	m_UI->actionResampleWithOctree->setEnabled(atLeastOneCloud);
	m_UI->actionApplyScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	m_UI->actionApplyTransformation->setEnabled(atLeastOneEntity);
	m_UI->actionComputeOctree->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionChangeColorLevels->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionEditGlobalShiftAndScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
	m_UI->actionCrop->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionSetUniqueColor->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	m_UI->actionSetColorGradient->setEnabled(atLeastOneCloud || atLeastOneMesh);
	m_UI->actionColorize->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
	m_UI->actionDeleteScanGrid->setEnabled(atLeastOneGrid);

	m_UI->actionScalarFieldFromColor->setEnabled(atLeastOneEntity && atLeastOneColor);
	m_UI->actionComputeMeshAA->setEnabled(atLeastOneCloud);
	m_UI->actionComputeMeshLS->setEnabled(selInfo.cloudCount == 1 && selInfo.cloudCount == m_selectedEntities.size());
	m_actionComputeSurfaceMesh->setEnabled(atLeastOneCloud);
	m_UI->actionMeshScanGrids->setEnabled(atLeastOneGrid);
	//actionComputeQuadric3D->setEnabled(atLeastOneCloud);
	m_UI->actionComputeBestFitBB->setEnabled(atLeastOneEntity);
	m_UI->actionComputeGeometricFeature->setEnabled(atLeastOneCloud);
	m_UI->actionRemoveDuplicatePoints->setEnabled(atLeastOneCloud);
	m_UI->actionFitPlane->setEnabled(atLeastOneEntity);
	m_UI->actionFitPlaneProxy->setEnabled(atLeastOneEntity);
	m_UI->actionFitSphere->setEnabled(atLeastOneCloud);
	m_UI->actionLevel->setEnabled(atLeastOneEntity);
	m_UI->actionFitFacet->setEnabled(atLeastOneEntity);
	m_UI->actionFitQuadric->setEnabled(atLeastOneCloud);
	m_UI->actionSubsample->setEnabled(atLeastOneCloud && selInfo.cloudCount == m_selectedEntities.size());

	m_UI->actionSNETest->setEnabled(atLeastOneCloud);
	m_UI->actionExportCloudInfo->setEnabled(atLeastOneEntity);
	m_UI->actionExportPlaneInfo->setEnabled(atLeastOneEntity);

	m_UI->actionFilterByValue->setEnabled(atLeastOneSF);
	m_UI->actionConvertToRGB->setEnabled(atLeastOneSF && !atLeastOneMesh);
	m_UI->actionConvertToRandomRGB->setEnabled(atLeastOneSF);
	m_UI->actionRenameSF->setEnabled(atLeastOneSF);
	m_UI->actionAddIdField->setEnabled(atLeastOneCloud);
	m_UI->actionSplitCloudUsingSF->setEnabled(atLeastOneSF);
	m_UI->actionComputeStatParams->setEnabled(atLeastOneSF);
	m_UI->actionComputeStatParams2->setEnabled(atLeastOneSF);
	m_UI->actionShowHistogram->setEnabled(atLeastOneSF);
	m_UI->actionGaussianFilter->setEnabled(atLeastOneSF);
	m_UI->actionBilateralFilter->setEnabled(atLeastOneSF);
	m_UI->actionDeleteScalarField->setEnabled(atLeastOneSF);
	m_UI->actionDeleteAllSF->setEnabled(atLeastOneSF);
	m_UI->actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false);
	m_UI->actionSFGradient->setEnabled(atLeastOneSF);
	m_UI->actionSetSFAsCoord->setEnabled(atLeastOneSF && atLeastOneCloud);
	m_UI->actionInterpolateSFs->setEnabled(atLeastOneCloud || atLeastOneMesh);

	m_UI->actionSamplePointsOnMesh->setEnabled(atLeastOneMesh && m_selectedEntities.size() == 1);
	m_UI->actionMeasureMeshSurface->setEnabled(atLeastOneMesh);
	m_UI->actionFlagMeshVertices->setEnabled(atLeastOneMesh);
	m_UI->actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh && m_selectedEntities.size()==1);
	m_UI->actionConvertTextureToColor->setEnabled(atLeastOneMesh);
	m_UI->actionSubdivideMesh->setEnabled(atLeastOneMesh);
	m_UI->actionFlipMeshTriangles->setEnabled(atLeastOneMesh);
	m_UI->actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
	m_UI->actionDistanceMap->setEnabled(atLeastOneMesh || atLeastOneCloud);

	/*m_UI->*/menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
	//actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);

	m_UI->actionOrientNormalsMST->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionOrientNormalsFM->setEnabled(atLeastOneCloud && atLeastOneNormal);
	m_UI->actionClearNormals->setEnabled(atLeastOneNormal);
	m_UI->actionInvertNormals->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
	m_UI->actionConvertNormalToDipDir->setEnabled(atLeastOneNormal);
	m_UI->actionClearColor->setEnabled(atLeastOneColor);
	m_UI->actionRGBToGreyScale->setEnabled(atLeastOneColor);
	m_UI->actionEnhanceRGBWithIntensities->setEnabled(atLeastOneColor);
	m_UI->actionColorFromScalarField->setEnabled(atLeastOneSF);
	// == 1
	bool exactlyOneEntity = (selInfo.selCount == 1);
	bool exactlyOneGroup = (selInfo.groupCount == 1);
	bool exactlyOneCloud = (selInfo.cloudCount == 1 && selInfo.cloudCount == m_selectedEntities.size());
	bool exactlyOneMesh = (selInfo.meshCount == 1);
	bool exactlyOneSF = (selInfo.sfCount == 1);
	bool exactlyOneSensor = (selInfo.sensorCount == 1);
	bool exactlyOneCameraSensor = (selInfo.cameraSensorCount == 1);
	m_UI->actionMeasureMeshVolume->setEnabled(atLeastOneMesh && exactlyOneEntity);
	m_UI->actionConvertPolylinesToMesh->setEnabled(atLeastOnePolyline || exactlyOneGroup);
	m_UI->actionSamplePointsOnPolyline->setEnabled(atLeastOnePolyline);
	m_UI->actionSmoothPolyline->setEnabled(atLeastOnePolyline);
    m_UI->actionSORFilter->setEnabled(exactlyOneCloud);

	m_UI->actionMeshTwoPolylines->setEnabled(selInfo.selCount == 2 && selInfo.polylineCount == 2);
	m_UI->actionCreateSurfaceBetweenTwoPolylines->setEnabled(m_UI->actionMeshTwoPolylines->isEnabled()); //clone of actionMeshTwoPolylines
	m_UI->actionModifySensor->setEnabled(exactlyOneSensor);
	m_UI->actionComputeDistancesFromSensor->setEnabled(atLeastOneCameraSensor || atLeastOneGBLSensor);
	m_UI->actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
	m_UI->actionViewFromSensor->setEnabled(exactlyOneSensor);
	m_UI->actionCreateGBLSensor->setEnabled(atLeastOneCloud);
	m_UI->actionCreateCameraSensor->setEnabled(selInfo.selCount <= 1); //free now
	m_UI->actionProjectUncertainty->setEnabled(exactlyOneCameraSensor);
	m_UI->actionCheckPointsInsideFrustum->setEnabled(exactlyOneCameraSensor);
	m_UI->actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
	m_actionAccuracyActiveAxisZ->setEnabled(exactlyOneCloud);
	m_UI->actionNoiseFilter->setEnabled(atLeastOneCloud);
	m_UI->actionUnroll->setEnabled(exactlyOneEntity);
	m_UI->actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
	m_UI->actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionAddClassificationSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
	m_UI->actionShowWaveDialog->setEnabled(exactlyOneCloud);
	m_UI->actionCompressFWFData->setEnabled(atLeastOneCloud);

	m_UI->actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
	m_UI->actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);

	//actionCreatePlane->setEnabled(true);
	m_UI->actionEditPlane->setEnabled(selInfo.planeCount == 1);
	m_UI->actionFlipPlane->setEnabled(selInfo.planeCount != 0);
	m_UI->actionComparePlanes->setEnabled(selInfo.planeCount == 2);

	m_UI->actionFindBiggestInnerRectangle->setEnabled(exactlyOneCloud);

	/*m_UI->*/menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount > 0);
	m_UI->actionExtractSections->setEnabled(atLeastOneCloud);
	m_UI->actionRasterize->setEnabled(exactlyOneCloud);
	m_UI->actionCompute2HalfDimVolume->setEnabled(selInfo.cloudCount == selInfo.selCount && selInfo.cloudCount == 1); //one or two clouds!
    m_actionCompute2HalfDimVolume->setEnabled(selInfo.cloudCount == selInfo.selCount &&  selInfo.cloudCount == 2);
	m_UI->actionPointListPicking->setEnabled(exactlyOneCloud || exactlyOneMesh);

	// == 2
	bool exactlyTwoEntities = (selInfo.selCount == 2);
	bool exactlyTwoClouds = (selInfo.cloudCount == 2);
	bool selectTwoCount = (selInfo.selCount == 2);

	m_UI->actionRegister->setEnabled(exactlyTwoEntities);
	m_UI->actionInterpolateColors->setEnabled(exactlyTwoEntities && atLeastOneColor);
	m_UI->actionPointPairsAlign->setEnabled(exactlyTwoClouds && selectTwoCount);
	m_coordinateConversionAciton->setEnabled(exactlyOneCloud);
	m_AccuracyverificationAciton->setEnabled(exactlyOneCloud);

	m_UI->actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
	m_UI->actionCloudCloudDist->setEnabled(exactlyTwoClouds);
	m_UI->actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
	m_UI->actionCloudPrimitiveDist->setEnabled(atLeastOneCloud && (atLeastOneMesh || atLeastOnePolyline));
	m_UI->actionCPS->setEnabled(exactlyTwoClouds);
	m_UI->actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);

	//>1
	bool atLeastTwoEntities = (selInfo.selCount > 1);

	m_UI->actionMerge->setEnabled(atLeastTwoEntities && (selInfo.cloudCount == m_selectedEntities.size() || selInfo.meshCount == m_selectedEntities.size()));
	m_UI->actionMatchBBCenters->setEnabled(atLeastTwoEntities);
	m_UI->actionMatchScales->setEnabled(atLeastTwoEntities);
	m_UI->actionPointPicking->setEnabled((atLeastOneMesh || atLeastOneCloud) && (selInfo.cloudCount + selInfo.meshCount) == m_selectedEntities.size());
	m_UI->actionSegment->setEnabled((atLeastOneMesh || atLeastOneCloud) && (selInfo.cloudCount + selInfo.meshCount) == m_selectedEntities.size());
	//standard plugins
	m_pluginUIManager->handleSelectionChanged();

	//janson
	outRoomClass_action_->setEnabled(exactlyOneCloud && activeWindow);
	inRoomClass_action_->setEnabled(exactlyOneCloud && activeWindow);
	floorextractionClass_action_->setEnabled(exactlyOneCloud && activeWindow);
    //shangle
    actionNotes->setEnabled(exactlyOneCloud && activeWindow);
    //[!]平面切割打开的逻辑是，当切割界面打开时则disabeld,关闭则亮。
	//carl
	colorByIntensityAction->setEnabled(atLeastOneEntity && selInfo.hasScalarFieldAndContainIntensity);


    //[!]    裁剪盒判断是否含有切割文件和dxf文件
    bool b2dSurfaceDxffile = false;
    bool bHasFileNode = false;
    bool bHasTinMesh = false;
    bool bHasNoteNode = false;

    for (auto obj : m_selectedEntities)
    {
        if (obj->getMetaData("graphicsentitydxf").toBool())
        {
            b2dSurfaceDxffile = true;
        }
        if (obj->isA(CC_TYPES::HIERARCHY_OBJECT) && !bHasFileNode)
        {
            bHasFileNode = true;
        }
        if (obj->getMetaData("TIN").toBool() && !bHasTinMesh)
        {
            bHasTinMesh = true;
        }
        if (obj->getMetaData("NoteNode").toBool() && !bHasNoteNode)
        {
            bHasNoteNode = true;
        }
        
    }
    //补洞
	m_actionFillHoles->setEnabled(exactlyOneMesh && (selInfo.meshCount == m_selectedEntities.size()) && !atLeastOneCloud && !bHasFileNode && !bHasTinMesh && !bHasNoteNode);
    m_actionComputeSurfaceMesh->setEnabled(!atLeastOneMesh && exactlyOneCloud && !bHasFileNode && !bHasNoteNode);
    m_pHeightFittingAction->setEnabled(exactlyOneCloud && !b2dSurfaceDxffile && !bHasNoteNode && !bHasTinMesh);
	m_pCreatePointCloudOrthphotoAction->setEnabled(exactlyOneCloud && !b2dSurfaceDxffile && !bHasNoteNode && !bHasTinMesh);
    bool isCrossSectionenable = (atLeastOneCloud || atLeastOneMesh) && !b2dSurfaceDxffile && ((selInfo.cloudCount + selInfo.meshCount) == m_selectedEntities.size());
    if (m_isClippingBoxOpen)
    {
        isCrossSectionenable = false;
    }
    setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", isCrossSectionenable);
	if (exactlyOneCloud)
	{
        bool b2dSurface = false;
        bool bVectorsNode = false;
        if (!m_selectedEntities.empty())
        {
             b2dSurface = m_selectedEntities[0]->getMetaData("is2dSurface").toBool();
             ccGLWindow* pWin3d = GetGLWindow("3D View");
             if (pWin3d)
             {
                 if (b2dSurface)
                 {
                     pWin3d->setPerspectiveState(false, true);
                     setActionIcon(m_PerspectiveAction, "orthographicIcon", "orthographicIcon", "orthographicDisable");
                     m_PerspectiveAction->setText(QCoreApplication::translate("MainWindow", "Orthographic projection", nullptr));
                 }


                 pWin3d->redraw(false);
                 updateViewModePopUpMenu(pWin3d);
                 updatePivotVisibilityPopUpMenu(pWin3d);
             }

             bVectorsNode = m_selectedEntities[0]->getMetaData("graphicsentitydxf").toBool();
        }
		mlsaction->setEnabled(!b2dSurface & !bVectorsNode);
		m_UI->actionTranslateRotate->setEnabled(!b2dSurface & !bVectorsNode);
		m_coordinateConversionAciton->setEnabled(!b2dSurface & !bVectorsNode);
		m_UI->actionSORFilter->setEnabled(!b2dSurface & !bVectorsNode);
		m_actionAccuracyActiveAxisZ->setEnabled(!b2dSurface & !bVectorsNode);
		m_AccuracyverificationAciton->setEnabled(!b2dSurface & !bVectorsNode);
		outRoomClass_action_->setEnabled(!b2dSurface & !bVectorsNode);
		inRoomClass_action_->setEnabled(!b2dSurface & !bVectorsNode);
		floorextractionClass_action_->setEnabled(!b2dSurface & !bVectorsNode);
		actionNotes->setEnabled(exactlyOneCloud && activeWindow && !bVectorsNode);
		m_SectionAnalysisAciton->setEnabled(atLeastOneCloud && !b2dSurface);
	}
    bool ishas2dsuifaceobj = false;
    for (auto curselectedobj : m_selectedEntities)
    {
        if (curselectedobj->getMetaData("is2dSurface").toBool())
        {
            ishas2dsuifaceobj = true;
            break;
        }
    }
    if (ishas2dsuifaceobj)
    {
        m_UI->actionCompute2HalfDimVolume->setEnabled(false);
        m_actionCompute2HalfDimVolume->setEnabled(false);
    }

	//是否有矢量线节点
	bool isHasVectorsNode = false;
	for (auto obj : m_selectedEntities)
	{
		if (obj->getMetaData("graphicsentitydxf").toBool())
		{
			isHasVectorsNode = true;
		}
	}
	if (isHasVectorsNode)
	{
		m_UI->actionSubsample->setEnabled(false);
		m_UI->actionLabelConnectedComponents->setEnabled(false);
		m_UI->actionComputeMeshLS->setEnabled(false);
		m_ContourAction->setEnabled(false);
		m_UI->actionCompute2HalfDimVolume->setEnabled(false); //one or two clouds!
        m_actionCompute2HalfDimVolume->setEnabled(false);
		m_SectionAnalysisAciton->setEnabled(false);
		m_UI->actionPointPicking->setEnabled(false);
		m_actionComputeSurfaceMesh->setEnabled(false);
		m_UI->actionSegment->setEnabled(false);
	}
    if (m_psWidget)
    {
        if (atLeastOneCloud && selInfo.cloudCount == m_selectedEntities.size())
        {
            m_psWidget->setUiEnabled(true);
        }
        else
        {
            m_psWidget->setUiEnabled(false);
        }
    }

    return;
}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->onWheelEvent(wheelDelta_deg);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->rotateBaseViewMat(rotMat);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoCameraPosChanged(const CCVector3d& P)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;


	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setCameraPos(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoPivotPointChanged(const CCVector3d& P)
{
	if (!m_UI->actionEnableCameraLink->isChecked())
		return;

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setPivotPoint(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::dispToConsole(QString message, ConsoleMessageLevel level/*=STD_CONSOLE_MESSAGE*/)
{
	switch (level)
	{
	case STD_CONSOLE_MESSAGE:
		ccConsole::Print(message);
		break;
	case WRN_CONSOLE_MESSAGE:
		ccConsole::Warning(message);
		break;
	case ERR_CONSOLE_MESSAGE:
		ccConsole::Error(message);
		break;
	}
}

void MainWindow::doActionLoadShader() //TODO
{
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
}

void MainWindow::doActionKMeans()//TODO
{
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
}

void MainWindow::doActionFrontPropagation() //TODO
{
	ccConsole::Error(tr("Not yet implemented! Sorry ..."));
}

/************** STATIC METHODS ******************/

MainWindow* MainWindow::TheInstance()
{
	if (!s_instance)
	{
		s_instance = new MainWindow();
	}
	return s_instance;
}

void MainWindow::DestroyInstance()
{
	delete s_instance;
	s_instance = nullptr;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindow*>& glWindows)
{
	const QList<QMdiSubWindow*> windows = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return;

	glWindows.clear();
	glWindows.reserve(windows.size());

	for (QMdiSubWindow *window : windows)
	{
		glWindows.push_back(GLWindowFromWidget(window->widget()));
	}
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
	const QList<QMdiSubWindow *> windows = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return nullptr;

	for (QMdiSubWindow *window : windows)
	{
		ccGLWindow* win = GLWindowFromWidget(window->widget());
		if (win->windowTitle() == title)
			return win;
	}

	return nullptr;
}

void MainWindow::RefreshAllGLWindow(bool only2D/*=false*/)
{
	TheInstance()->refreshAll(only2D);
}

void MainWindow::UpdateUI()
{
	TheInstance()->updateUI();
}

void MainWindow::setDBTreeSelectObject(ccHObject* pObject)
{
    m_ccRoot->selectEntity(pObject);
    if (m_pLastObject)
    {
        //m_pLastObject->setEnabled(false);
    }
    m_pLastObject = pObject;
}

void MainWindow::setPlanecuttingStatus(bool running)
{
	m_PlaneStatus = running;
}


void MainWindow::deleteDBTreeSelectObject(ccHObject* pObject)
{
    if (pObject && pObject == m_pLastObject)
    {
        m_pLastObject = nullptr;
    }
}

void MainWindow::freeDBTreeSelectObject()
{
    m_pLastObject = nullptr;
}

void MainWindow::setLeftDBTreeEnabled(bool enable)
{
	m_UI->DockableDBTree->setDisabled(!enable);
}

void MainWindow::setViewButtonDisabled(bool enable, bool onlyviewbutton)
{
	m_UI->actionSetViewTop->setDisabled(enable);
	m_UI->actionSetViewFront->setDisabled(enable);
	m_UI->actionSetViewLeft->setDisabled(enable);
	m_UI->actionSetViewBack->setDisabled(enable);
	m_UI->actionSetViewRight->setDisabled(enable);
	m_UI->actionSetViewBottom->setDisabled(enable);
	m_UI->actionSetViewIso1->setDisabled(enable);
	m_UI->actionSetViewIso2->setDisabled(enable);
    if (!onlyviewbutton)
    {
        m_UI->actionPointPicking->setDisabled(enable);
        m_UI->actionSegment->setDisabled(enable);
    }
}

void MainWindow::hidePropertiesDlg()
{
	m_ccRoot->hidePropertiesView();
}

void MainWindow::showPropertiesDlg(ccHObject* obj)
{
	m_ccRoot->showPropertiesView(obj);
}

void MainWindow::expandElement(ccHObject* obj, bool state)
{
	m_ccRoot->expandElement(obj, state);
}


bool MainWindow::getIsExpandElement(ccHObject* object)
{
    return m_ccRoot->getIsExpandElement(object);
}

void MainWindow::show2dWindow()
{
    if (m_ccRoot)
    {
        std::vector<ccHObject*> children;
        m_ccRoot->getRootEntity()->filterChildren(children);

        for (ccHObject* pObj : children)
        {
            if (!pObj)
                continue;

            pObj->prepareDisplayForRefresh_recursive();
            pObj->setDisplay_recursive(nullptr);
            pObj->prepareDisplayForRefresh_recursive();

            std::vector<ccHObject*> lstObj;
            pObj->filterChildren(lstObj);
            for (ccHObject* cloud : lstObj)
            {

                cloud->prepareDisplayForRefresh_recursive();
                cloud->setDisplay_recursive(nullptr);
                cloud->prepareDisplayForRefresh_recursive();
            }
        }
    }

	m_mdiArea->setShowType(FJDragableMdiArea::SHOWONLY2D);
	redrawAll();
}


void MainWindow::show3dWindow()
{
    ccGLWindow* win = MainWindow::GetGLWindow("3D View");

    if (m_ccRoot && win)
    {
        std::vector<ccHObject*> children;
        m_ccRoot->getRootEntity()->filterChildren(children);

        for (ccHObject* pObj : children)
        {
            if (!pObj)
                continue;
            ccGLWindow* win1 = static_cast<ccGLWindow*>(pObj->getDisplay());
            pObj->prepareDisplayForRefresh_recursive();
            pObj->setDisplay_recursive(win);
            pObj->prepareDisplayForRefresh_recursive();
            std::vector<ccHObject*> lstObj;
            pObj->filterChildren(lstObj);
            for (ccHObject* cloud : lstObj)
            {
                ccGLWindow* win = static_cast<ccGLWindow*>(cloud->getDisplay());
                cloud->prepareDisplayForRefresh_recursive();
                cloud->setDisplay_recursive(win);
                cloud->prepareDisplayForRefresh_recursive();
            }
            win->zoomGlobal(true);
        }
    }

	m_mdiArea->setShowType(FJDragableMdiArea::SHOWONLY3D);
	redrawAll();
}


ccDBRoot* MainWindow::db()
{
	return m_ccRoot;
}

void MainWindow::addEditPlaneAction(QMenu &menu) const
{
	menu.addAction(m_UI->actionEditPlane);
}

ccHObject* MainWindow::dbRootObject()
{
	return (m_ccRoot ? m_ccRoot->getRootEntity() : nullptr);
}

ccUniqueIDGenerator::Shared MainWindow::getUniqueIDGenerator()
{
	return ccObject::GetUniqueIDGenerator();
}

void MainWindow::createGLWindow(ccGLWindow*& window, QWidget*& widget) const
{
	bool stereoMode = QSurfaceFormat::defaultFormat().stereo();

	CreateGLWindow(window, widget, stereoMode);
	assert(window && widget);
}

void MainWindow::destroyGLWindow(ccGLWindow* view3D) const
{
	if (view3D)
	{
		view3D->setParent(nullptr);
		delete view3D;
	}
}

ccMainAppInterface::ccHObjectContext MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj)
{
	ccHObjectContext context;

	assert(obj);
	if (!m_ccRoot || !obj)
		return context;

	//mandatory (to call putObjectBackIntoDBTree)
	context.parent = obj->getParent();

	//remove the object's dependency to its father (in case it undergoes 'severe' modifications)
	if (context.parent)
	{
		context.parentFlags = context.parent->getDependencyFlagsWith(obj);
		context.childFlags = obj->getDependencyFlagsWith(context.parent);

		context.parent->removeDependencyWith(obj);
		obj->removeDependencyWith(context.parent);
	}

	m_ccRoot->removeElement(obj);

	return context;
}

void MainWindow::putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context)
{
	assert(obj);
	if (!obj || !m_ccRoot)
		return;

	if (context.parent)
	{
		context.parent->addChild(obj, context.parentFlags);
		obj->addDependency(context.parent, context.childFlags);
	}

	//DGM: we must call 'notifyGeometryUpdate' as any call to this method
	//while the object was temporarily 'cut' from the DB tree were
	//ineffective!
	obj->notifyGeometryUpdate();

	m_ccRoot->addElement(obj, false);
}

void MainWindow::putObjectIntoDBTreeByIndex(ccHObject* obj, const ccHObjectContext& context, const int & index)
{
	assert(obj);
	if (!obj || !m_ccRoot)
		return;

	if (context.parent)
	{
		context.parent->addChild(obj, context.parentFlags, index);
		obj->addDependency(context.parent, context.childFlags);
	}

	//DGM: we must call 'notifyGeometryUpdate' as any call to this method
	//while the object was temporarily 'cut' from the DB tree were
	//ineffective!
	obj->notifyGeometryUpdate();

	m_ccRoot->addElement(obj, true);
}

void MainWindow::doActionGlobalShiftSeetings()
{
	QDialog dialog(this);
	Ui_GlobalShiftSettingsDialog ui;
	ui.setupUi(&dialog);

	ui.maxAbsCoordSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxCoordinateAbsValue())));
	ui.maxAbsDiagSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxBoundgBoxDiagonal())));

	if (!dialog.exec())
	{
		return;
	}

	double maxAbsCoord = pow(10.0, static_cast<double>(ui.maxAbsCoordSpinBox->value()));
	double maxAbsDiag = pow(10.0, static_cast<double>(ui.maxAbsDiagSpinBox->value()));

	ccGlobalShiftManager::SetMaxCoordinateAbsValue(maxAbsCoord);
	ccGlobalShiftManager::SetMaxBoundgBoxDiagonal(maxAbsDiag);

	ccLog::Print(tr("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
		.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
		.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));

	//save to persistent settings
	{
		QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
		settings.beginGroup(ccPS::GlobalShift());
		settings.setValue(ccPS::MaxAbsCoord(), maxAbsCoord);
		settings.setValue(ccPS::MaxAbsDiag(), maxAbsDiag);
		settings.endGroup();
	}
}

void MainWindow::doActionCompressFWFData()
{
	for (ccHObject *entity : getSelectedEntities())
	{
		if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			continue;
		}

		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		cloud->compressFWFData();
	}
}

void MainWindow::doActionShowWaveDialog()
{
	if (!haveSelection())
		return;

	ccHObject* entity = haveOneSelection() ? m_selectedEntities.front() : nullptr;
	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select one point cloud!"));
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
	if (!cloud->hasFWF())
	{
		ccConsole::Error(tr("Cloud has no associated waveform information"));
		return;
	}

	ccWaveDialog* wDlg = new ccWaveDialog(cloud, m_pickingHub, this);
	wDlg->setAttribute(Qt::WA_DeleteOnClose);
	wDlg->setModal(false);
	wDlg->show();
}

void MainWindow::doActionCreatePlane()
{
	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
	peDlg->show();
}

void MainWindow::doActionEditPlane()
{
	if (!haveSelection())
	{
		assert(false);
		return;
	}

	ccPlane* plane = ccHObjectCaster::ToPlane(m_selectedEntities.front());
	if (!plane)
	{
		assert(false);
		return;
	}

	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
	peDlg->initWithPlane(plane);
	peDlg->show();
}

void MainWindow::doActionFlipPlane()
{
	if (!haveSelection())
	{
		assert(false);
		return;
	}

	for (ccHObject* entity : m_selectedEntities)
	{
		ccPlane* plane = ccHObjectCaster::ToPlane(entity);
		if (plane)
		{
			plane->flip();
			plane->prepareDisplayForRefresh();
		}
	}

	refreshAll();
	updatePropertiesView();
}

void MainWindow::doActionComparePlanes()
{
	if (m_selectedEntities.size() != 2)
	{
		ccConsole::Error(tr("Select 2 planes!"));
		return;
	}

	if (!m_selectedEntities.front()->isKindOf(CC_TYPES::PLANE) ||
		!m_selectedEntities[1]->isKindOf(CC_TYPES::PLANE))
	{
		ccConsole::Error(tr("Select 2 planes!"));
		return;
	}

	ccPlane* p1 = ccHObjectCaster::ToPlane(m_selectedEntities.front());
	ccPlane* p2 = ccHObjectCaster::ToPlane(m_selectedEntities[1]);

	QStringList info;
	info << tr("Plane 1: %1").arg(p1->getName());
	ccLog::Print(tr("[Compare] ") + info.last());

	info << tr("Plane 2: %1").arg(p2->getName());
	ccLog::Print(tr("[Compare] ") + info.last());

	CCVector3 N1;
	CCVector3 N2;
	PointCoordinateType d1;
	PointCoordinateType d2;
	p1->getEquation(N1, d1);
	p2->getEquation(N2, d2);

	double angle_rad = N1.angle_rad(N2);
	info << tr("Angle P1/P2: %1 deg.").arg(CCCoreLib::RadiansToDegrees(angle_rad));
	ccLog::Print(tr("[Compare] ") + info.last());

	PointCoordinateType planeEq1[4] = { N1.x, N1.y, N1.z, d1 };
	PointCoordinateType planeEq2[4] = { N2.x, N2.y, N2.z, d2 };
	CCVector3 C1 = p1->getCenter();
	ScalarType distCenter1ToPlane2 = CCCoreLib::DistanceComputationTools::computePoint2PlaneDistance(&C1, planeEq2);
	info << tr("Distance Center(P1)/P2: %1").arg(distCenter1ToPlane2);
	ccLog::Print(tr("[Compare] ") + info.last());

	CCVector3 C2 = p2->getCenter();
	ScalarType distCenter2ToPlane1 = CCCoreLib::DistanceComputationTools::computePoint2PlaneDistance(&C2, planeEq1);
	info << tr("Distance Center(P2)/P1: %1").arg(distCenter2ToPlane1);
	ccLog::Print(tr("[Compare] ") + info.last());

	//pop-up summary
	CS::Widgets::FramelessMessageBox::information(this, tr("Plane comparison"), info.join("\n"));
	forceConsoleDisplay();
}

void MainWindow::createRibbonMenu()
{
	//setWindowIcon(QIcon(":/FJ/FJ_images/topStatus_slices/logo.png"));
	setWindowTitle("FJD Trion Model");
	//setStatusBar(new QStatusBar());



	ribbon = ribbonBar();


	//创建快捷键 
	QShortcut* shortcut = new QShortcut(QKeySequence("Delete"), this);
	//QObject::connect(shortcut, &QShortcut::activated, m_UI->actionDelete, &QAction::trigger);
	QShortcut* shortcut1 = new QShortcut(QKeySequence("F11"), this);
	shortcut1->setContext(Qt::ShortcutContext::ApplicationShortcut);

	//QObject::connect(shortcut1, &QShortcut::activated, m_UI->actionExclusiveFullScreen, &QAction::trigger);
	//QObject::connect(shortcut1, &QShortcut::activated, [this]() {
	//	if (!m_uiFrozen)
	//	{
	//		m_UI->actionExclusiveFullScreen->trigger();
	//	}
	//	});
	QObject::connect(shortcut, &QShortcut::activated, [this]() {
		if (!m_uiFrozen)
		{
			m_UI->actionDelete->trigger();
		}
		});
	//QShortcut* shortcutF3 = new QShortcut(QKeySequence("F3"), this);
	//QObject::connect(shortcutF3, &QShortcut::activated, [this]() {
	//	toggleActiveWindowCenteredPerspective();
	//});

    // z轴自动拉平快捷键
    //QShortcut* shortcutZaxisLevel = new QShortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_H), this);
    //QObject::connect(shortcutZaxisLevel, &QShortcut::activated, [this]() {
    //    toggleActiveZaxisLevel();
    //    });
    
	//界面风格
	//setRibbonTheme(SARibbonMainWindow::RibbonTheme::NormalTheme);
	setRibbonTheme(SARibbonMainWindow::RibbonTheme::Office2013);
	ribbon->setRibbonStyle(SARibbonBar::RibbonStyle::OfficeStyle);
	//ribbon->setRibbonStyle(SARibbonBar::RibbonStyle::WpsLiteStyle);
	//ribbon->setRibbonStyle(SARibbonBar::RibbonStyle::OfficeStyleTwoRow);
	//ribbon->setRibbonStyle(SARibbonBar::RibbonStyle::WpsLiteStyleTwoRow);

	QPushButton* fileButton = new QPushButton(ribbon);
    fileButton->setObjectName("SARibbonApplicationButton");
	ribbon->setApplicationButton(fileButton);
	ribbon->applicationButton()->setText(QCoreApplication::translate("MainWindow", "File", nullptr));

	QMenu* fileMenu = new QMenu(fileButton);
	fileButton->setMenu(fileMenu);
	fileMenu->addAction(m_UI->actionOpen);
	fileMenu->addMenu(m_recentFiles->menu());
	fileMenu->addSeparator();
	fileMenu->addAction(m_UI->actionSave);
	QAction * preferencesettingAction = new QAction(this);
	preferencesettingAction->setText(QCoreApplication::translate("MainWindow", "Settings", nullptr));
	connect(preferencesettingAction, &QAction::triggered, this, &MainWindow::doActionpreferencesetting);
	fileMenu->addSeparator();
	fileMenu->addAction(preferencesettingAction);
	fileMenu->addSeparator();


	fileMenu->addAction(m_UI->actionCloseAll);
	m_UI->actionCloseAll->setDisabled(true);
	m_UI->actionQuit->setIcon(QIcon(QString::fromUtf8(":/CC/images/ccQuit.png")));
	fileMenu->addAction(m_UI->actionQuit);

	//在界面顶部添加快速访问栏
	SARibbonQuickAccessBar* quickAcessBar = ribbon->quickAccessBar();
	createQuickAcdessBar(quickAcessBar);

	SARibbonButtonGroupWidget* rightBar = ribbon->rightButtonGroup();
	createRightButtonGroup(rightBar);

	//添加开始标签页
	SARibbonCategory* category_Start = new SARibbonCategory(ribbon);
	category_Start->setCategoryName(QCoreApplication::translate("MainWindow", "Start", nullptr));
	category_Start->setObjectName("Start");
	ribbon->addCategoryPage(category_Start);
	createCategoryStart(category_Start);

	//添加编辑标签页
	SARibbonCategory* category_Edit = new SARibbonCategory(ribbon);
	category_Edit->setCategoryName(QCoreApplication::translate("MainWindow", "Edit", nullptr));
	category_Edit->setObjectName("edit");
	ribbon->addCategoryPage(category_Edit);
	createCategoryEdit(category_Edit);

	//添加算法测试标签页
	/*SARibbonCategory* category_Algorithm = new SARibbonCategory(ribbon);
	category_Algorithm->setCategoryName(QCoreApplication::translate("MainWindow", "algTest", nullptr));
	ribbon->addCategoryPage(category_Algorithm);
    createCategoryAlgorithm(category_Algorithm);*/


	//添加视图标签页
	SARibbonCategory* category_View = new SARibbonCategory(ribbon);
	category_View->setCategoryName(QCoreApplication::translate("MainWindow", "Display", nullptr));
	category_View->setObjectName("view");
	ribbon->addCategoryPage(category_View);
	createCategoryView(category_View);



	ribbon->showMinimumModeButton(true);//显示最小化面板按钮
    //[!].顶部导航切换
    connect(ribbon, &SARibbonBar::currentRibbonTabChanged, this, &MainWindow::slotCurrentRibbonTabChanged);

}

void MainWindow::createCategoryStart(SARibbonCategory* page)
{

	SARibbonPannel* pannelMappingItem = new SARibbonPannel(QCoreApplication::translate("MainWindowUI", "Data Resolving", nullptr));
	pannelMappingItem->setObjectName("Mapping");
	page->addPannel(pannelMappingItem);

	//[!].正射影像
	pannelMappingItem->addLargeAction(m_pCreatePointCloudOrthphotoAction);
	

	SARibbonPannel* pannelPointProcessItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Points Processing", nullptr));//点云处理
	pannelPointProcessItem->setObjectName("PointProcess");
	page->addPannel(pannelPointProcessItem);

	//倾斜拉平
	setActionIcon(m_actionAccuracyActiveAxisZ, "AccuracyNormal", "clickedpng/AccuracyClicked", "disabledpng/AccuracyDisable");
	m_actionAccuracyActiveAxisZ->setToolTip(QCoreApplication::translate("MainWindow", "Rectify incorrect point cloud data caused by improper operations during the scanning.", nullptr));
	pannelPointProcessItem->addLargeAction(m_actionAccuracyActiveAxisZ);

	//过滤离散点
	setActionIcon(m_UI->actionSORFilter, "SORFilter", "clickedpng/noFilters@2x", "disabledpng/noFilters@2x");
	m_UI->actionSORFilter->setToolTip(QCoreApplication::translate("MainWindowUI", "Remove the points far from their neighbors", nullptr));
	m_UI->actionSORFilter->setText(QCoreApplication::translate("MainWindow", "Delete outliers", nullptr));
	pannelPointProcessItem->addLargeAction(m_UI->actionSORFilter);

	SARibbonPannel* pannelRegistrationItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Registration", nullptr));//拼接
	pannelRegistrationItem->setObjectName("Registration");
	page->addPannel(pannelRegistrationItem);
	//点云拼接
	m_UI->actionPointPairsAlign->setText(QCoreApplication::translate("MainWindowUI", "Point Cloud Registration", nullptr));
	m_UI->actionPointPairsAlign->setToolTip(tr("Aligns two clouds by picking at least 3 equivalent point pairs"));
	pannelRegistrationItem->addLargeAction(m_UI->actionPointPairsAlign);
	//点云合并
	setActionIcon(m_UI->actionMerge, "mergeicon", "mergeiconselect", "mergeicondisable");
	pannelRegistrationItem->addLargeAction(m_UI->actionMerge);

	SARibbonPannel* pannelDataTransformItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Transformation", nullptr));//数据转换
	pannelDataTransformItem->setObjectName("DataTransform");
	page->addPannel(pannelDataTransformItem);
	//坐标转换
	m_coordinateConversionAciton->setText(tr("Coordinate Transformation"));
	m_coordinateConversionAciton->setToolTip(tr("Transform point clouds from the current coordinate system to the target coordinate system."));
	pannelDataTransformItem->addLargeAction(m_coordinateConversionAciton);
	//平移旋转
	setActionIcon(m_UI->actionTranslateRotate, "rotationicon", "rotationiconselect", "rotationicondisable");
	m_UI->actionTranslateRotate->setText(tr("Translation/Rotation"));
	m_UI->actionTranslateRotate->setToolTip(tr("Translation/rotation"));
	pannelDataTransformItem->addLargeAction(m_UI->actionTranslateRotate);
	
}

void MainWindow::createCategoryDrafting(SARibbonCategory* page)
{
	SARibbonPannel* pannelPlanecutItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Planecut", nullptr));//制图
	pannelPlanecutItem->setObjectName("Planecut");
	page->addPannel(pannelPlanecutItem);
	//平面切割
	actionPlaneCut->setText(QCoreApplication::translate("MainWindowUI", "Slice plane", nullptr));
	setActionIcon(actionPlaneCut, "PointCutNormalIcon", "PointCutClickedIcon", "PointCutDisableIcon");
	pannelPlanecutItem->addLargeAction(actionPlaneCut);
    actionPlaneCut->setToolTip(QCoreApplication::translate("MainWindowUI", "Manually select specific slice location.", nullptr));
}

void MainWindow::createCategoryEdit(SARibbonCategory* page)
{
	SARibbonPannel* pannelSamplingItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Sampling", nullptr));//采样
	pannelSamplingItem->setObjectName("Sampling");
	page->addPannel(pannelSamplingItem);
	//抽稀
	setActionIcon(m_UI->actionSubsample, "subSample@2x", "clickedpng/fillter@2x", "disabledpng/fillter@2x");
	pannelSamplingItem->addLargeAction(m_UI->actionSubsample);


	//聚类
	setActionIcon(m_UI->actionLabelConnectedComponents, "Extract", "clickedpng/jvlei@2x", "disabledpng/jvlei@2x");
	m_UI->actionLabelConnectedComponents->setToolTip(QCoreApplication::translate("MainWindow", "Classify point clouds by using the octree data structure.", nullptr));
	//pannelSliceItem->addLargeAction(m_UI->actionLabelConnectedComponents);


	SARibbonPannel* pannelClassifyItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Classification", nullptr));//分类
	pannelClassifyItem->setObjectName("Classify");
	page->addPannel(pannelClassifyItem);
	//分类
	outRoomClass_action_->setToolTip(QCoreApplication::translate("MainWindow", "Automatically classify point cloud data collected outdoors.", nullptr));
	connect(outRoomClass_action_, &QAction::triggered, this, &MainWindow::doActionOutDoor);

	pannelClassifyItem->addLargeAction(outRoomClass_action_);

    //aric.tang_2022.10.14
    //connect(forestTerrainAndVege_action_, &QAction::triggered, this, &MainWindow::doActionTerrain);
    //pannelClassifyItem->addLargeAction(forestTerrainAndVege_action_);
    //connect(forestRestAndTrees_action_, &QAction::triggered, this, &MainWindow::doActionTrees);
    //pannelClassifyItem->addLargeAction(forestRestAndTrees_action_);

	SARibbonPannel* pannelAnnotatedItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Note", nullptr));//注释
	pannelAnnotatedItem->setObjectName("Annotated");
	page->addPannel(pannelAnnotatedItem);
	//注释
	actionNotes->setText(QCoreApplication::translate("MainWindow", "Add Notes", nullptr));
	actionNotes->setToolTip(QCoreApplication::translate("MainWindow", "Provide point information.", nullptr));
	setActionIcon(actionNotes, "NotesNormalIcon", "NotesClickedIcon", "NotesDisabledIcon");
	pannelAnnotatedItem->addLargeAction(actionNotes);
    pannelAnnotatedItem->addLargeAction(actionNotes)->setFixedWidth(64);


	SARibbonPannel* pannelTriangleMeshItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "Triangular Mesh", nullptr));//三角网
	pannelTriangleMeshItem->setObjectName("TriangleMesh");
	page->addPannel(pannelTriangleMeshItem);
	//创建三角网
	setActionIcon(m_UI->actionComputeMeshLS, "ComputeSurfaceMeshNormal", "ComputeSurfaceMeshClick", "ComputeSurfaceMeshDisable");
	m_UI->actionComputeMeshLS->setText(QCoreApplication::translate("MainWindow", "Triangulation", nullptr));
	m_UI->actionComputeMeshLS->setToolTip(QCoreApplication::translate("MainWindow", "Triangulation", nullptr));
	pannelTriangleMeshItem->addLargeAction(m_UI->actionComputeMeshLS);
	//表面三角网
	//m_actionComputeSurfaceMesh->setText(QCoreApplication::translate("MainWindow", "Surface Mesh", nullptr));
	//m_actionComputeSurfaceMesh->setToolTip(QCoreApplication::translate("MainWindow", "Construct the triangulation network model of object surface", nullptr));
	//setActionIcon(m_actionComputeSurfaceMesh, "ComputeSurfaceMeshNormal", "ComputeSurfaceMeshClick", "ComputeSurfaceMeshDisable");
	//pannelTriangleMeshItem->addLargeAction(m_actionComputeSurfaceMesh);
	//connect(m_actionComputeSurfaceMesh, &QAction::triggered, [=]() {
	//	doActionComputeMesh(CCCoreLib::SURFACE_MESH);
	//});

	//等高线
	m_ContourAction->setText(tr("Contours"));
	m_ContourAction->setToolTip(tr("Connect points with the same elevation value into a closed curve."));
	setActionIcon(m_ContourAction, "ContourIconNormal", "ContourIconClicked", "ContourIconDisabled");
	pannelTriangleMeshItem->addLargeAction(m_ContourAction);


	//平滑
	setActionIcon(m_UI->actionSmoothMeshLaplacian, "smoothicon", "smoothiconselect", "smoothicondisable");
	m_UI->actionSmoothMeshLaplacian->setToolTip(QCoreApplication::translate("MainWindow", "Smooth a mesh", nullptr));
	pannelTriangleMeshItem->addLargeAction(m_UI->actionSmoothMeshLaplacian);
	//采样
	setActionIcon(m_UI->actionSamplePointsOnMesh, "samplingicon", "samplingiconselect", "samplingicondisable");
	m_UI->actionSamplePointsOnMesh->setToolTip(QCoreApplication::translate("MainWindow", "Sample points on the mesh.", nullptr));
	pannelTriangleMeshItem->addLargeAction(m_UI->actionSamplePointsOnMesh);
	//去噪
	//pannelRemoveNoise->addLargeAction(m_UI->actionNoiseFilter);

}


QIcon MainWindow::GetIconByPicName(const QString & path1, const QString & path2, const QString & path3)
{
	QIcon pIconTest(path1);
	pIconTest.addPixmap(QPixmap(path2), QIcon::Active, QIcon::On);
	pIconTest.addPixmap(QPixmap(path3), QIcon::Disabled, QIcon::Off);
	return pIconTest;
}

void MainWindow::SetQActionShowNormal()
{

	/*QString action_icon_path = QString(m_DirName + "/theme/qssimage/locationPoint2x.png");
	QIcon icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/zuobiaodian@2x.png", m_DirName + "/theme/qssimage/disabledpng/zuobiaodian@2x.png");
	point_info_action_->setIcon(icon);


	action_icon_path = QString(m_DirName + "/theme/qssimage/distance@2x.png");
	icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/DistancePicking@2x.png", m_DirName + "/theme/qssimage/disabledpng/DistancePicking@2x.png");
	point_point_distance_action_->setIcon(icon);


	action_icon_path = QString(m_DirName + "/theme/qssimage/angleMeasure@2x.png");
	icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/angleclick@2x.png", m_DirName + "/theme/qssimage/disabledpng/angledisabled@2x.png");
	points_angle_action_->setIcon(icon);


	action_icon_path = QString(m_DirName + "/theme/qssimage/heightMeasure@2x.png");
	icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/heightdismeasure@2x.png", m_DirName + "/theme/qssimage/disabledpng/heightdismeasure@2x.png");
	height_dis_in_action_->setIcon(icon);


	action_icon_path = QString(m_DirName + "/theme/qssimage/aeraMeasure@2x.png");
	icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/aerameasure@2x.png", m_DirName + "/theme/qssimage/disabledpng/aerameasure@2x.png");
	areaMeasure_action_->setIcon(icon);*/


	/*action_icon_path = QString(m_DirName + "/theme/qssimage/volumeMeasure@2x.png");
	icon = GetIconByPicName(action_icon_path, m_DirName + "/theme/qssimage/clickedpng/volumemeasure@2x.png", m_DirName + "/theme/qssimage/disabledpng/volumemeasure@2x.png");
	points_volume_action_->setIcon(icon);*/
}


void MainWindow::createCategoryView(SARibbonCategory* page)
{
	SARibbonPannel* pannel_view = page->addPannel(QCoreApplication::translate("MainWindow", "View", nullptr)); //视图面板
	pannel_view->setObjectName("pannelview");
	SARibbonPannel* pannel_pointDisplay = page->addPannel(QCoreApplication::translate("MainWindow", "Point Display", nullptr));//点显示面板
	pannel_pointDisplay->setObjectName("Appearance");
	//SARibbonPannel* pannel_lineDisplay = page->addPannel(QCoreApplication::translate("MainWindow", "line display", nullptr));//矢量线面板
	//SARibbonPannel* pannel_ceshiDisplay = page->addPannel(QCoreApplication::translate("MainWindow", "ceshi", nullptr));//测试用面板

	//背景色
	QIcon pIconDisplayOptions(m_DirName + "/theme/qssimage/backgroundColor@2x.png");
	pIconDisplayOptions.addPixmap(QPixmap(m_DirName + "/theme/qssimage/clickedpng/backgroundcolor@2x.png"), QIcon::Active, QIcon::Off);
	pIconDisplayOptions.addPixmap(QPixmap(m_DirName + "/theme/qssimage/disabledpng/backgroundcolor@2x.png"), QIcon::Disabled, QIcon::Off);
	m_UI->actionDisplayOptions->setIcon(QIcon(pIconDisplayOptions));
	pannel_view->addLargeAction(m_UI->actionDisplayOptions);

	//视角
	viewEye_->addAction(m_UI->actionSetViewLeft);
	viewEye_->addAction(m_UI->actionSetViewRight);
	viewEye_->addAction(m_UI->actionSetViewFront);
	viewEye_->addAction(m_UI->actionSetViewBack);
	viewEye_->addAction(m_UI->actionSetViewTop);
	viewEye_->addAction(m_UI->actionSetViewBottom);


	//viewEye_->addAction(m_UI->actionSetViewIso1);
	//viewEye_->addAction(m_UI->actionSetViewIso2);
    m_UI->actionSetViewIso1->setToolTip(QApplication::translate("MainWindowUI", "Set view to front isometric", nullptr));
   
    m_UI->actionSetViewIso2->setToolTip(QApplication::translate("MainWindowUI", "Set the view to back  isometric", nullptr));
    m_UI->actionSetViewIso1->setVisible(false);
    m_UI->actionSetViewIso2->setVisible(false);

	pannel_view->addLargeMenu(viewEye_);
    //裁剪盒
    SARibbonPannel* pannelCrossSectionItem = new SARibbonPannel(QCoreApplication::translate("MainWindow", "CrossSection", nullptr));
    pannelCrossSectionItem->setObjectName("CrossSection");
    setActionIcon(m_UI->actionCrossSection, "clippingbox-normal@2x", "clippingbox-clicked@2x", "clippingbox-disclicked@2x");
	m_UI->actionCrossSection->setText(QCoreApplication::translate("MainWindowUI", "Clipping Box", nullptr));
    m_UI->actionCrossSection->setToolTip(QCoreApplication::translate("MainWindowUI", "Hide, show, and clip a point cloud in a box.", nullptr));
	m_UI->actionCrossSection->setObjectName("CrosssectionBtn");
    pannel_view->addLargeAction(m_UI->actionCrossSection);
//    m_UI->actionCrossSection->setToolTip(QCoreApplication::translate("MainWindowUI", "Rebuild point clouds with the raw data.", nullptr));


	//设置

	
	pannel_pointDisplay->addLargeAction(m_EDLAction);
    pannel_pointDisplay->addLargeAction(m_EDLAction)->setFixedWidth(64);
	pannel_pointDisplay->addLargeAction(pcvaction);



	//点颜色
	m_pointcolormenu->addAction(m_UI->actionSetUniqueColor);
	m_pointcolormenu->addAction(m_UI->actionSetColorGradient);
	m_pointcolormenu->addAction(colorByIntensityAction);
	m_pointcolormenu->addAction(m_UI->actionRGBToGreyScale);
	m_pointcolormenu->addSeparator();
	m_pointcolormenu->addAction(m_UI->actionClearColor);

	//pannel_pointDisplay->addLargeMenu(m_pointcolormenu);
   // pannel_pointDisplay->addLargeAction(m_UI->actionAddConstantSF);
  //  pannel_pointDisplay->addLargeAction(m_UI->actionAddClassificationSF);


	//pannel_pointDisplay->addLargeAction(m_UI->actionSetUniqueColor);
	//GL过滤器
	//m_pluginUIManager->shaderAndFilterMenu()->setIcon(QIcon(QString(m_DirName + "/theme/qssimage/noFilters@2x.png")));
	//m_pluginUIManager->pluginMenu()->setIcon(QIcon(QString(m_DirName + "/theme/qssimage/mainwidget_treewidget_icon.png")));

	//pannel_pointDisplay->addLargeMenu(m_pluginUIManager->shaderAndFilterMenu());

	//Scalar RGB
	//QIcon pIconScalarRGB(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ScalarRGBNormalIcon.png");
	//pIconScalarRGB.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ScalarRGBNormalIcon.png"), QIcon::Active, QIcon::Off);
	//pIconScalarRGB.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ScalarRGBDisablelIcon.png"), QIcon::Disabled, QIcon::Off);
	//m_UI->actionConvertToRGB->setIcon(QIcon(pIconScalarRGB));
	//pannel_pointDisplay->addLargeAction(m_UI->actionConvertToRGB);



	////线显示
	////线宽
	//QWidget* lwWidget = new QWidget(pannel_pointDisplay);
	//QVBoxLayout* lwLayout = new QVBoxLayout;
	//QComboBox* lineWidthBox = new QComboBox(lwWidget);
	//lineWidthBox->addItems(pointSizeStr);
	//lineWidthBox->setObjectName("actionlinewidthcombox");
	//QLabel* labelLineWidth = new QLabel(lwWidget);
	//labelLineWidth->setText(QCoreApplication::translate("MainWindow", "line width", nullptr));
	//labelLineWidth->setObjectName("actionlinewidthlabel");
	//connect(lineWidthBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), [=](int index) {

	//	if (GetActiveGLWindow())
	//	{
	//		GetActiveGLWindow()->setLineWidth(float(index + 1));
	//		GetActiveGLWindow()->redraw();
	//	}
	//	update();
	//});
	//QLabel* labelcenterline = new QLabel(psWidget);
	//labelcenterline->setFixedHeight(2);
	//labelcenterline->setObjectName("actionpointsizecenterline");

	//lwLayout->addWidget(lineWidthBox);
	//lwLayout->addStretch();
	//lwLayout->addWidget(labelcenterline);
	//lwLayout->addStretch();
	//lwLayout->addWidget(labelLineWidth);
	//lwLayout->setContentsMargins(10, 10, 10, 0);
	//lwWidget->setLayout(lwLayout);
	//lwWidget->setObjectName("actionlinewidth");
	//pannel_lineDisplay->addLargeWidget(lwWidget);


	
}

int MainWindow::getGloblePointSize()
{
    if (m_psWidget)
        return m_psWidget->getComboBoxIndex();

    return 1;
}


void MainWindow::createCategoryAlgorithm(SARibbonCategory* page)
{
    SARibbonPannel* pannel_alg1 = new SARibbonPannel(QCoreApplication::translate("MainWindow", "alg1", nullptr));//算法1面板 

    pannel_alg1->addLargeAction(m_UI->actionPointPairsAlign);
    pannel_alg1->addLargeAction(m_UI->actionRegister);
    pannel_alg1->addLargeAction(m_UI->actionAlign);
    pannel_alg1->addLargeAction(m_UI->actionComputeBestICPRmsMatrix);

	/*QAction* testAction = m_pluginUIManager->GetActionByName("Train classifier");
	QAction* test0Action = m_pluginUIManager->GetActionByName("Classify");
    
	pannel_alg1->addLargeAction(testAction);
	pannel_alg1->addLargeAction(test0Action);*/


    QAction* qcanupoClassifyAction(new QAction);
    qcanupoClassifyAction->setText("Canupo Classify Test");
    pannel_alg1->addLargeAction(qcanupoClassifyAction);
	
	QAction* qcanupoClassifyAutoSegAction(new QAction);
	qcanupoClassifyAutoSegAction->setText("Canupo ClassifyAutoSeg Test");
	pannel_alg1->addLargeAction(qcanupoClassifyAutoSegAction);


 
    page->addPannel(pannel_alg1);
}

void MainWindow::createQuickAcdessBar(SARibbonQuickAccessBar* quickAccessBar)
{
	if (quickAccessBar == nullptr)
		return;

	quickAccessBar->setWindowIcon(QIcon(":/FJ/Icons/FJ_images/topStatus_slices/lox.png"));
	//quickAccessBar->setWindowTitle("FJD Trion Metahub");

	//quickAccessBar->addAction(createAction("save", ":CC/images/ccSave.png", "save-quickbar"));
	//quickAccessBar->addSeparator();
	QPixmap logopng(m_DirName + "/theme/qssimage/logo@2x.png");
	QLabel* actlogolabel = new QLabel(this);
	actlogolabel->setObjectName("logolabeltopleft");
	actlogolabel->setPixmap(logopng);
	actlogolabel->setStyleSheet("background: #1C1C1C;");
	quickAccessBar->addWidget(actlogolabel);
	//quickAccessBar->addAction(actlogo);
	//quickAccessBar->addAction(createAction("redo", ":/FJ/Icons/FJ_images/topStatus_slices/nextStep@2x.png"));
	//quickAccessBar->addSeparator();

}

//janson 重置action图标相关
void MainWindow::resetActionIcon(QAction* act, const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName)
{
	if (act == nullptr)
		return;

	act->setText(text);
	QIcon pIconTest(iconurlnormal);
	pIconTest.addPixmap(QPixmap(iconurlclicked), QIcon::Active, QIcon::Off);
	pIconTest.addPixmap(QPixmap(iconurldisabled), QIcon::Disabled, QIcon::Off);

	act->setIcon(pIconTest);
	act->setObjectName(objName);
	
}

//addbycarl,此处添加点击图标变黄功能
QAction* MainWindow::createAction(const QString& text, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled, const QString& objName)
{
	QAction* act = new QAction(this);
	act->setText(text);
	QIcon pIconTest(iconurlnormal);
	pIconTest.addPixmap(QPixmap(iconurlclicked), QIcon::Active, QIcon::Off);
	pIconTest.addPixmap(QPixmap(iconurldisabled), QIcon::Disabled, QIcon::Off);

	act->setIcon(pIconTest);
	act->setObjectName(objName);
	//act->setPriority(Qt::ToolButtonTextUnderIcon);
	return act;
}


QAction* MainWindow::createAction(const QString& text, const QString& iconurl, const QString& objName)
{
	QAction* act = new QAction(this);
	act->setText(text);
	act->setIcon(QIcon(iconurl));
	act->setObjectName(objName);
	return act;
}

QAction* MainWindow::createAction(const QString& text, const QString& iconurl)
{
	QAction* act = new QAction(this);
	act->setText(text);
	act->setIcon(QIcon(iconurl));
	act->setObjectName(text);
	return act;
}

void MainWindow::createRightButtonGroup(SARibbonButtonGroupWidget* rightBar)
{
	//QAction* actionHelp = createAction(tr("help"), ":/icon/icon/help.svg");
	//connect(actionHelp, &QAction::triggered, this, &MainWindow::onActionHelpTriggered);
	//rightBar->addAction(actionHelp);


	////添加搜索action
	//QAction* pLeadingAction = new QAction(this);
	////pLeadingAction->setIcon(QIcon(":CC/images/ccZoomIn.png"));
	//searchLineEidt_->addAction(pLeadingAction, QLineEdit::LeadingPosition);
	//QAction* pTrailingAction = searchLineEidt_->addAction(QIcon(":CC/images/ccZoomIn.png"), QLineEdit::TrailingPosition);

	//connect(pTrailingAction, &QAction::triggered, [=]() {
	//	//do something
	//	QString strText = searchLineEidt_->text();

	//	});

	//rightBar->addWidget(searchLineEidt_);


	//用户设置
	QMenu* m = new QMenu(QCoreApplication::translate("MainWindow", "Help", nullptr), this);

	m->addAction(/*m_UI->*/menuLanguage->menuAction());//语言翻译
	m->addAction(m_LicensingAciton);
	m->addAction(m_helpDocument_action);//帮助文档
	m->addAction(about_action);//关于
	m->setObjectName("Help");
	QLabel * spacelabel = new QLabel(this);
	spacelabel->setFixedWidth(20);

	rightBar->addMenu(m);
	rightBar->addWidget(spacelabel);

}

void MainWindow::initActionsMenus()
{
	menubar = new QMenuBar();

	SARibbonBar* ribbon = ribbonBar();

	menuFile = new QMenu(menubar);
	menuFile->setObjectName(QString::fromUtf8("menuFile"));
	menuDisplay = new QMenu(menubar);
	menuDisplay->setObjectName(QString::fromUtf8("menuDisplay"));
	menuToolbars = new QMenu(menuDisplay);
	menuToolbars->setObjectName(QString::fromUtf8("menuToolbars"));
	menuLights = new QMenu(menuDisplay);
	menuLights->setObjectName(QString::fromUtf8("menuLights"));
	QIcon icon64;
	icon64.addFile(QString::fromUtf8(":/CC/images/ccSunLight.png"), QSize(), QIcon::Normal, QIcon::Off);
	menuLights->setIcon(icon64);
	menuActiveScalarField = new QMenu(menuDisplay);
	menuActiveScalarField->setObjectName(QString::fromUtf8("menuActiveScalarField"));
	menuLanguage = new QMenu(this);
	menuLanguage->setObjectName(QString::fromUtf8("menuLanguage"));
	menuLanguage->setTitle(QCoreApplication::translate("MainWindow", "Language"));

	menuHelp = new QMenu(menubar);
	menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
	menuEdit = new QMenu(menubar);
	menuEdit->setObjectName(QString::fromUtf8("menuEdit"));
	menuScalarFields = new QMenu(menuEdit);
	menuScalarFields->setObjectName(QString::fromUtf8("menuScalarFields"));
	menuColors = new QMenu(menuEdit);
	menuColors->setObjectName(QString::fromUtf8("menuColors"));
	menuNormals = new QMenu(menuEdit);
	menuNormals->setObjectName(QString::fromUtf8("menuNormals"));
	menuOrientNormals = new QMenu(menuNormals);
	menuOrientNormals->setObjectName(QString::fromUtf8("menuOrientNormals"));
	menuConvert_to = new QMenu(menuNormals);
	menuConvert_to->setObjectName(QString::fromUtf8("menuConvert_to"));
	menuOctree = new QMenu(menuEdit);
	menuOctree->setObjectName(QString::fromUtf8("menuOctree"));
	menuMesh = new QMenu(menuEdit);
	menuMesh->setObjectName(QString::fromUtf8("menuMesh"));
	menuMeshScalarField = new QMenu(menuMesh);
	menuMeshScalarField->setObjectName(QString::fromUtf8("menuMeshScalarField"));
	menuSensors = new QMenu(menuEdit);
	menuSensors->setObjectName(QString::fromUtf8("menuSensors"));
	menuGroundBasedLidar = new QMenu(menuSensors);
	menuGroundBasedLidar->setObjectName(QString::fromUtf8("menuGroundBasedLidar"));
	QIcon icon65;
	icon65.addFile(QString::fromUtf8(":/CC/images/dbGBLSensorSymbol.png"), QSize(), QIcon::Normal, QIcon::Off);
	menuGroundBasedLidar->setIcon(icon65);
	menuCameraSensor = new QMenu(menuSensors);
	menuCameraSensor->setObjectName(QString::fromUtf8("menuCameraSensor"));
	QIcon icon66;
	icon66.addFile(QString::fromUtf8(":/CC/images/dbCamSensorSymbol.png"), QSize(), QIcon::Normal, QIcon::Off);
	menuCameraSensor->setIcon(icon66);
	menuToggle = new QMenu(menuEdit);
	menuToggle->setObjectName(QString::fromUtf8("menuToggle"));
	menuWaveform = new QMenu(menuEdit);
	menuWaveform->setObjectName(QString::fromUtf8("menuWaveform"));
	menuPlane = new QMenu(menuEdit);
	menuPlane->setObjectName(QString::fromUtf8("menuPlane"));
	menuGrid = new QMenu(menuEdit);
	menuGrid->setObjectName(QString::fromUtf8("menuGrid"));
	menuPolyline = new QMenu(menuEdit);
	menuPolyline->setObjectName(QString::fromUtf8("menuPolyline"));
	menuCloud = new QMenu(menuEdit);
	menuCloud->setObjectName(QString::fromUtf8("menuCloud"));
	menu3DViews = new QMenu(menubar);
	menu3DViews->setObjectName(QString::fromUtf8("menu3DViews"));
	menuTools = new QMenu(menubar);
	menuTools->setObjectName(QString::fromUtf8("menuTools"));
	menuSegmentation = new QMenu(ribbon);
	menuSegmentation->setObjectName(QString::fromUtf8("menuSegmentation"));
	menuProjection = new QMenu(menuTools);
	menuProjection->setObjectName(QString::fromUtf8("menuProjection"));
	menuStatistics = new QMenu(menuTools);
	menuStatistics->setObjectName(QString::fromUtf8("menuStatistics"));
	menuDistances = new QMenu(menuTools);
	menuDistances->setObjectName(QString::fromUtf8("menuDistances"));
	menuRegistration = new QMenu(menuTools);
	menuRegistration->setObjectName(QString::fromUtf8("menuRegistration"));
	menuOther = new QMenu(menuTools);
	menuOther->setObjectName(QString::fromUtf8("menuOther"));
	menuSandBox = new QMenu(menuTools);
	menuSandBox->setObjectName(QString::fromUtf8("menuSandBox"));
	menuFit = new QMenu(menuTools);
	menuFit->setObjectName(QString::fromUtf8("menuFit"));
	menuClean = new QMenu(ribbon);
	menuClean->setObjectName(QString::fromUtf8("menuClean"));
	menuVolume = new QMenu(menuTools);
	menuVolume->setObjectName(QString::fromUtf8("menuVolume"));
	menuBatchExport = new QMenu(menuTools);
	menuBatchExport->setObjectName(QString::fromUtf8("menuBatchExport"));

    actionPlaneCut = new QAction(this);
    actionPlaneCut->setObjectName("actionPlaneCut");
    m_pHeightFittingAction = new QAction(this);
    m_pHeightFittingAction->setObjectName("m_pHeightFittingAction");
	
    actionNotes = new QAction(this);
    actionNotes->setObjectName("actionNotes");
    actionPointMap = new QAction(this);
    actionPointMap->setObjectName("actionPointMap");


	QString action_text;
	QString action_icon_path;
	QString objectName;

	//编辑
	//选择平移
	action_text = QCoreApplication::translate("MainWindow", "coor transform", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/coorTransform@2x.png");
	objectName = "actioncoortransform";
	coor_transform_action_ = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/coortransform@2x.png", m_DirName + "/theme/qssimage/disabledpng/coortransform@2x.png", objectName);


	//坐标转换
	action_text = QCoreApplication::translate("MainWindow", "Coordinate conversion", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/coorTransform@2x.png");
	objectName = "actionCoordinateconversion";
	m_coordinateConversionAciton = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/coortransform@2x.png", m_DirName + "/theme/qssimage/disabledpng/coortransform@2x.png", objectName);

    //列队处理
    m_QueueupForProcessingAction = new QAction(this);

    //图像融合
    m_ImageFusionAction = new QAction(this);

	//等高线
	action_text = QCoreApplication::translate("MainWindow", "contour", nullptr);
	action_icon_path = FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/AccuracyverificationNormal.png";
	objectName = "contouration";
	m_ContourAction = createAction(action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/AccuracyverificationSelect.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/AccuracyverificationDisabled.png", objectName);

    //动态物剔除
    m_DynamicObjectRemovalAction = new QAction(this);

	//EDL按钮
	action_text = QCoreApplication::translate("MainWindow", "Boundary reinforcement", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/EDL.png");
	objectName = "actionEDL";
	m_EDLAction = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/EDLClick.png", m_DirName + "/theme/qssimage/EDLdisable.png", objectName);
    //setActionIcon(m_EDLAction, "EDL", "EDLClick", "EDLdisable");
	m_EDLAction->setToolTip(QCoreApplication::translate("MainWindow", "Enable EDL to enhance depth perception.", nullptr));
	//色带条设置
	action_text = QCoreApplication::translate("MainWindowUI", "Settings", nullptr);
	action_icon_path = FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/actionSettingPng.png";
	objectName = "actionribbonStrip";
	m_ribbonStripAction = createAction(action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/actionSettingClickedPng.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/actionSettingPngDisabled.png", objectName);
	

	m_actionAccuracyActiveAxisZ = new QAction(this);
	action_text = QCoreApplication::translate("MainWindow", "Rectification", nullptr);
	action_icon_path = FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/AccuracyNormal.png";
	objectName = "actionAccuracy";
	m_actionAccuracyActiveAxisZ = createAction(action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/AccuracyClicked.png",
		FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/clickedpng/AccuracyDisable.png", objectName);

	//[!].点云生成正射影像
	action_text = QCoreApplication::translate("MainWindow", "Orthophoto", nullptr);
	action_icon_path = FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage//orthophoto_normal.png";
	objectName = "creategenerateorthoimage";
	m_pCreatePointCloudOrthphotoAction = createAction(action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage//orthophoto_active.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage//orthophoto_disable.png", objectName);
	m_pCreatePointCloudOrthphotoAction->setToolTip(tr("Generate orthophotos from point slouds."));

	//表面三角网
	m_actionComputeSurfaceMesh = new QAction(this);

    //补洞
    m_actionFillHoles = new QAction(this);

    //两期对比
    m_actionCompute2HalfDimVolume = new QAction(this);

	//室外分类
	action_text = QCoreApplication::translate("MainWindow", "Outdoor", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/roomOut@2x.png");
	objectName = "actionoutRoomClass";
	outRoomClass_action_ = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/outRoomClass@2x.png", m_DirName + "/theme/qssimage/disabledpng/outRoomClass@2x.png", objectName);

    //aric.tang_2022.10.14 forest
    //剩余与单木分类
    //action_text = QCoreApplication::translate("MainWindow", "ground _trees", nullptr);
    //action_icon_path = QString(m_DirName + "/theme/qssimage/roomOut@2x.png");
    //objectName = "TerrainAndVege";
    //forestTerrainAndVege_action_ = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/outRoomClass@2x.png", m_DirName + "/theme/qssimage/disabledpng/outRoomClass@2x.png", objectName);
    ////剩余与单木分类
    //action_text = QCoreApplication::translate("MainWindow", "Trees", nullptr);
    //action_icon_path = QString(m_DirName + "/theme/qssimage/roomOut@2x.png");
    //objectName = "RestAndTrees";
    //forestRestAndTrees_action_ = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/outRoomClass@2x.png", m_DirName + "/theme/qssimage/disabledpng/outRoomClass@2x.png", objectName);

	//室内分类
	action_text = QCoreApplication::translate("MainWindow", "Indoor", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/roomIn@2x.png");
	objectName = "actioninRoomClass";
	inRoomClass_action_ = createAction(action_text, action_icon_path, m_DirName + "/theme/qssimage/clickedpng/inRoomClass@2x.png", m_DirName + "/theme/qssimage/disabledpng/inRoomClass@2x.png", objectName);

	//楼层提取
	action_text = QCoreApplication::translate("MainWindowUI", "Floor", nullptr);
	action_icon_path = QString(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/flooricon.png");
	objectName = "actionFloorextractionClass";
	floorextractionClass_action_ = createAction(action_text, action_icon_path, FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/flooricon.pngselect.png", FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/flooricondisable.png", objectName);


	//视图
	viewEye_ = new QMenu(this);
	viewEye_->setToolTipsVisible(true);
	action_text = QCoreApplication::translate("MainWindow", "Direction", nullptr);
	action_icon_path = QString(m_DirName + "/theme/qssimage/viewEye@2x.png");
	objectName = "";
	viewEye_->setTitle(action_text);
	viewEye_->setToolTip(QCoreApplication::translate("MainWindow", "Change point cloud views.", nullptr));
	QIcon pIconviewEye(action_icon_path);
	pIconviewEye.addPixmap(QPixmap(m_DirName + "/theme/qssimage/clickedpng/viewEye@2x.png"), QIcon::Active, QIcon::Off);
	pIconviewEye.addPixmap(QPixmap(m_DirName + "/theme/qssimage/disabledpng/viewEye@2x.png"), QIcon::Disabled, QIcon::Off);

	viewEye_->setIcon(pIconviewEye);
	viewEye_->setObjectName("actionvieweye");

	//点颜色
	m_pointcolormenu = new QMenu(this);
	action_text = QCoreApplication::translate("MainWindow", "Colors", nullptr);
	action_icon_path = QString(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/coloriconlarge.png");
	m_pointcolormenu->setTitle(action_text);
	QIcon pIconpointcolor(action_icon_path);
	pIconpointcolor.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/coloriconlargeclick.png"), QIcon::Active, QIcon::Off);
	pIconpointcolor.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/coloriconlargedisable.png"), QIcon::Disabled, QIcon::Off);

	m_pointcolormenu->setIcon(pIconpointcolor);
	m_pointcolormenu->setObjectName("actionpointcolor");

	//按强度着色
	action_text = QCoreApplication::translate("MainWindow", "Color By Intensity", nullptr);
	action_icon_path ="";
	objectName = "actionColorByIntensity";
	colorByIntensityAction = createAction(action_text, action_icon_path, action_icon_path, action_icon_path, objectName);
	connect(colorByIntensityAction, &QAction::triggered, this, &MainWindow::doActionColorByIntensity);

	//[!].点云生成正射影像


	//算法


	////增加搜索框
	//searchLineEidt_ = new QLineEdit(this);
	//searchLineEidt_->setFixedWidth(100);
	//searchLineEidt_->setPlaceholderText(QCoreApplication::translate("MainWindow", "please input something", nullptr));

	//关于
	action_text = QCoreApplication::translate("MainWindowUI", "About", nullptr);
	action_icon_path = QString::fromUtf8("");
	objectName = "";
	about_action = createAction(action_text, action_icon_path, objectName);
	connect(about_action, &QAction::triggered, this, &MainWindow::doActionAbout);

    action_text = QCoreApplication::translate("MainWindowUI", "Manual", nullptr);
    objectName = "HelpDocument";
	m_helpDocument_action = createAction(action_text, action_icon_path, objectName);
	connect(m_helpDocument_action, &QAction::triggered, this, &MainWindow::doActionhelpDocument);

	//[!].授权管理
	QString strLicensingName = QCoreApplication::translate("MainWindow", "License", nullptr);
	m_LicensingAciton = createAction(strLicensingName, QString::null, "");

	setActionIcon(m_UI->actionPointPairsAlign,"PointPairsAlignNormal","PointPairsAlignSelected","PointPairsAlignDisabled");
}

void MainWindow::doActionColorByIntensity()
{
	for (ccHObject *entity : getSelectedEntities()) {

		if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				int nsf = cloud->getNumberOfScalarFields();
				for (int i = 0; i < nsf; ++i)
				{
					QString sfName = cloud->getScalarFieldName(i);
					if (sfName.contains("intensity", Qt::CaseInsensitive))
					{
						cloud->setCurrentDisplayedScalarField(i);
						cloud->showSF(true);
					}
				}


				ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
				if (sf)
				{
					std::map<float, int> paramMap;
					for (size_t i = 0; i < sf->size(); i++) {
						paramMap[sf->getValue(i)]++;
					}
					int num = sf->size() * 0.1;
					int sumnum = 0;
					for (auto iter = paramMap.rbegin(); iter != paramMap.rend(); iter++)
					{
						sumnum += iter->second;
						if (sumnum >= num)
						{
							sf->setSaturationStop(iter->first);
							sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::GREY));
							redrawAll();
							break;
						}
					}
				}
			}
		}

	}
	if (m_ccRoot)
	{
		m_ccRoot->updatePropertiesView();
	}

}


void MainWindow::doActionAbout()
{
    //Utils::TargetLanguage targ;
    //QDialog dialog(this);
    //QCheckBox box1("CN", &dialog);
    //QCheckBox box2("CN_TW", &dialog);
    //QCheckBox box3("JA", &dialog);
    //QCheckBox box4("Russian", &dialog);
    //QCheckBox box5("Spanish", &dialog);
    //QCheckBox box6("Italian", &dialog);
    //QButtonGroup group;
    //group.addButton(&box1);
    //group.addButton(&box2);
    //group.addButton(&box3);
    //group.addButton(&box4);
    //group.addButton(&box5);
    //group.addButton(&box6);

    //QPushButton confirmButton("ok", &dialog);
    //QPushButton cancelButton("cancel", &dialog);
    //dialog.setMinimumSize(500, 300);
    //QVBoxLayout vLayout;
    //vLayout.addWidget(&box1);
    //vLayout.addWidget(&box2);
    //vLayout.addWidget(&box3);
    //vLayout.addWidget(&box4);
    //vLayout.addWidget(&box5);
    //vLayout.addWidget(&box6);

    //vLayout.addWidget(&confirmButton);
    //vLayout.addWidget(&cancelButton);
    //dialog.setLayout(&vLayout);

    //QObject::connect(&confirmButton, &QPushButton::clicked, [&]() {
    //    dialog.accept();
    //});
    //QObject::connect(&cancelButton, &QPushButton::clicked, [&]() {
    //    dialog.reject();
    //});

    //if (dialog.exec() != QDialog::Accepted)
    //{
    //    return;
    //}

    //if (box1.isChecked())
    //{
    //    targ = Utils::TargetLanguage::CN;
    //}
    //else if (box2.isChecked())
    //{
    //    targ = Utils::TargetLanguage::CN_TW;
    //}
    //else if (box3.isChecked())
    //{
    //    targ = Utils::TargetLanguage::JA;
    //}
    //else if (box4.isChecked())
    //{
    //    targ = Utils::TargetLanguage::Russian;
    //}
    //else if (box5.isChecked())
    //{
    //    targ = Utils::TargetLanguage::Spanish;

    //}
    //else if (box6.isChecked())
    //{
    //    targ = Utils::TargetLanguage::Italian;

    //}
    //else
    //{
    //    return;
    //}
    //Utils::publicExportFile exports;
    //exports.getTsFileStream(targ);


#ifdef METAHUBTRION_FOR_ONLINEUPDATE
	TrionAboutDlg dlg(this);
	dlg.exec();
#endif

#ifndef METAHUBTRION_FOR_ONLINEUPDATE
	CS::MetahubWidgets::MetahubAboutdialog dlg;
	QString strVersion(FJDTRITONMETAHUB_VERSIONS);

	dlg.critical(strVersion);
#endif

}



void MainWindow::slotShowError(QString str)
{
    CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), str);
}





void MainWindow::doActionOutDoor()
{
    InformationPrompt set(QApplication::translate("importantinformationtips", "The cloth resolution is inversely proportional to the steepness of the actual slope, and the steeper the slope, the smaller the cloth resolution.", nullptr));
    qDebug() << "feature extraction!";
    //[!].先选择节点
    ccGLWindow* win = getActiveGLWindow();
    if (!win || !haveSelection()) {
        return;
    }
    deselectAllOtherObject(false);
    win->redraw();
    //[!].选择选择的类型
    SortOutdoor::OutdoorClassifiedWindow dialog(this);
    int nRet = dialog.exec();
    std::vector<SortOutdoor::enFeatureDetect> listCurrentSelect = dialog.getSelectFentrueDetect();

    QMap<QString, QString>outdoordata = dialog.getOutExtractData();

    if (listCurrentSelect.empty() || nRet != QDialog::Accepted) {
        qDebug() << "current select featrue detect is null!";
        return;
    }

    //[!].显示等待框
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Calculating...", nullptr));
    //[!].并发执行线程中执行
    QFuture<bool> fu = QtConcurrent::run([=]() {
        for (ccHObject* entity : getSelectedEntities())
        {
            if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD)) {
                //[!].执行算法
                CS::ModelControl::CCHObjectControl control(entity);
                ccPointCloud* outParam = control.detectExtractionTrees(outdoordata);
                if (outParam)
                {
                    CS::ModelControl::CCHObjectControl controlName(entity->getParent());
                    QString name = controlName.createChildNodeName(entity->getName() + ".classification");
                    outParam->setName(name);
                    entity->getParent()->addChild(outParam);
                    if (m_ccRoot)
                    {
                        m_ccRoot->addElement(outParam, false);
                        outParam->setEnabled(true);
                    }
                    entity->setEnabled(false);
                }
                else {
                    return false;
                }
            }
        }
        qDebug() << "feature extraction finish!";
        return true;
    });
    while (!fu.isFinished()) {
        QThread::msleep(20);
        QCoreApplication::processEvents();
    }
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    if (fu.result())
    {
        win->redraw();
        MainWindow::TheInstance()->setGlobalZoom();
    }
    else
    {
        CS::Widgets::FramelessMessageBox::critical(MainWindow::TheInstance(),
            QApplication::translate("MainWindow", "Error", nullptr),
            QApplication::translate("MainWindow", "Ground point extraction failed. Please check the memory or modify the parameters and try again.", nullptr));
    }
}


void MainWindow::slotTimerCheckDog(void)
{
	//[!].判断是否有正在进行的业务
	if (!WaitingDialog::MetahublFramelessWaitingdialog::instance()->isHidden()){
		return;
	}

}

//[!]自适应标志 点云加入到GlDBRoot时，设置点云的属性
void MainWindow::updateGlWindowCloudPointSize()
{

}

//[!]自适应标志 设置选取的点云的点大小状态
void MainWindow::setEnterCloudSize(int index)
{
	bool updatePointSize = false;
    for (ccHObject* object : m_selectedEntities)
    {
        ccPointCloud* pCloud = dynamic_cast<ccPointCloud*>(object);
		if (!pCloud) {
			continue;
		}
		if (index)
		{
			pCloud->setPointSizeAdaptively(false);
            pCloud->setPointSize(static_cast<unsigned>(index));
		}
		else
		{
			pCloud->setPointSizeAdaptively(true);
		}
    }

	if (index)
	{
        emit signalUpdateEnterCloudSize(false, index);
	}
	else
	{
		emit signalUpdateEnterCloudSize(true, index);
	}




    emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
}
//[!]自适应标志 全局点大小修改后设置全局的大小
void MainWindow::setGlobalPointSize(int index)
{
    if (!m_ccRoot) {
        return;
    }
    ccHObject::Container clouds;
    m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
    for (auto pObject : clouds) {
        ccPointCloud* pCloud = dynamic_cast<ccPointCloud*>(pObject);
        if (pCloud) {
			pCloud->setPointSize(0);
            if (index) {
                pCloud->setPointSizeAdaptively(false);
			}
			else
			{
				pCloud->setPointSizeAdaptively(true);
			}
        }
    }
    std::vector<ccGLWindow*>pGlwindows;
    GetGLWindows(pGlwindows);
    for (ccGLWindow* pGlwindow : pGlwindows)
    {
    pGlwindow->setPointSize(index);
		ccHObject *pGlRoot =  pGlwindow->getGlwindowRootObject();
        ccHObject::Container clouds;
    pGlRoot->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
    for (auto pObject : clouds) {
        ccPointCloud* pCloud = dynamic_cast<ccPointCloud*>(pObject);
        if (pCloud) {
            pCloud->setPointSize(0);
            if (index) {
                pCloud->setPointSizeAdaptively(false);
            }
            else
            {
                pCloud->setPointSizeAdaptively(true);
            }
        }
    }
    }
    if (m_psWidget) {
        m_psWidget->getComBoBox()->setCurrentIndex(index);
    }

    update();
    emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
}
void MainWindow::updateStatusbarInformation(QString positionstr)
{
    if (positionstr == "EXIT" || positionstr.isEmpty())
    {
        QMainWindow::statusBar()->showMessage(QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr));
        return;
    }
	QMainWindow::statusBar()->showMessage(positionstr);
}

void MainWindow::updateStatusbarInformationType(QString positionstr, CS::Model::StatusMessageType type)
{
	switch (type)
	{
	case CS::Model::Normal:
		break;
	case CS::Model::Warning:
		break;
	case CS::Model::Error:
		break;
	default:
		break;
	}
	if (positionstr == "EXIT" || positionstr.isEmpty())
	{
		QMainWindow::statusBar()->showMessage(QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr));
		return;
	}
    if (positionstr == "The smaller the search radius, the greater the smoothness.") {
        positionstr = tr("The smaller the search radius, the greater the smoothness.");
        QString path = getDBTreeCloudFilePath(getSelectedEntities());
        signalDBTreeCloudFromPath(path);
                deselectAllOtherObject(false);
        ccGLWindow* win = getActiveGLWindow();
        if (win) {
            win->update();
        }
        update();
    }
	QMainWindow::statusBar()->showMessage(positionstr);
}
void MainWindow::createRenderViewers()
{
    ccGLWindow* viewer_2d = new3DView(true,false);
    viewer_2d->setAcceptDrops(false);
    viewer_2d->setLODEnabled(false);


	ccGLWindow* viewer_3d = new3DView(true,true);

    if (viewer_3d && viewer_2d)
    {
		m_mdiArea->setShowType(FJDragableMdiArea::SHOWONLY3D);
        //m_mdiArea->cascadeSubWindows();
        //viewer_2d->setInteractionMode(ccGLWindow::MODE_PAN_ONLY);//可以绕z轴旋转，所以先注释
		//viewer_2d->SetViewLock(true);
        viewer_2d->setObjectName("viewer2d");
        viewer_2d->setWindowTitle("2D View");
        viewer_2d->setWindowName("2D View");

        viewer_3d->setObjectName("viewer3d");
        viewer_3d->setWindowTitle("3D View");
        viewer_3d->setWindowName("3D View");

        ccGui::ParamStruct params = viewer_2d->getDisplayParameters();
        //black (text) & white (background) display by default
        //params.backgroundCol = ccColor::Rgbub(20, 20, 10);
        /*params.textDefaultCol = ccColor::white;
        params.drawBackgroundGradient = false;
        params.decimateMeshOnMove = false;
        params.displayCross = false;
        params.colorScaleUseShader = false;*/
        viewer_2d->setDisplayParameters(params, false);
        /*viewer_2d->setPerspectiveState(false, true);
        viewer_2d->setInteractionMode(ccGLWindow::INTERACT_PAN | ccGLWindow::INTERACT_ZOOM_CAMERA | ccGLWindow::INTERACT_CLICKABLE_ITEMS);
        viewer_2d->setPickingMode(ccGLWindow::NO_PICKING);
        viewer_2d->displayOverlayEntities(true, false);
        viewer_2d->setSunLight(true);
        viewer_2d->setCustomLight(false);*/




        QMdiSubWindow * subWin2D = m_mdiArea->get2dWindow();
        QMdiSubWindow * subWin3D = m_mdiArea->get3dWindow();

		//connect(m_pViewer3dAction, &QAction::triggered, [=]() {
		//	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dAction.png");
		//	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dActionClicked.png"), QIcon::Active, QIcon::Off);
		//	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/pViewer3dAction.png"), QIcon::Disabled, QIcon::Off);
		//	m_2D3DPopupButton->setIcon(pIcon);
		//	m_2D3DPopupButton->setToolTip(QCoreApplication::translate("MainWindow", "Viewer3D", nullptr));
		//	m_2D3DPopupButton->setStatusTip(m_2D3DPopupButton->toolTip());
  //          viewer_3d->lockRotationAxis(false, CCVector3d());
		//	redrawAll();
		//});

 
  //      connect(m_pViewer2dAction, &QAction::triggered, [=]() {
		//	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/viewer2dAction.png");
		//	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/viewer2dActionClicked.png"), QIcon::Active, QIcon::Off);
		//	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/viewer2dAction.png"), QIcon::Disabled, QIcon::Off);
		//	m_2D3DPopupButton->setIcon(pIcon);
		//	m_2D3DPopupButton->setToolTip(QCoreApplication::translate("MainWindow", "Viewer2D", nullptr));
		//	m_2D3DPopupButton->setStatusTip(m_2D3DPopupButton->toolTip());
  //          CCVector3d axis = CCVector3d(0, 0, 1);
  //          viewer_3d->setView(CC_VIEW_ORIENTATION::CC_TOP_VIEW);
  //          viewer_3d->lockRotationAxis(true, axis);
		//	redrawAll();
		//});
    }
}

void MainWindow::initGraphicInterface()
{
	
}


void MainWindow::slotShowPlaneCut()
{
    m_pViewerAction->setEnabled(false);
    changeTo2dView();
    m_leftContainWidget->setFixedWidth(360);
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        return;
    }
    win->setAcceptDrops(false);

    const ccViewportParameters& params = win->getViewportParameters();

    if (params.perspectiveView)
    {
        m_stateUseOrthographic = true;
        toggleActiveWindowCenteredPerspective();
    }
    else
    {
        m_stateUseOrthographic = false;
    }

    m_leftContainWidget->setWidget(m_pDockCenterWidget);
    m_leftContainWidget->setVisible(true);
    deselectAllOtherObject(false);
    QApplication::instance()->setProperty("MainWindowsEnable", false);
    m_pDockCenterWidget->enterWindow();

    if (m_pDockCenterWidget->isHidden())
    {
        m_pDockCenterWidget->show();
    }

    //平面分割窗口弹出时自动切换为2D3D同时出现状态
    {
        m_mdiArea->setShowType(FJDragableMdiArea::SHOWALL);
        ccGLWindow* viewer_3d = qobject_cast<ccGLWindow*>(m_mdiArea->get3dWindow()->widget());
        if (viewer_3d)
        {
            viewer_3d->setIsNeedDrawSelectRect(true);
        }
        ccGLWindow* viewer_2d = qobject_cast<ccGLWindow*>(m_mdiArea->get2dWindow()->widget());
        if (viewer_2d)
        {
            viewer_2d->setIsNeedDrawSelectRect(false);
        }
        redrawAll();
    }


    freezeUI(true);
	setGlobalZoom();
    return;
}


void MainWindow::slotActivatedNoteChanged(void)
{
    //[!].创建注释对话框       
    CS::MetahubWidgets::MetahubNoteDialog *pNoteDlg = new CS::MetahubWidgets::MetahubNoteDialog(this);
    //[!].更新model层数据到UI层
    pNoteDlg->updateUIFromModel();
    pNoteDlg->exec();
}

void MainWindow::slotRedrawActiveWindow(void)
{
    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        ccLog::Error(tr("[slotModelChanged] Failed to get dedicated 3D view!"));
        return;
    }
    win->redraw();
}

void MainWindow::slotRedraw2DWindow(void)
{
    ccGLWindow* win = GetGLWindow("2D View");
    if (!win)
    {
        ccLog::Error(tr("[slotRedrawAllWindows] Failed to get dedicated 2D view!"));
        return;
    }
    win->redraw();
}

void MainWindow::slotRedrawAllWindows(void)
{
    ccGLWindow* win = GetGLWindow("2D View");
    if (!win)
    {
        ccLog::Error(tr("[slotRedrawAllWindows] Failed to get dedicated 2D view!"));
        return;
    }
    win->redraw();

    win = GetGLWindow("3D View");
    if (!win)
    {
        ccLog::Error(tr("[slotRedrawAllWindows] Failed to get dedicated 3D view!"));
        return;
    }
    win->redraw();
    //QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
    //for (auto itWidget: subWindowList){
    //    auto pGlWidget = GLWindowFromWidget(itWidget);
    //    if (pGlWidget) {
    //        pGlWidget->redraw();
    //    }
    //}
    //return;
   
}

void MainWindow::slotRenewalDBTreeModel(void)
{
    std::vector<ccHObject*> list = CS::Model::ProjectModel::instance()->getSelectedEntitys();
    for (auto it : list) {  
        //[!].获取所有子对象
        std::vector<ccHObject *> childrens;
        MainWindow::TheInstance()->expandElement(it, false);
        it->filterChildren(childrens);
        for (auto itChild : childrens) {
			if (itChild->getMetaData("expandnode").toBool()) {
				MainWindow::TheInstance()->expandElement(itChild, true);
			}
        }
        MainWindow::TheInstance()->expandElement(it, true);
    }
    return;
}

void MainWindow::slotUpdateWindow(void)
{
    QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
    for (auto itWidget : subWindowList) {
        auto pGlWidget = GLWindowFromWidget(itWidget);
        if (pGlWidget) {
            pGlWidget->update();
        }
    }
    return;
}

void MainWindow::slotCurrentRibbonTabChanged(int index)
{
    //[!].顶部导航切换，先离开
    QList<SARibbonCategory*> listCategory = ribbon->categoryPages();
    for (auto pCategory : listCategory){
        pCategory->leaveWindow();
    }

    //[!].在进入
    SARibbonCategory* pCurrentCategory = ribbon->categoryByIndex(index);
    if (!pCurrentCategory) {
        return;
    }
    pCurrentCategory->enterWindow();

    return;
}

void MainWindow::createEmbeddedDockWidget()
{
    m_pDockWidgetPublic = new QDockWidget;
    this->setDockNestingEnabled(true);
    m_pDockWidgetPublic->setAllowedAreas(Qt::AllDockWidgetAreas);
    //this->tabifyDockWidget(m_UI->DockableProperties, m_pDockWidgetPublic);
    m_pDockWidgetPublic->hide();
}
void MainWindow::slotActionNotes()
{
    m_pViewerAction->setEnabled(false);
    changeTo2dView();
    InformationPrompt set(QApplication::translate("importantinformationtips", "LMB:hold and drag to rotate,RMB:hold and drag to translate,MMB:Scroll to zoom in / out.", nullptr));
    //[!].启动注释编辑功能
   //[!].启动注释编辑功能
    ccGui::ParamStruct parameters = ccGui::Parameters();
    parameters.m_bNotePickGLEnable = !parameters.m_bNotePickGLEnable;
    if (parameters.m_bNotePickGLEnable)
    {
        deselectAllOtherObject(true);
        refreshAll();
        updateUI();
    }
    //[!].设置鼠标拾取模式
    setPickMode(parameters.m_bNotePickGLEnable);
    freezeUI(parameters.m_bNotePickGLEnable);

    //[!].切换图标
    QString strIcon = parameters.m_bNotePickGLEnable ? "NotesClickedIcon" : "NotesNormalIcon";
    setActionIcon(actionNotes, strIcon, "NotesClickedIcon", "NotesDisabledIcon");

    //actionNotes->setIcon(CS::Core::ICore::resourceThemeImage(strIcon));
    //[!].先开
    SARibbonCategory* pCategorey = ribbon->categoryByObjectName("edit");
    if (pCategorey) {
        //[!].在找注释
        pCategorey->setDisabled(false);
        for (auto it : pCategorey->pannelList()) {
            if (it->objectName() == "Annotated") {
                it->setDisabled(false);
            }
            else {
                it->setDisabled(it->isEnabled());
            }
        }
    }
	if (!parameters.m_bNotePickGLEnable)
	{
        m_pViewerAction->setEnabled(true);
		updateUI();
	}
    return;
}

void MainWindow::slotUpdateRegisterCoderCategory(void)
{
    if (!ribbon){
        return;
    }

   
    return;
}

void MainWindow::doActionhelpDocument()
{
    if (!qApp->property("document").toBool())
    {
        m_pPdfoutDlg = new CS::Widgets::pdfDialog();
        m_pPdfoutDlg->show(); 
    }
   
}

void MainWindow::setActionIcon(QAction * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + normalPix + ".png");
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + clickedPix + ".png"), QIcon::Active, QIcon::Off);
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + disabledPix + ".png"), QIcon::Disabled, QIcon::Off);
	action->setIcon(pIcon);
}
void MainWindow::setActionIcon(QAction * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix,QSize& size)
{
    QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + normalPix + ".png");
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + clickedPix + ".png"), QIcon::Active, QIcon::Off);
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + disabledPix + ".png"), QIcon::Disabled, QIcon::Off);
    setIconSize(size);
    action->setIcon(pIcon);
}

void MainWindow::slotAccuracyVerification(void)
{
	if (!haveSelection())
	{
		ccConsole::Error(tr("Select at least one entity (point cloud or mesh)!"));
		return;
	}
    m_pViewerAction->setEnabled(false);
    changeTo2dView();

	ccHObject::Container alignedEntities;
	ccHObject::Container refEntities;
	try
	{
		ccHObject::Container entities;
		entities.reserve(m_selectedEntities.size());

		for (ccHObject* entity : m_selectedEntities)
		{
			//for now, we only handle clouds or meshes
			if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
			{
				entities.push_back(entity);
			}
		}
		if (entities.size() == 1)
		{
			alignedEntities = entities;
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(tr("Not enough memory"));
		return;
	}

	if (alignedEntities.empty())
	{
		ccLog::Error(tr("No aligned entity selected"));
		return;
	}
	deselectAllOtherObject(true);

	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		ccLog::Error(tr("[PointPairRegistration] Failed to create dedicated 3D view!"));
		return;
	}

	//we disable all windows
	disableAllBut(win);

	if (!m_pavDlg)
	{
		m_pavDlg = new ccPointAccuracyVerificationDlg(m_pickingHub, this, this);
		connect(m_pavDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
		connect(m_pavDlg, &ccPointAccuracyVerificationDlg::showMessage, this, &MainWindow::updateStatusbarInformation);
		connect(m_pavDlg, &ccPointAccuracyVerificationDlg::openPutDialog, this, [this]
		{

		});
        connect(m_pavDlg, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });
		registerOverlayDialog(m_pavDlg, Qt::TopRightCorner);
	}

	if (!m_pavDlg->init(win, alignedEntities, &refEntities))
	{
		deactivateRegisterPointPairTool(false);
	}

	freezeUI(true);
	setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
	if (!m_pavDlg->start())
	{
		deactivateRegisterPointPairTool(false);
	}
	else
	{
		updateOverlayDialogsPlacement();
		m_UI->actionPickRotationCenter->setDisabled(true);
		m_isClippingBoxCanCombination = true;
	}
    updateStatusbarInformation(QApplication::translate("importantinformationtips", "Ensure that the control point import format is consistent with the required format.", nullptr));
}


void MainWindow::slotEDLControl(void)
{
	static bool isEDLopen = false;
	isEDLopen = !isEDLopen;
	if (isEDLopen)
	{
		if (m_pluginUIManager->GetActionByName("EDL Shader"))
		{
			m_pluginUIManager->GetActionByName("EDL Shader")->trigger();
		}
        setActionIcon(m_EDLAction, "EDLClick", "EDL", "EDLdisable");
	}
	else
	{
		if (m_pluginUIManager->GetActionByName("Remove EDL"))
		{
			m_pluginUIManager->GetActionByName("Remove EDL")->trigger();
		}
        setActionIcon(m_EDLAction, "EDL", "EDLClick", "EDLdisable");
	}
}



void MainWindow::slotCoordinateConversion(void)
{
    if (!haveSelection())
    {
        ccConsole::Error(tr("Select at least one entity (point cloud or mesh)!"));
        return;
    }
    m_pViewerAction->setEnabled(false);
    changeTo2dView();

    ccHObject::Container alignedEntities;
    ccHObject::Container refEntities;
    try
    {
        ccHObject::Container entities;
        entities.reserve(m_selectedEntities.size());

        for (ccHObject* entity : m_selectedEntities)
        {
            //for now, we only handle clouds or meshes
            if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
            {
                entities.push_back(entity);
            }
        }
        if (entities.size() == 1)
        {
            alignedEntities = entities;
        }
    }
    catch (const std::bad_alloc&)
    {
        ccLog::Error(tr("Not enough memory"));
        return;
    }

    if (alignedEntities.empty())
    {
        ccLog::Error(tr("No aligned entity selected"));
        return;
    }
    deselectAllOtherObject(true);

    ccGLWindow* win = getActiveGLWindow();
    if (!win)
    {
        ccLog::Error(tr("[PointPairRegistration] Failed to create dedicated 3D view!"));
        return;
    }

    //we disable all windows
    disableAllBut(win);

    if (!m_pprDlg)
    {
        m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this, this);
        connect(m_pprDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::showMessage, this, &MainWindow::updateStatusbarInformation);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::signalPreview, this, &MainWindow::onPairRegistrationSignalPreview, Qt::UniqueConnection);
        connect(m_pprDlg, &ccPointPairRegistrationDlg::signalReset, this, &MainWindow::onPairRegistrationSignalReset, Qt::UniqueConnection);
        connect(m_pprDlg, &ccOverlayDialog::processFinished, [this]()
        {
            updateStatusbarInformation("EXIT");
        });

        registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
    }
    m_pprDlg->setCoordinateConversionStyle(true);

    if (!m_pprDlg->init(win, alignedEntities, &refEntities))
    {
        deactivateRegisterPointPairTool(false);
    }

    freezeUI(true);
    setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
    if (!m_pprDlg->start())
    {
        deactivateRegisterPointPairTool(false);
    }
    else
    {
        updateOverlayDialogsPlacement();
        m_isClippingBoxCanCombination = true;
        m_UI->actionPickRotationCenter->setDisabled(true);
    }
    updateStatusbarInformation(QApplication::translate("importantinformationtips", "Target identification should be carried out in intensity display mode.", nullptr));
}


 const QStringList MainWindow::getIsOpenFilesPath()
{
    return  m_IsOpenFilesPath;
}

 QString MainWindow::getDBTreeCloudFilePath(ccHObject::Container selectEnt)
 {
     if (m_IsOpenFilesPath.isEmpty()){
         return QString();
     }
     for (ccHObject *obj : selectEnt)
     {
         QString objName = obj->getName();
         if (!obj) {
             continue;
         }
         while (obj->getParent()){
             if (obj->getParent()->getName() == "DB Tree") {
                 break;
             }
             obj = obj->getParent();
         }
         QString str = obj->getName();
         QString fileName;
         QRegularExpression regex("(.*)\\.\\w+ \\(");
         QRegularExpressionMatch match = regex.match(str);
         if (match.hasMatch()) {
             fileName = match.captured(1);
             str.remove(fileName);
             int openParenIndex = str.indexOf("(");
             if (openParenIndex != -1) {
                 fileName += str.left(openParenIndex).trimmed();
             }
             else {
				 fileName = str;
             }
		 }
		 else
		 {
			 fileName = str;
		 }

         for (QString path : getIsOpenFilesPath()) {
             if (path.contains(fileName)){
                 QFileInfo fileInfo(path);
                 QDir dir(fileInfo.dir());
                 if (dir.exists()){
                    return dir.path();
                 }
                 else
                 {
                     QStringList paths = QStandardPaths::standardLocations(QStandardPaths::DesktopLocation);
                     if (!paths.isEmpty()) {
                         return paths.first();
                     }
                 }
             }
         }
     }
     return {};
 }

 bool MainWindow::threadSaveLasFile(ccHObject *obj, QString filePath, bool state /*= false*/)
 {
     QString outFilePath = QDir::fromNativeSeparators(filePath);
     QFileInfo file(outFilePath);
     if (file.suffix() != "las")
     {
         outFilePath += ".las";
     }
     QString oldFilePath = outFilePath;
     QFileInfo fileInfo(outFilePath);
     if (fileInfo.isFile())
     {
         QString fileName = fileInfo.fileName();
         QString suffix = fileInfo.suffix();
         int size = suffix.count() + 1;
         QString uniqueFileName = fileName;
         uniqueFileName.chop(size);
         int counter = 2;
         while (QFileInfo::exists(outFilePath)) {
             uniqueFileName.chop(1);
             uniqueFileName = uniqueFileName + QString::number(counter);
             outFilePath.chop(fileName.count());
             outFilePath.append(uniqueFileName);
             outFilePath.append("." + suffix);
             counter++;
         }
         QMessageBox::StandardButton button = CS::Widgets::FramelessMessageBox::question(
             this,
             tr("Rename file"),
             tr("A file with the same name already exists. Do you want to rename %1 to %2?").arg(fileInfo.fileName()).arg(uniqueFileName + "." + suffix));
         if (button != QMessageBox::StandardButton::Yes){
             if (QFile::remove(oldFilePath))
             {
                 outFilePath = oldFilePath;
             }
             else
             {
                 qDebug() << "remove old file error:" << oldFilePath;
                 outFilePath.clear();
             }
         }
     }
     QFuture<bool> fu = QtConcurrent::run([=]() {
         FileIOFilter::SaveParameters parameters;
         {
             parameters.alwaysDisplaySaveDialog = false;
             parameters.parentWidget = nullptr;
         };
         CC_FILE_ERROR result = CC_FERR_NO_ERROR;
         result = FileIOFilter::SaveToFile(obj, outFilePath, parameters, "LAS cloud (*.las *.laz)");
         if (result != CC_FERR_NO_ERROR) {
             return false;
         }
         return true;

     });
     while (!fu.isFinished()) {
         QThread::msleep(50);
         QCoreApplication::processEvents();
     }
     if (!fu.result()) {
         CS::Widgets::FramelessMessageBox::warning(MainWindow::TheInstance(),
             QApplication::translate("MainWindow", "Error", nullptr),
             QApplication::translate("MainWindow", "Failed to save file.", nullptr));
     }
     return fu.result();
 }

void MainWindow::deselectOtherObject()
{
	if (m_ccRoot)
	{
		ccHObject* treeRoot = m_ccRoot->getRootEntity();
		auto childNum = treeRoot->getChildrenNumber();
		for (auto i = 0; i < childNum; i++)
		{
			//setSelfAndSubObjectEnabled(treeRoot->getChild(i), false);
			setOtherFileObjectEnabled(treeRoot->getChild(i), false);
		}

		ccHObject* pFileNode = nullptr;
		for (ccHObject* entity : m_selectedEntities)
		{
			if (entity->getParent()->isKindOf(CC_TYPES::HIERARCHY_OBJECT))
			{
				pFileNode = entity->getParent();
				break;
			}
		}
        if (!pFileNode)
        {
            return;
        }
		childNum = pFileNode->getChildrenNumber();
		for (auto i = 0; i < childNum; i++)
		{
			pFileNode->getChild(i)->setEnabled(false);
		}

		for (ccHObject* entity : m_selectedEntities)
		{
			setSelfAndParentObjectEnabled(entity,true);
			setSelfAndSubObjectEnabled(entity, true);
		}
	}
}

void MainWindow::deselectAllOtherObject(bool isChildrenNoodVisiable)
{
	if (m_ccRoot)
	{
		ccHObject* treeRoot = m_ccRoot->getRootEntity();
		auto childNum = treeRoot->getChildrenNumber();
		for (auto i = 0; i < childNum; i++)
		{
			setSelfAndSubObjectEnabled(treeRoot->getChild(i), false);
		}
		for (ccHObject* entity : m_selectedEntities)
		{
			setSelfAndParentObjectEnabled(entity, true);
			if (isChildrenNoodVisiable)
			{
				setSelfAndSubObjectEnabled(entity, true);
			}
		}
	}
}

void MainWindow::selectAllObject()
{
	if (m_ccRoot)
	{
		ccHObject* treeRoot = m_ccRoot->getRootEntity();
		auto childNum = treeRoot->getChildrenNumber();
		for (auto i = 0; i < childNum; i++)
		{
			setSelfAndSubObjectEnabled(treeRoot->getChild(i), true);
		}		
	}
}

void MainWindow::setOtherFileObjectEnabled(ccHObject * obj, bool enable)
{
	if (obj->isKindOf(CC_TYPES::HIERARCHY_OBJECT))
	{
		obj->setEnabled(enable);
	}
}

void MainWindow::setSelfAndSubObjectEnabled(ccHObject * self, bool isenabled)
{
	self->setEnabled(isenabled);
	ccGenericPointCloud* pc = static_cast<ccGenericPointCloud*>(self);
	if (pc)
	{
		if (pc->isKindOf(CC_TYPES::POINT_OCTREE))
		{
			pc->setEnabled(false);
		}
		//if (pc->getOctree())
		//{
		//	pc->getOctree()->setDisplayMode(ccOctree::MEAN_POINTS);
		//	//pc->getOctree()->s
		//}
		//pc->deleteOctree();

	}
	auto childNum = self->getChildrenNumber();
	for (auto i = 0;i < childNum;i++)
	{
		setSelfAndSubObjectEnabled(self->getChild(i), isenabled);
	}
}

bool MainWindow::findSub2dObject(ccHObject * self, bool & isFind)
{
	auto childNum = self->getChildrenNumber();
	for (auto i = 0; i < childNum; i++)
	{
		if (self->getChild(i)->getMetaData("is2dSurface").toBool() && self->getChild(i)->isEnabled())
		{
			isFind = true;
			return true;
		}
		if (findSub2dObject(self->getChild(i), isFind))
		{
			return true;
		}
	}
	return false;
}

void MainWindow::measureLabelSelectedSlot(int uuid)
{
	ccHObject* treeRoot = m_ccRoot->getRootEntity();
	if (treeRoot)
	{
		ccHObject* obj = treeRoot->find(uuid);
		if (obj)
		{
			cc2DLabel* label = dynamic_cast<cc2DLabel*>(obj);
			if (label)
			{
				if (label->isVisible() && label->isEnabled() && label->isAreaAndMoreThanThreePoint())
				{
					ccGLMatrixd mat = label->getDirectionMatrixd();
					ccGLWindow* win = getActiveGLWindow();
					if (win->objectName() == "viewer2d")
					{
						return;
					}
					if (win)
					{
						if ((win->getInteractionMode() & ccGLWindow::INTERACTION_FLAG::INTERACT_ROTATE) == ccGLWindow::INTERACTION_FLAG::INTERACT_ROTATE)
						{
							win->setBaseViewMat(mat); 
						}
					}
				}
			}
		}
	}
}

void MainWindow::slotUIGenerateOrthoimageChanged(void)
{
	if (!ribbon){
		return;
	}

	//[!].单选而且必现是点云数据
	std::vector<ccHObject*> listSelectedEntitys = CS::Model::ProjectModel::instance()->getSelectedEntitys();
	if (listSelectedEntitys.empty()) {
		return;
	}
	ccHObject* pActiveEntity = listSelectedEntitys.at(0);
	if (!pActiveEntity){
		return;
	}
	if (!pActiveEntity->isKindOf(CC_TYPES::POINT_CLOUD)){
		return;
	}

	//[!].先禁用掉， 并且为3D视图
	ribbon->setEnabled(false);
	m_pViewerAction->setEnabled(false);
	changeTo2dView();
	setUseOrthographic(true);
	deselectAllOtherObject(false);
	UpdateUI();
	setLeftDBTreeEnabled(false);
	setViewButtonDisabled(true);
	setGlobalZoom();
	

	//[!].创建
	CS::MetahubWidgets::CreatePointCloudOrthoImageDialog *pDialog = new CS::MetahubWidgets::CreatePointCloudOrthoImageDialog(this);
	pDialog->setCurrentActiveEntity(pActiveEntity);
	pDialog->updateUIFromModel();
	connect(pDialog, &CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::accepted, this, [&]() {
		ribbon->setEnabled(true);
		setLeftDBTreeEnabled(true);
		setViewButtonDisabled(false);
		setUseOrthographic(false);
		m_pViewerAction->setEnabled(true);
	});
	pDialog->show();
	return;
}

void MainWindow::updatePerspectiveStateSlot(void)
{
	ccHObject* treeRoot = m_ccRoot->getRootEntity();
	auto childNum = treeRoot->getChildrenNumber();
	bool isFind2dObject = false;
	for (auto i = 0; i < childNum; i++)
	{
		if (findSub2dObject(treeRoot->getChild(i), isFind2dObject))
		{
			break;
		}
	}
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}
	const ccViewportParameters& params = win->getViewportParameters();
	if (isFind2dObject)
	{
		if (params.perspectiveView)
		{
			toggleActiveWindowCenteredPerspective();
		}
	}
	else
	{
		if (!params.perspectiveView)
		{
			toggleActiveWindowCenteredPerspective();
		}
	}
}

void MainWindow::setSelfAndParentObjectEnabled(ccHObject * self, bool isenabled)
{
	self->setEnabled(isenabled);
	ccGenericPointCloud* pc = static_cast<ccGenericPointCloud*>(self);
	if (pc)
	{
		if (pc->isKindOf(CC_TYPES::POINT_OCTREE))
		{
			pc->setEnabled(false);
		}
		//if (pc->getOctree())
		//{
		//	pc->getOctree()->setDisplayMode(ccOctree::MEAN_POINTS);
		//	//pc->getOctree()->s
		//}
		//pc->deleteOctree();

	}

	if (self->getParent())
	{
		setSelfAndParentObjectEnabled(self->getParent(),true);
	}
}


QString createZAxisNodeName(ccHObject* pObj)
{
    if (!pObj && !pObj->getParent())
    {
        return QString();
    }

    std::vector<ccHObject *> childrens;
    pObj->getParent()->filterChildren(childrens);


    auto findChildName = [=](int index)->bool {
        QString strName = QString("%1.adjust%2").arg(pObj->getName()).arg(index);

        std::vector<ccHObject *>::const_iterator it;
        it = std::find_if(childrens.cbegin(), childrens.cend(), [strName](ccHObject * rhs) {
            return rhs && rhs->getName() == strName;
        });

        return it == childrens.end() ? false : true;
    };

    int index = 1;
    do
    {
        if (!findChildName(index)) {
            break;
        }

    } while (index++);

    return QString("%1.adjust%2").arg(pObj->getName()).arg(index);
}

void MainWindow::toggleActiveZaxisLevel()
{
    //[!].获取选择的点云数据
    ccGLWindow* win = getActiveGLWindow();
    if (!win || !haveSelection())
    {
        return;
    }

    if (!haveOneSelection())
        return;
    deselectAllOtherObject(false);
    updateUI();
    update();

    //[!].显示等待框
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindow", "Rectifying the point cloud...", nullptr));

    bool output_error = true;
    ccPointCloud* outputCloud = nullptr;
    QFuture<void> fu = QtConcurrent::run([&]() {
        for (ccHObject* entity : CS::Model::ProjectModel::instance()->getSelectedEntitys()/*getSelectedEntities()*/)
        {
            if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
            {
                ccHObject* parent = entity->getParent();
                if (!parent)
                {
                    parent = m_ccRoot->getRootEntity();
                }

                if (outputCloud)
                {
                    outputCloud->setName(createZAxisNodeName(entity));
                    parent->addChild(outputCloud);
                    addToDB(outputCloud, false, true, false, false);
                }
                else
                {
                    output_error = false;
                }
            }
        }

    });
    while (!fu.isFinished()) {
        QThread::msleep(20);
        QCoreApplication::processEvents();
    }
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    if (CS::Model::ProjectModel::instance()->getSelectedEntitys().size() == 1)
    {
        CS::Model::ProjectModel::instance()->getSelectedEntitys()[0]->setEnabled(false);
    }
    if(outputCloud)
    {
        //修改菜单栏闪烁问题-carl.wang20230724
        //m_ccRoot->unselectAllEntities();
        outputCloud->setEnabled(true);
        m_ccRoot->selectEntity(outputCloud);
    }


    if (!output_error)
    {
        if (CS::Model::ProjectModel::instance()->getSelectedEntitys().size() > 0)
        {

            ccHObject* pObj = CS::Model::ProjectModel::instance()->getSelectedEntitys()[0];
            m_ccRoot->selectEntity(pObj);
            pObj->setEnabled(true);
            updateUI();
            update();
        }

        //点云矫正失败
        CS::Widgets::FramelessMessageBox::warning(this,
            QCoreApplication::translate("MainWindow", "Error", nullptr),
            QCoreApplication::translate("MainWindow", "Rectification failed. The point cloud cannot be rectified.", nullptr));
    }
    win->redraw();
    setGlobalZoom();
}


//aric.tang_2022.10.14
//seg ground and Treesandother by octree
void MainWindow::doActionTerrain()
{
    qDebug() << "Start performing feature extraction!";
    if (m_selectedEntities.size() != 1)
    {
        return;
    }

    ccPointCloud* cloud_in;
    ccPointCloud* ground;
    ccPointCloud* treeParts;
    
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));

    ccHObject *entity = m_selectedEntities[0];
    if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
    {
        ccHObject* parent = entity->getParent();
        if (!parent) {
            parent = m_ccRoot->getRootEntity();
        }
        
        cloud_in = ccHObjectCaster::ToPointCloud(entity);
        ground = cloud_in->cloneThis(nullptr, true);
        treeParts = cloud_in->cloneThis(nullptr, true);
        ground->clear();
        treeParts->clear();

    }
    ground->setName("ground");
    treeParts->setName("treeParts");
    ground->showColors(true);
    ground->showSF(false);
    treeParts->showColors(true);
    treeParts->showSF(false);
    addToDB(ground, false, true, false, false);
    addToDB(treeParts, false, true, false, false);

    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();

    return;
}

//seg trees and other by octree
void MainWindow::doActionTrees()
{
    qDebug() << "Start performing feature extraction!";
    if (m_selectedEntities.size() != 2)
    {
        return;
    }

    ccPointCloud* cloud_ground;
    ccPointCloud* cloud_treeParts;
    ccPointCloud* cloud_rest;
    ccHObject* ccGroup = new ccHObject(m_selectedEntities[1]->getName() + "tree");

    ccHObject *entity_ground = m_selectedEntities[0];
    ccHObject *entity_treeParts = m_selectedEntities[1];
    if ((entity_ground && entity_ground->isKindOf(CC_TYPES::POINT_CLOUD)) 
        && (entity_treeParts && entity_treeParts->isKindOf(CC_TYPES::POINT_CLOUD)))
    {

    }
    cloud_rest->setName("rest");
    cloud_rest->showColors(true);
    cloud_rest->showSF(false);
    addToDB(cloud_rest, false, true, false, false);
    addToDB(ccGroup, false, true, false, false);
    
    return;
}

void MainWindow::slotAccuracyActiveZaxisLevel()
{
	toggleActiveZaxisLevel();
}


void MainWindow::reviseStatusBarAttributes()
{
    //[!]显示坐标
    m_pShowCurrentPos = new QLabel(this);
    m_pShowCurrentPos->setStyleSheet("QLabel{background:#585858;background-color:rgb(255, 200, 4);color:#1c1c1c;}");
    QMainWindow::statusBar()->addPermanentWidget(m_pShowCurrentPos);
    m_pShowCurrentPos->setVisible(false);

    //[!]显示点数
    m_pShowPointNumberPos = new QLabel(this);
    m_pShowPointNumberPos->setStyleSheet("QLabel{background:#585858;background-color:rgb(255, 200, 4);color:#1c1c1c;}");
    QMainWindow::statusBar()->addPermanentWidget(m_pShowPointNumberPos);
    m_pShowPointNumberPos->setVisible(false);

    //[!]显示偏移值
    m_pShowOffsetPos = new QLabel(this);
    m_pShowOffsetPos->setStyleSheet("QLabel{background:#585858;background-color:rgb(255, 200, 4);color:#1c1c1c;}");
    QMainWindow::statusBar()->addPermanentWidget(m_pShowOffsetPos);
    m_pShowOffsetPos->setVisible(false);
}

void MainWindow::slotShowCurrentPos(CCVector3d pos)
{

    if (fabs(pos.x - 0) < EPSILON && fabs(pos.z - 0) < EPSILON && fabs(pos.z - 0) < EPSILON)
    {
        m_pShowCurrentPos->setVisible(false);
    }
    else
    {
        m_pShowCurrentPos->setVisible(true);
        ccGLWindow* pWin = getActiveGLWindow();
        unsigned int precision = 3;
        if (pWin)
        {
            precision = pWin->getDisplayParameters().displayedNumPrecision;
        }
        ccHObject::Container clouds;

        CCVector3d globelPos(0, 0, 0);
        CCVector3d globelOrigin(0, 0, 0);
        CCVector3d globelShift(0, 0, 0);
        bool first = true;
        bool equal = true;
        m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
        if (clouds.size() > 1)
        {
            for (ccHObject* entity : clouds)
            {

				if (entity->getMetaData("VectorGraphics").toBool()){//[!].二维矢量节点不参与
					continue;
				}
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
                if (cloud)
                {
                    ccShiftedObject* shiftedObj = ccHObjectCaster::ToShifted(cloud);
                    if (first)
                    {
                        globelOrigin = shiftedObj->getGlobalShift();
                        first = false;
                    }
                    globelShift = (shiftedObj->getGlobalShift().norm() == globelOrigin.norm()) ? globelOrigin : shiftedObj->getGlobalShift();
                    if (globelShift.norm() != globelOrigin.norm())
                    {
                        equal = false;
                        break;
                    }
                }
            }
            if (equal)
            {
                globelPos = pos - globelOrigin;
                m_pShowCurrentPos->setText(tr("Location") + ": " + QString(" %1 , %2 , %3 ").arg(QString::number(globelPos.x, 'f', precision))
                    .arg(QString::number(globelPos.y, 'f', precision)).arg(QString::number(globelPos.z, 'f', precision)));
            }
            else
            {
                globelPos = pos;
                m_pShowCurrentPos->setText(tr("Location") + ": " + QString(" %1 , %2 , %3 ").arg(QString::number(globelPos.x, 'f', precision))
                    .arg(QString::number(globelPos.y, 'f', precision)).arg(QString::number(globelPos.z, 'f', precision)));
            }
        }
        else if (clouds.size() == 1)
        {
            ccShiftedObject* shiftedObj = ccHObjectCaster::ToShifted(clouds[0]);
            globelOrigin = shiftedObj->getGlobalShift();

            globelPos = pos - globelOrigin;
            m_pShowCurrentPos->setText(tr("Location") + ": " + QString(" %1 , %2 , %3 ").arg(QString::number(globelPos.x, 'f', precision))
                .arg(QString::number(globelPos.y, 'f', precision)).arg(QString::number(globelPos.z, 'f', precision)));
        }
        else
        {
            m_pShowCurrentPos->setVisible(false);
        }

    }
}



void MainWindow::setMenuButtonEnable(QString ribbonObjName, QString pannelObjName, QString buttonObjName, bool isenable)
{
	SARibbonCategory* pCategorey = ribbon->categoryByObjectName(ribbonObjName);
	if (pCategorey) {
		//pCategorey->setDisabled(false);
		for (auto it : pCategorey->pannelList()) {
			if (it->objectName() == pannelObjName) {
				//it->setDisabled(false);
				for (auto subit : it->ribbonToolButtons())
				{
					if (subit->objectName() == buttonObjName) {
						subit->setEnabled(isenable);
					}
					else {
						//subit->setDisabled(subit->isEnabled());
					}
				}
			}
			else {
				//it->setDisabled(it->isEnabled());
			}
		}
	}
}

void MainWindow::onPairRegistrationSignalPreview()
{
    if (m_clipTool && m_isClippingBoxOpen)
    {
        m_clipTool->setCurrentButtonStatus(false);
        m_clipTool->closeDialog();
        m_pprDlg->recordCombinationWhether = true;
    }
    setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", false);
}


void MainWindow::onPairRegistrationSignalReset()
{
    if (m_clipTool &&  m_pprDlg->recordCombinationWhether)
    {
        m_pprDlg->recordCombinationWhether = false;
        activateClippingBoxMode();
        m_clipTool->setCurrentButtonStatus(true);
    }
    else
    {
        setMenuButtonEnable("view", "pannelview", "CrosssectionBtn", true);
    }
}
void MainWindow::setUseOrthographic(bool isuse)
{
	ccGLWindow* win = getActiveGLWindow();
	if (!win)
	{
		return;
	}
	const ccViewportParameters& params = win->getViewportParameters();
	if (isuse)
	{
		m_stateUseOrthographic = params.perspectiveView;
		if (params.perspectiveView)
		{
			toggleActiveWindowCenteredPerspective();
		}
		m_isPerspectiveChangeLock = true;
		m_PerspectiveAction->setEnabled(false);
	}
	else
	{
		m_isPerspectiveChangeLock = false;
		m_PerspectiveAction->setEnabled(true);
		if (params.perspectiveView != m_stateUseOrthographic)
		{
			toggleActiveWindowCenteredPerspective();
		}
	}
}
QAction* MainWindow::getMainWindowAction(const QString & strObjectName)
{

    return this->findChild<QAction*>(strObjectName);
}
void MainWindow::setMainWindowNavigationEnable(bool bEnable)
{
    SARibbonBar* pRibonBar = MainWindow::TheInstance()->getMainTopNavigation();
    if (pRibonBar) {
        pRibonBar->ribbonTabBar()->setEnabled(bEnable);
        pRibonBar->applicationButton()->setDisabled(!bEnable);
    }
    return;
}


void MainWindow::slotContourAction(void)
{
	if (!haveOneSelection())
	{
		return;
	}
    deselectAllOtherObject(true);
    setGlobalZoom();
	ccHObject* ent = m_selectedEntities.front();
	if (!ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccConsole::Error(tr("Select a point cloud!"));
		return;
	}
    freezeUI(true);
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	TrionContourDlg * contourDlg = new TrionContourDlg(cloud,getActiveGLWindow() ,this);
    contourDlg->setAttribute(Qt::WA_DeleteOnClose, true);
    connect(contourDlg, &TrionContourDlg::processFinshed, this, &MainWindow::slotExitFunction);
	contourDlg->show();
}



void MainWindow::resetClippingBoxTool()
{
    if (m_isClippingBoxOpen && m_clipTool)
    {
        m_clipTool->reset();
    }
}

void MainWindow::updateCombineColorMenuState()
{
}





