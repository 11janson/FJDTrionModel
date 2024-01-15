#include "createpointcloudbasemapdialog.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QToolButton>
#include <QPushButton>
#include <QComboBox>
#include <QThread>
#include <QtConcurrent>
#include <QFuture>
#include <QLabel>
#include "ccRasterGrid.h"
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "ccClipBox.h"
#include "mainwindow.h"
#include "cswidgets/framelessmessagebox.h"
#include "cswidgets/framelessfiledialog.h"
#include "libs/qCC_io/include/RasterGridFilter.h"
#include "cloudcomparewidgets/metahubframelesswaitdialog.h"
#include "ccGLWindow.h"
#include "ccRasterGrid.h"
#include "ccRasterizeTool.h"
#ifdef CC_GDAL_SUPPORT
//GDAL
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_api.h>
#endif


using namespace CS;
using namespace CS::MetahubWidgets;
using namespace CS::Widgets;

namespace CS {
	namespace MetahubWidgets {
		//! Layer types
		enum LayerType {
			LAYER_HEIGHT = 0,
			LAYER_RGB = 1,
			LAYER_SF = 2
		};
		class CreatePointCloudBasemapPrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit CreatePointCloudBasemapPrivate(CreatePointCloudBasemapDialog* ptr);
			virtual ~CreatePointCloudBasemapPrivate(void);
			
		private:
			/**
		  *@brief 创建界面
		  */
			void createWidget(void);
			/**
			*@brief 创建连接
			*/
			void createConnect(void);
			/**
			*@brief 创建翻译
			*/
			void retranslateUi();


		private:
			/**
			*@brief 刷新当前视图方向
			*/
			void refreshCurrentViewDirection(void);

			/**
			*@brief 关闭生成底图
			*/
			void closeCreatePointCloudBasemap(void);

			/*
			*@brief 生成底图预览结果
			*/
			void createCloudBasemapPreviewResult(void);

			/**
			*@brief 计算尺寸步长
			*/
			double computeGridSizeStep(ccBBox box);

			/**
			*@brief 生成点云正射图像
			*/
			int createCloudPointCloudOrthoImage(ccHObject* pActivateEntity);


			/**
			*@brief 设置当前激活的实体的显示使能
			*/
			void setActivateEntityDisplayEnable(bool bEnable);
		private:
			QComboBox* m_pDisplaysScalarsCombox = nullptr;
			ccRasterGrid m_grid;
			int m_nImageNameIndex = 0;
			ccPointCloud* m_pCurrentPointCloudBasemap = nullptr;
		private:
			ccHObject* m_pActivateEntity = nullptr;										///<当前激活的实体对象
			friend class CS::MetahubWidgets::CreatePointCloudBasemapDialog;
			CS::MetahubWidgets::CreatePointCloudBasemapDialog* m_qptr = nullptr;
		};

	}
}

CreatePointCloudBasemapPrivate::CreatePointCloudBasemapPrivate
(CreatePointCloudBasemapDialog* ptr)
{
	m_qptr = ptr;

}

CreatePointCloudBasemapPrivate::~CreatePointCloudBasemapPrivate(void)
{
}


void CreatePointCloudBasemapPrivate::createWidget(void)
{
	m_qptr->setFixedSize(QSize(450, 200));
	QHBoxLayout* pMainHBoxLayout = new QHBoxLayout();
	pMainHBoxLayout->setContentsMargins(16, 16, 16, 16);
	pMainHBoxLayout->setSpacing(0);
	pMainHBoxLayout->addStretch();
	QLabel* pDisplayScalarsLabel = new QLabel();
	pDisplayScalarsLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
	pDisplayScalarsLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	pDisplayScalarsLabel->setFixedWidth(120);
	pMainHBoxLayout->addWidget(pDisplayScalarsLabel, 0, Qt::AlignRight | Qt::AlignVCenter);
	pMainHBoxLayout->setSpacing(10);
	m_pDisplaysScalarsCombox = new QComboBox();
	m_pDisplaysScalarsCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pDisplaysScalarsCombox->setFixedWidth(250);
	m_pDisplaysScalarsCombox->setFixedHeight(32);
	pMainHBoxLayout->addWidget(m_pDisplaysScalarsCombox, 0, Qt::AlignLeft | Qt::AlignVCenter);
	pMainHBoxLayout->addStretch();
	pDisplayScalarsLabel->setText(QCoreApplication::translate("trafficsitesurveycategory", "Displays scalars", nullptr));
	m_qptr->setLayout(pMainHBoxLayout);
}

void CreatePointCloudBasemapPrivate::createConnect(void)
{
	connect(m_pDisplaysScalarsCombox, &QComboBox::currentTextChanged,
		this, [&] {
		m_qptr->updateUIFromModel();
	});


}

double CreatePointCloudBasemapPrivate::computeGridSizeStep(ccBBox box)
{
	if (!box.isValid()){
		return 0.0;
	}

	int nPDirection = 2;
	unsigned char Z;
	Z = nPDirection;
	const unsigned char X = Z == 2 ? 0 : Z + 1;
	const unsigned char Y = X == 2 ? 0 : X + 1;
	CCVector3d boxDiag = box.maxCorner().toDouble() - box.minCorner().toDouble();
	double dStep = (boxDiag.u[X] * 1.0) / (1920.0 - 1.5);
	return dStep;
}

int CreatePointCloudBasemapPrivate::createCloudPointCloudOrthoImage(ccHObject* pActivateEntity)
{
	ccGenericPointCloud* pActivePointCloud = ccHObjectCaster::ToGenericPointCloud(pActivateEntity);
	if (!pActivateEntity || !pActivePointCloud){
		return -1;
	}
	

	//[!].计算步长
	int nPDirection_ = 2;
	ccBBox box = pActivateEntity->getBB_recursive();
	double dStep = computeGridSizeStep(box);

	ccRasterGrid::ProjectionType projectionType = ccRasterGrid::ProjectionType::PROJ_MAXIMUM_VALUE;
	int nEmptyPixelFillType = ccRasterGrid::EmptyCellFillOption::FILL_MINIMUM_HEIGHT;
	bool interpolateEmptyCells = (nEmptyPixelFillType == int(ccRasterGrid::INTERPOLATE));
	int iInterPolate = ccRasterGrid::ProjectionType::PROJ_MINIMUM_VALUE;
	double maxEdgeLength = 0;

	int iScalar = m_pDisplaysScalarsCombox->currentData().toInt();
	QString strScalarName = m_pDisplaysScalarsCombox->currentText();
	bool bCheckInterPolate = true;//默认开启
	bool activeLayerIsSF = (iScalar != LAYER_HEIGHT && iScalar != LAYER_RGB);//当激活层不是高程和颜色时，其他的都认为是激活了sf标量
	bool activeLayerIsRGB = (iScalar == LAYER_RGB);
	iInterPolate = bCheckInterPolate ? iInterPolate : ccRasterGrid::INVALID_PROJECTION_TYPE;
	bool interpolateSF = activeLayerIsSF && (iInterPolate != ccRasterGrid::INVALID_PROJECTION_TYPE);
	ccRasterGrid::ProjectionType interpolateSFs = interpolateSF ? ccRasterGrid::ProjectionType(iInterPolate) : ccRasterGrid::INVALID_PROJECTION_TYPE;
	QString strPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation) + "//raster.tif";


	//[!].计算宽高大小
	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
	ccRasterGrid::ComputeGridSize(2, box, dStep, gridWidth, gridHeight);

	//[!].显示等待框
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(MainWindow::TheInstance());
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("createpointcloudorthoimagedialog", "Generating the orthophoto...", nullptr));
	QCoreApplication::processEvents();
	QThread::msleep(50);
	int nResult = 0;
	ccHObject* pContainer = new ccHObject();
	ccHObject* pResultHObject = nullptr;
	QFuture<void> fu = QtConcurrent::run([&]() {
		//每次网格处理之前必须先初始化
		if (!m_grid.init(gridWidth, gridHeight, dStep, box.minCorner())) {
			qDebug() << "error! Not enough memory";
			nResult = -2;
			return;
		}

		if (!m_grid.fillWith(pActivePointCloud,
			nPDirection_,
			projectionType,
			interpolateEmptyCells,
			maxEdgeLength,
			interpolateSFs,
			nullptr))
		{
			qDebug() << "error! fill with grid is error!";
			nResult = -1;
		}



		//[!].判断是否有效
		if (!m_grid.isValid()) {
			qDebug() << "error! create grid is error!";
			nResult = -1;
		}

		bool hasScalarFields = !m_grid.scalarFields.empty();
		int visibleSfIndex = -1;
		if (activeLayerIsSF && pActivePointCloud->isA(CC_TYPES::POINT_CLOUD)) {
			visibleSfIndex = static_cast<ccPointCloud*>(pActivePointCloud)->getScalarFieldIndexByName(qPrintable(strScalarName));
		}

		ccRasterizeTool::ExportBands exportBands;
		exportBands.height = iScalar == LAYER_HEIGHT;
		exportBands.rgb = iScalar == LAYER_RGB;
		exportBands.visibleSF = activeLayerIsSF;

		
		std::string rotate_matrix_inv_str;
		double dHeightForEmptyCells = 0.0;
		if (!ccRasterizeTool::ExportGeoTiff
		(strPath,
			exportBands,
			ccRasterGrid::EmptyCellFillOption(nEmptyPixelFillType),
			m_grid,
			box,
			nPDirection_,
			dHeightForEmptyCells,
			pActivePointCloud,
			visibleSfIndex,
			rotate_matrix_inv_str
		)) {
			qDebug() << "error! exe exportgeoTiff is error!";
			nResult = -1;
			return;
		}


		try
		{
			//[!].加载预览
			FileIOFilter::LoadParameters parameters;
			parameters.alwaysDisplayLoadDialog = false;
			parameters.parentWidget = nullptr;
			RasterGridFilter rasterFilter;
			CC_FILE_ERROR re = rasterFilter.loadFile(strPath, *pContainer, parameters);
			if (re != CC_FERR_NO_ERROR || !pContainer) {
				qDebug() << "error! exe load Tiff is error!";
				nResult = -1;
				return;
			}
		}
		catch (const std::bad_alloc&) {
			qDebug() << "error! load not enough memory";
			nResult = -2;
			return;
		}

	});
	while (!fu.isFinished()) {
		QThread::msleep(20);
		WaitingDialog::MetahublFramelessWaitingdialog::instance(MainWindow::TheInstance())->jumpToNextFrame();
		QCoreApplication::processEvents();
	}

	WaitingDialog::MetahublFramelessWaitingdialog::instance(MainWindow::TheInstance())->stopWaiting();
	if (pContainer->getChildrenNumber() > 0) {
		pResultHObject = pContainer->getFirstChild();
	}
	else {
		qDebug() << "error! load result pointcolud is nullptr!";
		nResult = -1;
	}

	m_pCurrentPointCloudBasemap = ccHObjectCaster::ToPointCloud(pResultHObject);
	if (!m_pCurrentPointCloudBasemap){
		nResult = -1;
	}
	return nResult;
}
void CreatePointCloudBasemapPrivate::setActivateEntityDisplayEnable(bool bEnable)
{
	ccGLWindow* win = MainWindow::TheInstance()->getActiveGLWindow();
	if (!m_pActivateEntity || !win) {
		return;
	}

	ccGLWindow* pWindow = bEnable ? win : nullptr;
	m_pActivateEntity->setDisplay_recursive(pWindow);
	return;
}

void CreatePointCloudBasemapPrivate::closeCreatePointCloudBasemap(void)
{
	ccGLWindow* win = MainWindow::TheInstance()->getActiveGLWindow();
	if (!m_pActivateEntity || !win) {
		return;
	}

	//[!].设置激活的对象的显示
	setActivateEntityDisplayEnable(true);
	if (m_nImageNameIndex > 0){
		m_pActivateEntity->setEnabled(false);
	}
	else {
		m_pActivateEntity->setEnabled(true);
	}
	//[!].有数据先清除再展示
	if (m_pCurrentPointCloudBasemap) {
		win->removeFromOwnDB(m_pCurrentPointCloudBasemap);
		m_pCurrentPointCloudBasemap->clear();
		delete m_pCurrentPointCloudBasemap;
		m_pCurrentPointCloudBasemap = nullptr;
	}

	return;
}
void CreatePointCloudBasemapPrivate::createCloudBasemapPreviewResult(void)
{
	ccGLWindow* win = MainWindow::TheInstance()->getActiveGLWindow();
	if (!m_pActivateEntity || !win){
		return;
	}

	//[!].有数据先清除再展示
	if (m_pCurrentPointCloudBasemap){
		win->removeFromOwnDB(m_pCurrentPointCloudBasemap);
		m_pCurrentPointCloudBasemap->clear();
		delete m_pCurrentPointCloudBasemap;
		m_pCurrentPointCloudBasemap = nullptr;
	}


	//[!].查看类型
	int nResult = 0;
	int nType = m_pDisplaysScalarsCombox->currentData().toInt();
	if (nType >= 0){//[!].标量展示
		nResult = createCloudPointCloudOrthoImage(m_pActivateEntity);
	}
	else {
		try
		{
			ccPointCloud* pPointCloud = ccHObjectCaster::ToPointCloud(m_pActivateEntity);
			if (pPointCloud) {
				m_pCurrentPointCloudBasemap = pPointCloud->cloneThis(nullptr, true);
			}
			if (!m_pCurrentPointCloudBasemap){
				nResult = -1;
			}

		}
		catch (const std::bad_alloc&) {
			qDebug() << "error! load not enough memory";
			nResult = -2;
			return;
		}

	}

	if (nResult < 0 || !m_pCurrentPointCloudBasemap){
		qDebug() << "create point cloudbasemap is error!";
		return;
	}
	m_pCurrentPointCloudBasemap->setMetaData("pointcloudbasemap", true);
	win->addToOwnDB(m_pCurrentPointCloudBasemap);

}

void CreatePointCloudBasemapPrivate::retranslateUi()
{
	QString strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Preview", nullptr);
	m_qptr->getApplyButton()->setText(strTranslate);
	m_qptr->getApplyButton()->setVisible(true);
}
void CreatePointCloudBasemapPrivate::refreshCurrentViewDirection(void)
{
	ccGLWindow* win = MainWindow::TheInstance()->getActiveGLWindow();
	if (win){
		win->setView(CC_TOP_VIEW);
	}
}



CreatePointCloudBasemapDialog::CreatePointCloudBasemapDialog(QWidget* parent)
	:CS::MetahubWidgets::MetahubFramelessDialog(parent)
	, m_dqtr(new CreatePointCloudBasemapPrivate(this))
{
	m_dqtr->createWidget();
	m_dqtr->createConnect();
	m_dqtr->retranslateUi();
}
CreatePointCloudBasemapDialog::~CreatePointCloudBasemapDialog(void)
{
}

void CreatePointCloudBasemapDialog::slotUIButtonOk(void)
{
	if (!m_dqtr->m_pActivateEntity || !m_dqtr->m_pCurrentPointCloudBasemap){
		return;
	}
	ccHObject* pParent = m_dqtr->m_pActivateEntity->getParent();
	if (!pParent){
		return;
	}


	

	m_dqtr->m_pActivateEntity->setEnabled(false);
	int nMaxNum = 0;
	QString strName = m_dqtr->m_pActivateEntity->getName();
	std::vector<ccHObject*> ChildsList;
	m_dqtr->m_pActivateEntity->filterChildren(ChildsList);
	for (int i = 0; i < ChildsList.size(); i++){
		ChildsList.at(i)->setEnabled(false);
		if (!ChildsList.at(i)->getMetaData("pointcloudbasemap").toBool()){
			continue;
		}

		int nIndex = ChildsList.at(i)->getMetaData("pointcloudbasemapnum").toInt();
		if (nIndex > nMaxNum){
			nMaxNum = nIndex;
		}
	}
	nMaxNum++;
	m_dqtr->m_nImageNameIndex++;
	m_dqtr->m_pCurrentPointCloudBasemap->setName(QString("%1_map%2").arg(strName).arg(nMaxNum));
	m_dqtr->m_pCurrentPointCloudBasemap->setMetaData("pointcloudbasemapnum", nMaxNum);
	m_dqtr->m_pCurrentPointCloudBasemap->setEnabled(true);
	m_dqtr->m_pCurrentPointCloudBasemap->setMetaData("axistype", QVector3D(0, 0, 1));
	m_dqtr->m_pCurrentPointCloudBasemap->setMetaData("top_plane_center", QVector3D(0, 0, 1));
	pParent->addChild(m_dqtr->m_pCurrentPointCloudBasemap);
	MainWindow::TheInstance()->addToDB(pParent, false, true, false, false);
	m_dqtr->m_pCurrentPointCloudBasemap = nullptr;
	updateUIFromModel();
	return;

}

void CreatePointCloudBasemapDialog::slotUIButtonCancel(void)
{
	//[!].检查是否有需要保存
	if (m_dqtr->m_pCurrentPointCloudBasemap) {
		CS::Widgets::FramelessMessageBox message(QMessageBox::Question,
			QCoreApplication::translate("forestryextractiondialog", "Tip", nullptr),
			QCoreApplication::translate("forestryextractiondialog", "Changes not saved. Are you sure you want to exit?", nullptr),
			QMessageBox::No | QMessageBox::Yes,
			MainWindow::TheInstance());
		int type = message.exec();
		if ((QMessageBox::StandardButton)type != QMessageBox::Yes) {
			return;
		}
	}

	m_dqtr->closeCreatePointCloudBasemap();
	QDialog::accept();

}
void CreatePointCloudBasemapDialog::slotUIButtonApply(void)
{
	//[!].预览
	if (!m_dqtr->m_pActivateEntity){
		return;
	}

	m_dqtr->setActivateEntityDisplayEnable(false);

	//[!].生成预览结果图
	m_dqtr->createCloudBasemapPreviewResult();
	MainWindow::TheInstance()->setGlobalZoom();
	m_dqtr->refreshCurrentViewDirection();
	updateUIFromModel();
}

void CreatePointCloudBasemapDialog::setCurrentActiveEntity(ccHObject* pHobject)
{
	m_dqtr->m_pActivateEntity = pHobject;
	//[!].更新标量信息
	m_dqtr->m_pDisplaysScalarsCombox->clear();
	m_dqtr->m_pDisplaysScalarsCombox->addItem(ccRasterGrid::GetDefaultFieldName(ccRasterGrid::PER_CELL_HEIGHT), LAYER_HEIGHT);
	if (m_dqtr->m_pActivateEntity->hasColors()) {
		m_dqtr->m_pDisplaysScalarsCombox->addItem("RGB", LayerType::LAYER_RGB);
	}
	if (m_dqtr->m_pActivateEntity->isA(CC_TYPES::POINT_CLOUD) && m_dqtr->m_pActivateEntity->hasScalarFields())
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_dqtr->m_pActivateEntity);
		for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i) {
			m_dqtr->m_pDisplaysScalarsCombox->addItem(pc->getScalarField(i)->getName(), QVariant(LayerType::LAYER_SF));
		}
	}

	m_dqtr->m_pDisplaysScalarsCombox->setEnabled(m_dqtr->m_pDisplaysScalarsCombox->count() > 1);
	int index = m_dqtr->m_pDisplaysScalarsCombox->findData(1);
	if (index >= 0) {//存在
		m_dqtr->m_pDisplaysScalarsCombox->setCurrentIndex(index);
	}
	else {
		m_dqtr->m_pDisplaysScalarsCombox->setCurrentIndex(0);
	}

	QString strTranslate = QCoreApplication::translate("trafficsitesurveycategory", "Orthophoto", nullptr);
	m_dqtr->m_pDisplaysScalarsCombox->addItem(strTranslate, QVariant(-1));

	//[!].刷新视角
	m_dqtr->refreshCurrentViewDirection();

	updateUIFromModel();
	return;
}

void CreatePointCloudBasemapDialog::updateUIFromModel(void)
{
	if (!m_dqtr->m_pActivateEntity){
		return;
	}
	this->getOKButton()->setEnabled(m_dqtr->m_pCurrentPointCloudBasemap? true : false);
	return;
}
#include "createpointcloudbasemapdialog.moc"