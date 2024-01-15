#include "createpointcloudorthoimagedialog.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QToolButton>
#include <QPushButton>
#include <QComboBox>
#include <QThread>
#include<QtConcurrent>
#include<QFuture>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QImage>
#include <QFile>
#include <QStandardItemModel>
#include <QStandardPaths>
#include <ManualSegmentationTools.h>
#include <QDebug>
#include <QFuture>
#include <QEvent>
#include "ccRasterizeTool.h"
#include "ccImage.h"
#include "ccProgressDialog.h"
#include "ccPickingHub.h"
#include "ccScalarField.h"
#include "ccGraphicalSegmentationTool.h"
#include "framelessmessagebox.h"
#include "ccPointCloud.h"
#include "mainwindow.h"
#include "ccshowrect.h"
#include "ccSurface.h"
#include "ccPolyline.h"
#include "ccRasterGrid.h"
#include "model/projectmodel.h"
#include "model/modelchangedlocker.h"
#include "ccClipBox.h"
#include "cswidgets/framelessmessagebox.h"
#include "cswidgets/framelessfiledialog.h"
#include "libs/qCC_io/include/RasterGridFilter.h"
#include "cloudcomparewidgets/metahubframelesswaitdialog.h"

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
		class CreatePointCloudOrthoImagePrivate : public QObject
		{
			Q_OBJECT
		public:
			explicit CreatePointCloudOrthoImagePrivate(CreatePointCloudOrthoImageDialog *ptr);
			virtual ~CreatePointCloudOrthoImagePrivate(void);

			//! Layer types
			enum LayerType {
				LAYER_HEIGHT = 0,
				LAYER_RGB = 1,
				LAYER_SF = 2
			};

            int getPDirection(){ return m_nPDirection_; }
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

		private slots:
			/**
			*@brief 投影方向发生变化
			*/
			void slotUIProjectionDirectionChanged(void);

			/**
			*@brief 拾取框选状态发生变化
			*/
			void slotUISegmentationModeStatusChanged(void);

			/**
			*@brief 拾取框选模式完成
			*/
			void slotUISegmentationModeFinish(void);

			/**
			*@brief UI网格步长发生变化
			*/
			void slotUIGridStepChanged(void);

			/**
			*@brief 点云预览
			*/
			void slotUIPointCloudResultPreviewChanged(void);

			/**
			*@brief UI重置预览
			*/
			void slotUIRestPreviewChanged(void);

		private:
			/**
			*@brief 退出正射影像对话框
			*/
			void closeCreatePointCloudOrthoDialog(void);

		protected:
			/**
			*@更新数据到UI层
			*/
			void updateUIFromModel(bool bEnalbeUnpdatePlan = true);

			/**
			*@brief 更新显示
			*/
			void updateUIDisplayFromModel(void);


			/**
			*@brief 更新框选点云数据的信息到UI界面
			*/
			void updateUIFromSegmentationPointCloud(void);


			/**
			*@brief 更新网格
			*/
			void updateUIGridStep(void);

			/**
			*@brief 更新UI分辨率
			*/
			void updateUIImageFram(void);

			/**
			*@brief 更新UI点云标量信息
			*/
			void updateUIPointClodScalar(void);

			/**
			*@brief 生成点云对应的GeoTiff文件
			*@param 同时输出的图片数据
			*@param 结果点云数据
			*@param 输入路径
			*/
			int createResultPoindClodGeoTiff(const QString &strPathName, ccHObject **pResultPointClod);

			/**
			*@brief 加载结果点云数据到视图
			*/
			int addPoindClodGeoTiffResultDb(const QString &strPath, const QString &strNodeName);

			/**
			*@brief 网格数据有效性警告
			*/
			bool exeWarnsGridDataValidation(void);

			/*
			*@brief 设置导出使能
			*/
			void setExportEnable(bool bEnable);

			/**
			*@brief 修改激活的实体是否显示
			*/
			void setActiveEntityVisible(bool bVisable);

			/**
			*@brief 执行重置结果预览
			*/
			void exeResetResultPreview(void);
		private:
			//[!].添加拾取点
			bool addAlignedPoint(CCVector3d& Pin);

			/**
			*@brief 更新参考平面
			*/
			void updateReferencePlane(void);

			/**
			*@brief 设置截取框选使能
			*/
			void setGraphicalSegmentationEnable(bool bEnable);

			/**
			*@brief 更新视图视角
			*@param int 0预览， 1重置
			*/
			void updateViewPerspective(int nPerspective);

			//! Returns strategy for empty cell filling (extended version)
			ccRasterGrid::EmptyCellFillOption getFillEmptyCellsStrategyExt(double& emptyCellsHeight,
				double& minHeight,
				double& maxHeight) const;

			//! Returns the empty cell strategy (for a given combo-box)
			ccRasterGrid::EmptyCellFillOption getFillEmptyCellsStrategy(QComboBox* comboBox) const;

	
		
		private:
			QLabel			*m_pRegionSelectionLabel = nullptr;
			QLabel			*m_pGridStepsLabel = nullptr;
			QLabel			*m_pImageSizeLabel = nullptr;
			QLabel			*m_pProjectionDirectionLabel = nullptr;
			QLabel			*m_pDisplaysScalarsLabel = nullptr;
			QLabel			*m_pInterpolateValuesLabel = nullptr;
			QLabel			*m_pEmptyPixelFillLabel = nullptr;
			QLabel			*m_pPixelHeightLabel = nullptr;
			QLabel			*m_pMaxSdeLengthLabel = nullptr;

			QDoubleSpinBox	*m_pGridStepsDoubleSpinBox = nullptr;		///<网格步长尺寸
			QLineEdit		*m_pImageSizeValueLabel = nullptr;			///<图像尺寸
			QComboBox		*m_pProjectionDirectionCombox = nullptr;	///<投影方向
			QComboBox		*m_pDisplaysScalarsCombox = nullptr;		///<显示标量
			QComboBox		*m_pInterpolateValuesCombox = nullptr;		///<插值取值
			QComboBox		*m_pEmptyPixelFillCombox = nullptr;			///<空像素填充
			QComboBox		*m_pPixelHeightCombox = nullptr;			///<像素高度
			QDoubleSpinBox	*m_pMaxSideLengthSpinBox = nullptr;			///<最大的边长
			
		private:
			ccGraphicalSegmentationTool *m_pSegmentTool = nullptr;
			ccPickingHub                *m_pickingHub = nullptr;
		private:
			ccHObject		*m_pCurrentActiveEntity = nullptr;			///<当前激活的实体
			ccGLWindow		*m_ccGLWindows3D = nullptr;
		private:
			QToolButton		*m_pRectSelectionButton = nullptr;
			QToolButton		*m_pPolylineSelectionButton = nullptr;
			QPushButton		*m_pResetButton = nullptr;						///<按钮
		private:
			ccSurface		*m_pSurface = nullptr;							//<参考平面														
			ccShowRect		*m_RectSurface = nullptr;						//<任意平面参考平面
			ccPointCloud	m_pickingPoints;								///<自定义平面选点
			bool			m_bResultPreviewStatus = false;					///<结果预览
            int             m_nPDirection_ = 2;                          //投影方向默认z轴
            unsigned int    m_gridWidth = 0;
            unsigned int    m_gridHeight = 0;
            double          m_dStep = 0;
            ccBBox          m_box;
		private:
			ccGenericPointCloud    *m_pActiveSegmentationPointCloud = nullptr;		///<当前激活的框选数据
			ccHObject			   *m_pPointCloudGeoTiff = nullptr;
			QString				   m_currentImageResult;
			ccRasterGrid m_grid;
			int m_nImageNameIndex = 1;
		private:
			friend class CS::MetahubWidgets::CreatePointCloudOrthoImageDialog;
			CS::MetahubWidgets::CreatePointCloudOrthoImageDialog	*m_pqtr = nullptr;

		};
	
		

	}
}

CreatePointCloudOrthoImagePrivate::CreatePointCloudOrthoImagePrivate(CreatePointCloudOrthoImageDialog *ptr)
	:m_pSurface(new ccSurface)
	,m_RectSurface(new ccShowRect)
{
	m_pqtr = ptr;

}

CreatePointCloudOrthoImagePrivate::~CreatePointCloudOrthoImagePrivate(void)
{
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::createWidget(void)
{
	m_pqtr->setFixedSize(QSize(465, 470));
	int nLabelFixeWidth = 180;
	int nFixeWidth = 184;
	int nFixeHeight = 32;
	QVBoxLayout *pVMainLayout = new QVBoxLayout();
	pVMainLayout->setContentsMargins(16, 16, 16, 16);
	pVMainLayout->setSpacing(15);

	//[!].区域选择
	QHBoxLayout *pVSectionRectLayout = new QHBoxLayout();
	pVSectionRectLayout->setContentsMargins(0, 0, 0, 0);
	pVSectionRectLayout->setSpacing(0);
	m_pRegionSelectionLabel = new QLabel();
	m_pRegionSelectionLabel->setAlignment(Qt::AlignLeft);
	m_pRegionSelectionLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pRegionSelectionLabel->setFixedWidth(130);
	pVSectionRectLayout->addWidget(m_pRegionSelectionLabel, 0, Qt::AlignLeft | Qt::AlignVCenter);
	pVSectionRectLayout->addSpacing(5);
	m_pickingHub = new ccPickingHub(MainWindow::TheInstance(), this);
	m_pSegmentTool = new ccGraphicalSegmentationTool(m_pqtr, m_pickingHub);
	m_pSegmentTool->setMultipleSegmentationEnable(false);
	m_pRectSelectionButton = m_pSegmentTool->findChild<QToolButton*>("actionSetRectangularSelectionbtn");
	if (m_pRectSelectionButton){
		pVSectionRectLayout->addWidget(m_pRectSelectionButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
		pVSectionRectLayout->addSpacing(35);
		m_pRectSelectionButton->setFixedSize(QSize(nFixeHeight, nFixeHeight));
	}
	m_pPolylineSelectionButton = m_pSegmentTool->findChild<QToolButton*>("actionSetPolylineSelectionbtn");
	if (m_pPolylineSelectionButton){
		pVSectionRectLayout->addWidget(m_pPolylineSelectionButton, 0, Qt::AlignLeft | Qt::AlignVCenter);
		m_pPolylineSelectionButton->setFixedSize(QSize(nFixeHeight, nFixeHeight));
	}

	pVSectionRectLayout->addStretch();
	pVMainLayout->addLayout(pVSectionRectLayout);



	//[!].网格尺寸
	QHBoxLayout *pVGridStepLayout = new QHBoxLayout();
	pVGridStepLayout->setContentsMargins(0, 0, 0, 0);
	pVGridStepLayout->setSpacing(0);
	QVBoxLayout *pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pGridStepsLabel = new QLabel();
	m_pGridStepsLabel->setAlignment(Qt::AlignLeft);
	m_pGridStepsLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pGridStepsLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pGridStepsLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pGridStepsDoubleSpinBox = new QDoubleSpinBox();
	m_pGridStepsDoubleSpinBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pGridStepsDoubleSpinBox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pGridStepsDoubleSpinBox, 0, Qt::AlignLeft | Qt::AlignTop);
	pVGridStepLayout->addLayout(pTempVLayout);
	pVGridStepLayout->addSpacing(20);

	//[!].图像尺寸
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pImageSizeLabel = new QLabel();
	m_pImageSizeLabel->setAlignment(Qt::AlignLeft);
	m_pImageSizeLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pImageSizeLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pImageSizeLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pImageSizeValueLabel = new QLineEdit();
	m_pImageSizeValueLabel->setAlignment(Qt::AlignLeft);
	m_pImageSizeValueLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pImageSizeValueLabel->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	m_pImageSizeValueLabel->setReadOnly(true);
	pTempVLayout->addWidget(m_pImageSizeValueLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	pVGridStepLayout->addLayout(pTempVLayout);
	pVGridStepLayout->addStretch();
	pVMainLayout->addLayout(pVGridStepLayout);
	

	//[!].投影方向以及标量设置
	QHBoxLayout *pHLayoutDir = new QHBoxLayout();
	pHLayoutDir->setContentsMargins(0, 0, 0, 0);
	pHLayoutDir->setSpacing(0);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pProjectionDirectionLabel = new QLabel();
	m_pProjectionDirectionLabel->setAlignment(Qt::AlignLeft);
	m_pProjectionDirectionLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pProjectionDirectionLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pProjectionDirectionLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pProjectionDirectionCombox = new QComboBox();
	m_pProjectionDirectionCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pProjectionDirectionCombox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pProjectionDirectionCombox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutDir->addLayout(pTempVLayout);
	pHLayoutDir->addSpacing(20);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pDisplaysScalarsLabel = new QLabel();
	m_pDisplaysScalarsLabel->setAlignment(Qt::AlignLeft);
	m_pDisplaysScalarsLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pDisplaysScalarsLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pDisplaysScalarsLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pDisplaysScalarsCombox = new QComboBox();
	m_pDisplaysScalarsCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pDisplaysScalarsCombox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pDisplaysScalarsCombox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutDir->addLayout(pTempVLayout);
	pHLayoutDir->addStretch();
	pVMainLayout->addLayout(pHLayoutDir);


	//[!].最大边长，插值取值
	QHBoxLayout *pHLayoutMaxLength = new QHBoxLayout();
	pHLayoutMaxLength->setContentsMargins(0, 0, 0, 0);
	pHLayoutMaxLength->setSpacing(0);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pMaxSdeLengthLabel = new QLabel();
	m_pMaxSdeLengthLabel->setAlignment(Qt::AlignLeft);
	m_pMaxSdeLengthLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pMaxSdeLengthLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pMaxSdeLengthLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pMaxSideLengthSpinBox = new QDoubleSpinBox();
	m_pMaxSideLengthSpinBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pMaxSideLengthSpinBox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pMaxSideLengthSpinBox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutMaxLength->addLayout(pTempVLayout);
	pHLayoutMaxLength->addSpacing(20);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pInterpolateValuesLabel = new QLabel();
	m_pInterpolateValuesLabel->setAlignment(Qt::AlignLeft);
	m_pInterpolateValuesLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pInterpolateValuesLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pInterpolateValuesLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pInterpolateValuesCombox = new QComboBox();
	m_pInterpolateValuesCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pInterpolateValuesCombox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pInterpolateValuesCombox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutMaxLength->addLayout(pTempVLayout);
	pHLayoutMaxLength->addStretch();
	pVMainLayout->addLayout(pHLayoutMaxLength);

	//[!].像素高度  空像素填充
	QHBoxLayout *pHLayoutPixelHeight = new QHBoxLayout();
	pHLayoutPixelHeight->setContentsMargins(0, 0, 0, 0);
	pHLayoutPixelHeight->setSpacing(0);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pPixelHeightLabel = new QLabel();
	m_pPixelHeightLabel->setAlignment(Qt::AlignLeft);
	m_pPixelHeightLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pPixelHeightLabel->setFixedWidth(nLabelFixeWidth);
	pTempVLayout->addWidget(m_pPixelHeightLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pPixelHeightCombox = new QComboBox();
	m_pPixelHeightCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pPixelHeightCombox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pPixelHeightCombox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutPixelHeight->addLayout(pTempVLayout);
	pHLayoutPixelHeight->addSpacing(20);
	pTempVLayout = new QVBoxLayout();
	pTempVLayout->setContentsMargins(0, 0, 0, 0);
	pTempVLayout->setSpacing(3);
	m_pEmptyPixelFillLabel = new QLabel();
	m_pEmptyPixelFillLabel->setAlignment(Qt::AlignLeft);
	m_pEmptyPixelFillLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pEmptyPixelFillLabel->setFixedWidth(nLabelFixeWidth - 20);
	pTempVLayout->addWidget(m_pEmptyPixelFillLabel, 0, Qt::AlignLeft | Qt::AlignTop);
	m_pEmptyPixelFillCombox = new QComboBox();
	m_pEmptyPixelFillCombox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pEmptyPixelFillCombox->setFixedSize(QSize(nFixeWidth, nFixeHeight));
	pTempVLayout->addWidget(m_pEmptyPixelFillCombox, 0, Qt::AlignLeft | Qt::AlignTop);
	pHLayoutPixelHeight->addLayout(pTempVLayout);
	pHLayoutPixelHeight->addStretch();
	pVMainLayout->addLayout(pHLayoutPixelHeight);
	pVMainLayout->addStretch();
	m_pqtr->getApplyButton()->setVisible(true);
	m_pqtr->setLayout(pVMainLayout);

	m_pResetButton = new QPushButton();
	m_pResetButton->setProperty("metahucancelbbutton", true);
	m_pResetButton->setFixedWidth(80);
	m_pResetButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_pqtr->bottomLayout()->insertWidget(2, m_pResetButton);


	//[!].初始化数据
	m_pGridStepsDoubleSpinBox->setRange(0.001, 99.0000);
	m_pGridStepsDoubleSpinBox->setDecimals(3);
	m_pGridStepsDoubleSpinBox->setSingleStep(0.001);
	m_pGridStepsDoubleSpinBox->setValue(0.001);

	//[!].最大边长
	m_pMaxSideLengthSpinBox->setRange(0, 1000);
	m_pMaxSideLengthSpinBox->setDecimals(0);
	m_pMaxSideLengthSpinBox->setSingleStep(1);
	m_pMaxSideLengthSpinBox->setValue(1);

	//[!].投影方向
	m_pProjectionDirectionCombox->addItem("X", 0);
	m_pProjectionDirectionCombox->addItem("Y", 1);
	m_pProjectionDirectionCombox->addItem("Z", 2);
	m_pProjectionDirectionCombox->addItem(QCoreApplication::translate("createpointcloudorthoimagedialog", "Custom", nullptr), 3);
	m_pProjectionDirectionCombox->setCurrentIndex(2);

	//[!].插值取值
	m_pInterpolateValuesCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_MAXIMUM_VALUE);
	m_pInterpolateValuesCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_AVERAGE_VALUE);
	m_pInterpolateValuesCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_MINIMUM_VALUE);
	m_pInterpolateValuesCombox->setCurrentIndex(1);

	//[!].空像素填充
	m_pEmptyPixelFillCombox->addItem("", ccRasterGrid::EmptyCellFillOption::FILL_MAXIMUM_HEIGHT);
	m_pEmptyPixelFillCombox->addItem("", ccRasterGrid::EmptyCellFillOption::FILL_AVERAGE_HEIGHT);
	m_pEmptyPixelFillCombox->addItem("", ccRasterGrid::EmptyCellFillOption::FILL_MINIMUM_HEIGHT);
	m_pEmptyPixelFillCombox->addItem("", ccRasterGrid::EmptyCellFillOption::INTERPOLATE);
	m_pEmptyPixelFillCombox->setCurrentIndex(3);

	//[!].像素高度
	m_pPixelHeightCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_MAXIMUM_VALUE);
	m_pPixelHeightCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_AVERAGE_VALUE);
	m_pPixelHeightCombox->addItem("", ccRasterGrid::ProjectionType::PROJ_MINIMUM_VALUE);
	m_pPixelHeightCombox->setCurrentIndex(0);
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::createConnect(void)
{
	//[!].投影方向发生变化
	connect(m_pProjectionDirectionCombox, &QComboBox::currentTextChanged,
		this, &CreatePointCloudOrthoImagePrivate::slotUIProjectionDirectionChanged);

	connect(m_pSegmentTool, &ccGraphicalSegmentationTool::signalSegmentationModeStatusChanged,
		this, &CreatePointCloudOrthoImagePrivate::slotUISegmentationModeStatusChanged);

	connect(m_pSegmentTool, &ccGraphicalSegmentationTool::signalSegmentationModeFinish,
		this, &CreatePointCloudOrthoImagePrivate::slotUISegmentationModeFinish);

	connect(m_pGridStepsDoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
		this, &CreatePointCloudOrthoImagePrivate::slotUIGridStepChanged);

	connect(m_pResetButton, &QPushButton::clicked,
		this, &CreatePointCloudOrthoImagePrivate::slotUIRestPreviewChanged);

	connect(m_pDisplaysScalarsCombox, &QComboBox::currentTextChanged,
		this, [&]{
		updateUIDisplayFromModel();
		//[!].预览模式下调整
	});

	connect(m_pEmptyPixelFillCombox, &QComboBox::currentTextChanged,
		this, [&] {
		updateUIDisplayFromModel();
	});
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::retranslateUi()
{
	QString strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Selection Tool", nullptr);
	m_pRegionSelectionLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Grid Step", nullptr);
	m_pGridStepsLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Image Size", nullptr);
	m_pImageSizeLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Projection Direction", nullptr);
	m_pProjectionDirectionLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Displayed Scalar", nullptr);
	m_pDisplaysScalarsLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Interpolation", nullptr);
	m_pInterpolateValuesLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Empty Pixel Filling", nullptr);
	m_pEmptyPixelFillLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Pixel Height", nullptr);
	m_pPixelHeightLabel->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Export", nullptr);
	m_pqtr->getOKButton()->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Preview", nullptr);
	m_pqtr->getApplyButton()->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Reset", nullptr);
	m_pResetButton->setText(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Orthophoto", nullptr);
	m_pqtr->setWindowTitle(strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Max. Edge Length ", nullptr);
	m_pMaxSdeLengthLabel->setText(strTranslate);

	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Max.", nullptr);
	m_pInterpolateValuesCombox->setItemText(0, strTranslate);
	m_pEmptyPixelFillCombox->setItemText(0, strTranslate);
	m_pPixelHeightCombox->setItemText(0, strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Mean", nullptr);
	m_pInterpolateValuesCombox->setItemText(1, strTranslate);
	m_pEmptyPixelFillCombox->setItemText(1, strTranslate);
	m_pPixelHeightCombox->setItemText(1, strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Min.", nullptr);
	m_pInterpolateValuesCombox->setItemText(2, strTranslate);
	m_pEmptyPixelFillCombox->setItemText(2, strTranslate);
	m_pPixelHeightCombox->setItemText(2, strTranslate);
	strTranslate = QCoreApplication::translate("createpointcloudorthoimagedialog", "Interpolated", nullptr);
	m_pEmptyPixelFillCombox->setItemText(3, strTranslate);
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUIProjectionDirectionChanged(void)
{
	m_nPDirection_ = m_pProjectionDirectionCombox->currentData().toInt();
	if (m_nPDirection_ == 3){//[!].自定义平面就先清理框选
		m_pSegmentTool->exeCancelSegmentModeStatus();
		//[!].禁用拾取
		slotUISegmentationModeFinish();
		exeResetResultPreview();
	}

	//[!].先清理数据
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.clear();

    //自定义投影方向时不更新网格尺寸，因为网格尺寸和投影方向挂钩
    if (m_nPDirection_ != 3)
    {
        updateUIFromSegmentationPointCloud();
    }
	updateUIFromModel();
	MainWindow::TheInstance()->setGlobalZoom();
	emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUISegmentationModeStatusChanged(void)
{

	//[!].获取当前模式
	ccGraphicalSegmentationTool::CurrentSegmentMode mode = m_pSegmentTool->getCuurentSegmentMode();
	if (mode == ccGraphicalSegmentationTool::CurrentSegmentMode::NOPICKINGMODE){
		slotUISegmentationModeFinish();
	}
	else {
		setExportEnable(false);
	}


	
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUISegmentationModeFinish(void)
{
	if (!m_pCurrentActiveEntity || !m_ccGLWindows3D){
		return;
	}

	//[!].判断
	ccHObject *polyContainer = m_pSegmentTool->getPolyContainer();
	if (!polyContainer || polyContainer->getChildrenNumber() == 0){
		m_pActiveSegmentationPointCloud = ccHObjectCaster::ToGenericPointCloud(m_pCurrentActiveEntity);
		updateUIFromSegmentationPointCloud();
		updateUIFromModel();
		return;
	}


	ccGLCameraParameters camera;
	m_ccGLWindows3D->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;
	//[!].进行获取选择的点云
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_pCurrentActiveEntity);
	if (!cloud){
		return;
	}
	
	int cloudSize = static_cast<int>(cloud->size());
	CCCoreLib::ReferenceCloud InsidIndexes(cloud);
	ccGenericPointCloud::VisibilityTableType selectionTable;
	selectionTable.resize(cloudSize);
	ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_pCurrentActiveEntity);
	
#if defined(_OPENMP)
#pragma omp parallel for
#endif
	for (int i = 0; i < cloudSize; ++i)
	{
		if (selectionTable[i] == CCCoreLib::POINT_VISIBLE) {
			const CCVector3* P3D = cloud->getPoint(i);
			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P3D, Q2D, &pointInFrustum);
			bool pointInside = false;

			{
				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
					static_cast<PointCoordinateType>(Q2D.y - half_h));
				ccHObject* child = nullptr;
				for (int i = 0; i < polyContainer->getChildrenNumber(); i++)
				{
					child = polyContainer->getChild(i);
					if (child && int(child->getClassID()) == int(CC_TYPES::POLY_LINE)) {
						ccPolyline* curPoly = ccHObjectCaster::ToPolyline(child);
						assert(curPoly);
						pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, curPoly);
						if (pointInside)
							break;
					}
				}
			}

			if (pointInside) {
				selectionTable[i] = CCCoreLib::POINT_VISIBLE;
			}
			else {
				selectionTable[i] = CCCoreLib::POINT_HIDDEN;
			}

		}

	}

	for (int i = 0; i < cloudSize; ++i){
		if (selectionTable[i] == CCCoreLib::POINT_VISIBLE) {
			InsidIndexes.addPointIndex(i);
		}
	}
	if (pc){
		m_pActiveSegmentationPointCloud = static_cast<ccGenericPointCloud*>(pc->partialClone(&InsidIndexes));
	}
	updateUIFromSegmentationPointCloud();
	updateUIFromModel();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUIGridStepChanged(void)
{
	updateUIImageFram();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUIPointCloudResultPreviewChanged(void)
{
	m_currentImageResult = QString::null;
	if (!m_pCurrentActiveEntity){
		return;
	}


	//[!].判断有效性
	if (!exeWarnsGridDataValidation()) {
		return;
	}


	QString strTitle = QCoreApplication::translate("forestrysectorcategory", "Error", nullptr);
	QString strText = QCoreApplication::translate("createpointcloudorthoimagedialog", "Extraction failure. Ensure that the data is valid.", nullptr);
	//[!].执行算法以及结果
	ccHObject *pResultObject = nullptr;
	QString strPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation) + "//raster.tif";
	int nResult = createResultPoindClodGeoTiff(strPath, &pResultObject);
	if (nResult == -1) {//[!].算法执行错误
		strText = QCoreApplication::translate("createpointcloudorthoimagedialog", "Preview failed, please confirm data validity.", nullptr);
	}else if (nResult == -2){//[!].内存不足
		strText = QCoreApplication::translate("createpointcloudorthoimagedialog", "Insufficient memory caused the orthoimage preview to fail. Please modify the parameters and try again.", nullptr);
	}
	if (nResult < 0) {
		CS::Widgets::FramelessMessageBox::critical(MainWindow::TheInstance(), strTitle, strText);
		return;
	}

	//[!].弹框提示报错 预览时先隐藏, 取消框选
	m_pSegmentTool->exeCancelSegmentModeStatus();
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.clear();
	m_currentImageResult = strPath;
	m_pickingPoints.setEnabled(false);
	ccHObject* polyContainer = m_pSegmentTool->getPolyContainer();
	if (polyContainer) {
		polyContainer->setEnabled(false);
	}
	
	//[!].先卸载掉
	if (m_pPointCloudGeoTiff) {
		m_ccGLWindows3D->removeFromOwnDB(m_pPointCloudGeoTiff);
		if (ccHObjectCaster::ToPointCloud(m_pPointCloudGeoTiff)){
			ccHObjectCaster::ToPointCloud(m_pPointCloudGeoTiff)->clear();
		}
		delete m_pPointCloudGeoTiff;
		m_pPointCloudGeoTiff = nullptr;
	}

	m_pPointCloudGeoTiff = pResultObject;
	if (m_pPointCloudGeoTiff){
		m_pPointCloudGeoTiff->setVisible(true);
		m_pPointCloudGeoTiff->setEnabled(true);
		m_ccGLWindows3D->addToOwnDB(m_pPointCloudGeoTiff);
	}

	//[!].显示
	m_bResultPreviewStatus = true;
	updateUIFromModel();
	setActiveEntityVisible(false);
	//[!].切换视图
	updateViewPerspective(0);
	MainWindow::TheInstance()->setGlobalZoom();
	emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
	m_ccGLWindows3D->blockSignals(true);
	return;
}

void CreatePointCloudOrthoImagePrivate::exeResetResultPreview(void)
{
	if (!m_pCurrentActiveEntity || !m_ccGLWindows3D) {
		return;
	}

	m_ccGLWindows3D->blockSignals(false);
	if (m_pPointCloudGeoTiff) {
		m_ccGLWindows3D->removeFromOwnDB(m_pPointCloudGeoTiff);
		delete m_pPointCloudGeoTiff;
		m_pPointCloudGeoTiff = nullptr;
	}
	m_bResultPreviewStatus = false;

	//[!].取消框选
	m_currentImageResult = QString::null;
	m_pSegmentTool->exeCancelSegmentModeStatus();
	slotUISegmentationModeFinish();
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.clear();


	m_pickingPoints.setEnabled(true);
	ccHObject* polyContainer = m_pSegmentTool->getPolyContainer();
	if (polyContainer) {
		polyContainer->setEnabled(true);
	}
	updateViewPerspective(1);
	//重置再次显示
	setActiveEntityVisible(true);
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::slotUIRestPreviewChanged(void)
{
	if (!m_pCurrentActiveEntity || !m_ccGLWindows3D) {
		return;
	}
	
	int nDirection = m_pProjectionDirectionCombox->currentData().toInt();
	if (nDirection == 3){
		slotUIProjectionDirectionChanged();
	}

	//[!].重置结果
	exeResetResultPreview();
	updateUIFromModel();
	MainWindow::TheInstance()->setGlobalZoom();
	emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::closeCreatePointCloudOrthoDialog(void)
{
	m_ccGLWindows3D->blockSignals(false);
	m_pSegmentTool->stop(true);
	m_pSegmentTool->entities().clear();
	m_pSegmentTool->close();
	m_ccGLWindows3D->removeFromOwnDB(m_pSurface);
	m_ccGLWindows3D->removeFromOwnDB(m_RectSurface);
	MainWindow::TheInstance()->pickingHub()->removeListener(m_pqtr);
	m_pCurrentActiveEntity->setEnabled(true);
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.resize(0);
	m_ccGLWindows3D->removeFromOwnDB(&m_pickingPoints);

	if (m_pPointCloudGeoTiff) {
		m_ccGLWindows3D->removeFromOwnDB(m_pPointCloudGeoTiff);
		if (ccHObjectCaster::ToPointCloud(m_pPointCloudGeoTiff)) {
			ccHObjectCaster::ToPointCloud(m_pPointCloudGeoTiff)->clear();
		}
		delete m_pPointCloudGeoTiff;
		m_pPointCloudGeoTiff = nullptr;
	}

	updateViewPerspective(1);
	qDebug() << "display name " << m_ccGLWindows3D->windowTitle();
	m_pCurrentActiveEntity->setVisible(true);
	m_pCurrentActiveEntity->setEnabled(true);
	setActiveEntityVisible(true);
	MainWindow::TheInstance()->setGlobalZoom();
	emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateUIFromModel(bool bEnalbeUnpdatePlan)
{
	setGraphicalSegmentationEnable(false);
	if (!m_pCurrentActiveEntity || !m_pActiveSegmentationPointCloud){
		return;
	}
	//[!}.显示box
	ccBBox gridBBox = m_pCurrentActiveEntity->getOwnBB();
	gridBBox.setValidity(true);
	//[!].更新面的显示
	updateReferencePlane();
	
	//[!].更新标量显示界面禁用
	updateUIDisplayFromModel();

	//[!].如果为预览模式下
	if (m_bResultPreviewStatus){
		m_pRectSelectionButton->setEnabled(false);
		m_pPolylineSelectionButton->setEnabled(false);
	}

	//[!].如果拾取框选的数据为空禁用预览和导出
	if (m_pActiveSegmentationPointCloud->size() == 0){
		m_pqtr->getOKButton()->setEnabled(false);
		m_pqtr->getApplyButton()->setEnabled(false);
	}

	//[!].判断是否已经预览有结果
	m_pqtr->getOKButton()->setEnabled(!m_currentImageResult.isEmpty());
	m_pResetButton->setEnabled(m_bResultPreviewStatus);
	return;

}

void CreatePointCloudOrthoImagePrivate::updateUIDisplayFromModel(void)
{
	int iScalar = m_pDisplaysScalarsCombox->currentData().toInt();
	m_pPixelHeightCombox->setEnabled(iScalar == 0 ? true : false);//[!].RGB模式
	m_pInterpolateValuesCombox->setEnabled(iScalar == 2 ? true : false); //[!].RGB和高程模式禁用
	//[!].最大边长限制
	m_pMaxSideLengthSpinBox->setEnabled(m_pEmptyPixelFillCombox->currentData().toInt() == int(ccRasterGrid::INTERPOLATE));
	return;
}

static cc2DLabel* createLabel(cc2DLabel* label, ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
{
	assert(label);
	label->addPickedPoint(cloud, pointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->displayPointLegend(true);
	label->setDisplay(display);
	return label;
}

static cc2DLabel* createLabel(ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
{
	return createLabel(new cc2DLabel, cloud, pointIndex, pointName, display);
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateUIFromSegmentationPointCloud(void)
{
	if (!m_pActiveSegmentationPointCloud){
		return;
	}

	//[!].计算网格尺寸
	updateUIGridStep();

	//[!].计算分辨率
	updateUIImageFram();
	return;

}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateUIGridStep(void)
{
	if (!m_pActiveSegmentationPointCloud) {
		return;
	}

	ModelChangedLocker<QDoubleSpinBox>  locker(m_pGridStepsDoubleSpinBox);

	//[!].计算网格尺寸
	ccBBox box = m_pActiveSegmentationPointCloud->getBB_recursive();
	CCVector3d c = box.maxCorner() - box.minCorner();

	//[!].获取投影方式
    //int nData = m_pProjectionDirectionCombox->currentData().toInt();
    int nData = m_nPDirection_;
	double dMax = std::max(c.x, c.y);
	if (nData == 0){//[!].X方向
		dMax = std::max(c.y, c.z);
	}else if (nData == 1){//[!].Y方向
		dMax = std::max(c.x, c.z);
	}else if (nData == 2){//[!].Z反向投影
		dMax = std::max(c.x, c.y);
	}else if (nData == 3){//[!].自定义方向
		double g = sqrt(c.x * c.x + c.y * c.y);
		dMax = sqrt(g * g + c.z * c.z);
	}
	double dS = (dMax * 2.5) / 1920.0;
	m_pGridStepsDoubleSpinBox->setValue(dS);
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateUIImageFram(void)
{

	if (!m_pActiveSegmentationPointCloud) {
		return;
	}

	ccPointCloud* pCloud = static_cast<ccPointCloud*>(m_pActiveSegmentationPointCloud);
	if (!pCloud) {
		return;
	}


	ccGLMatrix rotate_matrix_inv, rotate_matrix2;
	int nPDirection_ori = m_nPDirection_;
	if (m_nPDirection_ != 2)
	{
		CCVector3 z_axis(0, 0, 1);
		CCVector3 normal(0, 1, 0);
		CCVector3 t(0, 0, 0);

		if (m_nPDirection_ == 3 && m_pickingPoints.size() == 3)
		{
			//自定义平投影面时，将点云旋转至投影平面法向量与z轴重合
			m_nPDirection_ = 2;//按z轴投影后

			CCVector3 v1 = *m_pickingPoints.getPoint(0);
			CCVector3 v2 = *m_pickingPoints.getPoint(1);
			CCVector3 v3 = *m_pickingPoints.getPoint(2);

			float A = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
			float B = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
			float C = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);
			float D = -(A*v1[0] + B * v1[1] + C * v1[2]);

			normal = CCVector3(A, B, C);
			normal.normalize();

			//Eigen::Vector3d z_axis0(0, 0, 1);
			//Eigen::Vector3d normal0(A, B, C);
			//normal0.normalize();//转为单位向量
			//Eigen::Matrix3d rotationMatrix;
			//Eigen::Matrix4d transformMatrix;
			//transformMatrix.setIdentity();
			//Eigen::Vector3d t(0, 0, 0);

			//rotationMatrix = Eigen::Quaterniond::FromTwoVectors(normal0, z_axis0).toRotationMatrix();
			//transformMatrix.block<3, 3>(0, 0) = rotationMatrix;
			//transformMatrix.topRightCorner(3, 1) = t;
		}
		else if(m_nPDirection_ == 0)
		{	
			m_nPDirection_ = 2;//按z轴投影后

			normal = CCVector3(1, 0, 0);
		}
		else if (m_nPDirection_ == 1)
		{
			m_nPDirection_ = 2;//按z轴投影后

			normal = CCVector3(0, 1, 0);
		}

		// 求两向量旋转矩阵
		rotate_matrix2 = ccGLMatrix::FromToRotation(normal, z_axis);
		rotate_matrix2.setTranslation(t);
		rotate_matrix_inv = ccGLMatrix::FromToRotation(z_axis, normal);
		rotate_matrix_inv.setTranslation(-t);

		pCloud->setGLTransformation(rotate_matrix2);
		pCloud->applyGLTransformation_recursive();
	}


	//[!].计算网格尺寸
	m_box = m_pActiveSegmentationPointCloud->getBB_recursive();
	m_dStep = m_pGridStepsDoubleSpinBox->value();
    //int dAisx = m_pProjectionDirectionCombox->currentData().toInt();
    int dAisx = m_nPDirection_;


	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
	ccRasterGrid::ComputeGridSize(dAisx, m_box, m_dStep, m_gridWidth, m_gridHeight);
	QString strSize = QString("%1*%2").arg(m_gridWidth).arg(m_gridHeight);
	m_pImageSizeValueLabel->setText(strSize);
	QSize size(m_gridWidth, m_gridHeight);
	m_pImageSizeValueLabel->setProperty("imageframsize", size);

	if (nPDirection_ori != 2)
	{
		//自定义平投影面时，还原点云原始位置
		pCloud->setGLTransformation(rotate_matrix_inv);
		pCloud->applyGLTransformation_recursive();
		m_nPDirection_ = nPDirection_ori;
	}


	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateUIPointClodScalar(void)
{
	if (!m_pCurrentActiveEntity) {
		return;
	}

	//[!].更新标量信息
	m_pDisplaysScalarsCombox->clear();
	m_pDisplaysScalarsCombox->addItem(ccRasterGrid::GetDefaultFieldName(ccRasterGrid::PER_CELL_HEIGHT), LAYER_HEIGHT);
	if (m_pCurrentActiveEntity->hasColors()) {
		m_pDisplaysScalarsCombox->addItem("RGB", LAYER_RGB);
	}
	if (m_pCurrentActiveEntity->isA(CC_TYPES::POINT_CLOUD) && m_pCurrentActiveEntity->hasScalarFields())
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_pCurrentActiveEntity);
		for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i) {
			m_pDisplaysScalarsCombox->addItem(pc->getScalarField(i)->getName(), QVariant(LAYER_SF));
		}
	}
	m_pDisplaysScalarsCombox->setEnabled(m_pDisplaysScalarsCombox->count() > 1);

	int index = m_pDisplaysScalarsCombox->findData(1);
	if (index >= 0){//存在
		m_pDisplaysScalarsCombox->setCurrentIndex(index);
	}else{
		m_pDisplaysScalarsCombox->setCurrentIndex(0);
	}
	return;
}

int CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::createResultPoindClodGeoTiff(
	const QString& strPathName, ccHObject**pResultPointClod)
{
	if (!m_pActiveSegmentationPointCloud){
		return -1;
	}
	ccPointCloud* pCloud = static_cast<ccPointCloud*>(m_pActiveSegmentationPointCloud);
	if (!pCloud){
		return -1;
	}

    //int nPDirection = m_pProjectionDirectionCombox->currentData().toInt();

    ccGLMatrix rotate_matrix_inv, rotate_matrix2;
    //添加自定义平面算法相关测试代码
    int nPDirection_ori = m_nPDirection_;
    if (m_nPDirection_ != 2)
    {
		CCVector3 z_axis(0, 0, 1);
		CCVector3 normal(0, 1, 0);
		CCVector3 t(0, 0, 0);

		if (m_nPDirection_ == 3 && m_pickingPoints.size() == 3)
		{
			//自定义平投影面时，将点云旋转至投影平面法向量与z轴重合
			m_nPDirection_ = 2;//按z轴投影后

			CCVector3 v1 = *m_pickingPoints.getPoint(0);
			CCVector3 v2 = *m_pickingPoints.getPoint(1);
			CCVector3 v3 = *m_pickingPoints.getPoint(2);

			float A = (v2[1] - v1[1]) * (v3[2] - v1[2]) - (v2[2] - v1[2]) * (v3[1] - v1[1]);
			float B = (v2[2] - v1[2]) * (v3[0] - v1[0]) - (v2[0] - v1[0]) * (v3[2] - v1[2]);
			float C = (v2[0] - v1[0]) * (v3[1] - v1[1]) - (v2[1] - v1[1]) * (v3[0] - v1[0]);
			float D = -(A * v1[0] + B * v1[1] + C * v1[2]);

			normal = CCVector3(A, B, C);
			normal.normalize();

			//Eigen::Vector3d z_axis0(0, 0, 1);
			//Eigen::Vector3d normal0(A, B, C);
			//normal0.normalize();//转为单位向量
			//Eigen::Matrix3d rotationMatrix;
			//Eigen::Matrix4d transformMatrix;
			//transformMatrix.setIdentity();
			//Eigen::Vector3d t(0, 0, 0);

			//rotationMatrix = Eigen::Quaterniond::FromTwoVectors(normal0, z_axis0).toRotationMatrix();
			//transformMatrix.block<3, 3>(0, 0) = rotationMatrix;
			//transformMatrix.topRightCorner(3, 1) = t;
		}
		else if (m_nPDirection_ == 0)
		{
			m_nPDirection_ = 2;//按z轴投影后

			normal = CCVector3(1, 0, 0);
		}
		else if (m_nPDirection_ == 1)
		{
			m_nPDirection_ = 2;//按z轴投影后

			normal = CCVector3(0, 1, 0);
		}

		// 求两向量旋转矩阵
		rotate_matrix2 = ccGLMatrix::FromToRotation(normal, z_axis);
		rotate_matrix2.setTranslation(t);
		rotate_matrix_inv = ccGLMatrix::FromToRotation(z_axis, normal);
		rotate_matrix_inv.setTranslation(-t);

		pCloud->setGLTransformation(rotate_matrix2);
		pCloud->applyGLTransformation_recursive();
        
    }

	//[!].配置参数  投影与标量信息
	//int nPDirection = m_pProjectionDirectionCombox->currentData().toInt();  
	int iScalar = m_pDisplaysScalarsCombox->currentData().toInt();
	QString strScalarName = m_pDisplaysScalarsCombox->currentText();
	int iInterPolate = m_pInterpolateValuesCombox->currentData().toInt();
	bool bCheckInterPolate = true;//默认开启
	bool activeLayerIsSF = (iScalar != LAYER_HEIGHT && iScalar != LAYER_RGB);//当激活层不是高程和颜色时，其他的都认为是激活了sf标量
	bool activeLayerIsRGB = (iScalar == LAYER_RGB);
	iInterPolate = bCheckInterPolate ? iInterPolate : ccRasterGrid::INVALID_PROJECTION_TYPE;
	bool interpolateSF = activeLayerIsSF && (iInterPolate != ccRasterGrid::INVALID_PROJECTION_TYPE);

	ccRasterGrid::ProjectionType projectionType = ccRasterGrid::ProjectionType(m_pPixelHeightCombox->currentData().toInt());
	ccRasterGrid::ProjectionType interpolateSFs = interpolateSF ? ccRasterGrid::ProjectionType(iInterPolate) : ccRasterGrid::INVALID_PROJECTION_TYPE;
	int nEmptyPixelFillType = m_pEmptyPixelFillCombox->currentData().toInt();
	bool interpolateEmptyCells = (nEmptyPixelFillType == int(ccRasterGrid::INTERPOLATE));
	double maxEdgeLength = m_pMaxSideLengthSpinBox->value();

	//[!].计算网格尺寸
	QSize size = m_pImageSizeValueLabel->property("imageframsize").toSize();
	//grid size
	unsigned gridTotalSize = size.width() * size.height();
	if (gridTotalSize == 1){

        if (nPDirection_ori != 2)
        {
            pCloud->setGLTransformation(rotate_matrix_inv);
            pCloud->applyGLTransformation_recursive();
			m_nPDirection_ = nPDirection_ori;
        }

		return -1;
	}

	//[!].显示等待框
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(MainWindow::TheInstance());
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("createpointcloudorthoimagedialog", "Generating the orthophoto...", nullptr));
	QCoreApplication::processEvents();
	QThread::msleep(50);
	int nResult = 0;
	ccHObject* pContainer = new ccHObject();
	QFuture<void> fu = QtConcurrent::run([&]() {
		//每次网格处理之前必须先初始化
		if (!m_grid.init(m_gridWidth, m_gridHeight, m_dStep, m_box.minCorner())) {
			qDebug() << "error! Not enough memory";
			nResult = -2;
			return;
		}

		if (!m_grid.fillWith(m_pActiveSegmentationPointCloud,
			m_nPDirection_,
			projectionType,
			interpolateEmptyCells,
			maxEdgeLength,
			interpolateSFs,
			nullptr))
		{
			if (nPDirection_ori != 2)
			{
				pCloud->setGLTransformation(rotate_matrix_inv);
				pCloud->applyGLTransformation_recursive();
				m_nPDirection_ = nPDirection_ori;
			}
			qDebug() << "error! fill with grid is error!";
			nResult = -1;
		}



		//[!].判断是否有效
		if (!m_grid.isValid()) {
			if (nPDirection_ori != 2)
			{
				pCloud->setGLTransformation(rotate_matrix_inv);
				pCloud->applyGLTransformation_recursive();
				m_nPDirection_ = nPDirection_ori;
			}
			qDebug() << "error! create grid is error!";
			nResult = -1;
		}

		bool hasScalarFields = !m_grid.scalarFields.empty();
		int visibleSfIndex = -1;
		if (activeLayerIsSF && m_pActiveSegmentationPointCloud->isA(CC_TYPES::POINT_CLOUD)) {
			visibleSfIndex = static_cast<ccPointCloud*>(m_pActiveSegmentationPointCloud)->getScalarFieldIndexByName(qPrintable(strScalarName));
		}

		ccRasterizeTool::ExportBands exportBands;
		exportBands.height = iScalar == LAYER_HEIGHT;
		exportBands.rgb = iScalar == LAYER_RGB;
		exportBands.visibleSF = activeLayerIsSF;

		std::string rotate_matrix_inv_str = rotate_matrix_inv.toString().toStdString();
		double dHeightForEmptyCells = 0.0;
		if (!ccRasterizeTool::ExportGeoTiff
		(strPathName,
			exportBands,
			ccRasterGrid::EmptyCellFillOption(nEmptyPixelFillType),
			m_grid,
			m_box,
			m_nPDirection_,
			dHeightForEmptyCells,
			m_pActiveSegmentationPointCloud,
			visibleSfIndex,
			rotate_matrix_inv_str
		)){
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
			CC_FILE_ERROR re = rasterFilter.loadFile(strPathName, *pContainer, parameters);
			if (re != CC_FERR_NO_ERROR || !pContainer) {
				qDebug() << "error! exe load Tiff is error!";
				nResult = -1;
				return;
			}
		}
		catch (const std::bad_alloc&){
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
    if (nPDirection_ori != 2)
    {
        //自定义平投影面时，还原点云原始位置
        pCloud->setGLTransformation(rotate_matrix_inv);
        pCloud->applyGLTransformation_recursive();
		m_nPDirection_ = nPDirection_ori;
    }

	if (pContainer->getChildrenNumber() > 0) {
		*pResultPointClod = pContainer->getFirstChild();
	}else{
		qDebug() << "error! load result pointcolud is nullptr!";
		nResult = -1;
	}
	
	return nResult;
}



int CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::addPoindClodGeoTiffResultDb(
	const QString & strPath, 
	const QString & strNodeName)
{
	FileIOFilter::LoadParameters parameters;
	parameters.alwaysDisplayLoadDialog = false;
	parameters.parentWidget = nullptr;

	RasterGridFilter rasterFilter;
	ccHObject *pContainer = new ccHObject();
	CC_FILE_ERROR re =	rasterFilter.loadFile(strPath, *pContainer, parameters);
	if (re != CC_FERR_NO_ERROR || !pContainer){
		return -1;
	}

	ccHObject *pTifNode = nullptr;
	if (pContainer->getChildrenNumber() > 0){
		pTifNode = pContainer->getFirstChild();
	}
	if (!pTifNode){
		return -1;
	}

	pTifNode->setName(strNodeName);
	m_pCurrentActiveEntity->addChild(pTifNode);
	MainWindow::TheInstance()->addToDB(pTifNode, false, false, false, false);
	return 0;
}

bool CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::exeWarnsGridDataValidation(void)
{

	QSize size = m_pImageSizeValueLabel->property("imageframsize").toSize();
	unsigned gridTotalSize = size.width() * size.height();
	if (gridTotalSize <= 5000000) {
		return true;
	}

	CS::Widgets::FramelessMessageBox message(
		QMessageBox::Question,
		QCoreApplication::translate("createpointcloudorthoimagedialog", "Warn", nullptr),
		QCoreApplication::translate("createpointcloudorthoimagedialog", "The total number of pixels exceeds 5 million and the generation time is long, do you want to continue?"),
		QMessageBox::No | QMessageBox::Yes,
		MainWindow::TheInstance());
	int type = message.exec();
	if ((QMessageBox::StandardButton)type != QMessageBox::Yes){
		return false;
	}
	return true;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::setExportEnable(bool bEnable)
{
	m_pqtr->getOKButton()->setEnabled(!m_currentImageResult.isEmpty() && bEnable);
	m_pqtr->getApplyButton()->setEnabled(bEnable);
	m_pResetButton->setEnabled(bEnable && m_bResultPreviewStatus);
}



void CreatePointCloudOrthoImagePrivate::setActiveEntityVisible(bool bVisable)
{
	qDebug() << "set display";
	if (!m_ccGLWindows3D || !m_pCurrentActiveEntity){
		qDebug() << "set display error!";
		return;
	}

	ccGLWindow* pWindow = bVisable ? m_ccGLWindows3D : nullptr;
	m_pCurrentActiveEntity->setDisplay_recursive(pWindow);
	int nDataDirection = m_nPDirection_;
	if (m_pSurface && nDataDirection != 3) {
		m_pSurface->setVisible(bVisable);
	}

	if (m_RectSurface && nDataDirection == 3){
		m_RectSurface->setVisible(bVisable);
	}
	return;
}

bool CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::addAlignedPoint(CCVector3d & Pin)
{
	m_pickingPoints.addPoint(Pin.toFloat());
	QString pointName = QString("P%1").arg(m_pickingPoints.size());
	cc2DLabel* label = createLabel(&m_pickingPoints, m_pickingPoints.size() - 1, pointName, m_pickingPoints.getDisplay());
	m_pickingPoints.addChild(label);
	return false;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateReferencePlane(void)
{
	if (!m_pCurrentActiveEntity) {
		return;
	}

	bool bEnable = m_pCurrentActiveEntity->isEnabled();
	ccGenericPointCloud* inputCloud = ccHObjectCaster::ToGenericPointCloud(m_pCurrentActiveEntity);
	if (!inputCloud) {
		return;
	}
	//[!].当前投影方向
	double min = 0;
	ccBBox box = inputCloud->getBB_recursive();
	CCVector3d offset = inputCloud->getGlobalShift();
	CCVector3d origin_minconer = box.minCorner().toDouble() - offset;
    //int nDataDirection = m_pProjectionDirectionCombox->currentData().toInt();
    int nDataDirection = m_nPDirection_;
	if (nDataDirection == 0 || nDataDirection == 1 || nDataDirection == 2) {//[!].固定轴
		m_pSurface->setSurfaceVisible(true);
		ccSurface::EPivotType type;
		if (nDataDirection == 0) {
			type = ccSurface::EPivotType::XAXIS;
			min = origin_minconer.x + offset.x - box.getCenter().x;
		}
		else if (nDataDirection == 1) {
			type = ccSurface::EPivotType::YAXIS;
			min = origin_minconer.y + offset.y - box.getCenter().y;
		}
		else if (nDataDirection == 2) {
			type = ccSurface::EPivotType::ZAXIS;
			min = origin_minconer.z + offset.z - box.getCenter().z;
		}

		m_pSurface->setCalcBBox(box);
		m_pSurface->updateSliceDirection(type);
		m_pSurface->updateHeight(0);
		m_pSurface->setScaling(1.1);
		m_pSurface->updatePos(min);
		m_pSurface->setOpenZFighting(true);
		m_RectSurface->setVisible(false);
		m_pSurface->setVisible(true && bEnable);
		box.setValidity(true);
		setGraphicalSegmentationEnable(true);
	}
	else if (nDataDirection) {//[!].任意平面
		m_pSurface->setVisible(false);
		if (m_pickingPoints.size() == 3) {
			m_RectSurface->setVisible(true && bEnable);
			CCVector3d v1 = *m_pickingPoints.getPoint(0);
			CCVector3d v2 = *m_pickingPoints.getPoint(1);
			CCVector3d v3 = *m_pickingPoints.getPoint(2);
			float na = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
			float nb = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
			float nc = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);

			CCVector3f normal = { na / sqrt(na*na + nb * nb + nc * nc), nb / sqrt(na*na + nb * nb + nc * nc), nc / sqrt(na*na + nb * nb + nc * nc) };
			double paramz = 1;
			if (normal.z < 0) {
				paramz = -1;
			}
			v1 += normal /** offset*/ * paramz;
			v2 += normal/* * offset*/ * paramz;
			v3 += normal /** offset*/ * paramz;
			CCVector3d centerPoint = (v1 + v2 + v3) / 3.0;
			double distance = (v1 - centerPoint).norm();
			double rectlegnght = 0;
			if (inputCloud) {
				ccBBox box = inputCloud->getBB_recursive();
				double curdistance = box.getDiagNormd() / 2;
				rectlegnght = std::max(rectlegnght, curdistance);
			}

			CCVector3d rectpoint1 = centerPoint + (v1 - centerPoint) / distance * rectlegnght;
			CCVector3d rectpoint2 = centerPoint - (v1 - centerPoint) / distance * rectlegnght;
			CCVector3d downpoint = (centerPoint + normal.toDouble() * 10);
			float na2 = (rectpoint1[1] - centerPoint[1])*(downpoint[2] - centerPoint[2]) - (rectpoint1[2] - centerPoint[2])*(downpoint[1] - centerPoint[1]);
			float nb2 = (rectpoint1[2] - centerPoint[2])*(downpoint[0] - centerPoint[0]) - (rectpoint1[0] - centerPoint[0])*(downpoint[2] - centerPoint[2]);
			float nc2 = (rectpoint1[0] - centerPoint[0])*(downpoint[1] - centerPoint[1]) - (rectpoint1[1] - centerPoint[1])*(downpoint[0] - centerPoint[0]);
			CCVector3f norma2 = { na2 / sqrt(na2*na2 + nb2 * nb2 + nc2 * nc2), nb2 / sqrt(na2*na2 + nb2 * nb2 + nc2 * nc2), nc2 / sqrt(na2*na2 + nb2 * nb2 + nc2 * nc2) };
			CCVector3d rectpoint3 = centerPoint + norma2 * rectlegnght;
			CCVector3d rectpoint4 = centerPoint - norma2 * rectlegnght;
			std::vector<CCVector3d> param;
			param.push_back(rectpoint1);
			param.push_back(rectpoint3);
			param.push_back(rectpoint2);
			param.push_back(rectpoint4);
			param.push_back(rectpoint1 + normal * 0.1);
			param.push_back(rectpoint3 + normal * 0.1);
			param.push_back(rectpoint2 + normal * 0.1);
			param.push_back(rectpoint4 + normal * 0.1);
			m_RectSurface->setPointData(param);
			setGraphicalSegmentationEnable(true);
		}
		else
		{
			m_RectSurface->setVisible(false);
			setGraphicalSegmentationEnable(false);
		}
	}
	emit CS::Model::ProjectModel::instance()->signalRedrawAllWindows();
	return;
}

ccRasterGrid::EmptyCellFillOption CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::getFillEmptyCellsStrategy(QComboBox* comboBox) const
{
	if (!comboBox)
	{
		assert(false);
		return ccRasterGrid::LEAVE_EMPTY;
	}

	switch (comboBox->currentData().toInt())
	{
	case 0:
		return ccRasterGrid::LEAVE_EMPTY;
	case 1:
		return ccRasterGrid::FILL_MINIMUM_HEIGHT;
	case 2:
		return ccRasterGrid::FILL_AVERAGE_HEIGHT;
	case 3:
		return ccRasterGrid::FILL_MAXIMUM_HEIGHT;
	case 4:
		return ccRasterGrid::FILL_CUSTOM_HEIGHT;
	case 5:
		return ccRasterGrid::INTERPOLATE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::LEAVE_EMPTY;

}

void CS::MetahubWidgets::CreatePointCloudOrthoImagePrivate::updateViewPerspective(int nPerspective)
{
	int nDirection = m_pProjectionDirectionCombox->currentData().toInt();
	if (nPerspective == 0){//[!].预览
		//[!].判断是否为固定轴
		if (nDirection == 3){
			return;
		}

		if (nDirection == 0){
			m_ccGLWindows3D->setView(CC_VIEW_ORIENTATION::CC_LEFT_VIEW);
		} else if (nDirection == 1){
			m_ccGLWindows3D->setView(CC_VIEW_ORIENTATION::CC_FRONT_VIEW);
		}else if (nDirection == 2){
			m_ccGLWindows3D->setView(CC_VIEW_ORIENTATION::CC_TOP_VIEW);
		}
	}else if (nPerspective == 1){//[!].重置
		m_ccGLWindows3D->setView(CC_VIEW_ORIENTATION::CC_TOP_VIEW);
	}
	return;
}

ccRasterGrid::EmptyCellFillOption CreatePointCloudOrthoImagePrivate::getFillEmptyCellsStrategyExt(
	double& emptyCellsHeight, 
	double& minHeight, 
	double& maxHeight) const
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_pEmptyPixelFillCombox);

	emptyCellsHeight = 0.0;
	minHeight = m_grid.minHeight;
	maxHeight = m_grid.maxHeight;

	switch (fillEmptyCellsStrategy)
	{
	case ccRasterGrid::LEAVE_EMPTY:
		//nothing to do
		break;
	case ccRasterGrid::FILL_MINIMUM_HEIGHT:
		emptyCellsHeight = m_grid.minHeight;
		break;
	case ccRasterGrid::FILL_MAXIMUM_HEIGHT:
		emptyCellsHeight = m_grid.maxHeight;
		break;
	case ccRasterGrid::FILL_CUSTOM_HEIGHT:
	case ccRasterGrid::INTERPOLATE:
	{
		//double customEmptyCellsHeight = getCustomHeightForEmptyCells();
		////update min and max height by the way (only if there are invalid cells ;)
		//if (m_grid.validCellCount != m_grid.width * m_grid.height)
		//{
		//	if (customEmptyCellsHeight <= m_grid.minHeight)
		//		minHeight = customEmptyCellsHeight;
		//	else if (customEmptyCellsHeight >= m_grid.maxHeight)
		//		maxHeight = customEmptyCellsHeight;
		//	emptyCellsHeight = customEmptyCellsHeight;
		//}
	}
	break;
	case ccRasterGrid::FILL_AVERAGE_HEIGHT:
		//'average height' is a kind of 'custom height' so we can fall back to this mode!
		fillEmptyCellsStrategy = ccRasterGrid::FILL_CUSTOM_HEIGHT;
		emptyCellsHeight = m_grid.meanHeight;
		break;
	default:
		assert(false);
	}

	return fillEmptyCellsStrategy;

}

void CreatePointCloudOrthoImagePrivate::setGraphicalSegmentationEnable(bool bEnable)
{
	m_pRectSelectionButton->setEnabled(bEnable);
	m_pPolylineSelectionButton->setEnabled(bEnable);

	//[!].更新状态,功能只能
	setExportEnable(bEnable);
}

CreatePointCloudOrthoImageDialog::CreatePointCloudOrthoImageDialog(QWidget *parent) 
	:CS::MetahubWidgets::MetahubFramelessDialog(parent)
	,m_dqtr(new CreatePointCloudOrthoImagePrivate(this))
{
	m_dqtr->createWidget();
	m_dqtr->createConnect();
	m_dqtr->retranslateUi();
}
CreatePointCloudOrthoImageDialog::~CreatePointCloudOrthoImageDialog(void)
{
	if (m_dqtr->m_pSegmentTool){
		m_dqtr->m_pSegmentTool->stop(true);
		m_dqtr->m_pSegmentTool->entities().clear();
		m_dqtr->m_pSegmentTool->close();
	} 
	
}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::onItemPicked(const PickedItem& pi)
{
	int nDataDirection = m_dqtr->m_pProjectionDirectionCombox->currentData().toInt();
	if (nDataDirection != 3 || !m_dqtr->m_pCurrentActiveEntity){
		return;
	}

	if (m_dqtr->m_pickingPoints.size() >= 3){
		m_dqtr->m_pickingPoints.removeAllChildren();
		m_dqtr->m_pickingPoints.clear();
	}

	CCVector3d P = pi.P3D.toDouble();
	m_dqtr->addAlignedPoint(P);
	m_dqtr->updateReferencePlane();
	m_dqtr->updateUIFromSegmentationPointCloud();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::slotUIButtonOk(void)
{
	
	if (m_dqtr->m_currentImageResult.isEmpty()) {
		return;
	}
	QString strDefaultPath = CS::Model::ProjectModel::instance()->property("defalutpath").toString();
	QDir defaultPathDir(strDefaultPath);
	if (strDefaultPath.isEmpty() || !defaultPathDir.exists()) {
		strDefaultPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
	}

	//[!].选择文件保存的路径
	FramelessFileDialog fileDialog(this);
	QString location = strDefaultPath;
	QString selectFilter;
	QString strName = QString("orthophoto%1").arg(m_dqtr->m_nImageNameIndex);

	QString strTip = QCoreApplication::translate("createpointcloudorthoimagedialog", "Save File", nullptr);
	QString sFileName = fileDialog.getSaveFileName(this, strTip, location + "//" + strName + ".tif", "*.tif;;*.tiff", &selectFilter);
	if (sFileName.isEmpty() || selectFilter.isEmpty()) {
		return;
	}

	QFileInfo file(sFileName);
	strDefaultPath = file.absoluteDir().path();
	CS::Model::ProjectModel::instance()->setProperty("defalutpath", strDefaultPath);
	//[!].获取选择的文件后缀类型
	QFileInfo fi;
	fi = QFileInfo(sFileName);
	sFileName = fi.path() + QString("/") + fi.baseName() + selectFilter.remove('*');


	strTip = QCoreApplication::translate("createpointcloudorthoimagedialog", "Tips", nullptr);
	QString strInfo = QCoreApplication::translate("createpointcloudorthoimagedialog", "Exported successfully.", nullptr);
	QFile::remove(sFileName);
	bool bResult = QFile::copy(m_dqtr->m_currentImageResult, sFileName);
	if (!bResult) {
		strTip = QCoreApplication::translate("MainWindowUI", "Error", nullptr);
		strInfo = QCoreApplication::translate("MainWindowUI", "Selecting a path cannot save data,Please reselect the path.", nullptr);
		CS::Widgets::FramelessMessageBox::warning(this, strTip, strInfo);
		return;
	}


	if (strName.compare(fi.baseName(), Qt::CaseInsensitive) == 0) {
		m_dqtr->m_nImageNameIndex++;
	}

	//[!].退出
	m_dqtr->m_currentImageResult = QString::null;
	getOKButton()->setEnabled(!m_dqtr->m_currentImageResult.isEmpty());
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::slotUIButtonCancel(void)
{

	//[!].检查是否有需要保存
	if (!m_dqtr->m_currentImageResult.isEmpty()) {
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

	m_dqtr->closeCreatePointCloudOrthoDialog();
	QDialog::accept();
	
}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::slotUIButtonApply(void)
{
	m_dqtr->slotUIPointCloudResultPreviewChanged();
	return;
}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::setCurrentActiveEntity(ccHObject *pHobject)
{
	m_dqtr->m_ccGLWindows3D = MainWindow::GetGLWindow("3D View");
	if (!m_dqtr->m_ccGLWindows3D || !pHobject){
		return;
	}

	m_dqtr->m_pCurrentActiveEntity = pHobject;
	m_dqtr->m_bResultPreviewStatus = false;
	m_dqtr->m_pActiveSegmentationPointCloud = ccHObjectCaster::ToGenericPointCloud(pHobject);
	m_dqtr->m_pSegmentTool->entities().clear();
	m_dqtr->m_pSegmentTool->setCuttingMode(ccGraphicalSegmentationTool::POINTCLOUDCUTTING);
	m_dqtr->m_pSegmentTool->linkWith(m_dqtr->m_ccGLWindows3D);
	m_dqtr->m_pSegmentTool->addEntity(pHobject);
	m_dqtr->m_pSegmentTool->start();
	m_dqtr->m_pSegmentTool->setVisible(false);
	m_dqtr->m_pPolylineSelectionButton->setVisible(true);
	m_dqtr->m_pRectSelectionButton->setVisible(true);

	//[!].
	m_dqtr->m_ccGLWindows3D->addToOwnDB(m_dqtr->m_pSurface);
	m_dqtr->m_ccGLWindows3D->addToOwnDB(m_dqtr->m_RectSurface);
	m_dqtr->m_pSurface->setVisible(false);
	m_dqtr->m_RectSurface->setVisible(false);


	//[!].自定义平面选点
	m_dqtr->m_pickingPoints.clear();
	m_dqtr->m_ccGLWindows3D->addToOwnDB(&m_dqtr->m_pickingPoints);
	m_dqtr->m_pickingPoints.setEnabled(true);
	m_dqtr->m_pickingPoints.setVisible(false);

	//[!].更新框选的数据信息到界面
	m_dqtr->updateUIFromSegmentationPointCloud();
	//[!].更新标量
	m_dqtr->updateUIPointClodScalar();

	//[!].切换视图,顶部视图
	m_dqtr->updateViewPerspective(1);
	MainWindow::TheInstance()->pickingHub()->addListener(this, true);
	return;

}

void CS::MetahubWidgets::CreatePointCloudOrthoImageDialog::updateUIFromModel(void)
{
	m_dqtr->updateUIFromModel();
}
#include "createpointcloudorthoimagedialog.moc"