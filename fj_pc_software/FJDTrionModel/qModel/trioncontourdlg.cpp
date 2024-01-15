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

#include "ui_trioncontourdlg.h"
#include "trioncontourdlg.h"
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QRadioButton>
#include <QCheckBox>
#include <QToolButton>
#include <QButtonGroup>
#include "qglobal.h"
#include "fjpointcloudutil.h"
#include "QCoreApplication"
#include "ccQtHelpers.h"
#include "mainwindow.h"
#include "ccContourLinesGenerator.h"
#include "ccScalarField.h"
#include "ccProgressDialog.h"
#include "framelessmessagebox.h"
#include "modelcontrol\cchobjectcontrol.h"
#include "ccDBRoot.h"


static void SetButtonColor(QAbstractButton* button, const QColor &col,bool isvalid)
{
    if (button != nullptr)
    {
        if (isvalid)
        {
            button->setStyleSheet(QStringLiteral("* { background-color: rgb(%1,%2,%3);border: 1px solid #989898; }")
                .arg(col.red())
                .arg(col.green())
                .arg(col.blue())
            );
        }
        else
        {
            button->setStyleSheet(QStringLiteral("* { background-color: rgba(%1,%2,%3,%4);border: 1px solid #989898; }")
                .arg(col.red())
                .arg(col.green())
                .arg(col.blue())
                .arg(30)
            );
        }

    }
}

TrionContourDlg::TrionContourDlg(ccGenericPointCloud* cloud, ccGLWindow* glWindow, QWidget* parent/*=nullptr*/)
	: CS::Widgets::FramelessDialog(parent, Qt::Tool)
	, m_ui( new Ui::TrionContourDlg)
	, m_cloud(cloud)
	, m_glWindow(glWindow)
	, m_rasterCloud(nullptr)
{
	m_ui->setupUi(this->getContentHolder());
    QButtonGroup * radioButtonGroup = new QButtonGroup(this);
    radioButtonGroup->addButton(m_ui->radioButton_default);
    radioButtonGroup->addButton(m_ui->radioButton_fromcloud);
	this->bottomWidget()->setVisible(false);
    setWindowTitle(tr("Contours"));
    initDefaultSetting();
	connect(m_ui->doubleSpinBox_gridinterval, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &TrionContourDlg::slotParamUpdate);
	connect(m_ui->doubleSpinBox_start, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &TrionContourDlg::slotMaxOrMinHeightChanged);
    connect(m_ui->doubleSpinBox_stop, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &TrionContourDlg::slotMaxOrMinHeightChanged);
	connect(m_ui->doubleSpinBox_interval, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &TrionContourDlg::slotParamUpdate);
	connect(m_ui->doubleSpinBox_minvertices, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &TrionContourDlg::slotParamUpdate);
	connect(m_ui->radioButton_default, &QRadioButton::toggled, this, &TrionContourDlg::slotColorFromChanged);
	connect(m_ui->radioButton_fromcloud, &QRadioButton::toggled, this, &TrionContourDlg::slotColorFromChanged);
	connect(m_ui->spinBox_intermediatecontour, qOverload<int>(&QSpinBox::valueChanged), this, &TrionContourDlg::slotParamUpdate);
	connect(m_ui->spinBox_countingcurve, qOverload<int>(&QSpinBox::valueChanged), this, &TrionContourDlg::slotParamUpdate);
	connect(m_ui->toolButton_intermediatecontour, &QToolButton::clicked, this, &TrionContourDlg::slotTntermediateContourColorSelected);
	connect(m_ui->toolButton_countingcurve, &QToolButton::clicked, this, &TrionContourDlg::slotCountingCurveColorSelected);
	connect(m_ui->pushButton_preview, &QAbstractButton::clicked, this, &TrionContourDlg::generateContours);
	connect(m_ui->pushButton_save, &QAbstractButton::clicked, this, &TrionContourDlg::exportContourLines);
	connect(m_ui->pushButton_cancel, &QAbstractButton::clicked, this, &TrionContourDlg::close);
    connect(m_ui->toolButton_setting, &QToolButton::clicked, this, &TrionContourDlg::slotDetailSetting);
    connect(m_ui->pushButton, &QToolButton::clicked, this, &TrionContourDlg::slotRestore);
    m_ui->frame->setVisible(false);
    QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ArrowUp.png");
    m_ui->toolButton_setting->setIcon(closeicon);
    m_ui->pushButton_save->setEnabled(false);
}

void TrionContourDlg::slotMaxOrMinHeightChanged()
{
    ccPointCloud* curcloud = ccHObjectCaster::ToPointCloud(m_cloud);
    if (curcloud)
    {
        ccBBox box = curcloud->getOwnBB();
        CCVector3& dimSumMin = box.minCorner();
        CCVector3& dimSumMax = box.maxCorner();
        m_ui->doubleSpinBox_start->setRange(dimSumMin.z, m_ui->doubleSpinBox_stop->value());
        m_ui->doubleSpinBox_stop->setRange(m_ui->doubleSpinBox_start->value(),dimSumMax.z);
        m_ui->doubleSpinBox_interval->setValue((dimSumMax.z - dimSumMin.z) / 10.0);
        m_ui->doubleSpinBox_interval->setRange((dimSumMax.z - dimSumMin.z) / 200.0, dimSumMax.z - dimSumMin.z);
    }


    slotParamUpdate();
}

TrionContourDlg::~TrionContourDlg()
{
	if (m_rasterCloud)
	{
		//if (m_glWindow)
		//	m_glWindow->removeFromOwnDB(m_rasterCloud);
		delete m_rasterCloud;
		m_rasterCloud = nullptr;
	}
	delete m_ui;
}

void TrionContourDlg::slotRestore()
{
    initDefaultSetting();
    slotParamUpdate();
}

void TrionContourDlg::initDefaultSetting()
{
    ccPointCloud* curcloud = ccHObjectCaster::ToPointCloud(m_cloud);
    if (curcloud)
    {
        ccBBox box = curcloud->getOwnBB();
        CCVector3& dimSumMin = box.minCorner();
        CCVector3& dimSumMax = box.maxCorner();
        m_ui->doubleSpinBox_start->setValue(dimSumMin.z);
        m_ui->doubleSpinBox_stop->setValue(dimSumMax.z);
        m_ui->doubleSpinBox_interval->setValue((dimSumMax.z - dimSumMin.z) / 10.0);
        m_ui->doubleSpinBox_interval->setRange((dimSumMax.z - dimSumMin.z)/200.0, dimSumMax.z - dimSumMin.z);
    }
    m_ui->doubleSpinBox_gridinterval->setValue(2.0);
    m_ui->doubleSpinBox_minvertices->setValue(5);
    m_ui->spinBox_intermediatecontour->setValue(2);
    m_ui->spinBox_countingcurve->setValue(4);
    m_ui->radioButton_default->setChecked(true);
    m_TntermediateContourColor = QColor(255, 0, 0);
    m_CountingCurveColor = QColor(0, 0, 255);  
    SetButtonColor(m_ui->toolButton_intermediatecontour, m_TntermediateContourColor,true);
    SetButtonColor(m_ui->toolButton_countingcurve, m_CountingCurveColor,true);
}

void TrionContourDlg::closeEvent(QCloseEvent *event)
{
    if (m_contourLines.size() > 0)
    {
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question, tr("Tip"), tr("The data has not been saved. Are you sure you want to exit?"), QMessageBox::Cancel | QMessageBox::Ok, this);
        if (message_box.exec() != QMessageBox::Ok)
        {
            event->ignore();
            return;
        }
    }
    removeContourLines();
    emit processFinshed(true);
    QWidget::closeEvent(event);
}

void TrionContourDlg::slotColorFromChanged()
{
    if (m_ui->radioButton_default->isChecked())
    {
        m_ui->spinBox_intermediatecontour->setEnabled(true);
        m_ui->spinBox_countingcurve->setEnabled(true);
        m_ui->toolButton_intermediatecontour->setEnabled(true);
        m_ui->toolButton_countingcurve->setEnabled(true);
        m_ui->spinBox_intermediatecontour->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        m_ui->spinBox_countingcurve->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        SetButtonColor(m_ui->toolButton_countingcurve, m_CountingCurveColor, true);
        SetButtonColor(m_ui->toolButton_intermediatecontour, m_TntermediateContourColor, true);
    }
    else
    {
        m_ui->spinBox_intermediatecontour->setEnabled(false);
        m_ui->spinBox_countingcurve->setEnabled(false);
        m_ui->spinBox_intermediatecontour->setButtonSymbols(QAbstractSpinBox::NoButtons);
        m_ui->spinBox_countingcurve->setButtonSymbols(QAbstractSpinBox::NoButtons);
        m_ui->toolButton_intermediatecontour->setEnabled(false);
        m_ui->toolButton_countingcurve->setEnabled(false);
        SetButtonColor(m_ui->toolButton_countingcurve, m_CountingCurveColor, false);
        SetButtonColor(m_ui->toolButton_intermediatecontour, m_TntermediateContourColor, false);
    }
    slotParamUpdate();
}

void TrionContourDlg::slotDetailSetting()
{
    bool isdetailvisable = m_ui->frame->isVisible();
    m_ui->frame->setVisible(!isdetailvisable);
    QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ArrowUp.png");
    if (!isdetailvisable)
    {
        closeicon = QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/ArrowDown.png");
    }
    QSize cursize(width(),455+48-120+72);
    if (!isdetailvisable)
    {
        cursize = QSize(width(), 455 + 52 + 72);
    }
    setFixedSize(cursize);
    m_ui->toolButton_setting->setIcon(closeicon);
}

void TrionContourDlg::slotParamUpdate()
{
	m_ui->pushButton_preview->setEnabled(true);
}


void TrionContourDlg::slotTntermediateContourColorSelected()
{
	bool isvalid = true;
	QColor newclor = FJPointCloudUtil::getQColorDialogColor(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr), m_TntermediateContourColor, isvalid);
	if (isvalid)
	{
		m_TntermediateContourColor = newclor;
        SetButtonColor(m_ui->toolButton_intermediatecontour, m_TntermediateContourColor,true);
		slotParamUpdate();
	}
}


void TrionContourDlg::slotCountingCurveColorSelected()
{
	bool isvalid = true;
	QColor newclor = FJPointCloudUtil::getQColorDialogColor(QCoreApplication::translate("ccEntityAction", "Select Color", nullptr), m_CountingCurveColor, isvalid);
	if (isvalid)
	{
		m_CountingCurveColor = newclor;
        SetButtonColor(m_ui->toolButton_countingcurve, m_CountingCurveColor,true);
		slotParamUpdate();
	}
}


ccRasterGrid::EmptyCellFillOption TrionContourDlg::getFillEmptyCellsStrategyExt(double& emptyCellsHeight,
	double& minHeight,
	double& maxHeight) const
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = ccRasterGrid::EmptyCellFillOption::FILL_AVERAGE_HEIGHT;

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
		double customEmptyCellsHeight = 0;
		//update min and max height by the way (only if there are invalid cells ;)
		if (m_grid.validCellCount != m_grid.width * m_grid.height)
		{
			if (customEmptyCellsHeight <= m_grid.minHeight)
				minHeight = customEmptyCellsHeight;
			else if (customEmptyCellsHeight >= m_grid.maxHeight)
				maxHeight = customEmptyCellsHeight;
			emptyCellsHeight = customEmptyCellsHeight;
		}
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

ccPointCloud* TrionContourDlg::convertGridToCloudBase(const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
	bool interpolateSF,
	bool interpolateColors,
	bool resampleInputCloudXY,
	bool resampleInputCloudZ,
	ccGenericPointCloud* inputCloud,
	bool fillEmptyCells,
	double emptyCellsHeight,
	bool exportToOriginalCS) const
{
	//projection dimension
	const unsigned char Z = static_cast<unsigned char>(2);
	assert(Z <= 2);

	//cloud bounding-box
	ccBBox box = m_cloud ? m_cloud->getOwnBB() : ccBBox();
	assert(box.isValid());

	return m_grid.convertToCloud(exportedFields,
		interpolateSF,
		interpolateColors,
		resampleInputCloudXY,
		resampleInputCloudZ,
		inputCloud,
		Z,
		box,
		fillEmptyCells,
		emptyCellsHeight,
		exportToOriginalCS);
}

ccPointCloud* TrionContourDlg::convertGridToCloud(const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
	bool interpolateSF,
	bool interpolateColors,
	bool copyHillshadeSF,
	const QString& activeSFName,
	bool exportToOriginalCS) const
{
	if (!m_cloud || !m_grid.isValid())
		return nullptr;

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(emptyCellsHeight,
		minHeight,
		maxHeight);

	//call parent method
	ccPointCloud* cloudGrid = convertGridToCloudBase(exportedFields,
		interpolateSF,
		interpolateColors,
		/*resampleInputCloudXY=*/false,
		/*resampleInputCloudZ=*/false,
		/*inputCloud=*/m_cloud,
		/*fillEmptyCells=*/fillEmptyCellsStrategy != ccRasterGrid::LEAVE_EMPTY,
		emptyCellsHeight,
		exportToOriginalCS);

	//success?
	if (cloudGrid)
	{
		//add the hillshade SF
		if (copyHillshadeSF)
		{
			int hillshadeSFIdx = m_rasterCloud->getScalarFieldIndexByName("Hillshade");
			if (hillshadeSFIdx >= 0)
			{
				CCCoreLib::ScalarField* hillshadeField = m_rasterCloud->getScalarField(hillshadeSFIdx);
				if (hillshadeField->currentSize() == cloudGrid->size())
				{
					try
					{
						ccScalarField* hillshadeClone = new ccScalarField(*static_cast<ccScalarField*>(hillshadeField));
						cloudGrid->addScalarField(hillshadeClone);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[Rasterize] Not enough memory to export the hillshade field");
					}
				}
			}
		}

		//currently displayed SF
		int activeSFIndex = cloudGrid->getScalarFieldIndexByName(qPrintable(activeSFName));
		cloudGrid->showSF(activeSFIndex >= 0);
		if (activeSFIndex < 0 && cloudGrid->getNumberOfScalarFields() != 0)
		{
			//if no SF is displayed, we should at least set a valid one (for later)
			activeSFIndex = static_cast<int>(cloudGrid->getNumberOfScalarFields()) - 1;
		}
		cloudGrid->setCurrentDisplayedScalarField(activeSFIndex);

		cloudGrid->showColors(interpolateColors);

		//don't forget the original shift
		cloudGrid->copyGlobalShiftAndScale(*m_cloud);
	}

	return cloudGrid;
}

void TrionContourDlg::updateGridAndDisplay()
{
	//remove the previous cloud
	if (m_rasterCloud)
	{
		if (m_glWindow)
		{
			m_glWindow->removeFromOwnDB(m_rasterCloud);
			m_glWindow->redraw();
		}

		delete m_rasterCloud;
		m_rasterCloud = nullptr;
	}

	bool activeLayerIsSF = false;
	bool activeLayerIsRGB = false;
	bool interpolateSF = false;
	bool success = updateGrid(false);

	if (success && m_glWindow)
	{
		//convert grid to point cloud
		std::vector<ccRasterGrid::ExportableFields> exportedFields;
		try
		{
			//we always compute the default 'height' layer
			exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
			//but we may also have to compute the 'original SF(s)' layer(s)
			QString activeLayerName = ccRasterGrid::GetDefaultFieldName(ccRasterGrid::PER_CELL_HEIGHT);
			m_rasterCloud = convertGridToCloud(exportedFields,
				/*interpolateSF=*/interpolateSF,
				/*interpolateColors=*/activeLayerIsRGB,
				/*copyHillshadeSF=*/false,
				activeLayerName,
				false);
		}
		catch (const std::bad_alloc&)
		{
			//see below
		}

		//if (m_rasterCloud)
		//{
		//	m_glWindow->addToOwnDB(m_rasterCloud);
		//	ccBBox box = m_rasterCloud->getDisplayBB_recursive(false, m_glWindow);
		//	//update2DDisplayZoom(box);
		//}
		//else
		//{
		//	ccLog::Error("Not enough memory!");
		//	m_glWindow->redraw();
		//}
	}

}

bool TrionContourDlg::getGridSize(unsigned& gridWidth, unsigned& gridHeight) const
{
	//vertical dimension
	const unsigned char Z = static_cast<unsigned char>(2);

	//cloud bounding-box --> grid size
	ccBBox box = m_cloud->getOwnBB();

	//grid step
	double gridStep = m_ui->doubleSpinBox_gridinterval->value();

	return ccRasterGrid::ComputeGridSize(Z, box, gridStep, gridWidth, gridHeight);
}

bool TrionContourDlg::updateGrid(bool interpolateSF/*=false*/)
{
	if (!m_cloud)
	{
		return false;
	}

	//main parameters
	ccRasterGrid::ProjectionType projectionType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	ccRasterGrid::ProjectionType interpolateSFs = ccRasterGrid::INVALID_PROJECTION_TYPE;
	bool interpolateEmptyCells = false;
	double maxEdgeLength = 0.0;

	//cloud bounding-box --> grid size
	ccBBox box = m_cloud->getOwnBB();
	if (!box.isValid())
	{
		return false;
	}


	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
	if (!getGridSize(gridWidth, gridHeight))
	{
		return false;
	}

	//grid size
	unsigned gridTotalSize = gridWidth * gridHeight;
	if (gridTotalSize == 1)
	{
		/*if (CS::Widgets::FramelessMessageBox::question(	this,
									"Unexpected grid size",
									"The generated grid will only have 1 cell! Do you want to proceed anyway?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::No)
			return false;*/
	}
	else if (gridTotalSize > 10000000)
	{
		//if (CS::Widgets::FramelessMessageBox::question(this,
		//	QApplication::translate("ccVolumeCalcTool", "Big grid size",nullptr),
		//	QApplication::translate("ccVolumeCalcTool", "The generated grid will have more than 10.000.000 cells.You cannot be calculated anyway.",nullptr),
		//	QMessageBox::Close,
		//	QMessageBox::No) == QMessageBox::Close)
		//	return false;
	}

	removeContourLines();

	//grid step
	double gridStep = m_ui->doubleSpinBox_gridinterval->value();
	assert(gridStep != 0);

	//memory allocation
	CCVector3d minCorner = box.minCorner();
	if (!m_grid.init(gridWidth, gridHeight, gridStep, minCorner))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}

	//vertical dimension
	const unsigned char Z = static_cast<unsigned char>(2);

	ccProgressDialog pDlg(true, this);
	if (!m_grid.fillWith(m_cloud,
		Z,
		projectionType,
		interpolateEmptyCells,
		maxEdgeLength,
		interpolateSFs,
		nullptr))
	{
		return false;
	}

	//update volume estimate
	{
		double hSum = 0;
		unsigned filledCellCount = 0;
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[j];
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				if (std::isfinite(row[i].h))
				{
					hSum += row[i].h;
					++filledCellCount;
				}
			}
		}

		if (filledCellCount)
		{
			double cellArea = m_grid.gridStep * m_grid.gridStep;
			//m_UI->volumeLabel->setText(QString::number(hSum * cellArea));
			//m_UI->filledCellsPercentageLabel->setText(QString::number(static_cast<double>(100 * filledCellCount) / (m_grid.width * m_grid.height), 'f', 2) + " %");
		}

	}

	ccLog::Print(QString("[Rasterize] Current raster grid:\n\tSize: %1 x %2\n\tHeight values: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));

	return true;
}

void TrionContourDlg::update2DDisplayZoom(ccBBox& box)
{
	if (!m_glWindow || !m_grid.isValid())
		return;

	//equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	//we set the pivot point on the box center
	CCVector3 P = box.getCenter();
	m_glWindow->setPivotPoint(P);
	m_glWindow->setCameraPos(P);

	//we compute the pixel size (in world coordinates)
	{
		double realGridWidth = m_grid.width  * m_grid.gridStep;
		double realGridHeight = m_grid.height * m_grid.gridStep;

		static const int screnMargin = 20;
		int screenWidth = std::max(1, m_glWindow->glWidth() - 2 * screnMargin);
		int screenHeight = std::max(1, m_glWindow->glHeight() - 2 * screnMargin);

		int pointSize = 1;
		if (static_cast<int>(m_grid.width) < screenWidth
			&&	static_cast<int>(m_grid.height) < screenHeight)
		{
			int vPointSize = static_cast<int>(ceil(static_cast<float>(screenWidth) / m_grid.width));
			int hPointSize = static_cast<int>(ceil(static_cast<float>(screenHeight) / m_grid.height));
			pointSize = std::min(vPointSize, hPointSize);

			//if the grid is too small (i.e. necessary point size > 10)
			if (pointSize > 10)
			{
				pointSize = 10;
				screenWidth = m_grid.width  * pointSize;
				screenHeight = m_grid.height * pointSize;
			}
		}

		double targetWidth = realGridWidth;
		if (realGridHeight / screenHeight > realGridWidth / screenWidth)
		{
			targetWidth = (realGridHeight * screenWidth) / screenHeight;
		}

		m_glWindow->setCameraFocalToFitWidth(static_cast<float>(targetWidth));
		//m_glWindow->setPointSize(pointSize);
	}

	m_glWindow->invalidateViewport();
	m_glWindow->invalidateVisualization();
	m_glWindow->deprecate3DLayer();
	m_glWindow->redraw();
}


void TrionContourDlg::generateContours()
{
	updateGridAndDisplay();
	if (!m_grid.isValid() || !m_rasterCloud)
	{
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Warning, tr("Error"), tr("Failed to generate contours. Ensure that parameters are properly set and try again."), QMessageBox::Ok | QMessageBox::Cancel, this);
        if (message_box.exec() == QMessageBox::Cancel)
        {
            close();
        }
        return;
	}

	//initialize parameters
	ccContourLinesGenerator::Parameters params;
	{
		params.projectContourOnAltitudes = false;

		//use current layer for 'altitudes'
		params.altitudes = m_rasterCloud->getCurrentDisplayedScalarField();
		if (!params.altitudes)
		{
            CS::Widgets::FramelessMessageBox message_box(QMessageBox::Warning, tr("Error"), tr("Failed to generate contours. Ensure that parameters are properly set and try again."), QMessageBox::Ok | QMessageBox::Cancel, this);
            if (message_box.exec() == QMessageBox::Cancel)
            {
                close();
            }
			return;
		}
		params.emptyCellsValue = params.altitudes->getMin() - 1.0;

		//min and max 'altitudes'
		params.startAltitude = m_ui->doubleSpinBox_start->value();
		params.maxAltitude = m_ui->doubleSpinBox_stop->value();
		assert(params.startAltitude <= params.maxAltitude);

		//gap between levels
		params.step = m_ui->doubleSpinBox_interval->value();
		assert(params.step > 0);

		//minimum number of vertices per contour line
		params.minVertexCount = m_ui->doubleSpinBox_minvertices->value();
		assert(params.minVertexCount >= 3);

		//the parameters below are only required if GDAL is not supported (but we can set them anyway)
		params.ignoreBorders = true;
		params.parentWidget = this;
	}

	removeContourLines();

	//compute the grid min corner (2D)
	CCVector2d gridMinCorner;
	{
		const unsigned char Z = static_cast<unsigned char>(2);
		const unsigned char X = Z == 2 ? 0 : Z + 1;
		const unsigned char Y = X == 2 ? 0 : X + 1;
		gridMinCorner = CCVector2d(m_grid.minCorner.u[X], m_grid.minCorner.u[Y]);
	}

	//generate the contour lines
	std::vector<ccPolyline*> contourLinesVec;
	if (!ccContourLinesGenerator::GenerateContourLines(&m_grid,
		gridMinCorner,
		params,
		contourLinesVec))
	{
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Warning, tr("Error"), tr("Failed to generate contours. Ensure that parameters are properly set and try again."), QMessageBox::Ok | QMessageBox::Cancel, this);
        if (message_box.exec() == QMessageBox::Cancel)
        {
            close();
        }
		return;
	}
	std::vector<ccPolyline*> contourLines;
	for (ccPolyline* poly : contourLinesVec)
	{
		if (poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble() <= params.maxAltitude && poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble() >= params.startAltitude)
		{
			contourLines.push_back(poly);
		}
		else
		{
			delete poly;
			poly = nullptr;
		}
	}

    std::set<double> heightset;
    for (ccPolyline* poly : contourLines)
    {
        heightset.insert(poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble());
    }
    std::map<double, int> heighttoindexmap;
    int startindex = 1;
    std::set<double> drawnameheightset;
    for (auto curheight : heightset)
    {
        heighttoindexmap[curheight] = startindex;
        if (startindex % 5 == 0)
        {
            drawnameheightset.insert(curheight);
        }
        startindex++;
    }
    for (auto heightval : drawnameheightset)
    {
        double maxpoint = 0;
        ccPolyline* maxpointpoly = nullptr;
        for (ccPolyline* poly : contourLines)
        {
            if (poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble() == heightval)
            {
                if (poly->getLineLenght() > maxpoint)
                {
                    maxpointpoly = poly;
                    maxpoint = poly->getLineLenght();
                }
            }
        }
        if (maxpointpoly && maxpointpoly->size()> 1)
        {
            int polysize = maxpointpoly->size();
            CCVector3 firstPos;
            CCVector3 secondpos;
            int minyIndex = 0;
            maxpointpoly->getPoint(0, firstPos);
           // maxpointpoly->getAssociatedCloud()->getPoint(1, secondpos);
            for (int i = 1;i < polysize;i++)
            {
                CCVector3 curpos;
                maxpointpoly->getPoint(i, curpos);
                if (curpos.y < firstPos.y)
                {
                    firstPos = curpos;
                    minyIndex = i;
                }
            }
            if (minyIndex == 0)
            {
                maxpointpoly->getPoint(1, secondpos);
            }
            else if(minyIndex == (polysize - 1))
            {
                maxpointpoly->getPoint(polysize - 2, secondpos);
            }
            else
            {
                CCVector3 latterpos;
                maxpointpoly->getPoint(minyIndex - 1, secondpos);
                maxpointpoly->getPoint(minyIndex + 1, latterpos);
                if (latterpos.y < secondpos.y)
                {
                    secondpos = latterpos;
                }
            }
            maxpointpoly->setContourNameParam(maxpointpoly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble(), (firstPos + secondpos)/2.0, firstPos);
        }
    }

	for (ccPolyline* poly : contourLines)
	{
        if (poly->getAssociatedCloud())
        {
            ccGenericPointCloud* cloudpoint = dynamic_cast<ccGenericPointCloud*>(poly->getAssociatedCloud());
            if (cloudpoint)
            {
                cloudpoint->setVisible(false);
            }
        }
		addNewContour(poly,
			poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble(),
			poly->getMetaData(ccContourLinesGenerator::MetaKeySubIndex()).toUInt(), heighttoindexmap[poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toDouble()]);
	}

	if (!m_contourLines.empty())
	{
		m_ui->pushButton_save->setEnabled(true);
	}
    else
    {
        m_ui->pushButton_save->setEnabled(false);
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Critical, tr("Error"), tr("Failed to generate contours. Ensure that parameters are properly set and try again."), QMessageBox::Ok, this);
        message_box.exec();
    }
	if (m_rasterCloud)
	{
		//if (m_glWindow)
		//{
		//	m_glWindow->removeFromOwnDB(m_rasterCloud);
		//	m_glWindow->redraw();
		//}

		delete m_rasterCloud;
		m_rasterCloud = nullptr;
	}
	if (m_glWindow)
	{
		m_glWindow->redraw();
	}
    m_ui->pushButton_preview->setEnabled(false);
}

void TrionContourDlg::addNewContour(ccPolyline* poly, double height, unsigned subIndex,int index)
{
	assert(poly);
	if (poly->size() > 1)
	{
		//poly->setName(QString("Contour line value = %1 (#%2)").arg(height).arg(subIndex));
        poly->showVertices(true);
        poly->setName(QString("Contour line value = %1").arg(height));
		poly->setGlobalScale(m_cloud->getGlobalScale());
		poly->setGlobalShift(m_cloud->getGlobalShift());
        ccColor::Rgb tntermediatecontourcol(m_TntermediateContourColor.red(), m_TntermediateContourColor.green(), m_TntermediateContourColor.blue());
		poly->setColor(tntermediatecontourcol);
		//poly->setClosed(isClosed);
		if (m_ui->radioButton_fromcloud->isChecked())
		{
            poly->setWidth(2);
			ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
			if (activeLayer)
			{
				const ccColor::Rgb* col = activeLayer->getColor(height);
				if (col)
				{
					poly->setColor(*col);
				}
			}
		}
        else
        {
            poly->setWidth(m_ui->spinBox_intermediatecontour->value());
            //double layernum = (height - m_ui->doubleSpinBox_start->value()) / m_ui->doubleSpinBox_interval->value();
            if (index % 5 == 0)
            {
                ccColor::Rgb countingcurvecol(m_CountingCurveColor.red(), m_CountingCurveColor.green(), m_CountingCurveColor.blue());
                poly->setColor(countingcurvecol);
                poly->setWidth(m_ui->spinBox_countingcurve->value());
            }
        }
		poly->showColors(true);
		//vertices->setEnabled(false);

		if (m_glWindow)
			m_glWindow->addToOwnDB(poly);

		m_contourLines.push_back(poly);
	}
}

void TrionContourDlg::exportContourLines()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow || !m_cloud || m_contourLines.empty())
	{
		return;
	}
	//vertical dimension
	const unsigned char Z = static_cast<unsigned char>(2);
	assert(Z <= 2);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	ccHObject* group = new ccHObject(QString(m_cloud->getName())+".contour");
	for (auto poly : m_contourLines)
	{
		//now is the time to map the polyline coordinates to the right dimensions!
		ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(poly->getAssociatedCloud());
		assert(vertices);
		if (vertices && Z != 2)
		{
			for (unsigned j = 0; j < vertices->size(); ++j)
			{
				CCVector3* P = const_cast<CCVector3*>(vertices->getPoint(j));
				CCVector3 Q = *P;
				P->u[X] = Q.x;
				P->u[Y] = Q.y;
				P->u[Z] = Q.z;
			}
			vertices->invalidateBoundingBox();
			poly->invalidateBoundingBox();
		}

		poly->showColors(true);
		group->addChild(poly);
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(poly);
	}
	m_contourLines.resize(0);

    CS::ModelControl::CCHObjectControl controlName(m_cloud->getParent());
    QString name = controlName.createChildNodeName(m_cloud->getName() + ".contour");
    group->setName(name);
    m_cloud->getParent()->addChild(group);
	group->setDisplay_recursive(m_cloud->getDisplay());
    if (m_cloud)
    {
        m_cloud->setEnabled(false);
    }
	mainWindow->addToDB(group,false,false,true,false);
    mainWindow->db()->selectEntity(group);
    close();
}

void TrionContourDlg::removeContourLines()
{
	while (!m_contourLines.empty())
	{
		ccPolyline* poly = m_contourLines.back();
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(poly);
		delete poly;
		m_contourLines.pop_back();
	}
	if (m_glWindow)
		m_glWindow->redraw();
}

