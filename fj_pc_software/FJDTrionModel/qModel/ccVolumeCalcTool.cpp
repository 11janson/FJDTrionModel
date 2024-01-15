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

#include "ccVolumeCalcTool.h"
#include "ui_volumeCalcDlg.h"
#include<QDebug>
//Local
#include "ccPersistentSettings.h"
#include "mainwindow.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccCuboids.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QClipboard>
#include <QMessageBox>
#include <QSettings>
#include "cloudcompareutils/icore.h"
#include <QToolButton>
#include <QDesktopWidget>
#include <QAbstractItemView>
#include <framelessmessagebox.h>
#include "cswidgets/titlebar.h"
#include <QtConcurrent>
//System
#include <cassert>
#include "FJStyleManager.h"
#include "cloudcomparewidgets/metahubframelesswaitdialog.h"

#include "ccPlane.h"

#include <QAxObject>
#include <QAxWidget>
#include <QString>
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QDateTime>
#include "cswidgets_global.h"
#include "framelessfiledialog.h"
#include "ccPickingHub.h"
#include "ccDBRoot.h"

//armon.guo
//#include "modelcontrol/cchobjectcontrol.h"
//#include "modelcontrol/pointcloudcontrol.h"

using namespace CS;
using namespace CS::Widgets;
//using namespace CS::ModelControl;
ccVolumeCalcTool::ccVolumeCalcTool(ccPickingHub* pickingHub, ccGenericPointCloud* cloud1, ccGenericPointCloud* cloud2, QWidget* parent/*=nullptr*/)
    : QWidget(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
    , cc2Point5DimEditor()
    , m_cloud1(cloud1)
    , m_cloud2(cloud2)
    , m_ui(new Ui::VolumeCalcDialog)
    , m_pBoxContainer(nullptr)
    , m_pSurface(new ccSurface)
	, m_RectSurface(new ccShowRect)
	, m_pickingHub(pickingHub)
	, m_pickingPoints("aligned points")
{
	assert(m_pickingHub);
	//m_ui->setupUi(this->getContentHolder());
	m_ui->setupUi(this);
	ccGLWindow* win = MainWindow::TheInstance()->GetActiveGLWindow();
	if (win)
	{
		win->addToOwnDB(&m_pickingPoints);
	}
	m_pickingPoints.setEnabled(true);
	m_pickingPoints.setVisible(false);
    m_ui->groundEmptyValueDoubleSpinBox->setMaximum(99999999.9);
    m_ui->ceilEmptyValueDoubleSpinBox->setMaximum(99999999.9);

	//this->bottomWidget()->setVisible(false);
	m_ui->updatePushButton->setProperty("metahubokbutton", true);
	connect(m_ui->gridStepDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccVolumeCalcTool::updateGridInfo);
	connect(m_ui->gridStepDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccVolumeCalcTool::gridOptionChanged);
	connect(m_ui->groundEmptyValueDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccVolumeCalcTool::gridOptionChanged);
	connect(m_ui->ceilEmptyValueDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccVolumeCalcTool::gridOptionChanged);
    connect(m_ui->doubleSpinBox_2, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccVolumeCalcTool::slotEdgeLengthChanged);
    connect(m_ui->doubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccVolumeCalcTool::slotEdgeLengthChanged);
	connect(m_ui->projDimComboBox,					qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccVolumeCalcTool::projectionDirChanged);
	connect(m_ui->updatePushButton,					&QPushButton::clicked,                              this,	&ccVolumeCalcTool::updateGridAndDisplay);
	connect(m_ui->heightProjectionComboBox,			qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccVolumeCalcTool::gridOptionChanged);
	connect(m_ui->fillGroundEmptyCellsComboBox, qOverload<int>(&QComboBox::currentIndexChanged),        this, &ccVolumeCalcTool::groundFillEmptyCellStrategyChanged);
	connect(m_ui->fillCeilEmptyCellsComboBox, qOverload<int>(&QComboBox::currentIndexChanged),          this, &ccVolumeCalcTool::ceilFillEmptyCellStrategyChanged);	
	connect(m_ui->groundComboBox,					qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccVolumeCalcTool::groundSourceChanged);
	connect(m_ui->ceilComboBox,						qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccVolumeCalcTool::ceilSourceChanged);
	//connect(m_ui->clipboardPushButton,				&QPushButton::clicked,															this,	&ccVolumeCalcTool::exportToClipboard);
	//connect(m_ui->exportGridPushButton,				&QPushButton::clicked,															this,	&ccVolumeCalcTool::exportGridAsCloud);
	//connect(m_ui->precisionSpinBox,					qOverload<int>(&QSpinBox::valueChanged),					this,	&ccVolumeCalcTool::setDisplayedNumberPrecision);
	connect(m_ui->pushButton_create, &QPushButton::clicked, this, &ccVolumeCalcTool::oncreatebtn);
	connect(m_ui->pushButton_close, &QPushButton::clicked, this, &ccVolumeCalcTool::onclosebtn);
	connect(m_ui->toolButton_close, &QPushButton::clicked, this, &ccVolumeCalcTool::close);
    connect(m_ui->pushButton_export, &QPushButton::clicked, this, &ccVolumeCalcTool::slotExportWordFileToDesktop,Qt::UniqueConnection);
	connect(this, &ccVolumeCalcTool::signalExportFileExit, this, &ccVolumeCalcTool::slotExportFileExit, Qt::UniqueConnection);
    connect(m_ui->pushButton_reset, &QPushButton::clicked, this, &ccVolumeCalcTool::onresetbtn);
	QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
	m_ui->toolButton_close->setIcon(closeicon);
	if (m_cloud1)
	{
		m_pickingPoints.copyGlobalShiftAndScale(*m_cloud1);
	}
	if (m_cloud1 && !m_cloud2)
	{
		//the existing cloud is always the second by default
		std::swap(m_cloud1, m_cloud2);
	}
	assert(m_cloud2);

    ccGLMatrix trans;
    CCVector3 pos(9999, 0, 0);
    trans.setTranslation(pos);

	//custom bbox editor
	ccBBox gridBBox = m_cloud1 ? m_cloud1->getOwnBB() : ccBBox(); 
	if (m_cloud2)
	{
		gridBBox += m_cloud2->getOwnBB();
	}
	if (gridBBox.isValid())
	{
		createBoundingBoxEditor(gridBBox, this);
		//connect(m_ui->editGridToolButton, &QToolButton::clicked, this, &ccVolumeCalcTool::showGridBoxEditor);
	}
	else
	{
		//m_ui->editGridToolButton->setEnabled(false);
	}

	m_ui->groundComboBox->addItem(tr("Constant"));
    m_ui->groundComboBox->addItem(tr("Custom"));
	//m_ui->ceilComboBox->addItem("Constant");
	if (m_cloud1)
	{
		m_ui->groundComboBox->addItem(m_cloud1->getName());
		m_ui->ceilComboBox->addItem(m_cloud1->getName());
	}
	if (m_cloud2)
	{
		m_ui->groundComboBox->addItem(m_cloud2->getName());
		m_ui->ceilComboBox->addItem(m_cloud2->getName());
	}
	assert(m_ui->groundComboBox->count() >= 2);
    if (m_ui->groundComboBox->count() == 3)
    {
        m_ui->groundComboBox->setCurrentIndex(0);
    }
    else
    {
        m_ui->groundComboBox->setCurrentIndex(m_ui->groundComboBox->count() - 2);
    }

	m_ui->ceilComboBox->setCurrentIndex(m_ui->ceilComboBox->count()-1);
	adjustItemWidth(m_ui->groundComboBox);
	adjustItemWidth(m_ui->ceilComboBox);
	//add window
	//create2DView(m_ui->mapFrame);

    link3dMainViewer();

	if (m_glWindow)
	{
		/*ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		params.colorScaleShowHistogram = false;
		params.displayedNumPrecision = m_precision;
		m_glWindow->setDisplayParameters(params, true);*/

        m_glWindow->addToOwnDB(m_pSurface);
		m_glWindow->addToOwnDB(m_RectSurface);
	}

	
	loadSettings();

	updateGridInfo();

	gridIsUpToDate(false);
	setWindowTitle(QCoreApplication::translate("VolumeCalcDialog", "Volume Calculation", nullptr));	
	m_ui->label_title->setText(QCoreApplication::translate("VolumeCalcDialog", "Volume Calculation", nullptr));
	
    //更新平面位置
    connect(m_ui->groundComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccVolumeCalcTool::redrawConsultPlane);
    connect(m_ui->groundEmptyValueDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccVolumeCalcTool::redrawConsultPlane);
    connect(m_ui->ceilEmptyValueDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccVolumeCalcTool::redrawConsultPlane);
    connect(m_ui->ceilComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccVolumeCalcTool::redrawConsultPlane); 
    m_ui->projDimComboBox->setCurrentIndex(2);

	setFixedWidth(340);
	m_ui->pushButton_create->setEnabled(false);




    //[!]项目名称
    mlist.append("ProjectName");
    //[!]导出时间
    mlist.append("ExportTime");
    //[!]堆体长度
    mlist.append("Pilelength");
    //[!]堆体宽度
    mlist.append("Pilewidth");
    //[!]占地面积
    mlist.append("areacovered");
    //[!]正的体积
    mlist.append("Positivevolume");
    //[!]负的体积
    mlist.append("Negativevolume");
    //[!]挖方减去填方
    mlist.append("Cutminusfill");
    //[!]挖方加上填方
    mlist.append("Cutplusfill");
    //[!]正的平面面积
    mlist.append("Positiveplanearea");
    //[!]负的平面面积
    mlist.append("Negativeplanearea");
    //[!]图片
    mlist.append("ProjectPhoto");

	if (m_pickingHub)
	{
		m_pickingHub->addListener(this, true);
	}
}

void ccVolumeCalcTool::onresetbtn()
{
    if (m_pBoxContainer)
    {
        m_glWindow->removeFromOwnDB(m_pBoxContainer);
        delete m_pBoxContainer;
        m_pBoxContainer = nullptr;
    }
    gridIsUpToDate(false);
    MainWindow::TheInstance()->deselectAllOtherObject(true);
    MainWindow::TheInstance()->db()->updatePropertiesView();
    m_glWindow->redraw();
}

void ccVolumeCalcTool::adjustItemWidth(QComboBox * combox)
{
	QFontMetrics fm(combox->font());
	QRect rect;
	int max_len = 0;

	for (int i = 0; i < combox->count(); i++)
	{
		rect = fm.boundingRect(combox->itemText(i)); //获得字符串所占的像素大小
		if (max_len < rect.width())
		{
			max_len = rect.width();
		}
	}

	max_len *= 1.1;
	int w = qMax(max_len, combox->width());
	combox->view()->setFixedWidth(w);

}


ccVolumeCalcTool::~ccVolumeCalcTool()
{
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.resize(0);
	if (m_glWindow)
	{
        m_glWindow->removeFromOwnDB(&m_pickingPoints);
        m_glWindow->removeFromOwnDB(m_RectSurface);
	}
	m_pickingPoints.setDisplay(nullptr);
	delete m_ui;
	m_pickingHub->removeListener(this);
	m_pickingHub = nullptr;
    if (m_pBoxContainer)
    {
        delete m_pBoxContainer;
        m_pBoxContainer = nullptr;
    }

    if (m_pSurface)
    {
        delete m_pSurface;
        m_pSurface = nullptr;
    }

	if (m_RectSurface)
	{
		delete m_RectSurface;
		m_RectSurface = nullptr;
	}
}

void ccVolumeCalcTool::setDisplayedNumberPrecision(int precision)
{
	//update window
	if (m_glWindow)
	{
		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		params.displayedNumPrecision = precision;
		m_glWindow->setDisplayParameters(params, true);
		m_glWindow->redraw(true, false);
	}

	//update report
	//if (m_ui->clipboardPushButton->isEnabled())
	//{
	//	outputReport(m_lastReport);
	//}
}

void ccVolumeCalcTool::groundSourceChanged(int index)
{
	m_ui->fillGroundEmptyCellsComboBox->setEnabled(m_ui->groundComboBox->currentIndex() > 1);
	groundFillEmptyCellStrategyChanged(-1);
	int groundnum = m_ui->groundComboBox->count();
	int ceilnum = m_ui->ceilComboBox->count();
	int currentceilindex = m_ui->ceilComboBox->currentIndex();
    int currentgroundindex = m_ui->groundComboBox->currentIndex();
	if (index == 0 || index == 1)
	{
		if (groundnum == ceilnum)
		{
			m_ui->ceilComboBox->removeItem(0);
            m_ui->ceilComboBox->removeItem(0);
			if (currentceilindex == 0 || currentceilindex == 1)
			{
				m_ui->ceilComboBox->setCurrentIndex(0);
			}
			else
			{
				m_ui->ceilComboBox->setCurrentIndex(currentceilindex-2);
			}
			
		}
	}
	else
	{
		if (groundnum != ceilnum)
		{
            m_ui->ceilComboBox->insertItem(0, tr("Custom"));
			m_ui->ceilComboBox->insertItem(0, tr("Constant"));
			m_ui->ceilComboBox->setCurrentIndex(currentceilindex + 2);
		}
	}
	adjustItemWidth(m_ui->groundComboBox);
	adjustItemWidth(m_ui->ceilComboBox);
    if (m_ui->groundComboBox->currentText() == tr("Custom") || m_ui->ceilComboBox->currentText() == tr("Custom"))
    {
        m_ui->projDimComboBox->setCurrentIndex(2);
        m_ui->projDimComboBox->setEnabled(false);
        MainWindow::TheInstance()->updateStatusbarInformation(tr("Please pick three points on the point cloud to define the projection plane."));
    }
    else
    {
        m_ui->projDimComboBox->setEnabled(true);
        MainWindow::TheInstance()->updateStatusbarInformation("");
    }
    clearCustomParam();
    gridIsUpToDate(false);
}

void ccVolumeCalcTool::ceilSourceChanged(int)
{
	int groundnum = m_ui->groundComboBox->count();
	int ceilnum = m_ui->ceilComboBox->count();
	if (groundnum == ceilnum)
	{
		m_ui->fillCeilEmptyCellsComboBox->setEnabled(m_ui->ceilComboBox->currentIndex() > 1);
	}
	else
	{
		m_ui->fillCeilEmptyCellsComboBox->setEnabled(true);
	}
	
	ceilFillEmptyCellStrategyChanged(-1);
    if (m_ui->groundComboBox->currentText() == tr("Custom") || m_ui->ceilComboBox->currentText() == tr("Custom"))
    {
        m_ui->projDimComboBox->setCurrentIndex(2);
        m_ui->projDimComboBox->setEnabled(false);
        MainWindow::TheInstance()->updateStatusbarInformation(tr("Please pick three points on the point cloud to define the projection plane."));
    }
    else
    {
        m_ui->projDimComboBox->setEnabled(true);
        MainWindow::TheInstance()->updateStatusbarInformation("");
    }
    clearCustomParam();
    gridIsUpToDate(false);
}


bool ccVolumeCalcTool::showGridBoxEditor()
{
	if (cc2Point5DimEditor::showGridBoxEditor())
	{
		updateGridInfo();
		return true;
	}

	return false;
}

void ccVolumeCalcTool::updateGridInfo()
{
	m_ui->gridWidthLabel->setText(getGridSizeAsString());
}

double ccVolumeCalcTool::getGridStep() const
{
	return m_ui->gridStepDoubleSpinBox->value();
}

unsigned char ccVolumeCalcTool::getProjectionDimension() const
{
	int dim = m_ui->projDimComboBox->currentIndex();
	//assert(dim >= 0 && dim < 3);
	return static_cast<unsigned char>(dim);
}

void ccVolumeCalcTool::sfProjectionTypeChanged(int index)
{
	Q_UNUSED( index )
	
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::projectionDirChanged(int dir)
{
	Q_UNUSED( dir )
		
	clearCustomParam();
	updateGridInfo();
	gridIsUpToDate(false);
	// 自定义高度默认为包围盒对应轴的最小值
	ccBBox box = m_cloud2->getBB_recursive();
	CCVector3d offset = m_cloud2->getGlobalShift();
	CCVector3d origin_minconer = box.minCorner().toDouble() - offset;
	if (m_ui->projDimComboBox->currentText().compare("z", Qt::CaseInsensitive) == 0)
	{
		m_ui->groundEmptyValueDoubleSpinBox->setValue(origin_minconer.z);
		m_ui->ceilEmptyValueDoubleSpinBox->setValue(origin_minconer.z);
	}
	else if (m_ui->projDimComboBox->currentText().compare("x", Qt::CaseInsensitive) == 0)
	{
		m_ui->groundEmptyValueDoubleSpinBox->setValue(origin_minconer.x);
		m_ui->ceilEmptyValueDoubleSpinBox->setValue(origin_minconer.x);
	}
	else if (m_ui->projDimComboBox->currentText().compare("y", Qt::CaseInsensitive) == 0)
	{
		m_ui->groundEmptyValueDoubleSpinBox->setValue(origin_minconer.y);
		m_ui->ceilEmptyValueDoubleSpinBox->setValue(origin_minconer.y);
	}

	redrawConsultPlane();
}

void ccVolumeCalcTool::groundFillEmptyCellStrategyChanged(int)
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_ui->fillGroundEmptyCellsComboBox);

	m_ui->groundEmptyValueDoubleSpinBox->setEnabled( (m_ui->groundComboBox->currentIndex() <= 1)
													 || (fillEmptyCellsStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT) );

    m_ui->doubleSpinBox_2->setEnabled((fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE));
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::ceilFillEmptyCellStrategyChanged(int)
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_ui->fillCeilEmptyCellsComboBox);
	int groundnum = m_ui->groundComboBox->count();
	int ceilnum = m_ui->ceilComboBox->count();
	m_ui->ceilEmptyValueDoubleSpinBox->setEnabled( (m_ui->ceilComboBox->currentIndex() <= 1) && (groundnum == ceilnum));
    m_ui->doubleSpinBox->setEnabled((fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE));
	if (fillEmptyCellsStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT)
	{
		m_ui->ceilEmptyValueDoubleSpinBox->setEnabled(true);
	}
	//m_ui->ceilMaxEdgeLengthDoubleSpinBox->setEnabled(fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE);

	gridIsUpToDate(false);
}

void ccVolumeCalcTool::gridOptionChanged()
{
	gridIsUpToDate(false);
}

ccRasterGrid::ProjectionType ccVolumeCalcTool::getTypeOfProjection() const
{
	switch (m_ui->heightProjectionComboBox->currentIndex())
	{
	case 0:
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	case 1:
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	case 2:
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::INVALID_PROJECTION_TYPE;
}

void ccVolumeCalcTool::loadSettings()
{
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::VolumeCalculation());
	int projType				= settings.value("ProjectionType", m_ui->heightProjectionComboBox->currentIndex()).toInt();
	int projDim					= settings.value("ProjectionDim", m_ui->projDimComboBox->currentIndex()).toInt();
	int groundFillStrategy = settings.value("gFillStrategy", m_ui->fillGroundEmptyCellsComboBox->currentIndex()).toInt();
	int ceilFillStrategy = settings.value("cFillStrategy", m_ui->fillCeilEmptyCellsComboBox->currentIndex()).toInt();
	double step					= settings.value("GridStep", m_ui->gridStepDoubleSpinBox->value()).toDouble();
	double groundEmptyHeight	= settings.value("gEmptyCellsHeight", m_ui->groundEmptyValueDoubleSpinBox->value()).toDouble();
	m_groundMaxEdgeLength = settings.value("gMaxEdgeLength", 0).toDouble();
	double ceilEmptyHeight		= settings.value("cEmptyCellsHeight", m_ui->ceilEmptyValueDoubleSpinBox->value()).toDouble();
	m_ceilMaxEdgeLength = settings.value("cMaxEdgeLength", 0).toDouble();
	m_precision				= settings.value("NumPrecision", 3).toInt();
	settings.endGroup();

	m_ui->gridStepDoubleSpinBox->setValue(step);
	m_ui->heightProjectionComboBox->setCurrentIndex(projType);
	m_ui->fillGroundEmptyCellsComboBox->setCurrentIndex(groundFillStrategy);
	m_ui->fillCeilEmptyCellsComboBox->setCurrentIndex(ceilFillStrategy);
	m_ui->groundEmptyValueDoubleSpinBox->setValue(groundEmptyHeight);
	m_ui->ceilEmptyValueDoubleSpinBox->setValue(ceilEmptyHeight);

	m_ui->projDimComboBox->setCurrentIndex(projDim);
}

void ccVolumeCalcTool::slotUIButtonOk(void)
{
	saveSettings();
	//accept();
}

void ccVolumeCalcTool::saveSettingsAndAccept()
{
	saveSettings();
	//accept();
}

void ccVolumeCalcTool::saveSettings()
{
	QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
	settings.beginGroup(ccPS::VolumeCalculation());
	settings.setValue("ProjectionType", m_ui->heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim", m_ui->projDimComboBox->currentIndex());
	settings.setValue("gFillStrategy", m_ui->fillGroundEmptyCellsComboBox->currentIndex());
	settings.setValue("cFillStrategy", m_ui->fillCeilEmptyCellsComboBox->currentIndex());
	settings.setValue("GridStep", m_ui->gridStepDoubleSpinBox->value());
	settings.setValue("gEmptyCellsHeight", m_ui->groundEmptyValueDoubleSpinBox->value());
	settings.setValue("gMaxEdgeLength", m_groundMaxEdgeLength);
	settings.setValue("cEmptyCellsHeight", m_ui->ceilEmptyValueDoubleSpinBox->value());
	settings.setValue("cMaxEdgeLength", m_ceilMaxEdgeLength);
	settings.endGroup();
}

void ccVolumeCalcTool::gridIsUpToDate(bool state)
{
    if ((m_ui->groundComboBox->currentText() == tr("Custom") || m_ui->ceilComboBox->currentText() == tr("Custom")) && m_pickingPoints.size() != 3)
    {
        state = true;
    }
	if (state)
	{
		//standard button
		m_ui->updatePushButton->setStyleSheet("color: black; background-color:#6B5A1F;");
	}
	else
	{
		//red button
		m_ui->updatePushButton->setStyleSheet("color: black; background-color:#FFC804;");
	}
	m_ui->updatePushButton->setDisabled(state);
	m_ui->pushButton_create->setEnabled(m_pBoxContainer ? true : false);
}

ccPointCloud* ccVolumeCalcTool::ConvertGridToCloud(	ccRasterGrid& grid,
													const ccBBox& gridBox,
													unsigned char vertDim,
													bool exportToOriginalCS)
{
	assert(gridBox.isValid());
	assert(vertDim < 3);

	ccPointCloud* rasterCloud = nullptr;
	try
	{
		//we only compute the default 'height' layer
		std::vector<ccRasterGrid::ExportableFields> exportedFields;
		exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);

		rasterCloud = grid.convertToCloud(exportedFields,
			false,
			false,
			false,
			false,
			nullptr,
			vertDim,
			gridBox,
			false,
			std::numeric_limits<double>::quiet_NaN(),
			exportToOriginalCS);

		if (rasterCloud && rasterCloud->hasScalarFields())
		{
			rasterCloud->showSF(true);
			rasterCloud->setCurrentDisplayedScalarField(0);
			ccScalarField* sf = static_cast<ccScalarField*>(rasterCloud->getScalarField(0));
			assert(sf);
			QString m_str = QApplication::translate("ccVolumeCalcTool", "Relative height", nullptr);
			std::string str = m_str.toStdString();
			const char* ch = str.c_str();
			sf->setName(ch);
			sf->setSymmetricalScale(sf->getMin() < 0 && sf->getMax() > 0);
			rasterCloud->showSFColorsScale(true);
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ConvertGridToCloud] Not enough memory!");
		if (rasterCloud)
		{
			delete rasterCloud;
			rasterCloud = nullptr;
		}
	}

	return rasterCloud;
}

ccPointCloud* ccVolumeCalcTool::convertGridToCloud(bool exportToOriginalCS) const
{
ccPointCloud* rasterCloud = nullptr;
try
{
    //we only compute the default 'height' layer
    std::vector<ccRasterGrid::ExportableFields> exportedFields;
    exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
    rasterCloud = cc2Point5DimEditor::convertGridToCloud(exportedFields,
        false,
        false,
        false,
        false,
        nullptr,
        false,
        std::numeric_limits<double>::quiet_NaN(),
        exportToOriginalCS);

    if (rasterCloud)
    {
        if (rasterCloud->hasScalarFields())
        {
            rasterCloud->showSF(true);
            rasterCloud->setCurrentDisplayedScalarField(0);
            ccScalarField* sf = static_cast<ccScalarField*>(rasterCloud->getScalarField(0));
            assert(sf);
            QString m_str = QApplication::translate("ccVolumeCalcTool", "Relative height", nullptr);
            std::string str = m_str.toStdString();
            const char* ch = str.c_str();
            sf->setName(ch);
            sf->setSymmetricalScale(sf->getMin() < 0 && sf->getMax() > 0);
            rasterCloud->showSFColorsScale(true);
        }

        //keep Global Shift & Scale
        auto ground = getGroundCloud();
        auto ceil = getCeilCloud();

        if (ground.first && ground.first->isShifted())
        {
            rasterCloud->copyGlobalShiftAndScale(*ground.first);
        }
        else if (ceil.first && ceil.first->isShifted())
        {
            rasterCloud->copyGlobalShiftAndScale(*ceil.first);
        }
    }
}
catch (const std::bad_alloc&)
{
    ccLog::Error("Not enough memory!");
    if (rasterCloud)
    {
        delete rasterCloud;
        rasterCloud = nullptr;
    }
}

return rasterCloud;
}

ccCuboids* ccVolumeCalcTool::convertGridToBoxes() const
{
    ccCuboids* boxesContainer = nullptr;

    try
    {
        //we only compute the default 'height' layer
        std::vector<ccRasterGrid::ExportableFields> exportedFields;
        exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
        boxesContainer = new ccCuboids(m_grid);
        //if (m_cloud2)
        //{
        //    boxesContainer->copyGlobalShiftAndScale(*m_cloud2);
        //    if (boxesContainer->getAssociatedCloud())
        //    {
        //        boxesContainer->getAssociatedCloud()->copyGlobalShiftAndScale(*m_cloud2);
        //    }
        //}
        const unsigned char Z = getProjectionDimension();
        assert(Z <= 2);

        boxesContainer->convertGrid2Cuboids(exportedFields, Z, false);


    }
    catch (const std::bad_alloc&)
    {
        ccLog::Error("Not enough memory!");
        if (boxesContainer)
        {
            delete boxesContainer;
            boxesContainer = nullptr;
        }
    }

    return boxesContainer;
}

void ccVolumeCalcTool::updateGridAndDisplay()
{
    auto ground = getGroundCloud();
    auto ceil = getCeilCloud();
    ccBBox box = getCustomBBox();
    CCVector3f normal;
    CCVector3f z_aix = { 0,0,1 };
    //若选择任意面投影，则根据选定三个点将点云旋转至与z轴垂直平面
    if (m_ui->projDimComboBox->currentIndex() == 0)
    {
        if (ground.first == nullptr)
        {
            ground.second = ground.second + m_cloud2->getGlobalShift().x;
        }
        else
            ceil.second = ceil.second + m_cloud2->getGlobalShift().x;
    }
    if (m_ui->projDimComboBox->currentIndex() == 1)
    {
        if (ground.first == nullptr)
        {
            ground.second = ground.second + m_cloud2->getGlobalShift().y;
        }
        else
            ceil.second = ceil.second + m_cloud2->getGlobalShift().y;
    }
    if (m_ui->projDimComboBox->currentIndex() == 2)
    {
        if (ground.first == nullptr)
        {
            ground.second = ground.second + m_cloud2->getGlobalShift().z;
        }
        else
            ceil.second = ceil.second + m_cloud2->getGlobalShift().z;
    }
 //   if(m_ui->projDimComboBox->currentIndex()==3)
 //   {
	//	//CCVector3d v1 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(0));
	//	//CCVector3d v2 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(1));
	//	//CCVector3d v3 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(2));
 //       CCVector3d v1 = *m_pickingPoints.getPoint(0);
 //       CCVector3d v2 = *m_pickingPoints.getPoint(1);
 //       CCVector3d v3 = *m_pickingPoints.getPoint(2);
 //       //CCVector3f v1(-40.503, -15.279, 0.196);
	//	//CCVector3f v2(-23.919, -25.370, -0.415);
	//	//CCVector3f v3(-27.181, -28.838, -1.000);

	//	float na = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
	//	float nb = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
	//	float nc = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);

	//	normal = { na / sqrt(na*na + nb * nb + nc * nc), nb / sqrt(na*na + nb * nb + nc * nc), nc / sqrt(na*na + nb * nb + nc * nc) };
	//	v1 += normal * m_ui->groundEmptyValueDoubleSpinBox->value();
	//	v2 += normal * m_ui->groundEmptyValueDoubleSpinBox->value();
	//	v3 += normal * m_ui->groundEmptyValueDoubleSpinBox->value();
 //       if (normal.z < 0)
 //       {
 //           normal = normal *(-1);
 //       }
	//	ccGLMatrix  rotate_matrix = ccGLMatrix::FromToRotation(normal, z_aix);
 //       if (!(nullptr == ceil.first) && !(nullptr == ground.first))
 //       {
 //           ceil.first->setGLTransformation(rotate_matrix);
 //           ceil.first->applyGLTransformation_recursive();
 //           ground.first->setGLTransformation(rotate_matrix);
 //           ground.first->applyGLTransformation_recursive();
 //       }
	//	else if (!(nullptr == ceil.first))
	//	{
	//	    ceil.first->setGLTransformation(rotate_matrix);
	//	    ceil.first->applyGLTransformation_recursive();
	//	    box = ceil.first->getBB_recursive();
	//	    rotate_matrix.applyRotation(v1);
	//	    ground.second = v1.z;
	//	}
	//	else
	//	{
	//	    ground.first->setGLTransformation(rotate_matrix);
	//	    ground.first->applyGLTransformation_recursive();
	//	    box = ground.first->getBB_recursive();
	//	    rotate_matrix.applyRotation(v1);
	//	    ceil.second = v1.z;
	//	}
	//}

    double customHeight = 0.0;
    bool success;
    if (m_ui->groundComboBox->currentText() == tr("Custom") && ceil.first)
    {
        customHeight = ground.second;

        CCVector3d v1 = *m_pickingPoints.getPoint(0);
        CCVector3d v2 = *m_pickingPoints.getPoint(1);
        CCVector3d v3 = *m_pickingPoints.getPoint(2);

        v1.z += customHeight ;
        v2.z += customHeight ;
        v3.z += customHeight ;

        //CCVector3f v1(-39.169, -13.103, -0.118);
        //CCVector3f v2(-34.937, -20.019, 1.455);
        //CCVector3f v3(-33.889, -23.850, 0.266);
        float A = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
        float B = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
        float C = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);
        float D = -(A*v1[0] + B * v1[1] + C * v1[2]);
        std::vector<float> Para;
        Para.push_back(A);
        Para.push_back(B);
        Para.push_back(C);
        Para.push_back(D);
        success = updateGrid2(Para, ceil, box);
    }
    else if (m_ui->ceilComboBox->currentText() == tr("Custom") && ground.first)
    {
        customHeight = ceil.second;

        CCVector3d v1 = *m_pickingPoints.getPoint(0);
        CCVector3d v2 = *m_pickingPoints.getPoint(1);
        CCVector3d v3 = *m_pickingPoints.getPoint(2);

        v1.z += customHeight;
        v2.z += customHeight;
        v3.z += customHeight;

        //CCVector3f v1(-39.169, -13.103, -0.118);
        //CCVector3f v2(-34.937, -20.019, 1.455);
        //CCVector3f v3(-33.889, -23.850, 0.266);
        float A = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
        float B = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
        float C = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);
        float D = -(A*v1[0] + B * v1[1] + C * v1[2]);
        std::vector<float> Para;
        Para.push_back(A);
        Para.push_back(B);
        Para.push_back(C);
        Para.push_back(D);
        success = updateGrid2(Para, ground, box);
    }
    else
    {
        success = updateGrid(ground, ceil, box);
    }
	if (success && m_glWindow)
	{

        if (m_pBoxContainer)
        {
            m_glWindow->removeFromOwnDB(m_pBoxContainer);
            delete m_pBoxContainer;
            m_pBoxContainer = nullptr;
        }

        //[!].显示等待框
        WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(MainWindow::TheInstance());
        WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QApplication::translate("ccVolumeCalcTool","Calculating the volume... Please wait.",nullptr));
        //[!].并发执行线程中执行
        QtConcurrent::run([=]() mutable {

            QObject end;
            connect(&end, &QObject::destroyed, this, [=]() {

                WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
                });

                m_pBoxContainer = convertGridToBoxes();
                if (m_pBoxContainer)
                {
                    ccPointCloud* rasterCloud = dynamic_cast<ccPointCloud*>(m_pBoxContainer->getAssociatedCloud());
                    if (rasterCloud)
                    {

                        if (rasterCloud->hasScalarFields())
                        {
                            //rasterCloud->showColors(false);
                            //rasterCloud->showSF(true);
                            rasterCloud->setCurrentDisplayedScalarField(0);
                            ccScalarField* sf = static_cast<ccScalarField*>(rasterCloud->getScalarField(0));
                            assert(sf);
                            sf->computeMinAndMax();
                            QString m_str = QApplication::translate("ccVolumeCalcTool", "Relative height", nullptr);
                            std::string str = m_str.toStdString();
                            const char* ch = str.c_str();
                            sf->setName(ch);

                            sf->setSymmetricalScale(sf->getMin() < 0 && sf->getMax() > 0);
                            rasterCloud->showSFColorsScale(true);
                        }

						//[!].设置结果大坐标shit值
						ccGenericPointCloud *pCloud = m_cloud1 ? m_cloud1 : m_cloud2;
						if (pCloud){
							rasterCloud->copyGlobalShiftAndScale(*pCloud);
						}
                        rasterCloud->setVisible(false);
						
                    }
                    m_pBoxContainer->showColors(false);
                    m_pBoxContainer->showSF(true);
                    m_glWindow->addToOwnDB(m_pBoxContainer);
                }
                else
                {
                    ccLog::Error("Not enough memory!");
                    //m_glWindow->redraw();
                }


                m_glWindow->redraw();

            });
	}
    else if(m_ui->projDimComboBox->currentIndex() == 3)
    {
        ccGLMatrix  rotate_matrix2 = ccGLMatrix::FromToRotation(z_aix, normal);

        if (!(nullptr == ceil.first) && !(nullptr == ground.first))
        {
            ceil.first->setGLTransformation(rotate_matrix2);
            ceil.first->applyGLTransformation_recursive();
            ground.first->setGLTransformation(rotate_matrix2);
            ground.first->applyGLTransformation_recursive();
        }
        else if (!(nullptr == ceil.first))
        {
            ceil.first->setGLTransformation(rotate_matrix2);
            ceil.first->applyGLTransformation_recursive();
        }
        else
        {
            ground.first->setGLTransformation(rotate_matrix2);
            ground.first->applyGLTransformation_recursive();
        }
    }
	gridIsUpToDate(success);
	m_ui->pushButton_create->setEnabled(success);
    m_ui->pushButton_export->setEnabled(success);
}

void ccVolumeCalcTool::oncreatebtn()
{
    
	exportGridAsCloud();
	m_ui->pushButton_create->setEnabled(false);
    MainWindow::TheInstance()->setGlobalZoom();
}

void ccVolumeCalcTool::onclosebtn()
{
	close();
}

QString ccVolumeCalcTool::ReportInfo::toText(int precision) const
{
	QStringList m_volumelist = (QStringList() << u8"m³" << u8"dm³" << u8"cm³" << u8"mm³" << u8"ft³" << u8"in³");
	QStringList m_arealist = (QStringList() << u8"m²" << u8"dm²" << u8"cm²" << u8"mm²" << u8"ft²" << u8"in²");
	QLocale locale(QLocale::English);
	ccGui::ParamStruct m_parameters = ccGui::Parameters();
	


	QString m_pStrCube , m_pStrSquare ;
	double m_pVolume = 0, m_pSurfaced = 0, m_pAddedVolume = 0 ,m_pRemovedVolume = 0;
	switch (m_parameters.volumnunit)
	{
	case 0:
		m_pStrCube = u8"m³";
		m_pVolume = volume;
		m_pAddedVolume = addedVolume;
		m_pRemovedVolume = removedVolume;
		break;
	case 1:
		m_pStrCube = u8"dm³";
		m_pVolume = volume * 1000;
		m_pAddedVolume = addedVolume * 1000;
		m_pRemovedVolume = removedVolume * 1000;
		break;
	case 2:
		m_pStrCube = u8"cm³";
		m_pVolume = volume * 1000 * 1000;
		m_pAddedVolume = addedVolume * 1000 * 1000;
		m_pRemovedVolume = removedVolume * 1000 * 1000;
		break;
	case 3:
		m_pStrCube = u8"mm³";
		m_pVolume = volume * 1000 * 1000 *1000;
		m_pAddedVolume = addedVolume * 1000 * 1000 * 1000;
		m_pRemovedVolume = removedVolume * 1000 * 1000 * 1000;
		break;
	case 4:
		m_pStrCube = u8"ft³";
		m_pVolume = volume * 35.3147248277;
		m_pAddedVolume = addedVolume * 35.3147248277;
		m_pRemovedVolume = removedVolume * 35.3147248277;
		break;
	case 5:
		m_pStrCube = u8"in³";
		m_pVolume = volume * 61023.8445022;
		m_pAddedVolume = addedVolume * 61023.8445022;
		m_pRemovedVolume = removedVolume * 61023.8445022;
		break;
	default:
		break;
	}
	switch (m_parameters.areaunit)
	{
	case 0:
		m_pStrSquare = u8"m²";
		m_pSurfaced = surface;
		break;
	case 1:
		m_pStrSquare = u8"dm²";
		m_pSurfaced = surface * 100;
		break;
	case 2:
		m_pStrSquare = u8"cm²";
		m_pSurfaced = surface * 100 * 100;
		break;
	case 3:
		m_pStrSquare = u8"mm²";
		m_pSurfaced = surface * 100 * 100 * 100;
		break;
	case 4:
		m_pStrSquare = u8"ft²";
		m_pSurfaced = surface * 10.7639104;
		break;
	case 5:
		m_pStrSquare = u8"in²";
		m_pSurfaced = surface * 1550.0031;
		break;
	default:
		break;
	}
	QStringList reportText;
	if (m_parameters.isshowUnitlabel)
	{
		
		reportText << QString(tr("Volume")) + QString(":%1").arg(locale.toString(m_pVolume, 'f', 3)) + QString(m_pStrCube);
		reportText << QString(tr("Floor Area")) + QString(":%1").arg(locale.toString(m_pSurfaced, 'f',3)) + QString(m_pStrSquare);
		reportText << QString(tr("Added volume:(+)%1")).arg(locale.toString(m_pAddedVolume, 'f', 3)) + QString(m_pStrCube);
		reportText << QString(tr("Removed volume:(-)%1")).arg(locale.toString(m_pRemovedVolume, 'f',3)) + QString(m_pStrCube);
	}
	else
	{
		reportText << QString(tr("Volume")) + QString(":%1").arg(locale.toString(m_pVolume, 'f', 3)) ;
		reportText << QString(tr("Floor Area")) + QString(":%1").arg(locale.toString(m_pSurfaced, 'f', 3));
		reportText << QString(tr("Added volume:(+)%1")).arg(locale.toString(m_pAddedVolume, 'f', 3));
		reportText << QString(tr("Removed volume:(-)%1")).arg(locale.toString(m_pRemovedVolume, 'f',3));
	}
	return reportText.join("\n");
}

void ccVolumeCalcTool::outputReport(const ReportInfo& info)
{
	int precision = m_precision;

	m_ui->reportPlainTextEdit->setPlainText(info.toText(precision));
	m_ui->reportPlainTextEdit->setStyleSheet("font-size : 14px;border:1px solid #989898;");
	//below 7 neighbors per cell, at least one of the cloud is very sparse!
	//m_ui->spareseWarningLabel->setVisible(info.averageNeighborsPerCell < 7.0f);
	
	m_lastReport = info;
	//m_ui->clipboardPushButton->setEnabled(true);
}

bool SendError(const QString& message, QWidget* parentWidget)
{
	if (parentWidget)
	{
		ccLog::Error(message);
	}
	else
	{
		ccLog::Warning("[Volume] " + message);
	}
	return false;
}

bool ccVolumeCalcTool::ComputeVolume(	ccRasterGrid& grid,
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
										double groundHeight = std::numeric_limits<double>::quiet_NaN(),
										double ceilHeight = std::numeric_limits<double>::quiet_NaN(),
										QWidget* parentWidget/*=nullptr*/)
{
	if (	gridStep <= 1.0e-8
		||	gridWidth == 0
		||	gridHeight == 0
		||	vertDim > 2)
	{
		assert(false);
		ccLog::Warning("[Volume] Invalid input parameters");
		return false;
	}

	if (!ground && !ceil)
	{
		assert(false);
		ccLog::Warning("[Volume] No valid input cloud");
		return false;
	}

	if (!gridBox.isValid())
	{
		ccLog::Warning("[Volume] Invalid bounding-box");
		return false;
	}

	//grid size
	unsigned gridTotalSize = gridWidth * gridHeight;
	if (gridTotalSize == 1)
	{
		/*if (parentWidget && CS::Widgets::FramelessMessageBox::question(parentWidget, "Unexpected grid size", "The generated grid will only have 1 cell! Do you want to proceed anyway?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return false;*/
	}
	else if (gridTotalSize > 10000000)
	{
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question,
            QApplication::translate("ccVolumeCalcTool", "Big grid size",nullptr),QApplication::translate("ccVolumeCalcTool", "The generated grid will have more than 10.000.000 cells.You cannot be calculated anyway.",nullptr),
            QMessageBox::Close,
            parentWidget);
        message_box.changeButtonText(QDialogButtonBox::StandardButton::Close, tr("Quit"));
        message_box.exec();
        return false;
	}

	//memory allocation
	CCVector3d minCorner = gridBox.minCorner();
	if (!grid.init(gridWidth, gridHeight, gridStep, minCorner))
	{
		//not enough memory
		return SendError("Not enough memory", parentWidget);
	}

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	
	if (parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parentWidget));
		pDlg->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	}

	ccRasterGrid groundRaster;
	if (ground)
	{
		if (!groundRaster.init(gridWidth, gridHeight, gridStep, minCorner))
		{
			//not enough memory
			return SendError("Not enough memory", parentWidget);
		}

		if (groundRaster.fillWith(	ground,
									vertDim,
									projectionType,
									groundEmptyCellFillStrategy == ccRasterGrid::INTERPOLATE,
									groundMaxEdgeLength,
									ccRasterGrid::INVALID_PROJECTION_TYPE,
									pDlg.data()))
		{
			groundRaster.fillEmptyCells(groundEmptyCellFillStrategy, groundHeight);
			ccLog::Print(QString("[Volume] Ground raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(groundRaster.width).arg(groundRaster.height).arg(groundRaster.minHeight).arg(groundRaster.maxHeight));
		}
		else
		{
			return false;
		}
	}

	//ceil
	ccRasterGrid ceilRaster;
	if (ceil)
	{
		if (!ceilRaster.init(gridWidth, gridHeight, gridStep, minCorner))
		{
			//not enough memory
			return SendError("Not enough memory", parentWidget);
		}

		if (ceilRaster.fillWith(ceil,
								vertDim,
								projectionType,
								ceilEmptyCellFillStrategy == ccRasterGrid::INTERPOLATE,
								ceilMaxEdgeLength,
								ccRasterGrid::INVALID_PROJECTION_TYPE,
								pDlg.data()))
		{
			ceilRaster.fillEmptyCells(ceilEmptyCellFillStrategy, ceilHeight);
			ccLog::Print(QString("[Volume] Ceil raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(ceilRaster.width).arg(ceilRaster.height).arg(ceilRaster.minHeight).arg(ceilRaster.maxHeight));
		}
		else
		{
			return false;
		}
	}

	//update grid and compute volume
	{
		//if (pDlg)
		//{
		//	/*pDlg->setMethodTitle(QObject::tr("Volume computation"));
		//	pDlg->setInfo(QObject::tr("Cells: %1 x %2").arg(grid.width).arg(grid.height));*/
		//	pDlg->start();
		//	pDlg->show();
		//	QCoreApplication::processEvents();
		//}
		//CCCoreLib::NormalizedProgress nProgress(pDlg.data(), grid.width * grid.height);
		
		size_t ceilNonMatchingCount = 0;
		size_t groundNonMatchingCount = 0;
		size_t cellCount = 0;

		//at least one of the grid is based on a cloud
		grid.nonEmptyCellCount = 0;
		for (unsigned i = 0; i < grid.height; ++i)
		{
			for (unsigned j = 0; j < grid.width; ++j)
			{
				ccRasterCell& cell = grid.rows[i][j];

				bool validGround = true;
				cell.minHeight = groundHeight;
				if (ground)
				{
					cell.minHeight = groundRaster.rows[i][j].h;
					validGround = std::isfinite(cell.minHeight);
				}

				bool validCeil = true;
				cell.maxHeight = ceilHeight;
				if (ceil)
				{
					cell.maxHeight = ceilRaster.rows[i][j].h;
					validCeil = std::isfinite(cell.maxHeight);
				}

				if (validGround && validCeil)
				{
					cell.h = cell.maxHeight - cell.minHeight;
					cell.nbPoints = 1;

					reportInfo.volume += cell.h;
					if (cell.h < 0)
					{
						reportInfo.removedVolume -= cell.h;
                        reportInfo.removedSurface += 1;
                        reportInfo.surface += 1.0;

					}
					else if (cell.h >= 0)
					{
						reportInfo.addedVolume += cell.h;
                        reportInfo.addedSurface += 1;
                        reportInfo.surface += 1.0;
					}
					++grid.nonEmptyCellCount; // matching count
					++cellCount;
				}
				else
				{
					if (validGround)
					{
						++cellCount;
						++groundNonMatchingCount;
					}
					else if (validCeil)
					{
						++cellCount;
						++ceilNonMatchingCount;
					}
					cell.h = std::numeric_limits<double>::quiet_NaN();
					cell.nbPoints = 0;
				}

				cell.avgHeight = (groundHeight + ceilHeight) / 2;
				cell.stdDevHeight = 0;

				//if (pDlg && !nProgress.oneStep())
				//{
				//	ccLog::Warning("[Volume] Process cancelled by the user");
				//	return false;
				//}
			}
		}
		grid.validCellCount = grid.nonEmptyCellCount;

		//count the average number of valid neighbors
		{
			size_t validNeighborsCount = 0;
			size_t count = 0;
			for (unsigned i = 1; i < grid.height - 1; ++i)
			{
				for (unsigned j = 1; j < grid.width - 1; ++j)
				{
					ccRasterCell& cell = grid.rows[i][j];
					if (cell.h == cell.h)
					{
						for (unsigned k = i - 1; k <= i + 1; ++k)
						{
							for (unsigned l = j - 1; l <= j + 1; ++l)
							{
								if (k != i || l != j)
								{
									ccRasterCell& otherCell = grid.rows[k][l];
									if (std::isfinite(otherCell.h))
									{
										++validNeighborsCount;
									}
								}
							}
						}

						++count;
					}
				}
			}

			if (count)
			{
				reportInfo.averageNeighborsPerCell = static_cast<double>(validNeighborsCount) / count;
			}
		}

		reportInfo.matchingPrecent = static_cast<float>(grid.validCellCount * 100) / cellCount;
		reportInfo.groundNonMatchingPercent = static_cast<float>(groundNonMatchingCount * 100) / cellCount;
		reportInfo.ceilNonMatchingPercent = static_cast<float>(ceilNonMatchingCount * 100) / cellCount;
		float cellArea = static_cast<float>(grid.gridStep * grid.gridStep);
		reportInfo.volume *= cellArea;
		reportInfo.addedVolume *= cellArea;
		reportInfo.removedVolume *= cellArea;
		reportInfo.surface *= cellArea;
        reportInfo.addedSurface *= cellArea;
        reportInfo.removedSurface *= cellArea;
	}

	grid.setValid(true);

	return true;
}

bool ccVolumeCalcTool::ComputeVolume2(ccRasterGrid& grid,
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
    QWidget* parentWidget)
{
    //grid size
    unsigned gridTotalSize = gridWidth * gridHeight;
    if (gridTotalSize == 1)
    {
        /*if (parentWidget && CS::Widgets::FramelessMessageBox::question(parentWidget, "Unexpected grid size", "The generated grid will only have 1 cell! Do you want to proceed anyway?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
            return false;*/
    }
    else if (gridTotalSize > 10000000)
    {
        CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question,
            QApplication::translate("ccVolumeCalcTool", "Big grid size",nullptr), QApplication::translate("ccVolumeCalcTool", "The generated grid will have more than 10.000.000 cells.You cannot be calculated anyway.",nullptr),
            QMessageBox::Close,
            parentWidget);
        message_box.changeButtonText(QDialogButtonBox::StandardButton::Close, tr("Quit"));
        message_box.exec();
        return false;
    }
    //memory allocation
    CCVector3d minCorner = gridBox.minCorner();
    if (!grid.init(gridWidth, gridHeight, gridStep, minCorner))
    {
        //not enough memory
        return SendError("Not enough memory", parentWidget);
    }

    //progress dialog
    QScopedPointer<ccProgressDialog> pDlg(nullptr);

    if (parentWidget)
    {
        pDlg.reset(new ccProgressDialog(true, parentWidget));
        pDlg->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    }

    //ceil
    ccRasterGrid ceilRaster;
    if (ceil)
    {
        if (!ceilRaster.init(gridWidth, gridHeight, gridStep, minCorner))
        {
            //not enough memory
            return SendError("Not enough memory", parentWidget);
        }

        if (ceilRaster.fillWith(ceil,
            vertDim,
            projectionType,
            ceilEmptyCellFillStrategy == ccRasterGrid::INTERPOLATE,
            ceilMaxEdgeLength,
            ccRasterGrid::INVALID_PROJECTION_TYPE,
            pDlg.data()))
        {
            ceilRaster.fillEmptyCells(ceilEmptyCellFillStrategy, ceilHeight);
            ccLog::Print(QString("[Volume] Ceil raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(ceilRaster.width).arg(ceilRaster.height).arg(ceilRaster.minHeight).arg(ceilRaster.maxHeight));
        }
        else
        {
            return false;
        }
    }

    //update grid and compute volume
    {
        //if (pDlg)
        //{
        //	/*pDlg->setMethodTitle(QObject::tr("Volume computation"));
        //	pDlg->setInfo(QObject::tr("Cells: %1 x %2").arg(grid.width).arg(grid.height));*/
        //	pDlg->start();
        //	pDlg->show();
        //	QCoreApplication::processEvents();
        //}
        //CCCoreLib::NormalizedProgress nProgress(pDlg.data(), grid.width * grid.height);

        size_t ceilNonMatchingCount = 0;
        size_t groundNonMatchingCount = 0;
        size_t cellCount = 0;

        //at least one of the grid is based on a cloud
        grid.nonEmptyCellCount = 0;
        for (unsigned i = 0; i < grid.height; ++i)
        {
            for (unsigned j = 0; j < grid.width; ++j)
            {
                ccRasterCell& cell = grid.rows[i][j];
                if (vertDim == 0)
                {
                    cell.minHeight = ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->y*(-PlanePara[1] / PlanePara[0])
                        - ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->z*(PlanePara[2] / PlanePara[0]) - (PlanePara[3] / PlanePara[0]);
                }
                if (vertDim == 1)
                {
                    cell.minHeight = ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->x*(-PlanePara[0] / PlanePara[1])
                        - ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->z*(PlanePara[2] / PlanePara[1]) - (PlanePara[3] / PlanePara[1]);
                }
                if (vertDim == 2)
                {
                    cell.minHeight = ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->x*(-PlanePara[0] / PlanePara[2])
                        - ceil->getPoint(ceilRaster.rows[i][j].pointIndex)->y*(PlanePara[1] / PlanePara[2]) - (PlanePara[3] / PlanePara[2]);
                }

                bool validCeil = true;
                cell.maxHeight = ceilHeight;
                if (ceil)
                {
                    cell.maxHeight = ceilRaster.rows[i][j].h;
                    validCeil = std::isfinite(cell.maxHeight);
                }

                if (validCeil)
                {
                    cell.h = cell.maxHeight - cell.minHeight;
                    cell.nbPoints = 1;

                    reportInfo.volume += cell.h;
                    if (cell.h < 0)
                    {
                        reportInfo.removedVolume -= cell.h;
                        reportInfo.removedSurface += 1;
                        reportInfo.surface += 1.0;
                    }
                    else if (cell.h >= 0)
                    {
                        reportInfo.addedVolume += cell.h;
                        reportInfo.addedSurface += 1;
                        reportInfo.surface += 1.0;
                    }
                    ++grid.nonEmptyCellCount; // matching count
                    ++cellCount;
                }
                else
                {
                    cell.h = std::numeric_limits<double>::quiet_NaN();
                    cell.nbPoints = 0;
                }

                cell.avgHeight =  ceilHeight;
                cell.stdDevHeight = 0;

                //if (pDlg && !nProgress.oneStep())
                //{
                //	ccLog::Warning("[Volume] Process cancelled by the user");
                //	return false;
                //}
            }
        }
        grid.validCellCount = grid.nonEmptyCellCount;

        //count the average number of valid neighbors
        {
            size_t validNeighborsCount = 0;
            size_t count = 0;
            for (unsigned i = 1; i < grid.height - 1; ++i)
            {
                for (unsigned j = 1; j < grid.width - 1; ++j)
                {
                    ccRasterCell& cell = grid.rows[i][j];
                    if (cell.h == cell.h)
                    {
                        for (unsigned k = i - 1; k <= i + 1; ++k)
                        {
                            for (unsigned l = j - 1; l <= j + 1; ++l)
                            {
                                if (k != i || l != j)
                                {
                                    ccRasterCell& otherCell = grid.rows[k][l];
                                    if (std::isfinite(otherCell.h))
                                    {
                                        ++validNeighborsCount;
                                    }
                                }
                            }
                        }

                        ++count;
                    }
                }
            }

            if (count)
            {
                reportInfo.averageNeighborsPerCell = static_cast<double>(validNeighborsCount) / count;
            }
        }

        reportInfo.matchingPrecent = static_cast<float>(grid.validCellCount * 100) / cellCount;
        reportInfo.groundNonMatchingPercent = static_cast<float>(groundNonMatchingCount * 100) / cellCount;
        reportInfo.ceilNonMatchingPercent = static_cast<float>(ceilNonMatchingCount * 100) / cellCount;
        float cellArea = static_cast<float>(grid.gridStep * grid.gridStep);
        reportInfo.volume *= cellArea;
        reportInfo.addedVolume *= cellArea;
        reportInfo.removedVolume *= cellArea;
        reportInfo.surface *= cellArea;
        reportInfo.addedSurface *= cellArea;
        reportInfo.removedSurface *= cellArea;
    }

    grid.setValid(true);

    return true;
}

std::pair<ccGenericPointCloud*, double> ccVolumeCalcTool::getGroundCloud() const
{
	ccGenericPointCloud* groundCloud = nullptr;
	double groundHeight = std::numeric_limits<double>::quiet_NaN();
	switch (m_ui->groundComboBox->currentIndex())
	{
	case 0:
		groundHeight = m_ui->groundEmptyValueDoubleSpinBox->value();
		break;
    case 1:
        groundHeight = m_ui->groundEmptyValueDoubleSpinBox->value();
        break;
	case 2:
		groundCloud = m_cloud1 ? m_cloud1 : m_cloud2;
		break;
	case 3:
		groundCloud = m_cloud2;
		break;
	default:
		assert(false);
		break;
	}

	return { groundCloud, groundHeight };
}

std::pair<ccGenericPointCloud*, double> ccVolumeCalcTool::getCeilCloud() const
{
	ccGenericPointCloud* ceilCloud = nullptr;
	double ceilHeight = std::numeric_limits<double>::quiet_NaN();
	int groundnum = m_ui->groundComboBox->count();
	int ceilnum = m_ui->ceilComboBox->count();
	if (groundnum == ceilnum)
	{
		switch (m_ui->ceilComboBox->currentIndex())
		{
		case 0:
			ceilHeight = m_ui->ceilEmptyValueDoubleSpinBox->value();
			break;
        case 1:
            ceilHeight = m_ui->ceilEmptyValueDoubleSpinBox->value();
            break;
		case 2:
			ceilCloud = m_cloud1 ? m_cloud1 : m_cloud2;
			break;
		case 3:
			ceilCloud = m_cloud2;
			break;
		default:
			assert(false);
			break;
		}
	}
	else
	{
		switch (m_ui->ceilComboBox->currentIndex())
		{
		case 0:
			ceilCloud = m_cloud1 ? m_cloud1 : m_cloud2;
			break;
		case 1:
			ceilCloud = m_cloud2;
			break;
		default:
			assert(false);
			break;
		}
	}


	return { ceilCloud, ceilHeight };
}

bool ccVolumeCalcTool::updateGrid(/*ccPoint& point1, ccPoint& point2, ccPoint& point3*/std::pair<ccGenericPointCloud*, double> ground, std::pair<ccGenericPointCloud*, double> ceil,ccBBox box)
{
	if (!m_cloud2)
	{
		assert(false);
		return false;
	}

	//cloud bounding-box --> grid size
	//ccBBox box = getCustomBBox();
	//if (!box.isValid())
	//{
	//	return false;
	//}

	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
	if (!getGridSize(gridWidth, gridHeight))
	{
		return false;
	}

	//grid step
	double gridStep = getGridStep();
	assert(gridStep != 0);

	//ground
	//auto ground = getGroundCloud();
	//if (nullptr == ground.first && std::isnan(ground.second))
	//{
	//	assert(false);
	//	return false;
	//}

	//ceil
	//auto ceil = getCeilCloud();
	//if (nullptr == ceil.first && std::isnan(ceil.second))
	//{
	//	assert(false);
	//	return false;
	//}

	ccVolumeCalcTool::ReportInfo reportInfo;
    
	if (ComputeVolume(	m_grid,
						ground.first,
						ceil.first,
						box,
						getProjectionDimension(),
						gridStep,
						gridWidth,
						gridHeight,
						getTypeOfProjection(),
						getFillEmptyCellsStrategy(m_ui->fillGroundEmptyCellsComboBox),
						m_groundMaxEdgeLength,
						getFillEmptyCellsStrategy(m_ui->fillCeilEmptyCellsComboBox),
						m_ceilMaxEdgeLength,
						reportInfo,
						ground.second,
						ceil.second,
						this))
	{	
		outputReport(reportInfo);
       

        QString Positivevolume = QString::number(reportInfo.addedVolume, 'f', 3);
        QString Negativevolume = QString::number(reportInfo.removedVolume, 'f', 3);
        QString areacovered = QString::number(reportInfo.surface, 'f', 3);
        QString Cutminusfill = QString::number(reportInfo.addedVolume - reportInfo.removedVolume, 'f', 3);
        QString Cutplusfill = QString::number(reportInfo.addedVolume + reportInfo.removedVolume, 'f', 3);
        QString Positiveplanearea = QString::number(reportInfo.addedSurface, 'f', 3);
        QString Negativeplanearea = QString::number(reportInfo.removedSurface, 'f', 3);


        ccHObject::Container obj = MainWindow::TheInstance()->getSelectedEntities();
        if (obj.size() < 1)
        {
            return false;
        }
        ccBBox gridBBox = obj.at(0)->getOwnBB();
        double width = gridBBox.maxCorner().y - gridBBox.minCorner().y;
        double length = gridBBox.maxCorner().x - gridBBox.minCorner().x;
        QString Pilelength = QString::number(width, 'f', 3);
        QString Pilewidth = QString::number(length, 'f', 3);


       
        //[!]堆体长度
        map.insert("Pilelength", Pilelength);
        //[!]堆体宽度
        map.insert("Pilewidth", Pilewidth);
        //[!]正的体积
        map.insert("Positivevolume", Positivevolume);
        //[!]负的体积
        map.insert("Negativevolume", Negativevolume);
        //[!]挖方减去填方
        map.insert("Cutminusfill", Cutminusfill);
        //[!]挖方加上填方
        map.insert("Cutplusfill", Cutplusfill);
        //[!]占地面积
        map.insert("areacovered", areacovered);
        //[!]负的平面面积
        map.insert("Positiveplanearea", Positiveplanearea);
        //[!]正的平面面积
        map.insert("Negativeplanearea", Negativeplanearea);
        
		return true;
	}
	else
	{
		return false;
	}
}

bool ccVolumeCalcTool::updateGrid2(std::vector<float> Para, std::pair<ccGenericPointCloud*, double> ceil, ccBBox box)
{
    if (!m_cloud2)
    {
        assert(false);
        return false;
    }

    //cloud bounding-box --> grid size
    //ccBBox box = getCustomBBox();
    //if (!box.isValid())
    //{
    //	return false;
    //}

    unsigned gridWidth = 0;
    unsigned gridHeight = 0;
    if (!getGridSize(gridWidth, gridHeight))
    {
        return false;
    }

    //grid step
    double gridStep = getGridStep();
    assert(gridStep != 0);

    //ground
    //auto ground = getGroundCloud();
    //if (nullptr == ground.first && std::isnan(ground.second))
    //{
    //	assert(false);
    //	return false;
    //}

    //ceil
    //auto ceil = getCeilCloud();
    //if (nullptr == ceil.first && std::isnan(ceil.second))
    //{
    //	assert(false);
    //	return false;
    //}

    ccVolumeCalcTool::ReportInfo reportInfo;

    if (ComputeVolume2(m_grid,
        Para,
        ceil.first,
        box,
        getProjectionDimension(),
        gridStep,
        gridWidth,
        gridHeight,
        getTypeOfProjection(),
        getFillEmptyCellsStrategy(m_ui->fillCeilEmptyCellsComboBox),
        m_ceilMaxEdgeLength,
        reportInfo,
        ceil.second,
        this))
    {
        outputReport(reportInfo);


        QString Positivevolume = QString::number(reportInfo.addedVolume, 'f', 3);
        QString Negativevolume = QString::number(reportInfo.removedVolume, 'f', 3);
        QString areacovered = QString::number(reportInfo.surface, 'f', 3);
        QString Cutminusfill = QString::number(reportInfo.addedVolume - reportInfo.removedVolume, 'f', 3);
        QString Cutplusfill = QString::number(reportInfo.addedVolume + reportInfo.removedVolume, 'f', 3);
        QString Positiveplanearea = QString::number(reportInfo.addedSurface, 'f', 3);
        QString Negativeplanearea = QString::number(reportInfo.removedSurface, 'f', 3);


        ccHObject::Container obj = MainWindow::TheInstance()->getSelectedEntities();
        if (obj.size() < 1)
        {
            return false;
        }
        ccBBox gridBBox = obj.at(0)->getOwnBB();
        double width = gridBBox.maxCorner().y - gridBBox.minCorner().y;
        double length = gridBBox.maxCorner().x - gridBBox.minCorner().x;
        QString Pilelength = QString::number(width, 'f', 3);
        QString Pilewidth = QString::number(length, 'f', 3);



        //[!]堆体长度
        map.insert("Pilelength", Pilelength);
        //[!]堆体宽度
        map.insert("Pilewidth", Pilewidth);
        //[!]正的体积
        map.insert("Positivevolume", Positivevolume);
        //[!]负的体积
        map.insert("Negativevolume", Negativevolume);
        //[!]挖方减去填方
        map.insert("Cutminusfill", Cutminusfill);
        //[!]挖方加上填方
        map.insert("Cutplusfill", Cutplusfill);
        //[!]占地面积
        map.insert("areacovered", areacovered);
        //[!]负的平面面积
        map.insert("Positiveplanearea", Positiveplanearea);
        //[!]正的平面面积
        map.insert("Negativeplanearea", Negativeplanearea);

        return true;
    }
    else
    {
        return false;
    }
}

void ccVolumeCalcTool::exportToClipboard() const
{
	QClipboard* clipboard = QApplication::clipboard();
	if (clipboard)
	{
		clipboard->setText(m_ui->reportPlainTextEdit->toPlainText());
	}
}

void ccVolumeCalcTool::exportGridAsCloud()
{
    if (!m_pBoxContainer)
        return;

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow && m_glWindow)
	{
        m_glWindow->removeFromOwnDB(m_pBoxContainer);

        ccHObject* parent = nullptr;
        if (m_cloud1)
            parent = m_cloud1->getParent();
        else if (m_cloud2)
            parent = m_cloud2->getParent();

        if (parent)
            parent->addChild(m_pBoxContainer);


		mainWindow->addToDB(m_pBoxContainer, true, false);
		//多次创建名字应该不一样，carl
		int index = 0;
		if (m_pBoxContainer->getParent())
		{
			for (int i = 0; i < m_pBoxContainer->getParent()->getChildrenNumber();i++)
			{
				ccCuboids* cubo = dynamic_cast<ccCuboids*>(m_pBoxContainer->getParent()->getChild(i));
				if (cubo)
				{
					index++;
				}
			}
		}
		QString cuboName = m_pBoxContainer->getName();
		m_pBoxContainer->setName(cuboName + "_" + QString::number(index));
        ccHObject * child = new ccHObject;
        child->setName("Volumn:"  + QString::number(m_lastReport.volume, 'f', 3));
        m_pBoxContainer->addChild(child);
		ccLog::Print(QString("[Volume] Cloud '%1' successfully exported").arg(m_pBoxContainer->getName()));

        m_pBoxContainer = nullptr;
	}

}


int ccVolumeCalcTool::link3dMainViewer()
{
    MainWindow* mainWindow = MainWindow::TheInstance();
    if (mainWindow)
    {
        ccGLWindow* main3dViewer = mainWindow->getGLWindow(0);

        if (main3dViewer)
            m_glWindow = main3dViewer;
        else
            m_glWindow = nullptr;

        return 1;
    }
    else
        return 0;

    //	ccPointCloud* curcloud = convertGridToCloud(true);
    //	mainWindow->addToDB(curcloud);
    //	ccBBox box = curcloud->getDisplayBB_recursive(false, mainWindow->getActiveGLWindow());
    //	CCVector3 P = box.getCenter();
    //	mainWindow->getActiveGLWindow()->setPivotPoint(P);
    //	mainWindow->getActiveGLWindow()->setCameraPos(P);

    //	//we compute the pixel size (in world coordinates)
    //	{
    //		double realGridWidth = m_grid.width  * m_grid.gridStep;
    //		double realGridHeight = m_grid.height * m_grid.gridStep;

    //		static const int screnMargin = 20;
    //		int screenWidth = std::max(1, mainWindow->getActiveGLWindow()->glWidth() - 2 * screnMargin);
    //		int screenHeight = std::max(1, mainWindow->getActiveGLWindow()->glHeight() - 2 * screnMargin);

    //		int pointSize = 1;
    //		if (static_cast<int>(m_grid.width) < screenWidth
    //			&&	static_cast<int>(m_grid.height) < screenHeight)
    //		{
    //			int vPointSize = static_cast<int>(ceil(static_cast<float>(screenWidth) / m_grid.width));
    //			int hPointSize = static_cast<int>(ceil(static_cast<float>(screenHeight) / m_grid.height));
    //			pointSize = std::min(vPointSize, hPointSize);

    //			//if the grid is too small (i.e. necessary point size > 10)
    //			if (pointSize > 10)
    //			{
    //				pointSize = 10;
    //				screenWidth = m_grid.width  * pointSize;
    //				screenHeight = m_grid.height * pointSize;
    //			}
    //		}

    //		double targetWidth = realGridWidth;
    //		//if (realGridHeight / screenHeight > realGridWidth / screenWidth)
    //		//{
    //		//	targetWidth = (realGridHeight * screenWidth) / screenHeight;
    //		//}

    //		mainWindow->getActiveGLWindow()->setCameraFocalToFitWidth(static_cast<float>(targetWidth));
    //		//mainWindow->getActiveGLWindow()->setPointSize(pointSize);
    //	}

    //	mainWindow->getActiveGLWindow()->invalidateViewport();
    //	mainWindow->getActiveGLWindow()->invalidateVisualization();
    //	mainWindow->getActiveGLWindow()->deprecate3DLayer();
    //	mainWindow->getActiveGLWindow()->redraw();
    //}
    //else
    //{
    //	assert(false);
    //	delete m_rasterCloud;
    //}
}

void ccVolumeCalcTool::redrawConsultPlane()
{
    if (!m_pSurface || !m_glWindow || !m_RectSurface)
        return;

    /*if(m_pSurface)
        m_pSurface->setCalcBBox(ccBBox(CCVector3(0, 0, 0), CCVector3(100, 100, 100)));*/
	if (m_ui->projDimComboBox->currentIndex() != 3)
	{
		ccBBox box;
		CCVector3d offset;
		double height = 0;
		if (m_ui->groundComboBox->count() == 3 && m_ui->groundComboBox->currentIndex() == 0 &&
			(m_ui->ceilComboBox->count() == 1 || m_ui->ceilComboBox->currentIndex() == 2))
			//if (m_ui->groundComboBox->currentText() == "Constant" && m_ui->ceilComboBox->currentText() != "Constant")
		{
			if (m_cloud2)
			{
				offset = m_cloud2->getGlobalShift();
				box = m_cloud2->getBB_recursive();
				height = m_ui->groundEmptyValueDoubleSpinBox->value();
				m_pSurface->setSurfaceVisible(true);
			}
			else
				return;
		}
		else if (m_ui->ceilComboBox->count() == 3 && m_ui->ceilComboBox->currentIndex() == 0 &&
			(m_ui->groundComboBox->count() == 1 || m_ui->groundComboBox->currentIndex() == 2))
			//else if (m_ui->groundComboBox->currentText() != "Constant" && m_ui->ceilComboBox->currentText() == "Constant")
		{
			if (m_cloud2)
			{
				offset = m_cloud2->getGlobalShift();
				box = m_cloud2->getBB_recursive();
				height = m_ui->ceilEmptyValueDoubleSpinBox->value();
				m_pSurface->setSurfaceVisible(true);
			}
			else
				return;
		}
		else
		{
			m_pSurface->setSurfaceVisible(false);
		}


		//box外扩10%
		int dim = m_ui->projDimComboBox->currentIndex();
		ccSurface::EPivotType type;
		double min = 0;
		if (dim == 0)
		{
			type = ccSurface::EPivotType::XAXIS;
			//min = height - box.getCenter().x;
			min = height + offset.x - box.getCenter().x;
		}
		else if (dim == 1)
		{
			type = ccSurface::EPivotType::YAXIS;
			//min = height - box.getCenter().y;
			min = height + offset.y - box.getCenter().y;
		}
		else if (dim == 2)
		{
			type = ccSurface::EPivotType::ZAXIS;
			//min = height - box.getCenter().z;
			min = height + offset.z - box.getCenter().z;
		}


		m_pSurface->setCalcBBox(box);
		m_pSurface->updateSliceDirection(type);
		m_pSurface->updateHeight(0);//平面厚度
		m_pSurface->setScaling(1.1);

		m_pSurface->updatePos(min);
		m_pSurface->setOpenZFighting(true);
		m_RectSurface->setVisible(false);
		m_pSurface->setVisible(true);

        if (m_ui->groundComboBox->currentText() == tr("Custom") || m_ui->ceilComboBox->currentText() == tr("Custom"))
        {
            m_pSurface->setVisible(false);
            if (m_pickingPoints.size() == 3)
            {
                m_RectSurface->setVisible(true);
                //CCVector3d v1 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(0));
                //CCVector3d v2 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(1));
                //CCVector3d v3 = m_pickingPoints.toGlobal3d<PointCoordinateType>(*m_pickingPoints.getPoint(2));
                CCVector3d v1 = *m_pickingPoints.getPoint(0);
                CCVector3d v2 = *m_pickingPoints.getPoint(1);
                CCVector3d v3 = *m_pickingPoints.getPoint(2);
                float na = (v2[1] - v1[1])*(v3[2] - v1[2]) - (v2[2] - v1[2])*(v3[1] - v1[1]);
                float nb = (v2[2] - v1[2])*(v3[0] - v1[0]) - (v2[0] - v1[0])*(v3[2] - v1[2]);
                float nc = (v2[0] - v1[0])*(v3[1] - v1[1]) - (v2[1] - v1[1])*(v3[0] - v1[0]);

                CCVector3f normal = { na / sqrt(na*na + nb * nb + nc * nc), nb / sqrt(na*na + nb * nb + nc * nc), nc / sqrt(na*na + nb * nb + nc * nc) };
                double paramz = 1;
                if (normal.z < 0)
                {
                	paramz = -1;
                }
                double offset = m_ui->groundEmptyValueDoubleSpinBox->value();
                if (m_ui->ceilEmptyValueDoubleSpinBox->isEnabled())
                {
                    offset = m_ui->ceilEmptyValueDoubleSpinBox->value();
                }
                v1 += normal * offset * paramz;
                v2 += normal * offset * paramz;
                v3 += normal * offset * paramz;
                CCVector3d centerPoint = (v1 + v2 + v3) / 3.0;
                double distance = (v1 - centerPoint).norm();
                double rectlegnght = 0;
                if (m_cloud2)
                {
                    ccBBox box = m_cloud2->getBB_recursive();
                    double curdistance = box.getDiagNormd() / 2;
                    rectlegnght = std::max(rectlegnght, curdistance);
                }
                if (m_cloud1)
                {
                    ccBBox box = m_cloud1->getBB_recursive();
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

            }
            else if (m_pickingPoints.size() > 3)
            {
                //多点拟合参考平面
                CCCoreLib::GenericIndexedCloudPersist* pickingPointsCloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(&m_pickingPoints);

                ccPlane* pPlane = ccPlane::Fit(pickingPointsCloud);
                if (pPlane)
                {
                    CCVector3 C, N;
                    N = pPlane->getNormal();//法向量
                    C = pPlane->getCenter();//中心
                    pPlane->getBB_recursive();//包围盒
                    pPlane->getEquation();//平面方程

                    //根据以上信息（可选）可视化平面
                    //todo...

                }


            }
            else
            {
                m_RectSurface->setVisible(false);
            }
        }
	}
 
    m_glWindow->redraw();

}


void ccVolumeCalcTool::slotExportWordFileToDesktop()
{

    QString selectedFilter;
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("VolumeCalcTool");
    QString currentPath = settings.value("VolumeCalcTool", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
	ccHObject::Container points = MainWindow::TheInstance()->getSelectedEntities();
	QString pointname= "";
	if (!points.empty())
	{
		ccHObject * entPoint = MainWindow::TheInstance()->getSelectedEntities().at(0);
		pointname = entPoint->getName();
	}
	MainWindow::TheInstance()->setGlobalZoom();
    QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(MainWindow::TheInstance(),
        QApplication::translate("ccVolumeCalcTool", "Save File", nullptr),
		currentPath + QString("/%1.%2").arg(pointname,"docx"), "(*.docx)", &selectedFilter);
	QFileInfo files(outputFilename);
	QString path = files.absolutePath();
    if (outputFilename.isEmpty())
    {
        settings.group();
        return;
    }

    settings.setValue("VolumeCalcTool", path);
    settings.endGroup();

    if (!outputFilename.contains(".docx"))
    {
        outputFilename = outputFilename + ".docx";
    }
    QString Photo = CS::Core::ICore::getDefaultPath() + "/Photo.bmp";
    if (!MainWindow::TheInstance()->GetGLWindow("3D View")->renderToFile(Photo, 0.5f, false, false))
    {
        return;
    }
	Photo = QDir::toNativeSeparators(Photo);
    map.insert("ProjectPhoto", Photo);
    /*  QPixmap pix(Photo);
      double x = pix.width();
      double y = pix.height();
      double k = 400.0 / pix.height();
      x = x * k;
      pix = pix.scaled(x, 420.0, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
      pix.save(Photo);*/
    //[!].显示等待框
	QString language = CS::Core::ICore::getOverrideLanguage();
	QString docfile = "";
	if (language == "zh")
	{
		 docfile = QApplication::applicationDirPath() + "/share/doc/VolumeCalcToolExport_chinese.docx";

	}
	else
	{
		 docfile = QApplication::applicationDirPath() + "/share/doc/VolumeCalcToolExport_english.docx";
	}
	QFileInfo  file(outputFilename);
	if (file.exists())
	{
		if (!QFile::remove(outputFilename))
		{
			CS::Widgets::FramelessMessageBox::information(MainWindow::TheInstance(), QApplication::translate("VolumeCalcDialog", "Tips", nullptr), QApplication::translate("VolumeCalcDialog", "The file is open, please close it and try again.", nullptr));
			return;
		}
	}
	if (!QFile::copy(docfile, outputFilename))
	{
		qDebug() << "error : volumecalctool copy docfile " << endl;
		return;
	}

	if (!file.exists())
	{
		qDebug() << "error : volumecalctool not find docfile " << endl;
		return;
	}
	map.insert("ProjectName", file.baseName());
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(MainWindow::TheInstance());
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("VolumeCalcDialog", "please wait...", nullptr));
	outputFilename = QDir::toNativeSeparators(outputFilename);
	qDebug() << "outputFilename:" << outputFilename;
    QtConcurrent::run([=]() {

        QObject end;
        connect(&end, &QObject::destroyed, this, [=]() {
            WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
			emit signalExportFileExit();
        });
       
        map.insert("ExportTime", QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss"));

        QAxObject *m_word = new QAxObject();
        if (!m_word->setControl("kwps.Application"))
        {
            if (!m_word->setControl("Word.Application"))
            {
                return;
            }
        }
        m_word->dynamicCall("SetVisible (bool Visible)", "false");
        m_word->setProperty("DisplayAlerts", false);
        QAxObject* m_documents = m_word->querySubObject("Documents");
        if (!m_documents)
        {
            return;
        }
        m_documents->dynamicCall("Open(QString)", outputFilename);
        QAxObject* m_document = m_word->querySubObject("ActiveDocument");
        if (!m_document)
        {
            return;
        }
        //获取当前光标位置
        if (!m_document)
        {
            return;
        }

        if (map.size() < 12)
        {
            return;
        }
        for (int i = 0; i < mlist.size(); i++)
        {
            if (mlist.at(i).contains("ProjectPhoto"))
            {
				QAxObject*bookmark_pic = m_document->querySubObject("Bookmarks(QVariant)", QVariant(mlist.at(i)));
				qDebug() << "bookmark_pic" << bookmark_pic->asVariant();
				if (!bookmark_pic->isNull())
				{
					//bookmark_pic->dynamicCall("Select(void)");
					//QAxObject *range = bookmark_pic->querySubObject("Range");
     //               QList<QVariant>qList;
     //               qList << QVariant(map.value(mlist.at(i)));
     //               qList << QVariant(false);
     //               qList << QVariant(true);
					//qList << QVariant(20);
					//qList << QVariant(1);
					//qList << QVariant(496.125);
					//qList << QVariant(311.85);
     //               qList << range->asVariant();
     //               QAxObject *shapes = m_document->querySubObject("Shapes");
					//shapes->dynamicCall("AddPicture(const QString&, QVariant, QVariant ,QVariant,QVariant,QVariant,QVariant,QVariant)", qList);

					bookmark_pic->dynamicCall("Select(void)");
					QAxObject *range = bookmark_pic->querySubObject("Range");
					QVariant tmp = range->asVariant();
					QList<QVariant>qList;
					qList << QVariant(map.value(mlist.at(i)));
					qList << QVariant(false);
					qList << QVariant(true); 
					qList << tmp;
					QAxObject *Inlineshapes = m_document->querySubObject("InlineShapes");
					Inlineshapes->dynamicCall("AddPicture(const QString&, QVariant, QVariant ,QVariant)", qList);
				}

            }
            else
            {
				qDebug() << "begintext:";
                QAxObject* bookmark = m_document->querySubObject("Bookmarks(QString)", mlist.at(i));
                if (bookmark)
                {
                    bookmark->dynamicCall("Select(void)");
                    bookmark->querySubObject("Range")->setProperty("Text", map.value(mlist.at(i)));
                    delete bookmark;
                }
            }

        }

        m_document->dynamicCall("Save()");


        if (m_word) {
            m_word->setProperty("DisplayAlerts", true);
        }
        if (m_document) {
            m_document->dynamicCall("Close(bool)", true);
        }
        if (m_word) {
            m_word->dynamicCall("Quit()");
        }

        m_document = nullptr;
        if (m_documents) {
            delete m_documents;
        }
        m_documents = nullptr;
        if (m_word) {
            delete m_word;
        }
        m_word = nullptr;
        QFile::remove(Photo);
        return;


    });
}
void ccVolumeCalcTool::slotExportFileExit()
{
	CS::Widgets::FramelessMessageBox::information(MainWindow::TheInstance(), QApplication::translate("VolumeCalcDialog","Tips",nullptr), QApplication::translate("VolumeCalcDialog", "Exported successfully.",nullptr));
}

void ccVolumeCalcTool::onItemPicked(const PickedItem& pi)
{
	if ((m_ui->groundComboBox->currentText() != tr("Custom") && m_ui->ceilComboBox->currentText() != tr("Custom")) || !pi.entity)
	{
		return;
	}
    if (pi.entity != m_cloud2 && pi.entity != m_cloud1)
    {
        return;
    }
	if (m_pickingPoints.size() >= 3)
	{
		clearCustomParam();
        gridIsUpToDate(true);
	}
	CCVector3d P = pi.P3D.toDouble();
	addAlignedPoint(P); //picked points are always shifted by default
	if (m_pickingPoints.size() == 3)
	{
		gridIsUpToDate(false);
		m_ui->groundEmptyValueDoubleSpinBox->setValue(0);
		m_ui->ceilEmptyValueDoubleSpinBox->setValue(0);
	}
	redrawConsultPlane();
	//ccGLWindow* win = MainWindow::TheInstance()->GetActiveGLWindow();
	//if (win)
	//{
	//	win->redraw();
	//}
}

void ccVolumeCalcTool::clearCustomParam()
{
	m_pickingPoints.removeAllChildren();
	m_pickingPoints.clear();
	ccGLWindow* win = MainWindow::TheInstance()->GetActiveGLWindow();
	if (win)
	{
		win->redraw();
	}
}

static cc2DLabel* CreateLabel(cc2DLabel* label, ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
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

static cc2DLabel* CreateLabel(ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
{
	return CreateLabel(new cc2DLabel, cloud, pointIndex, pointName, display);
}

bool ccVolumeCalcTool::addAlignedPoint(CCVector3d& Pin)
{
	m_pickingPoints.addPoint(Pin.toFloat());
	QString pointName = QString("P%1").arg(m_pickingPoints.size());
	cc2DLabel* label = CreateLabel(&m_pickingPoints, m_pickingPoints.size()-1, pointName, m_pickingPoints.getDisplay());
	m_pickingPoints.addChild(label);

	return true;
}

void ccVolumeCalcTool::slotEdgeLengthChanged()
{
    m_groundMaxEdgeLength = m_ui->doubleSpinBox_2->value();
    m_ceilMaxEdgeLength = m_ui->doubleSpinBox->value();
    gridIsUpToDate(false);
}