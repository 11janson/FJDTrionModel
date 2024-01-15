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

#include <ccPointAccuracyVerificationDlg.h>
#include"ccaccuracyinformation.h"
//Local
#include "mainwindow.h"
#include "ccAskThreeDoubleValuesDlg.h"

//common
#include <ccPickingHub.h>

//qCC_gl
#include <ccGLWindow.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccSphere.h>
#include <ccScalarField.h>

//qCC_io
#include <ccGlobalShiftManager.h>

//CC_FBO
#include <ccGlFilter.h>

//CCCoreLib
#include <RegistrationTools.h>
#include <GeometricalAnalysisTools.h>

////pcl
//#include <pcl/kdtree/kdtree.h>
#include <cc2sm.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/centroid.hpp>
#include<libs/model/projectmodel.h>
//Qt
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QToolButton>
#include <QSettings>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QAxObject>
#include <QComboBox>
#include <QButtonGroup>
#include "cloudcompareutils/icore.h"
#include <framelessmessagebox.h>
#include "framelessfiledialog.h"
#include <QStandardPaths>
#include "FJStyleManager.h"
#include "pointpairregistrationinformationdlg.h"
#include "ccDBRoot.h"
#include "fjitemdoublespinboxdelegate.h"
#include "libs/cloudcomparewidgets/metahubframelesswaitdialog.h"
#include <QtConcurrent>
#include "ccBBox.h"
#include "libs/cloudcompareutils/exportdatatofile.h"
//default position of each columns in the aligned and ref. table widgets
static const int XYZ_COL_INDEX			= 1;
static const int RMS_COL_INDEX			= 4;

//minimum number of pairs to let the user click on the align button
static const unsigned MIN_PAIRS_COUNT = 3;

ccPointAccuracyVerificationDlg::ccPointAccuracyVerificationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent/*=nullptr*/)
	: ccOverlayDialog(parent)
	, m_alignedPoints("aligned points")
	, m_refPoints("reference points")
	, m_paused(false)
	, m_pickingHub(pickingHub)
	, m_app(app)
{
    //默认参数
    params.zTolerance = 0.10;
    params.matchingNeighborhood = 0.10;
	assert(m_pickingHub);

	setupUi(this);
	QButtonGroup * radioButtonGroup = new QButtonGroup(this);
	radioButtonGroup->addButton(radioButton_altitude);
	radioButtonGroup->addButton(radioButton_altandplane);
	connect(validToolButton,		&QPushButton::clicked,	this,	&ccPointAccuracyVerificationDlg::setAccuracyInformation);
	connect(toolButton, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::cancel);
	connect(cancelToolButton, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::cancel);
	connect(radioButton_altitude, &QRadioButton::toggled, this, &ccPointAccuracyVerificationDlg::updateTypeChanged);
    connect(this, &ccPointAccuracyVerificationDlg::updatedialogUIHeight, this, &ccPointAccuracyVerificationDlg::updateDialogUi);
	connect(toolButton_pick, &QToolButton::clicked, [=]() {
		m_isPickModeOpen = !m_isPickModeOpen;
		if (m_associatedWin && m_isPickModeOpen)
		{
			m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
			setActionIcon(toolButton_pick, "clickPointIconClicked", "clickPointIconClicked", "clickPointIconDisable");
		}
		if (m_associatedWin && !m_isPickModeOpen)
		{
			m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
			setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
		}
	});
            
	connect(pushButton_import, &QToolButton::clicked, [=]() {
		QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
		settings.beginGroup("PointAccuracyVerificationDlg");
		QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
		QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "(*.xlsx *.csv *.txt)");
		if (selectedFile.isEmpty())
		{
			settings.endGroup();
			return;
		}
		settings.setValue("filedoc", QFileInfo(selectedFile).absolutePath());
		std::vector<CCVector3d>  data;
		bool readSuccess = true;
		if (QFileInfo(selectedFile).suffix().contains("xlsx")){
			readSuccess = getExcelFileData(selectedFile, data);
		}
		else if(QFileInfo(selectedFile).suffix().contains("txt") || QFileInfo(selectedFile).suffix().contains("csv")){
			readSuccess = getTxtFileData(selectedFile, data);
		}
		else if(QFileInfo(selectedFile).suffix().contains("csv")){
			readSuccess = getCsvFileData(selectedFile, data);
		}
		else
		{
			readSuccess = false;
		}
		if (readSuccess && data.size() != 0)
		{
			int currentRow = refPointsTableWidget->rowCount();
			for (int row = currentRow - 1; row >= 0; row--)
			{
				removeRefPoint(row, false);
			}
			for (auto curdata : data)
			{
				addReferencePoint(curdata, nullptr, false);
			}
			emit showMessage(tr("Import File Success!"));
		}
		else
		{
			emit showMessage(tr("Import File Failed!"));
			CS::Widgets::FramelessMessageBox::critical(this, tr("Error"), tr("Import failed"));
		}
		settings.endGroup();
	});
    connect(refPointsTableWidget, &QTableWidget::itemDoubleClicked, this, [=](QTableWidgetItem *item) {
        if (item)
        {
            if (item->column() != 0)
            {
                return;
            }
            int rowindex = item->row();
            double downX = refPointsTableWidget->item(rowindex, 1)->text().toDouble();
            double downY = refPointsTableWidget->item(rowindex, 2)->text().toDouble();
            double downZ = refPointsTableWidget->item(rowindex, 3)->text().toDouble();
            if (m_associatedWin)
            {
                CCVector3d P(downX, downY, downZ);
                P = m_refPoints.toLocal3d<double>(P);
                m_associatedWin->setCameraPos(P);
                m_associatedWin->setPivotPoint(P, false, false);
                ccBBox zoomedBox = m_refPoints.getBB_recursive(true);
                double bbDiag = zoomedBox.getDiagNorm();
                if (CCCoreLib::LessThanEpsilon(bbDiag))
                {
                    bbDiag = 1.0;
                }
                double targetWidth = bbDiag;
                if (m_associatedWin->glHeight() < m_associatedWin->glWidth())
                {
                    targetWidth *= static_cast<double>(m_associatedWin->glWidth()) / m_associatedWin->glHeight();
                }
                double focalDistance = targetWidth / m_associatedWin->getViewportParameters().computeDistanceToWidthRatio();
                CCVector3d v(0, 0, focalDistance);
                m_associatedWin->moveCamera(v);
                m_associatedWin->redraw();
            }
        }

    });
    connect(alignedPointsTableWidget, &QTableWidget::itemDoubleClicked, this, [=](QTableWidgetItem *item) {
        if (item)
        {
            if (item->column() != 0)
            {
                return;
            }
            int rowindex = item->row();
            double downX = alignedPointsTableWidget->item(rowindex, 1)->text().toDouble();
            double downY = alignedPointsTableWidget->item(rowindex, 2)->text().toDouble();
            double downZ = alignedPointsTableWidget->item(rowindex, 3)->text().toDouble();
            if (m_associatedWin)
            {
                CCVector3d P(downX, downY, downZ);
                P = m_alignedPoints.toLocal3d<double>(P);
                m_associatedWin->setPivotPoint(P, false, false);
                m_associatedWin->setCameraPos(P);
                ccBBox zoomedBox = m_alignedPoints.getBB_recursive(true);
                double bbDiag = zoomedBox.getDiagNorm();
                if (CCCoreLib::LessThanEpsilon(bbDiag))
                {
                    bbDiag = 1.0;
                }
                double targetWidth = bbDiag;
                if (m_associatedWin->glHeight() < m_associatedWin->glWidth())
                {
                    targetWidth *= static_cast<double>(m_associatedWin->glWidth()) / m_associatedWin->glHeight();
                }
                double focalDistance = targetWidth / m_associatedWin->getViewportParameters().computeDistanceToWidthRatio();
                CCVector3d v(0, 0, focalDistance);
                m_associatedWin->moveCamera(v);
                m_associatedWin->redraw();
            }
        }

    });
	connect(refPointsTableWidget, &QTableWidget::itemSelectionChanged, this, [=]() {
		QList<QTableWidgetItem*>itemList = refPointsTableWidget->selectedItems();
		std::set<int> rows;
		for (auto curList : itemList)
		{
			rows.insert(curList->row());
		}
		if (itemList.size() == 0)
		{
			toolButton_delete->setEnabled(false);
			toolButton_up->setEnabled(false);
			toolButton_down->setEnabled(false);
		}
		else
		{
			toolButton_delete->setEnabled(true);
			int currentRow = refPointsTableWidget->currentRow();
			if (currentRow != (refPointsTableWidget->rowCount() - 1) && refPointsTableWidget->rowCount() > 1 && rows.size() == 1)
			{
				toolButton_down->setEnabled(true);
			}
			else
			{
				toolButton_down->setEnabled(false);
			}
			if (currentRow != 0 && refPointsTableWidget->rowCount() > 1 && rows.size() == 1)
			{
				toolButton_up->setEnabled(true);
			}
			else
			{
				toolButton_up->setEnabled(false);
			}
		}
	});

	connect(alignedPointsTableWidget, &QTableWidget::itemSelectionChanged, this, [=]() {
		QList<QTableWidgetItem*>itemList = alignedPointsTableWidget->selectedItems();
		std::set<int> rows;
		for (auto curList : itemList)
		{
			rows.insert(curList->row());
		}
		if (itemList.size() == 0)
		{
			toolButton_delete_2->setEnabled(false);
			toolButton_up_2->setEnabled(false);
			toolButton_down_2->setEnabled(false);
		}
		else
		{
			toolButton_delete_2->setEnabled(true);
			int currentRow = alignedPointsTableWidget->currentRow();
			if (currentRow != (alignedPointsTableWidget->rowCount() - 1) && alignedPointsTableWidget->rowCount() > 1 && rows.size() == 1)
			{
				toolButton_down_2->setEnabled(true);
			}
			else
			{
				toolButton_down_2->setEnabled(false);
			}
			if (currentRow != 0 && alignedPointsTableWidget->rowCount() > 1 && rows.size() == 1)
			{
				toolButton_up_2->setEnabled(true);
			}
			else
			{
				toolButton_up_2->setEnabled(false);
			}
		}
	});

	connect(refPointsTableWidget, &QTableWidget::itemChanged, [=](QTableWidgetItem *item) {
		if (m_isTableDataChangeLock || item->column() == 0 || item->column() == 4)
		{
			return;
		}
		int currentrow = item->row();
		double downX = refPointsTableWidget->item(currentrow, 1)->text().toDouble();
		double downY = refPointsTableWidget->item(currentrow, 2)->text().toDouble();
		double downZ = refPointsTableWidget->item(currentrow, 3)->text().toDouble();

		int currentRowCount = refPointsTableWidget->rowCount();
		std::vector<CCVector3d> tableData;
		//for (int i = 0; i < currentRowCount; i++)
		//{
		//	CCVector3d point;
		//	point.x = refPointsTableWidget->item(i, 1)->text().toDouble();
		//	point.y = refPointsTableWidget->item(i, 2)->text().toDouble();
		//	point.z = refPointsTableWidget->item(i, 3)->text().toDouble();
		//	if (point.x != 0 && point.y != 0 && point.z != 0)
		//	{
		//		bool shiftEnabled = m_alignedEntities.isShifted;
		//		CCVector3d Pshift = m_alignedEntities.shift;
		//		double scale = 1.0;
		//		if (ccGlobalShiftManager::Handle(point, 0, ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT, shiftEnabled, Pshift, nullptr, &scale))
		//		{
		//			m_refPoints.setGlobalShift(Pshift);
		//			m_refPoints.setGlobalScale(scale);
		//		}
		//	}
		//}

		for (int i = 0; i < currentRowCount; i++)
		{
			CCVector3d point;
			point.x = refPointsTableWidget->item(i, 1)->text().toDouble();
			point.y = refPointsTableWidget->item(i, 2)->text().toDouble();
			point.z = refPointsTableWidget->item(i, 3)->text().toDouble();
			CCVector3 P = m_refPoints.toLocal3pc<double>(CCVector3d(point.x, point.y, point.z));
			m_refPoints.setPointVec(i, P);
		}


		onPointCountChanged();
		if (m_associatedWin)
		{
			m_associatedWin->redraw();
		}
	});

    connect(alignedPointsTableWidget, &QTableWidget::itemChanged, [=](QTableWidgetItem *item) {
        if (m_isTableDataChangeLock || item->column() == 0 || item->column() == 4)
        {
            return;
        }
        int currentrow = item->row();
        double downX = alignedPointsTableWidget->item(currentrow, 1)->text().toDouble();
        double downY = alignedPointsTableWidget->item(currentrow, 2)->text().toDouble();
        double downZ = alignedPointsTableWidget->item(currentrow, 3)->text().toDouble();

        int currentRowCount = alignedPointsTableWidget->rowCount();
        std::vector<CCVector3d> tableData;
        //for (int i = 0; i < currentRowCount; i++)
        //{
        //    CCVector3d point;
        //    point.x = alignedPointsTableWidget->item(i, 1)->text().toDouble();
        //    point.y = alignedPointsTableWidget->item(i, 2)->text().toDouble();
        //    point.z = alignedPointsTableWidget->item(i, 3)->text().toDouble();
        //    if (point.x != 0 && point.y != 0 && point.z != 0)
        //    {
        //        bool shiftEnabled = m_alignedEntities.isShifted;
        //        CCVector3d Pshift = m_alignedEntities.shift;
        //        double scale = 1.0;
        //        if (ccGlobalShiftManager::Handle(point, 0, ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT, shiftEnabled, Pshift, nullptr, &scale))
        //        {
        //            m_alignedPoints.setGlobalShift(Pshift);
        //            m_alignedPoints.setGlobalScale(scale);
        //        }
        //    }
        //}
        for (int i = 0; i < currentRowCount; i++)
        {
            CCVector3d point;
            point.x = alignedPointsTableWidget->item(i, 1)->text().toDouble();
            point.y = alignedPointsTableWidget->item(i, 2)->text().toDouble();
            point.z = alignedPointsTableWidget->item(i, 3)->text().toDouble();
            CCVector3 P = m_alignedPoints.toLocal3pc<double>(CCVector3d(point.x, point.y, point.z));
            m_alignedPoints.setPointVec(i, P);
        }
        onPointCountChanged();
        if (m_associatedWin)
        {
            m_associatedWin->redraw();
        }
    });
    connect(toolButton_add, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::addSource);
    connect(toolButton_up, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::upTarget);
    connect(toolButton_down, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::downTarget);
    connect(toolButton_delete, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::deleteTarget);
    connect(toolButton_add_2, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::addTarget);
    connect(toolButton_up_2, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::upSource);
    connect(toolButton_down_2, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::downSource);
    connect(toolButton_delete_2, &QToolButton::clicked, this, &ccPointAccuracyVerificationDlg::deleteSource);
	toolButton_up->setEnabled(false);
	toolButton_down->setEnabled(false);
	toolButton_delete->setEnabled(false);
	toolButton_up_2->setEnabled(false);
	toolButton_down_2->setEnabled(false);
	toolButton_delete_2->setEnabled(false);

	m_alignedPoints.setEnabled(true);
	m_alignedPoints.setVisible(false);

	m_refPoints.setEnabled(true);
	m_refPoints.setVisible(false);
	fjItemDoubleSpinboxDelegate *delegate = new fjItemDoubleSpinboxDelegate();
	alignedPointsTableWidget->setItemDelegateForColumn(1, delegate);
	alignedPointsTableWidget->setItemDelegateForColumn(2, delegate);
	alignedPointsTableWidget->setItemDelegateForColumn(3, delegate);
	refPointsTableWidget->setItemDelegateForColumn(1, delegate);
	refPointsTableWidget->setItemDelegateForColumn(2, delegate);
	refPointsTableWidget->setItemDelegateForColumn(3, delegate);
	alignedPointsTableWidget->setSelectionBehavior(QTableWidget::SelectRows);
	refPointsTableWidget->setSelectionBehavior(QTableWidget::SelectRows);
	alignedPointsTableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
	alignedPointsTableWidget->verticalHeader()->setVisible(false);
	alignedPointsTableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	alignedPointsTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	refPointsTableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
	refPointsTableWidget->verticalHeader()->setVisible(false);
	refPointsTableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	refPointsTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	//alignedPointsTableWidget->setStyleSheet("QTableCornerButton::section{background-color:#ff0000;border: 1px solid #dddddd;}");
	QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
	toolButton->setIcon(closeicon);
	alignedPointsTableWidget->setColumnWidth(0, 81);
	refPointsTableWidget->setColumnWidth(0, 81);
	for (int curColumn = 1; curColumn <5; curColumn++)
	{
		alignedPointsTableWidget->setColumnWidth(curColumn, 155);
		refPointsTableWidget->setColumnWidth(curColumn, 155);
	}
	frame->setVisible(false);
    QSettings Languagesettings(CS::Core::ICore::getDefaultPath() + "/config/language.ini", QSettings::IniFormat);
    Languagesettings.beginGroup(QStringLiteral("Translation"));
    QString language = Languagesettings.value(QStringLiteral("Language")).toString();
    Languagesettings.endGroup();
    if (language == "ja")
    {
        pushButton_import->setMinimumWidth(100);
    }
}

void ccPointAccuracyVerificationDlg::updateTypeChanged()
{
    
	if (radioButton_altitude->isChecked())
	{
		m_CurrentCompareType = ALTITUDE;
		setIsDownPartHide(true);
		m_isPickModeOpen = false;
		if (m_associatedWin)
		{
			m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
		}
		//int currentRow = alignedPointsTableWidget->rowCount();
		//for (int row = currentRow - 1; row >= 0; row--)
		//{
		//	removeAlignedPoint(row, false);
		//}
		setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
	}
	else
	{
		m_CurrentCompareType = ALTITUDEANDPLANE;
		setIsDownPartHide(false);
		m_isPickModeOpen = true;
		if (m_associatedWin)
		{
			m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
		}
		setActionIcon(toolButton_pick, "clickPointIconClicked", "clickPointIconClicked", "clickPointIconDisable");
    }
    onPointCountChanged();

    QSettings Languagesettings(CS::Core::ICore::getDefaultPath() + "/config/language.ini", QSettings::IniFormat);
    Languagesettings.beginGroup(QStringLiteral("Translation"));
    QString language = Languagesettings.value(QStringLiteral("Language")).toString();
    Languagesettings.endGroup();
    if (radioButton_altitude->isChecked())
    {
        if (language == "ja")
        {
            emit updatedialogUIHeight(595);
        }
        else
        {
            emit updatedialogUIHeight(575);
        }
        this->move(this->pos().x(), this->pos().y() - 1);
    }
    else
    {
        if (language == "ja")
        {
            emit updatedialogUIHeight(776);
        }
        else
        {
            emit updatedialogUIHeight(756);
        }
        this->move(this->pos().x(), this->pos().y() + 1);
    }
}

void ccPointAccuracyVerificationDlg::addSource()
{
	CCVector3d P(0, 0, 0);
	addAlignedPoint(P, nullptr, false);
}

void ccPointAccuracyVerificationDlg::addTarget()
{
	CCVector3d P(0, 0, 0);
	addReferencePoint(P, nullptr, false);
}

ccPointAccuracyVerificationDlg::EntityContext::EntityContext(ccHObject* ent)
	: entity(ent)
	, originalDisplay(entity ? entity->getDisplay() : nullptr)
	, wasVisible(entity ? entity->isVisible() : false)
	, wasEnabled(entity ? entity->isEnabled() : false)
	, wasSelected(entity ? entity->isSelected() : false)
{
}

void ccPointAccuracyVerificationDlg::EntityContext::restore()
{
	if (entity)
	{
		entity->setDisplay(originalDisplay);
		entity->setVisible(wasVisible);
		entity->setEnabled(wasEnabled);
		entity->setSelected(wasSelected);
		if (originalDisplay)
			originalDisplay->redraw();
	}
}

void ccPointAccuracyVerificationDlg::EntityContexts::fill(const ccHObject::Container& entities)
{
	clear();
	isShifted = false;

	for (ccHObject* entity : entities)
	{
		if (!entity)
		{
			assert(false);
			continue;
		}

		if (!isShifted)
		{
			ccShiftedObject* shiftedEntity = ccHObjectCaster::ToShifted(entity);
			if (shiftedEntity && shiftedEntity->isShifted())
			{
				isShifted = true;
				shift = shiftedEntity->getGlobalShift(); //we can only consider the first shift!
				scale = shiftedEntity->getGlobalScale();
			}
		}

		insert(entity, EntityContext(entity));
	}
}

void ccPointAccuracyVerificationDlg::clear()
{
	validToolButton->setEnabled(false);

	while (alignedPointsTableWidget->rowCount() != 0)
		alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount() - 1);
	while (refPointsTableWidget->rowCount() != 0)
		refPointsTableWidget->removeRow(refPointsTableWidget->rowCount() - 1);

	m_alignedPoints.removeAllChildren();
	m_alignedPoints.resize(0);
	m_alignedPoints.setGlobalShift(0, 0, 0);
	m_alignedPoints.setGlobalScale(1.0);
	m_alignedEntities.clear();
	m_refPoints.removeAllChildren();
	m_refPoints.resize(0);
	m_refPoints.setGlobalShift(0, 0, 0);
	m_refPoints.setGlobalScale(1.0);
	m_referenceEntities.clear();
}

bool ccPointAccuracyVerificationDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;
	if (oldWin)
	{
		if (oldWin != win)
		{
			oldWin->disconnect(this);
		}

		oldWin->removeFromOwnDB(&m_alignedPoints);
		m_alignedPoints.setDisplay(nullptr);
		oldWin->removeFromOwnDB(&m_refPoints);
		m_refPoints.setDisplay(nullptr);

		m_pickingHub->removeListener(this);
	}

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	m_alignedEntities.restoreAll();
	m_referenceEntities.restoreAll();

	if (oldWin && MainWindow::TheInstance())
	{
		QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
		//if (subWindow)
			//subWindow->close();
	}

	if (m_associatedWin)
	{
		if (!m_pickingHub->addListener(this, true))
		{
			ccLog::Error("Picking mechanism is already in use! Close the other tool first, and then restart this one.");
			return false;
		}
		m_associatedWin->addToOwnDB(&m_alignedPoints);
		m_associatedWin->addToOwnDB(&m_refPoints);

		//m_associatedWin->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE);
		//m_associatedWin->displayNewMessage("(you can add points 'manually' if necessary)", ccGLWindow::LOWER_LEFT_MESSAGE, true, 3600);
		//m_associatedWin->displayNewMessage(QString("Pick equivalent points on both clouds (at least %1 pairs - mind the order)").arg(MIN_PAIRS_COUNT), ccGLWindow::LOWER_LEFT_MESSAGE, true, 3600);
	}

	return true;
}

bool ccPointAccuracyVerificationDlg::start()
{
	radioButton_altitude->setChecked(true);
	
	m_CurrentCompareType = ALTITUDE;
	validToolButton->setEnabled(false);
	toolButton_up->setEnabled(false);
	toolButton_down->setEnabled(false);
	toolButton_delete->setEnabled(false);
	toolButton_up_2->setEnabled(false);
	toolButton_down_2->setEnabled(false);
	toolButton_delete_2->setEnabled(false);
	setActionIcon(toolButton_down, "movedownnormal", "movedownclicked", "movedowndisabled");
	setActionIcon(toolButton_pick, "clickPointIconClicked", "clickPointIconClicked", "clickPointIconDisable");
	setActionIcon(toolButton_up, "moveupnormal", "moveupclicked", "moveupdisabled");
	setActionIcon(toolButton_delete, "deleterowsnormal", "deleterowsclicked", "deleterowsdisabled");
	setActionIcon(toolButton_down_2, "movedownnormal", "movedownclicked", "movedowndisabled");
	setActionIcon(toolButton_up_2, "moveupnormal", "moveupclicked", "moveupdisabled");
	setActionIcon(toolButton_delete_2, "deleterowsnormal", "deleterowsclicked", "deleterowsdisabled");
	setActionIcon(pushButton_import, "importdatanormal", "importdataclicked", "importdatadisabled");
	setActionIcon(toolButton_add, "addpointdatanormal", "addpointdataclicked", "addpointdatadisabled");
	setActionIcon(toolButton_add_2, "addpointdatanormal", "addpointdataclicked", "addpointdatadisabled");
	assert(!m_alignedEntities.empty());
	updateTypeChanged();
	return ccOverlayDialog::start();
}

void ccPointAccuracyVerificationDlg::stop(bool state)
{
	ccOverlayDialog::stop(state);
}

static void SetEnabled_recursive(ccHObject* ent)
{
	assert(ent);
	ent->setEnabled(true);
	if (ent->getParent())
		SetEnabled_recursive(ent->getParent());
}


bool ccPointAccuracyVerificationDlg::init(	ccGLWindow* win,
										const ccHObject::Container& alignedEntities,
										const ccHObject::Container* referenceEntities/*=nullptr*/)
{
	if (!win)
	{
		assert(false);
		return false;
	}

	clear();

	if (alignedEntities.empty())
	{
		ccLog::Error("[PointPairRegistration] Need at least one aligned entity!");
		return false;
	}
	


	m_alignedEntities.fill(alignedEntities);
	if (referenceEntities)
	{
		m_referenceEntities.fill(*referenceEntities);
	}
	
	//create dedicated 3D view
	if (!m_associatedWin)
	{
		//import GL filter so as to get the same rendering aspect!
		{
			//find an orgin display (we'll take the first valid one)
			ccGenericGLDisplay* sourceDisplay = nullptr;
			for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
			{
				sourceDisplay = it.key()->getDisplay();
				if (sourceDisplay)
					break;
			}
			if (!sourceDisplay && !m_referenceEntities.empty())
			{
				for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
				{
					sourceDisplay = it.key()->getDisplay();
					if (sourceDisplay)
						break;
				}
			}
			if (sourceDisplay)
			{
				ccGlFilter* filter = static_cast<ccGLWindow*>(sourceDisplay)->getGlFilter();
				if (filter)
					win->setGlFilter(filter->clone());
			}
		}
		linkWith(win);
		assert(m_associatedWin);
	}

	//add aligned entity to display
	ccViewportParameters originViewportParams;
	bool hasOriginViewportParams = false;
	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
	{
		ccHObject* aligned = it.key();
		if (aligned->getDisplay())
		{
			hasOriginViewportParams = true;
			originViewportParams = aligned->getDisplay()->getViewportParameters();
		}
		//DGM: it's already in the global DB!
		//m_associatedWin->addToOwnDB(aligned);
		aligned->setDisplay(m_associatedWin);
		aligned->setVisible(true);
		SetEnabled_recursive(aligned);
		//SetVisible_recursive(aligned);

		//add the 1-point child labels as well (if any)
		for (unsigned i = 0; i < aligned->getChildrenNumber(); ++i)
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(aligned->getChild(i));
			if (label && label->size() == 1)
			{
				m_alignedEntities.insert(label, EntityContext(label));
				m_associatedWin->addToOwnDB(label);
				label->setDisplay(m_associatedWin);
				label->setVisible(true);
			}
		}
	}

	//add reference entity (if any) to display
	for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
	{
		ccHObject* reference = it.key();
		if (!hasOriginViewportParams && reference->getDisplay())
		{
			hasOriginViewportParams = true;
			originViewportParams = reference->getDisplay()->getViewportParameters();
		}
		//DGM: it's already in the global DB!
		//m_associatedWin->addToOwnDB(reference);
		reference->setDisplay(m_associatedWin);
		reference->setVisible(true);
		SetEnabled_recursive(reference);
		//SetVisible_recursive(reference);

		//add the 1-point child labels as well (if any)
		for (unsigned i = 0; i < reference->getChildrenNumber(); ++i)
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(reference->getChild(i));
			if (label && label->size() == 1)
			{
				m_referenceEntities.insert(label, EntityContext(label));
				m_associatedWin->addToOwnDB(label);
				label->setDisplay(m_associatedWin);
				label->setVisible(true);
			}
		}
	}

	//showReferenceCheckBox->setChecked(!m_referenceEntities.empty());
	//showReferenceCheckBox->setEnabled(!m_referenceEntities.empty());
	//showAlignedCheckBox->setChecked(true);

	m_associatedWin->showMaximized();
	resetTitle();

	if (hasOriginViewportParams)
	{
		m_associatedWin->setViewportParameters(originViewportParams);
		m_associatedWin->redraw();
	}
	else
	{
		m_associatedWin->zoomGlobal();
		m_associatedWin->redraw(); //already called by zoomGlobal
	}

	onPointCountChanged();
	
	return true;
}

void ccPointAccuracyVerificationDlg::pause(bool state)
{
	m_paused = state;
	setDisabled(state);
}


void ccPointAccuracyVerificationDlg::onItemPicked(const PickedItem& pi)
{
	
	if (!m_associatedWin)
		return;
	
	//no point picking when paused!
	if (m_paused)
		return;

	if (!pi.entity)
		return;
	if (m_CurrentCompareType == ALTITUDE || !m_isPickModeOpen)
	{
		return;
	}

	if (m_alignedEntities.contains(pi.entity))
	{
		CCVector3d P = pi.P3D.toDouble();
		addAlignedPoint(P, pi.entity, true); //picked points are always shifted by default

	}
	else if (m_referenceEntities.contains(pi.entity))
	{
		CCVector3d P = pi.P3D.toDouble();
		addReferencePoint(P, pi.entity, true); //picked points are always shifted by default
	}
	else
	{
		if (pi.entity && pi.entity->isA(CC_TYPES::LABEL_2D))
		{
			ccLog::Error(tr("Point/label already picked"));
		}
		else
		{
			assert(false);
		}
		return;
	}
	m_associatedWin->redraw();
}

void ccPointAccuracyVerificationDlg::onPointCountChanged()
{
	bool canAlign = (m_alignedPoints.size() == m_refPoints.size() && m_refPoints.size() >= MIN_PAIRS_COUNT);
	validToolButton->setEnabled(false);
	updateAlignInfo();
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


void ccPointAccuracyVerificationDlg::addPointToTable(	QTableWidget* tableWidget,
													int rowIndex,
													const CCVector3d& P,
													QString pointName )
{
	assert(tableWidget);
	if (!tableWidget)
		return;

	//add corresponding row in table
	tableWidget->setRowCount(std::max<int>(rowIndex + 1, tableWidget->rowCount()));
	//tableWidget->setVerticalHeaderItem(rowIndex, new QTableWidgetItem(pointName));
	QTableWidgetItem* labelitem = new QTableWidgetItem(pointName);
	labelitem->setFlags(labelitem->flags() & ~Qt::ItemIsEditable);
	labelitem->setTextAlignment(Qt::AlignCenter);
    labelitem->setToolTip(tr("Double-click the ID of a selected point to move the point to the view center."));
	tableWidget->setItem(rowIndex, 0, labelitem);
	//add point coordinates
	for (int d = 0; d < 3; ++d)
	{
		if (d == 0)
		{
			m_isTableDataChangeLock = true;
		}
		if (d == 2)
		{
			m_isTableDataChangeLock = false;
		}
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setData(Qt::EditRole, QString::number(P.u[d], 'f', 3));
		item->setTextAlignment(Qt::AlignCenter);
		tableWidget->setItem(rowIndex, XYZ_COL_INDEX + d, item);
		tableWidget->setRowHeight(rowIndex, 32);
	}
	QTableWidgetItem* itemError = new QTableWidgetItem();
	itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
	tableWidget->setItem(rowIndex, XYZ_COL_INDEX + 3, itemError);
	int lastcurrentRow = tableWidget->rowCount();
	tableWidget->setCurrentCell(lastcurrentRow - 1, 1);

}

bool ccPointAccuracyVerificationDlg::addAlignedPoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=false*/)
{
	//if the input point is not shifted, we shift it to the aligned coordinate system
	assert(entity == nullptr || m_alignedEntities.contains(entity));

	//first point?
	if (m_alignedPoints.size() == 0)
	{
		//simply copy the cloud global shift/scale
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (cloud)
		{
			m_alignedPoints.copyGlobalShiftAndScale(*cloud);
		}
        auto cloudref = (m_alignedEntities.begin()).key();
        if (cloudref)
        {
            ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloudref);
            if (shiftcloud)
            {
                m_refPoints.copyGlobalShiftAndScale(*shiftcloud);
            }
        }
	}



	//transform the input point in the 'global world' by default
	if (shifted)
		Pin = m_alignedPoints.toGlobal3d<double>(Pin);

	//check that we don't duplicate points
	for (unsigned i = 0; i < m_alignedPoints.size(); ++i)
	{
		CCVector3d Pi = m_alignedPoints.toGlobal3d<PointCoordinateType>(*m_alignedPoints.getPoint(i));
		//if (CCCoreLib::LessThanSquareEpsilon((Pi - Pin).norm2d()))
		//{
		//	ccLog::Error("Point already picked or too close to an already selected one!");
		//	return false;
		//}
	}

	unsigned newPointIndex = m_alignedPoints.size();
	if (newPointIndex == m_alignedPoints.capacity() && !m_alignedPoints.reserve(newPointIndex + 1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}

	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_alignedPoints.toLocal3pc<double>(Pin);
	m_alignedPoints.addPoint(P);
	
	QString pointName = QString("R%1").arg(newPointIndex);

	//eventually add a label (or a sphere)
	//if (sphereRadius <= 0)
	{
		//if the picked point is associated to a label
		cc2DLabel* originLabel = nullptr;
		if (entity && entity->isA(CC_TYPES::LABEL_2D))
		{
			for (const EntityContext& ec : m_alignedEntities)
			{
				if (ec.entity == entity)
				{
					originLabel = ccHObjectCaster::To2DLabel(ec.entity);

					//we've found a label corresponding to the chosen point
					pointName += QString(" (%1)").arg(originLabel->getName());
					originLabel->setEnabled(false);

					//TODO add the label name to a new column?
				}
			}
		}

		cc2DLabel* label = CreateLabel(&m_alignedPoints, newPointIndex, pointName, m_associatedWin);
		label->setIsShowConfigColor(false);
		label->setPointColor(ccColor::red);
		if (originLabel)
		{
			//remember the associated label
			label->setMetaData("AssociatedLabelID", originLabel->getUniqueID());
		}
		m_alignedPoints.addChild(label);
	}


	//add corresponding row in table
	addPointToTable(alignedPointsTableWidget, newPointIndex, Pin, pointName);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	return true;
}

void ccPointAccuracyVerificationDlg::removeLabel(	ccPointCloud& points,
												unsigned childIndex,
												const EntityContexts& entities )
{
	if (childIndex >= points.getChildrenNumber())
	{
		assert(false);
		return;
	}
	
	cc2DLabel* label = ccHObjectCaster::To2DLabel(points.getChild(childIndex));
	if (label)
	{
		//if the label had an associated label
		if (label->hasMetaData("AssociatedLabelID"))
		{
			unsigned uuid = label->metaData()["AssociatedLabelID"].toUInt();
			for (const EntityContext& ec : entities)
			{
				if (ec.entity && ec.entity->getUniqueID() == uuid)
				{
					//restore the original entity status (normally it was enabled!)
					ec.entity->setEnabled(ec.wasEnabled);
					break;
				}
			}
		}
	}
	else
	{
		////should be a label!
		//assert(false);
	}
	points.removeChild(childIndex);
}

void ccPointAccuracyVerificationDlg::unstackAligned()
{
	unsigned pointCount = m_alignedPoints.size();
	if (pointCount == 0) //nothing to do
		return;

	assert(alignedPointsTableWidget->rowCount() > 0);
	alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount() - 1);

	//remove label
	assert(m_alignedPoints.getChildrenNumber() == pointCount);
	--pointCount;
	removeLabel(m_alignedPoints, pointCount, m_alignedEntities);
	//remove point
	m_alignedPoints.resize(pointCount);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointAccuracyVerificationDlg::removeAlignedPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_alignedPoints.size()))
	{
		ccLog::Error("[ccPointAccuracyVerificationDlg::removeAlignedPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_alignedPoints.size());
	//remove the label (or sphere)
	removeLabel(m_alignedPoints, index, m_alignedEntities);
	//remove array row
	alignedPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i = index + 1; i < pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_alignedPoints.getPoint(i - 1)) = *m_alignedPoints.getPoint(i);

		//new name
		QString pointName = QString("R%1").arg(i - 1);
		//update the label (if any)
		ccHObject* child = m_alignedPoints.getChild(i - 1);
		if (child)
		{
			if (child->isKindOf(CC_TYPES::LABEL_2D))
			{
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				label->clear();
				CreateLabel(label, &m_alignedPoints, static_cast<unsigned>(i - 1), pointName, m_associatedWin);
			}
			else //probably a sphere
			{
				child->setName(pointName);
			}
		}
		//update array
		QTableWidgetItem* labelitem = new QTableWidgetItem(pointName);
		labelitem->setFlags(labelitem->flags() & ~Qt::ItemIsEditable);
		labelitem->setTextAlignment(Qt::AlignCenter);
        labelitem->setToolTip(tr("Double-click the ID of a selected point to move the point to the view center."));
		alignedPointsTableWidget->setItem(i - 1, 0, labelitem);
	}
	m_alignedPoints.invalidateBoundingBox();
	
	pointCount--;
	assert(pointCount >= 0);
	m_alignedPoints.resize(static_cast<unsigned>(pointCount));

	if (m_alignedPoints.size() == 0)
	{
		//reset global shift (if any)
		m_alignedPoints.setGlobalShift(0, 0, 0);
		m_alignedPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_refPoints.size()))
	{
		removeRefPoint(index, false);
	}
}

bool ccPointAccuracyVerificationDlg::getExcelFileData(QString filename, std::vector<CCVector3d>& data) {
    Path = filename;
    data.clear();

    QXlsx::Document xlsx(filename);
    if (!xlsx.load()) {
        return false;
    }
    QXlsx::CellRange range = xlsx.dimension();
    int rowCount = range.rowCount();
    int columnCount = range.columnCount();
    if (columnCount == 3) {
        for (int i = 1; i <= rowCount; ++i) {
			CCVector3d d;
            for (int j = 1; j <= columnCount; ++j) {
                QVariant value = xlsx.read(i, j);
                if (!value.canConvert<double>()) {
                    qDebug() << "Error: Non-numeric data found in cell (" << i << "," << j << ")";
					return false;
                }
                if (j == 1)
                {
                    d.x = value.toDouble();
                }
                else if (j == 2)
                {
                    d.y = value.toDouble();
                }
                else
                {
                    d.z = value.toDouble();
                }
            }
            data.push_back(d);
        }
	}
    else if (columnCount == 4) {
        for (int i = 1; i <= rowCount; ++i) {
			CCVector3d d;
            for (int j = 2; j <= 4; ++j) {
                QVariant value = xlsx.read(i, j);
                if (!value.canConvert<double>()) {
                    qDebug() << "Error: Non-numeric data found in cell (" << i << "," << j << ")";
					return false;
                }
                if (j == 2)
                {
                    d.x = value.toDouble();
                }
                else if (j == 3)
                {
                    d.y = value.toDouble();
                }
                else
                {
                    d.z = value.toDouble();
                }
            }
            data.push_back(d);
        }
    }
	 else {
        qDebug() << "Error loading the file.";
		return false;
    }
	return true;
}

bool ccPointAccuracyVerificationDlg::getCsvFileData(const QString& filename, std::vector<CCVector3d>& data) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    QTextStream in(&file);
    QStringList firstLine = in.readLine().split(',');

    bool readFirstColumn = true;

    if (!firstLine.isEmpty()) {
        if (!isValidDouble(firstLine[0])) {
            readFirstColumn = false;
        }
    }
    else {
        file.close();
        return false;
    }

    do {
        QStringList columns = firstLine;

        if ((readFirstColumn && columns.size() >= 3) || (!readFirstColumn && columns.size() >= 4)) {
			int firstCount; 
			int lastCount;
			if (readFirstColumn)
			{
				firstCount = 0;
				lastCount = 3;
			}
			else 
			{
				firstCount = 1;
				lastCount = 4;
			}
			
			for (firstCount; firstCount < lastCount; firstCount++ ) {
                if (!isValidDouble(columns.at(firstCount))) {
                    qDebug() << "Invalid data detected in the file.";
                    file.close();
                    return false;
                }
            }           
			CCVector3d lineData;

            if (readFirstColumn) {
                lineData.x = columns[0].toDouble();
                lineData.y = columns[1].toDouble();
                lineData.z = columns[2].toDouble();
            }
            else {
                lineData.x = columns[1].toDouble();
                lineData.y = columns[2].toDouble();
                lineData.z = columns[3].toDouble();
            }

            data.push_back(lineData);
        }
        else {
            qDebug() << "Invalid data at row:" << data.size() + 1;
            file.close();
            return false;
        }
		firstLine = in.readLine().split(',');

    } while (!in.atEnd());

    file.close();
    return true;
}


bool ccPointAccuracyVerificationDlg::isValidDouble(const QString& str)
{
    bool ok;
    str.toDouble(&ok);
    return ok;
}

bool ccPointAccuracyVerificationDlg::addReferencePoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=true*/)
{
	assert(entity == nullptr || m_referenceEntities.contains(entity));

	ccGenericPointCloud* cloud = entity ? ccHObjectCaster::ToGenericPointCloud(entity) : nullptr;

	//first point?
	if (m_refPoints.size() == 0)
	{
		if (entity) //picked point
		{
			//simply copy the cloud global shift/scale
			if (cloud)
			{
				m_refPoints.copyGlobalShiftAndScale(*cloud);
			}
		}
		else //virtual point
		{
			//m_refPoints.setGlobalScale(1.0);
			//m_refPoints.setGlobalShift(0, 0, 0);

			//if (!shifted)
			//{
			//	//test that the input point has not too big coordinates
			//	//we use the aligned shift by default (if any)
			//	bool shiftEnabled = m_alignedEntities.isShifted;
			//	CCVector3d Pshift = m_alignedEntities.shift;
			//	double scale = 1.0;
			//	if (ccGlobalShiftManager::Handle(Pin, 0, ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT, shiftEnabled, Pshift,  nullptr, &scale))
			//	{
			//		m_refPoints.setGlobalShift(Pshift);
			//		m_refPoints.setGlobalScale(scale);
			//	}
			//}
            auto cloudref = (m_alignedEntities.begin()).key();
            if (cloudref)
            {
                ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloudref);
                if (shiftcloud)
                {
                    m_refPoints.copyGlobalShiftAndScale(*shiftcloud);
                }
            }
		}
	}

	PointCoordinateType sphereRadius = 0;

	//transform the input point in the 'global world' by default
	if (shifted && cloud)
	{
		Pin = cloud->toGlobal3d<double>(Pin);
	}

	//check that we don't duplicate points
	for (unsigned i = 0; i < m_refPoints.size(); ++i)
	{
		//express the 'Pi' point in the current global coordinate system
		CCVector3d Pi = m_refPoints.toGlobal3d<PointCoordinateType>(*m_refPoints.getPoint(i));
		//if (CCCoreLib::LessThanSquareEpsilon((Pi - Pin).norm2d()))
		//{
		//	ccLog::Error("Point already picked or too close to an already selected one!");
		//	return false;
		//}
	}

	//add point to the 'reference' set
	unsigned newPointIndex = m_refPoints.size();
	if (newPointIndex == m_refPoints.capacity() && !m_refPoints.reserve(newPointIndex + 1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}
	
	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_refPoints.toLocal3pc<double>(Pin);
	m_refPoints.addPoint(P);

	QString pointName = QString("C%1").arg(newPointIndex);

	//eventually add a label (or a sphere)
	//if (sphereRadius <= 0)
	{
		//if the picked point is associated to a label
		cc2DLabel* originLabel = nullptr;
		if (entity && entity->isA(CC_TYPES::LABEL_2D))
		{
			for (const EntityContext& ec : m_referenceEntities)
			{
				if (ec.entity == entity)
				{
					originLabel = ccHObjectCaster::To2DLabel(ec.entity);

					//we've found a label corresponding to the chosen point
					pointName += QString(" (%1)").arg(originLabel->getName());
					originLabel->setEnabled(false);

					//TODO add the label name to a new column?
				}
			}
		}

		cc2DLabel* label = CreateLabel(&m_refPoints, newPointIndex, pointName, m_associatedWin);
		label->setIsShowConfigColor(false);
		label->setPointColor(ccColor::green);
		if (originLabel)
		{
			//remember the associated label
			label->setMetaData("AssociatedLabelID", originLabel->getUniqueID());
		}
		m_refPoints.addChild(label);
	}

	//add corresponding row in table
	addPointToTable(refPointsTableWidget, newPointIndex, Pin, pointName);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	return true;
}

void ccPointAccuracyVerificationDlg::unstackRef()
{
	unsigned pointCount = m_refPoints.size();
	if (pointCount == 0)
		return;

	assert(refPointsTableWidget->rowCount() > 0);
	refPointsTableWidget->removeRow(refPointsTableWidget->rowCount() - 1);

	//remove label
	assert(m_refPoints.getChildrenNumber() == pointCount);
	pointCount--;
	removeLabel(m_refPoints, pointCount, m_referenceEntities);
	//remove point
	m_refPoints.resize(pointCount);

	if (pointCount == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0, 0, 0);
		m_refPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointAccuracyVerificationDlg::removeRefPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_refPoints.size()))
	{
		ccLog::Error("[ccPointAccuracyVerificationDlg::removeRefPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_refPoints.size());
	//remove the label (or sphere)
	removeLabel(m_refPoints, index, m_referenceEntities);
	//remove array row
	refPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i = index + 1; i < pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_refPoints.getPoint(i - 1)) = *m_refPoints.getPoint(i);

		//new name
		QString pointName = QString("C%1").arg(i - 1);
		//update the label (if any)
		ccHObject* child = m_refPoints.getChild(i - 1);
		if (child)
		{
			if (child->isKindOf(CC_TYPES::LABEL_2D))
			{
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				label->clear();
				CreateLabel(label, &m_refPoints, static_cast<unsigned>(i - 1), pointName, m_associatedWin);
			}
			else //probably a sphere
			{
				child->setName(pointName);
			}
		}
		//update array
		QTableWidgetItem* labelitem = new QTableWidgetItem(pointName);
		labelitem->setFlags(labelitem->flags() & ~Qt::ItemIsEditable);
		labelitem->setTextAlignment(Qt::AlignCenter);
        labelitem->setToolTip(tr("Double-click the ID of a selected point to move the point to the view center."));
		refPointsTableWidget->setItem(i - 1, 0, labelitem);
	}
	m_refPoints.invalidateBoundingBox();

	pointCount--;
	assert(pointCount >= 0);
	m_refPoints.resize(static_cast<unsigned>(pointCount));

	if (m_refPoints.size() == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0, 0, 0);
		m_refPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_alignedPoints.size()))
	{
		removeAlignedPoint(index, false);
	}
}

void ccPointAccuracyVerificationDlg::updateDialogUi(int height)
{
    setFixedHeight(height);
}

void ccPointAccuracyVerificationDlg::showAlignedEntities(bool state)
{
	if (m_alignedEntities.empty())
		return;

	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		it.key()->setVisible(state);
	m_alignedPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (true)
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

void ccPointAccuracyVerificationDlg::showReferenceEntities(bool state)
{
	if (m_referenceEntities.empty())
		return;

	for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
		it.key()->setVisible(state);
	m_refPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (true)
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

bool ccPointAccuracyVerificationDlg::callHornRegistration(CCCoreLib::PointProjectionTools::Transformation& trans, double& rms, bool autoUpdateTab)
{
	if (m_alignedEntities.empty())
	{
		assert(false);
		return false;
	}

	if (m_alignedPoints.size() != m_refPoints.size() || m_refPoints.size() < MIN_PAIRS_COUNT)
	{
		assert(false);
		ccLog::Error(QString("Need at least %1 points for each entity (and the same number of points in both subsets)!").arg(MIN_PAIRS_COUNT));
		return false;
	}

	//fixed scale?
	bool adjustScale = false;

	//call Horn registration method
	if (!CCCoreLib::HornRegistrationTools::FindAbsoluteOrientation(&m_alignedPoints, &m_refPoints, trans, !adjustScale))
	{
		ccLog::Error("Registration failed! (points are aligned?)");
		return false;
	}

	//apply constraints (if any)
	{
		int filters = 0;
		//switch (rotComboBox->currentIndex())
		//{
		//case 1:
		//	filters |= CCCoreLib::RegistrationTools::SKIP_RYZ;
		//	break;
		//case 2:
		//	filters |= CCCoreLib::RegistrationTools::SKIP_RXZ;
		//	break;
		//case 3:
		//	filters |= CCCoreLib::RegistrationTools::SKIP_RXY;
		//	break;
		//default:
		//	//nothing to do
		//	break;
		//}

		//if (!TxCheckBox->isChecked())
		//	filters |= CCCoreLib::RegistrationTools::SKIP_TX;
		//if (!TyCheckBox->isChecked())
		//	filters |= CCCoreLib::RegistrationTools::SKIP_TY;
		//if (!TzCheckBox->isChecked())
		//	filters |= CCCoreLib::RegistrationTools::SKIP_TZ;

		if (filters != 0)
		{
			CCCoreLib::RegistrationTools::FilterTransformation(trans, filters, trans);
		}
	}

	//compute RMS
	rms = CCCoreLib::HornRegistrationTools::ComputeRMS(&m_alignedPoints, &m_refPoints, trans);

	//if (autoUpdateTab)
	//{
	//	//display resulting RMS in colums
	//	if (rms >= 0)
	//	{
	//		assert(m_alignedPoints.size() == m_refPoints.size());
	//		for (unsigned i = 0; i < m_alignedPoints.size(); ++i)
	//		{
	//			const CCVector3* Ri = m_refPoints.getPoint(i);
	//			const CCVector3* Li = m_alignedPoints.getPoint(i);
	//			CCVector3d Lit = trans.apply(*Li);
	//			double dist = (Ri->toDouble() - Lit).norm();

	//			QTableWidgetItem* itemA = new QTableWidgetItem();
	//			itemA->setFlags(itemA->flags() & ~Qt::ItemIsEditable);
	//			itemA->setData(Qt::EditRole, dist);
	//			alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, itemA);
	//			QTableWidgetItem* itemR = new QTableWidgetItem();
	//			itemR->setData(Qt::EditRole, dist);
	//			itemR->setFlags(itemR->flags() & ~Qt::ItemIsEditable);
	//			refPointsTableWidget->setItem(i, RMS_COL_INDEX, itemR);
	//		}
	//	}
	//	else
	//	{
	//		//clear RMS columns
	//		clearRMSColumns();
	//	}
	//}

	return true;
}

void ccPointAccuracyVerificationDlg::clearRMSColumns()
{
	for (int i = 0; alignedPointsTableWidget->rowCount(); ++i)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setFlags(item->flags() & ~Qt::ItemIsEditable);
		item->setTextAlignment(Qt::AlignCenter);
		alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, item);
	}
	for (int i=0; refPointsTableWidget->rowCount(); ++i)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setFlags(item->flags() & ~Qt::ItemIsEditable);
		item->setTextAlignment(Qt::AlignCenter);
		refPointsTableWidget->setItem(i, RMS_COL_INDEX, item);
	}
}

void ccPointAccuracyVerificationDlg::resetTitle()
{
	if ( m_associatedWin != nullptr )
	{
		//m_associatedWin->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false);
		//m_associatedWin->displayNewMessage("[Point-pair registration]", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600);
	}
}

void ccPointAccuracyVerificationDlg::updateAlignInfo()
{
	//reset title
	resetTitle();

	CCCoreLib::PointProjectionTools::Transformation trans;
	double rms;

	/*if (	m_alignedPoints.size() == m_refPoints.size()
		&&	m_refPoints.size() >= 0
		&&	callHornRegistration(trans, rms, true))*/
    if (m_alignedPoints.size() == m_refPoints.size()
        && m_refPoints.size() > 0)
	{
		//QString rmsString = QString("Achievable RMS: %1").arg(rms);
		//m_associatedWin->displayNewMessage(rmsString, ccGLWindow::UPPER_CENTER_MESSAGE, true, 60 * 60);
		//resetToolButton->setEnabled(true);
		validToolButton->setEnabled(true);
	}
	else
	{
        if (m_refPoints.size() >0 && m_CurrentCompareType == ALTITUDE)
        {
            validToolButton->setEnabled(true);
        }
		//resetToolButton->setEnabled(false);
		//validToolButton->setEnabled(false);
	}

	m_associatedWin->redraw();
}



void ccPointAccuracyVerificationDlg::apply()
{
    std::vector<CCVector3> controlPoint;
    for (int i = 0; i < m_refPoints.size(); ++i)
    {
        controlPoint.push_back(*m_refPoints.getPoint(i));
    }
   if (m_CurrentCompareType == ALTITUDEANDPLANE)
   {
       std::vector<CCVector3> pickPoint;
       std::vector<CCVector3> controlPoint;
       for (int i = 0; i < m_refPoints.size(); ++i)
       {
           pickPoint.push_back(*m_alignedPoints.getPoint(i));
           controlPoint.push_back(*m_refPoints.getPoint(i));
       }
       params.zTolerance = getCurrentZTolerance();
       params.matchingNeighborhood = getMatchingNeighborhood();
       computeHeightAndPlaneAccuracy(pickPoint, controlPoint);
   }
   else
   {
       //创建点云对象
       pcl::PointCloud<pcl::PointXYZ>::Ptr temPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
       //ccHObject* aligned = nullptr;
       /*for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
       {
           ccHObject* aligned = it.key();
       }*/
       
       ccHObject* aligned = m_alignedEntities.firstKey();
       ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(aligned);
       //点云数据结构转换
       pcl::PointXYZ temp;
       CCVector3 clipRange(1.5, 1.5, 1.5);
       
       for (int i = 0; i < controlPoint.size(); ++i)
       {
           CCCoreLib::ReferenceCloud* cloudTem = new CCCoreLib::ReferenceCloud(nullptr);
           //Melone，高程精度验证取每个控制点一定范围内点云，减少后续KD-Tree查询时间
           ccBBox clipRange(controlPoint[i] - clipRange, controlPoint[i] + clipRange);
           cloudTem = (cloud->crop(clipRange));
           for (int j = 0; j < cloudTem->size(); j++)
           {
               temp.x = cloudTem->getPoint(j)->x;
               temp.y = cloudTem->getPoint(j)->y;
               temp.z = cloudTem->getPoint(j)->z;
               temPointCloud->push_back(temp);
           }
           delete cloudTem;
           cloudTem = nullptr;
       }
       params.zTolerance = getCurrentZTolerance();
       params.matchingNeighborhood = getMatchingNeighborhood();
       computeHeightAccuracy(controlPoint, temPointCloud);
   }
   getCloudPos();
   getControlPos();
   //stop(true);
}

void ccPointAccuracyVerificationDlg::cancel()
{
	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		it.key()->enableGLTransformation(false);

	stop(false);
}


void ccPointAccuracyVerificationDlg::setIsDownPartHide(bool ishide)
{
	frame->setVisible(!ishide);
}

ccPointAccuracyVerificationDlg::CurrentCompareType ccPointAccuracyVerificationDlg::getCurrentCompareType()
{
	return m_CurrentCompareType;
}


double ccPointAccuracyVerificationDlg::getCurrentZTolerance()
{
	return doubleSpinBox->value();
}



double ccPointAccuracyVerificationDlg::getMatchingNeighborhood()
{
	return doubleSpinBox_Neighborhood->value();
}


ccPointCloud ccPointAccuracyVerificationDlg::getControlPoint()
{
	return m_refPoints;
}


ccPointCloud ccPointAccuracyVerificationDlg::getPickPoints()
{
	return m_alignedPoints;
}

bool ccPointAccuracyVerificationDlg::getTxtFileData(QString filename, std::vector<CCVector3d> & data)
{
    Path = filename;
	data.clear();
	QFile file(filename);

	if (!file.open(QIODevice::ReadOnly))
	{
		return false;
	}

	QTextStream * read = new QTextStream(&file);
	QStringList Data = read->readAll().split("\n", QString::SkipEmptyParts);   //每行以\n区分

	for (int i = 0; i < Data.count(); ++i)
	{
		QString str = Data.at(i);
		str.replace(QRegExp("[\\s]+"), " ");
		QStringList strLine = str.split(" ");     //一行中的单元格以，区分
		if (strLine.size() < 3)
		{
			strLine = str.split(",");
			if (strLine.size() < 3)
			{
				break;
			}
		}
		else
		{
			if (strLine.size() == 4 && strLine.at(3).isEmpty())
			{
				strLine.pop_back();
			}
		}
		if (strLine.size() == 3)
		{
			strLine.push_front("none");
		}
		CCVector3d pos(0, 0, 0);
		bool success = true;
		pos.x = strLine.at(1).toDouble(&success);
		if (!success)
		{
			break;
		}
		pos.y = strLine.at(2).toDouble(&success);
		if (!success)
		{
			break;
		}
		pos.z = strLine.at(3).toDouble(&success);
		if (!success)
		{
			break;
		}
		data.push_back(pos);
	}
	file.close();
	return true;
}

void ccPointAccuracyVerificationDlg::setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
	action->setIconPixmap(normalPix, clickedPix, disabledPix);
}

void ccPointAccuracyVerificationDlg::upSource()
{
	int currentRow = alignedPointsTableWidget->currentRow();
	if (currentRow == 0)
	{
		return;
	}
	SwitchRowData(alignedPointsTableWidget, currentRow - 1, currentRow);
	alignedPointsTableWidget->setCurrentCell(currentRow - 1, 1);
}

void ccPointAccuracyVerificationDlg::downSource()
{
	int currentRow = alignedPointsTableWidget->currentRow();
	if (currentRow == (alignedPointsTableWidget->rowCount() - 1))
	{
		return;
	}
	SwitchRowData(alignedPointsTableWidget, currentRow, currentRow + 1);
	alignedPointsTableWidget->setCurrentCell(currentRow + 1, 1);
}

void ccPointAccuracyVerificationDlg::deleteSource()
{
	QList<QTableWidgetItem *> lists = alignedPointsTableWidget->selectedItems();
	std::set<int> rows;
	for (auto curList : lists)
	{
		rows.insert(curList->row());

	}
	for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
	{
		removeAlignedPoint(*iter, false);
	}
	if (alignedPointsTableWidget->rowCount() == 0)
	{
		toolButton_delete_2->setEnabled(false);
	}
	if (alignedPointsTableWidget->rowCount() == 1)
	{
		toolButton_up_2->setEnabled(false);
		toolButton_down_2->setEnabled(false);
	}
}


void ccPointAccuracyVerificationDlg::upTarget()
{
	int currentRow = refPointsTableWidget->currentRow();
	if (currentRow == 0)
	{
		return;
	}
	SwitchRowData(refPointsTableWidget, currentRow - 1, currentRow);
	refPointsTableWidget->setCurrentCell(currentRow - 1, 1);
}

void ccPointAccuracyVerificationDlg::downTarget()
{
	int currentRow = refPointsTableWidget->currentRow();
	if (currentRow == (refPointsTableWidget->rowCount() - 1))
	{
		return;
	}
	SwitchRowData(refPointsTableWidget, currentRow, currentRow + 1);
	refPointsTableWidget->setCurrentCell(currentRow + 1, 1);
}

void ccPointAccuracyVerificationDlg::deleteTarget()
{
	QList<QTableWidgetItem *> lists = refPointsTableWidget->selectedItems();
	std::set<int> rows;
	for (auto curList : lists)
	{
		rows.insert(curList->row());

	}
	for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
	{
		removeRefPoint(*iter, false);
	}
	if (refPointsTableWidget->rowCount() == 0)
	{
		toolButton_delete->setEnabled(false);
	}
	if (refPointsTableWidget->rowCount() == 1)
	{
		toolButton_up->setEnabled(false);
		toolButton_down->setEnabled(false);
	}
}

void ccPointAccuracyVerificationDlg::SwitchRowData(QTableWidget * table, int uprow, int downrow)
{
	double currentX = table->item(uprow, 1)->text().toDouble();
	double currentY = table->item(uprow, 2)->text().toDouble();
	double currentZ = table->item(uprow, 3)->text().toDouble();
	double downX = table->item(downrow, 1)->text().toDouble();
	double downY = table->item(downrow, 2)->text().toDouble();
	double downZ = table->item(downrow, 3)->text().toDouble();
	if (table->objectName() == "alignedPointsTableWidget")
	{
		m_alignedPoints.setPointVec(uprow, CCVector3(downX, downY, downZ));
		m_alignedPoints.setPointVec(downrow, CCVector3(currentX, currentY, currentZ));
	}

	if (table->objectName() == "refPointsTableWidget")
	{
		m_refPoints.setPointVec(uprow, CCVector3(downX, downY, downZ));
		m_refPoints.setPointVec(downrow, CCVector3(currentX, currentY, currentZ));
	}
	table->item(uprow, 1)->setText(QString::number(downX, 'f', 3));
	table->item(uprow, 2)->setText(QString::number(downY, 'f', 3));
	table->item(uprow, 3)->setText(QString::number(downZ, 'f', 3));
	table->item(downrow, 1)->setText(QString::number(currentX, 'f', 3));
	table->item(downrow, 2)->setText(QString::number(currentY, 'f', 3));
	table->item(downrow, 3)->setText(QString::number(currentZ, 'f', 3));
	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

//计算高程和平面位置精度
bool ccPointAccuracyVerificationDlg::computeHeightAndPlaneAccuracy(std::vector<CCVector3> pickPoint, std::vector<CCVector3> controlPoint)
{
    std::vector<PointPositionAccuracyType> deltaXYLZ(controlPoint.size());        //存储每个点的对应误差Delta
    std::vector<PointPositionAccuracyType> accuracyXYLZ(4);  //存储XYLZ每个维度上的中误差，平均误差，最大值，最小值
    CCVector3 pp(0, 0, 0);
    CCVector3 cp(0, 0, 0);
    if (controlPoint.size() == pickPoint.size())
    {
        for (int i = 0; i < pickPoint.size(); ++i)
        {
            pp = pickPoint[i];
            cp = controlPoint[i];
            //判断选点是否有效，防止误选；匹配邻域/Z容差
            if ((pp-cp).normd() > params.matchingNeighborhood || (pp - cp).z > params.zTolerance)
            {
                deltaXYLZ[i].deltaX = MAXDELTA;
                deltaXYLZ[i].deltaY = MAXDELTA;
                deltaXYLZ[i].deltaL = MAXDELTA;
                deltaXYLZ[i].deltaZ = MAXDELTA;
            }
            else
            {
                deltaXYLZ[i].deltaX = (pp - cp).x;
                deltaXYLZ[i].deltaY = (pp - cp).y;
                deltaXYLZ[i].deltaL = sqrt(pow((pp - cp).x, 2) + pow((pp - cp).y, 2));
                deltaXYLZ[i].deltaZ = (pp - cp).z;
            }
        }

        //中误差
        float temX = 0;
        float temY = 0;
        float temL = 0;
        float temZ = 0;
        int numCounter = 0;
        for (int i = 0; i < deltaXYLZ.size(); ++i)
        {
            if (deltaXYLZ[i].deltaX != MAXDELTA)
            {
                temX += pow(deltaXYLZ[i].deltaX, 2);
                temY += pow(deltaXYLZ[i].deltaY, 2);
                temL += (pow(deltaXYLZ[i].deltaX, 2) + pow(deltaXYLZ[i].deltaY, 2));
                temZ += pow(deltaXYLZ[i].deltaZ, 2);
                numCounter++;
            }
        }
        if (numCounter != 0)
        {
            accuracyXYLZ[0].MeanSquareError = sqrt(temX / numCounter);
            accuracyXYLZ[1].MeanSquareError = sqrt(temY / numCounter);
            accuracyXYLZ[2].MeanSquareError = sqrt(temL / numCounter);
            accuracyXYLZ[3].MeanSquareError = sqrt(temZ / numCounter);
        }
        else
        {
            accuracyXYLZ[0].MeanSquareError = ISNULL;
            accuracyXYLZ[1].MeanSquareError = ISNULL;
            accuracyXYLZ[2].MeanSquareError = ISNULL;
            accuracyXYLZ[3].MeanSquareError = ISNULL;
        }
        
        //平均误差
        temX = 0;
        temY = 0;
        temL = 0;
        temZ = 0;
        numCounter = 0;
        for (int i = 0; i < deltaXYLZ.size(); ++i)
        {
            if (deltaXYLZ[i].deltaX != MAXDELTA)
            {
                temX += abs(deltaXYLZ[i].deltaX);
                temY += abs(deltaXYLZ[i].deltaY);
                temL += deltaXYLZ[i].deltaL;
                temZ += abs(deltaXYLZ[i].deltaZ);
                numCounter++;
            }
        }
        if (numCounter != 0)
        {
            accuracyXYLZ[0].averageError = temX / numCounter;
            accuracyXYLZ[1].averageError = temY / numCounter;
            accuracyXYLZ[2].averageError = temL / numCounter;
            accuracyXYLZ[3].averageError = temZ / numCounter;
        }
        else
        {
            accuracyXYLZ[0].averageError = ISNULL;
            accuracyXYLZ[1].averageError = ISNULL;
            accuracyXYLZ[2].averageError = ISNULL;
            accuracyXYLZ[3].averageError = ISNULL;
        }

        //获取最大最小值
        std::vector<float>xx, yy, ll, zz;
        for (int i = 0; i < deltaXYLZ.size(); ++i)
        {
            if (deltaXYLZ[i].deltaX != MAXDELTA)
            {
                xx.push_back(abs(deltaXYLZ[i].deltaX));
                yy.push_back(abs(deltaXYLZ[i].deltaY));
                ll.push_back(deltaXYLZ[i].deltaL);
                zz.push_back(abs(deltaXYLZ[i].deltaZ));
            }
        }
        if (xx.size() == 0)
        {
            accuracyXYLZ[0].maxValue = ISNULL;
            accuracyXYLZ[1].maxValue = ISNULL;
            accuracyXYLZ[2].maxValue = ISNULL;
            accuracyXYLZ[3].maxValue = ISNULL;
            accuracyXYLZ[0].minValue = ISNULL;
            accuracyXYLZ[1].minValue = ISNULL;
            accuracyXYLZ[2].minValue = ISNULL;
            accuracyXYLZ[3].minValue = ISNULL;
        }
        else
        {
            accuracyXYLZ[0].maxValue = *max_element(xx.begin(), xx.end());
            accuracyXYLZ[1].maxValue = *max_element(yy.begin(), yy.end());
            accuracyXYLZ[2].maxValue = *max_element(ll.begin(), ll.end());
            accuracyXYLZ[3].maxValue = *max_element(zz.begin(), zz.end());
            accuracyXYLZ[0].minValue = *min_element(xx.begin(), xx.end());
            accuracyXYLZ[1].minValue = *min_element(yy.begin(), yy.end());
            accuracyXYLZ[2].minValue = *min_element(ll.begin(), ll.end());
            accuracyXYLZ[3].minValue = *min_element(zz.begin(), zz.end());
        }
        //获取数据
        planeDelta = deltaXYLZ;
        planeAverageDelta = accuracyXYLZ;
        return true;
    }
    else
    {
        return false;
    }
}

//计算高程位置精度
bool ccPointAccuracyVerificationDlg::computeHeightAccuracy(std::vector<CCVector3> controlPoint, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<CCVector3> pickPoint(controlPoint.size());
    /*m_XYZAccuracy.resize(3);*/
    //自动计算，输入控制点，查找点云中对应邻域点
    //先取出每个控制点一定范围内的点云，再建立KD树
    if (cloud->points.size() != 0)
    {
        pickPoint.resize(controlPoint.size());
        // 创建KD树
        pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
        kdtree.setInputCloud(cloud);
        pcl::PointXYZ searchPoint;
        std::vector<int> ptIdxByRadius;   //存储近邻索引
        std::vector<float> ptRadius;      //存储近邻对应距离的平方
        for (int i = 0; i < controlPoint.size(); i++)
        {
            searchPoint.x = controlPoint[i].x;
            searchPoint.y = controlPoint[i].y;
            searchPoint.z = controlPoint[i].z;
            if (kdtree.radiusSearch(searchPoint, params.matchingNeighborhood, ptIdxByRadius, ptRadius) > 0)
            {
                float temXX = 0, temYY = 0, temZZ = 0;
                for (int j = 0; j < ptIdxByRadius.size(); ++j)
                {
                    temXX += cloud->points[ptIdxByRadius[j]].x / ptIdxByRadius.size();
                    temYY += cloud->points[ptIdxByRadius[j]].y / ptIdxByRadius.size();
                    temZZ += cloud->points[ptIdxByRadius[j]].z / ptIdxByRadius.size();
                }
                pickPoint[i].x = temXX;
                pickPoint[i].y = temYY;
                pickPoint[i].z = temZZ;
            }
            else
            {
                //匹配邻域内未找到点，标记
                pickPoint[i].x = MAXDELTA;
                pickPoint[i].y = MAXDELTA;
                pickPoint[i].z = MAXDELTA;
            }
        }
        //精度指标
        std::vector<PointPositionAccuracyType> deltaZ(controlPoint.size()); ////存储每个点的对应误差Delta 以及每个对应点计算出的点云坐标值
        std::vector<PointPositionAccuracyType> accuracyZ(1);                     //存储Z维度上的中误差，平均误差，最大值，最小值

        //计算Delta
        for (int i = 0; i < controlPoint.size(); ++i)
        {
            //Z容差
            if (pickPoint[i].z != MAXDELTA)
            {
                if (pickPoint[i].z - controlPoint[i].z <= params.zTolerance)
                {
                    deltaZ[i].deltaZ = pickPoint[i].z - controlPoint[i].z;
                }
                else
                {
                    deltaZ[i].deltaZ = MAXDELTA;
                }
                CCVector3d Pi = m_refPoints.toGlobal3d<PointCoordinateType>(pickPoint[i]);
                deltaZ[i].pointCloudZCoord = Pi.z;
            }
            else
            {
                deltaZ[i].deltaZ = MAXDELTA;
                //No point found in the neighborhood
                deltaZ[i].pointCloudZCoord = MAXDELTA;
            }
            
        }

        //中误差
        float temZ = 0;
        int numCounter = 0;
        for (int i = 0; i < deltaZ.size(); ++i)
        {
            if (deltaZ[i].deltaZ != MAXDELTA)
            {
                temZ += pow(deltaZ[i].deltaZ, 2);
                numCounter++;
            }
        }
        if (numCounter != 0)
        {
            accuracyZ[0].MeanSquareError = sqrt(temZ / numCounter);
        }
        else
        {
            accuracyZ[0].MeanSquareError = ISNULL;
        }

        //平均误差
        temZ = 0;
        numCounter = 0;
        for (int i = 0; i < deltaZ.size(); ++i)
        {
            if (deltaZ[i].deltaZ != MAXDELTA)
            {
                temZ += abs(deltaZ[i].deltaZ);
                numCounter++;
            }
        }
        if (numCounter != 0)
        {
            accuracyZ[0].averageError = temZ / numCounter;
        }
        else
        {
            accuracyZ[0].averageError = ISNULL; 
        }

        //获取最大最小值
        std::vector<float>zz;
        for (int i = 0; i < deltaZ.size(); ++i)
        {
            if (deltaZ[i].deltaZ != MAXDELTA)
            {
                zz.push_back(abs(deltaZ[i].deltaZ));
            }
        }
        if (zz.size() == 0)
        {
            accuracyZ[0].maxValue = ISNULL;
            accuracyZ[0].minValue = ISNULL;
        }
        else
        {
            accuracyZ[0].maxValue = *max_element(zz.begin(), zz.end());
            accuracyZ[0].minValue = *min_element(zz.begin(), zz.end());
        }
        precisionDelta = deltaZ;
        precisionAverageDelta = accuracyZ;
        return true;
    }
    else
    {
        return false;
    }
}

void ccPointAccuracyVerificationDlg::setAccuracyInformation()
{
	//[!].显示等待框
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(MainWindow::TheInstance());
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Calculating... Please wait.", nullptr));
	validToolButton->setEnabled(false);
	//[!].并发执行线程中执行
	QFuture<void> fu = QtConcurrent::run([=]() {
		apply();
	});
	while (!fu.isFinished()) {
		QThread::msleep(20);
		QCoreApplication::processEvents();
	}
	WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
	validToolButton->setEnabled(true);
	emit openPutDialog();
}
void ccPointAccuracyVerificationDlg::getCloudPos()
{
    std::vector<CCVector3d> pos;
    for (int i = 0; i < refPointsTableWidget->rowCount(); i++)
    {
        CCVector3d point;
        point.x = refPointsTableWidget->item(i, 1)->text().toDouble();
        point.y = refPointsTableWidget->item(i, 2)->text().toDouble();
        point.z = refPointsTableWidget->item(i, 3)->text().toDouble();
        pos.push_back(point);
    }
    cloudpos = pos;
}

void ccPointAccuracyVerificationDlg::getControlPos()
{
    std::vector<CCVector3d> pos;
    for (int i = 0; i < alignedPointsTableWidget->rowCount(); i++)
    {
        CCVector3d point;
        point.x = alignedPointsTableWidget->item(i, 1)->text().toDouble();
        point.y = alignedPointsTableWidget->item(i, 2)->text().toDouble();
        point.z = alignedPointsTableWidget->item(i, 3)->text().toDouble();
        pos.push_back(point);
    }
    controlpos = pos;
}