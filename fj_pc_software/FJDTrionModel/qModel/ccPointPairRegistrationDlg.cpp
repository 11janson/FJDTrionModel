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

#include <ccPointPairRegistrationDlg.h>

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
#include <PCLConv.h>
#include <cc2sm.h>
#include <sm2cc.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/io.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "libs/cloudcompareutils/exportdatatofile.h"
#include <mutex>
//Qt
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QToolButton>
#include <QSettings>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QAxObject>
#include <QComboBox >
#include "cloudcompareutils/icore.h"
#include <framelessmessagebox.h>
#include "fjitemdoublespinboxdelegate.h"
#include "framelessfiledialog.h"
#include <QStandardPaths>
#include "FJStyleManager.h"
#include "pointpairregistrationinformationdlg.h"
#include "ccDBRoot.h"
#include <QCheckBox>
#include "ccClipBox.h"
#include <QDebug>
#include <QThread>
#include <QFuture>
#include <QButtonGroup>
#include <QtConcurrent>
#include <QRadioButton>
#include"cloudcomparewidgets/metahubframelesswaitdialog.h"
#include "checkboxheaderview.h"
#include "FileIOFilter.h"
#include "libs/cloudcompareutils/exportdatatofile.h"

//default position of each columns in the aligned and ref. table widgets
static const int XYZ_COL_INDEX = 2;
static const int RMS_COL_INDEX = 5;

//minimum number of pairs to let the user click on the align button
static const unsigned MIN_PAIRS_COUNT = 3;

static bool isHasHistory(ccPointPairRegistrationDlg::HistoryData historydata, int uuid) {
    return (historydata.AlignedPointsUuid == uuid || historydata.RefPointsUuid == uuid);
}

ccPointPairRegistrationDlg::ccPointPairRegistrationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent/*=nullptr*/)
    : ccOverlayDialog(parent)
    , m_alignedPoints("aligned points")
    , m_refPoints("reference points")
    , m_paused(false)
    , m_pickingHub(pickingHub)
    , m_app(app)
{
    assert(m_pickingHub);

    setupUi(this);
    QButtonGroup * radioButtonGroup = new QButtonGroup(this);
    radioButtonGroup->addButton(radioButton);
    QButtonGroup * radioButtonGroup1 = new QButtonGroup(this);
    radioButtonGroup1->addButton(coordinateButton);
    radioButtonGroup1->addButton(constituencyButton);
   

    m_paperRadius = doubleSpinBox_paper->value();
    m_ballRadius = doubleSpinBox->value();
    connect(toolButton_inporttransform, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::getTransformationMatrixFromTxt);
    connect(radioButton, &QRadioButton::toggled, this, &ccPointPairRegistrationDlg::slotRadioButtonClicked);
    connect(alignToolButton, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::align);
    connect(resetToolButton, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::reset);
    connect(validToolButton, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::apply);
    connect(cancelToolButton, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::cancel);
    connect(pushButton, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::slotOpenSavePath);
    connect(toolButton, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::cancel);
    connect(doubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double d) {m_ballRadius = d; });
    connect(doubleSpinBox_paper, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double d) {m_paperRadius = d; });
    //[!]分离实体八叉树
    connect(this, &ccPointPairRegistrationDlg::signalOctreeDeletion, this, &ccPointPairRegistrationDlg::slotOctreeDeletion, static_cast<Qt::ConnectionType>(Qt::QueuedConnection | Qt::BlockingQueuedConnection));

    connect(alignedPointsTableWidget, &QTableWidget::itemSelectionChanged, this, &ccPointPairRegistrationDlg::slotalignedItemSelectionChanged);
    connect(refPointsTableWidget, &QTableWidget::itemSelectionChanged, this, &ccPointPairRegistrationDlg::slotRefItemSelectionChanged);
    connect(alignedPointsTableWidget, &QTableWidget::itemChanged, this, &ccPointPairRegistrationDlg::slotaligneditemChanged);
    connect(refPointsTableWidget, &QTableWidget::itemChanged, this, &ccPointPairRegistrationDlg::slotRefitemChanged);

    connect(toolButton_pick, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::slotPickToolButtonClicked);
    connect(toolButton_ball, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::slotBallToolButtonClicked);
    connect(toolButton_paper, &QPushButton::clicked, this, &ccPointPairRegistrationDlg::slotPaperToolButtonClicked);

    connect(toolButton_add, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::addSource);
    connect(toolButton_import, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::importSource);
    connect(toolButton_up, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::upSource);
    connect(toolButton_down, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::downSource);
    connect(toolButton_delete, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::deleteSource);
    connect(toolButton_add_2, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::addTarget);
    connect(toolButton_import_2, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::importTarget);
    connect(toolButton_up_2, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::upTarget);
    connect(toolButton_down_2, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::downTarget);
    connect(toolButton_delete_2, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::deleteTarget);
    connect(toolButton_exportsource, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::exportSource);
    connect(toolButton_exporttarget, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::exportTarget);
    connect(icpButton, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::getIcpSetting);
    connect(doubleSpinBox_19, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccPointPairRegistrationDlg::setConstituencyParameter);
    connect(doubleSpinBox_21, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccPointPairRegistrationDlg::setConstituencyParameter);
    connect(doubleSpinBox_20, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ccPointPairRegistrationDlg::setConstituencyParameter);
    connect(this, &ccOverlayDialog::shortcutDoubleBondTriggered, this, &ccPointPairRegistrationDlg::onShortcutDoubleBondTriggered);
    connect(checkBox, &QCheckBox::toggled, this, [=]() {
        if (!checkBox->isChecked())
        {
            pushButton->setEnabled(false);
            lineEdit->setEnabled(false);
        }
        else
        {
            pushButton->setEnabled(true);
            lineEdit->setEnabled(true);

        }
    });
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
    delegate->setEnableEmptyStr(true);
    alignedPointsTableWidget->setItemDelegateForColumn(2, delegate);
    alignedPointsTableWidget->setItemDelegateForColumn(3, delegate);
    alignedPointsTableWidget->setItemDelegateForColumn(4, delegate);
    refPointsTableWidget->setItemDelegateForColumn(2, delegate);
    refPointsTableWidget->setItemDelegateForColumn(3, delegate);
    refPointsTableWidget->setItemDelegateForColumn(4, delegate);


    m_alignedCheckBoxHeader = new CheckBoxHeaderView(0, QPoint(9, 9), QSize(16, 16), Qt::Horizontal, alignedPointsTableWidget);
    alignedPointsTableWidget->setColumnCount(6);
    alignedPointsTableWidget->setHorizontalHeaderLabels(QStringList() << "      " << "ID" << "X(E)" << "Y(N)" << "Z(H)" << tr("Residual"));
    alignedPointsTableWidget->setHorizontalHeader(m_alignedCheckBoxHeader);

    m_refCheckBoxHeader = new CheckBoxHeaderView(0, QPoint(9, 9), QSize(16, 16), Qt::Horizontal, refPointsTableWidget);
    refPointsTableWidget->setColumnCount(6);
    refPointsTableWidget->setHorizontalHeaderLabels(QStringList() << "      " << "ID" << "X(E)" << "Y(N)" << "Z(H)" << tr("Residual"));
    refPointsTableWidget->setHorizontalHeader(m_refCheckBoxHeader);

    connect(m_alignedCheckBoxHeader, &CheckBoxHeaderView::signalCheckStateChanged, this, &ccPointPairRegistrationDlg::slotAlignedHeadviewerStateChanged);
    connect(m_refCheckBoxHeader, &CheckBoxHeaderView::signalCheckStateChanged, this, &ccPointPairRegistrationDlg::slotRefHeadviewerStateChanged);

    alignedPointsTableWidget->setSelectionBehavior(QTableWidget::SelectRows);
    refPointsTableWidget->setSelectionBehavior(QTableWidget::SelectRows);
    alignedPointsTableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    alignedPointsTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    alignedPointsTableWidget->verticalHeader()->setVisible(false);
    alignedPointsTableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
    refPointsTableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    refPointsTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    refPointsTableWidget->verticalHeader()->setVisible(false);
    refPointsTableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background-color:#484848;border: 1px solid #989898;height:32px;font-size:13px;color:#DDDDDD;}");
    //alignedPointsTableWidget->setStyleSheet("QTableCornerButton::section{background-color:#ff0000;border: 1px solid #dddddd;}");
    QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
    toolButton->setIcon(closeicon);
    alignedPointsTableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    refPointsTableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    alignedPointsTableWidget->setColumnWidth(0, 40);
    refPointsTableWidget->setColumnWidth(0, 40);
    alignedPointsTableWidget->setColumnWidth(1, 52);
    refPointsTableWidget->setColumnWidth(1, 52);
    for (int curColumn = 2; curColumn < 6; curColumn++)
    {
        alignedPointsTableWidget->setColumnWidth(curColumn, 141);
        refPointsTableWidget->setColumnWidth(curColumn, 141);
    }
    for (int i = 2; i <= 18; i++)
    {
        QString objname = "doubleSpinBox_" + QString::number(i);
        QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
        if (foundBox)
        {
            connect(foundBox, &QDoubleSpinBox::textChanged,this,&ccPointPairRegistrationDlg::checkMatrixData);
            connect(foundBox, &QDoubleSpinBox::editingFinished, this, &ccPointPairRegistrationDlg::checkMatrixData);
        }
    }
    stackedWidget->setCurrentWidget(page);

    //[!]切换配准模式
    connect(constituencyButton, &QRadioButton::toggled, this, &ccPointPairRegistrationDlg::switchingRegistrationMethods);
    
    //[!]设置选区方式
    connect(RadioModelButton, QOverload<bool>::of(&QPushButton::clicked), this, &ccPointPairRegistrationDlg::slotBoxSelectionOperation);

    connect(MultipleChoiceButton, QOverload<bool>::of(&QPushButton::clicked), this, &ccPointPairRegistrationDlg::slotMultipleSelectionOperation);

    connect(comboBox_pointsourceClone, QOverload<int>::of(&QComboBox::currentIndexChanged), [=]() {
    
        int index = comboBox_pointsourceClone->currentIndex();
        bool isVisibles = false;
        ccHObject::Container pVec = MainWindow::TheInstance()->getSelectedEntities();
        if (pVec.size() != 2) {
            return;
        }
        ccHObject*pObject0 = pVec[0];
        ccHObject*pObject1= pVec[1];
        index == 1 ? pObject0->setVisible(false) : pObject0->setVisible(true);
        index == 0 ? pObject1->setVisible(false) : pObject1->setVisible(true);


        if (m_associatedWin)
        {
            m_associatedWin->redraw();
        }
        
    });
    connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [=]() {
        if (MainWindow::TheInstance()->getSelectedEntities().size() != 2 || comboBox->count() != 2) {
            return;
        }
        int index = comboBox->currentIndex();
    });
}
void ccPointPairRegistrationDlg::onShortcutDoubleBondTriggered(int key, Qt::KeyboardModifiers modifiers)
{
    switch (key + modifiers)
    {
    case Qt::Key_Z + Qt::ControlModifier:
        break;
    default:
        break;
    }
}

void ccPointPairRegistrationDlg::slotBoxSelectionOperation(bool state)
{
    removeOverriddenShortcut(Qt::Key_Control);
    removeOverriddenShortcut(Qt::Key_Z);
}

void ccPointPairRegistrationDlg::slotMultipleSelectionOperation(bool state)
{
    addOverriddenShortcut(Qt::Key_Control);
    addOverriddenShortcut(Qt::Key_Z);


}
void ccPointPairRegistrationDlg::switchingRegistrationMethods()
{
    if (coordinateButton->isChecked())
    {
        resetToolButton->setEnabled(false);
        stackedWidget->setCurrentWidget(page);
        MainWindow::TheInstance()->setUseOrthographic(false);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
        }
        m_currentPickMode = PICKPOINT;
        icpButton->setVisible(true);
        setFixedHeight(820);
        for (EntityContext& ec : m_alignedEntities){
            if (m_bCurrentModelCombination)
            {
                ec.entity->m_displayBoundingBox = false;
            }
            else
            {
                ec.entity->m_displayBoundingBox = true;
            }
        }
        for (EntityContext& ec : m_referenceEntities) {
            if (m_bCurrentModelCombination)
            {
                ec.entity->m_displayBoundingBox = false;
            }
            else
            {
                ec.entity->m_displayBoundingBox = true;
            }
        }
    }
    else
    {
        clearChangeRegistrationType();

        checkBox_align->setChecked(true);
        checkBox_ref->setChecked(true);

        stackedWidget->setCurrentWidget(page_3);
        

        if (MainWindow::TheInstance()->getSelectedEntities().size() != 2 && comboBox_pointsource->count() !=2){
            qDebug() << "error cloud size";
            return;
        }
        clearLabel(false);
        MainWindow::TheInstance()->setUseOrthographic(true);
        resetToolButton->setEnabled(true);
        icpButton->setVisible(false);
        setFixedHeight(563);
    }
}

void ccPointPairRegistrationDlg::clearLabel(bool state)
{
    int childNum = m_refPoints.getChildrenNumber();
    for (int j = childNum - 1; j >= 0; j--)
    {
        cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_refPoints.getChild(j));
        if (label)
        {
            m_refPoints.removeChild(label);
        }
    }
    childNum = m_alignedPoints.getChildrenNumber();
    for (int j = childNum - 1; j >= 0; j--)
    {
        cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_alignedPoints.getChild(j));
        if (label)
        {
            m_alignedPoints.removeChild(label);
        }
    }

    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}

ccPointPairRegistrationDlg::EntityContext::EntityContext(ccHObject* ent)
    : entity(ent)
    , originalDisplay(entity ? entity->getDisplay() : nullptr)
    , wasVisible(entity ? entity->isVisible() : false)
    , wasEnabled(entity ? entity->isEnabled() : false)
    , wasSelected(entity ? entity->isSelected() : false)
{
}

void ccPointPairRegistrationDlg::EntityContext::restore()
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

void ccPointPairRegistrationDlg::EntityContexts::fill(const ccHObject::Container& entities)
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

void ccPointPairRegistrationDlg::clear()
{
    alignToolButton->setEnabled(false);
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
    clearMatrixData();
}

bool ccPointPairRegistrationDlg::linkWith(ccGLWindow* win)
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

    if (m_associatedWin)
    {
        m_associatedWin->addToOwnDB(&m_alignedPoints);
        m_associatedWin->addToOwnDB(&m_refPoints);
    }

    return true;
}

bool ccPointPairRegistrationDlg::start()
{
    if (!m_pickingHub->addListener(this, true))
    {
        return false;
    }
    lineEdit->clear();
    for (int curColumn = 2; curColumn < 6; curColumn++)
    {
        alignedPointsTableWidget->setColumnWidth(curColumn, 141);
        refPointsTableWidget->setColumnWidth(curColumn, 141);
    }
    m_Scalefactor = -1;
    m_TransformationMatrix.clear();
    m_isAlreadyAlign = false;
    checkBox_align->disconnect();
    checkBox_ref->disconnect();
    checkBox_align->setChecked(true);
    checkBox_ref->setChecked(true);
    checkBox->setChecked(false);
    pushButton->setEnabled(false);
    lineEdit->setEnabled(false);
    m_icpValueVector.push_back(1);
    m_icpValueVector.push_back(0.0001);
    connect(checkBox_align, QOverload<int>::of(&QCheckBox::stateChanged), [=](int index) {
        bool isCloudVisible = (index == 0 ? false : true);
        for (EntityContext& ec : m_alignedEntities)
        {
            if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
            {
                ec.entity->setVisible(isCloudVisible);
                if (m_bCurrentModelCombination)
                {
                    ec.entity->m_displayBoundingBox = false;
                }
                else
                {
                    ec.entity->m_displayBoundingBox = isCloudVisible;
                }
            }
        }
        int alienalbelnum = m_alignedPoints.getChildrenNumber();
        for (int i = 0; i < alienalbelnum; i++)
        {
            cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_alignedPoints.getChild(i));
            if (label)
            {
                label->setVisible(isCloudVisible);
            }
        }
        if (m_associatedWin)
        {
            m_associatedWin->redraw(false);
        }
    });
    connect(checkBox_ref, QOverload<int>::of(&QCheckBox::stateChanged), [=](int index) {
        bool isCloudVisible = (index == 0 ? false : true);
        for (EntityContext& ec : m_referenceEntities)
        {
            if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
            {
                ec.entity->setVisible(isCloudVisible);
                if (m_bCurrentModelCombination)
                {
                    ec.entity->m_displayBoundingBox = false;
                }
                else
                {
                    ec.entity->m_displayBoundingBox = isCloudVisible;
                }
            }
        }
        int refalbelnum = m_refPoints.getChildrenNumber();
        for (int i = 0; i < refalbelnum; i++)
        {
            cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_refPoints.getChild(i));
            if (label)
            {
                label->setVisible(isCloudVisible);
            }
        }
        if (m_associatedWin)
        {
            m_associatedWin->redraw(false);
        }
    });
    radioButton->setChecked(true);
    loadHistoryData();
    setActionIcon(toolButton_pick, "clickPointIconClicked", "clickPointIconClicked", "clickPointIconDisable");
    setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
    setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
    doubleSpinBox_paper->setEnabled(false);
    doubleSpinBox->setEnabled(false);
    resetToolButton->setEnabled(false);
    setActionIcon(toolButton_add, "addpointdatanormal", "addpointdataclicked", "addpointdatadisabled");
    setActionIcon(toolButton_import, "importdatanormal", "importdataclicked", "importdatadisabled");
    setActionIcon(toolButton_down, "movedownnormal", "movedownclicked", "movedowndisabled");
    setActionIcon(toolButton_up, "moveupnormal", "moveupclicked", "moveupdisabled");
    setActionIcon(toolButton_delete, "deleterowsnormal", "deleterowsclicked", "deleterowsdisabled");
    setActionIcon(toolButton_add_2, "addpointdatanormal", "addpointdataclicked", "addpointdatadisabled");
    setActionIcon(toolButton_import_2, "importdatanormal", "importdataclicked", "importdatadisabled");
    setActionIcon(toolButton_down_2, "movedownnormal", "movedownclicked", "movedowndisabled");
    setActionIcon(toolButton_up_2, "moveupnormal", "moveupclicked", "moveupdisabled");
    setActionIcon(toolButton_delete_2, "deleterowsnormal", "deleterowsclicked", "deleterowsdisabled");
    setActionIcon(toolButton_exportsource, "exportdatanormal", "exportdataclicked", "exportdatadisabled");
    setActionIcon(toolButton_exporttarget, "exportdatanormal", "exportdataclicked", "exportdatadisabled");
    setActionIcon(toolButton_inporttransform, "importdatanormal", "importdataclicked", "importdatadisabled");
    setPickPointMode(true);
    if (m_associatedWin)
    {
        m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
    }
    m_currentPickMode = PICKPOINT;

    for (ccHObject *entity : MainWindow::TheInstance()->getSelectedEntities())
    {
        comboBox_pointsourceClone->addItem(entity->getName());
        comboBox->addItem(entity->getName());
    }
    QString str = comboBox_pointsourceClone->itemText(0) + " && " + comboBox_pointsourceClone->itemText(1);
    comboBox_pointsourceClone->addItem(str);
    comboBox_pointsourceClone->setCurrentIndex(2);
    comboBox->setCurrentIndex(0);

    return ccOverlayDialog::start();
}

void ccPointPairRegistrationDlg::stop(bool state)
{
    m_icpValueVector.clear();
    reset();
    coordinateButton->setChecked(true);
    comboBox_pointsourceClone->clear();
    comboBox->clear();

    for (EntityContext& ec : m_alignedEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            ec.entity->setVisible(true);
            if (!m_bCurrentModelCombination)
                ec.entity->m_displayBoundingBox = true;
        }
    }for (EntityContext& ec : m_referenceEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            ec.entity->setVisible(true);
            if (!m_bCurrentModelCombination)
                ec.entity->m_displayBoundingBox = true;
        }
    }
    ccOverlayDialog::stop(state);
}

static void SetEnabled_recursive(ccHObject* ent)
{
    assert(ent);
    ent->setEnabled(true);
    if (ent->getParent())
        SetEnabled_recursive(ent->getParent());
}


bool ccPointPairRegistrationDlg::init(ccGLWindow* win,
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
    if (alignedEntities.size() > 0 && referenceEntities->size() > 0)
    {
        comboBox_pointsource->disconnect();
        comboBox_pointtarget->disconnect();
        comboBox_pointsource->clear();
        comboBox_pointtarget->clear();
        comboBox_pointsource->addItem(alignedEntities[0]->getName());
        comboBox_pointsource->addItem((*referenceEntities)[0]->getName());
        comboBox_pointsource->setCurrentIndex(0);
        comboBox_pointtarget->addItem(alignedEntities[0]->getName());
        comboBox_pointtarget->addItem((*referenceEntities)[0]->getName());
        comboBox_pointtarget->setCurrentIndex(1);
        comboBox_pointtarget->setEnabled(false);
        connect(comboBox_pointsource, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ccPointPairRegistrationDlg::slotCloudSourseChanged);
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
        //for (unsigned i = 0; i < aligned->getChildrenNumber(); ++i)
        //{
        //	cc2DLabel* label = ccHObjectCaster::To2DLabel(aligned->getChild(i));
        //	if (label && label->size() == 1)
        //	{
        //		m_alignedEntities.insert(label, EntityContext(label));
        //		m_associatedWin->addToOwnDB(label);
        //		label->setDisplay(m_associatedWin);
        //		label->setVisible(true);
        //	}
        //}
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
        reference->setDisplay(m_associatedWin);
        reference->setVisible(true);
        SetEnabled_recursive(reference);
    }


    m_associatedWin->showMaximized();

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

void ccPointPairRegistrationDlg::pause(bool state)
{
    m_paused = state;
    setDisabled(state);
}

bool ccPointPairRegistrationDlg::convertToSphereCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius)
{
    sphereRadius = 0;
    if (!entity
        || !entity->isKindOf(CC_TYPES::POINT_CLOUD)) //only works with cloud right now
    {
        //nothing to do
        return true;
    }

    //we'll now try to detect the sphere
    double searchRadius = m_ballRadius;
    double maxRMSPercentage = m_RMSvalue / 100.0;
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
    //ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
    assert(cloud);

    //crop points inside a box centered on the current point
    ccBBox box;
    box.add((P - CCVector3d(1, 1, 1)*searchRadius).toPC());
    box.add((P + CCVector3d(1, 1, 1)*searchRadius).toPC());
    CCCoreLib::ReferenceCloud* part = cloud->crop(box, true);

    /* ccPointCloud* temp = cloud->partialClone(part);
     MainWindow::TheInstance()->addToDB(temp);*/

    bool success = false;
    if (part && part->size() > 16)
    {
        PointCoordinateType radius;
        CCVector3 C;
        double rms;
        ccProgressDialog pDlg(true, this);
        pDlg.setMethodTitle(tr("Detect sphere"));
        pDlg.setInfo(tr("Detecting..."));
        //first roughly search for the sphere
        if (CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(part, 0.5, C, radius, rms, &pDlg, 0.9) == CCCoreLib::GeometricalAnalysisTools::NoError)
        {
            if (radius / searchRadius < 0.3 || radius / searchRadius > 2.0)
            {
                ccLog::Warning(QString("[ccPointPairRegistrationDlg] Detected sphere radius (%1) is too far from search radius!").arg(radius));
            }
            else
            {
                //now look again (more precisely)
                {
                    delete part;
                    box.clear();
                    box.add(C - CCVector3(1, 1, 1)*radius*static_cast<PointCoordinateType>(1.05)); //add 5%
                    box.add(C + CCVector3(1, 1, 1)*radius*static_cast<PointCoordinateType>(1.05)); //add 5%
                    part = cloud->crop(box, true);
                    if (part && part->size() > 16)
                        CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(part, 0.5, C, radius, rms, &pDlg, 0.99);
                }
                ccLog::Print(QString("[ccPointPairRegistrationDlg] Detected sphere radius = %1 (rms = %2)").arg(radius).arg(rms));
                if (radius / searchRadius < 0.3 || radius / searchRadius > 2.0)
                {
                    ccLog::Warning("[ccPointPairRegistrationDlg] Sphere radius is too far from search radius!");
                }
                else if (rms / searchRadius >= maxRMSPercentage)
                {
                    ccLog::Warning("[ccPointPairRegistrationDlg] RMS is too high!");
                }
                else
                {
                    sphereRadius = radius;
                    P = C;
                    success = true;
                }
            }
        }
        else
        {
            ccLog::Warning("[ccPointPairRegistrationDlg] Failed to fit a sphere around the picked point!");
        }
    }
    else
    {
        //not enough memory? No points inside the 
        ccLog::Warning("[ccPointPairRegistrationDlg] Failed to crop points around the picked point?!");
    }

    if (part)
        delete part;

    return success;
}

void ccPointPairRegistrationDlg::onItemPicked(const PickedItem& pi)
{
    if (stackedWidget->currentIndex() != 0)
    {
        return;
    }
    if (m_currentPickMode == NOPICK)
    {
        return;
    }

    if (!m_associatedWin)
        return;

    //no point picking when paused!
    if (m_paused)
        return;

    if (!pi.entity)
        return;

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
            //assert(false);
        }
        return;
    }
    m_associatedWin->redraw();
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


void ccPointPairRegistrationDlg::addPointToTable(QTableWidget* tableWidget,const CCVector3d& P,QString pointName,bool checked)
{
    if (!tableWidget)
        return;
    if (m_isAlreadyAlign)
    {
        reset();
    }
    //add corresponding row in table
    int rowIndex = tableWidget->rowCount();
    tableWidget->setRowCount(rowIndex +1);

    QCheckBox* box = new QCheckBox();
    box->setFixedSize(16, 16);
    QWidget *widget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setMargin(0);//一定要有
    layout->addWidget(box);
    layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
    widget->setLayout(layout);
    box->setChecked(checked);
    tableWidget->setCellWidget(rowIndex, 0, widget);
    if (tableWidget->objectName() == "alignedPointsTableWidget")
    {
        connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotAlignTableCheckStateChanged);
        updateTableWidgetCheckState(alignedPointsTableWidget);
    }
    else
    {
        connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotRefTableCheckStateChanged);
        updateTableWidgetCheckState(refPointsTableWidget);
    }


    //tableWidget->setVerticalHeaderItem(rowIndex, new QTableWidgetItem(pointName));
    QTableWidgetItem* labelitem = new QTableWidgetItem(pointName);
    //labelitem->setFlags(labelitem->flags() & ~Qt::ItemIsEditable);
    labelitem->setTextAlignment(Qt::AlignCenter);
    tableWidget->setItem(rowIndex, 1, labelitem);
    //add point coordinates
    for (int d = 0; d < 3; ++d)
    {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setData(Qt::EditRole, QString::number(P.u[d], 'f', 3));
        item->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(rowIndex, XYZ_COL_INDEX + d, item);
        tableWidget->setRowHeight(rowIndex, 32);
    }
    QTableWidgetItem* itemError = new QTableWidgetItem();
    itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
    itemError->setTextAlignment(Qt::AlignCenter);
    tableWidget->setItem(rowIndex, XYZ_COL_INDEX + 3, itemError);
    int lastcurrentRow = tableWidget->rowCount();
    tableWidget->setCurrentCell(lastcurrentRow - 1, 1);
    if (tableWidget->rowCount() > 4)
    {
        tableWidget->setColumnWidth(5, 128);
    }
}

bool ccPointPairRegistrationDlg::addAlignedPoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=false*/, bool isaddname/* = false*/, QString idname/* = nullptr*/)
{
    //if the input point is not shifted, we shift it to the aligned coordinate system
    assert(entity == nullptr || m_alignedEntities.contains(entity));
    ccGenericPointCloud* cloud = entity ? ccHObjectCaster::ToGenericPointCloud(entity) : nullptr;

    PointCoordinateType sphereRadius = 0;
    if (m_currentPickMode == PICKBALL && entity && !convertToSphereCenter(Pin, entity, sphereRadius))
        return false;

    //janson
    if (m_currentPickMode == PICKPAPER && entity && !convertToIntensityPaperCenter(Pin, entity, sphereRadius))
        return false;

    //transform the input point in the 'global world' by default
    if (cloud)
        Pin = cloud->toGlobal3d<double>(Pin);

    m_isTableDataChangeLock = true;
    int rowcount = alignedPointsTableWidget->rowCount();
    int currentemptyindex = -1;
    getFirstEmptyIndex(currentemptyindex, alignedPointsTableWidget);
    if (currentemptyindex < 0)
    {
        addPointToTable(alignedPointsTableWidget, Pin, "A" + QString::number(rowcount + 1), true);
    }
    else
    {
        alignedPointsTableWidget->item(currentemptyindex, 2)->setData(Qt::EditRole, QString::number(Pin.x, 'f', 3));
        alignedPointsTableWidget->item(currentemptyindex, 3)->setData(Qt::EditRole, QString::number(Pin.y, 'f', 3));
        alignedPointsTableWidget->item(currentemptyindex, 4)->setData(Qt::EditRole, QString::number(Pin.z, 'f', 3));
        slotalignedItemSelectionChanged();
    }
    m_isTableDataChangeLock = false;
    updateAlignPlontAndLabel();

    onPointCountChanged();

    return true;
}

bool ccPointPairRegistrationDlg::addReferencePoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=true*/, bool isaddname/* = false*/, QString idname/* = nullptr*/)
{
    assert(entity == nullptr || m_referenceEntities.contains(entity));

    ccGenericPointCloud* cloud = entity ? ccHObjectCaster::ToGenericPointCloud(entity) : nullptr;

    //first point?
    //if (m_refPoints.size() == 0)
    //{
    //    if (m_referenceEntities.size() > 0)
    //    {
    //        auto cloudref = (m_referenceEntities.begin()).key();
    //        if (cloudref)
    //        {
    //            ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloudref);
    //            if (shiftcloud)
    //            {
    //                m_refPoints.copyGlobalShiftAndScale(*shiftcloud);
    //            }
    //        }
    //    }
    //    else
    //    {
    //        auto cloudref = (m_alignedEntities.begin()).key();
    //        if (cloudref)
    //        {
    //            ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloudref);
    //            if (shiftcloud)
    //            {
    //                m_refPoints.copyGlobalShiftAndScale(*shiftcloud);
    //            }
    //        }
    //    }
    //}

    PointCoordinateType sphereRadius = 0;
    if (m_currentPickMode == PICKBALL && entity && !convertToSphereCenter(Pin, entity, sphereRadius))
        return false;

    //janson
    if (m_currentPickMode == PICKPAPER && entity && !convertToIntensityPaperCenter(Pin, entity, sphereRadius))
        return false;
    //transform the input point in the 'global world' by default
    if (cloud)
    {
        Pin = cloud->toGlobal3d<double>(Pin);
    }

    m_isTableDataChangeLock = true;
    int rowcount = refPointsTableWidget->rowCount();
    int currentemptyindex = -1;
    getFirstEmptyIndex(currentemptyindex, refPointsTableWidget);
    if (currentemptyindex < 0)
    {
        addPointToTable(refPointsTableWidget, Pin, "R" + QString::number(rowcount + 1), true);
    }
    else
    {
        refPointsTableWidget->item(currentemptyindex,2)->setData(Qt::EditRole, QString::number(Pin.x, 'f', 3));
        refPointsTableWidget->item(currentemptyindex, 3)->setData(Qt::EditRole, QString::number(Pin.y, 'f', 3));
        refPointsTableWidget->item(currentemptyindex, 4)->setData(Qt::EditRole, QString::number(Pin.z, 'f', 3));
        slotRefItemSelectionChanged();
    }
    m_isTableDataChangeLock = false;
    updateRefPlontAndLabel();
    onPointCountChanged();

    return true;
}


void ccPointPairRegistrationDlg::changeLabelName(ccPointCloud entity, int index, QString newName)
{
    int childNum = entity.getChildrenNumber();
    if (childNum > index)
    {
        ccHObject* child = entity.getChild(index);
        if (child)
        {
            if (child->isKindOf(CC_TYPES::LABEL_2D))
            {
                cc2DLabel* label = static_cast<cc2DLabel*>(child);
                label->setName(newName);
            }
        }
    }

}

void ccPointPairRegistrationDlg::slotOctreeDeletion(bool isCoarse)
{
    for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
    {
        ccMainAppInterface::ccHObjectContext objContext;
        int index = 0;
        if (it.key()->getParent())
        {
            index = it.key()->getParent()->getChildIndex(it.key());
            if (index < 0)
            {
                index = 0;
            }
        }
        if (m_app)
            objContext = m_app->removeObjectTemporarilyFromDBTree(it.key());
        it.key()->applyGLTransformation_recursive();
        if (m_referenceEntities.size() > 0 && isCoarse)
        {
            it.key()->setName(it.key()->getName() + "-Registration");
        }
        if (m_app)
        {
            m_app->putObjectIntoDBTreeByIndex(it.key(), objContext, index);
        }
    }
}


void ccPointPairRegistrationDlg::importSource()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("PointPairRegistration");
    QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "(*.xlsx *.csv *.txt)");
    if (selectedFile.isEmpty())
    {
        settings.endGroup();
        return;
    }
    settings.setValue("filedoc", QFileInfo(selectedFile).absolutePath());
    std::vector<CCVector3d>  data;
    std::vector<QString> indexNames;
    bool readSuccess = true;
    if (QFileInfo(selectedFile).suffix().contains("xlsx") || QFileInfo(selectedFile).suffix().contains("csv"))
    {
        readSuccess = getExcelFileData(selectedFile, data, indexNames);
    }
    else
    {
        readSuccess = getTxtAndCsvFileData(selectedFile, data, indexNames);
    }
    if (readSuccess && data.size() != 0)
    {
        m_isTableDataChangeLock = true;
        int currentRow = alignedPointsTableWidget->rowCount();
        for (int row = currentRow - 1; row >= 0; row--)
        {
            alignedPointsTableWidget->removeRow(row);
        }
        for (int i = 0; i < data.size(); i++)
        {
            QString idname = "A" + QString::number(i+1);
            if (indexNames.size() > i)
            {
                idname = indexNames[i];
            }
            addPointToTable(alignedPointsTableWidget, data[i], idname, true);
        }
        m_isTableDataChangeLock = false;
        updateAlignPlontAndLabel();
        onPointCountChanged();
        emit showMessage(tr("Import File Success!"));
    }
    else
    {
        emit showMessage(tr("Import failed"));
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
    }
    settings.endGroup();
}

void ccPointPairRegistrationDlg::addSource()
{
    m_isTableDataChangeLock = true;
    int rownumindex = alignedPointsTableWidget->rowCount();
    alignedPointsTableWidget->setRowCount(alignedPointsTableWidget->rowCount() + 1);
    QCheckBox* box = new QCheckBox();
    box->setFixedSize(16, 16);
    QWidget *widget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setMargin(0);//一定要有
    layout->addWidget(box);
    layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
    widget->setLayout(layout);
    box->setChecked(true);
    connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotAlignTableCheckStateChanged);
    alignedPointsTableWidget->setCellWidget(rownumindex, 0, widget);
    updateTableWidgetCheckState(alignedPointsTableWidget);
    QTableWidgetItem* labelitem = new QTableWidgetItem("A" + QString::number(rownumindex + 1));
    labelitem->setTextAlignment(Qt::AlignCenter);
    alignedPointsTableWidget->setItem(rownumindex, 1, labelitem);

    for (int d = 0; d < 3; ++d)
    {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setData(Qt::EditRole, "");
        item->setTextAlignment(Qt::AlignCenter);
        alignedPointsTableWidget->setItem(rownumindex, XYZ_COL_INDEX + d, item);
        alignedPointsTableWidget->setRowHeight(rownumindex, 32);
    }
    QTableWidgetItem* itemError = new QTableWidgetItem();
    itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
    itemError->setTextAlignment(Qt::AlignCenter);
    alignedPointsTableWidget->setItem(rownumindex, XYZ_COL_INDEX + 3, itemError);
    int lastcurrentRow = alignedPointsTableWidget->rowCount();
    if (alignedPointsTableWidget->rowCount() > 4)
    {
        alignedPointsTableWidget->setColumnWidth(5, 128);
    }
    m_isTableDataChangeLock = false;
    onPointCountChanged();
    alignedPointsTableWidget->setCurrentItem(alignedPointsTableWidget->item(rownumindex,1));
}

void ccPointPairRegistrationDlg::upSource()
{
    int currentRow = alignedPointsTableWidget->currentRow();
    if (currentRow == 0)
    {
        return;
    }
    SwitchRowData(alignedPointsTableWidget, currentRow - 1, currentRow);
    alignedPointsTableWidget->setCurrentCell(currentRow - 1, 1);
    updateAlignPlontAndLabel();
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::downSource()
{
    int currentRow = alignedPointsTableWidget->currentRow();
    if (currentRow == (alignedPointsTableWidget->rowCount() - 1))
    {
        return;
    }
    SwitchRowData(alignedPointsTableWidget, currentRow, currentRow + 1);
    alignedPointsTableWidget->setCurrentCell(currentRow + 1, 1);
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::deleteSource()
{
    QList<QTableWidgetItem *> lists = alignedPointsTableWidget->selectedItems();
    std::set<int> rows;
    bool isNeedShowQuestion = false;
    for (auto curList : lists)
    {
        rows.insert(curList->row());
        if (curList->row() < refPointsTableWidget->rowCount() && (!refPointsTableWidget->item(curList->row(), 2)->text().isEmpty() && !refPointsTableWidget->item(curList->row(), 3)->text().isEmpty() && !refPointsTableWidget->item(curList->row(), 4)->text().isEmpty()))
        {
            isNeedShowQuestion = true;
        }
    }
    bool isDeleteDual = false;
    if (isNeedShowQuestion)
    {
        isDeleteDual = CS::Widgets::FramelessMessageBox::question(this, tr("Delete dual point"), tr("Do you want to delete the equivalent reference point as well?"), QMessageBox::Yes | QMessageBox::No,
            QMessageBox::Yes) == QMessageBox::Yes;
    }
    for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
    {
        alignedPointsTableWidget->item(*iter, 2)->setText("");
        alignedPointsTableWidget->item(*iter, 3)->setText("");
        alignedPointsTableWidget->item(*iter, 4)->setText("");
    }


    if (isDeleteDual)
    {
        for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
        {
            if (*iter < refPointsTableWidget->rowCount())
            {
                refPointsTableWidget->item(*iter, 2)->setText("");
                refPointsTableWidget->item(*iter, 3)->setText("");
                refPointsTableWidget->item(*iter, 4)->setText("");
            }
        }
        updateRefPlontAndLabel();
    }
    toolButton_delete->setEnabled(false);
    if (m_isAlreadyAlign)
    {
        reset();
    }
    updateAlignPlontAndLabel();
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::deleteTarget()
{
    QList<QTableWidgetItem *> lists = refPointsTableWidget->selectedItems();
    std::set<int> rows;
    bool isNeedShowQuestion = false;
    for (auto curList : lists)
    {
        rows.insert(curList->row());
        if (curList->row() < alignedPointsTableWidget->rowCount() && (!refPointsTableWidget->item(curList->row(), 2)->text().isEmpty() && !refPointsTableWidget->item(curList->row(), 3)->text().isEmpty() && !refPointsTableWidget->item(curList->row(), 4)->text().isEmpty()))
        {
            isNeedShowQuestion = true;
        }
    }
    bool isDeleteDual = false;
    if (isNeedShowQuestion)
    {
        isDeleteDual = CS::Widgets::FramelessMessageBox::question(this, tr("Delete dual point"), tr("Do you want to delete the equivalent aligned point as well?"), QMessageBox::Yes | QMessageBox::No,
            QMessageBox::Yes) == QMessageBox::Yes;
    }
    for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
    {
        refPointsTableWidget->item(*iter, 2)->setText("");
        refPointsTableWidget->item(*iter, 3)->setText("");
        refPointsTableWidget->item(*iter, 4)->setText("");
    }

    if (isDeleteDual)
    {
        for (auto iter = rows.rbegin(); iter != rows.rend(); iter++)
        {
            if (*iter < alignedPointsTableWidget->rowCount())
            {
                alignedPointsTableWidget->item(*iter, 2)->setText("");
                alignedPointsTableWidget->item(*iter, 3)->setText("");
                alignedPointsTableWidget->item(*iter, 4)->setText("");
            }
        }
        updateAlignPlontAndLabel();
    }

    toolButton_delete_2->setEnabled(false);
    if (m_isAlreadyAlign)
    {
        reset();
    }
    updateRefPlontAndLabel();
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::addTarget()
{
    m_isTableDataChangeLock = true;
    int rownumindex = refPointsTableWidget->rowCount();
    refPointsTableWidget->setRowCount(refPointsTableWidget->rowCount() + 1);
    QCheckBox* box = new QCheckBox();
    box->setFixedSize(16, 16);
    QWidget *widget = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setMargin(0);//一定要有
    layout->addWidget(box);
    layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
    widget->setLayout(layout);
    box->setChecked(true);
    connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotRefTableCheckStateChanged);
    refPointsTableWidget->setCellWidget(rownumindex, 0, widget);
    updateTableWidgetCheckState(refPointsTableWidget);
    QTableWidgetItem* labelitem = new QTableWidgetItem("R" + QString::number(rownumindex + 1));
    labelitem->setTextAlignment(Qt::AlignCenter);
    refPointsTableWidget->setItem(rownumindex, 1, labelitem);

    for (int d = 0; d < 3; ++d)
    {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setData(Qt::EditRole, "");
        item->setTextAlignment(Qt::AlignCenter);
        refPointsTableWidget->setItem(rownumindex, XYZ_COL_INDEX + d, item);
        refPointsTableWidget->setRowHeight(rownumindex, 32);
    }
    QTableWidgetItem* itemError = new QTableWidgetItem();
    itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
    itemError->setTextAlignment(Qt::AlignCenter);
    refPointsTableWidget->setItem(rownumindex, XYZ_COL_INDEX + 3, itemError);
    int lastcurrentRow = refPointsTableWidget->rowCount();
    if (refPointsTableWidget->rowCount() > 4)
    {
        refPointsTableWidget->setColumnWidth(5, 128);
    }
    m_isTableDataChangeLock = false;
    onPointCountChanged();
    refPointsTableWidget->setCurrentItem(refPointsTableWidget->item(rownumindex, 1));
}

void ccPointPairRegistrationDlg::upTarget()
{
    int currentRow = refPointsTableWidget->currentRow();
    if (currentRow == 0)
    {
        return;
    }
    SwitchRowData(refPointsTableWidget, currentRow - 1, currentRow);
    refPointsTableWidget->setCurrentCell(currentRow - 1, 1);
    updateRefPlontAndLabel();
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::downTarget()
{
    int currentRow = refPointsTableWidget->currentRow();
    if (currentRow == (refPointsTableWidget->rowCount() - 1))
    {
        return;
    }
    SwitchRowData(refPointsTableWidget, currentRow, currentRow + 1);
    refPointsTableWidget->setCurrentCell(currentRow + 1, 1);
    updateRefPlontAndLabel();
    onPointCountChanged();
}

void ccPointPairRegistrationDlg::importTarget()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("PointPairRegistration");
    QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "(*.xlsx *.csv *.txt)");
    if (selectedFile.isEmpty())
    {
        settings.endGroup();
        return;
    }
    settings.setValue("filedoc", QFileInfo(selectedFile).absolutePath());
    std::vector<CCVector3d>  data;
    std::vector<QString> indexNames;
    bool readSuccess = true;
    if (QFileInfo(selectedFile).suffix().contains("xlsx") ||QFileInfo(selectedFile).suffix().contains("csv"))
    {
        readSuccess = getExcelFileData(selectedFile, data, indexNames);
    }
    else
    {
        readSuccess = getTxtAndCsvFileData(selectedFile, data, indexNames);
    }
    if (readSuccess && data.size() != 0)
    {
        m_isTableDataChangeLock = true;
        int currentRow = refPointsTableWidget->rowCount();
        for (int row = currentRow - 1; row >= 0; row--)
        {
            refPointsTableWidget->removeRow(row);
        }
        for (int i = 0; i < data.size(); i++)
        {
            QString idname = "R" + QString::number(i + 1);
            if (indexNames.size() > i)
            {
                idname = indexNames[i];
            }
            addPointToTable(refPointsTableWidget, data[i], idname, true);
        }
        m_isTableDataChangeLock = false;
        updateRefPlontAndLabel();
        onPointCountChanged();
    }
    else
    {
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
    }
    settings.endGroup();
}



void ccPointPairRegistrationDlg::SwitchRowData(QTableWidget * table, int uprow, int downrow)
{
    m_isTableDataChangeLock = true;
    QString upName = table->item(uprow, 1)->text();
    QString downName = table->item(downrow, 1)->text();
    QString currentX = table->item(uprow, 2)->text();
    QString currentY = table->item(uprow, 3)->text();
    QString currentZ = table->item(uprow, 4)->text();
    QString downX = table->item(downrow, 2)->text();
    QString downY = table->item(downrow, 3)->text();
    QString downZ = table->item(downrow, 4)->text();

    table->item(uprow, 1)->setText(downName);
    table->item(uprow, 2)->setText(downX);
    table->item(uprow, 3)->setText(downY);
    table->item(uprow, 4)->setText(downZ);
    table->item(downrow, 1)->setText(upName);
    table->item(downrow, 2)->setText(currentX);
    table->item(downrow, 3)->setText(currentY);
    table->item(downrow, 4)->setText(currentZ);

    m_isTableDataChangeLock = false;
    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}

bool ccPointPairRegistrationDlg::getExcelFileData(QString filename, std::vector<CCVector3d> & data, std::vector<QString> & indexNames)
{
    data.clear();
    indexNames.clear();
    QXlsx::Document xlsx(filename);
    QStringList sheetNames = xlsx.sheetNames();
    if (sheetNames.size() == 0)
    {
        return false;
    }
    xlsx.selectSheet(sheetNames[0]);
    int iRows = xlsx.dimension().lastRow();
    int iCols = xlsx.dimension().lastColumn();
    if (iRows < 1 || iCols < 3)
    {
        return false;
    }

    int startRow = 1;
    int startColumn = 1;
    //可以转换为double
    auto isDoubleValid = [](QVariant  curObj) -> bool {
        bool issuccess = true;
        double curDblData = curObj.toDouble(&issuccess);
        return issuccess;
        };
    //可以转换为double但不是int
    auto isDoubleNoIntValid = [](QVariant  curObj) -> bool {
        bool issuccess = true;
        double curDblData = curObj.toDouble(&issuccess);
        if (issuccess)
        {
            int curIntData = curObj.toInt();
            if ((curIntData - curDblData) == 0)
            {
                issuccess = false;
            }
        }
        return issuccess;
        };

    QVariant var13 = xlsx.cellAt(1, 3)->value();
    if (!isDoubleValid(var13))
    {
        startRow = 2;
    }
    if (iRows > 2)
    {
        QVariant var22 = xlsx.cellAt(2, 2)->value();
        if (!isDoubleNoIntValid(var22) && iCols >= 5)
        {
            startColumn = 3;
        }
        else
        {
            QVariant var21 = xlsx.cellAt(2, 1)->value();
            if (!isDoubleNoIntValid(var21) && iCols >= 4)
            {
                startColumn = 2;
            }
        }
    }
    if (iCols == 3)
    {
        startColumn = 1;
    }


    for (int i = startRow; i <= iRows; ++i)
    {
        CCVector3d pos(0, 0, 0);
        bool success = true;
        QVariant varx = xlsx.cellAt(i, startColumn)->value();
        pos.x = varx.toDouble(&success);
        if (!success)
        {
            break;
        }

        QVariant vary = xlsx.cellAt(i, startColumn+1)->value();
        pos.y = vary.toDouble(&success);
        if (!success)
        {
            break;
        }
        QVariant varz = xlsx.cellAt(i, startColumn + 2)->value();
        pos.z = varz.toDouble(&success);
        if (!success)
        {
            break;
        }
        if (startColumn == 2)
        {
            QVariant varIndexName = xlsx.cellAt(i, 1)->value();
            indexNames.push_back(varIndexName.toString());
        }
        data.push_back(pos);
    }
    return true;
}

bool ccPointPairRegistrationDlg::getTxtAndCsvFileData(QString filename, std::vector<CCVector3d> & data, std::vector<QString> & indexNames)
{
    data.clear();
    indexNames.clear();
    QFile file(filename);

    if (!file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    QTextStream * read = new QTextStream(&file);
    QStringList Data = read->readAll().split("\n", QString::SkipEmptyParts);   //每行以\n区分
    bool hasIndexColumn = true;
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
                hasIndexColumn = false;
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
            hasIndexColumn = false;
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
        indexNames.push_back(strLine.at(0));
    }
    if (!hasIndexColumn)
    {
        indexNames.clear();
    }
    file.close();
    return true;
}

void ccPointPairRegistrationDlg::setCoordinateConversionStyle(bool isshow)
{
    if (isshow)
    {
        //转换
        frame->setVisible(false);
        stackedWidget_2->setCurrentWidget(stackedWidget_2Page1);
        widget_3->setVisible(true);
        frame_2->setVisible(false);
        label_2->setText(tr("Reference point"));
        label_2->setVisible(true);
        label_4->setText(tr("Coordinate transformation"));
        label_3->setText(tr("Alignment point"));
        label_5->setVisible(false);
        icpButton->setVisible(false);
        label_RMSE->setVisible(true);
        setFixedHeight(820);
    }
    else
    {
        //配准
        label_RMSE->setVisible(false);
        frame->setVisible(true);
        stackedWidget_2->setCurrentWidget(stackedWidget_2Page2);
        widget_3->setVisible(false);
        frame_2->setVisible(true);
        label_2->setVisible(false);
        label_4->setText(tr("Point Cloud Registration"));
        label_3->setText(tr("Control point selection method"));
        label_5->setVisible(true);
        icpButton->setVisible(true);
        setFixedHeight(797);
    }
}

void ccPointPairRegistrationDlg::setPickPointMode(bool isShow)
{
    ccGui::ParamStruct m_parameters = ccGui::Parameters();
    m_parameters.m_bShowCapacityPoint = isShow;
    ccGui::Set(m_parameters);
}

void ccPointPairRegistrationDlg::setActionIcon(MetahubToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
    action->setIconPixmap(normalPix, clickedPix, disabledPix);
}


bool ccPointPairRegistrationDlg::convertToIntensityPaperCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius)
{
    sphereRadius = 0;

    if (!entity
        || !entity->isKindOf(CC_TYPES::POINT_CLOUD)) //only works with cloud right now
    {
        //nothing to do
        return true;
    }

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
    assert(cloud);

    CCCoreLib::ScalarField* intensitySF = nullptr;
    for (int i = 0; i < cloud->getNumberOfScalarFields(); i++)
    {
        QString name = cloud->getScalarFieldName(i);
        if (name.compare("intensity", Qt::CaseInsensitive) == 0)
            intensitySF = cloud->getScalarField(i);
    }

    QString sfName = cloud ? QString::fromLocal8Bit(cloud->getCurrentDisplayedScalarField()->getName()) : "";

    if (!intensitySF || !sfName.contains("intensity", Qt::CaseInsensitive))
        return false;

    ////we'll now try to detect the sphere
    //double searchRadius = radiusDoubleSpinBox->value();
    //double maxRMSPercentage = maxRmsSpinBox->value() / 100.0;
    //we'll now try to detect the sphere
    double searchRadius = m_paperRadius;
    double maxRMSPercentage = m_RMSvalue / 100.0;
    //ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
    //assert(cloud);

    //crop points inside a box centered on the current point
    ccBBox box;
    box.add((P - CCVector3d(1, 1, 1)*searchRadius).toPC());
    box.add((P + CCVector3d(1, 1, 1)*searchRadius).toPC());
    CCCoreLib::ReferenceCloud* part = cloud->crop(box, true);

    bool success = false;

    CCCoreLib::ReferenceCloud blackCloud_index(cloud);
    for (unsigned j = 0; j < part->size(); j++)
    {
        //const ScalarType val = part->getPointScalarValue(j);
        const ScalarType val = intensitySF->getValue(part->getPointGlobalIndex(j));
        //if (val >= 0 && val <= 5)
        if (val >= 0 && val <= 10)
        {
            blackCloud_index.addPointIndex(part->getPointGlobalIndex(j));
        }
    }

    ccPointCloud* blackCloud = cloud->partialClone(&blackCloud_index);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud = cc2smReader(blackCloud).getRawXYZ(false);
    if (!xyzCloud)
    {
        return false;
    }

    ////拟合标靶纸平面
    //std::vector<int> inliers;
    //pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(xyzCloud));
    //pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    //ransac.setDistanceThreshold(0.05);
    //ransac.computeModel();
    //ransac.getInliers(inliers);

    //std::vector<int> tmp;
    //ransac.getModel(tmp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::copyPointCloud(*xyzCloud, inliers, *planeCloud);

    *planeCloud = *xyzCloud;

    if (planeCloud->empty())
        return false;


    ////temp
    //PCLCloud cloudTemp;
    //TO_PCL_CLOUD(*planeCloud, cloudTemp);
    //ccPointCloud* temp0 = pcl2cc::Convert(cloudTemp);
    //temp0->setColor(ccColor::red);
    //temp0->showColors(true);
    //temp0->setPointSize(3.0);
    //MainWindow::TheInstance()->addToDB(temp0, false, false, true, true);


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(planeCloud);


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//创建欧式聚类分割对象
    ec.setClusterTolerance(0.03); //设置近邻搜索的搜索半径
    ec.setMinClusterSize(20); //设置最小聚类尺寸
    ec.setMaxClusterSize(20000);//修改最大聚类大小为20000
    ec.setSearchMethod(tree);
    ec.setInputCloud(planeCloud);
    ec.extract(cluster_indices);

    if (cluster_indices.size() > 0)
    {
        pcl::PointIndices result_indices;
        Eigen::Vector4f centroid;

        if (cluster_indices.size() == 1)
        {
            result_indices = cluster_indices[0];
        }
        else if (cluster_indices.size() > 1)
        {
            double num0 = cluster_indices[0].indices.size();
            double num1 = cluster_indices[1].indices.size();
            if ((num0 / num1 > 0.99) && (num0 / num1 < 2.0))
            {
                result_indices = cluster_indices[0];
                result_indices.indices.insert(result_indices.indices.end(), cluster_indices[1].indices.begin(), cluster_indices[1].indices.end());
            }
            else
            {
                result_indices = cluster_indices[0];
            }


        }

        pcl::compute3DCentroid(*planeCloud, result_indices, centroid);
        P = CCVector3d(centroid[0], centroid[1], centroid[2]);

        ////通过提取中心与包围盒中心距离判断结果是否正常
        //pcl::PointIndicesConstPtr inliers_ptr(new pcl::PointIndices(result_indices));
        //pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        //feature_extractor.setInputCloud(planeCloud);
        //feature_extractor.setIndices(inliers_ptr);
        //feature_extractor.compute();

        //pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
        //Eigen::Matrix3f rotation_matrix_OBB;
        //feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotation_matrix_OBB);
        //CCVector3 p2p = CCVector3(position_OBB.x - centroid[0], position_OBB.y - centroid[1], position_OBB.z - centroid[2]);
        //float dis = p2p.norm();


        success = true;


        ////temp
        //ccPointCloud* clusterCloud = new ccPointCloud;
        //pcl::PointXYZ temp;
        //for (int i = 0; i < result_indices.indices.size(); i++)
        //{
        //    temp = planeCloud->at(result_indices.indices.at(i));
        //    clusterCloud->addPoint(CCVector3(temp.x, temp.y, temp.z));
        //}

        //clusterCloud->setColor(ccColor::green);
        //clusterCloud->showColors(true);
        //clusterCloud->setPointSize(6.0);

        //MainWindow::TheInstance()->addToDB(clusterCloud, false, false, true, true);
    }


    if (part)
        delete part;

    return success;

}

//点云拼接函数
void ccPointPairRegistrationDlg::pointCloudSplicing()
{
    std::vector<ccGenericPointCloud*> clouds;
    //目标点云实体数量
    //拼接到目标点云上
    if (!m_alignedEntities.isEmpty())
    {
        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            ccGenericPointCloud* alignedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
            clouds.push_back(alignedCloud);
        }
    }

    //判断源点云实体 or 虚拟点
    if (!m_referenceEntities.isEmpty())
    {
        for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
        {
            ccGenericPointCloud* referencedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
            clouds.push_back(referencedCloud);
        }
    }

    //点云实体数量是否大于1，需要融合？ 等于1不需要融合
    if (clouds.size() > 1)
    {
        ccPointCloud* firstCloud = nullptr;
        ccMainAppInterface::ccHObjectContext firstCloudContext;
        CCCoreLib::ScalarField* ocIndexSF = nullptr;
        size_t cloudIndex = 0;

        for (size_t i = 0; i < clouds.size(); ++i)
        {
            ccPointCloud* pc = ccHObjectCaster::ToPointCloud(clouds[i]);
            if (!firstCloud)
            {
                firstCloud = pc;
            }
            else
            {
                unsigned countBefore = firstCloud->size();
                unsigned countAdded = pc->size();
                *firstCloud += pc;
                //融合成功
                if (firstCloud->size() == countBefore + countAdded)
                {
                    firstCloud->prepareDisplayForRefresh_recursive();
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
                    ccLog::Error(tr("Fusion failed! (not enough memory?)"));
                    break;
                }
                pc = nullptr;
            }
        }
        if (ocIndexSF)
        {
            ocIndexSF->computeMinAndMax();
            firstCloud->showSF(true);
        }
        ccDBRoot* dbRoot = MainWindow::TheInstance()->db();
        if (dbRoot)
        {
            if (m_referenceEntities.size() != 0)
            {
                ccHObject* toRemove = nullptr;
                //if the entity to remove is inside a group with a unique child, we can remove the group as well
                ccHObject* parent = m_referenceEntities.begin().key()->getParent();
                if (parent && parent->isA(CC_TYPES::HIERARCHY_OBJECT) && parent->getChildrenNumber() == 1)
                {
                    toRemove = parent;
                }
                else
                {
                    toRemove = m_referenceEntities.begin().key();
                }
                dbRoot->removeElement(toRemove);
                m_referenceEntities.clear();
            }
        }
    }
    else  //点云实体数量为1
    {
        ccLog::Error("No need to splicing.");
    }
}

void ccPointPairRegistrationDlg::exportSource()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("PointPairRegistration");
    QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this, tr("Select output file"), currentPath, "*.xlsx");
    if (outputFilename.isEmpty())
    {
        settings.endGroup();
        return;
    }
    saveExcelFileData(outputFilename, alignedPointsTableWidget);
    settings.setValue("filedoc", QFileInfo(outputFilename).absolutePath());
    settings.endGroup();
}

void ccPointPairRegistrationDlg::exportTarget()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("PointPairRegistration");
    QString currentPath = settings.value("filedoc", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString outputFilename = CS::Widgets::FramelessFileDialog::getSaveFileName(this, tr("Select output file"), currentPath, "*.xlsx");
    if (outputFilename.isEmpty())
    {
        settings.endGroup();
        return;
    }
    saveExcelFileData(outputFilename, refPointsTableWidget);
    settings.setValue("filedoc", QFileInfo(outputFilename).absolutePath());
    settings.endGroup();
}


void ccPointPairRegistrationDlg::getIcpSetting()
{
    icpRegistration  dialog(this);

    if (!m_icpValueVector.empty())
    {
        dialog.setIcpParameters(m_icpValueVector);
    }
    if (QDialog::Accepted == dialog.exec())
    {
        m_icpValueVector.clear();
        m_icpValueVector = dialog.getIpcParameters();
    }

;}


void ccPointPairRegistrationDlg::saveExcelFileData(QString filename, QTableWidget* table)
{
    filename = filename + ".xlsx";
    int currentRowCount = table->rowCount();
    std::vector<std::vector<QString>> tableData;
    for (int i = 0; i < currentRowCount; i++)
    {
        std::vector<QString> currowdata;
        QString datax = (table->item(i, 2)->text().isEmpty() ? QString::number(0, 'f', 3) : table->item(i, 2)->text());
        QString datay = (table->item(i, 3)->text().isEmpty() ? QString::number(0, 'f', 3) : table->item(i, 3)->text());
        QString dataz = (table->item(i, 4)->text().isEmpty() ? QString::number(0, 'f', 3) : table->item(i, 4)->text());
        currowdata.push_back(table->item(i, 1)->text());
        currowdata.push_back(datax);
        currowdata.push_back(datay);
        currowdata.push_back(dataz);
        tableData.push_back(currowdata);
    }


    QXlsx::Document xlsx;
    for (int i = 0; i < tableData.size(); i++) {
        for (int j = 0; j < 4; j++)
        {
            int AsciiNum = 65 + j;
            QString str = QString(QChar(AsciiNum)) + QString::number(i + 1);
            xlsx.write(str, tableData[i][j]);
        }
    }

    if (!xlsx.saveAs(filename))
    {
        return;
    }
    CS::Widgets::FramelessMessageBox::information(this, tr("Information"), tr("Save successfully."));
}

void ccPointPairRegistrationDlg::pretreat(pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_down)
{
    float LeafSize = 0.03;
    std::vector<int> indices_src;
    pcl::removeNaNFromPointCloud(*pcd_cloud, *pcd_cloud, indices_src);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(LeafSize, LeafSize, LeafSize);
    voxel_grid.setInputCloud(pcd_cloud);
    voxel_grid.filter(*pcd_down);
};

void ccPointPairRegistrationDlg::getOverlappedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, pcl::PointCloud<pcl::PointXYZ>& overlapped_cloud2)
{
    //double radius = 0.15;
    double radius = 0.5;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(radius);
    octree.setInputCloud(cloud1.makeShared());
    octree.addPointsFromInputCloud();
    for (size_t i = 0; i < cloud2.size(); ++i)
    {
        std::vector<int> indices;
        octree.voxelSearch(cloud2.points[i], indices);
        //pcl::PointCloud<pcl::PointXYZ> cloud_out;
        if (indices.size())
        {
            overlapped_cloud2.push_back(cloud2.points[i]);
        }
        else
        {
            qDebug() << "not have overlap point ";
            continue;
        }
    }
}

void ccPointPairRegistrationDlg::findRadiusClosedCloud(const pcl::PointCloud<pcl::PointXYZ >& indexPoints,
    const pcl::PointCloud<pcl::PointXYZ>& originCloud, pcl::PointCloud<pcl::PointXYZ>& targetCloud)
{
    double radius = 2.0;
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(originCloud.makeShared());

    pcl::Indices indices;
    std::vector<float> disVec;
    for (size_t i = 0; i < indexPoints.size(); ++i)
    {
        kdtree.radiusSearch(indexPoints.points[i], radius, indices, disVec);
        if (indices.size())
        {
            for (auto index : indices)
            {
                targetCloud.push_back(originCloud.points.at(index));
            }
        }
    }
}

void ccPointPairRegistrationDlg::removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_clean)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(10); //设置考虑查询点临近点数
    sor.setStddevMulThresh(1.0);//设置判断是否为离群点的阀值
    sor.filter(*cloud_clean);
}

std::mutex cloudLock;
void ccPointPairRegistrationDlg::fjCloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    int search_nub,
    float dis_p_th,
    float distance_th)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < input_cloud->size(); i++)  // for (auto point_i : *input_cloud)
    {
        pcl::PointXYZ point_i = input_cloud->points[i];
        pcl::Indices nn_indices;
        std::vector<float> nn_distances;
        if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z)) continue;
        if (kdtree.nearestKSearch(point_i, search_nub, nn_indices, nn_distances) > 0)
        {
            //double dis_min = pointDistance(point_i, point_i);
            double dis_min = sqrt((point_i.x - point_i.x)*(point_i.x - point_i.x) + (point_i.y - point_i.y)*(point_i.y - point_i.y) + (point_i.z - point_i.z)*(point_i.z - point_i.z));
            if (nn_distances[5] > 1.0) continue;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*input_cloud, nn_indices, centroid);
            pcl::PointXYZ cent;
            cent.x = centroid[0];
            cent.y = centroid[1];
            cent.z = centroid[2];
            //float distance = pointDistance(cent, point_i);
            float distance = sqrt((cent.x - point_i.x)*(cent.x - point_i.x) + (cent.y - point_i.y)*(cent.y - point_i.y) + (cent.z - point_i.z)*(cent.z - point_i.z));
            int size = nn_distances.size();
            float dis_p = nn_distances[size - 1] - nn_distances[0];
            if (dis_p < dis_p_th && distance < (dis_p*distance_th))
            {
                cloudLock.lock();
                output_cloud->points.push_back(point_i);
                cloudLock.unlock();
            }
        }
    }
}









void ccPointPairRegistrationDlg::getTransformationMatrixFromTxt()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("PointPairRegistration");
    QString currentPath = settings.value("TransformationMatrix", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString selectedFile = CS::Widgets::FramelessFileDialog::getOpenFileName(this, tr("Open file"), currentPath, "(*.txt)");
    if (selectedFile.isEmpty())
    {
        settings.endGroup();
        return;
    }
    settings.setValue("TransformationMatrix", QFileInfo(selectedFile).absolutePath());
    settings.endGroup();

    m_Scalefactor = -1;
    m_TransformationMatrix.clear();

    QFile file(selectedFile);

    if (!file.open(QIODevice::ReadOnly))
    {
        emit showMessage(tr("Import failed"));
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
        return;
    }

    QTextStream * read = new QTextStream(&file);
    QStringList Data = read->readAll().split("\n", QString::SkipEmptyParts);   //每行以\n区分
    bool hasIndexColumn = true;
    if (Data.size() < 7)
    {
        file.close();
        emit showMessage(tr("Import failed"));
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
        return;
    }
    bool isreadsuccess = true;
    m_Scalefactor = Data.at(6).toDouble(&isreadsuccess);
    if (!isreadsuccess)
    {
        emit showMessage(tr("Import failed"));
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
        return;
    }
    for (int i = 1; i < 5; ++i)
    {
        QString str = Data.at(i);
        str.replace(QRegExp("[\\s]+"), " ");
        QStringList strLine = str.split(" ");     //一行中的单元格以，区分
        if (strLine.size() < 4)
        {
            break;
        }

        double inputnum = 0;
        bool success = true;
        for (int j = 0; j < 4; ++j)
        {
            inputnum = strLine.at(j).toDouble(&success);
            if (!success)
            {
                break;
            }
            m_TransformationMatrix.push_back(inputnum);
        }
    }
    file.close();



    if (m_TransformationMatrix.size() == 16)
    {
        updateMatrixData();
        emit showMessage(tr("Import File Success!"));
    }
    else
    {
        m_Scalefactor = -1;
        m_TransformationMatrix.clear();
        emit showMessage(tr("Import failed"));
        CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Import failed"));
    }

}


void ccPointPairRegistrationDlg::slotAlignedHeadviewerStateChanged(bool checked)
{
    int currentRowCount = alignedPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        QWidget * itemwidget = alignedPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            box.at(0)->setChecked(checked);
        }
    }
}

void ccPointPairRegistrationDlg::slotRefHeadviewerStateChanged(bool checked)
{
    int currentRowCount = refPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        QWidget * itemwidget = refPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            box.at(0)->setChecked(checked);
        }
    }
}

void ccPointPairRegistrationDlg::align()
{

    alignToolButton->setEnabled(false);
    CCCoreLib::PointProjectionTools::Transformation trans;
    double rms;
    m_isCancelClickd = true;
    if (m_referenceEntities.size() == 0 && stackedWidget->currentIndex() == 1)
    {
        ccGLMatrixd currentGLMatrixd = getGLMatrixData();
        ccGLMatrixd transMatHaveShiftvalue = currentGLMatrixd.transposed();

        bool IsShifted = false;    
        //计算偏移信息
        CCVector3d PtsGlobalCoord = transMatHaveShiftvalue.getTranslationAsVec3D();
        CCVector3d Pshift(0, 0, 0);
        double scale = 1.0;
        if (ccGlobalShiftManager::Handle(PtsGlobalCoord, 0, ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG, true, Pshift, nullptr, &scale))
        {
            IsShifted = true;
        }
        CCVector3f transNoShift(PtsGlobalCoord.x + Pshift.x, PtsGlobalCoord.y + Pshift.y, PtsGlobalCoord.z + Pshift.z);
        ccGLMatrix transMat(transMatHaveShiftvalue.data());
        transMat.clearTranslation();
        transMat.setTranslation(transNoShift);
        ccGLMatrix currentGLMatrix(transMat.data());
        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            it.key()->setGLTransformation(currentGLMatrix);
        }
        m_alignedPoints.setGLTransformation(currentGLMatrix);

        resetToolButton->setEnabled(true);
        validToolButton->setEnabled(true);
    }
    else if (callHornRegistration(trans, rms, true))
    {
        //apply (scaled) transformation (if not fixed)
        bool adjustScale = false;
        if (adjustScale)
        {
            if (trans.R.isValid())
                trans.R.scale(trans.s);
        }

        ccGLMatrix transMat = FromCCLibMatrix<double, float>(trans.R, trans.T);
        //...virtually
        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            it.key()->setGLTransformation(transMat);
        }
        m_alignedPoints.setGLTransformation(transMat);

        resetToolButton->setEnabled(true);
        validToolButton->setEnabled(true);
    }
    emit signalPreview();
    m_isCancelClickd = false;
    MainWindow::TheInstance()->setGlobalZoom();
    m_isAlreadyAlign = true;
}

bool ccPointPairRegistrationDlg::callHornRegistration(CCCoreLib::PointProjectionTools::Transformation& trans, double& rms, bool autoUpdateTab)
{
    if (m_alignedEntities.empty())
    {
        clearRMSColumns();
        return false;
    }

    ccPointCloud alignedPoints;
    std::vector<int>  alignedindex;
    ccPointCloud refPoints;
    std::vector<int>  refindex;
    if (!getValidPointcloudAndIndex(alignedPoints, alignedindex, alignedPointsTableWidget) || !getValidPointcloudAndIndex(refPoints, refindex, refPointsTableWidget))
    {
        clearRMSColumns();
        return false;
    }

    if (alignedPoints.size() != refPoints.size() || refPoints.size() < MIN_PAIRS_COUNT)
    {
        clearRMSColumns();
        return false;
    }

    //fixed scale?
    bool adjustScale = false;

    //call Horn registration method
    if (!CCCoreLib::HornRegistrationTools::FindAbsoluteOrientation(&alignedPoints, &refPoints, trans, !adjustScale))
    {
        clearRMSColumns();
        return false;
    }

    //compute RMS
    rms = CCCoreLib::HornRegistrationTools::ComputeRMS(&alignedPoints, &refPoints, trans);

    if (autoUpdateTab)
    {
        //display resulting RMS in colums
        if (rms >= 0)
        {
            for (unsigned i = 0; i < alignedPoints.size(); ++i)
            {
                const CCVector3* Ri = refPoints.getPoint(i);
                const CCVector3* Li = alignedPoints.getPoint(i);
                CCVector3d Lit = trans.apply(*Li);
                double dist = (Ri->toDouble() - Lit).norm();
                if (dist < 1e-3)
                {
                    dist = 0;
                }
                QTableWidgetItem* itemA = new QTableWidgetItem();
                itemA->setFlags(itemA->flags() & ~Qt::ItemIsEditable);
                itemA->setData(Qt::EditRole, dist);
                itemA->setTextAlignment(Qt::AlignCenter);
                alignedPointsTableWidget->setItem(alignedindex[i], RMS_COL_INDEX, itemA);
                QTableWidgetItem* itemR = new QTableWidgetItem();
                itemR->setData(Qt::EditRole, dist);
                itemR->setFlags(itemR->flags() & ~Qt::ItemIsEditable);
                itemR->setTextAlignment(Qt::AlignCenter);
                refPointsTableWidget->setItem(refindex[i], RMS_COL_INDEX, itemR);
            }
        }
        else
        {
            //clear RMS columns
            clearRMSColumns();
        }
    }

    return true;
}

bool ccPointPairRegistrationDlg::getValidPointcloudAndIndex(ccPointCloud& cloudpoints, std::vector<int> & index, QTableWidget * table)
{
    cloudpoints.clear();
    index.clear();
    int currentRowCount = table->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = table->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
        }
        if (ischecked)
        {
            if (!table->item(i, 2)->text().isEmpty() && !table->item(i, 3)->text().isEmpty() && !table->item(i, 4)->text().isEmpty())
            {
                index.push_back(i);
                CCVector3d Pin;
                Pin.x = table->item(i, 2)->text().toDouble();
                Pin.y = table->item(i, 3)->text().toDouble();
                Pin.z = table->item(i, 4)->text().toDouble();
                if (cloudpoints.size() == 0)
                {
                    cloudpoints.setGlobalScale(1.0);
                    cloudpoints.setGlobalShift(0, 0, 0);
                    bool shiftEnabled = m_alignedEntities.isShifted;
                    CCVector3d Pshift = m_alignedEntities.shift;
                    double scale = 1.0;
                    if (ccGlobalShiftManager::Handle(Pin, 0, ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT, shiftEnabled, Pshift, nullptr, &scale))
                    {
                        cloudpoints.setGlobalShift(Pshift);
                        cloudpoints.setGlobalScale(scale);
                    }
                }
                CCVector3 P = cloudpoints.toLocal3pc<double>(Pin);
                cloudpoints.addPoint(P);
            }
            else
            {
                return false;
            }
        }
    }

    for (auto i : index)
    {
        CCVector3d point;
        point.x = table->item(i, 2)->text().toDouble();
        point.y = table->item(i, 3)->text().toDouble();
        point.z = table->item(i, 4)->text().toDouble();
        if (point.x != 0 && point.y != 0 && point.z != 0)
        {
            bool shiftEnabled = m_alignedEntities.isShifted;
            CCVector3d Pshift = m_alignedEntities.shift;
            double scale = 1.0;
            if (ccGlobalShiftManager::Handle(point, 0, ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT, shiftEnabled, Pshift, nullptr, &scale))
            {
                cloudpoints.setGlobalShift(Pshift);
                cloudpoints.setGlobalScale(scale);
            }
        }
    }
    int startnum = 0;
    for (auto i : index)
    {
        CCVector3d point;
        point.x = table->item(i, 2)->text().toDouble();
        point.y = table->item(i, 3)->text().toDouble();
        point.z = table->item(i, 4)->text().toDouble();
        CCVector3 P = cloudpoints.toLocal3pc<double>(CCVector3d(point.x, point.y, point.z));
        cloudpoints.setPointVec(startnum, P);
        startnum++;
    }
    return true;
}

void ccPointPairRegistrationDlg::clearRMSColumns()
{
    for (int i = 0; i < alignedPointsTableWidget->rowCount(); ++i)
    {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        item->setTextAlignment(Qt::AlignCenter);
        alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, item);
    }
    for (int i = 0; i < refPointsTableWidget->rowCount(); ++i)
    {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        item->setTextAlignment(Qt::AlignCenter);
        refPointsTableWidget->setItem(i, RMS_COL_INDEX, item);
    }
}

bool ccPointPairRegistrationDlg::getCanAlign()
{
    if (stackedWidget->currentIndex() == 0)
    {
        ccPointCloud alignedPoints;
        std::vector<int>  alignedindex;
        ccPointCloud refPoints;
        std::vector<int>  refindex;
        if (!getValidPointcloudAndIndex(alignedPoints, alignedindex, alignedPointsTableWidget) || !getValidPointcloudAndIndex(refPoints, refindex, refPointsTableWidget))
        {
            return false;
        }

        if (alignedPoints.size() != refPoints.size() || refPoints.size() < MIN_PAIRS_COUNT)
        {
            return false;
        }
        return true;
    }
    else
    {
        for (int i = 2; i < 19; i++)
        {
            QString objname = "doubleSpinBox_" + QString::number(i);
            QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
            if (foundBox)
            {
                if (foundBox->text().isEmpty())
                {
                    alignToolButton->setEnabled(false);
                    validToolButton->setEnabled(false);
                    return false;
                }
            }
        }
        return true;
    }
}

void ccPointPairRegistrationDlg::reset()
{
    if (constituencyButton->isChecked()) {
        RadioModelButton->setEnabled(true);
        RadioModelButton->setChecked(false);
        MultipleChoiceButton->setEnabled(true);
        MultipleChoiceButton->setChecked(false);
        doubleSpinBox_19->setEnabled(true);
        doubleSpinBox_20->setEnabled(true);
        doubleSpinBox_21->setEnabled(true);
        return;
    }
    
    if (m_alignedEntities.empty())
        return;

    for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    m_alignedPoints.enableGLTransformation(false);
    for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    m_refPoints.enableGLTransformation(false);
    if (stackedWidget->currentIndex() == 0)
    {
        updateAlignInfo();
    }

    bool canAlign = getCanAlign();
    alignToolButton->setEnabled(canAlign);
    resetToolButton->setEnabled(false);
    m_isAlreadyAlign = false;
    MainWindow::TheInstance()->setGlobalZoom();
    emit signalReset();
}

void ccPointPairRegistrationDlg::updateAlignInfo()
{

    CCCoreLib::PointProjectionTools::Transformation trans;
    double rms;

    if (callHornRegistration(trans, rms, true))
    {
        validToolButton->setEnabled(true);
        alignToolButton->setEnabled(true);
        label_RMSE->setText(tr("Final RMS:")+QString::number(rms));
    }
    else
    {
        validToolButton->setEnabled(false);
        alignToolButton->setEnabled(false);
        label_RMSE->setText(tr("Final RMS:") + QString::number(0));
    }

    m_associatedWin->redraw();
}

void ccPointPairRegistrationDlg::onPointCountChanged()
{
    bool canAlign = getCanAlign();
    alignToolButton->setEnabled(canAlign);
    validToolButton->setEnabled(false);
    updateAlignInfo();
}

void ccPointPairRegistrationDlg::slotOpenSavePath()
{
    QSettings settings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
    settings.beginGroup("ccPointPairRegistrationDlg");
    QString currentPath = settings.value("ccPointPairRegistrationDlg", QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)).toString();
    QString SaveName = CS::Widgets::FramelessFileDialog::getExistingDirectory(this, tr("Save file"), currentPath, QFileDialog::ShowDirsOnly);
    if (SaveName.isEmpty()) {
        settings.endGroup();
        return;
    }
    lineEdit->setText(SaveName);
    //QString ss = QFileInfo(SaveName).absolutePath();
    //SaveName += "/";
    settings.setValue("ccPointPairRegistrationDlg", SaveName);
    settings.endGroup();
}

void ccPointPairRegistrationDlg::setConstituencyParameter()
{
    double resoultion = doubleSpinBox_19->value();
    double n_optimal = doubleSpinBox_20->value();
    int neighborhood = doubleSpinBox_21->value();

}

void ccPointPairRegistrationDlg::apply()
{

    ccGLMatrixd currentGLMatrix; 
    bool isuseGLMatrix = false;
    if (m_referenceEntities.size() == 0 && stackedWidget->currentIndex()==1)
    {
        currentGLMatrix = getGLMatrixData();
        isuseGLMatrix = true;
    }
    ccPointCloud alignedPoints;
    std::vector<int>  alignedindex;
    ccPointCloud refPoints;
    std::vector<int>  refindex;
    if (!getValidPointcloudAndIndex(alignedPoints, alignedindex, alignedPointsTableWidget) || !getValidPointcloudAndIndex(refPoints, refindex, refPointsTableWidget))
    {
        return;
    }
    //删除历史记录
    {

        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            int clouduuid = it.key()->getUniqueID();
            auto iter = std::remove_if(m_HistoryDatas.begin(), m_HistoryDatas.end(), [clouduuid](HistoryData curhisdata) {
                return isHasHistory(curhisdata, clouduuid);
            });
            // 删除移动到末尾的元素
            m_HistoryDatas.erase(iter, m_HistoryDatas.end());
        }

        for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
        {
            int clouduuid = it.key()->getUniqueID();
            auto iter = std::remove_if(m_HistoryDatas.begin(), m_HistoryDatas.end(), [clouduuid](HistoryData curhisdata) {
                return isHasHistory(curhisdata, clouduuid);
            });
            // 删除移动到末尾的元素
            m_HistoryDatas.erase(iter, m_HistoryDatas.end());
        }

    }
    QString RMSstr = "";
    QString transMatstr = "";
    QString transMatSingalStepstr = "";
    QString scalestr = "";
    QString showstr = QCoreApplication::translate("ccPointPairRegistrationDlg", "Please wait for point cloud registration.", nullptr);
    if (m_referenceEntities.size() == 0)
    {
        showstr = QCoreApplication::translate("ccPointPairRegistrationDlg", "Transforming the coordinates... Please wait.", nullptr);
    }
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(showstr);	//[!].并发执行线程中执行
    QFuture<void> fu = QtConcurrent::run([&]() {

        CCCoreLib::PointProjectionTools::Transformation trans;
        CCCoreLib::PointProjectionTools::Transformation transICP;
        ccGLMatrix transTem;
        ccGLMatrix transTem2;
        double rms = -1.0;
        m_isCancelClickd = true;
        qDebug() << "apply 1";
        if (isuseGLMatrix)
        {
            RMSstr = QString::number(0, 'f', 3);
            scalestr = QString::number(m_Scalefactor, 'f', 6);
            ccGLMatrixd transMatHaveShiftvalue = currentGLMatrix.transposed();

            //don't forget global shift:
            //reference shift takes precedence on the aligned entities'
            bool referenceIsShifted = false;
            CCVector3d referenceShift(0, 0, 0);
            double referenceScale = 1.0;
            //计算偏移信息
            CCVector3d PtsGlobalCoord = transMatHaveShiftvalue.getTranslationAsVec3D();
            CCVector3d Pshift = refPoints.getGlobalShift();
            double scale = refPoints.getGlobalScale();
            if (ccGlobalShiftManager::Handle(PtsGlobalCoord, 0, ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG, true, Pshift, nullptr, &scale))
            {
                referenceIsShifted = true;
                referenceShift = Pshift;
                referenceScale = scale;
            }
            CCVector3f transNoShift(PtsGlobalCoord.x + referenceShift.x, PtsGlobalCoord.y + referenceShift.y, PtsGlobalCoord.z + referenceShift.z);
            ccGLMatrix transMat(transMatHaveShiftvalue.data());
            transMat.clearTranslation();
            transMat.setTranslation(transNoShift);
            //应用转换矩阵
            for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
                it.key()->setGLTransformation(transMat);
            emit signalOctreeDeletion(true);

            m_alignedPoints.setGLTransformation(transMat);
            m_alignedPoints.applyGLTransformation_recursive();//janson

            //if (m_referenceEntities.size() == 0)
            {
                transMatstr = transMatHaveShiftvalue.toString(3, '\t', true);
                transMatSingalStepstr = transMatHaveShiftvalue.toString(6, '\t');
            }
            if (!m_referenceEntities.isEmpty())
            {
                referenceIsShifted = m_referenceEntities.isShifted;
                referenceShift = m_referenceEntities.shift;
                referenceScale = m_referenceEntities.scale;
            }
            else if (refPoints.isShifted())
            {
                // shift was automatically applied (temporarily, and for display purposes)
                CCVector3d Pshift = refPoints.getGlobalShift();
                double scale = refPoints.getGlobalScale();
                CCVector3d Pin = refPoints.toGlobal3d(*refPoints.getPoint(0));
                if (ccGlobalShiftManager::Handle(Pin, 0, ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG, true, Pshift, nullptr, &scale))
                {
                    referenceIsShifted = true;
                    referenceShift = Pshift;
                    referenceScale = scale;
                }
            }
            qDebug() << "apply 10";

            bool alwaysDropShift = false;
            bool alwaysDropShiftQuestionAsked = false;
            for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
            {
                ccGenericPointCloud* alignedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
                if (alignedCloud)
                {
                    if (referenceIsShifted)
                    {
                        alignedCloud->setGlobalShift(referenceShift);
                        alignedCloud->setGlobalScale(referenceScale);
                        ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been updated to match the reference: (%1,%2,%3) [x%4]")
                            .arg(alignedCloud->getName())
                            .arg(referenceShift.x)
                            .arg(referenceShift.y)
                            .arg(referenceShift.z)
                            .arg(referenceScale));
                    }
                    else if (alignedCloud->isShifted()) // the aligned cloud is shifted, but not the reference cloud
                    {
                        //we'll ask the user confirmation before dropping the shift information on the aligned cloud
                        if (!alwaysDropShiftQuestionAsked)
                        {
                            //alwaysDropShift = (CS::Widgets::FramelessMessageBox::question(this, tr("Drop shift information?"), tr("Aligned cloud is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
                            alwaysDropShift = true;
                            alwaysDropShiftQuestionAsked = true;
                        }

                        if (alwaysDropShift)
                        {
                            alignedCloud->setGlobalShift(0, 0, 0);
                            alignedCloud->setGlobalScale(1.0);
                            ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been reset to match the reference!").arg(alignedCloud->getName()));
                        }
                    }
                }
            }
        }
        else if (callHornRegistration(trans, rms, false))
        {

            qDebug() << "apply 2";

            if (rms >= 0)
            {
                RMSstr = QString::number(rms, 'f', 3);
            }
            qDebug() << "apply 3";

            //apply (scaled) transformation (if not fixed)
            bool adjustScale = true;
            if (adjustScale && trans.R.isValid())
            {
                trans.R.scale(trans.s);
            }
            ccGLMatrix transMat = FromCCLibMatrix<double, float>(trans.R, trans.T);
            transTem = transMat;
            //...for real this time!
            assert(!m_alignedEntities.empty());
            qDebug() << "apply 4";

            //应用转换矩阵
            for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
                it.key()->setGLTransformation(transMat);
            qDebug() << "apply 6";
            emit signalOctreeDeletion(true);
            qDebug() << "apply 7";

            m_alignedPoints.setGLTransformation(transMat);
            m_alignedPoints.applyGLTransformation_recursive();//janson
            qDebug() << "apply 8";

            //don't forget global shift:
            //reference shift takes precedence on the aligned entities'
            bool referenceIsShifted = false;
            CCVector3d referenceShift(0, 0, 0);
            double referenceScale = 1.0;
            if (!m_referenceEntities.isEmpty())
            {
                referenceIsShifted = m_referenceEntities.isShifted;
                referenceShift = m_referenceEntities.shift;
                referenceScale = m_referenceEntities.scale;
            }
            else if (refPoints.isShifted())
            {
                // shift was automatically applied (temporarily, and for display purposes)
                CCVector3d Pshift = refPoints.getGlobalShift();
                double scale = refPoints.getGlobalScale();
                CCVector3d Pin = refPoints.toGlobal3d(*refPoints.getPoint(0));
                if (ccGlobalShiftManager::Handle(Pin, 0, ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG, true, Pshift, nullptr, &scale))
                {
                    referenceIsShifted = true;
                    referenceShift = Pshift;
                    referenceScale = scale;
                }
            }
            qDebug() << "apply 9";

            //if (m_referenceEntities.size() == 0)
            {
                ccGLMatrixd transMatAddShiftValue = FromCCLibMatrix<double, double>(trans.R, trans.T + referenceShift * -1);
                transMatstr = transMatAddShiftValue.toString(3, '\t', true);
                transMatSingalStepstr = transMatAddShiftValue.toString(6, '\t');

                if (adjustScale)
                {
                    scalestr = QString::number(1.0, 'f', 6);
                }
                else
                {
                    scalestr = QString::number(1.0, 'f', 6);
                }
            }
            qDebug() << "apply 10";

            bool alwaysDropShift = false;
            bool alwaysDropShiftQuestionAsked = false;
            for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
            {
                ccGenericPointCloud* alignedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
                if (alignedCloud)
                {
                    if (referenceIsShifted)
                    {
                        alignedCloud->setGlobalShift(referenceShift);
                        alignedCloud->setGlobalScale(referenceScale);
                        ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been updated to match the reference: (%1,%2,%3) [x%4]")
                            .arg(alignedCloud->getName())
                            .arg(referenceShift.x)
                            .arg(referenceShift.y)
                            .arg(referenceShift.z)
                            .arg(referenceScale));
                    }
                    else if (alignedCloud->isShifted()) // the aligned cloud is shifted, but not the reference cloud
                    {
                        //we'll ask the user confirmation before dropping the shift information on the aligned cloud
                        if (!alwaysDropShiftQuestionAsked)
                        {
                            //alwaysDropShift = (CS::Widgets::FramelessMessageBox::question(this, tr("Drop shift information?"), tr("Aligned cloud is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
                            alwaysDropShift = true;
                            alwaysDropShiftQuestionAsked = true;
                        }

                        if (alwaysDropShift)
                        {
                            alignedCloud->setGlobalShift(0, 0, 0);
                            alignedCloud->setGlobalScale(1.0);
                            ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been reset to match the reference!").arg(alignedCloud->getName()));
                        }
                    }
                }
            }
        }
        qDebug() << "apply 11";
        bool useICP = false;
        if (m_icpValueVector.size() == 2) {
            useICP = static_cast<bool>(m_icpValueVector.at(0));
        }
        //点云精配准
        if (m_referenceEntities.size() != 0 && useICP)
        {
            //Progress Bar
            qDebug() << "start  ccProgressDialog !";

            pcl::PointCloud<pcl::PointXYZ>::Ptr alignCloud1 = nullptr;//Aligned point
            pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud = nullptr;//Ref point  
            pcl::PointCloud<pcl::PointXYZ>::Ptr refPointsPCL = nullptr, alignPointsPCL = nullptr;
            pcl::PointCloud<pcl::PointXYZ>::Ptr alignCloud1thin(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr refCloudthin(new pcl::PointCloud<pcl::PointXYZ>);
            //点云数据转换
            ccHObject* aligned = m_alignedEntities.firstKey();
            alignCloud1 = cc2smReader(ccHObjectCaster::ToPointCloud(aligned)).getRawXYZ();
            //alignPointsPCL = cc2smReader(m_alignedPoints).getRawXYZ();

            ccHObject* reference = m_referenceEntities.firstKey();
            refCloud = cc2smReader(ccHObjectCaster::ToPointCloud(reference)).getRawXYZ();
            //refPointsPCL = cc2smReader(m_refPoints).getRawXYZ();

            fjCloud_filter(alignCloud1, alignCloud1thin, 30, 1.2, 1.5);//调用后处理建图中滤波函数进行薄化
            fjCloud_filter(refCloud, refCloudthin, 30, 1.2, 1.5);
            pcl::PointCloud<pcl::PointXYZ>::Ptr overlapAlignCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr overlapRefCloud(new pcl::PointCloud<pcl::PointXYZ>);

            getOverlappedCloud(*alignCloud1thin, *refCloudthin, *overlapRefCloud);//源点云中特征点附近的点
            getOverlappedCloud(*refCloudthin, *alignCloud1thin, *overlapAlignCloud); //目标点云中特征点附近的点

            //pcl::PointCloud<pcl::PointXYZ>::Ptr overlapAlignCloudclean(new pcl::PointCloud<pcl::PointXYZ>);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr overlapRefCloudclean(new pcl::PointCloud<pcl::PointXYZ>);
            //removeOutlier(overlapRefCloud, overlapRefCloudclean); //统计滤波去除离散点
            //removeOutlier(overlapAlignCloud, overlapAlignCloudclean);
           /* pcl::PointCloud<pcl::PointXYZ>::Ptr filteredAlignCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredRefCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setInputCloud(overlapAlignCloud);
            filter.setLeafSize(0.1f, 0.1f, 0.1f);
            filter.filter(*filteredAlignCloud);
            pcl::VoxelGrid<pcl::PointXYZ> filter1;
            filter1.setInputCloud(overlapRefCloud);
            filter1.setLeafSize(0.1f, 0.1f, 0.1f);
            filter1.filter(*filteredRefCloud);*///体素滤波
            //findRadiusClosedCloud(*refPointsPCL, *refCloud, *overlapRefCloud);//源点云中特征点附近的点
            //findRadiusClosedCloud(*alignPointsPCL, *alignCloud1, *overlapAlignCloud); //目标点云中特征点附近的点

            //// 测试代码 janson
            //pcl::PCDWriter w;
            //w.writeBinary("ref.pcd", *overlapRefCloud);
            //w.writeBinary("align.pcd", *overlapAlignCloud);
            //w.writeBinary("alignsample.pcd", *alignCloud1thin);
            //w.writeBinary("refsample.pcd", *refCloudthin);

            PCLCloud cloudTemp;
            TO_PCL_CLOUD(*overlapAlignCloud, cloudTemp);
            ccPointCloud* alg_cloud = pcl2cc::Convert(cloudTemp);

            PCLCloud cloudTemp1;
            TO_PCL_CLOUD(*overlapRefCloud, cloudTemp1);
            ccPointCloud* ref_cloud = pcl2cc::Convert(cloudTemp1);

            ccHObject* data = alg_cloud;
            ccHObject* model = ref_cloud;
            /* ccHObject* data = aligned;
             ccHObject* model = reference;*/
             //ICP
            double minRMSDecrease = 1.0e-4;
            qDebug() << " ccRegistrationDlg start -1!";
            if (std::isnan(minRMSDecrease))
            {
                ccLog::Error(tr("Invalid minimum RMS decrease value"));
                return;
            }
            qDebug() << " ccRegistrationDlg start -2!";

            if (minRMSDecrease < ccRegistrationDlg::GetAbsoluteMinRMSDecrease())
            {
                minRMSDecrease = ccRegistrationDlg::GetAbsoluteMinRMSDecrease();
                ccLog::Error(tr("Minimum RMS decrease value is too small.\n%1 will be used instead (numerical accuracy limit).").arg(minRMSDecrease, 0, 'E', 1));
            }
            CCCoreLib::ICPRegistrationTools::Parameters parameters;
            {
                parameters.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
                //parameters.minRMSDecrease = minRMSDecrease;
                parameters.minRMSDecrease = m_icpValueVector.at(1);
                parameters.nbMaxIterations = 25;
                parameters.adjustScale = true;
                parameters.filterOutFarthestPoints = false;

                //parameters.samplingLimit = (overlapAlignCloud->size() > overlapRefCloud->size()) ? overlapAlignCloud->size() : overlapRefCloud->size();
                /*int  overrationRef = static_cast<int>((float)cloud_Ref->size() / (float)refCloud.size() * 100);
                int overrationAlg = static_cast<int>((float)cloud_Align->size() / (float)alignCloud1.size() * 100);
                if (overrationRef <= overrationAlg)
                {
                    parameters.finalOverlapRatio = overrationRef / 100.0;
                }
                else
                {
                    parameters.finalOverlapRatio = overrationAlg / 100.0;
                }*/
                //if (overrationRef <= 10 && overrationAlg <= 10)
                {
                    parameters.finalOverlapRatio = 70 / 100.0;
                }
                //parameters.finalOverlapRatio = overration / 100.0;
                parameters.transformationFilters = 0;
                parameters.maxThreadCount = 16;
                parameters.useC2MSignedDistances = false;
                parameters.normalsMatching = CCCoreLib::ICPRegistrationTools::NO_NORMAL;
            }
            qDebug() << " ccRegistrationDlg -2  !";
            bool useDataSFAsWeights = false;
            bool useModelSFAsWeights = false;
            ccGLMatrix transMatICP;
            double finalError = 0.0;
            double finalScale = 1.0;
            unsigned finalPointCount = 0;
            qDebug() << " ccRegistrationDlg -3  !";

            if (ccRegistrationTools::ICP(data,
                model,
                transMatICP,
                finalScale,
                finalError,
                finalPointCount,
                parameters,
                useDataSFAsWeights,
                useModelSFAsWeights,
                this))
            {
                transTem2 = transMatICP;
                for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
                {
                    it.key()->setGLTransformation(transMatICP);
                }
                qDebug() << " ccRegistrationDlg -4  !";

                emit signalOctreeDeletion(false);
                m_alignedPoints.setGLTransformation(transMatICP);
                m_alignedPoints.applyGLTransformation_recursive();//janson

                ccGLMatrix lastMat = transTem * transTem2;

                qDebug() << " ccRegistrationDlg -5  !";

                if (rms >= 0)
                {
                    RMSstr = QString::number(finalError, 'f', 3);
                }
                transMatstr = lastMat.toString(3, '\t', true);
                transMatSingalStepstr = lastMat.toString(6, '\t');

                ccLog::Print("[PointPairRegistration] Applied transformation matrix:");
                ccLog::Print(lastMat.toString(12, ' ')); //full precision
                qDebug() << " ccRegistrationDlg -6  !";

               /* if (true)
                {
                    scalestr = QString::number(trans.s, 'f', 6);
                }
                else*/
                {
                    scalestr = QString::number(1.0, 'f', 1);
                }
                qDebug() << " ccRegistrationDlg -7  !";

                //don't forget global shift:
                //reference shift takes precedence on the aligned entities'
                bool referenceIsShifted = false;
                CCVector3d referenceShift(0, 0, 0);
                double referenceScale = 1.0;

                if (!m_referenceEntities.isEmpty())
                {
                    referenceIsShifted = m_referenceEntities.isShifted;
                    referenceShift = m_referenceEntities.shift;
                    referenceScale = m_referenceEntities.scale;
                }
                else if (refPoints.isShifted())
                {
                    // shift was automatically applied (temporarily, and for display purposes)
                    CCVector3d Pshift = refPoints.getGlobalShift();
                    double scale = refPoints.getGlobalScale();
                    CCVector3d Pin = refPoints.toGlobal3d(*refPoints.getPoint(0));
                    if (ccGlobalShiftManager::Handle(Pin, 0, ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG, true, Pshift, nullptr, &scale))
                    {
                        referenceIsShifted = true;
                        referenceShift = Pshift;
                        referenceScale = scale;
                    }
                }

                bool alwaysDropShift = false;
                bool alwaysDropShiftQuestionAsked = false;
                for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
                {
                    ccGenericPointCloud* alignedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
                    if (alignedCloud)
                    {
                        if (referenceIsShifted)
                        {
                            alignedCloud->setGlobalShift(referenceShift);
                            alignedCloud->setGlobalScale(referenceScale);
                            ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been updated to match the reference: (%1,%2,%3) [x%4]")
                                .arg(alignedCloud->getName())
                                .arg(referenceShift.x)
                                .arg(referenceShift.y)
                                .arg(referenceShift.z)
                                .arg(referenceScale));
                        }
                        else if (alignedCloud->isShifted()) // the aligned cloud is shifted, but not the reference cloud
                        {
                            //we'll ask the user confirmation before dropping the shift information on the aligned cloud
                            if (!alwaysDropShiftQuestionAsked)
                            {
                                //alwaysDropShift = (CS::Widgets::FramelessMessageBox::question(this, tr("Drop shift information?"), tr("Aligned cloud is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
                                alwaysDropShift = true;
                                alwaysDropShiftQuestionAsked = true;
                            }
                            if (alwaysDropShift)
                            {
                                alignedCloud->setGlobalShift(0, 0, 0);
                                alignedCloud->setGlobalScale(1.0);
                                ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been reset to match the reference!").arg(alignedCloud->getName()));
                            }
                        }
                    }
                }
            }
            delete alg_cloud;
            alg_cloud = nullptr;
            delete ref_cloud;
            ref_cloud = nullptr;

            //点云融合拼接
            //pointCloudSplicing();
        }
    });
    while (!fu.isFinished()) {
        QThread::msleep(20);
        QCoreApplication::processEvents();
    }
    WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
    qDebug() << "start  ccPointPairRegistrationInformationDlg -2!";
    if (m_referenceEntities.size() == 0)
    {
        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            it.key()->setName(it.key()->getName() + ".transform");
        }
    }
    if (m_referenceEntities.size() == 0 && checkBox->isChecked())
    {
        QString savepath = lineEdit->text();
        for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
        {
            savepath += ("/" + it.key()->getName() + ".las");
            CC_FILE_ERROR result = CC_FERR_NO_ERROR;
            FileIOFilter::SaveParameters parameters;
            {
                parameters.alwaysDisplaySaveDialog = true;
                parameters.parentWidget = this;
            }
            result = FileIOFilter::SaveToFile(it.key(), savepath, parameters, "LAS cloud (*.las *.laz)");
        }
    }

    setPickPointMode(false);
    if (m_associatedWin)
    {
        m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
    }
   
    m_isCancelClickd = false;

    MainWindow::TheInstance()->setGlobalZoom();

    ccPointPairRegistrationInformationDlg dlg(this);
    dlg.setTopMessage(RMSstr);
    dlg.setMidMessage(transMatstr);
    dlg.setTransMatstr(transMatSingalStepstr);
    dlg.setBottomMessage(scalestr);
    if (isuseGLMatrix)
    {
        dlg.setBottomMessage(QString::number(1.0, 'f', 6));
    }
    dlg.setSaveButtonVisiable(!isuseGLMatrix && m_referenceEntities.size() == 0);
    dlg.setWindowTitle(tr("Transformation Parameters"));
    dlg.exec();

    stop(true);
    qDebug() << "end  ccPointPairRegistrationInformationDlg -2!";
}

void ccPointPairRegistrationDlg::cancel()
{
    //保存历史数据
    {
        HistoryData data;
        for (const EntityContext& ec : m_alignedEntities)
        {
            if (ec.entity)
            {
                data.AlignedPointsUuid = ec.entity->getUniqueID();
            }
        }

        for (const EntityContext& ec : m_referenceEntities)
        {
            if (ec.entity)
            {
                data.RefPointsUuid = ec.entity->getUniqueID();
            }
        }
        for (int i = 0; i < refPointsTableWidget->rowCount(); i++)
        {
            bool ischecked = false;
            QWidget * itemwidget = refPointsTableWidget->cellWidget(i, 0);
            QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
            if (box.size() > 0)
            {
                ischecked = box.at(0)->isChecked();
            }
            std::vector<QString> pointstr;
            pointstr.push_back(refPointsTableWidget->item(i, 1)->text());
            pointstr.push_back(refPointsTableWidget->item(i, 2)->text());
            pointstr.push_back(refPointsTableWidget->item(i, 3)->text());
            pointstr.push_back(refPointsTableWidget->item(i, 4)->text());
            data.RefPointsdata.push_back(pointstr);
            data.RefPointsCheckstate.push_back(ischecked);
        }

        for (int i = 0; i < alignedPointsTableWidget->rowCount(); i++)
        {
            bool ischecked = false;
            QWidget * itemwidget = alignedPointsTableWidget->cellWidget(i, 0);
            QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
            if (box.size() > 0)
            {
                ischecked = box.at(0)->isChecked();
            }

            std::vector<QString> pointstr;
            pointstr.push_back(alignedPointsTableWidget->item(i, 1)->text());
            pointstr.push_back(alignedPointsTableWidget->item(i, 2)->text());
            pointstr.push_back(alignedPointsTableWidget->item(i, 3)->text());
            pointstr.push_back(alignedPointsTableWidget->item(i, 4)->text());
            data.AlignedPointsdata.push_back(pointstr);
            data.AlignedPointsCheckstate.push_back(ischecked);
        }
        bool isexist = false;
        for (auto & curdata : m_HistoryDatas)
        {
            if (curdata.AlignedPointsUuid == data.AlignedPointsUuid && curdata.RefPointsUuid == data.RefPointsUuid)
            {
                curdata.RefPointsdata = data.RefPointsdata;
                curdata.AlignedPointsdata = data.AlignedPointsdata;
                curdata.AlignedPointsCheckstate = data.AlignedPointsCheckstate;
                curdata.RefPointsCheckstate = data.RefPointsCheckstate;
                isexist = true;
            }
        }
        if (!isexist)
        {
            m_HistoryDatas.push_back(data);
        }
    }



    setPickPointMode(false);
    if (m_associatedWin)
    {
        m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
    }
    for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    stop(false);
}

void ccPointPairRegistrationDlg::setPointPairRegistrationCurrentButtonstate()
{

    for (EntityContext& ec : m_alignedEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            ec.entity->m_displayBoundingBox = checkBox_align->isChecked();
        }
    }

    for (EntityContext& ec : m_referenceEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {

            ec.entity->m_displayBoundingBox = checkBox_ref->isChecked();
        }
    }
    if (m_associatedWin)
    {
        m_associatedWin->redraw(false);
    }
}

void ccPointPairRegistrationDlg::slotalignedItemSelectionChanged()
{
    QList<QTableWidgetItem*>itemList = alignedPointsTableWidget->selectedItems();
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
        int currentRow = alignedPointsTableWidget->currentRow();
        if (currentRow != (alignedPointsTableWidget->rowCount() - 1) && alignedPointsTableWidget->rowCount() > 1 && rows.size() == 1)
        {
            toolButton_down->setEnabled(true);
        }
        else
        {
            toolButton_down->setEnabled(false);
        }
        if (currentRow != 0 && alignedPointsTableWidget->rowCount() > 1 && rows.size() == 1)
        {
            toolButton_up->setEnabled(true);
        }
        else
        {
            toolButton_up->setEnabled(false);
        }
        toolButton_delete->setEnabled(getDeleteButtonValid(alignedPointsTableWidget, rows));
    }
}

void ccPointPairRegistrationDlg::slotRefItemSelectionChanged()
{
    QList<QTableWidgetItem*>itemList = refPointsTableWidget->selectedItems();
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
        int currentRow = refPointsTableWidget->currentRow();
        if (currentRow != (refPointsTableWidget->rowCount() - 1) && refPointsTableWidget->rowCount() > 1 && rows.size() == 1)
        {
            toolButton_down_2->setEnabled(true);
        }
        else
        {
            toolButton_down_2->setEnabled(false);
        }
        if (currentRow != 0 && refPointsTableWidget->rowCount() > 1 && rows.size() == 1)
        {
            toolButton_up_2->setEnabled(true);
        }
        else
        {
            toolButton_up_2->setEnabled(false);
        }
        toolButton_delete_2->setEnabled(getDeleteButtonValid(refPointsTableWidget, rows));
    }
}

bool ccPointPairRegistrationDlg::getDeleteButtonValid(QTableWidget * table, const std::set<int> & index)
{
    for (auto i : index)
    {
        bool isvalid = (!table->item(i, 2)->text().isEmpty() || !table->item(i, 3)->text().isEmpty() || !table->item(i, 4)->text().isEmpty());
        if (isvalid)
        {
            return true;
        }
    }
    return false;
}

void ccPointPairRegistrationDlg::slotaligneditemChanged(QTableWidgetItem *item)
{
    if (m_isTableDataChangeLock || item->column() == 5)
    {
        return;
    }
    slotalignedItemSelectionChanged();
    int currentrow = item->row();
    bool isvalid = (!alignedPointsTableWidget->item(currentrow, 2)->text().isEmpty() && !alignedPointsTableWidget->item(currentrow, 3)->text().isEmpty() && !alignedPointsTableWidget->item(currentrow, 4)->text().isEmpty());
    if (!isvalid)
    {
        updateAlignInfo();
        return;
    }
    if (item->column() == 1)
    {
        int validindex = -1;
        for (int i = 0;i <= currentrow;i++)
        {
            bool iscurrowvalid = (!alignedPointsTableWidget->item(i, 2)->text().isEmpty() && !alignedPointsTableWidget->item(i, 3)->text().isEmpty() && !alignedPointsTableWidget->item(i, 4)->text().isEmpty());
            if (iscurrowvalid)
            {
                validindex++;
            }
        }
        int childNum = m_alignedPoints.getChildrenNumber();
        for (int j = childNum - 1; j >= 0; j--)
        {
            cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_alignedPoints.getChild(j));
            if (label)
            {
                auto pickpoint = label->getPickedPoint(0);
                if (pickpoint.index == validindex)
                {
                    label->setName(item->text());
                }
            }
        }
        if (m_associatedWin)
        {
            m_associatedWin->redraw();
        }
        return;
    }

    updateAlignPlontAndLabel();
    onPointCountChanged();
    if (m_isAlreadyAlign)
    {
        reset();
    }
    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}


void ccPointPairRegistrationDlg::slotRefitemChanged(QTableWidgetItem *item)
{
    if (m_isTableDataChangeLock || item->column() == 5)
    {
        return;
    }
    slotRefItemSelectionChanged();
    int currentrow = item->row();
    bool isvalid = (!refPointsTableWidget->item(currentrow, 2)->text().isEmpty() && !refPointsTableWidget->item(currentrow, 3)->text().isEmpty() && !refPointsTableWidget->item(currentrow, 4)->text().isEmpty());
    if (!isvalid)
    {
        updateAlignInfo();
        return;
    }
    if (item->column() == 1)
    {
        int validindex = -1;
        for (int i = 0; i <= currentrow; i++)
        {
            bool iscurrowvalid = (!refPointsTableWidget->item(i, 2)->text().isEmpty() && !refPointsTableWidget->item(i, 3)->text().isEmpty() && !refPointsTableWidget->item(i, 4)->text().isEmpty());
            if (iscurrowvalid)
            {
                validindex++;
            }
        }
        int childNum = m_refPoints.getChildrenNumber();
        for (int j = childNum - 1; j >= 0; j--)
        {
            cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_refPoints.getChild(j));
            if (label)
            {
                auto pickpoint = label->getPickedPoint(0);
                if (pickpoint.index == validindex)
                {
                    label->setName(item->text());
                }
            }
        }
        if (m_associatedWin)
        {
            m_associatedWin->redraw();
        }
        return;
    }
    updateRefPlontAndLabel();

    if (m_isAlreadyAlign)
    {
        reset();
    }
    onPointCountChanged();
    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}

void ccPointPairRegistrationDlg::updateAlignPlontAndLabel()
{
    m_alignedPoints.removeAllChildren();
    m_alignedPoints.clear();
    int currentRowCount = alignedPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        if (!alignedPointsTableWidget->item(i, 2)->text().isEmpty() && !alignedPointsTableWidget->item(i, 3)->text().isEmpty() && !alignedPointsTableWidget->item(i, 4)->text().isEmpty())
        {
            CCVector3d Pin;
            Pin.x = alignedPointsTableWidget->item(i, 2)->text().toDouble();
            Pin.y = alignedPointsTableWidget->item(i, 3)->text().toDouble();
            Pin.z = alignedPointsTableWidget->item(i, 4)->text().toDouble();
            if (m_alignedPoints.size() == 0)
            {
                if (m_alignedEntities.size() > 0)
                {
                    auto cloud = (m_alignedEntities.begin()).key();
                    if (cloud)
                    {
                        ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloud);
                        if (shiftcloud)
                        {
                            m_alignedPoints.copyGlobalShiftAndScale(*shiftcloud);
                        }
                    }
                }
            }
            CCVector3 P = m_alignedPoints.toLocal3pc<double>(Pin);
            m_alignedPoints.addPoint(P);
            cc2DLabel* label = CreateLabel(&m_alignedPoints, m_alignedPoints.size() - 1, alignedPointsTableWidget->item(i, 1)->text(), m_associatedWin);
            label->setIsShowConfigColor(false);
            label->setPointColor(ccColor::red);
            m_alignedPoints.addChild(label);
            if (m_referenceEntities.size() != 0)
            {
                label->setVisible(checkBox_align->isChecked());
            }
        }
    }
}

void ccPointPairRegistrationDlg::updateRefPlontAndLabel()
{
    m_refPoints.removeAllChildren();
    m_refPoints.clear();
    int currentRowCount = refPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        if (!refPointsTableWidget->item(i, 2)->text().isEmpty() && !refPointsTableWidget->item(i, 3)->text().isEmpty() && !refPointsTableWidget->item(i, 4)->text().isEmpty())
        {
            CCVector3d Pin;
            Pin.x = refPointsTableWidget->item(i, 2)->text().toDouble();
            Pin.y = refPointsTableWidget->item(i, 3)->text().toDouble();
            Pin.z = refPointsTableWidget->item(i, 4)->text().toDouble();
            if (m_refPoints.size() == 0)
            {
                if (m_referenceEntities.size() > 0)
                {
                    auto cloudref = (m_referenceEntities.begin()).key();
                    if (cloudref)
                    {
                        ccGenericPointCloud* shiftcloud = ccHObjectCaster::ToGenericPointCloud(cloudref);
                        if (shiftcloud)
                        {
                            m_refPoints.copyGlobalShiftAndScale(*shiftcloud);
                        }
                    }
                }
                else
                {
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
            CCVector3 P = m_refPoints.toLocal3pc<double>(Pin);
            m_refPoints.addPoint(P);
            cc2DLabel* label = CreateLabel(&m_refPoints, m_refPoints.size() - 1, refPointsTableWidget->item(i, 1)->text(), m_associatedWin);
            label->setIsShowConfigColor(false);
            label->setPointColor(ccColor::green);
            m_refPoints.addChild(label);
            if (m_referenceEntities.size() != 0)
            {
                label->setVisible(checkBox_ref->isChecked());
            }
        }
    }
}

void ccPointPairRegistrationDlg::slotPickToolButtonClicked()
{
    if (m_currentPickMode != PICKPOINT)
    {
        m_currentPickMode = PICKPOINT;
        setPickPointMode(true);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconClicked", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(false);
        doubleSpinBox->setEnabled(false);
    }
    else
    {
        m_currentPickMode = NOPICK;
        setPickPointMode(false);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(false);
        doubleSpinBox->setEnabled(false);
    }
}

void ccPointPairRegistrationDlg::slotBallToolButtonClicked()
{
    if (m_currentPickMode != PICKBALL)
    {
        m_currentPickMode = PICKBALL;
        setPickPointMode(true);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconClicked", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(false);
        doubleSpinBox->setEnabled(true);
    }
    else
    {
        m_currentPickMode = NOPICK;
        setPickPointMode(false);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(false);
        doubleSpinBox->setEnabled(false);
    }
}

void ccPointPairRegistrationDlg::slotPaperToolButtonClicked()
{
    if (m_currentPickMode != PICKPAPER)
    {

        bool isAlignHasSFIntensity = false;
        bool isAlignIsSFIntensity = false;
        for (const EntityContext& ec : m_alignedEntities)
        {
            if (ec.entity)
            {
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ec.entity);
                if (cloud)
                {
                    if (cloud->getCurrentDisplayedScalarField())
                    {
                        QString sfName = cloud ? QString::fromLocal8Bit(cloud->getCurrentDisplayedScalarField()->getName()) : "";
                        if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
                        {
                            isAlignIsSFIntensity = true;
                            //cloud->setCurrentCombinationMode(INTENSITY);
                        }
                    }

                    int nsf = cloud->getNumberOfScalarFields();
                    for (int i = 0; i < nsf; ++i)
                    {
                        QString sfName = cloud->getScalarFieldName(i);
                        if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
                        {
                            isAlignHasSFIntensity = true;
                            //cloud->setCurrentCombinationMode(INTENSITY);
                            break;
                        }
                    }
                    if (!cloud->sfShown())
                    {
                        isAlignIsSFIntensity = false;
                    }
                }
            }
        }
        bool isRefHasSFIntensity = false;
        bool isRefIsSFIntensity = false;
        for (const EntityContext& ec : m_referenceEntities)
        {
            if (ec.entity)
            {
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ec.entity);
                if (cloud)
                {
                    if (cloud->getCurrentDisplayedScalarField())
                    {
                        QString sfName = cloud ? QString::fromLocal8Bit(cloud->getCurrentDisplayedScalarField()->getName()) : "";
                        if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
                        {
                            isRefIsSFIntensity = true;
                            //cloud->setCurrentCombinationMode(INTENSITY);
                        }
                    }
                    int nsf = cloud->getNumberOfScalarFields();
                    for (int i = 0; i < nsf; ++i)
                    {
                        QString sfName = cloud->getScalarFieldName(i);
                        if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
                        {
                            isRefHasSFIntensity = true;
                            //cloud->setCurrentCombinationMode(INTENSITY);
                            break;
                        }
                    }
                    if (!cloud->sfShown())
                    {
                        isAlignIsSFIntensity = false;
                    }
                }
            }
        }
        if (!isAlignHasSFIntensity || (m_referenceEntities.size() > 0 && !isRefHasSFIntensity))
        {
            CS::Widgets::FramelessMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Error", nullptr), tr("Unable to switch to the intensity rendering mode. No intensity data in the point cloud."), QMessageBox::Abort);
            return;
        }
        else if (!isAlignIsSFIntensity || (m_referenceEntities.size() > 0 && !isRefIsSFIntensity))
        {
            CS::Widgets::FramelessMessageBox message(QMessageBox::Question, tr("Prompt"), tr("The target identification must be performed in the intensity rendering mode. Are you sure you want to switch to that mode?"),
                QMessageBox::Abort | QMessageBox::Ok, this);

            int type = message.exec();
            if (type == QMessageBox::Abort)
            {
                return;
            }
            else
            {
                
                MainWindow::TheInstance()->updateCombineColorMenuState();
                if (m_associatedWin)
                {
                    m_associatedWin->redraw(false);
                }
                if (MainWindow::TheInstance()->db())
                {
                    MainWindow::TheInstance()->db()->updatePropertiesView();
                }
            }
        }

        MainWindow::TheInstance()->updateCombineColorMenuState();

        m_currentPickMode = PICKPAPER;
        setPickPointMode(true);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconClicked", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(true);
        doubleSpinBox->setEnabled(false);
    }
    else
    {
        m_currentPickMode = NOPICK;
        setPickPointMode(false);
        if (m_associatedWin)
        {
            m_associatedWin->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
        }
        setActionIcon(toolButton_pick, "clickPointIconNormal", "clickPointIconClicked", "clickPointIconDisable");
        setActionIcon(toolButton_paper, "clickPaperIconNormal", "clickPaperIconClicked", "clickPaperIconDisabled");
        setActionIcon(toolButton_ball, "clickBallIconNormal", "clickBallIconClicked", "clickBallIconDisabled");
        doubleSpinBox_paper->setEnabled(false);
        doubleSpinBox->setEnabled(false);
    }
}

void ccPointPairRegistrationDlg::loadHistoryData()
{
    HistoryData data;
    for (const EntityContext& ec : m_alignedEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            data.AlignedPointsUuid = ec.entity->getUniqueID();
            break;
        }
    }

    for (const EntityContext& ec : m_referenceEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            data.RefPointsUuid = ec.entity->getUniqueID();
            break;
        }
    }
    for (auto & curdata : m_HistoryDatas)
    {
        if (curdata.AlignedPointsUuid == data.AlignedPointsUuid && curdata.RefPointsUuid == data.RefPointsUuid)
        {
            m_isTableDataChangeLock = true;
            alignedPointsTableWidget->setRowCount(curdata.AlignedPointsdata.size());
            for (int i = 0; i < curdata.AlignedPointsdata.size(); i++)
            {

                QCheckBox* box = new QCheckBox();
                box->setFixedSize(16, 16);
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout();
                layout->setMargin(0);//一定要有
                layout->addWidget(box);
                layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
                widget->setLayout(layout);
                box->setChecked(curdata.AlignedPointsCheckstate[i]);
                connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotAlignTableCheckStateChanged);
                alignedPointsTableWidget->setCellWidget(i, 0, widget);
                updateTableWidgetCheckState(alignedPointsTableWidget);
                QTableWidgetItem* labelitem = new QTableWidgetItem(curdata.AlignedPointsdata[i][0]);
                labelitem->setTextAlignment(Qt::AlignCenter);
                alignedPointsTableWidget->setItem(i, 1, labelitem);

                for (int d = 0; d < 3; ++d)
                {
                    QTableWidgetItem* item = new QTableWidgetItem();
                    item->setData(Qt::EditRole, curdata.AlignedPointsdata[i][d + 1]);
                    item->setTextAlignment(Qt::AlignCenter);
                    alignedPointsTableWidget->setItem(i, XYZ_COL_INDEX + d, item);
                    alignedPointsTableWidget->setRowHeight(i, 32);
                }
                QTableWidgetItem* itemError = new QTableWidgetItem();
                itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
                itemError->setTextAlignment(Qt::AlignCenter);
                alignedPointsTableWidget->setItem(i, XYZ_COL_INDEX + 3, itemError);
                int lastcurrentRow = alignedPointsTableWidget->rowCount();
                if (alignedPointsTableWidget->rowCount() > 4)
                {
                    alignedPointsTableWidget->setColumnWidth(5, 128);
                }
            }

            refPointsTableWidget->setRowCount(curdata.RefPointsdata.size());
            for (int i = 0; i < curdata.RefPointsdata.size(); i++)
            {
                QCheckBox* box = new QCheckBox();
                box->setFixedSize(16, 16);
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout();
                layout->setMargin(0);//一定要有
                layout->addWidget(box);
                layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
                widget->setLayout(layout);
                box->setChecked(curdata.RefPointsCheckstate[i]);
                connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotRefTableCheckStateChanged);
                refPointsTableWidget->setCellWidget(i, 0, widget);
                updateTableWidgetCheckState(refPointsTableWidget);
                QTableWidgetItem* labelitem = new QTableWidgetItem(curdata.RefPointsdata[i][0]);
                labelitem->setTextAlignment(Qt::AlignCenter);
                refPointsTableWidget->setItem(i, 1, labelitem);

                for (int d = 0; d < 3; ++d)
                {
                    QTableWidgetItem* item = new QTableWidgetItem();
                    item->setData(Qt::EditRole, curdata.RefPointsdata[i][d + 1]);
                    item->setTextAlignment(Qt::AlignCenter);
                    refPointsTableWidget->setItem(i, XYZ_COL_INDEX + d, item);
                    refPointsTableWidget->setRowHeight(i, 32);
                }
                QTableWidgetItem* itemError = new QTableWidgetItem();
                itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
                itemError->setTextAlignment(Qt::AlignCenter);
                refPointsTableWidget->setItem(i, XYZ_COL_INDEX + 3, itemError);
                int lastcurrentRow = refPointsTableWidget->rowCount();
                if (refPointsTableWidget->rowCount() > 4)
                {
                    refPointsTableWidget->setColumnWidth(5, 128);
                }
            }
            m_isTableDataChangeLock = false;
        }
        updateAlignPlontAndLabel();
        updateRefPlontAndLabel();
        onPointCountChanged();
    }
}

void ccPointPairRegistrationDlg::updateTableWidgetCheckState(QTableWidget * table)
{
    int numsum = 0;
    int currentRowCount = table->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = table->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
        }
        if (ischecked)
        {
            numsum++;
        }
    }
    if (numsum == 0)
    {
        if (table->objectName() == "alignedPointsTableWidget")
        {
            m_alignedCheckBoxHeader->blockSignals(true);
            m_alignedCheckBoxHeader->setCheckState(false);
            m_alignedCheckBoxHeader->blockSignals(false);
            m_alignedCheckBoxHeader->update();
        }
        else
        {
            m_refCheckBoxHeader->blockSignals(true);
            m_refCheckBoxHeader->setCheckState(false);
            m_refCheckBoxHeader->blockSignals(false);
            m_refCheckBoxHeader->update();
        }
    }
    else if (numsum == currentRowCount)
    {
        if (table->objectName() == "alignedPointsTableWidget")
        {
            m_alignedCheckBoxHeader->blockSignals(true);
            m_alignedCheckBoxHeader->setCheckState(true);
            m_alignedCheckBoxHeader->blockSignals(false);
            m_alignedCheckBoxHeader->update();
        }
        else
        {
            m_refCheckBoxHeader->blockSignals(true);
            m_refCheckBoxHeader->setCheckState(true);
            m_refCheckBoxHeader->blockSignals(false);
            m_refCheckBoxHeader->update();
        }
    }
}

void ccPointPairRegistrationDlg::slotAlignTableCheckStateChanged(int index)
{
    QCheckBox * checkbox = qobject_cast<QCheckBox*>(sender());
    int findindex = -1;
    onPointCountChanged();
    int numsum = 0;
    int currentRowCount = alignedPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = alignedPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
            if (box.at(0) == checkbox)
            {
                findindex = i;
            }
        }
        if (ischecked)
        {
            numsum++;
        }
    }
    if (numsum == 0)
    {
        m_alignedCheckBoxHeader->blockSignals(true);
        m_alignedCheckBoxHeader->setCheckState(false);
        m_alignedCheckBoxHeader->blockSignals(false);
        m_alignedCheckBoxHeader->update();
    }
    else if (numsum == currentRowCount)
    {
        m_alignedCheckBoxHeader->blockSignals(true);
        m_alignedCheckBoxHeader->setCheckState(true);
        m_alignedCheckBoxHeader->blockSignals(false);
        m_alignedCheckBoxHeader->update();
    }
    if (findindex >= 0 && refPointsTableWidget->rowCount() > findindex)
    {
        QWidget * itemwidget = refPointsTableWidget->cellWidget(findindex, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            box.at(0)->setChecked(checkbox->isChecked());
        }
    }
}

void ccPointPairRegistrationDlg::slotRefTableCheckStateChanged(int index)
{
    QCheckBox * checkbox = qobject_cast<QCheckBox*>(sender());
    int findindex = -1;
    onPointCountChanged();
    int numsum = 0;
    int currentRowCount = refPointsTableWidget->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = refPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
            if (box.at(0) == checkbox)
            {
                findindex = i;
            }
        }
        if (ischecked)
        {
            numsum++;
        }
    }
    if (numsum == 0)
    {
        m_refCheckBoxHeader->blockSignals(true);
        m_refCheckBoxHeader->setCheckState(false);
        m_refCheckBoxHeader->blockSignals(false);
        m_refCheckBoxHeader->update();
    }
    else if (numsum == currentRowCount)
    {
        m_refCheckBoxHeader->blockSignals(true);
        m_refCheckBoxHeader->setCheckState(true);
        m_refCheckBoxHeader->blockSignals(false);
        m_refCheckBoxHeader->update();
    }
    if (findindex >= 0 && alignedPointsTableWidget->rowCount() > findindex)
    {
        QWidget * itemwidget = alignedPointsTableWidget->cellWidget(findindex, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            box.at(0)->setChecked(checkbox->isChecked());
        }
    }
}

void ccPointPairRegistrationDlg::slotCloudSourseChanged(int index)
{
    comboBox_pointtarget->setCurrentIndex(1 - index);
    int currentUpRowCount = alignedPointsTableWidget->rowCount();
    std::vector<std::vector<QString>> uptableData;
    std::vector<bool> uptableCheckState;
    std::vector<std::vector<QString>> downtableData;
    std::vector<bool> downtableCheckState;
    for (int i = 0; i < currentUpRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = alignedPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
        }
        std::vector<QString> currowdata;
        currowdata.push_back(alignedPointsTableWidget->item(i, 1)->text());
        currowdata.push_back(alignedPointsTableWidget->item(i, 2)->text());
        currowdata.push_back(alignedPointsTableWidget->item(i, 3)->text());
        currowdata.push_back(alignedPointsTableWidget->item(i, 4)->text());
        uptableData.push_back(currowdata);
        uptableCheckState.push_back(ischecked);
    }

    int currentDownRowCount = refPointsTableWidget->rowCount();
    for (int i = 0; i < currentDownRowCount; i++)
    {
        bool ischecked = false;
        QWidget * itemwidget = refPointsTableWidget->cellWidget(i, 0);
        QList<QCheckBox*> box = itemwidget->findChildren<QCheckBox*>();
        if (box.size() > 0)
        {
            ischecked = box.at(0)->isChecked();
        }
        std::vector<QString> currowdata;
        currowdata.push_back(refPointsTableWidget->item(i, 1)->text());
        currowdata.push_back(refPointsTableWidget->item(i, 2)->text());
        currowdata.push_back(refPointsTableWidget->item(i, 3)->text());
        currowdata.push_back(refPointsTableWidget->item(i, 4)->text());
        downtableData.push_back(currowdata);
        downtableCheckState.push_back(ischecked);
    }
    m_refPoints.removeAllChildren();
    m_alignedPoints.removeAllChildren();
    for (auto iter = currentUpRowCount - 1; iter >= 0; iter--)
    {
        alignedPointsTableWidget->removeRow(iter);
    }

    for (auto iter = currentDownRowCount - 1; iter >= 0; iter--)
    {
        refPointsTableWidget->removeRow(iter);
    }
    std::swap(m_alignedEntities, m_referenceEntities);
    bool isCloudVisible = (checkBox_align->isChecked() ? true : false);
    for (EntityContext& ec : m_alignedEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            ec.entity->setVisible(isCloudVisible);
            if (m_bCurrentModelCombination)
            {
                ec.entity->m_displayBoundingBox = false;
            }
            else
            {
                ec.entity->m_displayBoundingBox = isCloudVisible;
            }
        }
    }
    isCloudVisible = (checkBox_ref->isChecked() ? true : false);
    for (EntityContext& ec : m_referenceEntities)
    {
        if (ec.entity && ec.entity->isA(CC_TYPES::POINT_CLOUD))
        {
            ec.entity->setVisible(isCloudVisible);
            if (m_bCurrentModelCombination)
            {
                ec.entity->m_displayBoundingBox = false;
            }
            else
            {
                ec.entity->m_displayBoundingBox = isCloudVisible;
            }
        }
    }
    m_isTableDataChangeLock = true;
    alignedPointsTableWidget->setRowCount(downtableData.size());
    for (int i = 0; i < downtableData.size(); i++)
    {

        QCheckBox* box = new QCheckBox();
        box->setFixedSize(16, 16);
        QWidget *widget = new QWidget();
        QHBoxLayout *layout = new QHBoxLayout();
        layout->setMargin(0);//一定要有
        layout->addWidget(box);
        layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
        widget->setLayout(layout);
        box->setChecked(downtableCheckState[i]);
        connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotAlignTableCheckStateChanged);
        alignedPointsTableWidget->setCellWidget(i, 0, widget);
        updateTableWidgetCheckState(alignedPointsTableWidget);
        QTableWidgetItem* labelitem = new QTableWidgetItem(downtableData[i][0]);
        labelitem->setTextAlignment(Qt::AlignCenter);
        alignedPointsTableWidget->setItem(i, 1, labelitem);

        for (int d = 0; d < 3; ++d)
        {
            QTableWidgetItem* item = new QTableWidgetItem();
            item->setData(Qt::EditRole, downtableData[i][d + 1]);
            item->setTextAlignment(Qt::AlignCenter);
            alignedPointsTableWidget->setItem(i, XYZ_COL_INDEX + d, item);
            alignedPointsTableWidget->setRowHeight(i, 32);
        }
        QTableWidgetItem* itemError = new QTableWidgetItem();
        itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
        itemError->setTextAlignment(Qt::AlignCenter);
        alignedPointsTableWidget->setItem(i, XYZ_COL_INDEX + 3, itemError);
        int lastcurrentRow = alignedPointsTableWidget->rowCount();
        if (alignedPointsTableWidget->rowCount() > 4)
        {
            alignedPointsTableWidget->setColumnWidth(5, 128);
        }
    }

    refPointsTableWidget->setRowCount(uptableData.size());
    for (int i = 0; i < uptableData.size(); i++)
    {
        QCheckBox* box = new QCheckBox();
        box->setFixedSize(16, 16);
        QWidget *widget = new QWidget();
        QHBoxLayout *layout = new QHBoxLayout();
        layout->setMargin(0);//一定要有
        layout->addWidget(box);
        layout->setAlignment(box, Qt::AlignCenter);//控件在布局中居中显示
        widget->setLayout(layout);
        box->setChecked(uptableCheckState[i]);
        connect(box, &QCheckBox::stateChanged, this, &ccPointPairRegistrationDlg::slotRefTableCheckStateChanged);
        refPointsTableWidget->setCellWidget(i, 0, widget);
        QTableWidgetItem* labelitem = new QTableWidgetItem(uptableData[i][0]);
        labelitem->setTextAlignment(Qt::AlignCenter);
        refPointsTableWidget->setItem(i, 1, labelitem);
        updateTableWidgetCheckState(refPointsTableWidget);
        for (int d = 0; d < 3; ++d)
        {
            QTableWidgetItem* item = new QTableWidgetItem();
            item->setData(Qt::EditRole, uptableData[i][d + 1]);
            item->setTextAlignment(Qt::AlignCenter);
            refPointsTableWidget->setItem(i, XYZ_COL_INDEX + d, item);
            refPointsTableWidget->setRowHeight(i, 32);
        }
        QTableWidgetItem* itemError = new QTableWidgetItem();
        itemError->setFlags(itemError->flags() & ~Qt::ItemIsEditable);
        itemError->setTextAlignment(Qt::AlignCenter);
        refPointsTableWidget->setItem(i, XYZ_COL_INDEX + 3, itemError);
        int lastcurrentRow = refPointsTableWidget->rowCount();
        if (refPointsTableWidget->rowCount() > 4)
        {
            refPointsTableWidget->setColumnWidth(5, 128);
        }
    }
    m_isTableDataChangeLock = false;
    updateAlignPlontAndLabel();
    updateRefPlontAndLabel();
    onPointCountChanged();

    bool isalignpointvisiavle = checkBox_align->isChecked();
    int alignlbelnum = m_alignedPoints.getChildrenNumber();
    for (int i = 0; i < alignlbelnum; i++)
    {
        cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_alignedPoints.getChild(i));
        if (label)
        {
            label->setVisible(isalignpointvisiavle);
        }
    }
    int refalbelnum = m_refPoints.getChildrenNumber();
    bool isrefpointvisiavle = checkBox_ref->isChecked();
    for (int i = 0; i < refalbelnum; i++)
    {
        cc2DLabel* label = dynamic_cast<cc2DLabel*>(m_refPoints.getChild(i));
        if (label)
        {
            label->setVisible(isrefpointvisiavle);
        }
    }
    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}

void ccPointPairRegistrationDlg::slotRadioButtonClicked()
{
    for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    m_alignedPoints.enableGLTransformation(false);
    for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    m_refPoints.enableGLTransformation(false);
    resetToolButton->setEnabled(false);
    if (radioButton->isChecked())
    {
        stackedWidget->setCurrentIndex(0);
        updateAlignPlontAndLabel();
        updateRefPlontAndLabel();
        onPointCountChanged();
        setFixedHeight(820);
    }
    else
    {
        label_RMSE->setText(tr("Final RMS:") + QString::number(0));
        stackedWidget->setCurrentIndex(1);
        m_alignedPoints.removeAllChildren();
        m_alignedPoints.clear();
        m_refPoints.removeAllChildren();
        m_refPoints.clear();
        checkMatrixData();
        setFixedHeight(642);
    }
    if (m_associatedWin)
    {
        m_associatedWin->redraw();
    }
}

void ccPointPairRegistrationDlg::clearMatrixData()
{
    for (int i = 2;i < 19;i++)
    {
        QString objname = "doubleSpinBox_" + QString::number(i);
        QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
        if (foundBox)
        {
            foundBox->clear();
        }
    }
}

void ccPointPairRegistrationDlg::updateMatrixData()
{
    for (int i = 2; i < 18; i++)
    {
        QString objname = "doubleSpinBox_" + QString::number(i);
        QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
        if (foundBox)
        {
            foundBox->setValue(m_TransformationMatrix[i-2]);
        }
    }
    doubleSpinBox_18->setValue(m_Scalefactor);
    checkMatrixData();
}

bool ccPointPairRegistrationDlg::checkMatrixData()
{
    for (int i = 2; i < 19; i++)
    {
        QString objname = "doubleSpinBox_" + QString::number(i);
        QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
        if (foundBox)
        {
            QString textstr = foundBox->text();
            if (foundBox->text().isEmpty())
            {
                alignToolButton->setEnabled(false);
                validToolButton->setEnabled(false);
                return false;
            }
        }
    }
    alignToolButton->setEnabled(true);
    validToolButton->setEnabled(true);
    return true;
}

ccGLMatrixd ccPointPairRegistrationDlg::getGLMatrixData()
{
    double* mat16d = new double[16];
    for (int i = 2; i < 18; i++)
    {
        QString objname = "doubleSpinBox_" + QString::number(i);
        QDoubleSpinBox *foundBox = stackedWidget->findChild<QDoubleSpinBox*>(objname);
        if (foundBox)
        {
            mat16d[i - 2] = foundBox->value();
        }
    }
    ccGLMatrixd mat(mat16d);
    delete[] mat16d;
    return mat;
}

void ccPointPairRegistrationDlg::getFirstEmptyIndex(int & index, QTableWidget * table)
{
    int currentRowCount = table->rowCount();
    for (int i = 0; i < currentRowCount; i++)
    {
        if (table->item(i, 2)->text().isEmpty() && table->item(i, 3)->text().isEmpty() && table->item(i, 4)->text().isEmpty())
        {
            index = i;
            return;
        }
    }
}

void ccPointPairRegistrationDlg::clearChangeRegistrationType()
{
    //[!]reset

    if (m_alignedEntities.empty())
        return;

    for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
        ccHObject* pObj = dynamic_cast<ccHObject*>(it.key());
        if (pObj) {
        
        }

    }
    m_alignedPoints.enableGLTransformation(false);
    for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
    {
        it.key()->enableGLTransformation(false);
    }
    m_refPoints.enableGLTransformation(false);
    if (stackedWidget->currentIndex() == 0)
    {
        updateAlignInfo();
    }

    bool canAlign = getCanAlign();
    alignToolButton->setEnabled(canAlign);
    resetToolButton->setEnabled(false);
    m_isAlreadyAlign = false;

    //[!]clear

    alignToolButton->setEnabled(false);
    validToolButton->setEnabled(false);

    while (alignedPointsTableWidget->rowCount() != 0)
        alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount() - 1);
    while (refPointsTableWidget->rowCount() != 0)
        refPointsTableWidget->removeRow(refPointsTableWidget->rowCount() - 1);

    clearMatrixData();
    //[!]判断裁切盒的当前状态
    if (recordCombinationWhether) {
        emit signalReset();
    }

    MainWindow::TheInstance()->setGlobalZoom();

}

icpRegistration::icpRegistration( QWidget* parent/*=nullptr*/)
    :CS::MetahubWidgets::MetahubFramelessDialog(parent)
{
    
    QHBoxLayout *pMainLayout = new QHBoxLayout;
    QVBoxLayout *pLeftLayout = new QVBoxLayout;
    QVBoxLayout *pRightLayout = new QVBoxLayout;

    QLabel *pIcpText = new QLabel(this);
    QLabel *pIcpValue = new QLabel(this);
    pLeftLayout->addWidget(pIcpText);
    pLeftLayout->addWidget(pIcpValue);
    pLeftLayout->setSpacing(16);
    QHBoxLayout *pCheckBoxLayout = new QHBoxLayout;
    m_pOpenCheckBox = new QCheckBox(this);
    m_pCloseCheckBox = new QCheckBox(this);
    QButtonGroup * radioButtonGroup1 = new QButtonGroup(this);
    radioButtonGroup1->addButton(m_pOpenCheckBox);
    radioButtonGroup1->addButton(m_pCloseCheckBox);
    m_pOpenCheckBox->setChecked(true);
    m_pCloseCheckBox->setChecked(false);
    m_pSpinBox = new QDoubleSpinBox(this);
    m_pSpinBox->setDecimals(4);
    m_pSpinBox->setSingleStep(0.0001);
    m_pSpinBox->setValue(0.0001);
    m_pSpinBox->setRange(0, 0.001);
    pCheckBoxLayout->addWidget(m_pOpenCheckBox);
    pCheckBoxLayout->addWidget(m_pCloseCheckBox);
    pCheckBoxLayout->addStretch();
    pCheckBoxLayout->setSpacing(48);
    pRightLayout->addLayout(pCheckBoxLayout);
    pRightLayout->addWidget(m_pSpinBox);
    m_pSpinBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pRightLayout->setSpacing(16);
    pMainLayout->addLayout(pLeftLayout);
    pMainLayout->addLayout(pRightLayout);
    pMainLayout->setSpacing(48);
    pMainLayout->setContentsMargins(16, 24, 16, 24);
    setLayout(pMainLayout);
    this->setFixedSize(576, 232);
    pIcpText->setText(QApplication::translate("ccPointPairRegistrationDlg", "ICP Registration", nullptr));
    pIcpValue->setText(QApplication::translate("ccPointPairRegistrationDlg", "Iterative Thresholding", nullptr));

    m_pOpenCheckBox->setText(QApplication::translate("ccPointPairRegistrationDlg", "Enable",nullptr));
    m_pCloseCheckBox->setText(QApplication::translate("ccPointPairRegistrationDlg", "Disable",nullptr));
    setWindowTitle(QApplication::translate("ccPointPairRegistrationDlg", "Calculation Settings", nullptr));

}
icpRegistration::~icpRegistration()
{
    close();
}

std::vector<double> icpRegistration::getIpcParameters()
{
    std::vector<double>vec;
    if (m_pOpenCheckBox->isChecked()){
        vec.push_back(1);
    }
    else
    {
        vec.push_back(0);
    }
    vec.push_back(m_pSpinBox->value());
    return vec;

}

void icpRegistration::setIcpParameters(std::vector<double>parameter)
{
    if (parameter.size() == 2) {
        static_cast<int>(parameter.at(0)) ? m_pOpenCheckBox->setChecked(true) : m_pCloseCheckBox->setChecked(true);
        m_pSpinBox->setValue(parameter.at(1));
    }
}
