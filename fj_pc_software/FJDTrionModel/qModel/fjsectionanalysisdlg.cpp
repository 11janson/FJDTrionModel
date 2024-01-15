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

#include "fjsectionanalysisdlg.h"

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


//Qt
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QToolButton>
#include <QDoubleSpinBox>
#include <QAxObject>
#include <QComboBox >
#include <QDebug>
#include <QStandardPaths>
#include <QTableWidget>
#include "FJStyleManager.h"
#include "ccDBRoot.h"
#include "ccHObjectCaster.h"
#include "fjsectionanalysisrenderwidget.h"
#include <algorithm>
#include <QPushButton>
#include <QToolButton>
#include "ccSectionAnalysisLine.h"
#include "titlebar.h"
#include "ccReservedIDs.h"
#include "ccGenericGLDisplay.h"
#include <QDoubleSpinBox>
#include <QAction>
#include <QLabel>
#include <QPixmap>
#include "framelessmessagebox.h"
#include "cloudcomparewidgets\metahubframelesswaitdialog.h"
#include <QtConcurrent>
//计算两条线段是否相交
bool isTwoLineIntersect(QPointF a, QPointF b, QPointF c, QPointF d)
{
    double minnum = 1e-15;
    double area_abc = (a.x() - c.x()) * (b.y() - c.y()) - (a.y() - c.y()) * (b.x() - c.x());
    double area_abd = (a.x() - d.x()) * (b.y() - d.y()) - (a.y() - d.y()) * (b.x() - d.x());
    if (area_abc * area_abd >= -minnum)
    {
        return false;
    }
    double area_cda = (c.x() - a.x()) * (d.y() - a.y()) - (c.y() - a.y()) * (d.x() - a.x());
    double area_cdb = area_cda + area_abc - area_abd;
    if (area_cda * area_cdb >= -minnum)
    {
        return false;
    }
    return true;
}

static void setActionIcon(QToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
    QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + normalPix + ".png");
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + clickedPix + ".png"), QIcon::Active, QIcon::Off);
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + disabledPix + ".png"), QIcon::Disabled, QIcon::Off);
    action->setIcon(pIcon);
}

static void SetButtonColorSelected(QAbstractButton* button)
{
    if (button != nullptr)
    {
        button->setStyleSheet("QToolButton{background-color:#585858;color:#1c1c1c;border:0px solid #989898;}\n"
            "QToolButton:hover{background-color:rgb(100,100,100);color:#1c1c1c;border:0px solid #989898;}\n"
            "QToolButton:disabled{background-color:#585858;color:#1c1c1c;border: 0px solid #666666;}");
    }
}

FJSectionAnalysisDlg::FJSectionAnalysisDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent/*=nullptr*/)
    : CS::Widgets::FramelessDialog(parent)
    , m_pickingHub(pickingHub)
    , m_app(app), m_Line(nullptr)
{
    assert(m_pickingHub);
    setupUi(this->getContentHolder());
    m_layout = new QVBoxLayout();
    m_layout->setContentsMargins(18, 2, 0, 0);
    m_layout->setSpacing(0);
    widget_list->setLayout(m_layout);
    setStyleSheet("background-color: #2B2B2B;");
    //listWidget->setColumnWidth(0,0);
    listWidget->setColumnWidth(0, 180);
    listWidget->setColumnWidth(1, 70);
    listWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    this->bottomWidget()->setVisible(false);
    QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/minimize.png");
    this->getTitleBar()->setTitleButtonIcon(1, pIcon);
    QIcon pIcons(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png");//btn_max.png
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png"), QIcon::Active, QIcon::Off);
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/buttonmaxicon.png"), QIcon::Disabled, QIcon::Off);
    this->getTitleBar()->setTitleButtonIcon(2, pIcons);
    QIcon closeicon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/smallwindowcloseicon.png");
    this->getTitleBar()->setTitleButtonIcon(4, closeicon);
    this->setMinButtonVisible(true);
    // 显示最大化按钮
    this->setMaxButtonVisible(true);
    this->setDialogResizable(true);
    this->setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 16px;\n"
        "color: #F2F2F2;\n"
        "background-color: #2B2B2B;\n"
        "line - height: 24px;\n");
    this->setTtitleBarButtonStyle(CS::Widgets::TitleBar::PredefinedButtonType::MinButton, "background-color:#2B2B2B;");
    this->setTtitleBarButtonStyle(CS::Widgets::TitleBar::PredefinedButtonType::MaxButton, "background-color:#2B2B2B;");
    this->setTtitleBarButtonStyle(CS::Widgets::TitleBar::PredefinedButtonType::CloseButton, "background-color:#2B2B2B;");
    this->setTtitleBarButtonStyle(CS::Widgets::TitleBar::PredefinedButtonType::RestoreButton, "background-color:#2B2B2B;");
    SetButtonColorSelected(toolButton_view3d);
    SetButtonColorSelected(toolButton_viewtop);
    SetButtonColorSelected(toolButton_viewface);
    setActionIcon(toolButton_view3d, "SectionAnalysis3DSelect", "SectionAnalysis3DSelect", "SectionAnalysis3DSelect");
    setActionIcon(toolButton_viewtop, "SectionAnalysisTopNormal", "SectionAnalysisTopNormal", "SectionAnalysisTopDisable");
    setActionIcon(toolButton_viewface, "SectionAnalysisFrontNormal", "SectionAnalysisFrontNormal", "SectionAnalysisFrontDisable");
    widget->setStyleSheet("background-color: rgb(63, 63, 63);");
    setWindowTitle(tr("Profile Analysis"));
    connect(this->getTitleBar()->GetButtonByType(2), &QToolButton::clicked, this, &FJSectionAnalysisDlg::slotUpdateWindowSize);
    connect(this->getTitleBar()->GetButtonByType(3), &QToolButton::clicked, this, &FJSectionAnalysisDlg::slotUpdateWindowSize);
    resize(1250,800);
}

void FJSectionAnalysisDlg::slotUpdateWindowSize()
{
    for (auto curcheckbox : m_checkBoxs)
    {
        if (curcheckbox)
        {
            curcheckbox->setMaximumWidth(widget->width()/3.0 - 12);
        }
    }
    update();
}

FJSectionAnalysisDlg::~FJSectionAnalysisDlg()
{
    qDebug() << "SectionAnalysisDlgclose34";
    //stop(true);
}


static void SetButtonColor(QLabel* button, const QColor &col)
{
    if (button != nullptr)
    {
        button->setStyleSheet(QStringLiteral("* { background-color: rgb(%1,%2,%3) }")
            .arg(col.red())
            .arg(col.green())
            .arg(col.blue())
        );
    }
}



bool FJSectionAnalysisDlg::linkWith(ccGLWindow* win)
{
    if (m_win)
    {
        QWidgetList topWidgets = QApplication::topLevelWidgets();
        foreach(QWidget* subwidget, topWidgets)
        {
            if (subwidget)
            {
                subwidget->installEventFilter(this);
            }
        }
    }
    connect(m_win, &ccGLWindow::mouseMoved, this, &FJSectionAnalysisDlg::updateMousePos);
    connect(m_win, &ccGLWindow::rightButtonNoMovedRelease, this, &FJSectionAnalysisDlg::stopPickPoint);
    connect(m_win, &ccGLWindow::leftButtonNoMovedRelease, this, &FJSectionAnalysisDlg::addPointToline);
    //m_associatedWin->addToOwnDB(&m_alignedPoints);
    //m_associatedWin->addToOwnDB(&m_refPoints);

    return true;
}

void FJSectionAnalysisDlg::updateMousePos(int x, int y, Qt::MouseButtons buttons)
{
    if (m_Line)
    {
        if (m_win && m_Line->getMousePosVisiable())
        {
            QPointF pos2D = m_win->toCenteredGLCoordinates(x, y);
            CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
                static_cast<PointCoordinateType>(pos2D.y()),
                0);
            ccGLCameraParameters camera;
            m_win->getGLCameraParameters(camera);
            const double half_w = camera.viewport[2] / 2.0;
            const double half_h = camera.viewport[3] / 2.0;
            CCVector3d Q3D;
            CCVector3d ppp(P.x + half_w, P.y + half_h, P.z);
            bool pointInFrustum = false;
            camera.unproject(ppp, Q3D);
            std::vector<CCVector3d> point3d;
            m_Line->getCurrentData(point3d);
            if (point3d.size() > 0)
            {
                std::vector < CCVector3d>  points2d;
                CCVector3 P3D = point3d[point3d.size() - 1].toFloat();
                CCVector3d P2D;
                camera.project(P3D, P2D);
                points2d.push_back(P2D);
                camera.project(Q3D, P2D);
                points2d.push_back(P2D);

                m_Line->setMousePos(points2d);
                m_win->redraw(true, true);
            }
        }
    }
}

void FJSectionAnalysisDlg::closeEvent(QCloseEvent *event)
{
    CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question, tr("Quit"), tr("All profiles and measurements will be cleared. Are you sure you want to close the window?"), QMessageBox::Cancel | QMessageBox::Ok, this);
    if (message_box.exec() != QMessageBox::Ok)
    {
        event->ignore();
        raise();
        activateWindow();
        return;
    }
    emit processFinshed(true);
    QWidget::closeEvent(event);
}

void FJSectionAnalysisDlg::updateThick()
{
    int index = listWidget->currentRow();
    if (index < 0)
    {
        return;
    }
    QString cloudName = listWidget->item(index, 0)->text();
    calculateSectionAnalysisData(cloudName, doubleSpinBox_thickness->value(), doubleSpinBox_offset->value(), false, comboBox->currentText(),false);
}

bool FJSectionAnalysisDlg::start()
{
    if (!m_pickingHub->addListener(this, true))
    {
        ccLog::Error("Picking mechanism is already in use! Close the other tool first, and then restart this one.");
        return false;
    }
    if (m_win)
    {
        ccGui::ParamStruct param = ccGui::Parameters();
        param.m_isSectionAnalysis = true;
        ccGui::Set(param);
        setViewChangeButtonEnable(true);
        toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measurecloseicondisable.png"));
        toolButton->setDisabled(true);
        doubleSpinBox_offset->setDisabled(true);
        pushButton->setDisabled(true);
        doubleSpinBox_offset->setValue(0);
        doubleSpinBox_thickness->setDisabled(true);
        doubleSpinBox_thickness->setValue(0.1);
        widget_list->setVisible(false);
        label_txt->setText("");
        setViewUp();
        m_win->setPickingMode(ccGLWindow::PICKING_MODE::POINT_PICKING);
        //widget->setVisible(true);
        //m_win->setInteractionMode(ccGLWindow::MODE_SECTIONANALYSIS);
        connect(listWidget, &QTableWidget::currentItemChanged, this, &FJSectionAnalysisDlg::slotCurrentItemSelectChanged);
        connect(widget, &FJSectionAnalysisRenderWidget::updateMessage, this, &FJSectionAnalysisDlg::updateLabelText);
        connect(widget, &FJSectionAnalysisRenderWidget::finshedPickPoint, this, &FJSectionAnalysisDlg::onFinshedPickPoint);
        connect(doubleSpinBox_offset, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FJSectionAnalysisDlg::onThicknessOrOffsetChanged);
        connect(doubleSpinBox_thickness, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &FJSectionAnalysisDlg::onThicknessOrOffsetChanged);
        connect(pushButton, &QPushButton::clicked, this, &FJSectionAnalysisDlg::updateThick);
        connect(toolButton, &QToolButton::clicked, this, &FJSectionAnalysisDlg::startMeasure);
        connect(toolButton_view3d, &QToolButton::clicked, this, &FJSectionAnalysisDlg::setView3d);
        connect(toolButton_viewtop, &QToolButton::clicked, this, &FJSectionAnalysisDlg::setViewUp);
        connect(toolButton_viewface, &QToolButton::clicked, this, &FJSectionAnalysisDlg::setViewFace);
        connect(checkBox, &QCheckBox::stateChanged, this, &FJSectionAnalysisDlg::updatePointColor);
        checkBox->setToolTip(tr("Click to turn on."));
        connect(FJStyleManager::Instance(), &FJStyleManager::SectionThicknessUpdate, this, &FJSectionAnalysisDlg::onSectionThicknessUpdate);
        if (!m_Line)
        {
            m_Line = new ccSectionAnalysisLine(QString("profilecurrent"));
            m_object->addChild(m_Line);
        }
        m_Line->setVisible(true);
        m_Line->setProjectionMode(m_ProjectionMode);
        if (m_ProjectionMode == LOOKDOWN)
        {
            m_Line->setRangeParam(m_heightMin, m_heightMax);
        }
        else
        {
            m_Line->setRangeParam(m_heightMinY, m_heightMaxY);
        }
        m_Line->setCurrentColor(Qt::red);
        comboBox->clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        MainWindow::TheInstance()->setViewButtonDisabled(true, true);
        QCoreApplication::processEvents();
		show();
		showNormal();
		return true;
	}
	return false;
}

void FJSectionAnalysisDlg::onFinshedPickPoint(QPointF start, QPointF end)
{
    int index = listWidget->currentRow();
    if (index < 0)
    {
        return;
    }
    QString cloudName = listWidget->item(index, 0)->text();
    int childNum = m_object->getChildrenNumber();
    for (int i = childNum - 1; i > 0; i--)
    {
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        if (label)
        {
            if (label->getName() == cloudName)
            {
                label->setPickPoint(start, end);
                label->setIsPickPoint(true);
            }

        }
    }
}

QString FJSectionAnalysisDlg::getCurrentSectionName()
{
    int index = listWidget->currentRow();
    if (index < 0)
    {
        return "";
    }
    QString cloudName = listWidget->item(index, 0)->text();
    return cloudName;
}

void FJSectionAnalysisDlg::updateLabelText(QString str)
{
    label_txt->setText(str);
}

void FJSectionAnalysisDlg::startMeasure()
{
    QString name = getCurrentSectionName();
    int childNum = m_object->getChildrenNumber();
    for (int i = childNum - 1; i > 0; i--)
    {
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        if (label)
        {
            if (label->getName() == name)
            {
                bool ismeasureopen = label->getIsMeasureOpen();
                label->setIsMeasureOpen(!ismeasureopen);
                widget->startMeasure(!ismeasureopen);
                if (!ismeasureopen)
                {
                    toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measureopenicon.png"));
                }
                else
                {
                    toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measurecloseiconclose.png"));
                }
            }

        }
    }

}

void FJSectionAnalysisDlg::stop(bool state)
{
    if (m_win)
    {
        qDebug() << "SectionAnalysisDlgclose1";
        QWidgetList topWidgets = QApplication::topLevelWidgets();
        qDebug() << "SectionAnalysisDlgclose2";
        foreach(QWidget* subwidget, topWidgets)
        {
            if (subwidget)
            {
                qDebug() << "SectionAnalysisDlgclose21";
                subwidget->removeEventFilter(this);
                qDebug() << "SectionAnalysisDlgclose22";
            }
        }
        qDebug() << "SectionAnalysisDlgclose3";
        m_pickingHub->removeListener(this);
        qDebug() << "SectionAnalysisDlgclose4";
        m_win->disconnect(this);
        qDebug() << "SectionAnalysisDlgclose5";
        m_win->setPickingMode(ccGLWindow::PICKING_MODE::NO_PICKING);
        qDebug() << "SectionAnalysisDlgclose6";
        m_win->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
        qDebug() << "SectionAnalysisDlgclose7";
        ccGui::ParamStruct param = ccGui::Parameters();
        param.m_isSectionAnalysis = false;
        ccGui::Set(param);
    }
    qDebug() << "SectionAnalysisDlgclose8";
    int childNum = m_object->getChildrenNumber();
    qDebug() << "SectionAnalysisDlgclose9";
    for (int i = childNum - 1; i >= 0; i--)
    {
        qDebug() << "SectionAnalysisDlgclose91";
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        qDebug() << "SectionAnalysisDlgclose92";
        if (label)
        {
            qDebug() << "SectionAnalysisDlgclose93";
            m_object->removeChild(i);
            qDebug() << "SectionAnalysisDlgclose94";
        }
    }
    m_Line = nullptr;
    qDebug() << "SectionAnalysisDlgclose10";
    m_SectionAnalysisData.clear();
    qDebug() << "SectionAnalysisDlgclose11";
    m_CurrentPointData.clear();
    qDebug() << "SectionAnalysisDlgclose12";
    bool isok = listWidget->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose13";
    listWidget->clear();
    qDebug() << "SectionAnalysisDlgclose14";
    m_currentSectionNum = -1;
    m_currentSectionVerticalNum = -1;
    m_currentSectionHorizontalNum = -1;
    listWidget->setRowCount(0);
    qDebug() << "SectionAnalysisDlgclose15";
    m_selectedObjectList.clear();
    qDebug() << "SectionAnalysisDlgclose16";
    widget->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose17";
    doubleSpinBox_offset->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose18";
    toolButton->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose19";
    doubleSpinBox_thickness->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose20";
    isok = pushButton->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose21";
    toolButton_view3d->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose22";
    toolButton_viewtop->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose23";
    toolButton_viewface->disconnect(this);
    qDebug() << "SectionAnalysisDlgclose24";
    disconnect(FJStyleManager::Instance(), &FJStyleManager::SectionThicknessUpdate, this, &FJSectionAnalysisDlg::onSectionThicknessUpdate);
    qDebug() << "SectionAnalysisDlgclose25";
    disconnect();
    qDebug() << "SectionAnalysisDlgclose26";
    SectionAnalysisData renderData;
    qDebug() << "SectionAnalysisDlgclose27";
    widget->setData(renderData);
    qDebug() << "SectionAnalysisDlgclose28";
    MainWindow::TheInstance()->setViewButtonDisabled(false, true);
    //linkWith(nullptr);
}


bool FJSectionAnalysisDlg::init(ccGLWindow* win, const ccHObject::Container& alignedEntities)
{

    if (!win)
    {
        return false;
    }
    m_win = win;

    if (alignedEntities.empty())
    {
        ccLog::Error("[PointPairRegistration] Need at least one aligned entity!");
        return false;
    }
    m_object = alignedEntities[0];
    m_selectedObjectList = alignedEntities;
    widget->setCurrentObjects(alignedEntities);
    calculateWidthAndHeightRange();
    ccGenericGLDisplay* sourceDisplay = m_object->getDisplay();
    if (sourceDisplay)
    {
        ccGlFilter* filter = static_cast<ccGLWindow*>(sourceDisplay)->getGlFilter();
        if (filter)
            win->setGlFilter(filter->clone());
    }
    linkWith(win);
    return true;
}

void FJSectionAnalysisDlg::addPointToline(int x, int y)
{
    //if (isActiveWindow())
    //{
    //	return;
    //}
    if (m_viewMode == VIEW3DMODE)
    {
        return;
    }
    if (!m_win)
    {
        return;
    }
    if ((x < 0 || y < 0 || x >= m_win->qtWidth() || y >= m_win->qtHeight()))
    {
        return;
    }
    QPointF pos2D = m_win->toCenteredGLCoordinates(x, y);
    CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()), 0);
    ccGLCameraParameters camera;
    m_win->getGLCameraParameters(camera);
    const double half_w = camera.viewport[2] / 2.0;
    const double half_h = camera.viewport[3] / 2.0;
    CCVector3d Q3D;
    CCVector3d ppp(P.x + half_w, P.y + half_h, P.z);
    camera.unproject(ppp, Q3D);
    if (m_viewMode == TOPVIEWMODE)
    {
        Q3D.z = m_heightMax;
    }
    if (m_viewMode == FRONTVIEWMODE)
    {
        Q3D.y = m_heightMaxY;
    }
    std::vector<QColor> colorData{ QColor(255,0,0,0),QColor(0,255,0,0) ,QColor(0,0,255,0) ,QColor(255,255,0,0) ,QColor(0,255,255,0),QColor(255,0,255,0),QColor(192,192,192,0) ,QColor(128,128,128,0) ,QColor(128,0,0,0) ,QColor(128,128,0,0),QColor(0,128,0,0),QColor(128,0,128,0) ,QColor(0,128,128,0) ,QColor(0,0,128,0) };
    if (m_CurrentPointData.size() == 0)
    {
        setViewChangeButtonEnable(false);
        m_CurrentPointData.push_back(Q3D);
        if (m_Line)
        {
            m_Line->setCurrentData(m_CurrentPointData);
            int index = (m_currentSectionNum + 1) % 14;
            m_Line->setCurrentColor(colorData[index]);
            m_Line->setMousePosVisiable(true);
            std::vector<CCVector3d> emptydata;
            m_Line->setMousePos(emptydata);
        }
    }
    else
    {
        m_CurrentPointData.push_back(Q3D);
        m_Line->setCurrentData(m_CurrentPointData);
    }
    m_win->redraw();
}

void FJSectionAnalysisDlg::onItemPicked(const PickedItem& pi)
{
    //if (isActiveWindow())
    //{
    //	return;
    //}
    if (!m_win)
        return;

    if (!pi.entity)
        return;
    if (m_viewMode != VIEW3DMODE)
    {
        return;
    }
    //if (pi.entity == m_object)
    {
        std::vector<QColor> colorData{ QColor(255,0,0,0),QColor(0,255,0,0) ,QColor(0,0,255,0) ,QColor(255,255,0,0) ,QColor(0,255,255,0),QColor(255,0,255,0),QColor(192,192,192,0) ,QColor(128,128,128,0) ,QColor(128,0,0,0) ,QColor(128,128,0,0),QColor(0,128,0,0),QColor(128,0,128,0) ,QColor(0,128,128,0) ,QColor(0,0,128,0) };
        CCVector3d P = pi.P3D.toDouble();
        if (m_CurrentPointData.size() == 0)
        {
            m_win->setInteractionMode(ccGLWindow::MODE_SEGMENT);
            setViewChangeButtonEnable(false);
            m_CurrentPointData.push_back(P);
            if (m_Line)
            {
                m_Line->setCurrentData(m_CurrentPointData);
                int index = (m_currentSectionNum + 1) % 14;
                m_Line->setCurrentColor(colorData[index]);
                m_Line->setMousePosVisiable(true);
                std::vector<CCVector3d> emptydata;
                m_Line->setMousePos(emptydata);
            }
        }
        else
        {
            m_CurrentPointData.push_back(P);
            m_Line->setCurrentData(m_CurrentPointData);
        }
    }
    m_win->redraw();

}

void FJSectionAnalysisDlg::stopPickPoint()
{
    if (!m_win)
    {
        return;
    }
    std::vector<QColor> colorData{ QColor(255,0,0,255),QColor(0,255,0,255) ,QColor(0,0,255,255) ,QColor(255,255,0,255) ,QColor(0,255,255,255),QColor(255,0,255,255),QColor(192,192,192,255) ,QColor(128,128,128,255) ,QColor(128,0,0,255) ,QColor(128,128,0,255),QColor(0,128,0,255),QColor(128,0,128,255) ,QColor(0,128,128,255) ,QColor(0,0,128,255) };
    if (m_CurrentPointData.size() > 1)
    {
        m_currentSectionNum++;
        if (m_ProjectionMode == LOOKDOWN)
        {
            m_currentSectionVerticalNum++;
        }
        else
        {
            m_currentSectionHorizontalNum++;
        }
        int colorindex = (m_currentSectionNum) % 14;
        QString suffex = (m_ProjectionMode == LOOKDOWN ? "_V" : "_H");
        int indexNum = (m_ProjectionMode == LOOKDOWN ? m_currentSectionVerticalNum : m_currentSectionHorizontalNum);
        ccSectionAnalysisLine * line = new ccSectionAnalysisLine(QString("profile" + suffex + QString::number(indexNum + 1)));
        line->setProjectionMode(m_ProjectionMode);
        line->setCurrentData(m_CurrentPointData);
        line->setCurrentColor(colorData[colorindex]);
        if (m_ProjectionMode == LOOKDOWN)
        {
            line->setRangeParam(m_heightMin, m_heightMax);
        }
        else
        {
            line->setRangeParam(m_heightMinY, m_heightMaxY);
        }
        line->setVisible(true);
        m_object->addChild(line);
        listWidget->blockSignals(true);
        int rowCount = listWidget->rowCount();
        listWidget->setRowCount(rowCount + 1);
        listWidget->setRowHeight(rowCount, 36);
        //QLabel* btn = new QLabel();
        //SetButtonColor(btn, colorData[colorindex]);
        //listWidget->setCellWidget(rowCount, 0, btn);

        QTableWidgetItem* item1 = new QTableWidgetItem();
        QString pngname = "qssimage/JSectionAnalysisColors" + QString::number(colorindex + 1) + ".png";
        item1->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + pngname));
        item1->setFlags(item1->flags() & ~Qt::ItemIsEditable);
        item1->setData(Qt::EditRole, QString("profile" + suffex + QString::number(indexNum + 1)));
        listWidget->setItem(rowCount, 0, item1);

        QToolButton* delButton = new QToolButton();
        delButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/SectionAnalysisDelete.png"));
        connect(delButton, &QToolButton::clicked, this, &FJSectionAnalysisDlg::onDelButtonPushed);
        listWidget->setCellWidget(rowCount, 1, delButton);
        listWidget->setCurrentCell(rowCount, 0);
        listWidget->blockSignals(false);
        m_CurrentPointData.clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->clear();
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->clear();
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        QCoreApplication::processEvents();
        calculateSectionAnalysisData(QString(QString("profile" + suffex + QString::number(indexNum + 1))), 0.1, 0, true,"X");
        m_Line->setCurrentData(m_CurrentPointData);
        showNormal();
        raise();
        activateWindow();
        m_win->redraw();
    }
    else if (m_CurrentPointData.size() == 1)
    {
        //m_PointData.pop_back();
        m_CurrentPointData.clear();
        m_Line->setCurrentData(m_CurrentPointData);
        updateData();
        m_win->redraw();
    }
    m_Line->setMousePosVisiable(false);
    m_Line->setThickness(0.1);
    if (m_viewMode == VIEW3DMODE)
    {
        m_win->setInteractionMode(ccGLWindow::MODE_SECTIONANALYSIS);
    }
    setViewChangeButtonEnable(true);
    //setFocus();

}

void FJSectionAnalysisDlg::onDelButtonPushed()
{
    CS::Widgets::FramelessMessageBox message_box(QMessageBox::Question, tr("Prompt"), tr("Are you sure you want to delete the profile?"), QMessageBox::Cancel | QMessageBox::Ok, this);
    bool isdelete = (message_box.exec() == QMessageBox::Ok);
    QCoreApplication::processEvents();
    QObject* senderButton = sender();
    //test 'aligned' buttons first
    {
        for (int i = 0; i < listWidget->rowCount(); ++i)
        {
            if (listWidget->cellWidget(i, 1) == senderButton)
            {
                if (!isdelete)
                {
                    listWidget->setCurrentItem(listWidget->item(i, 0));
                }
                else
                {
                    QString cloudName = listWidget->item(i, 0)->text();
                    int childNum = m_object->getChildrenNumber();
                    for (int j = childNum - 1; j > 0; j--)
                    {
                        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(j));
                        if (label)
                        {
                            if (label->getName() == cloudName)
                            {
                                m_object->removeChild(j);
                            }
                        }
                    }
                    m_SectionAnalysisData.erase(cloudName);
                    listWidget->removeRow(i);
                    auto currentitem = listWidget->currentItem();
                    updateCloudSelected(currentitem, true);
                    m_win->redraw();
                }
            }
        }
    }
}


void FJSectionAnalysisDlg::updateData()
{
    //if (m_win)
    //{
    //	m_win->setSectionAnalysis(m_PointData);
    //}
}

void FJSectionAnalysisDlg::calculateSectionAnalysisData(QString name, double thickness, double offset,bool isnewfile, QString direction, bool updateoffsetdir)
{
    CCVector3d stepoffset = CCVector3d(offset, 0, 0);
    if (direction == "Y")
    {
        stepoffset = CCVector3d(0, offset, 0);
    }
    else if (direction == "Z")
    {
        stepoffset = CCVector3d(0, 0, offset);
    }
    int childNum = m_object->getChildrenNumber();
    for (int i = childNum - 1; i > 0; i--)
    {
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        if (label)
        {
            if (label->getName() == name)
            {
                m_Waitingdlg = new WaitingDialog::MetahublFramelessWaitingdialog(this);
                m_Waitingdlg->startWaiting(tr("Updating... Please wait."));
                //WaitingDialog::MetahublFramelessWaitingdialog::instance()->setParent(this);
                //WaitingDialog::MetahublFramelessWaitingdialog::instance()->startWaiting(QCoreApplication::translate("MainWindowUI", "Loading...", nullptr));	//[!].并发执行线程中执行
                QtConcurrent::run([=]() {

                    QObject end;
                    connect(&end, &QObject::destroyed, this, [=]() {
                        m_Waitingdlg->stopWaiting();
                        delete m_Waitingdlg;
                        m_Waitingdlg = nullptr;
                        //WaitingDialog::MetahublFramelessWaitingdialog::instance()->stopWaiting();
                        if (isnewfile)
                        {
                            showNormal();
                        }
                        raise();
                        activateWindow();
                        auto currentitem = listWidget->currentItem();
                        updateCloudSelected(currentitem, updateoffsetdir);
                        m_win->redraw();
                    });

                    ProjectionDirection lineMode = label->getProjectionMode();
                    label->setThickness(thickness);
                    label->setOffset(stepoffset + label->getOffset());
                    double thick = thickness / 2.0;
                    std::vector<CCVector3d> dataList;
                    label->getCurrentData(dataList);
                    int pointNum = dataList.size();
                    if (pointNum <= 1)
                    {
                        return;
                    }
                    SectionAnalysisData renderData;
                    double pointMaxX = -9999999;
                    double pointMaxY = -9999999;
                    double pointMinX = 9999999;
                    double pointMinY = 9999999;
                    if (lineMode == LOOKDOWN)
                    {
                        for (auto & curpoint : dataList)
                        {
                            curpoint = curpoint + stepoffset + label->getOffset();
                            pointMaxX = std::max(pointMaxX, curpoint.x);
                            pointMaxY = std::max(pointMaxY, curpoint.y);
                            pointMinX = std::min(pointMinX, curpoint.x);
                            pointMinY = std::min(pointMinY, curpoint.y);
                        }
                        pointMaxX += thick;
                        pointMaxY += thick;
                        pointMinX -= thick;
                        pointMinY -= thick;
                        QVector<QPointF> vpf;
                        std::vector<QVector<QPointF>> pointDataList;
                        double x1 = dataList[0].x;
                        double y1 = dataList[0].y;
                        double x2 = dataList[1].x;
                        double y2 = dataList[1].y;
                        double x3 = 0;
                        double y3 = 0;
                        double dx = x2 - x1;
                        double dy = y2 - y1;
                        QPointF pointStartUp(x1 + dy / std::sqrt(dx*dx + dy * dy)*thick, y1 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                        QPointF pointStartDown(x1 - dy / std::sqrt(dx*dx + dy * dy)*thick, y1 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                        vpf.append(pointStartUp);
                        vpf.append(pointStartDown);
                        double xMax = 0;
                        double yMax = -9999999;
                        double yMin = 9999999;
                        for (int i = 1; i <= pointNum - 1; i++)
                        {
                            x1 = dataList[i - 1].x;
                            y1 = dataList[i - 1].y;
                            x2 = dataList[i].x;
                            y2 = dataList[i].y;
                            dx = x2 - x1;
                            dy = y2 - y1;
                            QPointF point1(x2 - dy / std::sqrt(dx*dx + dy * dy)*thick, y2 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                            QPointF point2(x2 + dy / std::sqrt(dx*dx + dy * dy)*thick, y2 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                            if (isTwoLineIntersect(vpf.at(0), point1, vpf.at(1), point2))
                            {
                                vpf.append(point1);
                                vpf.append(point2);
                            }
                            else
                            {
                                vpf.append(point2);
                                vpf.append(point1);
                            }
                            pointDataList.push_back(vpf);
                            xMax += std::sqrt(dx*dx + dy * dy);
                            vpf.clear();
                            if (i != pointNum - 1)
                            {
                                x1 = dataList[i].x;
                                y1 = dataList[i].y;
                                x2 = dataList[i + 1].x;
                                y2 = dataList[i + 1].y;
                                dx = x2 - x1;
                                dy = y2 - y1;
                                pointStartUp = QPointF(x1 + dy / std::sqrt(dx*dx + dy * dy)*thick, y1 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                                pointStartDown = QPointF(x1 - dy / std::sqrt(dx*dx + dy * dy)*thick, y1 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                                vpf.append(pointStartUp);
                                vpf.append(pointStartDown);
                            }
                        }
                        for (auto curPointCloud : m_selectedObjectList)
                        {
                            if (curPointCloud)
                            {
                                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(curPointCloud);
                                if (cloud)
                                {
                                    std::vector<std::pair<QColor, QPointF>> pointdatas;
                                    QString cloudname = cloud->getName();
                                    int cloudSize = cloud->size();
                                    for (int j = 0; j < cloudSize; j++)
                                    {
                                        CCVector3 P;
                                        cloud->getPoint(j, P);
                                        if (P.x >= pointMinX && P.x <= pointMaxX && P.y >= pointMinY && P.y <= pointMaxY)
                                        {
                                            QPointF Ppoint(P.x, P.y);
                                            double startX = 0.0;
                                            for (int k = 0; k < pointDataList.size(); k++)
                                            {
                                                x2 = dataList[k].x;
                                                y2 = dataList[k].y;
                                                x3 = dataList[k + 1].x;
                                                y3 = dataList[k + 1].y;
                                                dx = x2 - x3;
                                                dy = y2 - y3;
                                                QPolygonF polygon(pointDataList[k]);
                                                if (polygon.containsPoint(Ppoint, Qt::WindingFill))
                                                {
                                                    yMax = std::max(yMax, double(P.z));
                                                    yMin = std::min(yMin, double(P.z));
                                                    x1 = P.x;
                                                    y1 = P.y;
                                                    double cosNum = ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) - (x1 - x3)*(x1 - x3) - (y1 - y3)*(y1 - y3)) / 2.0 / std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) / std::sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3));
                                                    QPointF calculatePoint(std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))* std::abs(cosNum) + startX, P.z);
                                                    QColor pointcolor(255,0,0);
                                                    if (cloud->sfShown())
                                                    {
                                                        const ccColor::Rgb* sfrgb = cloud->getCurrentDisplayedScalarField()->getValueColor(j);
                                                        pointcolor = QColor(sfrgb->r, sfrgb->g, sfrgb->b);
                                                    }
                                                    else if(cloud->hasColors())
                                                    {
                                                        auto pointrgbcolor = cloud->getPointColor(j);
                                                        pointcolor = QColor(pointrgbcolor.r, pointrgbcolor.g, pointrgbcolor.b);
                                                    }
                                                    else
                                                    {
                                                        pointcolor = QColor(cloud->getTempColor().r, cloud->getTempColor().g, cloud->getTempColor().b);
                                                    }
                                                    pointdatas.push_back(std::make_pair(pointcolor,calculatePoint));
                                                    break;
                                                }
                                                startX += std::sqrt(dx*dx + dy * dy);
                                            }
                                        }
                                    }
                                    renderData.Pointdata.insert(std::pair(cloudname, pointdatas));
                                }
                            }
                        }
                        renderData.yStart = yMin;
                        renderData.yStop = (yMax - yMin)*1.15 + yMin;
                        renderData.xStop = xMax;
                        renderData.thickness = thickness;
                    }
                    else
                    {
                        for (auto & curpoint : dataList)
                        {
                            curpoint = curpoint + stepoffset + label->getOffset();
                            pointMaxX = std::max(pointMaxX, curpoint.z);
                            pointMaxY = std::max(pointMaxY, curpoint.x);
                            pointMinX = std::min(pointMinX, curpoint.z);
                            pointMinY = std::min(pointMinY, curpoint.x);
                        }
                        pointMaxX += thick;
                        pointMaxY += thick;
                        pointMinX -= thick;
                        pointMinY -= thick;
                        QVector<QPointF> vpf;
                        std::vector<QVector<QPointF>> pointDataList;
                        double x1 = dataList[0].z;
                        double y1 = dataList[0].x;
                        double x2 = dataList[1].z;
                        double y2 = dataList[1].x;
                        double x3 = 0;
                        double y3 = 0;
                        double dx = x2 - x1;
                        double dy = y2 - y1;
                        QPointF pointStartUp(x1 + dy / std::sqrt(dx*dx + dy * dy)*thick, y1 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                        QPointF pointStartDown(x1 - dy / std::sqrt(dx*dx + dy * dy)*thick, y1 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                        vpf.append(pointStartUp);
                        vpf.append(pointStartDown);
                        double xMax = 0;
                        double yMax = -9999999;
                        double yMin = 9999999;
                        for (int i = 1; i <= pointNum - 1; i++)
                        {
                            x1 = dataList[i - 1].z;
                            y1 = dataList[i - 1].x;
                            x2 = dataList[i].z;
                            y2 = dataList[i].x;
                            dx = x2 - x1;
                            dy = y2 - y1;
                            QPointF point1(x2 - dy / std::sqrt(dx*dx + dy * dy)*thick, y2 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                            QPointF point2(x2 + dy / std::sqrt(dx*dx + dy * dy)*thick, y2 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                            if (isTwoLineIntersect(vpf.at(0), point1, vpf.at(1), point2))
                            {
                                vpf.append(point1);
                                vpf.append(point2);
                            }
                            else
                            {
                                vpf.append(point2);
                                vpf.append(point1);
                            }
                            pointDataList.push_back(vpf);
                            xMax += std::sqrt(dx*dx + dy * dy);
                            vpf.clear();
                            if (i != pointNum - 1)
                            {
                                x1 = dataList[i].z;
                                y1 = dataList[i].x;
                                x2 = dataList[i + 1].z;
                                y2 = dataList[i + 1].x;
                                dx = x2 - x1;
                                dy = y2 - y1;
                                pointStartUp = QPointF(x1 + dy / std::sqrt(dx*dx + dy * dy)*thick, y1 - dx / std::sqrt(dx*dx + dy * dy)*thick);
                                pointStartDown = QPointF(x1 - dy / std::sqrt(dx*dx + dy * dy)*thick, y1 + dx / std::sqrt(dx*dx + dy * dy)*thick);
                                vpf.append(pointStartUp);
                                vpf.append(pointStartDown);
                            }
                        }
                        for (auto curPointCloud : m_selectedObjectList)
                        {
                            if (curPointCloud)
                            {
                                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(curPointCloud);
                                if (cloud)
                                {
                                    std::vector<std::pair<QColor, QPointF>> pointdatas;
                                    QString cloudname = cloud->getName();
                                    int cloudSize = cloud->size();
                                    for (int j = 0; j < cloudSize; j++)
                                    {
                                        CCVector3 P;
                                        cloud->getPoint(j, P);
                                        if (P.z >= pointMinX && P.z <= pointMaxX && P.x >= pointMinY && P.x <= pointMaxY)
                                        {
                                            QPointF Ppoint(P.z, P.x);
                                            double startX = 0.0;
                                            for (int k = 0; k < pointDataList.size(); k++)
                                            {
                                                x2 = dataList[k].z;
                                                y2 = dataList[k].x;
                                                x3 = dataList[k + 1].z;
                                                y3 = dataList[k + 1].x;
                                                dx = x2 - x3;
                                                dy = y2 - y3;
                                                QPolygonF polygon(pointDataList[k]);
                                                if (polygon.containsPoint(Ppoint, Qt::WindingFill))
                                                {
                                                    yMax = std::max(yMax, double(P.y));
                                                    yMin = std::min(yMin, double(P.y));
                                                    x1 = P.z;
                                                    y1 = P.x;
                                                    double cosNum = ((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3) - (x1 - x3)*(x1 - x3) - (y1 - y3)*(y1 - y3)) / 2.0 / std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) / std::sqrt((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3));
                                                    double thn = acos(cosNum);
                                                    QPointF calculatePoint(std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))*cos(thn) + startX, P.y);
                                                    QColor pointcolor(255, 0, 0);
                                                    if (cloud->sfShown())
                                                    {
                                                        const ccColor::Rgb* sfrgb = cloud->getCurrentDisplayedScalarField()->getValueColor(j);
                                                        pointcolor = QColor(sfrgb->r, sfrgb->g, sfrgb->b);
                                                    }
                                                    else if (cloud->hasColors())
                                                    {
                                                        auto pointrgbcolor = cloud->getPointColor(j);
                                                        pointcolor = QColor(pointrgbcolor.r, pointrgbcolor.g, pointrgbcolor.b);
                                                    }
                                                    else
                                                    {
                                                        pointcolor = QColor(cloud->getTempColor().r, cloud->getTempColor().g, cloud->getTempColor().b);
                                                    }
                                                    pointdatas.push_back(std::make_pair(pointcolor, calculatePoint));
                                                    break;
                                                }
                                                startX += std::sqrt(dx*dx + dy * dy);
                                            }
                                        }
                                    }
                                    renderData.Pointdata.insert(std::pair(cloudname, pointdatas));

                                }
                            }
                        }
                        renderData.yStart = yMin;
                        renderData.yStop = (yMax - yMin)*1.15 + yMin;
                        renderData.xStop = xMax;
                        renderData.thickness = thickness;
                    }
                    std::vector<bool> visiablelist;
                    for (auto curcloud : renderData.Pointdata)
                    {
                        visiablelist.push_back(true);
                    }
                    renderData.PointCloudVisiable = visiablelist;
                    m_SectionAnalysisData[name] = renderData;

                });

            }

        }
    }
}

void FJSectionAnalysisDlg::slotCurrentItemSelectChanged()
{
    auto currentitem = listWidget->currentItem();
    updateCloudSelected(currentitem, true);
}

void FJSectionAnalysisDlg::updateCloudSelected(QTableWidgetItem *item, bool updateoffsetdir)
{
    int index = listWidget->currentRow();
    for (auto box : m_checkBoxs)
    {
        m_layout->removeWidget(box);
        delete box;
        box = nullptr;
    }
    m_checkBoxs.clear();
    if (index < 0)
    {
        toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measurecloseicondisable.png"));
        toolButton->setDisabled(true);
        doubleSpinBox_offset->setDisabled(true);
        comboBox->clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        QCoreApplication::processEvents();
        //doubleSpinBox_offset->setValue(0);
        doubleSpinBox_thickness->setDisabled(true);
        doubleSpinBox_thickness->setValue(0.1);
        pushButton->setDisabled(true);
        SectionAnalysisData nullrenderData;
        widget->setData(nullrenderData);
        widget->setRenderMode(NONEMODE);
        widget_list->setVisible(false);
        label_txt->setText("");
        return;
    }
    pushButton->setDisabled(false);
    toolButton->setDisabled(false);
    widget_list->setVisible(true);
    QString cloudName = listWidget->item(index, 0)->text();
    widget->setData(m_SectionAnalysisData[cloudName]);
    int startindex = 0;
    for (auto curSectionSet : m_SectionAnalysisData[cloudName].Pointdata)
    {
        QCheckBox * newCheckBox = new QCheckBox(widget_list);
        QString checkboxStyle = "QCheckBox{color: #ffffff;background - color:transparent;}\n"
            "QCheckBox::indicator:checked{image:url(:/widgetControl/images/theme/selected.png);}\n"
            "QCheckBox::indicator:unchecked{image:url(:/widgetControl/images/theme/unselected.png);}";
        newCheckBox->setStyleSheet(checkboxStyle);
        newCheckBox->setText(curSectionSet.first + "                                                                                                            ");
        newCheckBox->setToolTip(curSectionSet.first);
        newCheckBox->setFixedHeight(20);
        newCheckBox->setMaximumWidth(widget->width() / 3.0 - 12);
        newCheckBox->setChecked(m_SectionAnalysisData[cloudName].PointCloudVisiable[startindex]);
        connect(newCheckBox, QOverload<int>::of(&QCheckBox::stateChanged), this, &FJSectionAnalysisDlg::checkboxStateChanged);
        m_checkBoxs.push_back(newCheckBox);
        m_layout->addWidget(newCheckBox,Qt::AlignLeft);
        startindex++;
    }
    widget_list->setFixedHeight(m_SectionAnalysisData[cloudName].Pointdata.size() *20 +2);
    int childNum = m_object->getChildrenNumber();
    for (int i = childNum - 1; i > 0; i--)
    {
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        if (label)
        {
            if (label->getName() == cloudName)
            {
                label->setIsSelected(true);
                doubleSpinBox_offset->setDisabled(false);
                doubleSpinBox_thickness->setDisabled(false);
                //doubleSpinBox_offset->setValue(label->getOffset());
                doubleSpinBox_thickness->setValue(label->getThickness());
                if (label->getProjectionMode() == LOOKDOWN)
                {
                    doubleSpinBox_offset->setRange(-m_xMax, m_xMax);
                    if (updateoffsetdir)
                    {
                        comboBox->clear();
                        comboBox->addItem("X");
                        comboBox->addItem("Y");
                        comboBox->setCurrentIndex(0);
                    }
                }
                else
                {
                    doubleSpinBox_offset->setRange(-m_zMax, m_zMax);
                    if (updateoffsetdir)
                    {
                        comboBox->clear();
                        comboBox->addItem("Z");
                        comboBox->addItem("X");
                        comboBox->setCurrentIndex(0);
                    }
                }
                QCoreApplication::processEvents();
                if (label->getIsPickPoint())
                {
                    QPointF start;
                    QPointF end;
                    label->getPickPoint(start, end);
                    widget->setPickPoint(start, end);
                }
                else
                {
                    widget->setRenderMode(NONEMODE);
                }
                if (!label->getIsMeasureOpen())
                {
                    toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measurecloseiconclose.png"));
                    widget->startMeasure(false);
                }
                else
                {
                    toolButton->setIcon(QIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/measureopenicon.png"));
                    widget->startMeasure(true);
                }
            }
            else
            {
                label->setIsSelected(false);
            }

        }
    }
    if (m_win)
    {
        m_win->redraw(false);
    }
}

void FJSectionAnalysisDlg::checkboxStateChanged(int)
{
    std::vector<bool> visiablelist;
    for (auto box : m_checkBoxs)
    {
        bool ischeck = box->isChecked();
        visiablelist.push_back(ischeck);
    }
    widget->setPointCloudVisiable(visiablelist);
    int currow = listWidget->currentRow();
    QString cloudName = listWidget->item(currow, 0)->text();
    m_SectionAnalysisData[cloudName].PointCloudVisiable = visiablelist;
}

void FJSectionAnalysisDlg::calculateWidthAndHeightRange()
{
    m_pointSum = 0;
    m_heightMin = 99999999;
    m_heightMax = -99999999;
    m_xMax = 0;
    m_heightMinY = 99999999;
    m_heightMaxY = -99999999;
    m_zMax = 0;
    int pointSize = m_selectedObjectList.size();
    if (pointSize > 0)
    {
        for (int i = 0; i < pointSize; i++)
        {
            ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedObjectList[i]);
            if (cloud)
            {
                m_pointSum += cloud->size();
                ccBBox box = cloud->getOwnBB();
                CCVector3& dimSumMin = box.minCorner();
                CCVector3& dimSumMax = box.maxCorner();
                m_xMax += std::abs(dimSumMax.x - dimSumMin.x);
                m_zMax += std::abs(dimSumMax.z - dimSumMin.z);
                m_heightMin = std::min(m_heightMin, double(dimSumMin.z));
                m_heightMax = std::max(m_heightMax, double(dimSumMax.z));
                m_heightMinY = std::min(m_heightMinY, double(dimSumMin.y));
                m_heightMaxY = std::max(m_heightMaxY, double(dimSumMax.y));
            }
        }
    }
}

bool FJSectionAnalysisDlg::eventFilter(QObject *obj, QEvent *e)
{
    if (e->type() == QEvent::KeyPress)
    {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(e);
        if (keyEvent->key() == Qt::Key_Escape)
        {
            if (m_CurrentPointData.size() > 0)
            {
                m_CurrentPointData.clear();
                m_CurrentPointData.push_back(CCVector3d(0, 0, 0));
                stopPickPoint();
            }
            else
            {
                if (widget->getRenderMode() == RUNNINGMODE)
                {
                    widget->setRenderMode(NONEMODE);
                }
            }
            return true;
        }
        else
        {
            return QDialog::eventFilter(obj, e);
        }

    }
    else
    {
        // standard event processing
        return QDialog::eventFilter(obj, e);
    }
}

void FJSectionAnalysisDlg::setView3d()
{
    ProjectionDirection oldmode = m_ProjectionMode;
    m_ProjectionMode = LOOKDOWN;
    if (m_Line)
    {
        m_Line->setProjectionMode(LOOKDOWN);
    }
    m_viewMode = VIEW3DMODE;
    setViewChangeButtonEnable(true);
    if (listWidget->rowCount()==0 && oldmode != m_ProjectionMode)
    {
        comboBox->clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        QCoreApplication::processEvents();
    }
    MainWindow::TheInstance()->setViewButtonDisabled(false, true);
    m_win->setView(CC_ISO_VIEW_1);
    m_win->setInteractionMode(ccGLWindow::MODE_SECTIONANALYSIS);
}

void FJSectionAnalysisDlg::setViewUp()
{
    ProjectionDirection oldmode = m_ProjectionMode;
    m_ProjectionMode = LOOKDOWN;
    if (m_Line)
    {
        m_Line->setProjectionMode(LOOKDOWN);
    }
    m_viewMode = TOPVIEWMODE;
    setViewChangeButtonEnable(true);
    if (listWidget->rowCount() == 0 && oldmode != m_ProjectionMode)
    {
        comboBox->clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        QCoreApplication::processEvents();
    }
    MainWindow::TheInstance()->setViewButtonDisabled(true, true);
    QCoreApplication::processEvents();
    m_win->setView(CC_TOP_VIEW);
    m_win->setInteractionMode(ccGLWindow::MODE_SEGMENT);
}

void FJSectionAnalysisDlg::setViewFace()
{
    ProjectionDirection oldmode = m_ProjectionMode;
    m_ProjectionMode = LOOKFACE;
    if (m_Line)
    {
        m_Line->setProjectionMode(LOOKFACE);
    }
    m_viewMode = FRONTVIEWMODE;
    setViewChangeButtonEnable(true);
    if (listWidget->rowCount() == 0 && oldmode != m_ProjectionMode)
    {
        comboBox->clear();
        if (m_ProjectionMode == LOOKDOWN)
        {
            comboBox->addItem("X");
            comboBox->addItem("Y");
            comboBox->setCurrentIndex(0);
        }
        else
        {
            comboBox->addItem("Z");
            comboBox->addItem("X");
            comboBox->setCurrentIndex(0);
        }
        QCoreApplication::processEvents();
    }
    MainWindow::TheInstance()->setViewButtonDisabled(true,true);
    m_win->setView(CC_FRONT_VIEW);
    m_win->setInteractionMode(ccGLWindow::MODE_SEGMENT);
}

void FJSectionAnalysisDlg::onThicknessOrOffsetChanged(double value)
{
    QString cloudName = getCurrentSectionName();
    int childNum = m_object->getChildrenNumber();
    for (int i = childNum - 1; i > 0; i--)
    {
        ccSectionAnalysisLine* label = dynamic_cast<ccSectionAnalysisLine*>(m_object->getChild(i));
        if (label)
        {
            if (label->getName() == cloudName)
            {
                //label->setOffset(doubleSpinBox_offset->value());
                label->setThickness(doubleSpinBox_thickness->value());
            }

        }
    }
    if (m_win)
    {
        m_win->redraw(false);
    }
}


void FJSectionAnalysisDlg::setViewChangeButtonEnable(bool isenable)
{
    if (m_viewMode == VIEW3DMODE)
    {
        setActionIcon(toolButton_view3d, "SectionAnalysis3DSelect", "SectionAnalysis3DSelect", "SectionAnalysis3DSelect");
        setActionIcon(toolButton_viewtop, "SectionAnalysisTopNormal", "SectionAnalysisTopNormal", "SectionAnalysisTopDisable");
        setActionIcon(toolButton_viewface, "SectionAnalysisFrontNormal", "SectionAnalysisFrontNormal", "SectionAnalysisFrontDisable");
    }
    else if (m_viewMode == TOPVIEWMODE)
    {
        setActionIcon(toolButton_view3d, "SectionAnalysis3dNormal", "SectionAnalysis3dNormal", "SectionAnalysis3DDisable");
        setActionIcon(toolButton_viewtop, "SectionAnalysisTopSelect", "SectionAnalysisTopSelect", "SectionAnalysisTopSelect");
        setActionIcon(toolButton_viewface, "SectionAnalysisFrontNormal", "SectionAnalysisFrontNormal", "SectionAnalysisFrontDisable");
    }
    else
    {
        setActionIcon(toolButton_view3d, "SectionAnalysis3dNormal", "SectionAnalysis3dNormal", "SectionAnalysis3DDisable");
        setActionIcon(toolButton_viewtop, "SectionAnalysisTopNormal", "SectionAnalysisTopNormal", "SectionAnalysisTopDisable");
        setActionIcon(toolButton_viewface, "SectionAnalysisFrontSelect", "SectionAnalysisFrontSelect", "SectionAnalysisFrontSelect");
    }
    if (isenable)
    {
        toolButton_view3d->setEnabled(true);
        toolButton_viewtop->setEnabled(true);
        toolButton_viewface->setEnabled(true);
    }
    else
    {
        toolButton_view3d->setEnabled(false);
        toolButton_viewtop->setEnabled(false);
        toolButton_viewface->setEnabled(false);
    }
}

void FJSectionAnalysisDlg::onSectionThicknessUpdate(QString name, double thick)
{
    QString cloudName = getCurrentSectionName();
    if (cloudName == name)
    {
        doubleSpinBox_thickness->blockSignals(true);
        doubleSpinBox_thickness->setValue(thick);
        doubleSpinBox_thickness->blockSignals(false);
    }
}

void FJSectionAnalysisDlg::updatePointColor()
{
    m_showPointColor = checkBox->isChecked();
    if (!m_showPointColor)
    {
        checkBox->setToolTip(tr("Click to turn on."));
    }
    else
    {
        checkBox->setToolTip(tr("Click to turn off."));
    }
    widget->setShowPointColor(m_showPointColor);
}

void FJSectionAnalysisDlg::updatePointSize()
{
    widget->updatePointSize();
}