#include "fjpointrenderdlg.h"
#include "ui_fjpointrenderdlg.h"
#include <QToolButton>
#include "ccEntityAction.h"
#include "model\projectmodel.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"
#include "fjpointcloudutil.h"
//aric.tang_2022.9.29_____color scale
#include <ccColorScalesManager.h>
#include "FJStyleManager.h"
#include <QDebug>
#include <QColorDialog>
#include "mainwindow.h"
#include "ccGLWindow.h"



FJPointRenderDlg::FJPointRenderDlg(QWidget *parent)
    : FJBaseWidget(parent)
    , ui(new Ui::FJPointRenderDlg)
{
    setContextMenuPolicy(Qt::NoContextMenu);
    ui->setupUi(this);
    setToolButtonStyle(ui->toolButton_altitude, false);
    setToolButtonStyle(ui->toolButton_intensity, false);
    setToolButtonStyle(ui->toolButton_gpstime, false);
    setToolButtonStyle(ui->toolButton_classification, false);
    setToolButtonStyle(ui->toolButton_userdata, false);
    setToolButtonStyle(ui->toolButton_uniquecolor, false);
    setToolButtonStyle(ui->toolButton_rgb, false);
    setToolButtonStyle(ui->toolButton_iwitha, false);
    setToolButtonStyle(ui->toolButton_Shadow, false);
    setActionIcon(ui->toolButton_altitude, "altitude", "altitudeselect", "altitudedisabled");
    setActionIcon(ui->toolButton_intensity, "intensityIcon", "intensityselectIcon", "intensityIcondisabled");
    setActionIcon(ui->toolButton_gpstime, "gpstimeIcon", "gpstimeselectIcon", "gpstimeIcondisabled");
    setActionIcon(ui->toolButton_classification, "classficationIcon", "classficationselectIcon", "classficationIcondisabled");
    setActionIcon(ui->toolButton_userdata, "userdataIcon", "userdataselectIcon", "userdataIcondisabled");
    setActionIcon(ui->toolButton_uniquecolor, "uniqueIcon", "uniqueselectIcon", "uniqueIcondisabled");
    setActionIcon(ui->toolButton_rgb, "rgbcoloricon", "rgbcoloriconselect", "rgbcoloricondisabled");
    setActionIcon(ui->toolButton_iwitha, "altitudewithintensity", "altitudewithintensityselect", "altitudewithintensitydisabled");
    setActionIcon(ui->toolButton_Shadow, "Shadownormal", "Shadowselect", "Shadowdisable");
    connect(ui->toolButton_uniquecolor, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {

            CS::Widgets::FramelessDialog outdlg(parent);
            outdlg.setWindowTitle(QCoreApplication::translate("ccEntityAction", "Background", nullptr));
            QColorDialog colordlg(Qt::white, parent);
            QObject::connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
            colordlg.setOptions(QColorDialog::NoButtons);
            outdlg.SetContentHolder(&colordlg);
            if (!outdlg.exec())
            {
                return;
            }
            QColor colour = colordlg.currentColor();

            if (!colour.isValid())
                return;

            for (ccHObject* entity : m_selectedEntities)
            {
                entity->setTempColor(ccColor::Rgb(colour.red(), colour.green(), colour.blue()), false);
                entity->showSF(false);
                entity->showColors(false);
                entity->setCurrentCombinationMode(UNIQUECOLOR);
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
                if (cloud)
                {
                    cloud->scaleFieldColorChange();
                }
            }
        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_altitude, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                FJPointCloudUtil::exportAltitudeColorScale(entity);
                entity->setCurrentCombinationMode(ALTITUDE);
                showColorByScalar("coord. z");
                FJPointCloudUtil::updateSaturationByRange(entity, 0.01, 0.99);
            }


        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_intensity, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "intensity"))
                {
                    entity->setCurrentCombinationMode(INTENSITY);
                    showColorByScalar("intensity");
                    FJPointCloudUtil::updateSaturationByRange(entity, 0, 0.95);
                }
            }

        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_gpstime, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "gpstime"))
                {
                    entity->setCurrentCombinationMode(GPSTIME);
                    showColorByScalar("gpstime");
                }
            }
        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_Shadow, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject * entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "Illuminance (PCV)"))
                {
                    entity->setCurrentCombinationMode(PCVMODE);
                    showColorByScalar("Illuminance (PCV)");
                }
            }
        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
        });
    connect(ui->toolButton_classification, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "classification"))
                {
                    entity->setCurrentCombinationMode(CLASSIFICATION);
                    showColorByScalar("classification");
                    FJPointCloudUtil::updateClassficationDataFromDoc(entity);
                }
            }

        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_userdata, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "userdata"))
                {
                    entity->setCurrentCombinationMode(USERDATA);
                    showColorByScalar("userdata");
                }
            }

        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_rgb, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (entity->hasColors())
                {
                    entity->setCurrentCombinationMode(RGBCOLOR);
                    entity->showColors(true);
                    entity->showSF(false);
                    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
                    if (cloud)
                    {
                        cloud->scaleFieldColorChange();
                    }
                }
            }

        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
    connect(ui->toolButton_iwitha, &QToolButton::clicked, [=]() {
        if (m_selectedEntities.size() > 0)
        {
            foreach(ccHObject *entity, m_selectedEntities)
            {
                if (FJPointCloudUtil::isCloudHasSF(entity, "intensity"))
                {
                    FJPointCloudUtil::exportAltitudeColorScale(entity);
                    FJPointCloudUtil::pointCloudCompositeRendering(entity, "coord. z", "intensity");
                    entity->setCurrentCombinationMode(ALTITUDEANDINTENSITY);
                }
            }

        }
        emit refreshAll(false);
        ccGLWindow* pWin = MainWindow::TheInstance()->GetActiveGLWindow();
        if (pWin)
        {
            pWin->redraw(false);
        }
        setCurrentSelectObject(m_selectedEntities);
    });
}

FJPointRenderDlg::~FJPointRenderDlg()
{
    delete ui;
}


//aric.tang_2022.9.29
/** brief set Color Scale
  * param[out] scale the color scale for two scalar fields combin
  * param[in] isuniquecolor color scale choose
  * param[in] map classification color map
*/
void FJPointRenderDlg::setColorScale(bool isuniquecolor)
{
    if (isuniquecolor)
    {
        //aric.tang_2022.9.29_____color scale
        ccColorScalesManager::GetUniqueInstance()->addScale(FJPointCloudUtil::createCombinScale(15, m_selectedEntities[0]->getUniqueColor()));
        ccPointCloud *cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
        ccScalarField *sf = cloud->getCurrentDisplayedScalarField();
        sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::PARAMS_COMBIN));
        cloud->showSF(true);
    }
    else
    {
        //aric.tang_2022.9.29_____color scale
        std::vector<QColor> test;
        test.push_back(Qt::blue);
        test.push_back(Qt::cyan);
        test.push_back(Qt::green);
        test.push_back(Qt::yellow);
        test.push_back(Qt::red);
        ccColorScalesManager::GetUniqueInstance()->addScale(FJPointCloudUtil::createHighScale(17, test));
        ccPointCloud *cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
        ccScalarField *sf = cloud->getCurrentDisplayedScalarField();
        sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::HIGH_COMBIN));
        //sf->setColorRampSteps(10);
        //2023.1.6_aric.tang
        if (sf)
        {
            //std::map<float, int> paramMap;
            //for (size_t i = 0; i < sf->size(); i++) {
            //    paramMap[sf->getValue(i)]++;
            //}
            //int num = sf->size() * 0.05;
            //int sumnum = 0;
            //for (auto iter = paramMap.rbegin(); iter != paramMap.rend(); iter++)
            //{
            //    sumnum += iter->second;
            //    if (sumnum >= num)
            //    {
            //        sf->setSaturationStop(iter->first);
            //        break;
            //    }
            //}
        }
        cloud->showSF(true);
    }
}


void FJPointRenderDlg::InitFJStyle()
{

}

void FJPointRenderDlg::setAllButtonDisabled()
{
    ui->toolButton_altitude->setEnabled(false);
    ui->toolButton_intensity->setEnabled(false);
    ui->toolButton_gpstime->setEnabled(false);
    ui->toolButton_classification->setEnabled(false);
    ui->toolButton_userdata->setEnabled(false);
    ui->toolButton_uniquecolor->setEnabled(false);
    ui->toolButton_rgb->setEnabled(false);
    ui->toolButton_iwitha->setEnabled(false);
    ui->toolButton_Shadow->setEnabled(false);
}

void FJPointRenderDlg::setCurrentSelectObject(ccHObject::Container selectedEntities, bool buttonClick/* = true*/)
{
    setToolButtonStyle(ui->toolButton_altitude, false);
    setToolButtonStyle(ui->toolButton_intensity, false);
    setToolButtonStyle(ui->toolButton_gpstime, false);
    setToolButtonStyle(ui->toolButton_classification, false);
    setToolButtonStyle(ui->toolButton_userdata, false);
    setToolButtonStyle(ui->toolButton_uniquecolor, false);
    setToolButtonStyle(ui->toolButton_rgb, false);
    setToolButtonStyle(ui->toolButton_iwitha, false);
    setToolButtonStyle(ui->toolButton_Shadow, false);
    setActionIcon(ui->toolButton_altitude, "altitude", "altitudeselect", "altitudedisabled");
    setActionIcon(ui->toolButton_intensity, "intensityIcon", "intensityselectIcon", "intensityIcondisabled");
    setActionIcon(ui->toolButton_gpstime, "gpstimeIcon", "gpstimeselectIcon", "gpstimeIcondisabled");
    setActionIcon(ui->toolButton_classification, "classficationIcon", "classficationselectIcon", "classficationIcondisabled");
    setActionIcon(ui->toolButton_userdata, "userdataIcon", "userdataselectIcon", "userdataIcondisabled");
    setActionIcon(ui->toolButton_uniquecolor, "uniqueIcon", "uniqueselectIcon", "uniqueIcondisabled");
    setActionIcon(ui->toolButton_rgb, "rgbcoloricon", "rgbcoloriconselect", "rgbcoloricondisabled");
    setActionIcon(ui->toolButton_iwitha, "altitudewithintensity", "altitudewithintensityselect", "altitudewithintensitydisabled");
    setActionIcon(ui->toolButton_Shadow, "Shadownormal", "Shadowselect", "Shadowdisable");
    m_selectedEntities.clear();
    m_selectedEntities = selectedEntities;

    ui->toolButton_uniquecolor->setEnabled(false);
    ui->toolButton_altitude->setEnabled(false);
    ui->toolButton_intensity->setEnabled(false);
    ui->toolButton_gpstime->setEnabled(false);
    ui->toolButton_classification->setEnabled(false);
    ui->toolButton_userdata->setEnabled(false);
    ui->toolButton_rgb->setEnabled(false);
    ui->toolButton_iwitha->setEnabled(false);
    ui->toolButton_Shadow->setEnabled(false);
    //if (m_selectedEntities.size() > 1 && !buttonClick)
    //{
    //    foreach(ccHObject *entity, m_selectedEntities)
    //    {
    //        entity->setCurrentCombinationMode(OTHER);
    //    }
    //}
    foreach(ccHObject *entity, m_selectedEntities)
    {
        FJPointCloudUtil::getUpdateCurrentCombinationMode(entity);
        if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
        {
            std::vector<CurrentCombinationMode> supportModeVec = getSupportModes(entity);
            CurrentCombinationMode mode = entity->getCurrentCombinationMode();
            applySelectedMode(mode, supportModeVec);
        }
    }
    emit updateRibbonStripAction();

    /*if (m_selectedEntities.size() > 1)
    {
        ui->toolButton_altitude->setEnabled(false);
        ui->toolButton_intensity->setEnabled(false);
        ui->toolButton_gpstime->setEnabled(false);
        ui->toolButton_classification->setEnabled(false);
        ui->toolButton_userdata->setEnabled(false);
        ui->toolButton_uniquecolor->setEnabled(true);
    }
    else if (m_selectedEntities.size() == 1)
    {
        ui->toolButton_uniquecolor->setEnabled(false);

        ui->toolButton_altitude->setEnabled(false);
        ui->toolButton_intensity->setEnabled(false);
        ui->toolButton_gpstime->setEnabled(false);
        ui->toolButton_classification->setEnabled(false);
        ui->toolButton_userdata->setEnabled(false);
        ccHObject *entity = m_selectedEntities[0];
        FJPointCloudUtil::getUpdateCurrentCombinationMode(entity);
        if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
        {
            std::vector<CurrentCombinationMode> supportModeVec = getSupportModes();
            CurrentCombinationMode mode = entity->getCurrentCombinationMode();
            applySelectedMode(mode, supportModeVec);
        }
    }
    else
    {
        ui->toolButton_altitude->setEnabled(false);
        ui->toolButton_intensity->setEnabled(false);
        ui->toolButton_gpstime->setEnabled(false);
        ui->toolButton_classification->setEnabled(false);
        ui->toolButton_userdata->setEnabled(false);
        ui->toolButton_uniquecolor->setEnabled(false);
    }*/
}

std::vector<CurrentCombinationMode> FJPointRenderDlg::getSupportModes(ccHObject *entity)
{
    std::vector<CurrentCombinationMode> supportModeVec;
    //if (m_selectedEntities.size() != 1)
    //{
    //	return supportModeVec;
    //}
    //ccHObject *entity = m_selectedEntities[0];
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
    if (cloud)
    {
        int nsf = cloud->getNumberOfScalarFields();
        for (int i = 0; i < nsf; ++i)
        {
            QString sfName = cloud->getScalarFieldName(i);
            //if (sfName.compare("coord. z", Qt::CaseInsensitive) == 0)
            //{
            //	supportModeVec.push_back(ALTITUDE);
            //}
            supportModeVec.push_back(ALTITUDE);
            if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
            {
                supportModeVec.push_back(INTENSITY);
                supportModeVec.push_back(ALTITUDEANDINTENSITY);
            }
            if (sfName.compare("gpstime", Qt::CaseInsensitive) == 0)
            {
                supportModeVec.push_back(GPSTIME);
            }
            if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
            {
                supportModeVec.push_back(CLASSIFICATION);
            }
            if (sfName.compare("userdata", Qt::CaseInsensitive) == 0)
            {
                supportModeVec.push_back(USERDATA);
            }
            if (sfName.compare("Illuminance (PCV)", Qt::CaseInsensitive) == 0)
            {
                supportModeVec.push_back(PCVMODE);
            }
        }
        if (cloud->hasColors())
        {
            supportModeVec.push_back(RGBCOLOR);
        }
    }
    return supportModeVec;
}

void FJPointRenderDlg::applySelectedMode(const CurrentCombinationMode & currentMode, const std::vector<CurrentCombinationMode> & supportModeVec)
{
    ui->toolButton_uniquecolor->setEnabled(true);
    switch (currentMode)
    {
    case ALTITUDE:
        setToolButtonStyle(ui->toolButton_altitude, true);
        setActionIcon(ui->toolButton_altitude, "altitudeselect", "altitudeselect", "altitudedisabled");
        ui->toolButton_altitude->setEnabled(true);
        break;
    case GPSTIME:
        setToolButtonStyle(ui->toolButton_gpstime, true);
        setActionIcon(ui->toolButton_gpstime, "gpstimeselectIcon", "gpstimeselectIcon", "gpstimeIcondisabled");
        ui->toolButton_gpstime->setEnabled(true);
        break;
    case USERDATA:
        setToolButtonStyle(ui->toolButton_userdata, true);
        setActionIcon(ui->toolButton_userdata, "userdataselectIcon", "userdataselectIcon", "userdataIcondisabled");
        ui->toolButton_userdata->setEnabled(true);
        break;
    case INTENSITY:
        setToolButtonStyle(ui->toolButton_intensity, true);
        setActionIcon(ui->toolButton_intensity, "intensityselectIcon", "intensityselectIcon", "intensityIcondisabled");
        ui->toolButton_intensity->setEnabled(true);
        break;
    case CLASSIFICATION:
        setToolButtonStyle(ui->toolButton_classification, true);
        setActionIcon(ui->toolButton_classification, "classficationselectIcon", "classficationselectIcon", "classficationIcondisabled");
        ui->toolButton_classification->setEnabled(true);
        break;
    case UNIQUECOLOR:
        setToolButtonStyle(ui->toolButton_uniquecolor, true);
        setActionIcon(ui->toolButton_uniquecolor, "uniqueselectIcon", "uniqueselectIcon", "uniqueIcondisabled");
        ui->toolButton_uniquecolor->setEnabled(true);
        break;
    case OTHER:
        break;
    case PCVMODE:
        setToolButtonStyle(ui->toolButton_Shadow, true);
        setActionIcon(ui->toolButton_Shadow, "Shadowselect", "Shadowselect", "Shadowdisable");
        ui->toolButton_Shadow->setEnabled(true);
        break;
    case RGBCOLOR:
        setToolButtonStyle(ui->toolButton_rgb, true);
        setActionIcon(ui->toolButton_rgb, "rgbcoloriconselect", "rgbcoloriconselect", "rgbcoloricondisabled");
        ui->toolButton_rgb->setEnabled(true);
        break;
    case ALTITUDEANDINTENSITY:
        setToolButtonStyle(ui->toolButton_iwitha, true);
        setActionIcon(ui->toolButton_iwitha, "altitudewithintensityselect", "altitudewithintensityselect", "altitudewithintensitydisabled");
        ui->toolButton_iwitha->setEnabled(true);
        break;
    }
    for (auto curmode : supportModeVec)
    {
        switch (curmode)
        {
        case ALTITUDE:
            ui->toolButton_altitude->setEnabled(true);
            break;
        case GPSTIME:
            ui->toolButton_gpstime->setEnabled(true);
            break;
        case USERDATA:
            ui->toolButton_userdata->setEnabled(true);
            break;
        case INTENSITY:
            ui->toolButton_intensity->setEnabled(true);
            break;
        case CLASSIFICATION:
            ui->toolButton_classification->setEnabled(true);
            break;
        case UNIQUECOLOR:
            ui->toolButton_uniquecolor->setEnabled(true);
            break;
        case OTHER:
            break;
        case PCVMODE:
            ui->toolButton_Shadow->setEnabled(true);
            break;
        case RGBCOLOR:
            ui->toolButton_rgb->setEnabled(true);
            break;
        case ALTITUDEANDINTENSITY:
            ui->toolButton_iwitha->setEnabled(true);
            break;
        }
    }
}

void FJPointRenderDlg::showColorByScalar(QString scalarName)
{
    for (ccHObject *entity : m_selectedEntities) {

        if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
        {
            ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
            if (cloud)
            {
                int nsf = cloud->getNumberOfScalarFields();
                for (int i = 0; i < nsf; ++i)
                {
                    QString sfName = cloud->getScalarFieldName(i);
                    if (sfName.compare(scalarName, Qt::CaseInsensitive) == 0)
                    {
                        cloud->setCurrentDisplayedScalarField(i);
                        cloud->showSF(true);
                    }
                }
            }
        }

    }
}

void FJPointRenderDlg::setToolButtonStyle(QToolButton * btn, bool isSelected)
{
    if (isSelected)
    {
        btn->setStyleSheet("QToolButton{background-color:transparent;color:#FFC805;border:0px solid #101010;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:hover{background-color:#555555;color:#FFC805;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:disabled{background-color:transparent;color:#838383;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:pressed{background-color:#555555;color:#FFC805;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}");
    }
    else
    {
        btn->setStyleSheet("QToolButton{background-color:transparent;color:#ffffff;border:0px solid #101010;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:hover{background-color:#555555;color:#FFC805;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:disabled{background-color:transparent;color:#838383;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}\n"
            "QToolButton:pressed{background-color:#555555;color:#FFC805;margin-top:4px;margin-bottom:4px;margin-left:4px;margin-right:-8px;}");
    }
}

void FJPointRenderDlg::setActionIcon(QToolButton * action, const QString& normalPix, const QString& clickedPix, const QString& disabledPix)
{
    QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + normalPix + ".png");
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + clickedPix + ".png"), QIcon::Active, QIcon::Off);
    pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + disabledPix + ".png"), QIcon::Disabled, QIcon::Off);
    action->setIcon(pIcon);
}