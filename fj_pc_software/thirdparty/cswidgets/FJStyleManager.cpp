#include "FJStyleManager.h"
#include <QApplication>
FJStyleManager* FJStyleManager::instance = new FJStyleManager();
QString FJStyleManager::Getcurrentthemepath()
{
    QString path = QApplication::applicationDirPath();
    switch (m_currenttype)
    {
    case Black:
        path += QString("/theme/Black/");
        break;
    case White:
        path += QString("/theme/White/");
        break;
    case Blue:
        path += QString("/theme/Blue/");
        break;
    default:
        path += QString("/theme/Black/");
        break;
    }
    return path;
}

void FJStyleManager::ShiftStyle(ThemeStyleType type)
{
    m_currenttype = type;
    emit ThemeStyleShift();
}

void FJStyleManager::AddManagerWidget(FJBaseWidget * widget)
{
    if(widget)
    {
        connect(this, SIGNAL(ThemeStyleShift()), widget, SLOT(InitStyleslot()));
    }
}

void FJStyleManager::updatePerspectiveState()
{
	emit updatePerspectiveStateSignal();
}

void FJStyleManager::updateSectionThickness(QString name, double thick)
{
	emit SectionThicknessUpdate(name,thick);
}

ThemeStyleType FJStyleManager::GetcurrentStyle()
{
    return m_currenttype;
}



void FJStyleManager::DestoryConnect()
{
    disconnect();
}

void FJStyleManager::areaMeasureLabelSelected(int uuid)
{
	emit measureLabelSelected(uuid);
}

void FJStyleManager::deleteVertexArrays(unsigned int id)
{
    emit signalDeleteVertexArrays(id);
}

void FJStyleManager::deleteBuffers(unsigned int id)
{
    emit signalDeleteBuffers(id);
}


void FJStyleManager::removePointCloudObject(int uuid)
{
    emit pointcloudRemoved(uuid);
}

void FJStyleManager::showErrorMessage(QString mes)
{
    emit signalErrorMessage(mes);
}

void FJStyleManager::updateMappingPointSize(bool state ,int size)
{
    emit signalUpdateMappingPointSize(state , size);
}
