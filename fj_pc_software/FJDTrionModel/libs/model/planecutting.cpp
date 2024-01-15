#include "planecutting.h"
#include<QDebug>
#include<QString>
#include"ccHObjectCaster.h"
#include"ccBox.h"
#include"ccPointCloud.h"

#include "modelcontrol/cchobjectcontrol.h"


using namespace CS;
using namespace CS::Model;
#define EPSILON 0.000000001
PlaneCutting::PlaneCutting(QObject *parent) : Serailizable(parent)
{
    qRegisterMetaType<EDataType>("EDataType");
}


PlaneCutting::~PlaneCutting()
{

}
PlaneCutting::PlaneCutting(const PlaneCutting &rh)
{

}

PlaneCutting &PlaneCutting::operator =(const PlaneCutting &rh)
{
    return *this;
}

void PlaneCutting::setSlicePositionCoordinates(CCVector3d pos)
{
    if (fabs(m_PointPos.x - pos.x) > EPSILON)
    {
        m_PointPos.x = pos.x;
       
    }
    
    if (fabs(m_PointPos.y - pos.y) > EPSILON)
    {
        m_PointPos.y = pos.y;
       
    }
    if (fabs(m_PointPos.z - pos.z) > EPSILON)
    {
        m_PointPos.z = pos.z;
      
    }
    emit signalSlicePointChanged();
}
void PlaneCutting::setCurrsetSection(EAxisType axis)
{
    if (m_CurrentAxisType != axis)
    {
        m_CurrentAxisType = axis;
        emit signalSliceDirectionChanged();
    }
}

void PlaneCutting::resetParamer(void)
{
    m_PointCloudScope = 0;
    m_PointPos = CCVector3d(0, 0, 0);
    m_Thickness = 0.1;
    m_CurrentAxisType = ZAXIS;

    m_ScreenThickness = 0.1;
    m_ScreenPointCloudScope = 0;
}

void PlaneCutting::setSliceThickness(float SliceThickValue)
{
    if (fabs(m_Thickness - SliceThickValue) > EPSILON)
    {
        m_Thickness = SliceThickValue;
        emit signalSliceThicknessChanged();
    }
}

void PlaneCutting::setScreenSliceThickness(float SliceThickValue)
{
	if (fabs(m_ScreenThickness - SliceThickValue) > EPSILON)
	{
		m_ScreenThickness = SliceThickValue;
		emit signalScreenSliceThicknessChanged();
	}
}

void PlaneCutting::setPlanePos(float SliderPosition)
{
    if (fabs(m_PointCloudScope - SliderPosition) > EPSILON)
    {
        m_PointCloudScope = SliderPosition;
        emit signalSlicePosChanged();
    }
}

void  PlaneCutting::setScreenPlanePos(float SliderPosition)
{
	if (fabs(m_ScreenPointCloudScope - SliderPosition) > EPSILON)
	{
		m_ScreenPointCloudScope = SliderPosition;
		emit signalScreenSlicePosChanged();
	}
}


bool PlaneCutting::setCreatebutton(bool CreateButton)
{
    return CreateButton;
}

QJsonObject PlaneCutting::serailize() const
{
    QJsonObject leafObject;
    return leafObject;
}
void PlaneCutting::unserailize(const QJsonObject &leafObject)
{
    return;
}


void PlaneCutting::slotCreateButton()
{
	emit signalCreateEntity();
}


EAxisType PlaneCutting::getCurrsetSection()
{
    return m_CurrentAxisType;
}

float PlaneCutting::getSliceCoordinates()
{
    return m_PointCloudScope;
}

float PlaneCutting::getScreenSliceCoordinates()
{
	return m_ScreenPointCloudScope;
}


float PlaneCutting::getSliceThickness()
{
    return m_Thickness;
}

float PlaneCutting::getScreenSliceThickness()
{
	return m_ScreenThickness;
}

CCVector3d PlaneCutting::getSlicePositionCoordinates()
{

    return m_PointPos;
}

void PlaneCutting::slotChangeMode()
{
    qDebug() << "changeModel";
}
