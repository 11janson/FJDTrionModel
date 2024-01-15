#include "projectmodel.h"
#include <QDebug>
#include "ccHObject.h"
#include "ccNote.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "model/planecutting.h"




using namespace CS;
using namespace  CS::Model;

ProjectModel::ProjectModel(QObject *parent) : Serailizable(parent)
{
    //[!].注册数据类型
    qRegisterMetaType<std::weak_ptr<CS::Model::PlaneCutting>>("std::weak_ptr<CS::Model::PlaneCutting>");
   

    //[!].创建对象
    m_pContextPlaneCutting = std::make_shared<CS::Model::PlaneCutting>();


}

ProjectModel * CS::Model::ProjectModel::instance(void)
{
    static ProjectModel instance;
    return &instance;
}

QJsonObject CS::Model::ProjectModel::serailize() const
{
    QJsonObject object;
    return object;
}

void CS::Model::ProjectModel::unserailize(const QJsonObject &)
{
}

void CS::Model::ProjectModel::setMainWindow(QMainWindow * pMainWidnwos)
{
    m_MainWindow = pMainWidnwos;
    return;
}

QMainWindow *CS::Model::ProjectModel::getMainWindow(void)
{
    return m_MainWindow;
}


void CS::Model::ProjectModel::updateDBTreeCloudFromPath(QString path)
{
    m_saveDBtreeCloudFromPath = path;
}

QString CS::Model::ProjectModel::getSelectCloudFromPath()
{
    return m_saveDBtreeCloudFromPath;
}

void CS::Model::ProjectModel::clearSelectEntitys(void)
{
    if (m_listSelectedEntity.empty()) {
        return;
    }

    m_listSelectedEntity.clear();
    emit signalSelectedEntityChanged();
    return;
}

void CS::Model::ProjectModel::appendSelectEntity(ccHObject * pEntity)
{
    std::vector<ccHObject *>::const_iterator it;
    it = std::find_if(m_listSelectedEntity.cbegin(), m_listSelectedEntity.cend(), [pEntity](ccHObject * rhs) {
        return rhs && rhs == pEntity;
    });

    if (it != m_listSelectedEntity.cend()) {
        //[!].已经有就不添加
        return;
    }
    m_listSelectedEntity.push_back(pEntity);
    emit signalSelectedEntityChanged();
   
}

void CS::Model::ProjectModel::removeSelectEntity(ccHObject * pEntity)
{
    std::vector<ccHObject *>::const_iterator it;
    it = std::find_if(m_listSelectedEntity.cbegin(), m_listSelectedEntity.cend(), [pEntity](ccHObject * rhs) {
        return rhs && rhs == pEntity;
    });

    if (it == m_listSelectedEntity.cend()) {
        //[!].没有找到
        return;
    }
    m_listSelectedEntity.erase(it);
    emit signalSelectedEntityChanged();
}

std::vector<ccHObject*> CS::Model::ProjectModel::getSelectedEntitys(void)
{
    return  m_listSelectedEntity;
}

void CS::Model::ProjectModel::addPointClouds(ccPointCloud* clouds)
{
    bool exists = false;
    for (ccPointCloud* ptr : m_listPointClouds) 
    {
        if (ptr == clouds) 
        {
            exists = true;
            break;
        }
    }

    if (!exists)
        m_listPointClouds.push_back(clouds);
}

std::vector<ccPointCloud*> CS::Model::ProjectModel::getPointClouds(void)
{
    return m_listPointClouds;
}

void CS::Model::ProjectModel::deletePointClouds(ccPointCloud* clouds)
{
    if (!clouds)
    {
        return;
    }
    auto it = m_listPointClouds.begin();
    while (it != m_listPointClouds.end()) {
        if (*it == clouds) {
            // 找到要删除的指针，使用 erase 删除它
            it = m_listPointClouds.erase(it);
        }
        else {
            ++it;
        }
    }
}


void CS::Model::ProjectModel::setPickedScreenPos(const CCVector2d& pos)
{
	m_PickedScreenPos = pos;
	emit signalScreenPickedEntityPointPosChanged();
}

CCVector2d CS::Model::ProjectModel::getPickedScreenPos(void)
{
	return m_PickedScreenPos;
}

void CS::Model::ProjectModel::setPickedEntityPointPos(const QVector3D& pose, const CCVector2d& pos)
{
    m_PickedEntityPointPose = pose;
	//m_PickedScreenPos = pos;
    emit signalPickedEntityPointPosChanged();
}

QVector3D CS::Model::ProjectModel::getPickedEntityPointPos(void)
{
    return m_PickedEntityPointPose;
}

std::weak_ptr<PlaneCutting> CS::Model::ProjectModel::getContextPlaneCutting(void)
{
    return m_pContextPlaneCutting;
}



void CS::Model::ProjectModel::setAttachedGLOwnEntity(ccHObject * pEntity)
{
	m_pAttachedGLEntity = pEntity;
}

ccHObject * CS::Model::ProjectModel::getAttachedGLOwnEntity(void)
{
	return m_pAttachedGLEntity;
}

void CS::Model::ProjectModel::setActivatedNote(ccNote * pActivateNote)
{
    if (!pActivateNote) {
        return;
    }

    m_pActivatedNode = pActivateNote;
    emit signalActivatedNoteChanged();
}
ccNote * CS::Model::ProjectModel::getActivatedNote(void)
{
    return m_pActivatedNode;
}
