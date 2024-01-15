#include "cchobjectcontrol.h"
#include <QDebug>
#include "ccHObject.h"
#include "ccBBox.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"

#include"core/Standard/qCSF/include/CSF.h"
#include "GenericProgressCallback.h"

#include "pcl/io/pcd_io.h" 
#include "pcl/point_types.h" 
#include "pcl/point_cloud.h"  

#include "cloudcompareutils/publicutils.h"
#include "metahubcomputecl/metahubcomputecl.h"
#include<QApplication>
#include <qdir.h>
//aric.tang_2022.10.25
#include "ccGenericPointCloud.h"
#include "ScalarField.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <QVector3D>
using namespace std;


using namespace CS::ModelControl;
CCHObjectControl::CCHObjectControl(ccHObject* ptr) : QObject()
{
    m_pHobject = ptr;
}

ccPointCloud* CS::ModelControl::CCHObjectControl::detectExtractionTrees(QMap<QString, QString> map)
{
    ccPointCloud* outParam;
    ccPointCloud* offgroundpoint;
    try {
        qDebug() << "Start performing feature extraction!";
        if (!m_pHobject) {
            qDebug() << "hobject is nullptr or source list pointcloud is empty!";
            return nullptr;
        }
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_pHobject);
        outParam = cloud->cloneThis(nullptr, true);
        outParam->setVisible(true);
        int sfIdx = outParam->getScalarFieldIndexByName("Classification");
        if (sfIdx < 0)
        {
            // create the scalar field Classification if needed
            sfIdx = outParam->addScalarField("Classification");
            if (sfIdx < 0)
            {
                ccLog::Error(tr("Not enough memory"));
                return false;
            }
        }
        CCCoreLib::ScalarField* classifSF = outParam->getScalarField(sfIdx);

        pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_in_surf(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_dst(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_not_tree(new pcl::PointCloud<pcl::PointXYZI>);

      

        offgroundpoint = new ccPointCloud();
        std::vector<int> groundIndexes;
        std::vector<int> offGroundIndexes;

        if (map.contains("GROUND"))
        {
            if ("1" == map.value("GROUND"))
            {
                wl::PointCloud csfPC;
                CSF cloudpos(csfPC);
                cloudpos.params.cloth_resolution = map.value("RESOLUTION").toDouble();
                cloudpos.params.class_threshold = map.value("THRESHOLD").toDouble();
                cloudpos.params.rigidness = map.value("GROUNDTYPE").toInt();

                for (int i = 0; i < outParam->size(); i++)
                {
                    wl::Point tmpPoint;
                    tmpPoint.x = outParam->getPoint(i)->x;
                    tmpPoint.y = -outParam->getPoint(i)->z;
                    tmpPoint.z = outParam->getPoint(i)->y;
                    csfPC.push_back(tmpPoint);
                }
                if (cloudpos.do_filtering(groundIndexes, offGroundIndexes) != 0)
                {
                    if (outParam)
                    {
                        delete outParam;
                        outParam = nullptr;
                    }
                    if (offgroundpoint)
                    {
                        delete offgroundpoint;
                        offgroundpoint = nullptr;
                    }
                    return nullptr;
                }
                for (unsigned j = 0; j < groundIndexes.size(); ++j)
                {
                    classifSF->setValue(groundIndexes[j], 1);
                }

                for (unsigned k = 0; k < offGroundIndexes.size(); ++k)
                {
                    classifSF->setValue(offGroundIndexes[k], 0);
                    CCVector3 curPoint;
                    outParam->getPoint(k, curPoint);
                    offgroundpoint->addPoint(curPoint);
                }
                classifSF->computeMinAndMax();
                int intensitysfIdx = offgroundpoint->addScalarField("Intensity");
                if (intensitysfIdx >= 0)
                {
                    CCCoreLib::ScalarField* intensitySF = offgroundpoint->getScalarField(intensitysfIdx);
                    for (unsigned k = 0; k < offGroundIndexes.size(); ++k)
                    {
                        intensitySF->setValue(k, offGroundIndexes[k]);
                    }
                    intensitySF->computeMinAndMax();
                }
            }
        }
        qDebug() << "algorithm execution ends!";
    }
    catch (const std::exception& ex)
    {
        qCritical() << "Caught exception of type: " << QString(typeid(ex).name());
        qCritical() << "Exception details: " << QString(ex.what());

        if (outParam)
        {
            delete outParam;
            outParam = nullptr;
        }
        if (offgroundpoint)
        {
            delete offgroundpoint;
            offgroundpoint = nullptr;
        }
        return nullptr;
    }
    catch (...) {
        qCritical() << "Caught an unknown exception";

        if (outParam)
        {
            delete outParam;
            outParam = nullptr;
        }
        if (offgroundpoint)
        {
            delete offgroundpoint;
            offgroundpoint = nullptr;
        }
        return nullptr;
    }
    return outParam;
}

int CS::ModelControl::CCHObjectControl::setClassifyTypeChange(const int &nType)
{
	if (!m_pHobject) {
		qDebug() << "hobject is nullptr or source list pointcloud is empty!";
		return -1;
	}

	//[!]�ж����ͶԲ���
	if (!m_pHobject->getMetaData("Classify").toBool()) {
		qDebug() << "Classify is error!";
		return -1;
	}

	if (!m_pHobject->isKindOf(CC_TYPES::POINT_CLOUD)) {
		qDebug() << "is not pointcloud!";
		return -1;
	}

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_pHobject);
	if (!cloud) {
		return -1;
	}
	int selftype = cloud->getMetaData("featuredetect").toInt();
	if (selftype == nType)
	{
		return 0;
	}
	int nIndex = cloud->getMetaData("index").toInt();

	QString strName = QString::null;
	switch (nType)
	{
	case 0:
		strName = tr("Other");
		cloud->setColor(ccColor::Rgb(170, 170, 170));
		break;
	case 7:
		strName = tr("Ceiling");
		cloud->setColor(ccColor::Rgb(100, 149, 237));
		break;
	case 17:
		strName = tr("Wall");
		cloud->setColor(ccColor::Rgb(210, 180, 140));
		break;
	case 20:
		strName = tr("Floor");
		cloud->setColor(ccColor::Rgb(139, 69, 19));
		break;

	default:
		strName = tr("Unknown");
		cloud->setColor(ccColor::Rgb(255, 255, 255));
		break;

	}
	ccHObject* parent = m_pHobject->getParent();
	if (parent)
	{
		unsigned childnum = parent->getChildrenNumber();
		for (auto i = nIndex+1; i < childnum; i++)
		{
			ccHObject* child = parent->getChild(i);
			if (child)
			{
				QString FileName = parent->getName();
				child->setName(QString("%1[%2]").arg(FileName).arg(i-1));
				child->setMetaData("index", i - 1);
			}
		}
	}
	QString strFileName = cloud->getMetaData("filename").toString();
	cloud->setName(QString("%1-%2").arg(strFileName).arg(strName));
	cloud->setMetaData("featuredetect", nType);
	cloud->showColors(true);
	switchIndoorClassifyType(nType, m_pHobject, strName);
	return 0;
}

void CS::ModelControl::CCHObjectControl::switchIndoorClassifyType(const int &nType, ccHObject* selfCloud, const QString & typeName)
{	
	ccHObject* parent = selfCloud->getParent();
	if (parent)
	{
		ccHObject* pparent = parent->getParent();
		if (pparent)
		{
			bool existtype = false;
			unsigned childnum = pparent->getChildrenNumber();
			for (auto i = 0;i < childnum;i++)
			{
				ccHObject* child = pparent->getChild(i);
				if (child)
				{
					int type = child->getMetaData("indoortype").toInt();
					if (type == nType)
					{
						existtype = true;
						unsigned cchildnum = child->getChildrenNumber();
						QString FileName = selfCloud->getName();
						selfCloud->setName(QString("%1[%2]").arg(FileName).arg(cchildnum));
						selfCloud->setMetaData("index", cchildnum);
						parent->detachChild(selfCloud);
						child->addChild(selfCloud);
					}
				}
			}
			if (!existtype)
			{
				ccHObject* ccGroup = new ccHObject(pparent->getName() + "-" + typeName);
				QString FileName = selfCloud->getName();
				selfCloud->setName(QString("%1[%2]").arg(FileName).arg(0));
				selfCloud->setMetaData("index", 0);
				parent->detachChild(selfCloud);
				ccGroup->addChild(selfCloud);
				pparent->addChild(ccGroup);
			}
		}
	}
}

int CCHObjectControl::calculateGraphicsEntityAxisRange(CCVector3d dirAxis, CCVector2d & stRange)
{
    if (!m_pHobject) {
        qDebug() << "hobject is nullptr or source list pointcloud is empty!";
        return -1;
    }

    //[!].����������������
    ccBBox pBox = m_pHobject->getBB_recursive();
    CCVector3d maxCorner =  pBox.maxCorner();
    CCVector3d minCorner = pBox.minCorner();



    float max = maxCorner.dot(dirAxis);
    float min = minCorner.dot(dirAxis);
    stRange.x = max;
    stRange.y = min;
    return 0;
}
CCVector3d CCHObjectControl::exeTransformEntityBoxZ(QVector3D pos)
{
	if (!m_pHobject) {
		qDebug() << "hobject is nullptr or source list pointcloud is empty!";
		return CCVector3d();
	}

	ccBBox pBox = m_pHobject->getBB_recursive();
	CCVector3d maxCorner = pBox.maxCorner();
	CCVector3d minCorner = pBox.minCorner();
	//[!].根据矢量计算实体平面范围
	CCVector3d pose3D(pos.x(), pos.y(), pos.z());
   // pose3D.z = maxCorner.z + 0.1;
    return pose3D;
}


CCVector3d CS::ModelControl::CCHObjectControl::exeTransformEntityPosition(QVector3D pos)
{

	if (m_pHobject == NULL)
		return CCVector3d(pos.x(), pos.y(), pos.z());

	//[!].根据矢量计算实体平面范围
	CCVector3d pose3D(pos.x(), pos.y(), pos.z());
	QVector3D axis = m_pHobject->getMetaData("axistype").value<QVector3D>();
	QVector3D up_plane_center = m_pHobject->getMetaData("top_plane_center").value<QVector3D>();
	CCVector3d dir(axis.x(), axis.y(), axis.z());
	CCVector2d range;
	calculateGraphicsEntityAxisRange(dir, range);
	//[!].range x是最大值，y是最小值
	if (dir.x >= 1) {
		pose3D.x = range.x;
	}
	else if (dir.y >= 1) {
		pose3D.y = range.x;
	}
	else if (dir.z >= 1) {
		pose3D.z = range.x;
	}
	else
	{
      
		// 法向量和点云包围盒中点构成一个（非零）平面，求pos在该平面的投影点
		CCVector3d centpoint(up_plane_center.x(), up_plane_center.y(), up_plane_center.z());
        if (centpoint.norm() > 0){
			double a = axis.x(); double b = axis.y(); double c = axis.z();
			double t = a * centpoint.x + b * centpoint.y + c * centpoint.z - (a * pos.x() + b * pos.y() + c * pos.z());
			t /= a * a + b * b + c * c;
			pose3D.x = pos.x() + a * t;
			pose3D.y = pos.y() + b * t;
			pose3D.z = pos.z() + c * t;
        }
        else {
            pose3D.z = range.x;
        }
	
	}
	return pose3D;
}

void CS::ModelControl::CCHObjectControl::outputPGMinfo()
{
	if (!m_pHobject || !m_pHobject->getParent() || !m_pHobject->getParent()->getParent()) {
		qDebug() << "error nullptr";
		return;
	}

	ccHObject* parNode = m_pHobject->getParent();
	ccHObject* parparNode = m_pHobject->getParent()->getParent();

	//[!].��ȡ�����ļ�����
	QString parName = parNode->getName();
	QString parparName = parparNode->getName();
	QString path0 = parName.section(QRegExp("[()]"), 1, 1);
	QString path1 = parparName.section(QRegExp("[()]"), 1, 1);

	QString path;
	QDir dir;
	if (!dir.exists(path0))
	{
		if ((!dir.exists(path1)))
			return;
		else
			path = path1;
	}
	else
	{
		path = path0;
	}

	QByteArray mstr = Utils::PublicUtils::GBKtoGb(path);
	std::string YAML_FILE = std::string(mstr) + "/config.yaml";
	std::string out_pgm_file = std::string(mstr) + "/test_map_2";


	//[!].����ת��
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_pHobject);

	//�����
	ccPointCloud* groundCloud = nullptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	if (parparNode->getChildrenNumber() > 0)
	{
		for (int i = 0; i < parparNode->getChildrenNumber(); i++)
		{
			if (parparNode->getChild(i) && parparNode->getChild(i)->getName() == "ground_cloud")
				groundCloud = ccHObjectCaster::ToPointCloud(parparNode->getChild(i));
		}
	}
}
QString CCHObjectControl::createChildNodeName(const QString & strBaseName)
{

    if (!m_pHobject) {
        qDebug() << "hobject is nullptr or source list pointcloud is empty!";
        return QString::null;
    }


    //[!].获取所有孩子节点
    std::vector<ccHObject *> childrens;
    m_pHobject->filterChildren(childrens);

    //[!].查找是否已经有存在的
    auto findChildName = [=](int index)->bool {
        QString strName = QString("%1%2").arg(strBaseName).arg(index);

        std::vector<ccHObject *>::const_iterator it;
        it = std::find_if(childrens.cbegin(), childrens.cend(), [strName](ccHObject * rhs) {
            return rhs && rhs->getName() == strName;
        });

        return it == childrens.end() ? false : true;
    };

    int index = 1;
    do
    {
        if (!findChildName(index)) {
            break;
        }

    } while (index++);
    return QString("%1%2").arg(strBaseName).arg(index);
}







