#pragma once
#include <QOBject>
#include <memory>
#include <vector>
#include <map>
#include "CCGeom.h"
#include "pcl/io/pcd_io.h" 
#include "pcl/point_types.h" 
#include "pcl/point_cloud.h"  
#include "modelcontrol_global.h"

class ccHObject;
class ccPointCloud;

namespace CCCoreLib {
    class GenericProgressCallback;
}

namespace CS {
	namespace ModelControl {
		class MODELCONTROLSHARED_EXPORT CCHObjectControl : public QObject
		{
			Q_OBJECT
		public:
			explicit CCHObjectControl(ccHObject *ptr);
		public:
			/**
			*@brief �����˲�����
			*/
			std::shared_ptr<ccPointCloud> appendFilterPointCloud(std::vector<ccHObject*> listPointCloud);
			/**
			*@brief ������
			*/
			ccPointCloud* detectPointCloud(void);
			/**
			*@brief ��ȡ����
			*/
			std::vector<ccPointCloud*> detectFloors(const float &fPercent);
			/**
			*@brief ��ȡ��
			*@param Դ��������
			*/
			std::shared_ptr<ccPointCloud> detectTrees(std::vector<ccHObject*> listPointCloud);

			/**
			*@brief ��ȡ������ȡ������չ�汾�͵���
			*/
			ccPointCloud* detectExtractionTrees(QMap<QString, QString>);
			/**
			*@brief ��ȡ��������
			*/
            //2022.9.16__Aric.tang
			bool detectInDoorFeature(std::vector<ccPointCloud*> &outparamvec, const std::vector<int> & listCurrentSelectint);
            //aric.tang_2022.10.14
            //tree_one by octree
            bool segGroundAndTreeParts(ccPointCloud* cloud_in, ccPointCloud* Terrain, ccPointCloud* Vegetation);
            bool segTreesAndParts(ccPointCloud* Terrain_in, ccPointCloud* Vegetation_in, ccPointCloud* Vegetation_rest_cloud, ccHObject* trees);
			/**
			*@brief ���÷�������
			*/
			int setClassifyTypeChange(const int &nType);

			/**
			*@brief ת�����ڵ��Ʒ��ൽ�½ڵ�
			*/
			void switchIndoorClassifyType(const int &nType, ccHObject* selfCloud,const QString & typeName);
            /**
            *@brief ����ʵ���᷽��Χ��
            */
            int calculateGraphicsEntityAxisRange(CCVector3d dirAxis, CCVector2d &stRange);
			/**
			*@brief 转化实体店位置
			*/
			CCVector3d exeTransformEntityPosition(QVector3D pos);

            CCVector3d exeTransformEntityBoxZ(QVector3D pos);



			/*
			*@brief 点云转PGM2d图
			*/
			std::vector<ccPointCloud*> ply2pgmCloud();
			void outputPGMinfo();

            /*
            @brief 建图z轴倾斜时，z轴拉平
            */
            ccPointCloud* levelZaxis();

            /**
            *@brief 生成孩子节点名称
            *@parame const QString &strBaseName, 
            */
            QString createChildNodeName(const QString &strBaseName);

            /**
            *@brief 执行林业地面点提取
            *@prame 网格
            *@prame 厚度
            */
            ccPointCloud* exeForestryGroundExtract(const double &dGrid,
                const double &dThickness, const double &dPointDensity, const double &dMesh, int &nResultError, CCCoreLib::GenericProgressCallback* progressCb);

            /**
            *@brief 获取点云分类结果
            */
            std::map<int, std::shared_ptr<ccPointCloud>> getPointCloudClassifyResult(void);

            /**
            *@brief 获取融合分类
            */
            std::shared_ptr<ccPointCloud>  getPointCloudMergeClassify(std::vector<int> listClassify);

            /**
            *@brief 执行单木分割,执行3D forset单木分割
            */
            int exeForestRestAndTrees(ccPointCloud* rootNode, pcl::PointCloud<pcl::PointXYZI>::Ptr pGround,
                pcl::PointCloud<pcl::PointXYZI>::Ptr pOther,
                double dTreeCrown,
                double dTreeHeight,
                int iMinpointSize,
                std::vector<ccPointCloud*> &result,
                ccPointCloud* &pOtherPointCloud,
                std::vector<pcl::PointCloud<pcl::PointXYZI>> &Threes
            );

            /**
            *@brief 林业单木提取
            */
            int exeForestsegmentTerrainTrees(
                ccPointCloud* pGround,
                ccPointCloud* pOther,
                int nMinPointNums,
                float fVoxelSize,
                float fIteraNum,
                float fTerrainDistance,
                float fThreshold,
                int nMinNumEle,
                ccPointCloud* &pOtherPointCloud,
                std::vector<pcl::PointCloud<pcl::PointXYZI>> &ThreesXYZI,
                std::vector<ccPointCloud*> &resultTrees);

            bool exeTreeSeperation(ccPointCloud* pOther,
                double dSearchRadius,
                double dVerticalResol,
                std::vector<ccPointCloud*> &result,
                ccPointCloud* &pOtherPointCloud,
                std::vector<pcl::PointCloud<pcl::PointXYZI>> &Threes
                );

            /**
            *@brief 获取子节点列表
            */
            std::vector<ccHObject*> getChildren(void);

            /**
            *@brief 获取子节点
            */
            ccHObject* getChildNode(const QString &strName);

            /**
            *@brief 执行点云分类
            */
            ccPointCloud *exePointCloudClassify(ccPointCloud* pPointCloud, int nValue);

            /**
            *@brief增加特征值特征标示
            */
            bool addScalarFeatureField(const QString &strFieldFeature);

            /**
          *@brief 执行计算单木树的位置
          */
            ccPointCloud* exeCalculateTreePosition(QVector3D &d);
            /**
             *@brief 计算林业单木树树根云的位置
            */
            int calcForestryTreePointCloudPosition(ccPointCloud *pPointCloud, QVector3D& d);
            /**
            *@brief 创建林业树ID
            */
            int createForestryTreeID(void);



		private:
			ccHObject *m_pHobject = nullptr;

		};

	}
}
