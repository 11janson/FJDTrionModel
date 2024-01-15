//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################

#include "MLSSmoothingUpsampling.h"

//Local
#include "dialogs/MLSDialog.h"
#include "../utils/PCLConv.h"
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//PCL
#include <pcl/surface/mls.h>
#include <pcl/common/io.h> // for getFieldIndex
#include <pcl/search/kdtree.h> // for KdTree

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccScalarField.h>
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QThread>
#include <QtConcurrent>
#include <QFuture>
#include"model/projectmodel.h"
#ifdef LP_PCL_PATCH_ENABLED
#include "../utils/copy.h"
#endif

template <typename PointInT, typename PointOutT>
static int SmoothMLS(	const typename pcl::PointCloud<PointInT>::Ptr &incloud,
						const MLSParameters &params,
						typename pcl::PointCloud<PointOutT>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
				, pcl::PointIndicesPtr &mapping_ids
#endif
	)
{
	typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT>);

#ifdef _OPENMP
	//create the smoothing object
	pcl::MovingLeastSquares< PointInT, PointOutT > smoother;
	smoother.setNumberOfThreads(omp_get_max_threads());
#else
	pcl::MovingLeastSquares< PointInT, PointOutT > smoother;
#endif
	smoother.setInputCloud(incloud);
	smoother.setSearchMethod(tree);	
	smoother.setSearchRadius(params.search_radius_);
	smoother.setComputeNormals(params.compute_normals_);
	if (params.polynomial_fit_)
	{
		smoother.setPolynomialOrder(params.order_);
		smoother.setSqrGaussParam(params.sqr_gauss_param_);
	}

	switch (params.upsample_method_)
	{
	case (MLSParameters::NONE):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::NONE );
			//no need to set other parameters here!
			break;
		}

	case (MLSParameters::SAMPLE_LOCAL_PLANE):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::SAMPLE_LOCAL_PLANE);
			smoother.setUpsamplingRadius(params.upsampling_radius_);
			smoother.setUpsamplingStepSize(params.upsampling_step_);
			break;
		}

	case (MLSParameters::RANDOM_UNIFORM_DENSITY):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::RANDOM_UNIFORM_DENSITY );
			smoother.setPointDensity(params.step_point_density_);
			break;
		}

	case (MLSParameters::VOXEL_GRID_DILATION):
		{
			smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::VOXEL_GRID_DILATION);
			smoother.setDilationVoxelSize(static_cast<float>(params.dilation_voxel_size_));
			smoother.setDilationIterations(params.dilation_iterations_);
			break;
		}
	}

	smoother.process(*outcloud);

#ifdef LP_PCL_PATCH_ENABLED
	mapping_ids = smoother.getCorrespondingIndices();
#endif

	return BaseFilter::Success;
}

MLSSmoothingUpsampling::MLSSmoothingUpsampling()
	: BaseFilter(FilterDescription(	QCoreApplication::translate("MLSSmoothingUpsampling", "Reconstructing...", nullptr),
									"MLS",
		QCoreApplication::translate("MLSSmoothingUpsampling", "", nullptr),
									":/toolbar/PclUtils/icons/mls_smoothing.png"))
{
}

MLSSmoothingUpsampling::~MLSSmoothingUpsampling()
{
}

int MLSSmoothingUpsampling::compute()
{
	//pointer to selected cloud
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		return InvalidInput;
	}

	ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
    //[!].获取线程数
    int maxThreadCount = QThread::idealThreadCount();
    QMutex sMutex;

    int iChunkCount = maxThreadCount;
    //[!].计算分组策略
    if (cloud->size() > 10000000){
        //[!].千万级别上亿
        iChunkCount = maxThreadCount * 30 + (iChunkCount / 30000000) * 10;
    }
    else if (cloud->size() > 5000000){
        //[!].五百万以上
        iChunkCount = 5;
    }
    else if (cloud->size() > 1000000){
        iChunkCount = 2;
    }
    else
    {
        iChunkCount = 1;
    }
    
    //[!].点云数据分组
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> chunkCloudxyz = cc2smReader(cloud).getRawXYZChunk(iChunkCount);
    if (chunkCloudxyz.empty()) {
        return NotEnoughMemory;
    }

    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> ccPointCloudReslut;

    //[!].mls执行算法功能
    auto functionSmoothMLS = [&](pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud){

        //create storage for outcloud
        pcl::PointCloud<pcl::PointNormal>::Ptr rawCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
#ifdef LP_PCL_PATCH_ENABLED
        pcl::PointIndicesPtr mappingIndices;
        SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(xyzCloud, m_parameters, rawCloudWithNormals, mappingIndices);
#else
        SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(xyzCloud, m_parameters, rawCloudWithNormals);
#endif

        {
            QMutexLocker lock(&sMutex);
            ccPointCloudReslut.push_back(rawCloudWithNormals);
        }
    };

    
   
    //[!].执行算法
    QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount * 2);
    QFuture<void> fureslut = QtConcurrent::map(chunkCloudxyz, functionSmoothMLS);
    while (!fureslut.isFinished()) {
        QThread::msleep(20);
        QCoreApplication::processEvents();
    }

   
    

    //[!].合并点云数据
    if (ccPointCloudReslut.empty()) {
        return ComputationError;
    }
    auto it = ccPointCloudReslut.begin();
    pcl::PointCloud<pcl::PointNormal>::Ptr pCloudWithNormals = *it;
    ccPointCloudReslut.erase(it);
    unsigned int pCoundSize = 0;
    for (int i = 0; i < ccPointCloudReslut.size(); i++) {
        auto pCound = ccPointCloudReslut[i];
        pCoundSize += pCound->size();
    }

    unsigned int index = pCloudWithNormals->size();
    pCloudWithNormals->resize(pCloudWithNormals->size() + pCoundSize);
    for (int i = 0; i < ccPointCloudReslut.size(); i++) {
        auto pCound = ccPointCloudReslut[i];

        for (int j = 0; j < pCound->size(); j++) {
            pCloudWithNormals->at(index).x = pCound->at(j).x;
            pCloudWithNormals->at(index).y = pCound->at(j).y;
            pCloudWithNormals->at(index).z = pCound->at(j).z;
           // if (cloud->hasNormals()) 
            {
                pCloudWithNormals->at(index).normal_x = pCound->at(j).normal_x;
                pCloudWithNormals->at(index).normal_y = pCound->at(j).normal_y;
                pCloudWithNormals->at(index).normal_z = pCound->at(j).normal_z;

            }
            index++;
        }
    }


    PCLCloud cloudWithNormals;
    TO_PCL_CLOUD(*pCloudWithNormals, cloudWithNormals);
    ccPointCloud* outputCCCloud = pcl2cc::Convert(cloudWithNormals);
    if (outputCCCloud) {
#ifdef LP_PCL_PATCH_ENABLED
        //copy the original scalar fields here
        copyScalarFields(cloud, outputCCCloud, mappingIndices, true);
        //copy the original colors here
        copyRGBColors(cloud, outputCCCloud, mappingIndices, true);
#endif
        //copy global shift & scale
        outputCCCloud->copyGlobalShiftAndScale(*cloud);

    }
    ccHObject * parentCloud = static_cast<ccHObject*>(cloud)->getParent();
	cloud->setEnabled(false);
	if (!parentCloud){
        return 0;
	}
    int count = 2;
    outputCCCloud->setDisplay(cloud->getDisplay());
    QString cloudName = cloud->getName() + QString(".smoothed1");
    while (1) {
        bool found = false;
        for (int i = 0; i < parentCloud->getChildrenNumber(); i++) {
            if (parentCloud->getChild(i)->getName() == cloudName) {
                cloudName.chop(1);
                cloudName += QString::number(count);
                count++;
                found = true;
                break;
            }
        }
        if (!found){
            break;
        }
    }
    outputCCCloud->setName(cloudName); 
    cloud->getParent()->addChild(outputCCCloud);
    if (!m_PutSaveFilePath.isEmpty()) {
        QDir dirs(m_PutSaveFilePath);
        m_PutSaveFilePath = dirs.filePath(outputCCCloud->getName());
    }
	emit newEntity(outputCCCloud);
	return Success;
}

int MLSSmoothingUpsampling::getParametersFromDialog()
{
	MLSDialog dialog(m_app ? m_app->getMainWindow() : nullptr);
	dialog.setWindowTitle(QCoreApplication::translate("MLSDialog", "MLS smooth processing", nullptr));
    dialog.setFileSavePath(CS::Model::ProjectModel::instance()->getSelectCloudFromPath());
	if (!dialog.exec())
	{
		return CancelledByUser;
	}
    if (!dialog.getSaveFilePath().isEmpty()) {
        QDir dir(dialog.getSaveFilePath());
        m_PutSaveFilePath = dir.path();
    }
	//we need to read all the parameters and put them into m_parameters
	m_parameters.search_radius_ = dialog.search_radius->value();
	m_parameters.compute_normals_ = true;
	m_parameters.polynomial_fit_ = true;
	m_parameters.order_ = 2;
	m_parameters.sqr_gauss_param_ = dialog.search_radius->value() * dialog.search_radius->value();

	m_parameters.upsample_method_ = dialog.GetCurrentMethod();

	m_parameters.upsampling_radius_ = dialog.upsampling_radius->value();
	m_parameters.upsampling_step_ = dialog.upsampling_step_size->value();
	m_parameters.step_point_density_ = dialog.step_point_density->value();
	m_parameters.dilation_voxel_size_ = dialog.dilation_voxel_size->value();

	return Success;
}

QString MLSSmoothingUpsampling::getSaveCloudPath()
{
    return m_PutSaveFilePath;
}

void MLSSmoothingUpsampling::clearOutPutPointCloudPath()
{
    m_PutSaveFilePath.clear();
}

template int SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(	const pcl::PointCloud<pcl::PointXYZ>::Ptr &incloud,
															const MLSParameters &params,
															pcl::PointCloud<pcl::PointNormal>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
															,	pcl::PointIndicesPtr &used_ids
#endif
  );
