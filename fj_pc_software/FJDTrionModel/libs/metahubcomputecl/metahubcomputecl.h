#pragma once
#include "metahubcomputecl_global.h"
#include "computeopenclhelper.h"
#include <memory>
#include <vector>

#include "pcl/point_types.h" 
#include "pcl/point_cloud.h"  

class ccHObject;
class ccPointCloud;
namespace CS {
	namespace MetahubComputeCl {

		class MetahubComputeclPrivate;
		class METAHUBCOMPUTECL_EXPORT MetahubComputecl : public ComputeOpenClHelper
		{
			Q_OBJECT
		public:
			explicit MetahubComputecl(QObject *parent = nullptr);

			/**
			*@brief 数据类转化
			*/
			int convertPointCloud(ccPointCloud *pDesPointCloud, 
				pcl::PointCloud<pcl::PointXYZI>::Ptr  pSrcPointCloud);
		private:
			friend class CS::MetahubComputeCl::MetahubComputeclPrivate;
			CS::MetahubComputeCl::MetahubComputeclPrivate *m_pDptr = nullptr;
		};

	}
}


