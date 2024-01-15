#include "metahubcomputecl.h"
#include <QDebug>
#include "ccHObject.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"

#include "pcl/io/pcd_io.h" 
#include "pcl/point_types.h" 
#include "pcl/point_cloud.h"  


/*****************�Զ�����������**********************/

struct stPointXYZI { float data_c[4]; };
struct stPointXYZ { float data_c[3]; };
BOOST_COMPUTE_ADAPT_STRUCT(stPointXYZI, stPointXYZI, (data_c))
BOOST_COMPUTE_ADAPT_STRUCT(stPointXYZ, stPointXYZ, (data_c))

//BOOST_COMPUTE_ADAPT_STRUCT(pcl::PointXYZI, pcl::PointXYZI, (data_c))



//[!].��ֵת��
BOOST_COMPUTE_FUNCTION(
	stPointXYZ,
	convertPoint,
	(stPointXYZI p),
	{
		stPointXYZ stPoint;
		stPoint.data_c[0] = p.data_c[0];
		stPoint.data_c[1] = p.data_c[1];
		stPoint.data_c[2] = p.data_c[2];
		return stPoint;
	}
);


using namespace CS;
using namespace CS::MetahubComputeCl;
namespace CS {
	namespace MetahubComputeCl {
		class MetahubComputeclPrivate
		{
		public:
			MetahubComputeclPrivate(CS::MetahubComputeCl::MetahubComputecl *pQptr);
			~MetahubComputeclPrivate();

		private:
			friend class CS::MetahubComputeCl::MetahubComputecl;
			CS::MetahubComputeCl::MetahubComputecl *m_pQptr = nullptr;
		};
		
	}
}

MetahubComputeclPrivate::MetahubComputeclPrivate(
	CS::MetahubComputeCl::MetahubComputecl *pQptr)
{
	m_pQptr = pQptr;
}

MetahubComputeclPrivate::~MetahubComputeclPrivate()
{

}

MetahubComputecl::MetahubComputecl(QObject *parent /* = nullptr */)
	:ComputeOpenClHelper(parent)
	,m_pDptr(new CS::MetahubComputeCl::MetahubComputeclPrivate(this))
{
	//[!].��ʼ��
	CS::MetahubComputeCl::ComputeOpenClHelper::initComputeOpenCL();

}


int MetahubComputecl::convertPointCloud(ccPointCloud * pDesPointCloud, 
	pcl::PointCloud<pcl::PointXYZI>::Ptr pSrcPointCloud)
{

	//[!].������Ч�Լ��
	if (!pDesPointCloud || !pSrcPointCloud) {
		qDebug() << "the data is invalid!";
		return -1;
	}

	//[!].��ȡgup�豸������
	compute::context context = this->getDeviceContext();
	compute::device gpu = this->m_contextDevice;

	std::vector<stPointXYZI> host_vector;
	int nSize = pSrcPointCloud->size();
	host_vector.reserve(nSize);


	//host_vector.insert(host_vector.begin(), (void*)(pSrcPointCloud->data()), pSrcPointCloud->data() + nSize);
	

	
	//[!].���� �豸���� ���ҿ������ݵ�gpu
	boost::compute::command_queue queue(context, gpu);
	compute::vector<stPointXYZI> device_vector(host_vector.size(), context);
	compute::copy(
		host_vector.begin(), host_vector.end(), device_vector.begin(), queue
	);

	//[!].�ռ����ݽ��
	auto gpu_areas = compute::vector<stPointXYZ>(nSize, context);
	compute::transform(
		device_vector.begin(),
		device_vector.end(),
		gpu_areas.begin(),
		convertPoint,
		queue
	);

	
	//[!]�������ظ�copy��cpu
	auto cpu_data = std::vector<stPointXYZ>(gpu_areas.size());
	compute::copy(gpu_areas.begin(), gpu_areas.end(), cpu_data.begin(), queue);

	//[!].���ݿ��������
	pDesPointCloud->reserve(nSize);
	memcpy((void*)(pDesPointCloud->getPoint(0)), cpu_data.data(), nSize);
	return 0;
}
