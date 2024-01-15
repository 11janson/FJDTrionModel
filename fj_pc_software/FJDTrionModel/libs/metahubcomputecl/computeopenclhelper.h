#pragma once
#include <QObject>
#include <QDebug>
#include "metahubcomputecl_global.h"
#include <boost/compute.hpp>
#include "boost/compute/core.hpp"
#include "boost/compute/device.hpp"
#include "boost/compute/container/vector.hpp"
#include "boost/compute/container/valarray.hpp"
#include "boost/compute/command_queue.hpp"
#include "boost/compute/algorithm/transform.hpp"
#include "boost/compute/algorithm/sort.hpp"

namespace compute = boost::compute;
#define BOOST_COMPUTE_DEBUG_KERNEL_COMPILATION
#define BOOST_COMPUTE_HAVE_THREAD_LOCAL
#define BOOST_COMPUTE_THREAD_SAFE
#define BOOST_COMPUTE_USE_OFFLINE_CACHE

namespace CS {
	namespace MetahubComputeCl {
		class METAHUBCOMPUTECL_EXPORT ComputeOpenClHelper : public QObject
		{
			Q_OBJECT
		public:
			explicit ComputeOpenClHelper(QObject *parent = nullptr);

		protected:
			/**
			*@brief 初始化ComputeOpenCL
			*/
			void initComputeOpenCL(void);
			/**
			*@brief 获取设备绑定的上下文
			*/
			compute::context getDeviceContext(void);
			

		protected:
			compute::device	m_contextDevice;			///<绑定设备
		
		};
	}
}


