#include "computeopenclhelper.h"
#include <QDebug>
using namespace CS::MetahubComputeCl;

ComputeOpenClHelper::ComputeOpenClHelper(QObject *parent /* = nullptr */)
	:QObject(parent)
{

}

void ComputeOpenClHelper::initComputeOpenCL()
{
	m_contextDevice = compute::system::default_device();
	qDebug() << "version: " << m_contextDevice.version().c_str();
	qDebug() << "driver version: " << m_contextDevice.driver_version().c_str();

}

compute::context CS::MetahubComputeCl::ComputeOpenClHelper::getDeviceContext(void)
{
	return compute::context(m_contextDevice);
}
