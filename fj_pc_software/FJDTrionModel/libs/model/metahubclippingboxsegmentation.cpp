#include "metahubclippingboxsegmentation.h"
#include<QDebug>
#include<QString>
using namespace CS;
using namespace CS::Model;
ClippingBoxAndSegment* ClippingBoxAndSegment::instance = nullptr;
QMutex  ClippingBoxAndSegment::m_mutexClippingBoxAndSegment;
ClippingBoxAndSegment::ClippingBoxAndSegment(QObject *parent) 
{

}


ClippingBoxAndSegment::~ClippingBoxAndSegment()
{

}

ClippingBoxAndSegment* ClippingBoxAndSegment::GetInstance()
{
    if (instance == nullptr)
    {
        QMutexLocker locker(&m_mutexClippingBoxAndSegment);
        if (instance == nullptr)
        {
            instance = new ClippingBoxAndSegment();
        }
    }
    return instance;
}
