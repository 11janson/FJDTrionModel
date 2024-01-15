#pragma once
/**
* @brief 数据对象的变化锁
*
* 如果对象加锁，则不立即发送对象变化的信号，当当前Locker对象释放是再发送
*/
template<typename ModelType>
class ModelChangedLocker
{
public:
    ModelChangedLocker(ModelType *pModelObj)
        :m_pModelObj(pModelObj)
    {
        m_pModelObj->blockSignals(true);
    }
    ~ModelChangedLocker(){
        m_pModelObj->blockSignals(false);
       
    }
protected:
    ModelType   *m_pModelObj;
};


template<typename ObjectType>
class ObjectSignalLocker
{
public:
    ObjectSignalLocker(ObjectType *pObject)
        :m_pObject(pObject)
    {
        m_pObject->blockSignals(true);
    }
    ~ObjectSignalLocker() {
        m_pObject->blockSignals(false);
    }
protected:
    ObjectType   *m_pObject=nullptr;
};