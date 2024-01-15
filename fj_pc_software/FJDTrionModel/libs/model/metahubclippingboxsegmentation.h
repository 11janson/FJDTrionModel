#pragma once

//##########################################################################
//#            新增裁剪盒与切割数据层                                      #
//#                                                                        #
//#                                                                        #
//#                                                                        #
//##########################################################################
#include <QObject>

#include<QMutex>
#include "model_global.h"



namespace CS {
    namespace Model {
        class MODEL_EXPORT ClippingBoxAndSegment : public QObject
        {
            Q_OBJECT
        public:
            explicit ClippingBoxAndSegment(QObject *parent = nullptr);
            virtual ~ClippingBoxAndSegment();
            static ClippingBoxAndSegment* GetInstance();
        public:
            bool disPlayInorOutPoint = true;
        private:
            static ClippingBoxAndSegment *instance;
            static QMutex  m_mutexClippingBoxAndSegment;

        };
    }
}




