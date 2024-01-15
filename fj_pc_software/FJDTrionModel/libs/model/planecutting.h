#pragma once
#include <QObject>
#include <memory>
#include <vector>
#include "model_global.h"
#include "cloudcompareutils/serailizable.h"
#include"CCGeom.h"
#include"projectmodel.h"
enum  EAxisType
{
    XAXIS = 1,
    YAXIS,
    ZAXIS
};

enum EDataType
{
	SliceDirection = 1,
	SlicePoint,
	SlicePos,
	SliceThickness
};

namespace CS {
    namespace Model {

        class MODEL_EXPORT PlaneCutting : public Serailizable
        {
            Q_OBJECT
        public:
            explicit PlaneCutting(QObject *parent = nullptr);
            virtual ~PlaneCutting();
            PlaneCutting(const PlaneCutting &rh);
            PlaneCutting &operator =(const PlaneCutting &rh);
        public:
            virtual QJsonObject serailize() const;
            virtual void unserailize(const QJsonObject &);
        signals:
            void signalModelChanged();
            void signalSliceDirectionChanged();
            void signalSlicePointChanged();
            void signalSlicePosChanged();
			void signalScreenSlicePosChanged();
            void signalSliceThicknessChanged();
			void signalScreenSliceThicknessChanged();
            void signalCreateEntity();
        public slots:
            /**
            *@brief UI界面创建按钮槽
            */
            void slotCreateButton();
        public:
            /**
            *@brief 轴方向
            */
            void setCurrsetSection(EAxisType);
            /**
             *@brief 切面位置坐标
             */
            void setSlicePositionCoordinates(CCVector3d pos);
            /**
             *@brief 切面位置滑条
             */
            void  setPlanePos(float);
			/**
			 *@brief 垂直屏幕切面位置滑条
			 */
			void  setScreenPlanePos(float);
            /**
            *@brief 创建按钮状态
            */
            bool setCreatebutton(bool);
            /**
           *@brief 切面厚度
           */
            void setSliceThickness(float);
			/**
			*@brief 垂直屏幕切面厚度
			*/
			void setScreenSliceThickness(float);
            /**
            *@brief 重置参数
            */
            void resetParamer(void);
        public:
            /**
            *@brief 轴方向
            */
            EAxisType getCurrsetSection();
            /**
             *@brief 切面坐标
             */
            CCVector3d getSlicePositionCoordinates();
            /**
             *@brief 切面位置滑条
             */
            float getSliceCoordinates();
            /**
             *@brief 切面位置厚度
             */
            float getSliceThickness();
			/**
			 *@brief 垂直屏幕切面位置厚度
			 */
			float getScreenSliceThickness();
			/**
		 *@brief 垂直屏幕切面位置滑条
		 */
			float getScreenSliceCoordinates();
        private:
            CCVector3d         m_PointPos;                       //x,y,z坐标
            float              m_PointCloudScope = 0;            //滑条位置
            float              m_Thickness = 0.1;                  //切割厚度
            bool               m_CreatButton = false;            //创建按钮
            EAxisType          m_CurrentAxisType=ZAXIS;

			float              m_ScreenThickness = 0.1;                  //切割厚度
			float              m_ScreenPointCloudScope = 0;            //滑条位置

        public slots:
            void slotChangeMode();
        };
        Q_DECLARE_METATYPE(CS::Model::PlaneCutting);


    }
}




