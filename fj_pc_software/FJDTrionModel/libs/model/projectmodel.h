#pragma once
#include <QObject>
#include <memory>
#include <vector>
#include <QPointF>
#include <QMainWindow>
#include <QVector3D>
#include "cc2DLabel.h"
#include "model_global.h"
#include "cloudcompareutils/serailizable.h"


class ccHObject;
class ccPointCloud;
class ccNote;
class ccStaticLabel;
namespace CS {
    namespace Model {
        enum StatusMessageType
        {
            Normal,     ///< 常态
            Warning,    ///< 警告
            Error,      ///< 错误
        };

        class PlaneCutting;
        class MODEL_EXPORT ProjectModel : public Serailizable
        {
            Q_OBJECT
        private:
            explicit ProjectModel(QObject *parent = nullptr);

        public:
            /**
            *@brief 获取实例
            */
            static ProjectModel* instance(void);
        protected:
            /**
            *@brief 序列化
            */
            virtual QJsonObject serailize() const;
            /**
            *@brief 反序列化
            */
            virtual void unserailize(const QJsonObject &);

        public:/**************************静态标签*********************************/
        signals:
            /**
            *@brief 选择当前的静态标签
            */
            void signalCurrentSelectStaticLableChanged(ccStaticLabel *pStaticLabel);

        public:
            /****************************主体框架信号***********************************************/
            /**
            *@brief 设置主框架
            */
            void setMainWindow(QMainWindow *pMainWidnwos);

            /**
            *@brief 获取主框架
            */
            QMainWindow *getMainWindow(void);
            /**
            *@brief 状态栏信息
            */
            void updateDBTreeCloudFromPath(QString path);
            /**
            *@brief 获取当前选择的点云来源路径
            */
            QString getSelectCloudFromPath();

        signals:
            /**
            *@biref 刷新激活的窗口 DB增加数据的时候
            */
            void signalRedrawActiveWindow(void);

            /**
            *@biref 刷新2d的窗口 
            */
            void signalRedraw2DWindow(void);

            /**
            *@brief 更新所有的窗口 DB增加数据子窗口都刷新
            */
            void signalRedrawAllWindows(void);

            /**
            *@brief 更新DB数据model层数据 DB增加子节点更新Tree
            */
            void signalRenewalDBTreeModel(void);

            /**
            *@刷新界面 DB节点没变，只是刷新三维渲染
            */
            void signalUpdateWindow(void);

            /**
            *@brief 状态栏信息
            */
            void signalStatusMessage(QString strMsg, StatusMessageType type);
            /**
            *@brief 注册激活更新UI模块
            */
            void signalRegisterActivateUpdateUIModel(void);
            /**
            *@brief 将需要保存的点云属性发送给保存接口
            */
            void signalSaveCloudToLocal(ccHObject* obj, QString filePath,bool state = false);

        public:
            /*****************************实体选择部分**************************************/
            /**
            *@brief 清除当前选择的实体对象
            */
            void clearSelectEntitys(void);

            /**
            *@brief 添加选择的实体对象
            */
            void appendSelectEntity(ccHObject *pEntity);

            /**
            *@brief 移除选择的实体对象
            */
            void removeSelectEntity(ccHObject *pEntity);
         
            /**
            *@brief 获取当前选择的实体对象
            */
            std::vector<ccHObject *> getSelectedEntitys(void);

            std::vector<ccPointCloud*> getPointClouds(void);

            void addPointClouds(ccPointCloud* clouds);

            void deletePointClouds(ccPointCloud* clouds);

            void clearPointClouds() { m_listPointClouds.clear(); }
        signals:
            /**
            *@brief 选择的事情对象发生改变
            */
            void signalSelectedEntityChanged(void);

            /**
            *@brief 移除选择的实体
            */
            void signalRemoveSelectedEntityChanged(void);

        public: 

            /**
            *@brief 设置鼠标拾取实体点位置
            */
            void setPickedEntityPointPos(const QVector3D& pose, const CCVector2d& pos);

			/**
			*@brief 设置鼠标拾取实体点屏幕位置
			*/
			void setPickedScreenPos(const CCVector2d& pos);

            /**
            *@brief 获取鼠标拾取点的位置
            */
            QVector3D getPickedEntityPointPos(void);

			CCVector2d getPickedScreenPos();
        signals:
            /**
            *@brief 当前拾取的发生变化
            */
            void signalPickedPointChanged(void);

            /**
            *@brief 实体选择点位置发生变化
            */
            void signalPickedEntityPointPosChanged(void);

			/**
			*@brief 垂直屏幕功能实体选择点位置发生变化
			*/
			void signalScreenPickedEntityPointPosChanged(void);


            //// void GLWindows mouse rightMouse pressed
            void signalGLWindowsMousePressed(QVector3D pos);
			/**
			*@brief GLWindows鼠标右键点击，折线段用
			*/
			void signalGLWindowsRightMouseClicked(void);

            /**
            *@brie GLWindows鼠标移动
            */
            void signalGLWindowsMouseMove(QVector3D pos);
		
			/**
			*@brie GLWindows鼠标移动
			*/
			void signalGLWindowsMouseMoveEfficiency(CCVector3d pos);

            /**
            *@brief GLWindows实体选择中
            */
            void signalGLWindowsEntityChanged(ccHObject* entity);

            /**
            *@brief GLWindows----hover
            */
            void signalGLWindowsEntityHovering(ccHObject* entity, QVector3D pos);
          

            
			/**
			*@brief GLWindows 区域选择
			*/
			void signalGLWindowsRectSelect(CCVector3d topLeft, CCVector3d bottomRight);

			/**
			*@brief 鼠标左键移动元素编辑元素释放
			*/
			void signalGLWindowMouseLeftMovedRelease(void);

			/**
			*@brief GLWindows Hover功能
			*/
			void signalGLWindowMouseHoverSelect(int selectedID);


			/**
			*@brief GLWindows  鼠标释放选择
			*/
			void signalGLWindowsMouseReleasePicking(void);
        public:
            /**
            *@brief 设置激活的注释
            */
            void setActivatedNote(ccNote *pActivateNote);

            /**
            *@brief 获取当前激活的注释
            */
            ccNote* getActivatedNote(void);
        signals:
            /**
            *@brief 当前激活的注释发生改变
            */
            void signalActivatedNoteChanged(void);

        public:
            /**
            *@brief 获取绑定平面分割
            */
            std::weak_ptr<PlaneCutting> getContextPlaneCutting(void);
		public:
			/**
			*@brief 添加附着在GL中自己的实体
			*/
			void setAttachedGLOwnEntity(ccHObject *pEntity);

			/**
			*@brief 附着在GL中自己的实体
			*/
			ccHObject* getAttachedGLOwnEntity(void);

        private:
			ccHObject		*m_pAttachedGLEntity = nullptr;
            std::vector<ccHObject *>                            m_listSelectedEntity;                       //< 当前激活的实体列表;
            std::shared_ptr<CS::Model::PlaneCutting>			m_pContextPlaneCutting;                     ///<获取绑定平面分割
			std::vector<ccPointCloud *>                            m_listPointClouds;                       //< 当前激活的实体列表;
        private:
            ccNote *m_pActivatedNode = nullptr;                                                                ///<当前的激活的注释
        private:
            QVector3D   m_PickedEntityPointPose;                                                              ///< 鼠标拾取实体点位置
            QMainWindow     *m_MainWindow = nullptr;                                                          ///< 
			CCVector2d   m_PickedScreenPos;                                                              ///< 鼠标拾取实体点位置
        private:
            QString m_saveDBtreeCloudFromPath;
        };

    }
}



