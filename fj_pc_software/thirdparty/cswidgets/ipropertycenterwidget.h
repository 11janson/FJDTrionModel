#pragma once
#include "cswidgets_global.h"
#include "icenterwidget.h"
#include <QLabel>
#include <QTextEdit>
#include <QTableWidget>
#include <QWidget>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>

namespace CS {
	namespace Widgets {
		class CSWIDGETS_EXPORT IPropertyCenterWidget : public ICenterWidget
		{
			Q_OBJECT
		public:
			explicit IPropertyCenterWidget(Utils::Id id, QWidget *parent = 0);
			virtual ~IPropertyCenterWidget();
		public:
			/**
			* @brief 实时翻译时，刷新UI文字显示
			*/
			virtual void retranslateUi();

			/**
			*@brief 设置标题名称
			*/
			void setCaptionName(QString strName);

			/**
			*@brief 设置详情描述
			*/
			void setDetailDescription( QStringList list);
			/**
			*@brief 添加文字到TableWidget
			*/
			void addStrToTableWidget(int colmn,  QStringList &);
			/**
			*@brief 添加Pic到TableWidget
			*/
			void addPicToTableWidget(int row, QString &strName);

			/**
			*@brief 更新工件高级参数图片显示区
			*/
			void updateSeniorParamLabelStyle();

			/**
			*@brief 更新详细参数显示
			*/
			void updataWeldElaLabelStyle();
            /**
            *@brief 更新SpainBox值
            */
            void updataSpainBoxFromParam(QStringList listParam);
            /**
            *@brief 更新参数值
            */
            QStringList updataParamFromSpainBox();
		private:
            /**
            *@brief 创建界面
            */
			void createWidgets();
            /**
            *@brief 创建连接
            */
			void createConnects();	
			/**
			*@brief 创建基础参数显示
			*/
			void createBasisParam();
            /**
            *@brief 设置SpinBox属性
            */
            void setSpinBoxProperty(int nId, int nDecimals, double nMin, double nMax, double SingleStep);
	
		public slots:
			virtual void slotModifyParam(void);
            virtual void slotRevertParam(void);
		signals:
			 void signalModifyParam(void);
             void signalRevertParam(void);
		protected slots:
			void  changeParamInfo(void);
			void  confirmModifyParam(void);
            void  revertParam(void);
		protected:	
			QWidget			*m_pSetParamWidget = nullptr;					///<工件私有参数设置区
			QLabel			*m_pShowParamPicLabel = nullptr;				///<工件详细信息描述区
			QTableWidget    *m_pDetailContentTabel = nullptr;				///<详情内容列表
			QPushButton     *m_pThumbnailButton = nullptr;
		protected:
			int nIndex = 1;
			QLabel			*m_pDisplayNameLabel = nullptr;					///<显示名称
			QTextEdit		*m_pDetailTextEdit = nullptr;					///<详情描述	
			QWidget			*m_pWeldElaborateWidget = nullptr;				///<工件详细信息描述区
			QStackedWidget	*m_pContextStackedWidget = nullptr;				///<工件参数设置区
			
			QPushButton		*m_pPageSwitchButton = nullptr;					///<基础参数页面切换button
			QPushButton		*m_pAlterParamButton = nullptr;
            QPushButton		*m_pRevertParamButton = nullptr;                ///< 重置参数
			QWidget			*m_pFirstTabWidget = nullptr;					///<工件私有参数及详情显示区			
			QWidget			*m_pSecondTabWidget = nullptr;					///<工件共有信息描述区

            QMap<int, QLabel *> m_pLabelMap;
            QMap<int, QDoubleSpinBox *> m_pSpinBoxMap;
		};		

	}
}


