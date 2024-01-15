#pragma once
#include <QDialog>
#include "cswidgets/framelessdialog.h"

#include "cloudcomparewidgets/metahublframelessdialog.h"
#include<QStringList>
#include<QMap>
namespace SortOutdoor {
	enum enFeatureDetect {
		Trees    =  0,
		Other    =  1,
        Ground   =  2,
	};

	class OutdoorClassifiedWindowPrivate;
	class OutdoorClassifiedWindow : public CS::MetahubWidgets::MetahubFramelessDialog
	{
		Q_OBJECT
	public:
		OutdoorClassifiedWindow(QWidget *parent = 0);
		~OutdoorClassifiedWindow();

	public:
		/**
		*@brief 获取当前选择的特征提取
		*/
		std::vector<enFeatureDetect> getSelectFentrueDetect(void);

        /**
        *@brief 获取选择特征数据
        */
        QMap<QString,QString> getOutExtractData();
    private:
        QMap<QString, QString> m_OutDoorDataMap;
	private:
		friend class OutdoorClassifiedWindowPrivate;
		OutdoorClassifiedWindowPrivate *m_dptr = nullptr;
		

	};
}


