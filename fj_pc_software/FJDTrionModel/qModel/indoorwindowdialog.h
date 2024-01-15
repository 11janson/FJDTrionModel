#pragma once
#include <QDialog>
#include "cswidgets/framelessdialog.h"
#include "cloudcomparewidgets/metahublframelessdialog.h"

namespace SortIndoor {
	enum enFeatureDetect {
		other = 0,
		floor = 13,
		ceiling = 14,
		wall = 12
	};	
	class IndoorClassifiedWindowPrivate;
	class IndoorClassifiedWindow : public CS::MetahubWidgets::MetahubFramelessDialog
	{

		Q_OBJECT
	public:
		IndoorClassifiedWindow(QWidget *parent = 0);
		~IndoorClassifiedWindow();
	public:
		/**
		*@brief 获取当前选择的特征提取
		*/
		std::vector<enFeatureDetect> getSelectFentrueDetect(void);
	private:
		friend class IndoorClassifiedWindowPrivate;
		IndoorClassifiedWindowPrivate *m_dptr = nullptr;
		

	};
}


