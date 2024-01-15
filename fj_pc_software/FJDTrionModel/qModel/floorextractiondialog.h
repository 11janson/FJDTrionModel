#pragma once
#include <QDialog>
#include "cswidgets/framelessdialog.h"
#include "cloudcomparewidgets/metahublframelessdialog.h"

namespace floorextraction {

	class FloorExtractiondialogPrivate;
	class FloorExtractiondialog : public CS::MetahubWidgets::MetahubFramelessDialog
	{

		Q_OBJECT
	public:
		FloorExtractiondialog(QWidget *parent = 0);
		~FloorExtractiondialog();
	public:
		int getDialogData();
	private:
		friend class FloorExtractiondialogPrivate;
		FloorExtractiondialogPrivate *m_dptr = nullptr;
	};
}


