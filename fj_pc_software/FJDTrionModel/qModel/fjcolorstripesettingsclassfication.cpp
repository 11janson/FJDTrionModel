#include "fjcolorstripesettingsclassfication.h"
#include "ui_fjcolorstripesettingsclassfication.h"
#include "fjpointcloudutil.h"
#include "ccPointCloud.h"
#include "ccHObjectCaster.h"
#include "ccScalarField.h"
#include <QPushButton>
#include "mainwindow.h"
#include "ccDBRoot.h"

FJColorStripeSettingsClassfication::FJColorStripeSettingsClassfication(QWidget *parent) :
	FJColorStripeSettingBase(parent),
    ui(new Ui::FJColorStripeSettingsClassfication)
{
    ui->setupUi(this->getContentHolder());
	connect(ui->widget, &FJClassficationTableWidget::dataChanged, this, &FJColorStripeSettingsClassfication::refreashClassficationShow);
	setWindowTitle(QCoreApplication::translate("MainWindow", "Scalar Settings", nullptr));
	GetCancelButton()->setVisible(false);
	GetApplyButton()->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
	GetApplyButton()->setVisible(true);
	connect(GetApplyButton(), &QAbstractButton::clicked, [=](){
		ui->widget->initClassficationColorSetting(m_data);
		refreashClassficationShow();
	});
}

void FJColorStripeSettingsClassfication::refreashClassficationShow()
{
	std::vector<ClassficationData> data = ui->widget->getClassficationColor();
	std::vector<ClassficationData> savedData = FJPointCloudUtil::getClassficationDataFromDoc();
	std::set<int> notshowid;
	for (auto curdata : data)
	{
		if (curdata.isShow == false)
		{
			notshowid.insert(curdata.id);
		}
		for (auto & cursavedData : savedData)
		{
			if (cursavedData.id == curdata.id)
			{
				cursavedData = curdata;
				break;
			}
		}
	}
	m_object->setCurrentNotShowClassficationSet(notshowid);

	FJPointCloudUtil::saveClassficationDataToDoc(savedData);
	FJPointCloudUtil::updateAllPointCloudClassficationDataFromDoc();
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_object);
    if (cloud)
    {
        cloud->scaleFieldColorChange();
        cloud->visibilityArrayChange();
    }
	if (m_Win)
	{
		m_Win->redraw(false);
	}
}

FJColorStripeSettingsClassfication::~FJColorStripeSettingsClassfication()
{
    delete ui;
}


void FJColorStripeSettingsClassfication::init(ccHObject * ent, ccGLWindow * Win)
{
	std::vector<ClassficationData> savedData = FJPointCloudUtil::getClassficationDataFromDoc();
	std::vector<ClassficationData> data;
	if (ent)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (cloud)
		{
			ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
				QString sfName = sf->getName();
				if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
				{
					std::set<int> idset;
					std::set<int> notShowId = cloud->getCurrentNotShowClassficationSet();
					for (size_t i = 0; i < sf->size(); i++) {
						idset.insert(sf->getValue(i));
					}
					for (auto curid : idset)
					{
						bool isexist = false;
						bool isShown = (notShowId.find(curid) == notShowId.end());
						for (auto cursavedata : savedData)
						{
							if (cursavedata.id == curid)
							{
								auto finddata = cursavedata;
								if (!isShown)
								{
									finddata.isShow = false;
								}
								data.push_back(finddata);
								isexist = true;
								break;
							}
						}
						if (!isexist)
						{
							QString newname = QCoreApplication::translate("MainWindow", "Class", nullptr) + QString::number(curid);
							ClassficationData newdata(curid, newname,true,QColor(std::rand() % 255, std::rand() % 255, std::rand() % 255));
							if (!isShown)
							{
								newdata.isShow = false;
							}
							data.push_back(newdata);
							savedData.push_back(newdata);
							sort(savedData.begin(), savedData.end(), [&](ClassficationData & x, ClassficationData & y)->bool {return (x.id < y.id); });
						}
					}
					FJPointCloudUtil::saveClassficationDataToDoc(savedData);
				}

			}
		}
	}
	m_data.clear(); 
	m_data = data;
	ui->widget->initClassficationColorSetting(data);

}