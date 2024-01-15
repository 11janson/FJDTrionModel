//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "BaseFilter.h"
//qCC_db
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>

//CCCoreLib
#include <CCPlatform.h>

//qCC
#include <ccMainAppInterface.h>

//Qt
#include <QAction>
#include <QFuture>
#include <QApplication>
#include <qtconcurrentrun.h>
#include <FJframelessprocessdialog.h>
#include"model/projectmodel.h"
#include"cswidgets/framelessmessagebox.h"
//system
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif
#include "ccProgressDialog.h"

class InformationPrompt;
BaseFilter::BaseFilter(FilterDescription desc, ccMainAppInterface* app)
	: m_desc(desc)
	, m_action(new QAction(desc.icon, desc.entryName, this))
	, m_app(app)
	, m_showProgress(true)
{
	m_action->setStatusTip(m_desc.statusTip);

	//connect this action
	connect(m_action, &QAction::triggered, this, &BaseFilter::performAction);
}

void BaseFilter::throwError(int errCode)
{
	QString errMsg = getErrorMessage(errCode);

	if (errCode == CancelledByUser)
	{
		// don't emit this particular 'error' message
		ccLog::Warning("[qPCL] " + errMsg);
	}
	else if (errCode < 0)
	{
		//DGM: as libraries shouldn't issue message themselves, it should be sent to the plugin via a signal
		emit newErrorMessage(errMsg);
	}
}

void BaseFilter::updateSelectedEntities(const ccHObject::Container& selectedEntities)
{
	m_selectedEntities = selectedEntities;

	if (m_action)
	{
		m_action->setEnabled(checkSelected());
	}
}

void BaseFilter::performAction()
{
	CS::Model::ProjectModel::instance()->signalStatusMessage("The smaller the search radius, the greater the smoothness.",CS::Model::StatusMessageType::Normal);
	//check if selected entities are good
	if (!checkSelected())
	{
		assert(false);
		throwError(InvalidInput);
		return;
	}

	//get parameters from the dialog (if any)
	int result = getParametersFromDialog();

	if (result != Success)
	{
		throwError(result);
		return;
	}

	//if so, go ahead with start()
	result = start();
	if (result != Success)
	{
		throwError(result);
        CS::Widgets::FramelessMessageBox::critical(CS::Model::ProjectModel::instance()->getMainWindow(), tr("Error"), tr("Out of memory, please close other software or delete useless data and try again!"));
		return;
	}

	CS::Model::ProjectModel::instance()->signalStatusMessage("", CS::Model::StatusMessageType::Normal);

}

bool BaseFilter::checkSelected() const
{
	// default mode: only one cloud
	return (m_selectedEntities.size() == 1 && m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
}

static BaseFilter* s_filter = nullptr;
static int s_computeStatus = BaseFilter::ComputationError;

static void DoCompute()
{
	if (s_filter)
	{
		s_computeStatus = s_filter->compute();
	}
	else
	{
		s_computeStatus = BaseFilter::ComputationError;
	}
}

int BaseFilter::start()
{
	static bool s_computing = false;
	if (s_computing)
	{
		return ThreadAlreadyInUse;
	}

    ccProgressDialog progressCb(false, (QWidget *)(m_app->getMainWindow()));
    progressCb.setMethodTitle(tr("Loading"));
    progressCb.setInfo(tr("Processing, please wait."));
	progressCb.setFixedWidth(350);
	//progressCb.setFixedSize(350,190);
	progressCb.setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	progressCb.setWindowModality(Qt::ApplicationModal);
	progressCb.setRange(0,100);
	progressCb.setValue(0);
	if (m_showProgress)
	{
		progressCb.setWindowTitle(m_desc.filterName);
		progressCb.show();
		QApplication::processEvents();
	}
	s_computeStatus = ComputationError;
	s_filter = this;
	s_computing = true;

    //[!].���ݲ�ͬ�ĵ�����������Ԥ����ͬ��ʱ��
    int iPountCount = 0;
    ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
    if (!cloud) {
        return CancelledByUser;
    }
    iPountCount = cloud->size();

    float iMinuteCount = 3; //����
    if (iPountCount > 10000000){//[!].ǧ�򼶱�
        iMinuteCount = iPountCount / 40000000 + 3;
    }
    else {

        iMinuteCount = 2;
    }
    
	QFuture<void> future = QtConcurrent::run(DoCompute);
    //int ioff

    unsigned int iTotalMilliseconds = iMinuteCount * 60 * 1000; //�ܺ�����
    unsigned int indexCount = iTotalMilliseconds / 100;
    unsigned int index = 0;
	int progress = 0;
	while (!future.isFinished()){
        QThread::msleep(20);
        index += 20;
        QApplication::processEvents();
        if (index >= indexCount){
            progress++;
            index = 0;
        }
		if (m_showProgress)
		{
			if (progress < 98)
			{
				progressCb.setValue(progress);
			}
			else
			{
				progressCb.setValue(98);
			}	
		} 
	}
	int result = s_computeStatus;
	s_filter = nullptr;
	s_computing = false;
    if (!getSaveCloudPath().isEmpty() && result== Success) {
        progressCb.setValue(98);
        progressCb.setInfo(tr("Saving, please wait"));
        Sleep(500);
        QApplication::processEvents();
        emit CS::Model::ProjectModel::instance()->signalSaveCloudToLocal(getFirstSelectedEntity(), getSaveCloudPath());
        clearOutPutPointCloudPath();
    }
    progressCb.setValue(100);
    Sleep(500);
	if (m_showProgress)
	{
		progressCb.close();
		QApplication::processEvents();
	}
    
	return result;
}

QString BaseFilter::getErrorMessage(int errorCode) const
{
	switch (errorCode)
	{
	case ComputationError:
		return tr("Errors while computing");
		break;

	case InvalidInput:
		return tr("Internal error: invalid input");
		break;

	case ThreadAlreadyInUse:
		return tr("Internal error: thread already in use");
		break;

	case CancelledByUser:
		return tr("Process cancelled by user");

	case InvalidParameters:
		return tr("Invalid parameters");

	case NotEnoughMemory:
		return tr("Not enough memory");

	case Success:
		return {};

	default:
		return tr("Undefined error in filter %1: %2").arg(m_desc.filterName).arg(errorCode);
	}

	return {};
}

ccPointCloud* BaseFilter::getFirstSelectedEntityAsCCPointCloud() const
{
	ccHObject* entity = getFirstSelectedEntity();
	if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
	{
		return ccHObjectCaster::ToPointCloud(entity);
	}
	else
	{
		return nullptr;
	}
}

ccHObject* BaseFilter::getFirstSelectedEntity() const
{
	//do we have any selected entity?
	if (m_selectedEntities.empty())
		return nullptr;

	return m_selectedEntities.front();
}

void BaseFilter::getSelectedEntitiesThatAreCCPointCloud(ccHObject::Container& entities) const
{
	getSelectedEntitiesThatAre(CC_TYPES::POINT_CLOUD, entities);
}

void BaseFilter::getSelectedEntitiesThatAre(CC_CLASS_ENUM kind, ccHObject::Container& entities) const
{
	entities.reserve(m_selectedEntities.size());
	for (ccHObject* entity : m_selectedEntities)
	{
		if (entity && entity->isA(kind))
		{
			entities.push_back(entity);
		}
	}
	entities.shrink_to_fit();
}
