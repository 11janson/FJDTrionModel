//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qPCV.h"
#include "ccPcvDlg.h"
#include "PCVCommand.h"

//CCCoreLib
#include <PCV.h>
#include <ScalarField.h>

//qCC_db
#include <ccGenericMesh.h>
#include <ccGenericPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QMainWindow>
#include <QProgressBar>

//persistent settings during a single session
static bool s_firstLaunch				= true;
static int s_raysSpinBoxValue			= 256;
static int s_resSpinBoxValue			= 1024;
static bool s_mode180CheckBoxState		= true;
static bool s_closedMeshCheckBoxState	= false;


qPCV::qPCV(QObject* parent/*=nullptr*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qPCV/info.json")
	, m_action(nullptr)
{
}

void qPCV::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		bool elligibleEntitiies = false;
		int cloudandmeshnum = 0;
		for (ccHObject* obj : selectedEntities)
		{
			if (obj && (obj->isKindOf(CC_TYPES::POINT_CLOUD) || obj->isKindOf(CC_TYPES::MESH)))
			{
				elligibleEntitiies = true;
				cloudandmeshnum++;
			}
		}
		m_action->setEnabled(elligibleEntitiies && cloudandmeshnum == selectedEntities.size());
	}
}

QList<QAction *> qPCV::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());

		connect(m_action, &QAction::triggered, this, &qPCV::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qPCV::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	ccHObject::Container candidates;
	bool hasMeshes = false;
	for (ccHObject* obj : selectedEntities)
	{
		if (!obj)
		{
			assert(false);
			continue;
		}
		
		if (obj->isA(CC_TYPES::POINT_CLOUD))
		{
			//we need a real point cloud
			candidates.push_back(obj);
		}
		else if (obj->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(obj);
			if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
			{
				//we need a mesh with a real point cloud
				candidates.push_back(obj);
				hasMeshes = true;
			}
		}
	}

	//ccPcvDlg dlg(m_app->getMainWindow());
	//dlg.setWindowTitle(QCoreApplication::translate("ccPcvDlg", "PCV processing", nullptr));
	//restore previous dialog state
	//if (!s_firstLaunch)
	//{
	//	dlg.raysSpinBox->setValue(s_raysSpinBoxValue);
	//	dlg.mode180CheckBox->setChecked(s_mode180CheckBoxState);
	//}


	//for using clouds normals as rays
	std::vector<ccGenericPointCloud*> cloudsWithNormals;
	ccHObject* root = m_app->dbRootObject();
	if (root)
	{
		ccHObject::Container clouds;
		root->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
		
		for (auto & pointCloud : clouds)
		{
			//we keep only clouds with normals
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud( pointCloud );
			
			if (cloud && cloud->hasNormals())
			{
				cloudsWithNormals.push_back(cloud);
				QString cloudTitle = QStringLiteral("%1 - %2 points").arg(cloud->getName()).arg(cloud->size());
				if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
				{
					cloudTitle.append(QStringLiteral(" (%1)").arg(cloud->getParent()->getName()));
				}

			}
		}
	}
	


	//save dialog state
	s_firstLaunch				= false;
	s_raysSpinBoxValue = 256;//dlg.raysSpinBox->value();
	s_mode180CheckBoxState = false;// dlg.mode180CheckBox->isChecked();


	unsigned rayCount = 256;// dlg.raysSpinBox->value();
	unsigned resolution = s_resSpinBoxValue;
	bool meshIsClosed = (hasMeshes ? s_closedMeshCheckBoxState : false);
	bool mode360 = true;// !dlg.mode180CheckBox->isChecked();

	//PCV type ShadeVis
	std::vector<CCVector3> rays;
	if (!PCV::GenerateRays(rayCount, rays, mode360))
	{
		m_app->dispToConsole("Failed to generate the set of rays", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (rays.empty())
	{
		assert(false);
		m_app->dispToConsole("No ray was generated?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return;
	}

	ccProgressDialog pcvProgressCb(true, m_app->getMainWindow());
	pcvProgressCb.setAutoClose(false);

	PCVCommand::Process(candidates, rays, meshIsClosed, resolution, &pcvProgressCb, m_app);

	pcvProgressCb.close();

	//currently selected entities parameters may have changed!
	m_app->updateUI();
	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}

void qPCV::registerCommands(ccCommandLineInterface* cmd)
{
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCVCommand));
}
