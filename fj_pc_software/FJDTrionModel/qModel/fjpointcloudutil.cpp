#include "fjpointcloudutil.h"
#include "ccHObject.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"
#include "ccHObjectCaster.h"
#include "ScalarField.h"
#include "ccLog.h"
#include "ccGenericPointCloud.h"
#include "ccMesh.h"
#include "ccPolyline.h"
#include "ccFacet.h"
#include <QColor>
#include "ccColorScale.h"
#include "ccColorTypes.h"
#include "cloudcompareutils\icore.h"
#include <QTextStream>
#include <QDomDocument>
#include <QDomElement>
#include <QColorDialog>
#include <QCoreApplication>
#include "FJStyleManager.h"
#include "mainwindow.h"
#include "ccDBRoot.h"

#include<QStandardPaths>
#include<QDir>
#include<qDebug>
#include <Windows.h>
#include <AclAPI.h>
#include"libs/cloudcompareutils/publicutils.h"
#include <codecvt>
#include <QTextCodec>
#include "cswidgets/framelessmessagebox.h"
CurrentCombinationMode FJPointCloudUtil::getUpdateCurrentCombinationMode(ccHObject * entity)
{
	if (entity && entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		if (!entity->sfShown())
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
			if (cloud)
			{
				if (entity->colorsShown() && cloud->hasColors())
				{
					entity->setCurrentCombinationMode(RGBCOLOR);
					return RGBCOLOR;
				}
				else
				{
					entity->setCurrentCombinationMode(UNIQUECOLOR);
					return UNIQUECOLOR;
				}
			}
		}
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
				QString currentSFName = QString(sf->getName());
				if (currentSFName.compare("Illuminance (PCV)", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(PCVMODE);
				}
				else if (currentSFName.compare("gpstime", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(GPSTIME);
				}
				else if (currentSFName.compare("coord. z", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(ALTITUDE);
				}
				else if (currentSFName.compare("userdata", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(USERDATA);
				}
				else if (currentSFName.compare("intensity", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(INTENSITY);
				}
				else if (currentSFName.compare("classification", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(CLASSIFICATION);
				}
				//else if (currentSFName.compare("Illuminance (PCV)", Qt::CaseInsensitive) == 0)
				//{
				//	entity->setCurrentCombinationMode(ALTITUDEANDUNIQUECOLOR);
				//}
				//else if (currentSFName.compare("Illuminance (PCV)", Qt::CaseInsensitive) == 0)
				//{
				//	entity->setCurrentCombinationMode(INTENSITYANDUNIQUECOLOR);
				//}
				else if (currentSFName.compare("coord. z + intensity", Qt::CaseInsensitive) == 0 || currentSFName.compare("intensity + coord. z", Qt::CaseInsensitive) == 0)
				{
					entity->setCurrentCombinationMode(ALTITUDEANDINTENSITY);
				}
				else
				{
					entity->setCurrentCombinationMode(OTHER);
				}
			}
			else
			{
				entity->setCurrentCombinationMode(OTHER);
			}
		}
		return entity->getCurrentCombinationMode();
	}
	return OTHER;
}


void FJPointCloudUtil::pointCloudCompositeRendering(ccHObject * entity, int sf1Idx, int sf2Idx)
{
	bool lockedVertices;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
	if (lockedVertices)
	{
		return ;
	}
	if (cloud)
	{
		if (!cloud || !cloud->hasScalarFields())
		{
			ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] No input cloud, or cloud has no SF?!");
			return;
		}

		unsigned sfCount = cloud->getNumberOfScalarFields();
		CCCoreLib::ScalarField* sf1 = nullptr;
		{
			if (sf1Idx >= static_cast<int>(sfCount))
			{
				ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
				return;
			}
			sf1 = cloud->getScalarField(sf1Idx);
		}

		CCCoreLib::ScalarField* sf2 = nullptr;
		{
			if (sf2Idx >= static_cast<int>(sfCount))
			{
				ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
				return;
			}
			sf2 = cloud->getScalarField(sf2Idx);
		}

		//output SF
		int sfIdx = -1;
		{
			//generate new sf name based on the operation
			QString sf1Name(sf1->getName());
			QString sf2Name;
			if (sf2)
			{
				sf2Name = sf2->getName();
			}
			QString sfName = QString("%1 + %2").arg(sf1Name, sf2Name);

			sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
			if (sfIdx >= 0)
			{
				if (sfIdx == sf1Idx || sfIdx == sf2Idx)
				{
					ccLog::Warning(QString("[ccScalarFieldArithmeticsDlg::apply] Resulting scalar field would have the same name as one of the operand (%1)! Rename it first...").arg(sfName));
					return ;
				}
				cloud->deleteScalarField(sfIdx);
			}

			sfIdx = cloud->addScalarField(qPrintable(sfName));
			if (sfIdx < 0)
			{
				ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Failed to create destination SF! (not enough memory?)");
				return ;
			}
		}
		CCCoreLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);
		if (sfDest)
		{
			unsigned valCount = sf1->currentSize();

			//resize destination SF
			if (!sfDest->resizeSafe(valCount))
			{
				ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Not enough memory!");
				cloud->deleteScalarField(sfIdx);
				sfDest = nullptr;
				return;
			}
			for (unsigned i = 0; i < valCount; ++i)
			{
				ScalarType val = CCCoreLib::NAN_VALUE;

				//we must handle 'invalid' values
				const ScalarType& val1 = sf1->getValue(i);
				if (ccScalarField::ValidValue(val1))
				{
					const ScalarType& val2 = sf2->getValue(i);
					if (ccScalarField::ValidValue(val2))
					{
						val = val1 + val2;
					}
				}
				sfDest->setValue(i, val);
			}

			sfDest->computeMinAndMax();
			cloud->setCurrentDisplayedScalarField(sfIdx);
			static_cast<ccScalarField*>(sfDest)->setMinDisplayed((static_cast<ccScalarField*>(sf1)->displayRange()).start() + (static_cast<ccScalarField*>(sf2)->displayRange()).start());
			static_cast<ccScalarField*>(sfDest)->setMaxDisplayed((static_cast<ccScalarField*>(sf1)->displayRange()).stop() + (static_cast<ccScalarField*>(sf2)->displayRange()).stop());
		}
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh_recursive();
	}
}

//aric.tang_2022.11.21_high as the status for color scale
void FJPointCloudUtil::pointCloudCompositeRenderingHigh(ccHObject * entity, int sf1Idx, int sf2Idx)
{
    bool lockedVertices;
    float persion = 0.01;
    float persion2 = 0.01;
    double statusstopval = 0.0;
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
    if (lockedVertices)
    {
        return;
    }

    if (cloud)
    {
        if (!cloud || !cloud->hasScalarFields())
        {
            ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] No input cloud, or cloud has no SF?!");
            return;
        }

        unsigned sfCount = cloud->getNumberOfScalarFields();
        CCCoreLib::ScalarField* sf1 = nullptr;
        {
            if (sf1Idx >= static_cast<int>(sfCount))
            {
                ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
                return;
            }
            sf1 = cloud->getScalarField(sf1Idx);
            //statusstopval = sf1->getValueByPercent(0.95);
        }

        CCCoreLib::ScalarField* sf2 = nullptr;
        {
            if (sf2Idx >= static_cast<int>(sfCount))
            {
                ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
                return;
            }
            sf2 = cloud->getScalarField(sf2Idx);
        }

        //output SF
        int sfIdx = -1;
        {
            //generate new sf name based on the operation
            QString sf1Name(sf1->getName());
            QString sf2Name;
            if (sf2)
            {
                sf2Name = sf2->getName();
                std::vector<double> percentlist{ 0.05 ,0.95 };
                std::vector<ScalarType>  resultlist;
                sf2->getValueByPercentList(percentlist, resultlist);
                persion = resultlist[0];
                persion2 = resultlist[1];

                //aric.tang_2022.12.6_for high color
//                std::map<float, int> paramMap;
//#pragma omp parallel for
//                for (int i = 0; i < sf2->size(); i++)
//                {
//                    paramMap[sf2->getValue(i)]++;
//                }
//                int num = sf2->size() * 0.1;
//                int sum = 0;
//                for (auto iter = paramMap.begin(); iter != paramMap.end(); iter++)
//                {
//                    sum += iter->second;
//                    if (sum >= num)
//                    {
//                        persion = iter->first;
//                        break;
//                    }
//                }
            }
            QString sfName = QString("%1 + %2").arg(sf1Name, sf2Name);

            sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
            if (sfIdx >= 0)
            {
                if (sfIdx == sf1Idx || sfIdx == sf2Idx)
                {
                    ccLog::Warning(QString("[ccScalarFieldArithmeticsDlg::apply] Resulting scalar field would have the same name as one of the operand (%1)! Rename it first...").arg(sfName));
                    return;
                }
                cloud->deleteScalarField(sfIdx);
            }

            sfIdx = cloud->addScalarField(qPrintable(sfName));
            if (sfIdx < 0)
            {
                ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Failed to create destination SF! (not enough memory?)");
                return;
            }
        }
        CCCoreLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);
        if (sfDest)
        {
            unsigned valCount = sf1->currentSize();

            //resize destination SF
            if (!sfDest->resizeSafe(valCount))
            {
                ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Not enough memory!");
                cloud->deleteScalarField(sfIdx);
                sfDest = nullptr;
                return;
            }
#pragma omp parallel for
            for (int i = 0; i < valCount; ++i)
            {
                ScalarType val = CCCoreLib::NAN_VALUE;

                //we must handle 'invalid' values
                //aric.tang_2022.12.5_change value
                const ScalarType& val1_pre = sf1->getValue(i);
                int val1_int = (int)(val1_pre * 1000.0);
                const ScalarType& val1 = (float)val1_int * 0.001;
                //aric.tang_2022.12.6_for high color
                const ScalarType& val2_pre = sf2->getValue(i);
                float val2_judge;
                if (val2_pre < persion)
                {
                    val2_judge = 0.0;
                }
                else if (val2_pre < persion2)
                {
                    val2_judge = (val2_pre - persion) / (persion2 - persion) * 255.0;
                }
                else
                {
                    val2_judge = 255.0;
                }
                const ScalarType& val2 = val2_judge * 0.000001;

                if (val1 < 0.0)
                {
                    val = val1 + (val2 + 0.000001) * (-1.0);
                }
                else
                {
                    val = val1 + val2 + 0.000001;
                }
                sfDest->setValue(i, val);
            }

            sfDest->computeMinAndMax();
            cloud->setCurrentDisplayedScalarField(sfIdx);
            //static_cast<ccScalarField*>(sfDest)->setMinDisplayed((static_cast<ccScalarField*>(sf1)->displayRange()).start() + (static_cast<ccScalarField*>(sf2)->displayRange()).start());
            //static_cast<ccScalarField*>(sfDest)->setMaxDisplayed((static_cast<ccScalarField*>(sf1)->displayRange()).stop() + (static_cast<ccScalarField*>(sf2)->displayRange()).stop());
            std::vector<QColor> test;
            test.push_back(Qt::blue);
            test.push_back(Qt::cyan);
            test.push_back(Qt::green);
            test.push_back(Qt::yellow);
            test.push_back(Qt::red);
            ccColorScalesManager::GetUniqueInstance()->addScale(FJPointCloudUtil::createHighScale(17, test));
            static_cast<ccScalarField*>(sfDest)->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::HIGH_COMBIN));
            static_cast<ccScalarField*>(sfDest)->setSaturationStop((static_cast<ccScalarField*>(sf1)->displayRange().stop() - static_cast<ccScalarField*>(sf1)->displayRange().start()) * 0.80 + static_cast<ccScalarField*>(sf1)->displayRange().start());
        }
        cloud->showSF(true);
        cloud->prepareDisplayForRefresh_recursive();
    }
}

int FJPointCloudUtil::getSFIndexFromEntity(ccHObject * entity, const QString & sfName)
{
	int index = -1;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (cloud)
	{
		int nsf = cloud->getNumberOfScalarFields();
		for (int i = 0; i < nsf; ++i)
		{
			QString curName = cloud->getScalarFieldName(i);
			if (curName.compare(sfName, Qt::CaseInsensitive) == 0)
			{
				index = i;
				break;
			}
		}
	}
	return index;
}


void FJPointCloudUtil::pointCloudCompositeRendering(ccHObject * entity, const QString & sf1Name, const QString &  sf2Name)
{
    //aric.tang_2022.11.21_judge if has coord.z
    if (sf1Name.compare("coord. z", Qt::CaseInsensitive) == 0 && sf2Name.compare("intensity", Qt::CaseInsensitive) == 0)
    {
        int index1 = getSFIndexFromEntity(entity, sf1Name);
        int index2 = getSFIndexFromEntity(entity, sf2Name);
        if (index1 >= 0 && index2 >= 0 && index1 != index2)
        {
            pointCloudCompositeRenderingHigh(entity, index1, index2);
        }
    }
    //else if (sf2Name.compare("coord. z", Qt::CaseInsensitive) == 0 && sf1Name.compare("intensity", Qt::CaseInsensitive) == 0)
    //{
    //    int index1 = getSFIndexFromEntity(entity, sf2Name);
    //    int index2 = getSFIndexFromEntity(entity, sf1Name);
    //    if (index1 >= 0 && index2 >= 0 && index1 != index2)
    //    {
    //        pointCloudCompositeRenderingHigh(entity, index1, index2);
    //    }
    //}
    else
    {
        int index1 = getSFIndexFromEntity(entity, sf1Name);
        int index2 = getSFIndexFromEntity(entity, sf2Name);
        if (index1 >= 0 && index2 >= 0 && index1 != index2)
        {
            pointCloudCompositeRendering(entity, index1, index2);
        }
    }
}

void FJPointCloudUtil::setEntityUniqueColor(ccHObject * ent, QColor colour)
{
	if (ent->isA(CC_TYPES::POINT_CLOUD) || ent->isA(CC_TYPES::MESH))
	{
		ccPointCloud* cloud = nullptr;
		if (ent->isA(CC_TYPES::POINT_CLOUD))
		{
			cloud = static_cast<ccPointCloud*>(ent);
		}
		else
		{
			ccMesh* mesh = static_cast<ccMesh*>(ent);
			ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
			if (!vertices
				|| !vertices->isA(CC_TYPES::POINT_CLOUD)
				|| (vertices->isLocked() && !mesh->isAncestorOf(vertices)))
			{
				ccLog::Warning(QObject::tr("[SetColor] Can't set color for mesh '%1' (vertices are not accessible)").arg(ent->getName()));
			}

			cloud = static_cast<ccPointCloud*>(vertices);
		}

		cloud->setColor(ccColor::FromQColora(colour));
		ent->setUniqueColor(colour);
		cloud->showColors(true);
		cloud->showSF(false); //just in case
		cloud->prepareDisplayForRefresh();

		if (ent != cloud)
		{
			ent->showColors(true);
		}
		else if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
		{
			cloud->getParent()->showColors(true);
			cloud->getParent()->showSF(false); //just in case
		}
	}
	else if (ent->isKindOf(CC_TYPES::PRIMITIVE))
	{
		ccGenericPrimitive* prim = ccHObjectCaster::ToPrimitive(ent);
		ccColor::Rgb col(static_cast<ColorCompType>(colour.red()),
			static_cast<ColorCompType>(colour.green()),
			static_cast<ColorCompType>(colour.blue()));
		prim->setColor(col);
		ent->showColors(true);
		ent->showSF(false); //just in case
		ent->prepareDisplayForRefresh();
		ent->setUniqueColor(colour);
	}
	else if (ent->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
		poly->setColor(ccColor::FromQColor(colour));
		ent->showColors(true);
		ent->showSF(false); //just in case
		ent->prepareDisplayForRefresh();
		ent->setUniqueColor(colour);
	}
	else if (ent->isA(CC_TYPES::FACET))
	{
		ccFacet* facet = ccHObjectCaster::ToFacet(ent);
		facet->setColor(ccColor::FromQColor(colour));
		ent->showColors(true);
		ent->showSF(false); //just in case
		ent->prepareDisplayForRefresh();
		ent->setUniqueColor(colour);
	}
	else
	{
		ccLog::Warning(QObject::tr("[SetColor] Can't change color of entity '%1'").arg(ent->getName()));
	}
}



QSharedPointer<ccColorScale> FJPointCloudUtil::createCombinScale(int scaleType, QColor unique_color)
{
	QString name = "Params Combin";
	ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType)));

	int h_, s_, v_, a_;
	int *h = &h_;
	int *s = &s_;
	int *v = &v_;
	int *a = &a_;
	unique_color.getHsv(h, s, v, a);

	ccColor::Rgb col1 = ccColor::Convert::hsv2rgb((float)h_, ((float)0 / 255.0), ((float)v_ / 255.0));
	QColor colorbar1;
	colorbar1.setRgb(col1.r, col1.g, col1.b);
	scale->insert(ccColorScaleElement(0.0, colorbar1), false);

	ccColor::Rgb col2 = ccColor::Convert::hsv2rgb((float)h_, (1.0 / 3.0), ((float)v_ / 255.0));
	QColor colorbar2;
	colorbar2.setRgb(col2.r, col2.g, col2.b);
	scale->insert(ccColorScaleElement(1.0 / 3.0, colorbar2), false);

	ccColor::Rgb col3 = ccColor::Convert::hsv2rgb((float)h_, (2.0 / 3.0), ((float)v_ / 255.0));
	QColor colorbar3;
	colorbar3.setRgb(col3.r, col3.g, col3.b);
	scale->insert(ccColorScaleElement(2.0 / 3.0, colorbar3), false);

	ccColor::Rgb col4 = ccColor::Convert::hsv2rgb((float)h_, (1.0), ((float)v_ / 255.0));
	QColor colorbar4;
	colorbar4.setRgb(col4.r, col4.g, col4.b);
	scale->insert(ccColorScaleElement(1.0, colorbar4), false);

	scale->update();
	scale->setLocked(true);

	return scale;
}

void FJPointCloudUtil::updateSaturationByRange(ccHObject * ent, double start, double stop)
{
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
    if (cloud)
    {

        unsigned sfCount = cloud->getNumberOfScalarFields();
        ccScalarField * sf1 = cloud->getCurrentDisplayedScalarField();
        if (sf1)
        {
            std::vector<double> percentlist;
            percentlist.push_back(start);
            percentlist.push_back(stop);
            std::vector<ScalarType>  resultlist;
            sf1->getValueByPercentList(percentlist, resultlist);
            if (resultlist.size() == 2)
            {
                sf1->setSaturationStart(resultlist[0]);
                sf1->setSaturationStop(resultlist[1]);
            }
        }
    }
}

void FJPointCloudUtil::Classification_combin(ccHObject * entity, const QString & sf1Name, const QString &  sf2Name)
{
	int index1 = FJPointCloudUtil::getSFIndexFromEntity(entity, sf1Name);
	int index2 = FJPointCloudUtil::getSFIndexFromEntity(entity, sf2Name);
	if (index1 >= 0 && index2 >= 0 && index1 != index2)
	{
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (lockedVertices)
		{
			return;
		}
		if (cloud)
		{
			if (!cloud || !cloud->hasScalarFields())
			{
				ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] No input cloud, or cloud has no SF?!");
				return;
			}

			unsigned sfCount = cloud->getNumberOfScalarFields();
			CCCoreLib::ScalarField* sf1 = nullptr;
			{
				if (index1 >= static_cast<int>(sfCount))
				{
					ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
					return;
				}
				sf1 = cloud->getScalarField(index1);
			}

			CCCoreLib::ScalarField* sf2 = nullptr;
			{
				if (index2 >= static_cast<int>(sfCount))
				{
					ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Invalid SF1 index!");
					return;
				}
				sf2 = cloud->getScalarField(index2);
			}

			//output SF
			int sfIdx = -1;
			{
				//generate new sf name based on the operation
				QString sf1Name(sf1->getName());
				QString sf2Name;
				if (sf2)
				{
					sf2Name = sf2->getName();
				}
				QString sfName = QString("%1 * %2").arg(sf1Name, sf2Name);

				sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
				if (sfIdx >= 0)
				{
					if (sfIdx == index1 || sfIdx == index2)
					{
						ccLog::Warning(QString("[ccScalarFieldArithmeticsDlg::apply] Resulting scalar field would have the same name as one of the operand (%1)! Rename it first...").arg(sfName));
						return;
					}
					cloud->deleteScalarField(sfIdx);
				}

				sfIdx = cloud->addScalarField(qPrintable(sfName));
				if (sfIdx < 0)
				{
					ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Failed to create destination SF! (not enough memory?)");
					return;
				}
			}
			CCCoreLib::ScalarField* sfDest = cloud->getScalarField(sfIdx);
			float min = sf2->getMin();
			float max = sf2->getMax();
			if (sfDest)
			{
				unsigned valCount = sf1->currentSize();

				//resize destination SF
				if (!sfDest->resizeSafe(valCount))
				{
					ccLog::Warning("[ccScalarFieldArithmeticsDlg::apply] Not enough memory!");
					cloud->deleteScalarField(sfIdx);
					sfDest = nullptr;
					return;
				}
				for (unsigned i = 0; i < valCount; ++i)
				{
					ScalarType val = CCCoreLib::NAN_VALUE;

					//we must handle 'invalid' values
					const ScalarType& val1 = sf1->getValue(i);
					if (ccScalarField::ValidValue(val1))
					{
						const ScalarType& val2 = sf2->getValue(i);
						if (ccScalarField::ValidValue(val2))
						{
                            val = val1 + ((val2 - min) / (max - min));
						}
					}
					sfDest->setValue(i, val);
				}

				sfDest->computeMinAndMax();
				cloud->setCurrentDisplayedScalarField(sfIdx);
				cloud->getCurrentDisplayedScalarField()->setColorScale(FJPointCloudUtil::getClassificationScale(entity));
				ccScalarField* ccsf2 = static_cast<ccScalarField*>(sf2);
				if (ccsf2)
				{
					entity->setClassficationCombineSingleSFRange(((ccsf2->displayRange()).start() -min) / (max - min), ((ccsf2->displayRange()).stop() -min) / (max - min));
				}
			}
			cloud->showSF(true);
			cloud->prepareDisplayForRefresh_recursive();
		}
	}

	//ccScalarField *sf = cloud->getCurrentDisplayedScalarField();
	//sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::CLASS_COMBIN));
}

void FJPointCloudUtil::setEntityColorScale(ccHObject * ent, ccColorScalesManager::DEFAULT_SCALES scaleType)
{
	if (ent)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (cloud)
		{
			ccScalarField *sf = cloud->getCurrentDisplayedScalarField();
			sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(scaleType));
		}
	}
}

void FJPointCloudUtil::exportAltitudeColorScale(ccHObject * ent)
{
	if (ent)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (cloud)
		{
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare("coord. z", Qt::CaseInsensitive) == 0)
				{
					return;
				}
			}
		}
		bool exportDims[3] = { false,false,true };
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent);
		if (pc == nullptr)
		{
			return;
		}

		if (!pc->exportCoordToSF(exportDims))
		{
			ccLog::Error(QObject::tr("The process failed!"));
			return; //true because we want the UI to be updated anyway
		}

		if (ent != pc)
		{
			ent->showSF(true); //for meshes
		}
		ent->prepareDisplayForRefresh_recursive();
	}
}

void FJPointCloudUtil::updateAltitudeColorScale(ccHObject * ent)
{
	if (ent)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
		if (cloud)
		{
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare("coord. z", Qt::CaseInsensitive) == 0)
				{
					QString orignalsfName = cloud->getCurrentDisplayedScalarField()->getName();
					bool isshowsf = cloud->sfShown();
					bool exportDims[3] = { false,false,true };
					if (!cloud->exportCoordToSF(exportDims))
					{
						ccLog::Error(QObject::tr("The process failed!"));
						return; //true because we want the UI to be updated anyway
					}
					if (!isshowsf)
					{
						cloud->showSF(false);
						cloud->showColors(true);
					}
					else
					{
						setCloudSF(ent, orignalsfName);
					}

					ent->prepareDisplayForRefresh_recursive();
					return;
				}
			}
		}
	}
}

void FJPointCloudUtil::setNotShowColorScale(ccHObject * ent)
{
	if (ent)
	{
		ent->showSF(false);
		ent->showColors(true);
	}
}

std::vector<ClassficationData> FJPointCloudUtil::getInitClassficationData()
{
	std::vector<ClassficationData> data;
	ClassficationData data1(0, QCoreApplication::translate("MainWindow", "Unclassified", nullptr), true, QColor(255, 255, 255));
	data.push_back(data1);
	ClassficationData data2(1, QCoreApplication::translate("MainWindow", "Ground", nullptr), true, QColor(139, 69, 19));
	data.push_back(data2);
	ClassficationData data3(2, QCoreApplication::translate("MainWindow", "Low vegetation", nullptr), true, QColor(34, 139, 34));
	data.push_back(data3);
	ClassficationData data4(3, QCoreApplication::translate("MainWindow", "Medium vegetation", nullptr), true, QColor(0, 128, 0));
	data.push_back(data4);
	ClassficationData data5(4, QCoreApplication::translate("MainWindow", "High vegetation", nullptr), true, QColor(0, 100, 0));
	data.push_back(data5);
	ClassficationData data6(5, QCoreApplication::translate("MainWindow", "Building", nullptr), true, QColor(178, 34, 34));
	data.push_back(data6);
	ClassficationData data7(6, QCoreApplication::translate("MainWindow", "Low point", nullptr), true, QColor(0, 191, 255));
	data.push_back(data7);
	ClassficationData data8(7, QCoreApplication::translate("MainWindow", "Model keypoint", nullptr), true, QColor(218, 165, 32));
	data.push_back(data8);
	ClassficationData data9(8, QCoreApplication::translate("MainWindow", "Water", nullptr), true, QColor(0, 0, 255));
	data.push_back(data9);
	ClassficationData data10(9, QCoreApplication::translate("MainWindow", "Bridge", nullptr), true, QColor(160, 82, 45));
	data.push_back(data10);
	ClassficationData data11(10, QCoreApplication::translate("MainWindow", "Inactive point", nullptr), true, QColor(105, 105, 105));
	data.push_back(data11);
	ClassficationData data12(11, QCoreApplication::translate("MainWindow", "Overlap", nullptr), true, QColor(95, 158, 160));
	data.push_back(data12);
	ClassficationData data13(12, QCoreApplication::translate("MainWindow", "Wall", nullptr), true, QColor(112, 128, 144));
	data.push_back(data13);
	ClassficationData data14(13, QCoreApplication::translate("MainWindow", "Floor", nullptr), true, QColor(210, 180, 140));
	data.push_back(data14);
	ClassficationData data15(14, QCoreApplication::translate("MainWindow", "Ceiling", nullptr), true, QColor(240, 135, 132));
	data.push_back(data15);
	ClassficationData data16(15, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(15), true, QColor(255, 254, 145));
	data.push_back(data16);
	ClassficationData data17(16, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(16), true, QColor(161, 251, 142));
	data.push_back(data17);
	ClassficationData data18(17, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(17), true, QColor(235, 51, 36));
	data.push_back(data18);
	ClassficationData data19(18, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(18), true, QColor(254, 253, 86));
	data.push_back(data19);
	ClassficationData data20(19, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(19), true, QColor(161, 250, 79));
	data.push_back(data20);
	ClassficationData data21(20, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(20), true, QColor(119, 67, 66));
	data.push_back(data21);
	ClassficationData data22(21, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(21), true, QColor(241, 155, 89));
	data.push_back(data22);
	ClassficationData data23(22, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(22), true, QColor(117, 249, 77));
	data.push_back(data23);
	ClassficationData data24(23, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(23), true, QColor(192, 192, 192));
	data.push_back(data24);
	ClassficationData data25(24, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(24), true, QColor(240, 134, 80));
	data.push_back(data25);
	ClassficationData data26(25, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(25), true, QColor(117, 250, 97));
	data.push_back(data26);
	ClassficationData data27(26, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(26), true, QColor(80, 127, 128));
	data.push_back(data27);
	ClassficationData data28(27, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(27), true, QColor(24, 62, 12));
	data.push_back(data28);
	ClassficationData data29(28, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(28), true, QColor(117, 250, 141));
	data.push_back(data29);
	ClassficationData data30(29, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(29), true, QColor(57, 16, 123));
	data.push_back(data30);
	ClassficationData data31(30, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(30), true, QColor(66, 20, 18));
	data.push_back(data31);
	ClassficationData data32(31, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(31), true, QColor(159, 252, 253));
	data.push_back(data32);
	ClassficationData data33(32, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(32), true, QColor(129, 127, 38));
	data.push_back(data33);
	ClassficationData data34(33, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(33), true, QColor(161, 27, 124));
	data.push_back(data34);
	ClassficationData data35(34, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(34), true, QColor(115, 251, 253));
	data.push_back(data35);
	ClassficationData data36(35, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(35), true, QColor(47, 123, 233));
	data.push_back(data36);
	ClassficationData data37(36, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(36), true, QColor(0, 18, 154));
	data.push_back(data37);
	ClassficationData data38(37, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(37), true, QColor(22, 65, 124));
	data.push_back(data38);
	ClassficationData data39(38, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(38), true, QColor(0, 12, 123));
	data.push_back(data39);
	ClassficationData data40(39, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(39), true, QColor(126, 132, 247));
	data.push_back(data40);
	ClassficationData data41(40, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(40), true, QColor(115, 43, 245));
	data.push_back(data41);
	ClassficationData data42(41, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(41), true, QColor(53, 128, 187));
	data.push_back(data42);
	ClassficationData data43(42, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(42), true, QColor(0, 2, 61));
	data.push_back(data43);
	ClassficationData data44(43, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(43), true, QColor(97, 33, 103));
	data.push_back(data44);
	ClassficationData data45(44, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(44), true, QColor(58, 8, 62));
	data.push_back(data45);
	ClassficationData data46(45, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(45), true, QColor(239, 136, 190));
	data.push_back(data46);
	ClassficationData data47(46, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(46), true, QColor(238, 138, 248));
	data.push_back(data47);
	ClassficationData data48(47, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(47), true, QColor(234, 63, 247));
	data.push_back(data48);
	ClassficationData data49(48, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(48), true, QColor(127, 130, 187));
	data.push_back(data49);
	ClassficationData data50(49, QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(49), true, QColor(117, 22, 63));
	data.push_back(data50);
	return data;
}

QColor FJPointCloudUtil::getColorByClassficationID(int id)
{
	std::vector<ClassficationData> data = FJPointCloudUtil::getClassficationDataFromDoc();
	for (auto curdata : data)
	{
		if (curdata.id == id)
		{
			return curdata.color;
		}
	}
	return QColor(255,255,255);
}

void FJPointCloudUtil::setNameByClassficationID(int id, QString name)
{
	std::vector<ClassficationData> data = FJPointCloudUtil::getClassficationDataFromDoc();
	for (auto & curdata : data)
	{
		if (curdata.id == id)
		{
			curdata.name = name;
		}
	}
	saveClassficationDataToDoc(data);
}

void FJPointCloudUtil::setColorByClassficationID(int id, QColor color)
{
	std::vector<ClassficationData> data = FJPointCloudUtil::getClassficationDataFromDoc();
	for (auto & curdata : data)
	{
		if (curdata.id == id)
		{
			curdata.color = color;
		}
	}
	saveClassficationDataToDoc(data);
}

std::vector<ClassficationData> FJPointCloudUtil::getClassficationDataFromDoc()
{
	std::vector<ClassficationData> data = FJPointCloudUtil::getInitClassficationData();
	QString fileName = CS::Core::ICore::getDefaultPath() + "/config/classficationsdata.xml";
	QFile m_file(fileName);
	if (!m_file.open(QIODevice::ReadOnly | QFile::Text))
	{
		return data;
	}

	QDomDocument doc;
	if (!doc.setContent(&m_file))
	{
		m_file.close();
		return data;
	}

	QDomElement root = doc.documentElement();           //读取根节点
	QDomNode node = root.firstChild();                  //读取第一个父节点

	while (!node.isNull())
	{
		QDomNodeList sonList = node.childNodes();       //读取子结点集合
		QString rootName = node.toElement().tagName();  //读取父节点名字
		if (rootName.compare("classfication") == 0) //读取子结点数据
		{
			ClassficationData newdata(0, "", true, QColor(255, 255, 255));
			int red = 255;
			int green = 255;
			int blue = 255;
			for (int sonNode = 0; sonNode < sonList.size(); sonNode++)
			{
				QDomElement sonElement = sonList.at(sonNode).toElement();       //获取子结点
				if (sonElement.toElement().tagName().compare("Id") == 0)          //与取出子结点进行对比
				{
					newdata.id = sonElement.text().toInt();
				}
				else if (sonElement.toElement().tagName().compare("Name") == 0)
				{
					newdata.name = sonElement.text();
				}
				else if (sonElement.toElement().tagName().compare("isShow") == 0)
				{
					newdata.isShow = true;
				}
				else if (sonElement.toElement().tagName().compare("R") == 0)
				{
					red = sonElement.text().toInt();
				}
				else if (sonElement.toElement().tagName().compare("G") == 0)
				{
					green = sonElement.text().toInt();
				}
				else if (sonElement.toElement().tagName().compare("B") == 0)
				{
					blue = sonElement.text().toInt();
				}
			}
			newdata.color = QColor(red, green, blue);
			bool isfind = false;
			for (auto & curExistData: data)
			{
				if (curExistData.id == newdata.id)
				{
					curExistData.color = newdata.color;
					curExistData.isShow = true;
					if (curExistData.id > 14)
					{
						QString enName = "Class_" + QString::number(curExistData.id);
						QString zhName = QString(u8"类别") + "_"+ QString::number(curExistData.id);
                        QString jaName = QString(u8"類別") + "_" + QString::number(curExistData.id);
                        QString twName = QString(u8"カテゴリー") + "_" + QString::number(curExistData.id);
						if (newdata.name != enName && newdata.name != zhName && newdata.name != jaName && newdata.name != twName)
						{
							curExistData.name = newdata.name;
						}
					}
					isfind = true;
					break;
				}
			}
			if (!isfind)
			{
				if (newdata.id > 49)
				{
					QString enName = "Class_" + QString::number(newdata.id);
					QString zhName = QString(u8"类别") + "_" + QString::number(newdata.id);
                    QString jaName = QString(u8"類別") + "_" + QString::number(newdata.id);
                    QString twName = QString(u8"カテゴリー") + "_" + QString::number(newdata.id);
					if (newdata.name == enName || newdata.name == zhName || newdata.name == jaName || newdata.name == twName)
					{
						newdata.name = QCoreApplication::translate("MainWindow", "Class", nullptr) + "_" + QString::number(newdata.id);
					}
				}
				data.push_back(newdata);
			}
		}
		node = node.nextSibling();                      //读取下一父节点
	}
	//nameData[1] = QCoreApplication::translate("MainWindow", "Points sampling on mesh", nullptr);
	m_file.close();

	return data;
}

void FJPointCloudUtil::saveClassficationDataToDoc(const std::vector<ClassficationData> & data)
{

	QString fileName = CS::Core::ICore::getDefaultPath() + "/config/classficationsdata.xml";
	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		return ;
	}

	QTextStream out(&file);
	QDomDocument doc;
	QDomProcessingInstruction instruction;
	instruction = doc.createProcessingInstruction("xml", "version = \'1.0\' encoding=\'UTF-8\'");
	doc.appendChild(instruction);
	QDomElement root = doc.createElement("data");   //创建顶节点
	doc.appendChild(root);

	for (const auto  & curdata : data)
	{
		QDomElement itemRootElement = doc.createElement("classfication"); //创建父节点
		{
			QDomElement node1 = doc.createElement("Id");           //创建子结点
			QDomText domText1 = doc.createTextNode("Id");
			domText1.setData(QString::number(curdata.id));  //设置子结点数据
			node1.appendChild(domText1);                           //将子节点数据绑定
			itemRootElement.appendChild(node1);                    //将子节点绑定到父节点

			QDomElement node2 = doc.createElement("Name");
			QDomText domText2 = doc.createTextNode("Name");
			domText2.setData(curdata.name);
			node2.appendChild(domText2);
			itemRootElement.appendChild(node2);

			QDomElement node3 = doc.createElement("isShow");
			QDomText domText3 = doc.createTextNode("isShow");
			domText3.setData(QString::number(curdata.isShow));
			node3.appendChild(domText3);
			itemRootElement.appendChild(node3);

			QDomElement node4 = doc.createElement("R");
			QDomText domText4 = doc.createTextNode("R");
			domText4.setData(QString::number(curdata.color.red()));
			node4.appendChild(domText4);
			itemRootElement.appendChild(node4);

			QDomElement node5 = doc.createElement("G");
			QDomText domText5 = doc.createTextNode("G");
			domText5.setData(QString::number(curdata.color.green()));
			node5.appendChild(domText5);
			itemRootElement.appendChild(node5);

			QDomElement node6 = doc.createElement("B");
			QDomText domText6 = doc.createTextNode("B");
			domText6.setData(QString::number(curdata.color.blue()));
			node6.appendChild(domText6);
			itemRootElement.appendChild(node6);
		}
		root.appendChild(itemRootElement);
	}

	doc.save(out, 4);       //each line space of file is 4
	file.close();
}

QSharedPointer<ccColorScale> FJPointCloudUtil::createClassificationScale(int scaleType, std::map<int, QColor> class_map)
{
	QString name = "Class Combin";
	ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType)));
	//int num = 0;
	scale->setLocked(false);
	if (class_map.size() < 2)
	{
		scale->insert(ccColorScaleElement((0.0), class_map.begin()->second), false);
		scale->insert(ccColorScaleElement((0.5), class_map.begin()->second), false);
		scale->insert(ccColorScaleElement((1.0), class_map.begin()->second), false);
	}
	else
	{
		if (scaleType == 16)
		{
            for (auto it = class_map.begin(); it != class_map.end(); it++)
            {
                scale->insert(ccColorScaleElement((float)(((it->first) - (class_map.begin()->first)) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), it->second), false);
                //num++;
            }
		}
        else
        {
            scale->insert(ccColorScaleElement(((float)((class_map.begin()->first) - (class_map.begin()->first)) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), class_map.begin()->second), false);
            scale->insert(ccColorScaleElement((((float)((class_map.begin()->first) - (class_map.begin()->first)) + 0.5) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), class_map.begin()->second), false);
            for (auto it = ++class_map.begin(); it != --class_map.end(); it++)
            {
                scale->insert(ccColorScaleElement(((float)(((it->first) - (class_map.begin()->first)) - 0.4) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), it->second), false);
                scale->insert(ccColorScaleElement(((float)(((it->first) - (class_map.begin()->first)) + 0.5) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), it->second), false);
                //num++;
            }
            scale->insert(ccColorScaleElement((((float)((class_map.rbegin()->first) - (class_map.begin()->first)) - 0.4) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), class_map.rbegin()->second), false);
            scale->insert(ccColorScaleElement(((float)((class_map.rbegin()->first) - (class_map.begin()->first)) / (float)((class_map.rbegin()->first) - (class_map.begin()->first))), class_map.rbegin()->second), false);
        }
		
	}
	scale->update();


	return scale;
}
//aric.tang_2022.11.21_create hight scale
QSharedPointer<ccColorScale> FJPointCloudUtil::createHighScale(int scaleType, std::vector<QColor> high_map)
{
    QString name = "High Combin";
    ccColorScale::Shared scale(new ccColorScale(name, QString::number(scaleType)));
    //int num = 0;
    scale->insert(ccColorScaleElement((0.0), high_map[0]), false);
    scale->insert(ccColorScaleElement((0.2), high_map[1]), false);
    scale->insert(ccColorScaleElement((0.4), high_map[2]), false);
    scale->insert(ccColorScaleElement((0.6), high_map[3]), false);
    scale->insert(ccColorScaleElement((1.0), high_map[4]), false);
    //scale->insert(ccColorScaleElement((0.1), Qt::darkBlue), false);
    //scale->insert(ccColorScaleElement((0.3), Qt::darkGreen), false);
    //scale->insert(ccColorScaleElement((0.6), Qt::darkYellow), false);
    //scale->insert(ccColorScaleElement((1.0), Qt::darkRed), false);

    scale->update();
    //scale->setLocked(true);

    return scale;
}

void FJPointCloudUtil::updateAllPointCloudClassficationDataFromDoc()
{
	ccHObject::Container rects;
	if (MainWindow::TheInstance()->db())
	{
		if (MainWindow::TheInstance()->db()->getRootEntity())
		{
			MainWindow::TheInstance()->db()->getRootEntity()->filterChildren(rects, true, CC_TYPES::POINT_CLOUD);
			for (auto curobj : rects)
			{
				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(curobj);
				if (cloud && cloud->sfShown())
				{
					auto sf = cloud->getCurrentDisplayedScalarField();
					if (sf)
					{
						QString currentSFName = sf->getName();
						if (currentSFName.compare("classification", Qt::CaseInsensitive) == 0)
						{
							FJPointCloudUtil::updateClassficationDataFromDoc(curobj);
						}
						if (currentSFName.compare("classification * gpstime", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * coord. z", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * intensity", Qt::CaseInsensitive) == 0)
						{
							FJPointCloudUtil::updateClassficationDataFromDoc(curobj,false);
						}
					}
				}
			}
		}
	}
}

void FJPointCloudUtil::updateClassficationDataFromDoc(ccHObject * entity, bool isChangesf)
{
	std::vector<ClassficationData> savedData = FJPointCloudUtil::getClassficationDataFromDoc();
	std::vector<ClassficationData> data;
	int minValue = 99999;
	int maxValue = 0;
	if (entity)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf)
					{
						QString sfName = sf->getName();
						if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
						{
							if (isChangesf)
							{
								cloud->setCurrentCombinationMode(CLASSIFICATION);
								cloud->setCurrentDisplayedScalarField(i);
							}
							std::set<int> idset;
							for (size_t i = 0; i < sf->size(); i++) {
								idset.insert(sf->getValue(i));
							}
							minValue = sf->saturationRange().start();
							maxValue = sf->saturationRange().stop();
							for (auto curid : idset)
							{
								bool isexist = false;
								for (auto cursavedata : savedData)
								{
									if (cursavedata.id == curid)
									{
										data.push_back(cursavedata);
										isexist = true;
										break;
									}
								}
								if (!isexist)
								{
									QString newname = QString(("classification")) + QString::number(curid);
									ClassficationData newdata(curid, newname, true, QColor(255, 255, 255));
									data.push_back(newdata);
								}
							}
						}
					}
				}
			}
		}
	}
	std::map<int, QColor> test;
    //aric.tang_2023.2.9_single color gradient
	//for (auto curdata : savedData)
	//{
	//	if (curdata.id >= minValue && curdata.id <= maxValue)
	//	{
	//		test.insert(std::pair<int, QColor>(curdata.id, curdata.color));
	//	}
	//}
	//ccColorScalesManager::GetUniqueInstance()->addScale(createClassificationScale(16, test));
	if (entity)
	{
		ccPointCloud *cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			ccScalarField *sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
                QString currentSFName = sf->getName();
                for (auto curdata : savedData)
                {
                    if (currentSFName.compare("classification", Qt::CaseInsensitive) == 0)
                    {
                        if (curdata.id >= minValue && curdata.id <= maxValue)
                        {
                            test.insert(std::pair<int, QColor>(curdata.id, curdata.color));
                        }
                    }
                    if (currentSFName.compare("classification * gpstime", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * coord. z", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * intensity", Qt::CaseInsensitive) == 0)
                    {
                        if (curdata.id >= minValue && curdata.id <= (maxValue + 1))
                        {
                            test.insert(std::pair<int, QColor>(curdata.id, curdata.color));
                        }
                    }
                }

                if (currentSFName.compare("classification", Qt::CaseInsensitive) == 0)
                {
                    sf->setColorScale(createClassificationScale(20, test));
                }
                if (currentSFName.compare("classification * gpstime", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * coord. z", Qt::CaseInsensitive) == 0 || currentSFName.compare("classification * intensity", Qt::CaseInsensitive) == 0)
                {
                    sf->setColorScale(createClassificationScale(16, test));
                }


				//sf->setColorScale(createClassificationScale(20, test));
				sf->setColorRampSteps(255);
				cloud->showSF(true);
			}
		}
	}
}

QSharedPointer<ccColorScale> FJPointCloudUtil::getClassificationScale(ccHObject * entity)
{
	std::vector<ClassficationData> savedData = FJPointCloudUtil::getClassficationDataFromDoc();
	std::vector<ClassficationData> data;
	int minValue = 99999;
	int maxValue = 0;
	if (entity)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud)
		{
			int nsf = cloud->getNumberOfScalarFields();
			for (int i = 0; i < nsf; ++i)
			{
				QString sfName = cloud->getScalarFieldName(i);
				if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
				{
					ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
					if (sf)
					{
						QString sfName = sf->getName();
						if (sfName.compare("classification", Qt::CaseInsensitive) == 0)
						{
							std::set<int> idset;
							for (size_t i = 0; i < sf->size(); i++) {
								idset.insert(sf->getValue(i));
							}
							minValue = sf->saturationRange().start();
							maxValue = sf->saturationRange().stop();
							for (auto curid : idset)
							{
								bool isexist = false;
								for (auto cursavedata : savedData)
								{
									if (cursavedata.id == curid)
									{
										data.push_back(cursavedata);
										isexist = true;
										break;
									}
								}
								if (!isexist)
								{
									QString newname = QString(("classification")) + QString::number(curid);
									ClassficationData newdata(curid, newname, true, QColor(255, 255, 255));
									data.push_back(newdata);
								}
							}
						}
					}
				}
			}
		}
	}
	std::map<int, QColor> test;
    //aric.tang_2023.2.9_single color gradient
	for (auto curdata : savedData)
	{
		if (curdata.id >= minValue && curdata.id <= (maxValue + 1))
		{
			test.insert(std::pair<int, QColor>(curdata.id, curdata.color));
		}
	}
	return createClassificationScale(16, test);
}

bool FJPointCloudUtil::isCloudPointSupportScaleSetting(ccHObject * ent)
{
	CurrentCombinationMode mode = ent->getCurrentCombinationMode();
    if (mode == CLASSIFICATION || mode == ALTITUDE || mode == GPSTIME || mode == USERDATA || mode == INTENSITY || mode == ALTITUDEANDINTENSITY)
	{
		return true;
	}
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
    if (cloud && ent->colorsShown() && cloud->getScalarFieldIndexByName("HasColor") >= 0)
    {
        return true;
    }
	return false;
}


bool FJPointCloudUtil::isCloudHasSF(ccHObject * ent, QString name)
{
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
	if (cloud)
	{
		int nsf = cloud->getNumberOfScalarFields();
		for (int i = 0; i < nsf; ++i)
		{
			QString sfName = cloud->getScalarFieldName(i);
			if (sfName.compare(name, Qt::CaseInsensitive) == 0)
			{
				return true;
			}
		}
	}
	return false;
}


bool FJPointCloudUtil::isCloudIsCurrentSF(ccHObject * ent, QString name)
{
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
	if (cloud)
	{
		if (cloud->getCurrentDisplayedScalarField())
		{
			QString sfName = cloud ? QString::fromLocal8Bit(cloud->getCurrentDisplayedScalarField()->getName()) : "";
			if (sfName.contains("intensity", Qt::CaseInsensitive))
			{
				return true;
			}
		}
	}
	return false;
}


void FJPointCloudUtil::setCloudSF(ccHObject * ent, QString name)
{
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
	if (cloud)
	{
		int nsf = cloud->getNumberOfScalarFields();
		for (int i = 0; i < nsf; ++i)
		{
			QString sfName = cloud->getScalarFieldName(i);
			if (sfName.compare(name, Qt::CaseInsensitive) == 0)
			{
				cloud->setCurrentDisplayedScalarField(i);
				cloud->showSF(true);
                cloud->scaleFieldColorChange();
			}
		}
	}
}

void FJPointCloudUtil::updateLabelName(ccHObject * ent)
{
	bool hasMetaData = false;
	int type = ent->getMetaData("measuretype").toInt(&hasMetaData);
	if (hasMetaData)
	{
		QString typeName = "";
		switch(type)
		{
		case 0:
			typeName = "point";
			break;
		case 1:
			typeName = "distance";
			break;
		case 2:
			typeName = "angle";
			break;
		case 3:
			typeName = "height";
			break;
		case 4:
			typeName =  "area";
			break;
		case 5:
			typeName = "volume";
			break;
		default:
			break;
		}
		if (ent->getParent())
		{
			int childNum = ent->getParent()->getChildrenNumber();
			bool hasSameLabel = false;
			int maxSameIndex = 0;
			for (int i = 0;i < childNum;i++)
			{
				bool hasChildMetaData = false;
				int childtype = ent->getParent()->getChild(i)->getMetaData("measuretype").toInt(&hasChildMetaData);
				if (ent->getParent()->getChild(i) != ent && hasChildMetaData && childtype == type)
				{
					hasSameLabel = true;
					maxSameIndex = std::max(maxSameIndex, ent->getParent()->getChild(i)->getMetaData("measureIndex").toInt(&hasChildMetaData));
				}
			}
			if (hasSameLabel)
			{
				ent->setName(typeName + "_" + QString::number(maxSameIndex+1));
				ent->setMetaData("measureIndex", maxSameIndex+1);
			}
			else
			{
				ent->setName(typeName + "_" + QString::number(1));
				ent->setMetaData("measureIndex",1);
			}
		}
	}
}

void FJPointCloudUtil::setActionIcon(QToolButton * button, const QString& iconurlnormal, const QString& iconurlclicked, const QString& iconurldisabled)
{
	QIcon pIcon(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + iconurlnormal + ".png");
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + iconurlclicked + ".png"), QIcon::Active, QIcon::Off);
	pIcon.addPixmap(QPixmap(FJStyleManager::Instance()->Getcurrentthemepath() + "qssimage/" + iconurldisabled + ".png"), QIcon::Disabled, QIcon::Off);
	if (button)
	{
		button->setIcon(pIcon);
	}
}

QColor FJPointCloudUtil::getQColorDialogColor(const QString & title, const QColor & origoncolor, bool & isvalid)
{
    CS::Widgets::FramelessDialog outdlg(MainWindow::TheInstance());
    outdlg.setTitleBarLabelStyleSheet("width: 32px;\n"
        "font-size: 16px;\n"
        "padding-left: 5px;\n"
        "color: #F2F2F2;\n"
        "background-color:transparent;\n"
        "line-height: 24px;\n");
    outdlg.setWindowTitle(title);
    QColorDialog colordlg(origoncolor, MainWindow::TheInstance());
    QObject::connect(&colordlg, &QDialog::finished, &outdlg, &QDialog::close);
    colordlg.setOptions(QColorDialog::NoButtons);
    outdlg.SetContentHolder(&colordlg);
    if (!outdlg.exec())
    {
        isvalid = false;
        return origoncolor;
    }
    QColor newCol = colordlg.currentColor();
    if (newCol.isValid())
    {
        isvalid = true;
        return newCol;
    }
    else
    {
        isvalid = false;
        return origoncolor;
    }
}

QString FJPointCloudUtil::getInfoDataTempDataPath()
{
	QString tempPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
	QDir dir(tempPath);
	if (!dir.exists("infodatatemp"))
	{
        if (!dir.mkdir("infodatatemp"))
        {
            return QString();
        }
	}
    return QDir::toNativeSeparators(dir.filePath("infodatatemp"));

}

QString FJPointCloudUtil::getFjSlamTempDataPath()
{
    QString tempPath = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    QDir dir(tempPath);
	if (!dir.exists("fjslamtemp"))
    {
        if (!dir.mkdir("fjslamtemp"))
        {
            QString();
        }
    }
    return QDir::toNativeSeparators(dir.filePath("fjslamtemp"));
}

void FJPointCloudUtil::cleanInfoDataFiles()
{
	QString infodataputpath = FJPointCloudUtil::getInfoDataTempDataPath();
	QDir dirtemp(infodataputpath);
    QStringList names = dirtemp.entryList(QDir::Files);
    for (int i = 0; i < names.size(); i++)
    {
        QFile::remove(dirtemp.filePath(names.at(i)));
    }
}
void FJPointCloudUtil::cleanFJslamDataFiles()
{
    QString fjslamtemp = FJPointCloudUtil::getFjSlamTempDataPath();
    QDir dirtemp(fjslamtemp);
    QStringList names = dirtemp.entryList(QDir::Files);
    qDebug() << "topath" << fjslamtemp << "dir filesSize:" << names.size();

    for (int i = 0; i < names.size(); i++)
    {
        if (!QFile::remove(dirtemp.filePath(names.at(i)))) {
            qWarning() << "topath:" << fjslamtemp << ".delete file faild:" << dirtemp.filePath(names.at(i));
            continue;
        }
        qDebug() << "topath:" << fjslamtemp << ".dir fileName:" << dirtemp.filePath(names.at(i));

    }
}

static bool isFront(std::vector<int> firstdata, std::vector<int> seconddata)
{
    if (seconddata.size() == 0)
    {
        return true;
    }

    if (firstdata.size() == 0)
    {
        return false;
    }
    int num = seconddata.size() > firstdata.size() ? firstdata.size() : seconddata.size();
    for (int i = 0;i < num;i++)
    {
        if (seconddata[seconddata.size() - i -1] == firstdata[firstdata.size() - i - 1])
        {
            continue;
        }
        if (seconddata[seconddata.size() - i - 1] < firstdata[firstdata.size() - i - 1])
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return true;
}

static int getIndexFromVec(std::vector<std::vector<int>> data, int index)
{
    std::vector<int> curdata = data[index];
    int sortIndex = 0;
    for (int i = 0;i < data.size();i++)
    {
        if ( i != index)
        {
            if (isFront(data[i],curdata))
            {
                sortIndex++;
            }
        }
    }
    return sortIndex;
}

void FJPointCloudUtil::sortEntityByTree(std::vector<ccHObject *> & entitys)
{
    if (entitys.size() <= 1)
    {
        return;
    }
    std::vector<std::vector<int>> data;
    for (int j = 0; j < entitys.size(); j++)
    {
        data.push_back(getEntityIndexFromTree(entitys[j]));
    }
    std::multimap<int, ccHObject *> sorteddata;
    for (int i = 0;i < entitys.size();i++)
    {
        sorteddata.insert(std::pair(getIndexFromVec(data,i), entitys[i]));
    }
    entitys.clear();
    for (auto curdata : sorteddata)
    {
        entitys.push_back(curdata.second);
    }
}

std::vector<int> FJPointCloudUtil::getEntityIndexFromTree(ccHObject * entity)
{
    std::vector<int> result;
    ccHObject * root = MainWindow::TheInstance()->db()->getRootEntity();
    ccHObject * curentity = entity;
    if (root)
    {
        while (root->getChildIndex(curentity) == -1)
        {
            result.push_back(curentity->getParent()->getChildIndex(curentity));
            curentity = curentity->getParent();
            if (result.size()>20)
            {
                break;
            }
        }
        result.push_back(root->getChildIndex(curentity));
    }
    return result;
}

std::wstring FJPointCloudUtil::to_wstring(const std::string &str_in) {
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    return conv.from_bytes(str_in);
}

bool FJPointCloudUtil::file_exist(std::string file_name) {
    return std::ifstream(to_wstring(file_name), std::ios::in).is_open();
}

std::ifstream FJPointCloudUtil::get_ifstream(std::string file_name) {
    std::ifstream fin(to_wstring(file_name), std::ios::in);
    return fin;
}

std::ifstream FJPointCloudUtil::get_ifstream_binary(std::string file_name) {
    std::ifstream fin(to_wstring(file_name), std::ios::in | std::ios::binary);
    return fin;
}
static QTextCodec *uft8code = NULL;
QString FJPointCloudUtil::Utf8ToQStr(const char* s)
{
    if (NULL == uft8code)
    {
        uft8code = QTextCodec::codecForName("UTF-8");
        if (NULL == uft8code)
            return "";
    }

    return uft8code->toUnicode(s);
}

std::string FJPointCloudUtil::convertQStringPathToStringPath(QString path)
{
	QString publicPath = QDir::toNativeSeparators(path);
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    return converter.to_bytes(publicPath.toStdWString());
}

void FJPointCloudUtil::setLineEditStyle(QLineEdit * edit, bool isvalid)
{
    if (edit)
    {
        if (isvalid)
        {
            edit->setStyleSheet("border:1px solid #666666;");
        }
        else
        {
            edit->setStyleSheet("border:1px solid #ff0000;");
        }
    }
}

QMap<QString, QString> FJPointCloudUtil::copyFilesAndReturnPaths(const QString& source_directory, const QString& destination_directory, const QStringList& file_names) {
	QMap<QString, QString> local_file_paths;

	QDir sourceDir(source_directory);
	QDir destinationDir(destination_directory);

	for (const QString& filename : file_names) {
		QString specific_file = sourceDir.filePath(filename);

		// 检查特定文件是否存在
		QFileInfo fileInfo(specific_file);
		if (fileInfo.exists()) {
			// 构建目标文件路径
			QString destination_file = destinationDir.filePath(filename);

			// 如果目标文件已存在，先删除它
			if (QFile::exists(destination_file)) {
				QFile::remove(destination_file);
			}

			// 拷贝文件到目标文件夹
			if (QFile::copy(specific_file, destination_file)) {
				local_file_paths[filename] = destination_file; // 将新的文件路径添加到局部 QMap
			}
		}
	}

	return local_file_paths;
}

bool FJPointCloudUtil::getDirWriteable(QString path,bool isDisplayMessbox /*false*/)
{
    bool isWriteReadEnabel = true;
    if (path.isEmpty())
    {
        return false;
    }
    QFileInfo directory(path);
    if (!directory.exists())
	{
        isWriteReadEnabel = false;
	}
		if (!directory.isWritable())
		{
			isWriteReadEnabel = false;
		}
    QDir dir(path);
    QFile file(dir.filePath("FJPointCloudUtiltest.txt"));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        isWriteReadEnabel = false;
    }
    file.close();
    file.remove();
    if (isWriteReadEnabel)
    {
        return true;
    }
    else
    {
        if (isDisplayMessbox)
        {
            CS::Widgets::FramelessMessageBox::critical(MainWindow::TheInstance(),
                QCoreApplication::translate("MainWindowUI", "Error", nullptr),
                QCoreApplication::translate("MainWindowUI", "Selecting a path cannot save data,Please reselect the path.", nullptr));
        }
        return false;
    }
}
