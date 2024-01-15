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
#include "cc2sm.h"

//Local
#include "my_point_types.h"
#include "PCLConv.h"
#include <QDebug>

//PCL
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//system
#include <assert.h>

using namespace pcl;

cc2smReader::cc2smReader(ccPointCloud* cccloud)
	: m_ccCloud(cccloud)
{
	assert(m_ccCloud);
}

cc2smReader::cc2smReader(ccPointCloud& ccCloudObject)
    : m_ccCloud(&ccCloudObject)
{
    assert(m_ccCloud);
}


std::string cc2smReader::GetSimplifiedSFName(const QString& ccSfName)
{
	QString simplified = ccSfName.simplified();
	simplified.replace(' ', '_');

	return simplified.toStdString();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cc2smReader::getRawXYZ(bool toGlobal) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<PointXYZ>::Ptr xyzCloud(new PointCloud<PointXYZ>);
	try
	{
		unsigned pointCount = m_ccCloud->size();
		xyzCloud->resize(pointCount);

        bool isShift = m_ccCloud->isShifted();

        const CCVector3* P = nullptr;
        CCVector3d realP;
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = m_ccCloud->getPoint(i);

            if (toGlobal && isShift)
            {
                realP = m_ccCloud->toGlobal3d(*P);

                xyzCloud->at(i).x = static_cast<float>(realP.x);
                xyzCloud->at(i).y = static_cast<float>(realP.y);
                xyzCloud->at(i).z = static_cast<float>(realP.z);
            }
            else
            {
                xyzCloud->at(i).x = static_cast<float>(P->x);
                xyzCloud->at(i).y = static_cast<float>(P->y);
                xyzCloud->at(i).z = static_cast<float>(P->z);
            }
			
		}
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return xyzCloud;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> cc2smReader::getRawXYZChunk(
    int nCheckCount,
    bool toGlobal /*= false*/)
{
    if (!m_ccCloud)
    {
        assert(false);
        return {};
    }

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> listChunk;
    listChunk.resize(nCheckCount);

    //[!].获取点云个数,计算每个块的数据大小
    unsigned pointCount = m_ccCloud->size();
    bool isShift = m_ccCloud->isShifted();
    CCVector3d realP;
    unsigned int chunkSize = pointCount / nCheckCount;

    unsigned int cloudindex = 0;
    for (int i = 0; i < nCheckCount; i++)
    {
        if (i == nCheckCount - 1){
            //[!]最后一次
            chunkSize = pointCount - cloudindex;
        }
        try
        {
            PointCloud<PointXYZ>::Ptr xyzCloud(new PointCloud<PointXYZ>);
            xyzCloud->resize(chunkSize);
            for (int j = 0; j < chunkSize; j++)
            {
                if (cloudindex < m_ccCloud->size()) {
                    const CCVector3* P = m_ccCloud->getPoint(cloudindex);
                    if (toGlobal && isShift)
                    {
                        realP = m_ccCloud->toGlobal3d(*P);
                        xyzCloud->at(j).x = static_cast<float>(realP.x);
                        xyzCloud->at(j).y = static_cast<float>(realP.y);
                        xyzCloud->at(j).z = static_cast<float>(realP.z);
                    }
                    else
                    {
                        xyzCloud->at(j).x = static_cast<float>(P->x);
                        xyzCloud->at(j).y = static_cast<float>(P->y);
                        xyzCloud->at(j).z = static_cast<float>(P->z);
                    }
                }
                cloudindex++;
            }
            listChunk[i] = xyzCloud;
        }
        catch (...)
        {
            //any error (memory, etc.)
            listChunk.clear();
            qDebug() << "any error (memory, etc.)";
        }
    }
    return listChunk;
}

PCLCloud::Ptr cc2smReader::getXYZ() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<PointXYZ>::Ptr xyzCloud = getRawXYZ();
	if (!xyzCloud)
	{
		return {};
	}

	PCLCloud::Ptr outputCloud;
	try
	{
		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(*xyzCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}
	
	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getNormals() const
{
	if (!m_ccCloud || !m_ccCloud->hasNormals())
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;
	try
	{
		PointCloud<OnlyNormals> normalsCloud;

		unsigned pointCount = m_ccCloud->size();
		normalsCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3& N = m_ccCloud->getPointNormal(i);
			normalsCloud[i].normal_x = N.x;
			normalsCloud[i].normal_y = N.y;
			normalsCloud[i].normal_z = N.z;
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(normalsCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}
	
	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getColors() const
{
	if (!m_ccCloud || !m_ccCloud->hasColors())
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;

	try
	{
		PointCloud<OnlyRGB> rgbCloud;
		unsigned pointCount = m_ccCloud->size();
		rgbCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const ccColor::Rgb& rgb = m_ccCloud->getPointColor(i);
			rgbCloud[i].r = static_cast<uint8_t>(rgb.r);
			rgbCloud[i].g = static_cast<uint8_t>(rgb.g);
			rgbCloud[i].b = static_cast<uint8_t>(rgb.b);
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(rgbCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getFloatScalarField(const QString& fieldName) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	int sfIdx = m_ccCloud->getScalarFieldIndexByName(qPrintable(fieldName));
	if (sfIdx < 0)
	{
		return {};
	}

	CCCoreLib::ScalarField* sf = m_ccCloud->getScalarField(sfIdx);
	assert(sf);

	PCLCloud::Ptr outputCloud;

	try
	{
		PointCloud<FloatScalar> sfCloud;

		unsigned pointCount = m_ccCloud->size();
		sfCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = sf->getValue(i);
			sfCloud[i].S5c4laR = static_cast<float>(scalar);
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(sfCloud, *outputCloud);

		//Now change the name of the scalar field -> we cannot have any space into the field name
		//NOTE this is a little trick to put any number of scalar fields in a message PointCloud2 object
		//We use a point type with a generic scalar field named scalar. we load the scalar field and
		//then we change the name to the needed one
		outputCloud->fields[0].name = GetSimplifiedSFName(fieldName);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getAsSM() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	//list of scalar fields
	QStringList scalarFields;
	for (unsigned i = 0; i < m_ccCloud->getNumberOfScalarFields(); ++i)
	{
		scalarFields << m_ccCloud->getScalarField(static_cast<int>(i))->getName();
	}

	return getAsSM(true, m_ccCloud->hasNormals(), m_ccCloud->hasColors(), scalarFields);
}

static PCLCloud::Ptr SetOrAdd(PCLCloud::Ptr firstCloud, PCLCloud::Ptr secondCloud)
{
	if (!secondCloud)
	{
		assert(false);
		return {};
	}

	if (firstCloud)
	{
		try
		{
			PCLCloud::Ptr tempCloud(new PCLCloud); //temporary cloud
			pcl::concatenateFields(*firstCloud, *secondCloud, *tempCloud);
			return tempCloud;
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("Not enough memory");
			return nullptr;
		}
	}
	else
	{
		return secondCloud;
	}
}

PCLCloud::Ptr cc2smReader::getAsSM(bool xyz, bool normals, bool rgbColors, const QStringList& scalarFields) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;

	unsigned pointCount = m_ccCloud->size();

	try
	{
		if (xyz)
		{
			PCLCloud::Ptr xyzCloud = getXYZ();
			if (!xyzCloud)
			{
				return {};
			}

			outputCloud = SetOrAdd(outputCloud, xyzCloud);
		}

		if (normals && m_ccCloud->hasNormals())
		{
			PCLCloud::Ptr normalsCloud = getNormals();
			if (!normalsCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, normalsCloud);
		}

		if (rgbColors && m_ccCloud->hasColors())
		{
			PCLCloud::Ptr rgbCloud = getColors();
			if (!rgbCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, rgbCloud);
		}

		for (const QString& sfName : scalarFields)
		{
			PCLCloud::Ptr sfCloud = getFloatScalarField(sfName);
			if (!sfCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, sfCloud);
		}
	}
	catch (...)
	{
		//any error (memory, etc.)
		outputCloud.reset();
	}

	return outputCloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr cc2smReader::getAsPointNormal() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<pcl::PointNormal>::Ptr pcl_cloud(new PointCloud<pcl::PointNormal>);

	unsigned pointCount = m_ccCloud->size();

	try
	{
		pcl_cloud->resize(pointCount);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* P = m_ccCloud->getPoint(i);
		pcl_cloud->at(i).x = static_cast<float>(P->x);
		pcl_cloud->at(i).y = static_cast<float>(P->y);
		pcl_cloud->at(i).z = static_cast<float>(P->z);
	}

	if (m_ccCloud->hasNormals())
	{
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* N = m_ccCloud->getNormal(i);
			pcl_cloud->at(i).normal_x = static_cast<float>(N->x);
			pcl_cloud->at(i).normal_y = static_cast<float>(N->y);
			pcl_cloud->at(i).normal_z = static_cast<float>(N->z);
		}
	}
	
	return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cc2smReader::getRawXYZI(bool toGlobal) const
{
    if (!m_ccCloud)
    {
        assert(false);
        return {};
    }

    PointCloud<PointXYZI>::Ptr xyzCloud(new PointCloud<PointXYZI>);
    try
    {
        unsigned pointCount = m_ccCloud->size();
        xyzCloud->resize(pointCount);

        int index = m_ccCloud->getScalarFieldIndexByName("intensity");
        CCCoreLib::ScalarField* sf = nullptr;
        if(index != -1)
            sf = m_ccCloud->getScalarField(index);

        bool isShift = m_ccCloud->isShifted();

        const CCVector3* P = nullptr;
        CCVector3d realP;
        for (unsigned i = 0; i < pointCount; ++i)
        {
            const CCVector3* P = m_ccCloud->getPoint(i);

            if (toGlobal && isShift)
            {
                realP = m_ccCloud->toGlobal3d(*P);

                xyzCloud->at(i).x = static_cast<float>(realP.x);
                xyzCloud->at(i).y = static_cast<float>(realP.y);
                xyzCloud->at(i).z = static_cast<float>(realP.z);
            }
            else
            {
                xyzCloud->at(i).x = static_cast<float>(P->x);
                xyzCloud->at(i).y = static_cast<float>(P->y);
                xyzCloud->at(i).z = static_cast<float>(P->z);
            }

            if (sf)
                xyzCloud->at(i).intensity = static_cast<float>(sf->getValue(i));
        }
    }
    catch (...)
    {
		qDebug() << "any error (memory, etc.)";
        return {};
    }

    return xyzCloud;
}

pcl::PointCloud<PointXYZRGBITime>::Ptr cc2smReader::getRawXYZRGBIT(bool toGlobal) const
{
    if (!m_ccCloud)
    {
        assert(false);
        return {};
    }

    PointCloud<PointXYZRGBITime>::Ptr xyzCloud(new PointCloud<PointXYZRGBITime>);
    try
    {
        unsigned pointCount = m_ccCloud->size();
        xyzCloud->resize(pointCount);

        int index = m_ccCloud->getScalarFieldIndexByName("Intensity");
        int index1 = m_ccCloud->getScalarFieldIndexByName("GpsTime");

        CCCoreLib::ScalarField* sf = nullptr;
        CCCoreLib::ScalarField* sf1 = nullptr;
        if (index != -1)
            sf = m_ccCloud->getScalarField(index);
        if (index1 != -1)
            sf1 = m_ccCloud->getScalarField(index1);

        bool isShift = m_ccCloud->isShifted();

        const CCVector3* P = nullptr;
        CCVector3d realP;
        for (unsigned i = 0; i < pointCount; ++i)
        {
            const CCVector3* P = m_ccCloud->getPoint(i);
            const ccColor::Rgb& rgb = m_ccCloud->getPointColor(i);
            
            if (toGlobal && isShift)
            {
                realP = m_ccCloud->toGlobal3d(*P);

                xyzCloud->at(i).x = static_cast<float>(realP.x);
                xyzCloud->at(i).y = static_cast<float>(realP.y);
                xyzCloud->at(i).z = static_cast<float>(realP.z);
            }
            else
            {
                xyzCloud->at(i).x = static_cast<float>(P->x);
                xyzCloud->at(i).y = static_cast<float>(P->y);
                xyzCloud->at(i).z = static_cast<float>(P->z);
            }

            xyzCloud->at(i).r = static_cast<uint8_t>(rgb.r);
            xyzCloud->at(i).g = static_cast<uint8_t>(rgb.g);
            xyzCloud->at(i).b = static_cast<uint8_t>(rgb.b);

            if (sf)
                xyzCloud->at(i).intensity = static_cast<float>(sf->getValue(i));
            if (sf1)
                xyzCloud->at(i).gpsTime = static_cast<float>(sf1->getValue(i));
        }
    }
    catch (...)
    {
        //any error (memory, etc.)
        return {};
    }

    return xyzCloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cc2smReader::getRawXYZIndex(bool toGlobal) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<PointXYZI>::Ptr xyzCloud(new PointCloud<PointXYZI>);
	try
	{
		unsigned pointCount = m_ccCloud->size();
		xyzCloud->resize(pointCount);

		int index = m_ccCloud->getScalarFieldIndexByName("intensity");
		CCCoreLib::ScalarField* sf = nullptr;
		if (index != -1)
			sf = m_ccCloud->getScalarField(index);

		bool isShift = m_ccCloud->isShifted();

		const CCVector3* P = nullptr;
		CCVector3d realP;
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = m_ccCloud->getPoint(i);

			if (toGlobal && isShift)
			{
				realP = m_ccCloud->toGlobal3d(*P);

				xyzCloud->at(i).x = static_cast<float>(realP.x);
				xyzCloud->at(i).y = static_cast<float>(realP.y);
				xyzCloud->at(i).z = static_cast<float>(realP.z);
			}
			else
			{
				xyzCloud->at(i).x = static_cast<float>(P->x);
				xyzCloud->at(i).y = static_cast<float>(P->y);
				xyzCloud->at(i).z = static_cast<float>(P->z);
			}

			//if (sf)
			xyzCloud->at(i).intensity = static_cast<float>(i);
		}
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return xyzCloud;
}

float cc2smReader::computeCloudResolution()
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input = getRawXYZ();

    double res = 0.0;
    if (input)
    { 
        int n_points = 0;
        int nres;
        std::vector<int> indices(2);
        std::vector<float> sqr_distances(2);
        pcl::search::KdTree<pcl::PointXYZ> tree;
        tree.setInputCloud(input);

        for (size_t i = 0; i < input->size(); i++)
        {
            if (!std::isfinite(input->at(i).x))
                continue;

            nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
            if (nres == 2)
            {
                res += sqrt(sqr_distances[1]);
                ++n_points;
            }

        }

        if (n_points != 0)
            res /= n_points;
    }

    return float(res);
}
