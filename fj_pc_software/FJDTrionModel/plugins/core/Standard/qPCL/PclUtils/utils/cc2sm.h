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
#ifndef Q_PCL_PLUGIN_CC2SM_H
#define Q_PCL_PLUGIN_CC2SM_H

//Local 
#include "PCLCloud.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Qt
#include <QStringList>

//system
#include <set>
#include <string>

class ccPointCloud;
struct PointXYZRGBITime;

//! CC to PCL cloud converter
class cc2smReader
{
public:
    explicit cc2smReader(ccPointCloud* ccCloud);
    explicit cc2smReader(ccPointCloud& ccCloudObject);

	//! Converts the ccPointCloud to a pcl::PointCloud2 cloud
	/** This is useful for saving a ccPointCloud into a PCD file.
		For pcl filters other methods are suggested (to get only the necessary bits of data)
	**/
	PCLCloud::Ptr getAsSM() const;
	PCLCloud::Ptr getAsSM(bool xyz, bool normals, bool rgbColors, const QStringList& scalarFields) const;

	//! Converts the ccPointCloud to a 'pcl::PointXYZ' cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr getRawXYZ() const;

    /**
    *@brief 点云转换
    toGlobal:当ccPointCloud存在大坐标偏移时，是否转换为全局坐标。默认为true
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRawXYZ(bool toGlobal = true) const;

    /**
    *@brief 把点云分块处理
    */
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> getRawXYZChunk(int nCheckCount, bool toGlobal = false);

    /**
    *@brief 强度点云转换
    toGlobal:当ccPointCloud存在大坐标偏移时，是否转换为全局坐标。默认为true
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getRawXYZI(bool toGlobal = true) const;

    /**
    *@brief 强度+gps时间点云转换
    toGlobal:当ccPointCloud存在大坐标偏移时，是否转换为全局坐标。默认为true
    */
    pcl::PointCloud<PointXYZRGBITime>::Ptr getRawXYZRGBIT(bool toGlobal = true) const;

	/**
	*@brief index点云转换,将索引暂存在index中，室内提取用carl.wang
	toGlobal:当ccPointCloud存在大坐标偏移时，是否转换为全局坐标。默认为true
	*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr getRawXYZIndex(bool toGlobal = true) const;

	//! Converts the ccPointCloud to a 'pcl::PointNormal' cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr getAsPointNormal() const;

	static std::string GetSimplifiedSFName(const QString& ccSfName);

    /*
    *@brief 计算点云平均间隔 janson
    */
    float computeCloudResolution();

protected:

	PCLCloud::Ptr getXYZ() const;
	PCLCloud::Ptr getNormals() const;
	PCLCloud::Ptr getColors() const;
	PCLCloud::Ptr getFloatScalarField(const QString& sfName) const;

	//! Associated cloud
	const ccPointCloud* m_ccCloud;//输入指针
};

#endif // Q_PCL_PLUGIN_CC2SM_H
