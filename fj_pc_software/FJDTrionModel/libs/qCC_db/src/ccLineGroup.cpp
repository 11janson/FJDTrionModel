//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccIncludeGL.h"

//Local
#include "ccLineGroup.h"
#include "ccBasicTypes.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"
#include "ccCone.h"

//Qt
#include <QSharedPointer>
#include <qdebug.h>

//System
#include <assert.h>
#include <string.h>

#define PI 3.14159265359
#define SIN(c)					(sin(c * PI / 180.0))			//弧度=角度乘以π后再除以180
#define COS(c)					(cos(c * PI / 180.0))
//'Delta' character
static const QChar MathSymbolDelta(0x0394);

static const QString CENTER_STRING = QObject::tr("Center");
static const char POINT_INDEX_0[]  = "pi0";
static const char POINT_INDEX_1[]  = "pi1";
static const char POINT_INDEX_2[]  = "pi2";
static const char ENTITY_INDEX_0[] = "ei0";
static const char ENTITY_INDEX_1[] = "ei1";
static const char ENTITY_INDEX_2[] = "ei2";


QString ccLineGroup::PickedPoint::itemTitle() const
{
	if (entityCenterPoint)
	{
		QString title = CENTER_STRING;
		if (entity())
			title += QString("@%1").arg(entity()->getUniqueID());
		return title;
	}
	else
	{
		return QString::number(index);
	}
}

QString ccLineGroup::PickedPoint::prefix(const char* pointTag) const
{
	if (entityCenterPoint)
	{
		return CENTER_STRING;
	}
	else if (_cloud)
	{
		return QString("Point #") + pointTag;
	}
	else if (_mesh)
	{
		return QString("Point@Tri#") + pointTag;
	}

	assert(false);
	return QString();
}

CCVector3 ccLineGroup::PickedPoint::getPointPosition() const
{
	CCVector3 P;

	if (_cloud)
	{
		if (entityCenterPoint)
		{
			return _cloud->getOwnBB().getCenter();
		}
		else
		{
			P = *_cloud->getPointPersistentPtr(index);
		}
	}
	else if (_mesh)
	{
		if (entityCenterPoint)
		{
			return _mesh->getOwnBB().getCenter();
		}
		else
		{
			_mesh->computePointPosition(index, uv, P);
		}
	}
	else
	{
		assert(false);
	}

	return P;
}

unsigned ccLineGroup::PickedPoint::getUniqueID() const
{
	if (_cloud)
		return _cloud->getUniqueID();
	if (_mesh)
		return _mesh->getUniqueID();

	assert(false);
	return 0;
}

ccGenericPointCloud* ccLineGroup::PickedPoint::cloudOrVertices() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh->getAssociatedCloud();
	
	assert(false);
	return nullptr;
}

ccHObject* ccLineGroup::PickedPoint::entity() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh;

	assert(false);
	return nullptr;
}

ccLineGroup::ccLineGroup(GenericIndexedCloudPersist* associatedCloud, unsigned uniqueID, QString name/*=QString()*/)
	: ccShiftedObject("LineGroup", uniqueID)
	, Polyline(associatedCloud)
	, m_showFullBody(true)
	, m_dispPointsLegend(false)
	, m_dispIn2D(true)
	, m_relMarkerScale(1.0f)
	, m_box(nullptr)
	, isShowBox(false)
{
	//m_screenPos[0] = m_screenPos[1] = 0.05f;

	//clear(false);

	//lockVisibility(false);
	//setEnabled(true);


	ccGenericPointCloud* cloud = dynamic_cast<ccGenericPointCloud*>(associatedCloud);
	if (cloud)
	{
		//no need to call ccPolyline::the copyGlobalShiftAndScalemethod
		//as it will try to set the Global Shift & Scale info on the associated cloud!
		ccShiftedObject::copyGlobalShiftAndScale(*cloud);
	}
}

QString ccLineGroup::GetSFValueAsString(const LabelInfo1& info, int precision)
{
	if (info.hasSF)
	{
		if (!ccScalarField::ValidValue(info.sfValue))
		{
			return "NaN";
		}
		else
		{
			QString sfVal = QString::number(info.sfValue, 'f', precision);
			if (info.sfValueIsShifted)
			{
				sfVal = QString::number(info.sfShiftedValue, 'f', precision) + QString(" (shifted: %1)").arg(sfVal);
			}
			return sfVal;
		}
	}
	else
	{
		return QString();
	}
}

QString ccLineGroup::getTitle(int precision, CC_DRAW_CONTEXT& context) const
{
	QString title;
	size_t count = m_pickedPoints.size();
	
	if (count == 1)
	{
		title = m_name;
		title.replace(POINT_INDEX_0, m_pickedPoints[0].itemTitle());

		//if available, we display the point SF value
		LabelInfo1 info;
		getLabelInfo1(info);
		if (info.hasSF)
		{
			QString sfVal = GetSFValueAsString(info, precision);
			title = QString("%1 = %2").arg(info.sfName, sfVal);
		}
	}
	else if (count == 2)
	{
		LabelInfo2 info;
		getLabelInfo2(info);
		//display distance by default
		double dist = info.diff.normd();
		if (context.isshowUnitlabel)
		{
			QString unit = m_diameterlist[context.lengthunit];
			title = QString("Distance: %1").arg(dist * m_lengthunitshift[context.lengthunit], 0, 'f', precision) + unit;
		}
		else
		{
			title = QString("Distance: %1").arg(dist * m_lengthunitshift[context.lengthunit], 0, 'f', precision);
		}

	}
	else if (count == 3)
	{
		LabelInfo3 info;
		getLabelInfo3(info);
		//display area by default
		title = QString("Area: %1").arg(info.area, 0, 'f', precision);
		if (context.isshowUnitlabel)
		{
			QString unit = m_arealist[context.areaunit];
			title = QString("Area: %1").arg(info.area * m_areaunitshift[context.areaunit], 0, 'f', precision) + unit;
		}
		else
		{
			title = QString("Area: %1").arg(info.area * m_areaunitshift[context.areaunit], 0, 'f', precision);
		}
	}
	else
	{
		//大于三个点
        //计算多边形面积
		float area = 0.0;
		int num = m_pickedPoints.size();

		if (num > 3)
		{
			CCVector3f va, vb, res;
			for (int i = 0; i < num; i++)
			{
				int j = (i + 1) % num;
				va = m_pickedPoints[i].getPointPosition();
				vb = m_pickedPoints[j].getPointPosition();
				res += va.cross(vb);
			}

			area = 0.5 * res.norm();
		}
		if (context.isshowUnitlabel)
		{
			QString unit = m_arealist[context.areaunit];
			title = QString("area: %1\t").arg(area * m_areaunitshift[context.areaunit], 0, 'f', precision) + unit;
		}
		else
		{
			title = QString("area: %1\t").arg(area * m_areaunitshift[context.areaunit], 0, 'f', precision);
		}
		
	}


	return title;
}

QString ccLineGroup::getName() const
{
	QString processedName = m_name;

	size_t count = m_pickedPoints.size();
	if (count > 0)
	{
		processedName.replace(POINT_INDEX_0, m_pickedPoints[0].itemTitle());
		if (count > 1)
		{
			processedName.replace(ENTITY_INDEX_0, QString::number(m_pickedPoints[0].getUniqueID()));

			processedName.replace(POINT_INDEX_1, m_pickedPoints[1].itemTitle());
			processedName.replace(ENTITY_INDEX_1, QString::number(m_pickedPoints[1].getUniqueID()));

			if (count > 2)
			{
				processedName.replace(POINT_INDEX_2, m_pickedPoints[2].itemTitle());
				processedName.replace(ENTITY_INDEX_2, QString::number(m_pickedPoints[2].getUniqueID()));
			}
		}
	}

	return processedName;
}

void ccLineGroup::setPosition(float x, float y)
{
	m_screenPos[0] = x;
	m_screenPos[1] = y;
}

bool ccLineGroup::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	assert(screenHeight > 0 && screenWidth > 0);

	m_screenPos[0] += static_cast<float>(dx) / screenWidth;
	m_screenPos[1] += static_cast<float>(dy) / screenHeight;

	return true;
}

void ccLineGroup::clear(bool ignoreDependencies)
{
	if (ignoreDependencies)
	{
		m_pickedPoints.resize(0);
	}
	else
	{
		//remove all dependencies first!
		while (!m_pickedPoints.empty())
		{
			PickedPoint& pp = m_pickedPoints.back();
			if (pp.entity())
				pp.entity()->removeDependencyWith(this);
			m_pickedPoints.pop_back();
		}
	}

	m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
	m_labelROI = QRect(0, 0, 0, 0);
	setVisible(false);
	setName("Label");
}

void ccLineGroup::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.

	//check that associated clouds are not about to be deleted!
	size_t pointsToRemove = 0;
	{
		for (size_t i = 0; i < m_pickedPoints.size(); ++i)
			if (m_pickedPoints[i].entity() == obj)
				++pointsToRemove;
	}

	if (pointsToRemove == 0)
		return;

	if (pointsToRemove == m_pickedPoints.size())
	{
		clear(true); //don't call clear as we don't want/need to update input object's dependencies!
	}
	else
	{
		//remove only the necessary points
		size_t j = 0;
		for (size_t i = 0; i < m_pickedPoints.size(); ++i)
		{
			if (m_pickedPoints[i].entity() != obj)
			{
				if (i != j)
					std::swap(m_pickedPoints[i], m_pickedPoints[j]);
				j++;
			}
		}
		assert(j != 0);
		m_pickedPoints.resize(j);
	}

	updateName();
}

void ccLineGroup::updateName()
{
	switch (m_pickedPoints.size())
	{
	case 0:
	{
		setName("Label");
	}
	break;

	case 1:
	{
		setName(m_pickedPoints[0].prefix(POINT_INDEX_0));
	}
	break;
	
	case 2:
	{
		if (m_pickedPoints[0].entity() == m_pickedPoints[1].entity())
		{
			setName(	QString("Vector ") + m_pickedPoints[0].prefix(POINT_INDEX_0)
					+	QString(" - ")     + m_pickedPoints[1].prefix(POINT_INDEX_1) );
		}
		else
		{
			setName(	QString("Vector ") + m_pickedPoints[0].prefix(POINT_INDEX_0) + QString("@") + ENTITY_INDEX_0
					+	QString(" - ")     + m_pickedPoints[1].prefix(POINT_INDEX_1) + QString("@") + ENTITY_INDEX_1 );
		}
	}
	break;
	
	case 3:
	{
		if (	m_pickedPoints[0].entity() == m_pickedPoints[2].entity() && m_pickedPoints[1].entity() == m_pickedPoints[2].entity() )
		{
			setName(	QString("Triplet ") + m_pickedPoints[0].prefix(POINT_INDEX_0)
					+	QString(" - ")      + m_pickedPoints[1].prefix(POINT_INDEX_1)
					+	QString(" - ")      + m_pickedPoints[2].prefix(POINT_INDEX_2) );
		}
		else
		{
			setName(	QString("Triplet ") + m_pickedPoints[0].prefix(POINT_INDEX_0) + QString("@") + ENTITY_INDEX_0
					+	QString(" - ")      + m_pickedPoints[1].prefix(POINT_INDEX_1) + QString("@") + ENTITY_INDEX_1
					+	QString(" - ")      + m_pickedPoints[2].prefix(POINT_INDEX_2) + QString("@") + ENTITY_INDEX_2 );
		}
	}
	break;
	
	}
}

bool ccLineGroup::addPickedPoint(ccGenericPointCloud* cloud, unsigned pointIndex, bool entityCenter/*=false*/)
{
	if (!cloud || pointIndex >= cloud->size())
		return false;

	PickedPoint pp;
	pp._cloud = cloud;
	pp.index = pointIndex;
	pp.entityCenterPoint = entityCenter;

	return addPickedPoint(pp);

	return true;
}

bool ccLineGroup::addPickedPoint(ccGenericMesh* mesh, unsigned triangleIndex, const CCVector2d& uv, bool entityCenter/*=false*/)
{
	if (!mesh || triangleIndex >= mesh->size())
		return false;

	PickedPoint pp;
	pp._mesh = mesh;
	pp.index = triangleIndex;
	pp.uv = uv;
	pp.entityCenterPoint = entityCenter;

	return addPickedPoint(pp);
}

bool ccLineGroup::addPickedPoint(ccLineGroup* label, unsigned pointIndex)
{
	if (!label || label->size() == 0 || label->size() <= pointIndex)
		return false;

	PickedPoint pp;
	pp = label->getPickedPoint(pointIndex);

	return addPickedPoint(pp);
}

bool ccLineGroup::addPickedPoint(const PickedPoint& pp)
{
	/*if (m_pickedPoints.size() == 3)
	{
		return false;
	}*/

    try
    {
        m_pickedPoints.resize(m_pickedPoints.size() + 1);
    }
    catch (const std::bad_alloc&)
    {
        //not enough memory
        return false;
    }

    m_pickedPoints.back() = pp;


	//we want to be notified whenever an associated mesh is deleted (in which case
	//we'll automatically clear the label)
	if (pp.entity())
		pp.entity()->addDependency(this, DP_NOTIFY_OTHER_ON_DELETE);
	//we must also warn the cloud or mesh whenever we delete this label
	//--> DGM: automatically done by the previous call to addDependency!

	updateName();

	return true;
}

bool ccLineGroup::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = (uint32_t)m_pickedPoints.size();
	if (out.write((const char*)&count, 4) < 0)
		return WriteError();

	//points & associated cloud ID (dataVersion >= 20)
	for (std::vector<PickedPoint>::const_iterator it = m_pickedPoints.begin(); it != m_pickedPoints.end(); ++it)
	{
		//point index
		uint32_t index = static_cast<uint32_t>(it->index);
		if (out.write((const char*)&index, 4) < 0)
			return WriteError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = static_cast<uint32_t>(it->_cloud ? it->_cloud->getUniqueID() : 0);
		if (out.write((const char*)&cloudID, 4) < 0)
			return WriteError();

		//mesh ID (dataVersion >= 49 - will be retrieved later --> make sure that the mesh is saved alongside!)
		uint32_t meshID = static_cast<uint32_t>(it->_mesh ? it->_mesh->getUniqueID() : 0);
		if (out.write((const char*)&meshID, 4) < 0)
			return WriteError();
		
		//uv coordinates in the triangle (dataVersion >= 49)
		if (out.write((const char*)it->uv.u, sizeof(double) * 2) < 0)
			return WriteError();

		//entity center point (dataVersion >= 50)
		if (out.write((const char*)&(it->entityCenterPoint), sizeof(bool)) < 0)
			return WriteError();
	}

	//Relative screen position (dataVersion >= 20)
	if (out.write((const char*)m_screenPos, sizeof(float) * 2) < 0)
		return WriteError();

	//Collapsed state (dataVersion >= 20)
	if (out.write((const char*)&m_showFullBody, sizeof(bool)) < 0)
		return WriteError();

	//Show in 2D boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispIn2D, sizeof(bool)) < 0)
		return WriteError();

	//Show point(s) legend boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispPointsLegend, sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool ccLineGroup::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = 0;
	if (in.read((char*)&count, 4) < 0)
		return ReadError();

	//points & associated cloud/mesh ID (dataVersion >= 20)
	assert(m_pickedPoints.empty());
	for (uint32_t i = 0; i < count; ++i)
	{
		//point index
		uint32_t index = 0;
		if (in.read((char*)&index, 4) < 0)
			return ReadError();

		//cloud ID (will be retrieved later)
		{
			uint32_t cloudID = 0;
			if (in.read((char*)&cloudID, 4) < 0)
				return ReadError();

			if (cloudID != 0)
			{
				try
				{
					m_pickedPoints.resize(m_pickedPoints.size() + 1);
					m_pickedPoints.back().index = static_cast<unsigned>(index);
					//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'PickedPoint::_cloud' pointer!!!
					*(uint32_t*)(&m_pickedPoints.back()._cloud) = cloudID;
				}
				catch (const std::bad_alloc)
				{
					return MemoryError();
				}
			}
		}

		if (dataVersion >= 49)
		{
			//mesh ID (dataVersion >= 49 - will be retrieved later)
			uint32_t meshID = 0;
			if (in.read((char*)&meshID, 4) < 0)
				return ReadError();

			//uv coordinates in the triangle (dataVersion >= 49)
			CCVector2d uv;
			if (in.read((char*)uv.u, sizeof(double) * 2) < 0)
				return ReadError();

			if (meshID != 0)
			{
				try
				{
					m_pickedPoints.resize(m_pickedPoints.size() + 1);
					m_pickedPoints.back().index = static_cast<unsigned>(index);
					m_pickedPoints.back().uv = uv;
					//[DIRTY] WARNING: temporarily, we set the mesh unique ID in the 'PickedPoint::_mesh' pointer!!!
					*(uint32_t*)(&m_pickedPoints.back()._mesh) = meshID;
				}
				catch (const std::bad_alloc)
				{
					return MemoryError();
				}
			}
		}

		//entity center point (dataVersion >= 50)
		bool entityCenterPoint = false;
		if (dataVersion >= 50)
		{
			if (in.read((char*)&entityCenterPoint, sizeof(bool)) < 0)
				return ReadError();
		}
		m_pickedPoints.back().entityCenterPoint = entityCenterPoint;
	}

	//Relative screen position (dataVersion >= 20)
	if (in.read((char*)m_screenPos, sizeof(float) * 2) < 0)
		return ReadError();

	//Collapsed state (dataVersion >= 20)
	if (in.read((char*)&m_showFullBody, sizeof(bool)) < 0)
		return ReadError();

	if (dataVersion > 20)
	{
		//Show in 2D boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispIn2D, sizeof(bool)) < 0)
			return ReadError();

		//Show point(s) legend boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispPointsLegend, sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

void AddPointCoordinate(QStringList& body, QString pointShortName, const CCVector3& P, const ccShiftedObject& shiftedObject, int precision)
{
	bool isShifted = shiftedObject.isShifted();

	QString coordStr = pointShortName;
	if (isShifted)
	{
		body << coordStr;
		coordStr = QString("  [shifted]");
	}

	coordStr += QString(" (%1;%2;%3)").arg(P.x, 0, 'f', precision).arg(P.y, 0, 'f', precision).arg(P.z, 0, 'f', precision);
	body << coordStr;

	if (isShifted)
	{
		CCVector3d Pg = shiftedObject.toGlobal3d(P);
		QString globCoordStr = QString("  [original] (%1;%2;%3)").arg(Pg.x, 0, 'f', precision).arg(Pg.y, 0, 'f', precision).arg(Pg.z, 0, 'f', precision);
		body << globCoordStr;
	}
}

void AddPointCoordinate(QStringList& body, const ccLineGroup::PickedPoint& pp, int precision, QString pointName = QString())
{
	QString pointShortName;
	ccShiftedObject* shiftedObject = nullptr;
	
	if (pp._cloud)
	{
		shiftedObject = pp._cloud;
		if (pp.entityCenterPoint)
			pointShortName = CENTER_STRING + QString("@%1").arg(pp._cloud->getUniqueID());
		else
			pointShortName = QString("P#%0").arg(pp.index);
	}
	else if (pp._mesh)
	{
		ccGenericPointCloud* vertices = pp._mesh->getAssociatedCloud();
		if (!vertices)
		{
			assert(false);
			return;
		}
		shiftedObject = vertices;
		if (pp.entityCenterPoint)
			pointShortName = CENTER_STRING + QString("@%1").arg(pp._mesh->getUniqueID());
		else
			pointShortName = QString("Tri#%0").arg(pp.index);
	}

	if (!pointName.isEmpty())
		pointShortName = QString("%1 (%2)").arg(pointName, pointShortName);

	assert(shiftedObject);
	AddPointCoordinate(body, pointShortName, pp.getPointPosition(), *shiftedObject, precision);
}

void ccLineGroup::getLabelInfo1(LabelInfo1& info) const
{
	info = LabelInfo1();

	if (m_pickedPoints.size() != 1)
		return;

	const PickedPoint& pp = m_pickedPoints[0];

	if (!pp.entityCenterPoint)
	{
		//cloud and point index
		if (pp._cloud)
		{
			//normal
			info.hasNormal = pp._cloud->hasNormals();
			if (info.hasNormal)
			{
				info.normal = pp._cloud->getPointNormal(pp.index);
			}
			//color
			info.hasRGB = pp._cloud->hasColors();
			if (info.hasRGB)
			{
				info.color = pp._cloud->getPointColor(pp.index);
			}
			//scalar field
			info.hasSF = pp._cloud->hasDisplayedScalarField();
			if (info.hasSF)
			{
				ccScalarField* sf = nullptr;

				//fetch the real scalar field if possible
				if (pp._cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					sf = static_cast<ccPointCloud*>(pp._cloud)->getCurrentDisplayedScalarField();
				}

				if (sf)
				{
					info.sfValue = sf->getValue(pp.index);
					info.sfName = sf->getName();
					if (ccScalarField::ValidValue(info.sfValue) && sf->getGlobalShift() != 0)
					{
						info.sfShiftedValue = sf->getGlobalShift() + info.sfValue;
						info.sfValueIsShifted = true;
					}
				}
				else
				{
					info.sfValue = pp._cloud->getPointScalarValue(pp.index);
					info.sfName = "Scalar";
				}
			}
		}
		else if (pp._mesh)
		{
			CCVector3d w(pp.uv, 1.0 - pp.uv.x - pp.uv.y);
			//normal
			info.hasNormal = pp._mesh->hasNormals();
			if (info.hasNormal)
			{
				pp._mesh->interpolateNormalsBC(pp.index, w, info.normal);
			}
			//color
			info.hasRGB = pp._mesh->hasColors();
			if (info.hasRGB)
			{
				pp._mesh->interpolateColorsBC(pp.index, w, info.color);
			}
			//scalar field
			info.hasSF = pp._mesh->hasDisplayedScalarField();
			if (info.hasSF)
			{
				CCCoreLib::VerticesIndexes* vi = pp._mesh->getTriangleVertIndexes(pp.index);
				assert(vi);

				//fetch the real scalar field name if possible
				ccGenericPointCloud* vertices = pp._mesh->getAssociatedCloud();
				assert(vertices);

				ccScalarField* sf = nullptr;

				//fetch the real scalar field if possible
				if (vertices->isA(CC_TYPES::POINT_CLOUD))
				{
					sf = static_cast<ccPointCloud*>(vertices)->getCurrentDisplayedScalarField();
				}

				ScalarType s1 = CCCoreLib::NAN_VALUE;
				ScalarType s2 = CCCoreLib::NAN_VALUE;
				ScalarType s3 = CCCoreLib::NAN_VALUE;

				if (sf)
				{
					s1 = sf->getValue(vi->i1);
					s2 = sf->getValue(vi->i2);
					s3 = sf->getValue(vi->i3);
				}
				else
				{
					s1 = vertices->getPointScalarValue(vi->i1);
					s2 = vertices->getPointScalarValue(vi->i2);
					s3 = vertices->getPointScalarValue(vi->i3);
				}
			
				//interpolate the SF value
				if (ccScalarField::ValidValue(s1) && ccScalarField::ValidValue(s2) && ccScalarField::ValidValue(s3))
				{
					info.sfValue = static_cast<ScalarType>(s1 * w.u[0] + s2 * w.u[1] + s3 * w.u[2]);
				}

				if (sf)
				{
					info.sfName = sf->getName();
					if (ccScalarField::ValidValue(info.sfValue) && sf->getGlobalShift() != 0)
					{
						info.sfShiftedValue = sf->getGlobalShift() + info.sfValue;
						info.sfValueIsShifted = true;
					}
				}
				else
				{
					info.sfName = "Scalar";
				}
			}
		}
	}
}

void ccLineGroup::getLabelInfo2(LabelInfo2& info) const
{
	info = LabelInfo2();

	if (m_pickedPoints.size() != 2)
		return;

	//1st point
	CCVector3 P1 = m_pickedPoints[0].getPointPosition();
	//2nd point
	CCVector3 P2 = m_pickedPoints[1].getPointPosition();

	info.diff = P2 - P1;
}

void ccLineGroup::getLabelInfo3(LabelInfo3& info) const
{
	info = LabelInfo3();

	if (m_pickedPoints.size() != 3)
		return;

	//1st point
	CCVector3 P1 = m_pickedPoints[0].getPointPosition();
	//2nd point
	CCVector3 P2 = m_pickedPoints[1].getPointPosition();
	//3rd point
	CCVector3 P3 = m_pickedPoints[2].getPointPosition();

	//area
	CCVector3 P1P2 = P2 - P1;
	CCVector3 P1P3 = P3 - P1;
	CCVector3 P2P3 = P3 - P2;
	CCVector3 N = P1P2.cross(P1P3); //N = ABxAC
	info.area = N.norm() / 2;

	//normal
	N.normalize();
	info.normal = N;

	//edges length
	info.edges.u[0] = P1P2.normd();  //edge 1-2
	info.edges.u[1] = P2P3.normd();  //edge 2-3
	info.edges.u[2] = P1P3.normd();  //edge 3-1

	//angle
	info.angles.u[0] = CCCoreLib::RadiansToDegrees( P1P2.angle_rad( P1P3) ); //angleAtP1
	info.angles.u[1] = CCCoreLib::RadiansToDegrees( P2P3.angle_rad(-P1P2) ); //angleAtP2
	info.angles.u[2] = CCCoreLib::RadiansToDegrees( P1P3.angle_rad( P2P3) ); //angleAtP3 (should be equal to 180-a1-a2!)
}

QStringList ccLineGroup::getLabelContent(int precision) const
{
	QStringList body;

	switch (m_pickedPoints.size())
	{
	case 0:
		//can happen if the associated cloud(s) has(ve) been deleted!
		body << "Deprecated";
		break;

	case 1: //point
	{
		LabelInfo1 info;
		getLabelInfo1(info);

		//coordinates
		AddPointCoordinate(body, m_pickedPoints[0], precision);

		//normal
		if (info.hasNormal)
		{
			QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x, 0, 'f', precision).arg(info.normal.y, 0, 'f', precision).arg(info.normal.z, 0, 'f', precision);
			body << normStr;
		}
		//color
		if (info.hasRGB)
		{
			QString colorStr = QString("Color: (%1;%2;%3;%4)").arg(info.color.r).arg(info.color.g).arg(info.color.b).arg(info.color.a);
			body << colorStr;
		}
		//scalar field
		if (info.hasSF)
		{
			QString sfVal = GetSFValueAsString(info, precision);
			QString sfStr = QString("%1 = %2").arg(info.sfName, sfVal);
			body << sfStr;
		}
	}
	break;

	case 2: //vector
	{
		LabelInfo2 info;
		getLabelInfo2(info);

		//distance is now the default label title
		//PointCoordinateType dist = info.diff.norm();
		//QString distStr = QString("Distance = %1").arg(dist,0,'f',precision);
		//body << distStr;

		QString vecStr =	MathSymbolDelta + QString("X: %1\t").arg(info.diff.x, 0, 'f', precision)
						+	MathSymbolDelta + QString("Y: %1\t").arg(info.diff.y, 0, 'f', precision)
						+	MathSymbolDelta + QString("Z: %1"  ).arg(info.diff.z, 0, 'f', precision);

		body << vecStr;

		PointCoordinateType dXY = sqrt(info.diff.x*info.diff.x + info.diff.y*info.diff.y);
		PointCoordinateType dXZ = sqrt(info.diff.x*info.diff.x + info.diff.z*info.diff.z);
		PointCoordinateType dZY = sqrt(info.diff.z*info.diff.z + info.diff.y*info.diff.y);

		vecStr =	MathSymbolDelta + QString("XY: %1\t").arg(dXY, 0, 'f', precision)
				+	MathSymbolDelta + QString("XZ: %1\t").arg(dXZ, 0, 'f', precision)
				+	MathSymbolDelta + QString("ZY: %1"  ).arg(dZY, 0, 'f', precision);
		body << vecStr;

		AddPointCoordinate(body, m_pickedPoints[0], precision);
		AddPointCoordinate(body, m_pickedPoints[1], precision);
	}
	break;

	case 3: //triangle/plane
	{
		LabelInfo3 info;
		getLabelInfo3(info);

		//area
		QString areaStr = QString("Area = %1").arg(info.area, 0, 'f', precision);
		body << areaStr;

		//coordinates
		AddPointCoordinate(body, m_pickedPoints[0], precision, "A");
		AddPointCoordinate(body, m_pickedPoints[1], precision, "B");
		AddPointCoordinate(body, m_pickedPoints[2], precision, "C");

		//normal
		QString normStr = QString("Normal: (%1;%2;%3)").arg(info.normal.x, 0, 'f', precision).arg(info.normal.y, 0, 'f', precision).arg(info.normal.z, 0, 'f', precision);
		body << normStr;

		//angles
		QString angleStr = QString("Angles: A=%1 - B=%2 - C=%3 deg.")
			.arg(info.angles.u[0], 0, 'f', precision)
			.arg(info.angles.u[1], 0, 'f', precision)
			.arg(info.angles.u[2], 0, 'f', precision);
		body << angleStr;

		//edges
		QString edgesStr = QString("Edges: AB=%1 - BC=%2 - CA=%3")
			.arg(info.edges.u[0], 0, 'f', precision)
			.arg(info.edges.u[1], 0, 'f', precision)
			.arg(info.edges.u[2], 0, 'f', precision);
		body << edgesStr;
	}
	break;

	default:
		//assert(false);
		break;

	}

	return body;
}


bool ccLineGroup::acceptClick(int x, int y, Qt::MouseButton button)
{
	if (button == Qt::RightButton)
	{
		if (m_labelROI.contains(x - m_lastScreenPos[0], y - m_lastScreenPos[1]))
		{
			//toggle collapse state
			m_showFullBody = !m_showFullBody;
			return true;
		}
	}

	return false;
}

void ccLineGroup::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	bool m_mode2D = true;
	bool m_foreground = true;
	ccColor::Rgb m_rgbColor = ccColor::red;
	PointCoordinateType m_width = 2;
	bool m_showVertices = true;
	float m_vertMarkWidth = 5;

	unsigned vertCount = size();
	if (vertCount < 2)
		return;

	bool draw = false;

	if (MACRO_Draw3D(context))
	{
		draw = !m_mode2D;
	}
	else if (m_mode2D)
	{
		bool drawFG = MACRO_Foreground(context);
		draw = ((drawFG && m_foreground) || (!drawFG && !m_foreground));
	}

	if (!draw)
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
		glFunc->glPushName(getUniqueIDForDisplay());

	if (isColorOverridden())
		ccGL::Color4v(glFunc, getTempColor().rgba);
	else if (colorsShown())
		ccGL::Color3v(glFunc, m_rgbColor.rgb);

	//display polyline
	if (m_width != 0)
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(static_cast<GLfloat>(m_width));
	}

	//vertices visibility
	const ccGenericPointCloud::VisibilityTableType* _verticesVisibility = nullptr;
	{
		ccGenericPointCloud* verticesCloud = dynamic_cast<ccGenericPointCloud*>(getAssociatedCloud());
		if (verticesCloud)
		{
			_verticesVisibility = &(verticesCloud->getTheVisibilityArray());
		}
	}
	bool visFiltering = (_verticesVisibility && _verticesVisibility->size() >= vertCount);

	if (visFiltering)
	{
		glFunc->glBegin(GL_LINES);
		unsigned maxIndex = (m_isClosed ? vertCount : vertCount - 1);
		for (unsigned i = 0; i < maxIndex; ++i)
		{
			if (_verticesVisibility->at(i) != CCCoreLib::POINT_VISIBLE) // segment is hidden
				continue;

			unsigned nextIndex = ((i + 1) % vertCount);
			if (_verticesVisibility->at(nextIndex) != CCCoreLib::POINT_VISIBLE) // segment is hidden
				continue;

			ccGL::Vertex3v(glFunc, getPoint(i)->u);
			ccGL::Vertex3v(glFunc, getPoint(nextIndex)->u);
		}
		glFunc->glEnd();
	}
	else
	{
		//DGM: we do the 'GL_LINE_LOOP' manually as I have a strange bug
		//on one on my graphic cards with this mode!
		//glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);
		glFunc->glBegin(GL_LINE_STRIP);
		for (unsigned i = 0; i < vertCount; ++i)
		{
			ccGL::Vertex3v(glFunc, getPoint(i)->u);
		}
		if (m_isClosed)
		{
			ccGL::Vertex3v(glFunc, getPoint(0)->u);
		}
		glFunc->glEnd();
	}

	//display arrow
#if 0
	if (m_showArrow && m_arrowIndex < vertCount && (m_arrowIndex > 0 || m_isClosed))
	{
		unsigned i0 = (m_arrowIndex == 0 ? vertCount - 1 : m_arrowIndex - 1);
		unsigned i1 = m_arrowIndex;

		if (!visFiltering || (_verticesVisibility->at(i0) == CCCoreLib::POINT_VISIBLE && _verticesVisibility->at(i1) == CCCoreLib::POINT_VISIBLE))
		{
			const CCVector3* P0 = getPoint(i0);
			const CCVector3* P1 = getPoint(i1);
			//direction of the last polyline chunk
			CCVector3 u = *P1 - *P0;
			u.normalize();

			if (m_mode2D)
			{
				u *= -m_arrowLength;
				static const PointCoordinateType s_defaultArrowAngle = CCCoreLib::DegreesToRadians(static_cast<PointCoordinateType>(15.0));
				static const PointCoordinateType cost = cos(s_defaultArrowAngle);
				static const PointCoordinateType sint = sin(s_defaultArrowAngle);
				CCVector3 A(cost * u.x - sint * u.y, sint * u.x + cost * u.y, 0);
				CCVector3 B(cost * u.x + sint * u.y, -sint * u.x + cost * u.y, 0);
				glFunc->glBegin(GL_POLYGON);
				ccGL::Vertex3v(glFunc, (A + *P1).u);
				ccGL::Vertex3v(glFunc, (B + *P1).u);
				ccGL::Vertex3v(glFunc, (*P1).u);
				glFunc->glEnd();
			}
			else
			{
				if (!c_unitArrow)
				{
					c_unitArrow = QSharedPointer<ccCone>(new ccCone(0.5, 0.0, 1.0));
					c_unitArrow->showColors(true);
					c_unitArrow->showNormals(false);
					c_unitArrow->setVisible(true);
					c_unitArrow->setEnabled(true);
				}
				if (colorsShown())
					c_unitArrow->setTempColor(m_rgbColor);
				else
					c_unitArrow->setTempColor(context.pointsDefaultCol);
				//build-up unit arrow own 'context'
				CC_DRAW_CONTEXT markerContext = context;
				markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
				markerContext.display = nullptr;

				glFunc->glMatrixMode(GL_MODELVIEW);
				glFunc->glPushMatrix();
				ccGL::Translate(glFunc, P1->x, P1->y, P1->z);
				ccGLMatrix rotMat = ccGLMatrix::FromToRotation(u, CCVector3(0, 0, CCCoreLib::PC_ONE));
				glFunc->glMultMatrixf(rotMat.inverse().data());
				glFunc->glScalef(m_arrowLength, m_arrowLength, m_arrowLength);
				ccGL::Translate(glFunc, 0.0, 0.0, -0.5);
				c_unitArrow->draw(markerContext);
				glFunc->glPopMatrix();
			}
		}
	}
#endif
	if (m_width != 0)
	{
		glFunc->glPopAttrib();
	}

	//display vertices
	if (m_showVertices)
	{
		glFunc->glPushAttrib(GL_POINT_BIT);
		glFunc->glPointSize(static_cast<GLfloat>(m_vertMarkWidth));

		glFunc->glBegin(GL_POINTS);
		for (unsigned i = 0; i < vertCount; ++i)
		{
			if (!visFiltering || _verticesVisibility->at(i) == CCCoreLib::POINT_VISIBLE)
			{
				ccGL::Vertex3v(glFunc, getPoint(i)->u);
			}
		}
		glFunc->glEnd();

		glFunc->glPopAttrib();
	}

	if (pushName)
	{
		glFunc->glPopName();
	}
	//if (m_pickingMode == POINT_VOLUME1 && MACRO_Draw3D(context))
	//{

	//	if (isVisible() && isShowBox)
	//	{
	//		if (isSelected())
	//			m_box->draw(context, ccColor::red);
	//		else
	//			m_box->draw(context, context.labelDefaultMarkerCol);
	//	}	

	//}
	//
	//if (m_pickingMode != POINT_VOLUME1 && m_pickedPoints.empty())
	//	return;

	////2D foreground only
	//if (!MACRO_Foreground(context))
	//	return;

	////Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
	//if (MACRO_VirtualTransEnabled(context))
	//	return;

	//if (MACRO_Draw3D(context))
	//	drawMeOnly3D(context);
	//else if (MACRO_Draw2D(context))
	//	drawMeOnly2D(context);


	
}

//unit point marker
static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);

void ccLineGroup::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	//janson 22022.6.28
	if (m_pickingMode == POINT_VOLUME1 && ((context.drawingFlags & CC_DRAW_FAST_NAMES_ONLY) != CC_DRAW_FAST_NAMES_ONLY))
	{
		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
			return;

		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());
		//project the point in 2D
		CCVector3 P3D = m_box->getCenter();
		camera.project(P3D, m_box->pos2D);
	}

	size_t count = m_pickedPoints.size();
	if (count == 0)
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		//not particularly fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	//we always project the points in 2D (maybe useful later, even when displaying the label during the 2D pass!)
	ccGLCameraParameters camera;
	//we can't use the context 'ccGLCameraParameters' (viewport, modelView matrix, etc. )
	//because it doesn't take the temporary 'GL transformation' into account!
	//context.display->getGLCameraParameters(camera);
	glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
	glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
	glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

	//don't do this in picking mode!
	if (!pushName)
	{
		for (size_t i = 0; i < count; i++)
		{
			//project the point in 2D
			CCVector3 P3D = m_pickedPoints[i].getPointPosition();
			camera.project(P3D, m_pickedPoints[i].pos2D);
            ccLog::Print(QString("i = %1, x = %2, y = %2, z = %3").arg(i).arg(QString::number(m_pickedPoints[i].pos2D.x))
            .arg(QString::number(m_pickedPoints[i].pos2D.y)).arg(QString::number(m_pickedPoints[i].pos2D.z)));
        }
	}

	//bool loop = false;


	//display point marker as spheres
	{
		if (!c_unitPointMarker)
		{
			c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, nullptr, "PointMarker", 12));
			c_unitPointMarker->showColors(true);
			c_unitPointMarker->setVisible(true);
			c_unitPointMarker->setEnabled(true);
		}

		//build-up point maker own 'context'
		CC_DRAW_CONTEXT markerContext = context;
		markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
		markerContext.display = nullptr;

		if (isSelected() && !pushName)
			c_unitPointMarker->setTempColor(ccColor::green);
		else
			c_unitPointMarker->setTempColor(context.labelDefaultMarkerCol);

		const ccViewportParameters& viewportParams = context.display->getViewportParameters();
		for (size_t i = 0; i < count; i++)
		{
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			CCVector3 P = m_pickedPoints[i].getPointPosition();
			ccGL::Translate(glFunc, P.x, P.y, P.z);
			float scale = context.labelMarkerSize * m_relMarkerScale;
			if (viewportParams.perspectiveView && viewportParams.zFar > 0)
			{
				//in perspective view, the actual scale depends on the distance to the camera!
				double d = (camera.modelViewMat * P).norm();
				double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
				scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
			}
			glFunc->glScalef(scale, scale, scale);
			m_pickedPoints[i].markerScale = scale;
			c_unitPointMarker->draw(markerContext);
			glFunc->glPopMatrix();
		}
	}



	if (pushName)
	{
		glFunc->glPopName();
	}
}

//display parameters
static const int c_margin = 5;
static const int c_tabMarginX = 5;
static const int c_tabMarginY = 2;
static const int c_arrowBaseSize = 3;
//static const int c_buttonSize = 10;

static const ccColor::Rgba c_darkGreen(0, 200, 0, 255);

//! Data table
struct Tab
{
	//! Default constructor
	Tab(int _maxBlockPerRow = 2)
		: maxBlockPerRow(_maxBlockPerRow)
		, blockCount(0)
		, rowCount(0)
		, colCount(0)
	{}

	//! Sets the maximum number of blocks per row
	/** \warning Must be called before adding data!
	**/
	inline void setMaxBlockPerRow(int maxBlock) { maxBlockPerRow = maxBlock; }

	//! Adds a 2x3 block (must be filled!)
	int add2x3Block()
	{
		//add columns (if necessary)
		if (colCount < maxBlockPerRow * 2)
		{
			colCount += 2;
			colContent.resize(colCount);
			colWidth.resize(colCount, 0);
		}
		int blockCol = (blockCount % maxBlockPerRow);
		//add new row
		if (blockCol == 0)
			rowCount += 3;
		++blockCount;

		//return the first column index of the block
		return blockCol * 2;
	}

	//! Updates columns width table
	/** \return the total width
	**/
	int updateColumnsWidthTable(const QFontMetrics& fm)
	{
		//compute min width of each column
		int totalWidth = 0;
		for (int i = 0; i < colCount; ++i)
		{
			int maxWidth = 0;
			for (int j = 0; j < colContent[i].size(); ++j)
				maxWidth = std::max(maxWidth, fm.width(colContent[i][j]));
			colWidth[i] = maxWidth;
			totalWidth += maxWidth;
		}
		return totalWidth;
	}

	//! Maximum number of blocks per row
	int maxBlockPerRow;
	//! Number of 2x3 blocks
	int blockCount;
	//! Number of rows
	int rowCount;
	//! Number of columns
	int colCount;
	//! Columns width
	std::vector<int> colWidth;
	//! Columns content
	std::vector<QStringList> colContent;
};
#if 0
bool point2Round(std::vector<ccLineGroup::PickedPoint> points, double& radius, CCVector2& center, CC_DRAW_CONTEXT& context)
{
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    CCVector2 point1 = CCVector2(points[0].pos2D.x-halfW, points[0].pos2D.y-halfH);
    CCVector2 point2 = CCVector2(points[1].pos2D.x-halfW, points[1].pos2D.y-halfH);
    CCVector2 point3 = CCVector2(points[2].pos2D.x-halfW, points[2].pos2D.y-halfH);

    float A, B, C, D;

    float x1x1 = point1.x * point1.x;
    float y1y1 = point1.y * point1.y;
    float x2x2 = point2.x * point2.x;
    float y2y2 = point2.y * point2.y;
    float x3x3 = point3.x * point3.x;
    float y3y3 = point3.y * point3.y;

    float x2y3 = point2.x * point3.y;
    float x3y2 = point3.x * point2.y;

    float x2_x3 = point2.x - point3.x;
    float y2_y3 = point2.y - point3.y;

    float x1x1py1y1 = x1x1 + y1y1;
    float x2x2py2y2 = x2x2 + y2y2;
    float x3x3py3y3 = x3x3 + y3y3;

    A = point1.x * y2_y3 - point1.y * x2_x3 + x2y3 - x3y2;
    B = x1x1py1y1 * (-y2_y3) + x2x2py2y2 * (point1.y - point3.y) + x3x3py3y3 * (point2.y - point1.y);
    C = x1x1py1y1 * x2_x3 + x2x2py2y2 * (point3.x - point1.x) + x3x3py3y3 * (point1.x - point2.x);
    D = x1x1py1y1 * (x3y2 - x2y3) + x2x2py2y2 * (point1.x*point3.y - point3.x * point1.y) + x3x3py3y3 * (point2.x*point1.y - point1.x * point2.y);


    center.x = -B / (2 * A);
    center.y = -C / (2 * A);
    radius = sqrt((B*B + C * C - 4 * A*D) / (4 * A*A));

    if (!A)
    {
        return false;
    }
    else
    {
        printf("%.4lf %.4lf %.4lf\n", center.x, center.y, radius);
        return true;
    }
}

float Angle(CCVector2 form, CCVector2 to)
{
    float x = to.x - form.x;
    float y = to.y - form.y;

    float hy = sqrt(pow(x, 2) + pow(y, 2));

    float cos = x / hy;
    float radian = acos(cos);

    float angle = 180 / (PI / radian);

    //if (x <= 0 && y > 0) angle = 180 - angle;
    //if (x <= 0 && y < 0) angle = 180 + angle;
    //if (x > 0 && y <=0) angle = 360 - angle;

    if (y < 0) angle = 360 - angle;   // if (y < 0) angle = - angle;   //-180-180
    else if ((y == 0) && (x < 0)) angle = 180;

    return angle;
}

bool ArcCross(CCVector2 p1, CCVector2 p2, CCVector2 p3, CC_DRAW_CONTEXT& context)
{
    float halfW = context.glW / 2.0f;
    float halfH = context.glH / 2.0f;

    p1 = CCVector2(p1.x - halfW, p1.y - halfH);
    p2 = CCVector2(p2.x - halfW, p2.y - halfH);
    p3 = CCVector2(p3.x - halfW, p3.y - halfH);

    GLfloat fRet = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
    return fRet < 0;
}


/*
 * Draw Circle by Bresenham Algorithm
 * @para < xc, yc - 圆心: (xc, yc) >
 * @para < r - 半径 >
 * @para < deltaX - 坐标系每个小格的间距，用于控制精细度 >
 */
void drawCircle_Bresenham(GLfloat xc, GLfloat yc, GLfloat r, const GLfloat deltaX, QOpenGLFunctions_2_1 *glFunc) 
{
	GLfloat xi = -r, yi = 0; /* 圆上点 (xi, yi) */
	GLfloat du_l; /* upper - lower */
	glFunc->glBegin(GL_POINTS);
	while (abs(xi) >= abs(yi)) {
		// 根据圆的八向对称，只计算其中八分之一的点，然后对称得出其他点
		// 假设圆心在原点，先求点，再平移
		glFunc->glVertex2f(xc + xi, yc + yi);
		glFunc->glVertex2f(xc - xi, yc + yi);
		glFunc->glVertex2f(xc + xi, yc - yi);
		glFunc->glVertex2f(xc - xi, yc - yi);
		glFunc->glVertex2f(xc + yi, yc + xi);
		glFunc->glVertex2f(xc - yi, yc + xi);
		glFunc->glVertex2f(xc + yi, yc - xi);
		glFunc->glVertex2f(xc - yi, yc - xi);

		xi += deltaX; //　下一个x
		float yi_1 = sqrt(pow((GLfloat)r, 2) - pow((GLfloat)xi, 2)); // yi+1
		du_l = 2 * (GLfloat)yi + deltaX - 2 * yi_1;
		yi = (du_l <= 0) ? (int)yi_1 + deltaX : (int)yi_1;
	}
	glFunc->glEnd();
}
#endif

void ccLineGroup::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
#if 0
	assert((m_pickingMode == POINT_VOLUME1) || !m_pickedPoints.empty());

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	float halfW = context.glW / 2.0f;
	float halfH = context.glH / 2.0f;

	size_t count = m_pickedPoints.size();
	assert((m_pickingMode == POINT_VOLUME1) || count != 0);

	//we should already be in orthoprojective & centered mode
	//glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);


    //we always project the points in 2D (maybe useful later, even when displaying the label during the 2D pass!)
    ccGLCameraParameters camera;
    //we can't use the context 'ccGLCameraParameters' (viewport, modelView matrix, etc. )
    //because it doesn't take the temporary 'GL transformation' into account!
    //context.display->getGLCameraParameters(camera);
    glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
    glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
    glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());
    {
        if (!c_unitPointMarker)
        {
            c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, nullptr, "PointMarker", 12));
            c_unitPointMarker->showColors(true);
            c_unitPointMarker->setVisible(true);
            c_unitPointMarker->setEnabled(true);
        }

        //build-up point maker own 'context'
        CC_DRAW_CONTEXT markerContext = context;
        markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
        markerContext.display = nullptr;

        if (isSelected() && !pushName)
            c_unitPointMarker->setTempColor(ccColor::green);
        else
            c_unitPointMarker->setTempColor(context.labelDefaultMarkerCol);

        const ccViewportParameters& viewportParams = context.display->getViewportParameters();
        for (size_t i = 0; i < count; i++)
        {
            glFunc->glMatrixMode(GL_MODELVIEW);
            glFunc->glPushMatrix();
            CCVector3 P = m_pickedPoints[i].getPointPosition();
            ccGL::Translate(glFunc, P.x, P.y, P.z);
            float scale = context.labelMarkerSize * m_relMarkerScale;
            if (viewportParams.perspectiveView && viewportParams.zFar > 0)
            {
                //in perspective view, the actual scale depends on the distance to the camera!
                double d = (camera.modelViewMat * P).norm();
                double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
                scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
            }
            glFunc->glScalef(scale, scale, scale);
            m_pickedPoints[i].markerScale = scale;
            c_unitPointMarker->draw(markerContext);
            glFunc->glPopMatrix();
        }
    }

    if (m_pickedPoints.size() == 3)
    {
        float mSweepAngle_;
        float mStartAngle_;

        double radius = 0.0;
        CCVector2 center(0, 0);
        point2Round(m_pickedPoints, radius, center, context);

        float OA = Angle(center, CCVector2(m_pickedPoints[0].pos2D.x - halfW, m_pickedPoints[0].pos2D.y - halfH));   //求两个点的角度0~360
        float OB = Angle(center, CCVector2(m_pickedPoints[1].pos2D.x - halfW, m_pickedPoints[1].pos2D.y - halfH));
        float OC = Angle(center, CCVector2(m_pickedPoints[2].pos2D.x - halfW, m_pickedPoints[2].pos2D.y - halfH));

        float fMax = std::max(OA, OC);
        float fMin = std::min(OA, OC);
        if (OB > fMin && OB < fMax)
            mSweepAngle_ = fMax - fMin;
        else
            mSweepAngle_ = 360 - (fMax - fMin);

        //起始角
        BOOL bCross = ArcCross(CCVector2(m_pickedPoints[0].pos2D.x, m_pickedPoints[0].pos2D.y),
            CCVector2(m_pickedPoints[1].pos2D.x, m_pickedPoints[1].pos2D.y), CCVector2(m_pickedPoints[2].pos2D.x, m_pickedPoints[2].pos2D.y), context);//逆时针=0,顺时针=1
        mStartAngle_ = bCross ? OC : OA;


        glFunc->glLineWidth(2.0);	//设置画笔大小
        ccGL::Color4v(glFunc, ccColor::red.rgba);//设置颜色

		drawCircle_Bresenham(center.x, center.y, radius, 0.0001, glFunc);

  //      //GL_LINE_STRIP不闭合
  //      glFunc->glBegin(GL_LINE_STRIP);
		//GLfloat angle = 0;//mStartAngle_;
  //      GLdouble fStep = mSweepAngle_ / 1000;	//循环次数   DOTCOUNT画点的个数
  //      for (; angle <= 360/*mStartAngle_ + mSweepAngle_*/; angle += fStep)
  //      {
  //          GLfloat x = radius * COS(angle) + center.x;
  //          GLfloat y = radius * SIN(angle) + center.y;
  //          glFunc->glVertex2f(x, y);
  //      }
  //      glFunc->glEnd();
    }
    

	//hack: we display the label connecting 'segments' and the point(s) legend
	//in 2D so that they always appear above the entities
	{
		//test if the label points are visible
		size_t visibleCount = 0;
		for (unsigned j = 0; j < count; ++j)
		{
			if (m_pickedPoints[j].pos2D.z >= 0.0 && m_pickedPoints[j].pos2D.z <= 1.0)
			{
				++visibleCount;
			}
		}

		if (visibleCount)
		{
			glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
			glFunc->glDisable(GL_DEPTH_TEST);

			//contour segments (before the labels!)
			if (count > 1)
			{
				//segment width
				const float c_sizeFactor = 4.0f;
				glFunc->glPushAttrib(GL_LINE_BIT);
				glFunc->glLineWidth(c_sizeFactor * context.renderZoom);

				//we draw the segments
				if (isSelected())
					ccGL::Color4v(glFunc, ccColor::red.rgba);
				else
					ccGL::Color4v(glFunc, context.labelDefaultMarkerCol.rgba/*ccColor::green.rgba*/);

				glFunc->glBegin(count == 2 ? GL_LINES : GL_LINE_STRIP);
				for (unsigned j = 0; j < count; ++j)
				{
					glFunc->glVertex2d(m_pickedPoints[j].pos2D.x - halfW, m_pickedPoints[j].pos2D.y - halfH);
				}
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_LINE_BIT
			}

			//no need to display the point(s) legend in picking mode
			if (m_dispPointsLegend && !pushName)
			{
				QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
				//font.setPointSize(font.pointSize() + 2);
				font.setBold(true);
				static const QChar ABC[3] = { 'A', 'B', 'C' };

				//draw the label 'legend(s)'
				for (size_t j = 0; j < count; j++)
				{
					QString title;
					if (count == 1)
						title = getName(); //for single-point labels we prefer the name
					else if (count == 3)
						title = ABC[j]; //for triangle-labels, we only display "A","B","C"

					context.display->displayText(title,
						static_cast<int>(m_pickedPoints[j].pos2D.x) + context.labelMarkerTextShift_pix,
						static_cast<int>(m_pickedPoints[j].pos2D.y) + context.labelMarkerTextShift_pix,
						ccGenericGLDisplay::ALIGN_DEFAULT,
						context.labelOpacity / 100.0f,
						&ccColor::white,
						&font);
				}
			}

			glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT
		}
		else if(m_pickingMode != POINT_VOLUME1)
		{
			//no need to draw anything (might be confusing)
			if (pushName)
			{
				glFunc->glPopName();
			}
			return;
		}
	}

	if (!m_dispIn2D)
	{
		//nothing to do
		if (pushName)
		{
			glFunc->glPopName();
		}
		return;
	}
#endif
}

bool ccLineGroup::pointPicking(	const CCVector2d& clickPos,
								const ccGLCameraParameters& camera,
								int& nearestPointIndex,
								double& nearestSquareDist) const
{
	nearestPointIndex = -1;
	nearestSquareDist = -1.0;
	{
		//back project the clicked point in 3D
		CCVector3d clickPosd(clickPos.x, clickPos.y, 0.0);
		CCVector3d X(0, 0, 0);
		if (!camera.unproject(clickPosd, X))
		{
			return false;
		}

		clickPosd.z = 1.0;
		CCVector3d Y(0, 0, 0);
		if (!camera.unproject(clickPosd, Y))
		{
			return false;
		}

		CCVector3d xy = (Y - X);
		xy .normalize();

		for (unsigned i = 0; i < size(); ++i)
		{
			const PickedPoint& pp = getPickedPoint(i);
			if (pp.markerScale == 0)
			{
				//never displayed
				continue;
			}

			const CCVector3 P = pp.getPointPosition();

			//warning: we have to handle the relative GL transformation!
			ccGLMatrix trans;
			bool noGLTrans = pp.entity() ? !pp.entity()->getAbsoluteGLTransformation(trans) : true;

			CCVector3d Q2D;
			bool insideFrustum = false;
			if (noGLTrans)
			{
				camera.project(P, Q2D, &insideFrustum);
			}
			else
			{
				CCVector3 P3D = P;
				trans.apply(P3D);
				camera.project(P3D, Q2D, &insideFrustum);
			}

			if (!insideFrustum)
			{
				continue;
			}

			// closest distance to XY
			CCVector3d XP = (P.toDouble() - X);
			double squareDist = (XP - XP.dot(xy) * xy).norm2();

			if (squareDist <= pp.markerScale * pp.markerScale)
			{
				if (nearestPointIndex < 0 || squareDist < nearestSquareDist)
				{
					nearestSquareDist = squareDist;
					nearestPointIndex = i;
				}
			}
		}
	}

	return (nearestPointIndex >= 0);
}
