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
#include "cc2DLabel.h"
#include "ccBasicTypes.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccGenericMesh.h"
#include "ccScalarField.h"
#include "ccSphere.h"
//Qt
#include <QSharedPointer>
#include <qdebug.h>
#include <QApplication>
#include <QVector3D>
//System
#include <assert.h>
#include <string.h>
#include "QFontMetrics"
#include "ccHObjectCaster.h"

//'Delta' character
static const QChar MathSymbolDelta(0x0394);

static const QString CENTER_STRING = QObject::tr("Center");
static const char POINT_INDEX_0[]  = "pi0";
static const char POINT_INDEX_1[]  = "pi1";
static const char POINT_INDEX_2[]  = "pi2";
static const char ENTITY_INDEX_0[] = "ei0";
static const char ENTITY_INDEX_1[] = "ei1";
static const char ENTITY_INDEX_2[] = "ei2";


QString cc2DLabel::PickedPoint::itemTitle() const
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

QString cc2DLabel::PickedPoint::prefix(const char* pointTag) const
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

CCVector3 cc2DLabel::PickedPoint::getPointPosition() const
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

unsigned cc2DLabel::PickedPoint::getUniqueID() const
{
	if (_cloud)
		return _cloud->getUniqueID();
	if (_mesh)
		return _mesh->getUniqueID();

	assert(false);
	return 0;
}

ccGenericPointCloud* cc2DLabel::PickedPoint::cloudOrVertices() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh->getAssociatedCloud();
	
	assert(false);
	return nullptr;
}

ccHObject* cc2DLabel::PickedPoint::entity() const
{
	if (_cloud)
		return _cloud;
	if (_mesh)
		return _mesh;

	assert(false);
	return nullptr;
}

cc2DLabel::cc2DLabel(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
	, m_showFullBody(true)
	, m_dispPointsLegend(false)
	, m_dispIn2D(true)
	, m_relMarkerScale(1.0f)
	, m_box(nullptr)
	, isShowBox(false)
    , m_showNormal(true)
    , m_showRGB(true)
    , m_showShifted(true)
{
	m_screenPos[0] = m_screenPos[1] = 0.05f;

	clear(false);

	lockVisibility(false);
	setEnabled(true);
}

QString cc2DLabel::GetSFValueAsString(const LabelInfo1& info, int precision)
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

QString cc2DLabel::getTitle(int precision, CC_DRAW_CONTEXT& context) const
{
	QString title;
	size_t count = m_pickedPoints.size();
	
	if (count == 1 && m_pickingMode != POINT_MESH_VOLUME)
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
    if (count == 1 && m_pickingMode == POINT_MESH_VOLUME)
    {
        QString unit = m_volumelist[context.volumnunit];
        title = m_name;
        if (context.isshowUnitlabel)
        {
            title = QString("Volume: %1").arg(title.toDouble() * m_volumnunitshift[context.volumnunit], 0, 'f', precision) + unit;
        }
        else
        {
            title = QString("Volume: %1").arg(title.toDouble() * m_volumnunitshift[context.volumnunit], 0, 'f', precision);
        }
    }
	if (m_pickingMode == POINT_AERA)
	{
		if (context.isshowUnitlabel)
		{
			QString unit = m_arealist[context.areaunit];
			title = QString("area: %1\t").arg(m_measureResult * m_areaunitshift[context.areaunit], 0, 'f', precision) + unit;
		}
		else
		{
			title = QString("area: %1\t").arg(m_measureResult * m_areaunitshift[context.areaunit], 0, 'f', precision);
		}
	}

	return title;
}

QString cc2DLabel::getName() const
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

void cc2DLabel::setPosition(float x, float y)
{
	m_screenPos[0] = x;
	m_screenPos[1] = y;
}

bool cc2DLabel::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	assert(screenHeight > 0 && screenWidth > 0);

	m_screenPos[0] += static_cast<float>(dx) / screenWidth;
	m_screenPos[1] += static_cast<float>(dy) / screenHeight;
	//限制无法拖出窗口
	if (m_screenPos[0] <= 0 || m_screenPos[0] >= 1)
	{
		m_screenPos[0] -= static_cast<float>(dx) / screenWidth;
	}
	if (m_screenPos[1] <= 0 || m_screenPos[1] >= 1)
	{
		m_screenPos[1] -= static_cast<float>(dy) / screenHeight;
	}

	return true;
}

void cc2DLabel::clear(bool ignoreDependencies)
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

void cc2DLabel::onDeletionOf(const ccHObject* obj)
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

void cc2DLabel::updateName()
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

bool cc2DLabel::addPickedPoint(ccGenericPointCloud* cloud, unsigned pointIndex, bool entityCenter/*=false*/)
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

bool cc2DLabel::addPickedPoint(ccGenericMesh* mesh, unsigned triangleIndex, const CCVector2d& uv, bool entityCenter/*=false*/)
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

bool cc2DLabel::addPickedPoint(cc2DLabel* label, unsigned pointIndex)
{
	if (!label || label->size() == 0 || label->size() <= pointIndex)
		return false;

	PickedPoint pp;
	pp = label->getPickedPoint(pointIndex);

	return addPickedPoint(pp);
}

bool cc2DLabel::addPickedPoint(const PickedPoint& pp)
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

bool cc2DLabel::toFile_MeOnly(QFile& out) const
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

bool cc2DLabel::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
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

void AddPointCoordinates(QStringList& body, QString pointShortName, const CCVector3& P, const ccShiftedObject& shiftedObject, int precision)
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

void AddPointCoordinates(QStringList& body, const cc2DLabel::PickedPoint& pp, int precision, QString pointName = QString())
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
	AddPointCoordinates(body, pointShortName, pp.getPointPosition(), *shiftedObject, precision);
}

void cc2DLabel::getLabelInfo1(LabelInfo1& info) const
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

void cc2DLabel::getLabelInfo2(LabelInfo2& info) const
{
	info = LabelInfo2();

	if (m_areaPointList.size() != 2)
		return;

	//1st point
	CCVector3 P1 = m_areaPointList[0].toFloat();
	//2nd point
	CCVector3 P2 = m_areaPointList[1].toFloat();

	info.diff = P2 - P1;
}

void cc2DLabel::getLabelInfo3(LabelInfo3& info) const
{
	info = LabelInfo3();

	if (m_areaPointList.size() != 3)
		return;

	//1st point
	CCVector3 P1 = m_areaPointList[0].toFloat();
	//2nd point
	CCVector3 P2 = m_areaPointList[1].toFloat();
	//3rd point
	CCVector3 P3 = m_areaPointList[2].toFloat();

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
	if (!(info.angles.u[1] >= 0 && info.angles.u[1] <= 180))
	{
		info.angles.u[1] = 0.000;
	}
}

QStringList cc2DLabel::getLabelContent(int precision) const
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
		AddPointCoordinates(body, m_pickedPoints[0], precision);

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

		AddPointCoordinates(body, m_pickedPoints[0], precision);
		AddPointCoordinates(body, m_pickedPoints[1], precision);
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
		AddPointCoordinates(body, m_pickedPoints[0], precision, "A");
		AddPointCoordinates(body, m_pickedPoints[1], precision, "B");
		AddPointCoordinates(body, m_pickedPoints[2], precision, "C");

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


bool cc2DLabel::acceptClick(int x, int y, Qt::MouseButton button)
{
	if (button == Qt::RightButton)
	{
		if (m_labelROI.contains(x - m_lastScreenPos[0], y - m_lastScreenPos[1]))
		{
			//toggle collapse state
			//m_showFullBody = !m_showFullBody;
			return true;
		}
	}

	return false;
}

void cc2DLabel::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (!isVisible())
    {
        return;
    }
	if (m_pickingMode == POINT_VOLUME && MACRO_Draw3D(context))
	{

		if (isVisible() && isShowBox)
		{
			if (isSelected())
				m_box->draw(context, ccColor::red);
			else
				m_box->draw(context, context.labelDefaultMarkerCol);
		}	

	}
	if (m_pickingMode == POINT_INFO && m_pickedPoints.empty())
	{
		return;
	}
	//if (m_pickingMode != POINT_VOLUME && m_pickedPoints.empty() && !(m_pickingMode == POINT_AERA && m_areaPointList.size() >= 3))
	//	return;

	//2D foreground only
	if (!MACRO_Foreground(context))
		return;

	//Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
	if (MACRO_VirtualTransEnabled(context))
		return;

	if (MACRO_Draw3D(context))
		drawMeOnly3D(context);
	else if (MACRO_Draw2D(context))
		drawMeOnly2D(context);


	
}

//unit point marker
static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);

void cc2DLabel::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	//janson 22022.6.28
	if (m_pickingMode == POINT_VOLUME && ((context.drawingFlags & CC_DRAW_FAST_NAMES_ONLY) != CC_DRAW_FAST_NAMES_ONLY))
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
	int areaPointNum = m_areaPointList.size();;

	if (count == 0 && areaPointNum == 0)
	{
		return;
	}
	if ((m_pickingMode == POINT_AERA && areaPointNum < 3) || (m_pickingMode == POINT_HEIGHT && areaPointNum < 2) || (m_pickingMode == POINTS_ANGLE && areaPointNum < 3) || (m_pickingMode == POINT_POINT_DISTANCE && areaPointNum < 2))
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
		//if (m_pickingMode != POINT_AERA)
		{
			for (size_t i = 0; i < count; i++)
			{
				//project the point in 2D
                CCVector3 P3D = m_pickedPoints[i].getPointPosition();
				//CCVector3d P3D = m_pickedPoints[i].getPointPosition();
    //            if (m_pickedPoints[i].entity())
    //            {
    //                ccShiftedObject* shiftcloud = static_cast<ccShiftedObject*>(m_pickedPoints[i].entity());
    //                if (shiftcloud)
    //                {
    //                    P3D -= shiftcloud->getGlobalShift();
    //                }
    //            }
				camera.project(P3D, m_pickedPoints[i].pos2D);
			}
		}
		//else
		{
			m_areaPoint2dList.clear();
			bool allinFrustum = true;
			for (size_t i = 0; i < areaPointNum; i++)
			{
				//project the point in 2D
				CCVector3 P3D = m_areaPointList[i].toFloat();
				CCVector3d P2D;
				bool inFrustum = false;
				bool success = camera.project(P3D,P2D, &inFrustum);
				if (!inFrustum)
				{
					allinFrustum = false;
					m_areaPoint2dList.clear();
					break;
				}
				m_areaPoint2dList.push_back(P2D);
			}
		}
	}
	if (count == 1)
	{
        CCVector3 P = m_pickedPoints[0].getPointPosition();
        ccColor::Rgba pointcol = ccColor::red;
        if (isSelected() && !pushName)
        {
            pointcol = ccColor::red;
        }
        else
        {
            if (m_isShowConfigColor)
            {
                pointcol = context.labelDefaultMarkerCol;
            }
            else
            {
                pointcol = m_pointColor;
            }

        }
        glFunc->glPushAttrib(GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT | GL_POINT_BIT);
        glFunc->glEnable(GL_POINT_SMOOTH);
        glFunc->glColor4ub(pointcol.r, pointcol.g, pointcol.b, 255);
        glFunc->glPointSize(context.labelOriginalMarkerSize*2);
        glFunc->glBegin(GL_POINTS);
        ccGL::Vertex3v(glFunc, P.u);
        glFunc->glEnd();
        glFunc->glPopAttrib(); //GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT | GL_POINT_BIT --> will switch the light off
	}
	else
	{
		if ((areaPointNum == 3 && m_pickingMode == POINTS_ANGLE))
		{
			glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
			glFunc->glEnable(GL_BLEND);

			//we draw the triangle
			glFunc->glColor4ub(255, 255, 0, 128);
			glFunc->glBegin(GL_TRIANGLES);
			for (size_t i = 0; i < areaPointNum; i++)
			{
				//project the point in 2D
				CCVector3 P3D = m_areaPointList[i].toFloat();
				ccGL::Vertex3v(glFunc, P3D.u);
			}
			glFunc->glEnd();

			glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT
		}
	}

	//绘制高度测量结果的直角边
	if (areaPointNum == 2 && m_pickingMode == POINT_HEIGHT)
	{
		CCVector3d pointLower = (m_areaPointList[0].z < m_areaPointList[1].z) ? m_areaPointList[0] : m_areaPointList[1];
		CCVector3d pointUpper = (m_areaPointList[0].z > m_areaPointList[1].z) ? m_areaPointList[0] : m_areaPointList[1];
		CCVector3 thirdPoint(pointUpper.x, pointUpper.y, pointLower.z);
		CCVector3d textPos = (pointUpper + thirdPoint) / static_cast<PointCoordinateType>(2);
		camera.project(textPos, m_HeightMeasureTextPos);
		double distanceLower = std::sqrt((pointLower.x - thirdPoint.x) * (pointLower.x - thirdPoint.x) + (pointLower.y - thirdPoint.y) * (pointLower.y - thirdPoint.y) + (pointLower.z - thirdPoint.z) * (pointLower.z - thirdPoint.z));
		double distanceUpper = std::sqrt((pointUpper.x - thirdPoint.x) * (pointUpper.x - thirdPoint.x) + (pointUpper.y - thirdPoint.y) * (pointUpper.y - thirdPoint.y) + (pointUpper.z - thirdPoint.z) * (pointUpper.z - thirdPoint.z));
		double distanceMin = std::min(distanceLower, distanceUpper) / 6.0;
		CCVector3 anglePointLower = thirdPoint + CCVector3((pointLower.x - thirdPoint.x)/ distanceLower* distanceMin, (pointLower.y - thirdPoint.y) / distanceLower * distanceMin, (pointLower.z - thirdPoint.z) / distanceLower * distanceMin);
		CCVector3 anglePointUpper = thirdPoint + CCVector3((pointUpper.x - thirdPoint.x) / distanceUpper * distanceMin, (pointUpper.y - thirdPoint.y) / distanceUpper * distanceMin, (pointUpper.z - thirdPoint.z) / distanceUpper * distanceMin);
		CCVector3 anglePointCenter = CCVector3(anglePointLower.x, anglePointLower.y, anglePointUpper.z);

		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(2);

		//we draw the segments
		if (isSelected())
			ccGL::Color4v(glFunc, ccColor::red.rgba);
		else
			ccGL::Color4v(glFunc, context.labelDefaultMarkerCol.rgba/*ccColor::green.rgba*/);
		glFunc->glLineStipple(1, 0xAAAA);
		glFunc->glDisable(GL_DEPTH_TEST);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(pointLower.x, pointLower.y, pointLower.z);
		glFunc->glVertex3f(thirdPoint.x, thirdPoint.y, thirdPoint.z);
		glFunc->glVertex3f(thirdPoint.x, thirdPoint.y, thirdPoint.z);
		glFunc->glVertex3f(pointUpper.x, pointUpper.y, pointUpper.z);
		glFunc->glVertex3f(anglePointLower.x, anglePointLower.y, anglePointLower.z);
		glFunc->glVertex3f(anglePointCenter.x, anglePointCenter.y, anglePointCenter.z);
		glFunc->glVertex3f(anglePointCenter.x, anglePointCenter.y, anglePointCenter.z);
		glFunc->glVertex3f(anglePointUpper.x, anglePointUpper.y, anglePointUpper.z);
		glFunc->glEnd();
		glFunc->glPopAttrib(); //GL_LINE_BIT
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

static bool line2DLineIntersection(
	const std::vector<CCVector3d>& line1,
	const std::vector<CCVector3d>& line2, CCVector3d& intersection)
{

	if (line1.size() < 2 || line2.size() < 2) {
		return false;
	}

	CCVector3d p1 = line1[0];
	CCVector3d p2 = line1[1];
	CCVector3d p3 = line2[0];
	CCVector3d p4 = line2[1];

	QLineF lineq1(QPointF(p1.x, p1.y), QPointF(p2.x, p2.y));
	QLineF lineq2(QPointF(p3.x, p3.y), QPointF(p4.x, p4.y));

	CCVector2d v1 = CCVector2d(p2.x, p2.y) - CCVector2d(p1.x, p1.y);
	CCVector2d v2 = CCVector2d(p4.x, p4.y) - CCVector2d(p3.x, p3.y);

	double epsilon = 1e-1;
	double dVCross = v1.cross(v2);
	if (std::abs(dVCross) < 0.1) {
		return false;
	}

	QPointF p;
	QLineF::IntersectionType type = lineq1.intersects(lineq2, &p);
	if (type != QLineF::BoundedIntersection) {
		return false;
	}

	CCVector3d p2d;
	p2d.x = p.x();
	p2d.y = p.y();
	CCVector3d p3d = p2d;
	intersection = p3d;
	return true;
}

static bool segment2DSegmentIntersection(
	const std::vector<CCVector3d>& segment1,
	const std::vector<CCVector3d>& segment2, CCVector3d& intersection)
{
	if (segment1.size() < 2 || segment2.size() < 2) {
		return false;
	}

	if (!line2DLineIntersection(segment1, segment2, intersection)) {
		return false;
	}
	return true;
}

static bool segmentPolylinesIntersection(
	std::vector<CCVector3d> polylines1,
	const bool& bClose1,
	std::vector<CCVector3d> polylines2,
	const bool& bClose2,
	std::vector<CCVector3d>& intersection)
{

	if (polylines1.size() < 2 || polylines2.size() < 2) {
		return false;
	}

	if (bClose1) {
		polylines1.push_back(polylines1.at(0));
	}

	if (bClose2) {
		polylines2.push_back(polylines2.at(0));
	}

	for (int i = 0; i < polylines1.size() - 1; i++) {
		std::vector<CCVector3d> segment1;
		segment1.push_back(polylines1.at(i));
		segment1.push_back(polylines1.at(i + 1));

		for (int j = 0; j < polylines2.size() - 1; j++) {
			std::vector<CCVector3d> segment2;
			segment2.push_back(polylines2.at(j));
			segment2.push_back(polylines2.at(j + 1));
			CCVector3d inters;
			bool bResult = segment2DSegmentIntersection(segment1, segment2, inters);
			if (bResult) {
				intersection.push_back(inters);
			}
		}
	}

	return !intersection.empty();

}

void cc2DLabel::drawPolygonFilling(std::vector<CCVector3d> points, CC_DRAW_CONTEXT& context)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (!glFunc || points.size() < 3) {
		return;
	}
	glFunc->glPushAttrib(GL_LINE_BIT);
	double minx = std::numeric_limits<double>::max();
	double maxx = std::numeric_limits<double>::min();
	double miny = std::numeric_limits<double>::max();
	double maxy = std::numeric_limits<double>::min();

	//[!].线计算最大最小点
	for (int i = 0; i < points.size(); i++) {
		CCVector3d p = points[i];
		minx = std::min(p.x, minx);
		maxx = std::max(p.x, maxx);

		miny = std::min(p.y, miny);
		maxy = std::max(p.y, maxy);
	}

	std::vector<CCVector3d> listpoints;
	CCVector3d topleft = CCVector3d(minx, maxy, 0);
	CCVector3d topright = CCVector3d(maxx, maxy, 0);
	CCVector3d bottomright = CCVector3d(maxx, miny, 0);
	CCVector3d bottomleft = CCVector3d(minx, miny, 0);

	CCVector3d wr = (topright - topleft);
	CCVector3d hr = (topleft - bottomleft);
	wr.normalize();
	hr.normalize();
	double dw = (CCVector3d(minx, maxy, 0) - CCVector3d(maxx, maxy, 0)).norm();//宽
	double dh = (CCVector3d(minx, maxy, 0) - CCVector3d(minx, miny, 0)).norm();//[!];
	double ds = 25.0;//[!].
	int wc = 3;
	int hc = 3;
	if (dw < ds) {
		wc = 3;
		ds = dw / wc;
	}
	else {
		wc = dw / ds;
	}
	hc = dh / ds;

	//[!].交点排序
	auto exeIntersectionSort = [](std::vector<CCVector3d>& points, CCVector3d p) {
		std::sort(points.begin(), points.end(), [=](const CCVector3d& hr1, const CCVector3d& hr2) {

			//[!].计算距离

			CCVector3d diffvec = p - hr1;
			double d1 = QVector3D(diffvec.x, diffvec.y, diffvec.z).length();
			diffvec = p - hr2;
			double d2 = QVector3D(diffvec.x, diffvec.y, diffvec.z).length();
			return d1 < d2;
			});

		};

	glFunc->glEnable(GL_LINE_STIPPLE);
	glFunc->glLineStipple(2, 0x0FFF);
	//[!].线绘制X方向
	for (int i = 0; i < wc; i++) {

		CCVector3d p1 = bottomleft + ds * (i + 1) * wr;
		CCVector3d p2 = topleft + ds * (i + 1) * wr;
		//[!].计算交点，并排序
		std::vector<CCVector3d> segment;
		std::vector<CCVector3d> intersection;
		segment.push_back(p1);
		segment.push_back(p2);

		segmentPolylinesIntersection(segment,false, points, false, intersection);
		if (intersection.size() < 2) {
			continue;
		}
		exeIntersectionSort(intersection, p1);
		for (int j = 0; j < intersection.size() - 1; j = j + 2) {
			CCVector3d ip1 = intersection[j];
			CCVector3d ip2 = intersection[j + 1];
			QColor cl = QColor(255, 128, 0);
			glFunc->glLineWidth(static_cast<GLfloat>(2));
			glFunc->glColor4f(cl.redF(), cl.greenF(), cl.blueF(), cl.alphaF());
			glFunc->glBegin(GL_LINE_STRIP);
			glFunc->glVertex2f(ip1.x, ip1.y);
			glFunc->glVertex2f(ip2.x, ip2.y);
			glFunc->glEnd();
		}
	}

	for (int i = 0; i < hc; i++) {
		CCVector3d p1 = bottomleft + ds * (i + 1) * hr;
		CCVector3d p2 = bottomright + ds * (i + 1) * hr;
		//[!].计算交点，并排序
		std::vector<CCVector3d> segment;
		std::vector<CCVector3d> intersection;
		segment.push_back(p1);
		segment.push_back(p2);
		segmentPolylinesIntersection(segment, false, points, false, intersection);
		if (intersection.size() < 2) {
			continue;
		}
		exeIntersectionSort(intersection, p1);
		for (int j = 0; j < intersection.size() - 1; j = j + 2) {
			CCVector3d ip1 = intersection[j];
			CCVector3d ip2 = intersection[j + 1];
			QColor cl = QColor(255, 128, 0);
			glFunc->glLineWidth(static_cast<GLfloat>(2));
			glFunc->glColor4f(cl.redF(), cl.greenF(), cl.blueF(), cl.alphaF());
			glFunc->glBegin(GL_LINE_STRIP);
			glFunc->glVertex2f(ip1.x, ip1.y);
			glFunc->glVertex2f(ip2.x, ip2.y);
			glFunc->glEnd();
		}
	}
	glFunc->glPopAttrib();
}

void cc2DLabel::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
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
	int areaPointNum = m_areaPoint2dList.size();
	if (m_pickingMode == POINT_AERA && areaPointNum >= 3)
	{
		std::vector<CCVector3d> copypoint = m_areaPoint2dList;
		for (auto & cutcopypoint : copypoint)
		{
			cutcopypoint.x -= halfW;
			cutcopypoint.y -= halfH;
		}
		copypoint.push_back(copypoint[0]);
		drawPolygonFilling(copypoint, context);
	}
	assert((m_pickingMode == POINT_VOLUME) || count != 0 || areaPointNum != 0);

	if ((m_pickingMode == POINT_AERA && areaPointNum < 3) || (m_pickingMode == POINT_HEIGHT && areaPointNum < 2) || (m_pickingMode == POINTS_ANGLE && areaPointNum < 3) || (m_pickingMode == POINT_POINT_DISTANCE && areaPointNum < 2))
	{
		return;
	}

	//we should already be in orthoprojective & centered mode
	//glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);

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

		if (visibleCount || areaPointNum != 0)
		{
			glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
			glFunc->glDisable(GL_DEPTH_TEST);

			//contour segments (before the labels!)
			if (areaPointNum > 1)
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

				if (areaPointNum == 2)
				{
					glFunc->glBegin(GL_LINES);
				}
				else
				{
					if (m_pickingMode == POINT_POINT_DISTANCE || m_pickingMode == POINTS_ANGLE)
					{
						glFunc->glBegin(GL_LINES);
					}
					else
					{
						glFunc->glBegin(GL_LINE_LOOP);
					}
				}
				if (m_pickingMode == POINT_POINT_DISTANCE || m_pickingMode == POINTS_ANGLE)
				{
					glFunc->glVertex2d(m_areaPoint2dList[0].x - halfW, m_areaPoint2dList[0].y - halfH);

					for (int ii = 1; ii < areaPointNum; ii++)
					{
						glFunc->glVertex2d(m_areaPoint2dList[ii].x - halfW, m_areaPoint2dList[ii].y - halfH);
						if (ii != (areaPointNum - 1))
						{
							glFunc->glVertex2d(m_areaPoint2dList[ii].x - halfW, m_areaPoint2dList[ii].y - halfH);
						}
					}
				}
				else
				{
					for (unsigned j = 0; j < areaPointNum; ++j)
					{
						glFunc->glVertex2d(m_areaPoint2dList[j].x - halfW, m_areaPoint2dList[j].y - halfH);
					}
				}
				glFunc->glEnd();
				glFunc->glPopAttrib(); //GL_LINE_BIT
			}

			//no need to display the point(s) legend in picking mode
			if (m_dispPointsLegend && !pushName )
			{
				QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
				//font.setPointSize(font.pointSize() + 2);
				font.setBold(true);
				static const QChar ABC[3] = { 'A', 'B', 'C' };

				//draw the label 'legend(s)'
				if (m_pickingMode == POINTS_ANGLE)
				{
					//for (size_t j = 0; j < areaPointNum; j++)
					//{
					//	QString title;
					//	if (areaPointNum == 3)
					//		title = ABC[j]; //for triangle-labels, we only display "A","B","C"

					//	context.display->displayText(title,
					//		static_cast<int>(m_areaPoint2dList[j].x) + context.labelMarkerTextShift_pix,
					//		static_cast<int>(m_areaPoint2dList[j].y) + context.labelMarkerTextShift_pix,
					//		ccGenericGLDisplay::ALIGN_DEFAULT,
					//		context.labelOpacity / 100.0f,
					//		&ccColor::white,
					//		&font);
					//}
				}
				if (count == 1)
				{
					QString title;
					title = getName();
					context.display->displayText(title,
						static_cast<int>(m_pickedPoints[0].pos2D.x) + context.labelMarkerTextShift_pix,
						static_cast<int>(m_pickedPoints[0].pos2D.y) + context.labelMarkerTextShift_pix,
						ccGenericGLDisplay::ALIGN_DEFAULT,
						context.labelOpacity / 100.0f,
						&ccColor::white,
						&font);
				}
			}

			glFunc->glPopAttrib(); //GL_DEPTH_BUFFER_BIT
		}
		else if(m_pickingMode != POINT_VOLUME)
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

	//label title
	const int precision = context.dispNumberPrecision;
	QString title = getTitle(precision, context);

#define DRAW_CONTENT_AS_TAB
#ifdef DRAW_CONTENT_AS_TAB
	//draw contents as an array
	Tab tab(4);
	int rowHeight = 0;
#else
	//simply display the content as text
	QStringList body;
#endif

	//render zoom
	int margin        = static_cast<int>(c_margin        * context.renderZoom);
	int tabMarginX    = static_cast<int>(c_tabMarginX    * context.renderZoom);
	int tabMarginY    = static_cast<int>(c_tabMarginY    * context.renderZoom);
	int arrowBaseSize = static_cast<int>(c_arrowBaseSize * context.renderZoom);

	int titleHeight = 0;
	QFont bodyFont;
	QFont titleFont;
	if (!pushName)
	{
		/*** label border ***/
		bodyFont = context.display->getLabelDisplayFont(); //takes rendering zoom into account!
		titleFont = bodyFont; //takes rendering zoom into account!
		//titleFont.setBold(true);

		QFontMetrics titleFontMetrics(titleFont);
		titleHeight = titleFontMetrics.height();

		QFontMetrics bodyFontMetrics(bodyFont);
		rowHeight = bodyFontMetrics.height();

		//get label box dimension
		int dx = 100;
		int dy = 0;
		QFontMetrics fm(titleFont);
		QRect rect = fm.boundingRect(title);
		//int buttonSize    = static_cast<int>(c_buttonSize * context.renderZoom);
		{
			//base box dimension
			dx = std::max(dx, rect.width());
			dy += margin;		//top vertical margin
			dy += titleHeight;	//title

			if (m_showFullBody)
			{
#ifdef DRAW_CONTENT_AS_TAB
				try
				{
					if (count == 0)
					{
						if (m_pickingMode == POINT_VOLUME)
						{
							double v = m_box->computeVolume();
							if (context.isshowUnitlabel)
							{								
								QString unit = m_volumelist[context.volumnunit];
								title = QString::number(v*m_volumnunitshift[context.volumnunit], 'f', precision) + unit;
							}
							else
							{
								title = QString::number(v*m_volumnunitshift[context.volumnunit], 'f', precision);
							}
						}
					}
					else if (count == 1 && m_pickingMode!= POINT_DENSITY)
					//if (m_pickingMode == POINT_INFO)
					{
						LabelInfo1 info;
						getLabelInfo1(info);

						ccGenericPointCloud* cloud = m_pickedPoints[0].cloudOrVertices();
						assert(cloud);
						bool isShifted = cloud->isShifted();
						CCVector3 P = m_pickedPoints[0].getPointPosition();
						//1st block: X, Y, Z (local)
                        if(m_showShifted)
						{
							int c = tab.add2x3Block();
							QChar suffix;
							if (isShifted)
							{
								suffix = 'l'; //'l' for local
							}

							if (context.isshowUnitlabel)
							{
								QString unit = m_diameterlist[context.lengthunit];
								tab.colContent[c] << QString("X:") ; tab.colContent[c + 1] << QString::number(P.x * m_lengthunitshift[context.lengthunit], 'f', precision)+unit;
								tab.colContent[c] << QString("Y:"); tab.colContent[c + 1] << QString::number(P.y * m_lengthunitshift[context.lengthunit], 'f', precision)+unit;
								tab.colContent[c] << QString("Z:") ; tab.colContent[c + 1] << QString::number(P.z * m_lengthunitshift[context.lengthunit], 'f', precision)+unit;
							}
							else
							{
								tab.colContent[c] << QString("X:"); tab.colContent[c + 1] << QString::number(P.x * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << QString("Y:"); tab.colContent[c + 1] << QString::number(P.y * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << QString("Z:"); tab.colContent[c + 1] << QString::number(P.z * m_lengthunitshift[context.lengthunit], 'f', precision);
							}

						}
						//next block:  X, Y, Z (global)
						if (isShifted && m_showShifted)
						{
							int c = tab.add2x3Block();
							CCVector3d Pd = cloud->toGlobal3d(P);
							if (context.isshowUnitlabel)
							{
								QString unit = m_diameterlist[context.lengthunit];
								tab.colContent[c] << "Xg:"; tab.colContent[c + 1] << QString::number(Pd.x * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
								tab.colContent[c] << "Yg:"; tab.colContent[c + 1] << QString::number(Pd.y * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
								tab.colContent[c] << "Zg:"; tab.colContent[c + 1] << QString::number(Pd.z * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
							}
							else
							{
								tab.colContent[c] << "Xg:"; tab.colContent[c + 1] << QString::number(Pd.x * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << "Yg:"; tab.colContent[c + 1] << QString::number(Pd.y * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << "Zg:"; tab.colContent[c + 1] << QString::number(Pd.z * m_lengthunitshift[context.lengthunit], 'f', precision);
							}

						}
						//next block: normal
						if (info.hasNormal && m_showNormal)
						{
							int c = tab.add2x3Block();
							if (context.isshowUnitlabel)
							{
								QString unit = m_diameterlist[context.lengthunit];
								tab.colContent[c] << "Nx:"; tab.colContent[c + 1] << QString::number(info.normal.x * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
								tab.colContent[c] << "Ny:"; tab.colContent[c + 1] << QString::number(info.normal.y * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
								tab.colContent[c] << "Nz:"; tab.colContent[c + 1] << QString::number(info.normal.z * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
							}
							else
							{
								tab.colContent[c] << "Nx:"; tab.colContent[c + 1] << QString::number(info.normal.x * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << "Ny:"; tab.colContent[c + 1] << QString::number(info.normal.y * m_lengthunitshift[context.lengthunit], 'f', precision);
								tab.colContent[c] << "Nz:"; tab.colContent[c + 1] << QString::number(info.normal.z * m_lengthunitshift[context.lengthunit], 'f', precision);
							}

						}

						//next block: RGB color
						//if (info.hasRGB && m_showRGB)
						//{
						//	int c = tab.add2x3Block();
						//	tab.colContent[c] << "R"; tab.colContent[c + 1] << QString::number(info.color.r);
						//	tab.colContent[c] << "G"; tab.colContent[c + 1] << QString::number(info.color.g);
						//	tab.colContent[c] << "B"; tab.colContent[c + 1] << QString::number(info.color.b);
						//}
					}
					if (m_areaPoint2dList.size() == 2 && m_pickingMode == POINT_HEIGHT)
					{
						LabelInfo2 info;
						getLabelInfo2(info);

						if (context.isshowUnitlabel)
						{
							QString unit = m_diameterlist[context.lengthunit];
							title = MathSymbolDelta + QString("Z:") + QString::number(fabs(info.diff.z) * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
						}
						else
						{
							title = MathSymbolDelta + QString("Z:") + QString::number(fabs(info.diff.z) * m_lengthunitshift[context.lengthunit], 'f', precision);
						}
					}
					if (m_areaPoint2dList.size() == 3 && m_pickingMode == POINTS_ANGLE)
					{
						LabelInfo3 info;
						getLabelInfo3(info);

						int c = tab.add2x3Block();
						if (context.isshowUnitlabel)
						{
							QString unit = m_anglelist[context.angleunit];
							//tab.colContent[c] << "angle.A:"; tab.colContent[c + 1] << QString::number(info.angles.u[0] * m_angleunitshift[context.angleunit], 'f', precision) + unit;
							tab.colContent[c] << "angle:"; tab.colContent[c + 1] << QString::number(info.angles.u[1] * m_angleunitshift[context.angleunit], 'f', precision) + unit;
							//tab.colContent[c] << "angle.C:"; tab.colContent[c + 1] << QString::number(info.angles.u[2] * m_angleunitshift[context.angleunit], 'f', precision) + unit;
						}
						else
						{
							//tab.colContent[c] << "angle.A:"; tab.colContent[c + 1] << QString::number(info.angles.u[0] * m_angleunitshift[context.angleunit], 'f', precision);
							tab.colContent[c] << "angle:"; tab.colContent[c + 1] << QString::number(info.angles.u[1] * m_angleunitshift[context.angleunit], 'f', precision);
							//tab.colContent[c] << "angle.C:"; tab.colContent[c + 1] << QString::number(info.angles.u[2] * m_angleunitshift[context.angleunit], 'f', precision);
						}



						//if (context.isshowUnitlabel)
						//{
						//	QString unit = m_anglelist[context.angleunit];
						//	title = "angle.A" + QString::number(info.angles.u[0] * m_angleunitshift[context.angleunit], 'f', precision) + unit + ",angle.B" + QString::number(info.angles.u[1] * m_angleunitshift[context.angleunit], 'f', precision) + unit + ",angle.C" + QString::number(info.angles.u[2] * m_angleunitshift[context.angleunit], 'f', precision) + unit;
						//}
						//else
						//{
						//	title = "angle.A" + QString::number(info.angles.u[0] * m_angleunitshift[context.angleunit], 'f', precision) + ",angle.B" + QString::number(info.angles.u[1] * m_angleunitshift[context.angleunit], 'f', precision) + ",angle.C" + QString::number(info.angles.u[2] * m_angleunitshift[context.angleunit], 'f', precision);
						//}

					}
					if (m_pickingMode == POINT_POINT_DISTANCE && m_areaPoint2dList.size() >= 2)
					{
						if (context.isshowUnitlabel)
						{
							QString unit = m_diameterlist[context.lengthunit];
							title = QString("distance:") + QString::number(fabs(m_measureResult) * m_lengthunitshift[context.lengthunit], 'f', precision) + unit;
						}
						else
						{
							title = QString("distance:") + QString::number(fabs(m_measureResult) * m_lengthunitshift[context.lengthunit], 'f', precision);
						}
					}
					//[!]密度测量的标签
					if (m_pickingMode == POINT_DENSITY)
					{
						title = QApplication::translate("cc2DLabel", "Density:",nullptr) + QString::number(m_measureResult,'f',3) + "pts/" + u8"m²";
					}
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					assert(!pushName);
					return;
				}
				QFontMetrics fmlater(titleFont);
				QRect rectlater = fm.boundingRect(title);
				dx = std::max(dx, rectlater.width());
				//compute min width of each column
				int totalWidth = tab.updateColumnsWidthTable(bodyFontMetrics);

				int tabWidth = totalWidth + tab.colCount * (2 * tabMarginX); //add inner margins
				dx = std::max(dx, tabWidth);
				dy += tab.rowCount * (rowHeight + 2 * tabMarginY); //add inner margins
				//we also add a margin every 3 rows
				dy += std::max(0, (tab.rowCount / 3) - 1) * margin;
				dy += margin;		//bottom vertical margin
				if (m_areaPoint2dList.size() == 3 && m_pickingMode == POINTS_ANGLE)
				{
					dy -= rowHeight*3;
				}
#else
				body = getLabelContent(precision);
				if (!body.empty())
				{
					dy += margin;	//vertical margin above separator
					for (int j = 0; j < body.size(); ++j)
					{
						dx = std::max(dx, bodyFontMetrics.width(body[j]));
						dy += rowHeight; //body line height
					}
					dy += margin;	//vertical margin below text
				}
#endif //DRAW_CONTENT_AS_TAB
			}

			dx += margin * 2;	// horizontal margins
		}

		//main rectangle
		m_labelROI = QRect(0, 0, dx, dy);

		//close button
		//m_closeButtonROI.right()   = dx-margin;
		//m_closeButtonROI.left()    = m_closeButtonROI.right()-buttonSize;
		//m_closeButtonROI.bottom()  = margin;
		//m_closeButtonROI.top()     = m_closeButtonROI.bottom()+buttonSize;

		//automatically elide the title
		//title = titleFontMetrics.elidedText(title, Qt::ElideRight, m_closeButtonROI[0] - 2 * margin);
	}

	//draw label rectangle
	const int xStart = static_cast<int>(context.glW * m_screenPos[0]);
	const int yStart = static_cast<int>(context.glH * (1.0f - m_screenPos[1]));

	m_lastScreenPos[0] = xStart;
	m_lastScreenPos[1] = yStart - m_labelROI.height();

	//colors
	bool highlighted = (!pushName && isSelected());
	//default background color
	unsigned char alpha = static_cast<unsigned char>((context.labelOpacity / 100.0) * 255);
	ccColor::Rgbaub defaultBkgColor(context.labelDefaultBkgCol, alpha);
	//default border color (mustn't be totally transparent!)
	ccColor::Rgbaub defaultBorderColor(ccColor::red, 255);
	if (!highlighted)
	{
		//apply only half of the transparency
		unsigned char halfAlpha = static_cast<unsigned char>((50.0 + context.labelOpacity / 200.0) * 255);
		defaultBorderColor = ccColor::Rgbaub(context.labelDefaultBkgCol, halfAlpha);
	}

	glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
	glFunc->glEnable(GL_BLEND);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glTranslatef(static_cast<GLfloat>(xStart - halfW), static_cast<GLfloat>(yStart - halfH), 0);

	if (!pushName)
	{
		//compute arrow base position relatively to the label rectangle (for 0 to 8)
		int arrowBaseConfig = 0;

		//compute arrow head position
		CCVector3d arrowDest2D(0, 0, 0);
		if (m_pickingMode == POINT_VOLUME)
		{
			if (m_box)
			{
				arrowDest2D = m_box->pos2D;
			}
		}
		else
		{
			for (size_t i = 0; i < count; ++i)
			{
				arrowDest2D += m_pickedPoints[i].pos2D;
			}
			arrowDest2D /= static_cast<PointCoordinateType>(count);
		}
		if ((m_pickingMode == POINT_POINT_DISTANCE || m_pickingMode == POINT_AERA) && areaPointNum >1)
		{
			arrowDest2D = (m_areaPoint2dList[areaPointNum - 1] + m_areaPoint2dList[areaPointNum - 2])/ static_cast<PointCoordinateType>(2);
		}

		if (m_pickingMode == POINTS_ANGLE && areaPointNum == 3)
		{
			arrowDest2D = m_areaPoint2dList[1];

			//for (size_t i = 0; i < areaPointNum; ++i)
			//{
			//	arrowDest2D += m_areaPoint2dList[i];
			//}
			//arrowDest2D /= static_cast<PointCoordinateType>(areaPointNum);
			//arrowDest2D = (m_areaPoint2dList[areaPointNum - 1] + m_areaPoint2dList[areaPointNum - 2]) / static_cast<PointCoordinateType>(2);
		}

		if (m_pickingMode == POINT_HEIGHT)
		{
			arrowDest2D = m_HeightMeasureTextPos;
		}
		
		//arrowDest2D.x -= halfW;
		//arrowDest2D.y -= halfH;

		int iArrowDestX = static_cast<int>(arrowDest2D.x - xStart);
		int iArrowDestY = static_cast<int>(arrowDest2D.y - yStart);
		{
			if (iArrowDestX < m_labelROI.left()) //left
				arrowBaseConfig += 0;
			else if (iArrowDestX > m_labelROI.right()) //Right
				arrowBaseConfig += 2;
			else  //Middle
				arrowBaseConfig += 1;

			if (iArrowDestY > -m_labelROI.top()) //Top
				arrowBaseConfig += 0;
			else if (iArrowDestY < -m_labelROI.bottom()) //Bottom
				arrowBaseConfig += 6;
			else  //Middle
				arrowBaseConfig += 3;
		}

		//we make the arrow base start from the nearest corner
		if (arrowBaseConfig != 4) //4 = label above point!
		{
			glFunc->glColor4ubv(defaultBorderColor.rgba);
			glFunc->glBegin(GL_LINE_LOOP);
            glFunc->glLineWidth(2.0f );
			glFunc->glVertex2i(iArrowDestX, iArrowDestY);
			switch (arrowBaseConfig)
			{
			case 0: //top-left corner
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top() - 2 * arrowBaseSize);
				//glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
				//glFunc->glVertex2i(m_labelROI.left() + 2 * arrowBaseSize, -m_labelROI.top());
				break;
			case 1: //top-middle edge
				glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), -m_labelROI.top());
				//glFunc->glVertex2i(std::min(m_labelROI.right(), iArrowDestX + arrowBaseSize), -m_labelROI.top());
				break;
			case 2: //top-right corner
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top() - 2 * arrowBaseSize);
				//glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.top());
				//glFunc->glVertex2i(m_labelROI.right() - 2 * arrowBaseSize, -m_labelROI.top());
				break;
			case 3: //middle-left edge
				glFunc->glVertex2i(m_labelROI.left(), std::min(-m_labelROI.top(), iArrowDestY + arrowBaseSize));
				//glFunc->glVertex2i(m_labelROI.left(), std::max(-m_labelROI.bottom(), iArrowDestY - arrowBaseSize));
				break;
			case 4: //middle of rectangle!
				break;
			case 5: //middle-right edge
				glFunc->glVertex2i(m_labelROI.right(), std::min(-m_labelROI.top(), iArrowDestY + arrowBaseSize));
				//glFunc->glVertex2i(m_labelROI.right(), std::max(-m_labelROI.bottom(), iArrowDestY - arrowBaseSize));
				break;
			case 6: //bottom-left corner
				glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom() + 2 * arrowBaseSize);
				//glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
				//glFunc->glVertex2i(m_labelROI.left() + 2 * arrowBaseSize, -m_labelROI.bottom());
				break;
			case 7: //bottom-middle edge
				glFunc->glVertex2i(std::max(m_labelROI.left(), iArrowDestX - arrowBaseSize), -m_labelROI.bottom());
				//glFunc->glVertex2i(std::min(m_labelROI.right(), iArrowDestX + arrowBaseSize), -m_labelROI.bottom());
				break;
			case 8: //bottom-right corner
				glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom() + 2 * arrowBaseSize);
				//glFunc->glVertex2i(m_labelROI.right(), -m_labelROI.bottom());
				//glFunc->glVertex2i(m_labelROI.right() - 2 * arrowBaseSize, -m_labelROI.bottom());
				break;
			}
			glFunc->glEnd();
		}
	}
	//main rectangle
	glFunc->glColor4ubv(defaultBkgColor.rgba);
	glFunc->glBegin(GL_QUADS);
	glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
	glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right() + 10, -m_labelROI.bottom());
	glFunc->glVertex2i(m_labelROI.right() + 10, -m_labelROI.top());
	glFunc->glEnd();
	//if (highlighted)
	{


		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(3.0f * context.renderZoom);
		glFunc->glColor4ubv(defaultBorderColor.rgba);
		glFunc->glBegin(GL_LINE_LOOP);
		glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.top());
		glFunc->glVertex2i(m_labelROI.left(), -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right() + 10, -m_labelROI.bottom());
		glFunc->glVertex2i(m_labelROI.right() + 10, -m_labelROI.top());
		glFunc->glEnd();
		glFunc->glPopAttrib(); //GL_LINE_BIT
	}

	//draw close button
	//glFunc->glColor4ubv(ccColor::black.rgba);
	//glFunc->glBegin(GL_LINE_LOOP);
	//glFunc->glVertex2i(m_closeButtonROI.left(),-m_closeButtonROI.top());
	//glFunc->glVertex2i(m_closeButtonROI.left(),-m_closeButtonROI.bottom());
	//glFunc->glVertex2i(m_closeButtonROI.right(),-m_closeButtonROI.bottom());
	//glFunc->glVertex2i(m_closeButtonROI.right(),-m_closeButtonROI.top());
	//glFunc->glEnd();
	//glFunc->glBegin(GL_LINES);
	//glFunc->glVertex2i(m_closeButtonROI.left()+2,-m_closeButtonROI.top()+2);
	//glFunc->glVertex2i(m_closeButtonROI.right()-2,-m_closeButtonROI.bottom()-2);
	//glFunc->glVertex2i(m_closeButtonROI.right()-2,-m_closeButtonROI.top()+2);
	//glFunc->glVertex2i(m_closeButtonROI.left()+2,-m_closeButtonROI.bottom()-2);
	//glFunc->glEnd();

	//display text
	if (!pushName)
	{
		int xStartRel = margin;
		int yStartRel = 0;
		yStartRel -= titleHeight;

		ccColor::Rgba defaultTextColor;
		if (context.labelOpacity < 40)
		{
			//under a given opacity level, we use the default text color instead!
			defaultTextColor = context.textDefaultCol;
		}
		else
		{
			defaultTextColor = ccColor::Rgba(	255 - context.labelDefaultBkgCol.r,
												255 - context.labelDefaultBkgCol.g,
												255 - context.labelDefaultBkgCol.b,
												context.labelDefaultBkgCol.a);
		}

		//label title
		context.display->displayText(title,
			xStart + xStartRel,
			yStart + yStartRel,
			ccGenericGLDisplay::ALIGN_DEFAULT,
			0,
			&defaultTextColor,
			&titleFont);
		yStartRel -= margin;

		if (m_showFullBody)
		{
#ifdef DRAW_CONTENT_AS_TAB
			int xCol = xStartRel;
			for (int c = 0; c < tab.colCount; ++c)
			{
				int width = tab.colWidth[c] + 2 * tabMarginX;
				int height = rowHeight + 2 * tabMarginY;

				int yRow = yStartRel;
				int actualRowCount = std::min(tab.rowCount, tab.colContent[c].size());

				bool labelCol = ((c & 1) == 0);
				const ccColor::Rgba* textColor = labelCol ? &ccColor::white : &defaultTextColor;

				for (int r = 0; r < actualRowCount; ++r)
				{
					if (r && (r % 3) == 0)
						yRow -= margin;

					if (labelCol)
					{
						//draw background
						int rgbIndex = (r % 3);
						if (rgbIndex == 0)
							glFunc->glColor4ubv(ccColor::red.rgba);
						else if (rgbIndex == 1)
							glFunc->glColor4ubv(c_darkGreen.rgba);
						else if (rgbIndex == 2)
							glFunc->glColor4ubv(ccColor::blue.rgba);

						//glFunc->glBegin(GL_QUADS);
						//if (m_areaPoint2dList.size() == 3 && m_pickingMode == POINTS_ANGLE)
						//{
						//	glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow + height- margin);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow - height + height- margin);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow - height + height- margin);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow + height- margin);
						//}
						//else
						//{
						//	glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol, -m_labelROI.top() + yRow - height);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow - height);
						//	glFunc->glVertex2i(m_labelROI.left() + xCol + width, -m_labelROI.top() + yRow);
						//}
						//glFunc->glEnd();
					}

					const QString& str = tab.colContent[c][r];

					int xShift = 0;
					if (labelCol)
					{
						//align characters in the middle
						xShift = (tab.colWidth[c] - QFontMetrics(bodyFont).width(str)) / 2;
					}
					else
					{
						//align digits on the right
						xShift = tab.colWidth[c] - QFontMetrics(bodyFont).width(str);
					}
					if (m_areaPoint2dList.size() == 3 && m_pickingMode == POINTS_ANGLE)
					{
						context.display->displayText(str,
							xStart + xCol + tabMarginX + xShift,
							yStart + yRow - rowHeight + rowHeight, ccGenericGLDisplay::ALIGN_DEFAULT, 0, textColor, &bodyFont);
					}
					else
					{
						context.display->displayText(str,
							xStart + xCol + tabMarginX + xShift,
							yStart + yRow - rowHeight, ccGenericGLDisplay::ALIGN_DEFAULT, 0, textColor, &bodyFont);
					}

					yRow -= height;
				}

				xCol += width;
			}
#else
			if (!body.empty())
			{
				//display body
				yStartRel -= margin;
				for (int i = 0; i < body.size(); ++i)
				{
					yStartRel -= rowHeight;
					context.display->displayText(body[i], xStart + xStartRel, yStart + yStartRel, ccGenericGLDisplay::ALIGN_DEFAULT, 0, defaultTextColor.rgb, &bodyFont);
				}
			}
#endif //DRAW_CONTENT_AS_TAB
		}
	}

	glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT

	glFunc->glPopMatrix();

	if (pushName)
	{
		glFunc->glPopName();
	}
}

bool cc2DLabel::pointPicking(	const CCVector2d& clickPos,
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

bool cc2DLabel::isAreaAndMoreThanThreePoint()
{
	if (m_pickingMode == POINT_AERA && m_areaPointList.size() >= 3)
	{
		return true;
	}
	return false;
}