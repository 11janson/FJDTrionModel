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

#include "FileIOFilter.h"
#include "QSettings"
#include "qdebug.h"
//CLOUDS
#include "AsciiFilter.h"
#include "BinFilter.h"

//#include "PcdFilter.h"//2022.3.30janson

//MESHES
#include "PlyFilter.h"

//OTHERS
#include "DepthMapFileFilter.h"
#include "DxfFilter.h"
#include "ImageFileFilter.h"
#include "RasterGridFilter.h"
#include "ShpFilter.h"

#include "ccPointCloud.h"

//Qt
#include <QFileInfo>
#include "cloudcompareutils/icore.h"

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//system
#include <cassert>
#include <vector>
#include "ccPointCloud.h"
#include "ccHObjectCaster.h"
#include "ccColorScalesManager.h"
#include "ccScalarField.h"

//! Available filters
/** Filters are uniquely recognized by their 'file filter' string.
	We use a std::vector so as to keep the insertion ordering!
**/
static FileIOFilter::FilterContainer s_ioFilters;

static unsigned s_sessionCounter = 0;

// This extra definition is required in C++11.
// In C++17, class-level "static constexpr" is implicitly inline, so these are not required.
constexpr float FileIOFilter::DEFAULT_PRIORITY;

FileIOFilter::FileIOFilter( const FileIOFilter::FilterInfo &info ) :
	m_filterInfo( info )
{
#ifdef QT_DEBUG	
	if ( !(m_filterInfo.features & DynamicInfo) )
	{
		checkFilterInfo();
	}
#endif
}

bool FileIOFilter::importSupported() const
{
	return m_filterInfo.features & Import;
}

bool FileIOFilter::exportSupported() const
{
	return m_filterInfo.features & Export;
}

const QStringList& FileIOFilter::getFileFilters( bool onImport ) const
{
	if ( onImport )
	{
		return m_filterInfo.importFileFilterStrings;
	}
	
	return m_filterInfo.exportFileFilterStrings;
}

QString FileIOFilter::getDefaultExtension() const
{
	return m_filterInfo.defaultExtension;
}

void FileIOFilter::setImportExtensions( const QStringList &extensions )
{
	m_filterInfo.importExtensions = extensions;
}

void FileIOFilter::setImportFileFilterStrings( const QStringList &filterStrings )
{
	m_filterInfo.importFileFilterStrings = filterStrings;
}

void FileIOFilter::setExportFileFilterStrings(const QStringList &filterStrings)
{
	m_filterInfo.exportFileFilterStrings = filterStrings;
}

void FileIOFilter::checkFilterInfo() const
{
#ifdef QT_DEBUG
	// Check info for consistency
	if ( m_filterInfo.features & Import )
	{
		if ( m_filterInfo.importFileFilterStrings.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter marked as import, but no filter strings set: %1" ).arg( m_filterInfo.id ) );
		}
		
		if ( m_filterInfo.importExtensions.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter marked as import, but no extensions set: %1" ).arg( m_filterInfo.id ) );
		}
	}
	else
	{
		if ( !m_filterInfo.importFileFilterStrings.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter not marked as import, but filter strings are set: %1" ).arg( m_filterInfo.id ) );
		}
		
		if ( !m_filterInfo.importExtensions.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter not marked as import, but extensions are set: %1" ).arg( m_filterInfo.id ) );
		}
	}
	
	if ( m_filterInfo.features & Export )
	{
		if ( m_filterInfo.exportFileFilterStrings.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter marked as export, but no filter strings set: %1" ).arg( m_filterInfo.id ) );
		}
	}
	else
	{
		if ( !m_filterInfo.exportFileFilterStrings.isEmpty() )
		{
			qWarning()<<( QStringLiteral( "I/O filter not marked as export, but filter strings are set: %1" ).arg( m_filterInfo.id ) );
		}		
	}
#endif
}

void FileIOFilter::ResetSesionCounter()
{
	s_sessionCounter = 0;
}

unsigned FileIOFilter::IncreaseSesionCounter()
{
	return ++s_sessionCounter;
}

void FileIOFilter::InitInternalFilters()
{
	//from the most useful to the less one!
	Register(Shared(new BinFilter()));
	Register(Shared(new AsciiFilter()));

	Register(Shared(new PlyFilter()));

#ifdef CC_DXF_SUPPORT
	Register(Shared(new DxfFilter()));
#endif
//屏蔽未规划格式
//#ifdef CC_SHP_SUPPORT
//	Register(Shared(new ShpFilter()));
//#endif
#ifdef CC_GDAL_SUPPORT
	Register(Shared(new RasterGridFilter()));
#endif
	//Register(Shared(new ImageFileFilter()));
	//Register(Shared(new DepthMapFileFilter()));

	//2022.3.30janson
	//Register(Shared(new PcdFilter()));

}

void FileIOFilter::Register(Shared filter)
{
	if (!filter)
	{
		assert(false);
		return;
	}

	// check for an existing copy of this filter or one with the same ID
	const QString id = filter->m_filterInfo.id;

	auto compareFilters = [filter, id] ( const Shared& filter2 )
	{
		return (filter == filter2) || (filter2->m_filterInfo.id == id);
	};
	
	if ( std::any_of( s_ioFilters.cbegin(), s_ioFilters.cend(), compareFilters ) ) 
	{
		qWarning()<<( QStringLiteral( "[FileIOFilter] I/O filter already registered with id '%1'" ).arg( id ) );
		
		return;
	}
	
	// insert into the list, sorted by priority first, id second
	auto comparePriorities = [] ( const Shared& filter1, const Shared& filter2 ) -> bool
	{
		if ( filter1->m_filterInfo.priority == filter2->m_filterInfo.priority )
		{
			return filter1->m_filterInfo.id < filter2->m_filterInfo.id;
		}
		
		return filter1->m_filterInfo.priority < filter2->m_filterInfo.priority;
	};
	
	auto pos = std::upper_bound( s_ioFilters.begin(), s_ioFilters.end(), filter, comparePriorities );
	
	s_ioFilters.insert( pos, filter );
}

void FileIOFilter::UnregisterAll()
{
	for (auto & filter : s_ioFilters)
	{
		filter->unregister();
	}
	
	s_ioFilters.clear();
}

FileIOFilter::Shared FileIOFilter::GetFilter(const QString& fileFilter, bool onImport)
{
	if (!fileFilter.isEmpty())
	{
		for (FilterContainer::const_iterator it=s_ioFilters.begin(); it!=s_ioFilters.end(); ++it)
		{
			QStringList otherFilters = (*it)->getFileFilters(onImport);
			if (otherFilters.contains(fileFilter))
				return *it;
		}
	}

	return Shared(nullptr);
}

const FileIOFilter::FilterContainer& FileIOFilter::GetFilters()
{
	return s_ioFilters;
}

FileIOFilter::Shared FileIOFilter::FindBestFilterForExtension(const QString& ext)
{
	const QString lowerExt = ext.toLower();
	
	for ( const auto &filter : s_ioFilters )
	{
		if ( filter->m_filterInfo.importExtensions.contains( lowerExt ) )
		{
			return filter;
		}
	}

	return FileIOFilter::Shared( nullptr );
}

FileIOFilter::Shared FileIOFilter::getCADDXFFilter(const QString& ext)
{
	const QString lowerExt = ext.toLower();

	for (const auto &filter : s_ioFilters)
	{
		if (filter->m_filterInfo.id.compare("CAD_DXF Filter") == 0)
		{
			return filter;
		}
	}

	return FileIOFilter::Shared(nullptr);
}

QStringList FileIOFilter::ImportFilterList()
{
	QStringList	list{ QObject::tr( "All (*.*)" ) };
	
	for ( const auto &filter : s_ioFilters )
	{
		if ( filter->importSupported() )
		{
			list += filter->m_filterInfo.importFileFilterStrings;
		}
	}	
	list << "geotiff(*.tiff *.tif)";
	return list;
}

ccHObject* FileIOFilter::LoadFromFile(	const QString& filename,
										LoadParameters& loadParameters,
										Shared filter,
										CC_FILE_ERROR& result)
{
	if (!filter)
	{
		ccLog::Error(QString("[Load] Internal error (invalid input filter)").arg(filename));
		result = CC_FERR_CONSOLE_ERROR;
		assert(false);
		return nullptr;
	}
    ccHObject* container; 
    try
    {
        
        //check file existence
        QFileInfo fi(filename);
        if (!fi.exists())
        {
	        ccLog::Error(QString("[Load] File '%1' doesn't exist!").arg(filename));
	        result = CC_FERR_CONSOLE_ERROR; 
	        return nullptr;
        }

        //load file
        container = new ccHObject();
        result = CC_FERR_NO_ERROR;
	
        //we start a new 'action' inside the current sessions
        unsigned sessionCounter = IncreaseSesionCounter();
        loadParameters.sessionStart = (sessionCounter == 1);

        if (filter->m_filterInfo.id.compare("CAD_DXF Filter") == 0)
        {
	        //保证编辑后再导出为CAD DXF
	        delete container;
	        container = nullptr;
	        container = new ccPointCloud();
        }

        result = filter->loadFile(filename,
            *container,
            loadParameters);
        if (result == CC_FERR_NO_ERROR)
        {
            qDebug() << (QString("[I/O] File '%1' loaded successfully").arg(filename));
        }
        if (result == CC_FERR_TYPE_CAD_DXF)
        {
            return nullptr;
        }
        else
        {
            DisplayErrorMessage(result, "loading", fi.baseName());
        }

		////[!].判断是否为CAD dxf 节点的二维矢量模块
		if (container && container->getMetaData("is2dSurface").toBool()) {
			return container;
		}

        //void* ptr = operator new(std::numeric_limits<std::size_t>::max());
        unsigned childCount = container->getChildrenNumber();
        if (childCount != 0)
        {
            //we set the main container name as the full filename (with path)
            container->setName(QString("%1").arg(fi.fileName()));
            for (unsigned i = 0; i < childCount; ++i)
            {
                ccHObject* child = container->getChild(i);
                QString newName = child->getName();
                if (newName.startsWith("unnamed"))
                {
                    //we automatically replace occurrences of 'unnamed' in entities names by the base filename (no path, no extension)
                    newName.replace(QString("unnamed"), fi.completeBaseName());
                    child->setName(newName);
                }
                else if (newName.isEmpty())
                {
                    //just in case
                    child->setName(fi.completeBaseName());
                }

				if (child->hasColors()){
					child->showColors(true);
					child->showSF(false);
				}
				else if (child->hasScalarFields()){
					child->showSF(true);
				}
                ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(child);
                if (cloud)
                {
                    if (cloud->isShifted())
                    {
                        QSettings shiftsettings(CS::Core::ICore::getDefaultPath() + "/config/config.ini", QSettings::IniFormat);
                        shiftsettings.beginGroup("EntityPointShift");
                        shiftsettings.setValue("x", cloud->getGlobalShift().x);
                        shiftsettings.setValue("y", cloud->getGlobalShift().y);
                        shiftsettings.setValue("z", cloud->getGlobalShift().z);
                        shiftsettings.setValue("scale", cloud->getGlobalScale());
                        shiftsettings.endGroup();
                    }
                    cloud->adjustGlobalShift();
                    int nsf = cloud->getNumberOfScalarFields();
                    for (int i = 0; i < nsf; ++i)
                    {
                        QString sfName = cloud->getScalarFieldName(i);
                        if (sfName.compare("intensity", Qt::CaseInsensitive) == 0)
                        {
                            ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
                            if (sf)
                            {
                                sf->setColorScale(ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::GREY));
                            }
                        }
                        if (sfName.compare("time", Qt::CaseInsensitive) == 0)
                        {
                            cloud->setScalarFieldName(i, "GpsTime");
                        }
                    }
                }
            }
        }
        else
        {
            delete container;
            container = nullptr;
        }
    }
    catch (const std::exception& ex) {
        qCritical() << "Caught exception of type: " << QString(typeid(ex).name());
        qCritical() << "Exception details: " << QString(ex.what());
        if (container)
        {
            container->removeAllChildren();
            delete container;
            container = nullptr;
        }
        result = CC_FERR_NOT_ENOUGH_MEMORY;
    }
    catch (...) {
        qCritical() << "Caught an unknown exception";
        if (container)
        {
            container->removeAllChildren();
            delete container;
            container = nullptr;
        }
        result = CC_FERR_CONSOLE_ERROR;
    }
    return container;
}

ccHObject* FileIOFilter::LoadFromFile(	const QString& filename,
										LoadParameters& loadParameters,
										CC_FILE_ERROR& result,
										const QString& fileFilter )
{
	Shared filter(nullptr);
	
	//if the right filter is specified by the caller
	if (!fileFilter.isEmpty())
	{
		filter = GetFilter(fileFilter, true);
		if (!filter)
		{
			ccLog::Error(QString("[Load] Internal error: no I/O filter corresponds to filter '%1'").arg(fileFilter));
			result = CC_FERR_CONSOLE_ERROR;
			return nullptr;
		}
	}
	else //we need to guess the I/O filter based on the file format
	{
		//look for file extension (we trust Qt on this task)
		QString extension = QFileInfo(filename).suffix();
		if (extension.isEmpty())
		{
			ccLog::Error("[Load] Can't guess file format: no file extension");
			result = CC_FERR_CONSOLE_ERROR;
			return nullptr;
		}

		//convert extension to file format
		filter = FindBestFilterForExtension(extension);

		//unknown extension?
		if (!filter)
		{
			ccLog::Error(QString("[Load] Can't guess file format: unhandled file extension '%1'").arg(extension));
			result = CC_FERR_CONSOLE_ERROR;
			return nullptr;
		}
	}
	ccHObject* pObj = nullptr;
    //区分cad dxf和geometry dxf
    if (QFileInfo(filename).suffix().compare("dxf") == 0)
    {
        pObj = LoadFromFile(filename, loadParameters, filter, result);
        if (pObj)
        {
            return pObj;
        }
        else
        {
            QString extension = QFileInfo(filename).suffix();
            filter = getCADDXFFilter(extension);
            pObj = LoadFromFile(filename, loadParameters, filter, result);
        }
    }
    else
    {
        pObj = LoadFromFile(filename, loadParameters, filter, result);
    }

	return pObj;
}

CC_FILE_ERROR FileIOFilter::SaveToFile(	ccHObject* entities,
										const QString& filename,
										const SaveParameters& parameters,
										Shared filter)
{
	if (!entities || filename.isEmpty() || !filter)
		return CC_FERR_BAD_ARGUMENT;

	//if the file name has no extension, we had a default one!
	QString completeFileName(filename);
	if (QFileInfo(filename).suffix().isEmpty())
	{
		completeFileName += QString(".%1").arg(filter->getDefaultExtension());
	}
	
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	try
	{
		result = filter->saveToFile(entities, completeFileName, parameters);
	}
	catch(...)
	{
		qWarning()<<(QString("[I/O] CC has caught an unhandled exception while saving file '%1'").arg(filename));
		result = CC_FERR_CONSOLE_ERROR;
	}

	if (result == CC_FERR_NO_ERROR)
	{
		ccLog::Print(QString("[I/O] File '%1' saved successfully").arg(filename));
	}
	else
	{
		DisplayErrorMessage(result,"saving",filename);
	}

	return result;
}

CC_FILE_ERROR FileIOFilter::SaveToFile(	ccHObject* entities,
										const QString& filename,
										const SaveParameters& parameters,
										const QString& fileFilter)
{
	if (fileFilter.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	Shared filter = GetFilter(fileFilter,false);
	if (!filter)
	{
		ccLog::Error(QString("[Load] Internal error: no filter corresponds to filter '%1'").arg(fileFilter));
		return CC_FERR_UNKNOWN_FILE;
	}

	return SaveToFile(entities, filename, parameters, filter);
}

void FileIOFilter::DisplayErrorMessage(CC_FILE_ERROR err, const QString& action, const QString& filename)
{
	QString errorStr;

	bool warning = false;
	switch(err)
	{
	case CC_FERR_NO_ERROR:
		return; //no message will be displayed!
	case CC_FERR_BAD_ARGUMENT:
		errorStr = "bad argument (internal)";
		break;
	case CC_FERR_UNKNOWN_FILE:
		errorStr = "unknown file";
		break;
	case CC_FERR_WRONG_FILE_TYPE:
		errorStr = "wrong file type (check header)";
		break;
	case CC_FERR_WRITING:
		errorStr = "writing error (disk full/no access right?)";
		break;
	case CC_FERR_READING:
		errorStr = "reading error (no access right?)";
		break;
	case CC_FERR_NO_SAVE:
		errorStr = "nothing to save";
		break;
	case CC_FERR_NO_LOAD:
		errorStr = "nothing to load";
		break;
	case CC_FERR_BAD_ENTITY_TYPE:
		errorStr = "incompatible entity/file types";
		break;
	case CC_FERR_CANCELED_BY_USER:
		errorStr = "process canceled by user";
		warning = true;
		break;
	case CC_FERR_NOT_ENOUGH_MEMORY:
		errorStr = "not enough memory";
		break;
	case CC_FERR_MALFORMED_FILE:
		errorStr = "malformed file";
		break;
	case CC_FERR_BROKEN_DEPENDENCY_ERROR:
		errorStr = "dependent entities missing (see Console)";
		break;
	case CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN:
		errorStr = "the file was written by a plugin but none of the loaded plugins can deserialize it";
		break;
	case CC_FERR_THIRD_PARTY_LIB_FAILURE:
		errorStr = "the third-party library in charge of saving/loading the file has failed to perform the operation";
		break;
	case CC_FERR_THIRD_PARTY_LIB_EXCEPTION:
		errorStr = "the third-party library in charge of saving/loading the file has thrown an exception";
		break;
	case CC_FERR_NOT_IMPLEMENTED:
		errorStr = "this function is not implemented yet!";
		break;
	case CC_FERR_CONSOLE_ERROR:
		errorStr = "see console";
		break;
	default:
		return; //no message will be displayed!
	}

	QString outputString = QString("An error occurred while %1 '%2': ").arg(action,filename) + errorStr;
	if (warning)
		qWarning()<<(outputString);
	else
		ccLog::Error(outputString);
}

bool FileIOFilter::CheckForSpecialChars(const QString& filename)
{
	return (filename.normalized(QString::NormalizationForm_D) != filename);
}

bool FileIOFilter::HandleGlobalShift(	const CCVector3d& P,
										CCVector3d& Pshift,
										bool& preserveCoordinateShift,
										LoadParameters& loadParameters,
										bool useInputCoordinatesShiftIfPossible/*=false*/)
{
	bool shiftAlreadyEnabled = (	(nullptr != loadParameters._coordinatesShiftEnabled)
								&&	(*loadParameters._coordinatesShiftEnabled)
								&&	(nullptr != loadParameters._coordinatesShift) );
	if (shiftAlreadyEnabled)
	{
		Pshift = *loadParameters._coordinatesShift;
		preserveCoordinateShift = loadParameters.preserveShiftOnSave;
	}
	
	bool applyAll = false;
	if (	sizeof(PointCoordinateType) < 8
		&&	ccGlobalShiftManager::Handle(	P,
											0,
											loadParameters.shiftHandlingMode,
											shiftAlreadyEnabled || useInputCoordinatesShiftIfPossible,
											Pshift,
											&preserveCoordinateShift,
											nullptr,
											&applyAll) )
	{
		// we save coordinates shift information
		if (applyAll || loadParameters.shiftHandlingMode == ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT) //in command line mode, with the 'AUTO GLOBAL SHIFT' option, we want to retrieve the applied coordinate shift
		{
			if (nullptr != loadParameters._coordinatesShiftEnabled)
			{
				*loadParameters._coordinatesShiftEnabled = true;
			}
			if (nullptr != loadParameters._coordinatesShift)
			{
				*loadParameters._coordinatesShift = Pshift;
			}
			loadParameters.preserveShiftOnSave = preserveCoordinateShift;
		}

		return true;
	}

	return false;
}
