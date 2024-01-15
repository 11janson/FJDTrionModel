
cmake_minimum_required( VERSION 3.10 )
#软件名称
SET(IDE_TARGET_NAME FJDTrionModel)
add_compile_definitions(IDE_TARGET_NAME="${IDE_TARGET_NAME}")

project(${IDE_TARGET_NAME})

# One shouldn't generate the BUILD project directly in the SOURCES folder!
if ( ${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
	if ( NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED )
		message(FATAL_ERROR "It is not advised to BUILD the binaries directly in the SOURCE folder!\n If you want to proceed with this option, just CONFIGURE the project once again" )
		set( SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE )
	endif()
endif()

# Add our cmake module path so we don't need relative paths for these
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" )

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:10240000")   # april.qiu

################################配置路径######################
MESSAGE("config thirdparty and lib plugins home path")
SET(LIBS_HOME ${CMAKE_CURRENT_SOURCE_DIR}/libs)
SET(PLUGINS_HOME ${CMAKE_CURRENT_SOURCE_DIR}/plugins)
SET(CODE_HOME ${CMAKE_CURRENT_SOURCE_DIR})


include(${CMAKE_CURRENT_LIST_DIR}/config.cmake)
include( CMakePolicies )
include( CMakeSetCompilerOptions )
include( DeployQt )

SET(TRANSLATE_PATH ${CMAKE_BINARY_DIR}/bin/Release)
include(${THIRDPARTY_CMAKE}/csconfig.cmake)
include(${THIRDPARTY_CMAKE}/csutil.cmake)
include(${THIRDPARTY_CMAKE}/csoutputconfig.cmake)

set(CMAKE_INSTALL_PREFIX ${BIN_OUTPUT_PATH} CACHE PATH "default install prefix" FORCE)
include(${THIRDPARTY_CMAKE}/csfindqt.cmake)

SET(CS_LIBRARY_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

# 是否启动在线更新
OPTION(ONLINE_UPDATE  "The Trion run is Online Update" OFF)

# 是否启动在线更新
IF(ONLINE_UPDATE)
    ADD_DEFINITIONS(-DMETAHUBTRION_FOR_ONLINEUPDATE)
	MESSAGE(WARNING "Online update has been activated")
	#TrionModelStart
	add_subdirectory( TrionModelStart )
ENDIF()

# 是否启动崩溃回传功能
OPTION(CrashBack  "The Trion run is CrashBack" OFF)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG:FULL /OPT:REF,ICF")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG:FULL /OPT:REF,ICF")
# 是否启动崩溃回传功能
IF(CrashBack)
 	add_compile_definitions(StartCrashApplication=1)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG:FULL /OPT:REF,ICF")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG:FULL /OPT:REF,ICF")
	MESSAGE(WARNING "CrashBack has been activated")
	#CrashBack
	add_subdirectory( CrashBack )
else()
	add_compile_definitions(StartCrashApplication=0)
ENDIF()
# 崩溃回传运行环境
OPTION(CrashbackRunningEnvironment  "Check as formal environment" OFF)

IF(CrashbackRunningEnvironment)
    add_compile_definitions(CrashApplication=1)
	add_compile_definitions(CrashApplicationInsert=1)
    add_compile_definitions(CrashApplicationBucketName=1)
else()
    add_compile_definitions(CrashApplication=0)
	add_compile_definitions(CrashApplicationInsert=0)
    add_compile_definitions(CrashApplicationBucketName=0)
ENDIF()
## 全局Include路径
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
    ${CMAKE_CURRENT_SOURCE_DIR}/qCC
    ${VCPKG_HOME}
    ${CMAKE_BINARY_DIR}
	${THIRDPARTY_ROOT}
)


# 是否启动加密
OPTION(SENTINElLDK_DONGLE  "The Trion run is dongle" ON)
# 是否启动加密
IF(SENTINElLDK_DONGLE)
    ADD_DEFINITIONS(-DMETAHUBTRION_FOR_DONGLE)
	MESSAGE(WARNING "Trion sentinelLDK  encryption has been activated")
ENDIF()

#开启交通警察线程勘察
OPTION(TRAFFICSITESURVEY  "The trion trafficsitesurvey is enable" OFF)
IF(TRION_TRAFFICSITESURVEY)
    ADD_DEFINITIONS(-DTRION_TRAFFICSITESURVEY)
	MESSAGE(WARNING "Turn on the software to trafficsitesurvey by module")
ENDIF()

#检查是否开启按模块激活激活
OPTION(REGISTER_ACTIVATIONMODULE  "Turn on the software to register activation by module" ON)
IF(REGISTER_ACTIVATIONMODULE)
    ADD_DEFINITIONS(-DREGISTER_ACTIVATIONMODULE)
	MESSAGE(WARNING "Turn on the software to register activation by module")
ENDIF()


#修改日志输出类型
ADD_DEFINITIONS(-DQT_MESSAGELOGCONTEXT)
ADD_DEFINITIONS(-DSPDLOG_WCHAR_FILENAMES)
## google breakpad 的根目录
IF(WIN32) ### 暂支持Win
    IF(MSVC)
        SET(GBREAKPAD_ROOT ${THIRDPARTY_ROOT}/google_breakpad CACHE PATH  "google breakpad root path ")
        IF (NOT (EXISTS "${GBREAKPAD_ROOT}"))    
            MESSAGE(STATUS "google breakpad root path not found in ${GBREAKPAD_ROOT}")
        ENDIF()
    ENDIF()
ENDIF()



# CCViewer
option( OPTION_BUILD_CCVIEWER "Check to compile CCViewer project" OFF )

# Quad buffer stereo support
option( OPTION_GL_QUAD_BUFFER_SUPPORT "Check to compile CloudCompare and ccViewer with Quad Buffer support" OFF )

if ( OPTION_GL_QUAD_BUFFER_SUPPORT )
    # Add the define for all libs and applications
	add_definitions( -DCC_GL_WINDOW_USE_QWINDOW )
endif()

# Testing
option( BUILD_TESTING "Build tests for CC" OFF )
if ( BUILD_TESTING )
	include( CTest )
endif()

# Default debug suffix for libraries.
set( CMAKE_DEBUG_POSTFIX "d" )


# Define target folders
# (now that ccViewer can have its own plugins, qCC and ccViewer must fall in separate folders!
if(WIN32 OR APPLE) 
	set( CLOUDCOMPARE_DEST_FOLDER CloudCompare )
	set( CCVIEWER_DEST_FOLDER ccViewer )
	if (OPTION_GL_QUAD_BUFFER_SUPPORT)
		set ( CLOUDCOMPARE_DEST_FOLDER ${CLOUDCOMPARE_DEST_FOLDER}Stereo )
		set ( CCVIEWER_DEST_FOLDER ${CCVIEWER_DEST_FOLDER}Stereo )
	endif()
else()
	set( CLOUDCOMPARE_DEST_FOLDER bin )
	set( CCVIEWER_DEST_FOLDER bin )
endif()

if( WIN32 )
    set( INSTALL_DESTINATIONS ${CLOUDCOMPARE_DEST_FOLDER} )
    
    if( ${OPTION_BUILD_CCVIEWER} )
        list( APPEND INSTALL_DESTINATIONS ${CCVIEWER_DEST_FOLDER} )
	endif()
elseif( UNIX AND NOT APPLE )
	# RPATH Linux/Unix: (dynamic) libs are put in $prefix/$lib/cloudcompare,
	# since they are only used by qCC/ccViewer
	include( GNUInstallDirs )
	set( LINUX_INSTALL_SHARED_DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}/cloudcompare" )
	set( CMAKE_INSTALL_RPATH ${LINUX_INSTALL_SHARED_DESTINATION} )
	set( INSTALL_DESTINATIONS ${CMAKE_INSTALL_PREFIX})
endif()

# Load advanced scripts
include( CMakeInclude )
include( Install )

# Add external libraries
include( CMakeExternalLibs )

# Contrib. libraries (mainly for I/O)
include( AllSupport )

add_subdirectory(ccqfc_softlink)
# Internal libs used by both CloudCompare & ccViewer
add_subdirectory( libs )

# Plugins
add_subdirectory( plugins )

# qCC
add_subdirectory( qModel )


# CCViewer
if( OPTION_BUILD_CCVIEWER )
	add_subdirectory( ccViewer )
endif()



IF (MSVC)
	message("debugger environment: ${PROJECT_PATH}")
	set_target_properties(${IDE_TARGET_NAME} PROPERTIES VS_DEBUGGER_ENVIRONMENT "${PROJECT_PATH}")  
ENDIF() 

include(configShare)

