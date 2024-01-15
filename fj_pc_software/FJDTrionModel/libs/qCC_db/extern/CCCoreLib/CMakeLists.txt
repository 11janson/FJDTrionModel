# SPDX-License-Identifier: MIT
# Copyright © Andy Maloney <asmaloney@gmail.com>

cmake_minimum_required( VERSION 3.10 )

project( FJDCoreLib
	DESCRIPTION
		"Core CloudCompare data structures & algorithms for handling point clouds"
	LANGUAGES
		CXX
	VERSION
		1.0
)

# Options
option( CCCORELIB_USE_CGAL
	"Compile CCCoreLib with CGAL (to enable Delaunay 2.5D triangulation with a GPL-compliant licence)"
	OFF
)
option( CCCORELIB_USE_TBB
	"Compile CCCoreLib with Intel Threading Building Blocks lib (enables some parallel processing )"
	OFF
)
option( CCCORELIB_USE_QT_CONCURRENT
	"Compile CCCoreLib with QtConcurrent (to enable parallel processing)"
	ON
)
option( CCCORELIB_SHARED
	"Compile CCCoreLib as a shared library"
	ON
)
option( CCCORELIB_SCALAR_DOUBLE
	"Define ScalarType as double (instead of float)"
	OFF
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add the library (shared or static)
if ( CCCORELIB_SHARED )
	add_library( ${PROJECT_NAME} SHARED )

	set_target_properties( ${PROJECT_NAME}
		PROPERTIES
			CXX_VISIBILITY_PRESET hidden
			VISIBILITY_INLINES_HIDDEN 1
	)
else()
	add_library( ${PROJECT_NAME} STATIC )

	target_compile_definitions( ${PROJECT_NAME}
		PRIVATE
			CC_CORE_LIB_STATIC_DEFINE
	)
endif()

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
		DEBUG_POSTFIX d
)

add_library( ${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME} )

# Generate the export header file
include( GenerateExportHeader )

generate_export_header( FJDCoreLib
	EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
	EXPORT_MACRO_NAME CC_CORE_LIB_API
)

target_sources( FJDCoreLib
	PRIVATE
		${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
)

install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
	DESTINATION
		include/CCCoreLib
)

target_include_directories( ${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/exports>
		$<INSTALL_INTERFACE:include/CCCoreLib>
)

# ccache
# https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program( CCACHE_PROGRAM ccache )

if ( CCACHE_PROGRAM )
	set_target_properties( ${PROJECT_NAME}
		PROPERTIES
			CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}"
			C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}"
	)
endif()

#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

#ADD_DEFINITIONS(-DNDEBUG)
# 设置 Release 构建类型下的宏定义
#if (CMAKE_BUILD_TYPE MATCHES Release)
    add_definitions(-DNDEBUG)
#endif()
# Main sources and includes
add_subdirectory( include )
add_subdirectory( src )

# Compiler & definitions
target_compile_features( ${PROJECT_NAME}
	PRIVATE
		cxx_std_14
)

set_target_properties( ${PROJECT_NAME}
	PROPERTIES
		CXX_EXTENSIONS OFF
)

target_compile_definitions( ${PROJECT_NAME}
	PRIVATE
		"$<$<CONFIG:DEBUG>:CC_DEBUG>"
)

if ( CCCORELIB_SCALAR_DOUBLE )
	target_compile_definitions( ${PROJECT_NAME}
		PUBLIC
			CC_CORE_LIB_USES_DOUBLE
	)
else()
	target_compile_definitions( ${PROJECT_NAME}
		PUBLIC
			CC_CORE_LIB_USES_FLOAT
	)
endif()

# Windows-specific flags
if ( WIN32 )
	# VLD for mem leak checking
	option( CCCORELIB_USE_VISUAL_LEAK_DETECTOR
		"Check to activate compilation (in debug) with Visual Leak Detector"
		OFF
	)

	if ( CCCORELIB_USE_VISUAL_LEAK_DETECTOR )
		target_compile_definitions( ${PROJECT_NAME}
			PRIVATE
				CC_CORE_LIB_USES_VLD
		)
	endif()

	# Disable SECURE_SCL
	# See https://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/
	target_compile_definitions( ${PROJECT_NAME}
		PRIVATE
			"$<$<CONFIG:RELEASE>:_SECURE_SCL=0>"
	)

	target_compile_definitions( ${PROJECT_NAME}
		PRIVATE
			_CRT_SECURE_NO_WARNINGS
			__STDC_LIMIT_MACROS
			NOMINMAX
	)
	
	# Release模式下异常捕获开启异步模式，捕获cgal库异常
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /EHa")

endif()

# TBB (optional)
# Must come before CGAL so it can use TBB properly
if ( CCCORELIB_USE_TBB )
	find_package( TBB COMPONENTS tbb CONFIG )

	if ( TBB_FOUND )
		target_link_libraries( ${PROJECT_NAME}
			PUBLIC
				${TBB_IMPORTED_TARGETS}
		)

		target_compile_definitions( ${PROJECT_NAME}
			PUBLIC
				CC_CORE_LIB_USES_TBB
		)
	endif()
endif()

#针对vcpkg的boost配置添加此项 janson 2022.10.24
set(BOOST_ROOT ${BOOST_INCLUDE_DIR} CACHE PATH "")
set(Boost_LIBRARY_DIR_DEBUG "" CACHE PATH "") #添加boost的debug库路径

# CGAL (optional)
if ( CCCORELIB_USE_CGAL )
	find_package( CGAL REQUIRED )
	if( WIN32 )
		# Need to force the visibility of these variables so that we can use them later
		set (GMP_LIBRARIES ${GMP_LIBRARIES} PARENT_SCOPE) 
		set (MPFR_LIBRARIES ${MPFR_LIBRARIES} PARENT_SCOPE) 
	endif()

	if ( CCCORELIB_USE_TBB )
		if ( TBB_FOUND )
			# Once Linux libcgal-dev >= 5.0, target_compile_definitions replaced by:
			#  CGAL_target_use_TBB( CCCoreLib )

			target_compile_definitions( ${PROJECT_NAME}
				PRIVATE
					CGAL_LINKED_WITH_TBB
					NOMINMAX
			)
		else()
			message( WARNING "CGAL cannot compile with TBB (TBB not found)" )
		endif()
	endif()

	target_link_libraries( ${PROJECT_NAME}
		PUBLIC
			CGAL::CGAL
	)

	target_compile_definitions( ${PROJECT_NAME}
		PUBLIC
			CC_CORE_LIB_USES_CGAL_LIB
	)
endif()

# QT (optional)
if ( CCCORELIB_USE_QT_CONCURRENT )
	find_package( Qt5
		COMPONENTS
			Concurrent
		REQUIRED
	)

	find_package( Qt5
	COMPONENTS
		Core
	REQUIRED
	)
	find_package( Qt5
	COMPONENTS
		Gui
	REQUIRED
	)

    set_target_properties( ${PROJECT_NAME} PROPERTIES
        AUTOMOC OFF
        AUTORCC OFF
        AUTOUIC OFF
    )

	target_link_libraries( ${PROJECT_NAME}
		PUBLIC
            Qt5::Concurrent
	)

	target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		Qt5::Core
	)

	target_link_libraries( ${PROJECT_NAME}
	PUBLIC
		Qt5::Gui
	)

	target_compile_definitions( ${PROJECT_NAME}
		PUBLIC
			CC_CORE_LIB_USES_QT_CONCURRENT
	)
endif()

# Install
# See: https://cliutils.gitlab.io/modern-cmake/chapters/install/installing.html
install(
	TARGETS
        ${PROJECT_NAME}
	EXPORT
        ${PROJECT_NAME}-targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include
)

install(
	EXPORT
        ${PROJECT_NAME}-targets
	FILE
		CCCoreLibTargets.cmake
	NAMESPACE
        ${PROJECT_NAME}::
	DESTINATION
		lib/cmake/${PROJECT_NAME}
)

# CMake Package Files
include( CMakePackageConfigHelpers )

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file( CCCoreLibConfigVersion.cmake
	VERSION
		${PACKAGE_VERSION}
	COMPATIBILITY
		AnyNewerVersion
)

install(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/CCCoreLibConfigVersion.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/CCCoreLibConfig.cmake"
	DESTINATION
		lib/cmake/${PROJECT_NAME}
)

# Export
export(
	TARGETS
        ${PROJECT_NAME}
	NAMESPACE
        ${PROJECT_NAME}::
	FILE
		CCCoreLibTargets.cmake
)
