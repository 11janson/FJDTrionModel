project(${IDE_TARGET_NAME} VERSION 2.12.0 )
include( CMakePolicies NO_POLICY_SCOPE )


##################################加载配置的库###################################
include(Saribbon)
include(ccqfc_dependencies)
include(libs_dependencies)

include(${THIRDPARTY_CMAKE}/opencv_world.cmake)

include_directories( ${CloudComparePlugins_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/db_tree )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/ui_templates )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${LIBS_SARIBBON_SRC} )

# 版本
string(TIMESTAMP COMPILE_TIME %Y%m%d)
set(BUILD_TIME ${COMPILE_TIME})
set(TRITONMETAHUB_VERSIONS "1.000.D.0100.${BUILD_TIME}" CACHE STRING "1.000.D.0100")
add_compile_definitions(FJDTRITONMETAHUB_VERSIONS="${TRITONMETAHUB_VERSIONS}")
message("${VERSIONS}")
file( GLOB header_list *.h  )
file( GLOB source_list *.cpp )

file( GLOB ui_list ui_templates/*.ui )
file( GLOB qrc_list *.qrc )

file( GLOB txt_list TODO.txt ${CloudCompareProjects_SOURCE_DIR}/CHANGELOG.md )

add_subdirectory( db_tree )
add_subdirectory( pluginManager )
add_subdirectory( extern/QCustomPlot EXCLUDE_FROM_ALL )

qt5_wrap_ui( generated_ui_list ${ui_list} )
qt5_add_resources( generated_qrc_list ${qrc_list} )

if( MSVC )
	# App icon with MSVC
	set( rc_list images/icon/cc_icon.rc )

	#to get rid of the (system) console
	add_executable( ${PROJECT_NAME} WIN32 ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${txt_list} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/windows/qt5.natvis )
elseif( APPLE )
	add_executable( ${PROJECT_NAME} MACOSX_BUNDLE ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${txt_list} )
else()
	add_executable( ${PROJECT_NAME} ${header_list} ${source_list} ${generated_ui_list} ${generated_qrc_list} ${rc_list} ${txt_list} )
endif()

if( WIN32 )
	DeployQt(
		TARGET
			${PROJECT_NAME}
		DEPLOY_PATH
			${CLOUDCOMPARE_DEST_FOLDER}
	)
else()
	DeployQt(
		TARGET
			${PROJECT_NAME}
		DEPLOY_PATH
			${PROJECT_NAME}
	)
endif()

find_package( Qt5 COMPONENTS
	AxBase
    AxContainer
	Xml
    Network
    REQUIRED
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJ4_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} PRIVATE
    FJDAppCommon
	qpdflib
    QCustomPlot
    Qt5::PrintSupport
    Qt5::Network
	modelcontrol
	trionmetahubutils  
	QPCL_2ccCloud_LIB
)


if( CCCORELIB_USE_QT_CONCURRENT )
	# We want *deployqt to handle this dependency, so instead of just copying the file
	# link to the target.
	target_link_libraries( ${PROJECT_NAME} PRIVATE
		Qt5::Concurrent
		Qt5::AxBase
		Qt5::AxContainer
		Qt5::Xml
	)
endif()

# contrib. libraries support
if( APPLE )
	target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_MAC_FRAMEWORK_DIR} )
elseif( UNIX )
	target_link_contrib( ${PROJECT_NAME} lib/cloudcompare )
else()
	target_link_contrib( ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} )
endif()

# Add custom preprocessor definitions
if( WIN32 )
	if (MSVC)
		set_target_properties( ${PROJECT_NAME} PROPERTIES LINK_FLAGS " /MANIFEST:NO" )
	endif()
endif()

# App icon with Code::Blocks/MinGW
if( WIN32 )
	if( MINGW )
		if (NOT CMAKE_RC_COMPILER)
			set( CMAKE_RC_COMPILER windres )
		endif()
		add_custom_command( TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_RC_COMPILER} -i ${CMAKE_CURRENT_SOURCE_DIR}/images/icon/cc_icon.rc --input-format=rc -o ${CMAKE_CURRENT_BINARY_DIR}/cc_icon.res -O coff )
	endif()
endif()

if (NOT APPLE)
	# Install program.
	# On macOs, the DeployQt step will install the bundle that contains the executable with
	# library paths properly set, reinstalling the executable here would break the bundle.
	install_ext( TARGETS ${PROJECT_NAME} ${CLOUDCOMPARE_DEST_FOLDER} "" )
endif()

# Auxiliary files
set( auxFiles  ${CloudCompareProjects_SOURCE_DIR}/CHANGELOG.md bin_other/license.txt bin_other/global_shift_list_template.txt )

if( WIN32 )
	# Additional auxiliary file(s)
	file( GLOB batch_list bin_other/*.bat )
	list( APPEND auxFiles ${batch_list} )
endif()

# Install auxiliary files
if( WIN32 OR APPLE )
	foreach( filename ${auxFiles} )
		install_ext( FILES ${filename} ${CLOUDCOMPARE_DEST_FOLDER} "" )
	endforeach()
else( UNIX )
	foreach( filename ${auxFiles} )
		install_ext( FILES ${filename} share/cloudcompare "" )
	endforeach()
endif()


# Put together our Application bundle on macOS
if( APPLE )    
    add_subdirectory( Mac )
endif()

# Translation
add_subdirectory(translations)


#target_link_libraries(${PROJECT_NAME} ${QT_DIR}lib/Qt5Network.lib)
## crashhandler
IF(WIN32) ### 暂支持Win
    IF(MSVC)
        include(${THIRDPARTY_ROOT}/crashreporter/google_breakpad.cmake)
    ENDIF()
ENDIF()


##设置管理员权限
IF(WIN32) ### 暂支持Win
    IF(MSVC)
        #SET(CMAKE_EXE_LINKER_FLAGS     "${CMAKE_EXE_LINKER_FLAGS} /level='requireAdministrator' /uiAccess='false'")
        #SET_TARGET_PROPERTIES(${IDE_TARGET_NAME} PROPERTIES LINK_FLAGS    "/MANIFESTUAC:\"level='requireAdministrator' uiAccess='false'\" /SUBSYSTEM:WINDOWS")
    ENDIF()
ENDIF()




## Shaders
# Copy shader dirs into our shadow build directory
file( COPY ${FJD_FBO_LIB_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )
file( COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION "${CMAKE_BINARY_DIR}" )

# Export common shader files to all install destinations
if( APPLE )	
	install( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION ${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders/ColorRamp )
elseif( UNIX )
	install( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert DESTINATION share/cloudcompare/shaders/Bilateral )
	install( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag DESTINATION share/cloudcompare/shaders/ColorRamp )
else()
	install_ext( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Bilateral )
	install_ext( FILES ${FJD_FBO_LIB_SOURCE_DIR}/shaders/Bilateral/bilateral.vert ${CLOUDCOMPARE_DEST_FOLDER} /shaders/Bilateral )
	install_ext( FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/ColorRamp/color_ramp.frag ${CLOUDCOMPARE_DEST_FOLDER} /shaders/ColorRamp )
endif()

# Install plugins and shaders in the correct folder for each platform
if( APPLE )	
	InstallPlugins(
		DEST_PATH "${CLOUDCOMPARE_MAC_PLUGIN_DIR}"
		SHADER_DEST_PATH "${CLOUDCOMPARE_MAC_BASE_DIR}/Contents/Shaders"
	)
elseif( UNIX )
	InstallPlugins(
		DEST_PATH "${CMAKE_INSTALL_LIBDIR}/cloudcompare/plugins"
		SHADER_DEST_PATH share/cloudcompare/shaders
	)
else()
	InstallPlugins(
		DEST_PATH "${CLOUDCOMPARE_DEST_FOLDER}"
		DEST_FOLDER "plugins"
		SHADER_DEST_PATH "${CLOUDCOMPARE_DEST_FOLDER}"
		SHADER_DEST_FOLDER "shaders"
	)
endif()
set_target_properties(FJDTrionModel PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)
