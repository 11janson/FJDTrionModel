project( QFJD_DB_LIB )

include(ccqfc_dependencies)

#链接库
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubutils.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubutilsd.lib)
#link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/QCC_GL_LIB.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/QCC_GL_LIBd.lib)

#include_directories(${LIBS_HOME}/qCC_glWindow/include)

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

add_subdirectory( extern/CCCoreLib EXCLUDE_FROM_ALL )

target_link_libraries( ${PROJECT_NAME}
    FJDCoreLib
    FJD_FBO_LIB
)

target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD )

set_target_properties( ${PROJECT_NAME} PROPERTIES
	CXX_VISIBILITY_PRESET hidden
)

InstallSharedLibrary( TARGET FJDCoreLib )
InstallSharedLibrary( TARGET ${PROJECT_NAME} )

include( cmake/InstallCGALDependencies.cmake )
