project( QFJD_GL_LIB )

#include(ccqfc_dependencies)

#链接库
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubutils.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubutilsd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/model.lib debug ${LIBRARY_OUTPUT_PATH}/modeld.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/graphicsviewvector.lib debug ${LIBRARY_OUTPUT_PATH}/graphicsviewvectord.lib)

add_library( ${PROJECT_NAME} SHARED )

add_subdirectory( include )
add_subdirectory( src )

target_compile_definitions( ${PROJECT_NAME} PRIVATE CCGLWINDOW_LIB_BUILD )

target_link_libraries( ${PROJECT_NAME} PRIVATE
	QFJD_DB_LIB
    cswidgets
)

if ( OPTION_USE_OCULUS_SDK )
    target_link_OCULUS_SDK( ${PROJECT_NAME} )
endif()

InstallSharedLibrary( TARGET ${PROJECT_NAME} )
