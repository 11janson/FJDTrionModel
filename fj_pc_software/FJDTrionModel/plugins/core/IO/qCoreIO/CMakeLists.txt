option( PLUGIN_IO_QCORE "Install the CoreIO plugin" ON )

#链接库
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubutils.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubutilsd.lib)

if( PLUGIN_IO_QCORE )
	project( QCORE_IO_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} TYPE io )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
