option( PLUGIN_IO_QADDITIONAL "Install qAdditionalIO plugin" ON )

#链接库
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubutils.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubutilsd.lib)

if( PLUGIN_IO_QADDITIONAL )
	project( QADDITIONAL_IO_PLUGIN )
	AddPlugin( NAME ${PROJECT_NAME} TYPE io )
	set_target_properties(${PROJECT_NAME} PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
