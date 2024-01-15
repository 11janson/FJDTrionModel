option( PLUGIN_STANDARD_QPCL "Install qPCL plugin" ON )

if( PLUGIN_STANDARD_QPCL )	  
	project( QPCL_PLUGIN )
	AddPlugin( NAME ${PROJECT_NAME} )
	set_target_properties(${PROJECT_NAME} PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)

	add_subdirectory( doc )
	add_subdirectory( include )
	add_subdirectory( src )
	
	add_subdirectory( PclUtils ) # PCL bridge library
	add_subdirectory( PclIO ) # PCL I/O plugin (to load PCD files)
	 
	target_link_libraries( ${PROJECT_NAME} PRIVATE QPCL_PLUGIN_UTILS_LIB )
		
	# Import PCL dlls (if any, WIN32 only)
	include( cmake/ExportPCLDlls.cmake )
	export_PCL_dlls( "${CLOUDCOMPARE_DEST_FOLDER}" )
	
	target_link_libraries( ${PROJECT_NAME}  PRIVATE ${CS_PROJECT_NAME} )
	include_directories(${LIBS_HOME}/model)
endif()
