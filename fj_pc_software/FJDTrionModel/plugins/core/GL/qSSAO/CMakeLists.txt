option( PLUGIN_GL_QSSAO "Install qSSAO plugin" ON )

if( PLUGIN_GL_QSSAO )
	
	project( QSSAO_GL_PLUGIN )
	AddPlugin(
		NAME ${PROJECT_NAME}
		TYPE gl
		SHADER_FOLDER SSAO
	)
	set_target_properties( ${PROJECT_NAME} PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( shaders )

	add_subdirectory( extern/Randomkit )
   
	target_link_libraries( ${PROJECT_NAME} PRIVATE RandomKitLib )
endif()
