option( PLUGIN_GL_QEDL "Install qEDL plugin" ON )

if( PLUGIN_GL_QEDL )
	project( QEDL_GL_PLUGIN )
	AddPlugin(
		NAME ${PROJECT_NAME}
		TYPE gl
		SHADER_FOLDER EDL
	)
	set_target_properties( ${PROJECT_NAME} PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( shaders )
endif()
