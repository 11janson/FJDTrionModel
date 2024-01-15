option( PLUGIN_STANDARD_QCSF "Install qCSF plugin" ON )

if( PLUGIN_STANDARD_QCSF )
	project( QCSF_PLUGIN )
	AddPlugin( NAME ${PROJECT_NAME} )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
	set_target_properties( ${PROJECT_NAME} PROPERTIES VS_GLOBAL_VcpkgEnabled FALSE)

	add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<$<CONFIG:Release>:${BIN_OUTPUT_PATH}/plugins/QCSF_PLUGIN.dll>" 
    "$<$<CONFIG:Debug>:${BIN_OUTPUT_PATH}/plugins/QCSF_PLUGINd.dll>" 
	"$<$<CONFIG:RelWithDebInfo>:${BIN_OUTPUT_PATH}/plugins/QCSF_PLUGINrd.dll>" 
    "${BIN_OUTPUT_PATH}"
)
endif()

