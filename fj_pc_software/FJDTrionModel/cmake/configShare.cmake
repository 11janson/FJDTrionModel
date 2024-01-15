message(_HIBLUE_ "---Start---Configuring CloudCompare Static Data:")
include_directories(${LIBRARY_OUTPUT_PATH}/../libs/qCC_db/extern/CCCoreLib/exports)
#############################################################
ADD_CUSTOM_COMMAND(OUTPUT  COPY_SHARE_FILES
    COMMAND echo "Coping theme files..."
    COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_OUTPUT_PATH}/theme
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CODE_HOME}/bin-x64/theme ${BIN_OUTPUT_PATH}/theme/
	
	COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_OUTPUT_PATH}/shaders/EDL
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PLUGINS_HOME}/core/GL/qEDL/shaders/EDL ${BIN_OUTPUT_PATH}/shaders/EDL
	
	COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_OUTPUT_PATH}/shaders/SSAO
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PLUGINS_HOME}/core/GL/qSSAO/shaders/SSAO ${BIN_OUTPUT_PATH}/shaders/SSAO
	
)


ADD_CUSTOM_TARGET(fjdynamicsshare ALL
    DEPENDS COPY_SHARE_FILES
    SOURCES ${themes_files}  ${config_files} ${translations_files}
    )
##项目分组
#set_target_properties(fjdynamicsshare PROPERTIES FOLDER share)


