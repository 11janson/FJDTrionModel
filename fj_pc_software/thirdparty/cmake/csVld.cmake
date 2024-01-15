MESSAGE("set vld library config ---")
SET(VLD_HOME ${CCQFC_ROOT}/vld)
 
include_directories(${VLD_HOME})
link_directories(${VLD_HOME})
link_libraries(optimized vld debug vld)

IF (MSVC)
	set(PROJECT_PATH "${PROJECT_PATH}" "${VLD_HOME};")
	set(PROJECT_PATH ${PROJECT_PATH} PARENT_SCOPE)
ENDIF()

file(COPY ${VLD_HOME}/vld.lib DESTINATION ${LIB_DIR} )
file(COPY ${VLD_HOME}/dbghelp.dll DESTINATION ${BIN_DIR} )
file(COPY ${VLD_HOME}/vld_x64.dll DESTINATION ${BIN_DIR})
