colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad crash_generation_app app:")
#####################################################
# Source files.
SET(SOURCES
	${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/abstract_class.cc
	${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/crash_generation_app.cc
)
#####################################################
# Header files.
SET(HEADERS
	${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/crash_generation_app.h
)

#####################################################
# Resource files.
SET (RESOURCES
)
########################################################
# Other files
SET(OTHER_FILES
	crash_generation_app.cmake
	${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/small.ico
	${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/crash_generation_app.ico
)
IF(WIN32)
	LIST(APPEND OTHER_FILES 
		${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/crash_generation_app.rc
	)
	SET(HEADERS
		${HEADERS}		
		${GBREAKPAD_ROOT}/client/windows/tests/crash_generation_app/resource.h
	)
ENDIF(WIN32)
IF(WIN32)
	#  dependencies
	set(DEPENDENCIES
		Wininet.lib
	)
ENDIF()
#依赖项目
SET(WIS_MODULE_DEPS
#	common
#	exception_handler
#	crash_generation_client
#	crash_generation_server
)


