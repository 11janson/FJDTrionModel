colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad crash_generation_server library:")
#####################################################
# Source files.
SET(SOURCES
	${GBREAKPAD_ROOT}/client/windows/crash_generation/client_info.cc
	${GBREAKPAD_ROOT}/client/windows/crash_generation/crash_generation_server.cc
	${GBREAKPAD_ROOT}/client/windows/crash_generation/minidump_generator.cc
)
#####################################################
# Header files.
SET(HEADERS
	${GBREAKPAD_ROOT}/client/windows/crash_generation/crash_generation_server.h
	${GBREAKPAD_ROOT}/client/windows/crash_generation/client_info.h
	${GBREAKPAD_ROOT}/client/windows/crash_generation/minidump_generator.h
)

#####################################################
# Resource files.
SET (RESOURCES
)
########################################################
# Other files
SET(OTHER_FILES
	crash_generation_server.cmake
)
# dependencies
set(DEPENDENCIES
	
)
#依赖项目
SET(CS_MODULE_DEPS
)