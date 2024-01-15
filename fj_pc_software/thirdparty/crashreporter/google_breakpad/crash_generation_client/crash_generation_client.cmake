colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad crash_generation_client library:")
#####################################################
# Source files.
SET(SOURCES
    ${SOURCES}
    ${GBREAKPAD_ROOT}/client/windows/crash_generation/crash_generation_client.cc
)
#####################################################
# Header files.
SET(HEADERS
    ${HEADERS}
    ${GBREAKPAD_ROOT}/client/windows/crash_generation/crash_generation_client.h
)

#####################################################
# Resource files.
SET (RESOURCES
    ${RESOURCES}
)
########################################################
# Other files
SET(OTHER_FILES
    ${OTHER_FILES}
    ${CMAKE_CURRENT_LIST_DIR}/crash_generation_client.cmake
)
# dependencies
set(DEPENDENCIES
    ${DEPENDENCIES}
)
#依赖项目
SET(CS_MODULE_DEPS
   ${CS_MODULE_DEPS}
)