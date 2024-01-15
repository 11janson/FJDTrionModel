colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad exception_handler library:")
#####################################################
# Source files.
SET(SOURCES
    ${SOURCES}
    ${GBREAKPAD_ROOT}/client/windows/handler/exception_handler.cc
)
#####################################################
# Header files.
SET(HEADERS
    ${HEADERS}
    ${GBREAKPAD_ROOT}/client/windows/handler/exception_handler.h
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
    ${CMAKE_CURRENT_LIST_DIR}/exception_handler.cmake
)
# dependencies
set(DEPENDENCIES
    ${DEPENDENCIES}
)
#依赖项目
SET(CS_MODULE_DEPS
    ${CS_MODULE_DEPS}
)
