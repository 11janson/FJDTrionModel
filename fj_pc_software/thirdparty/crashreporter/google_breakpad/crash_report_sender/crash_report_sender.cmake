colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad crash_report_sender library:")
#####################################################
# Source files.
SET(SOURCES
    ${SOURCES}
    ${GBREAKPAD_ROOT}/client/windows/sender/crash_report_sender.cc
)
#####################################################
# Header files.
SET(HEADERS
    ${HEADERS}
    ${GBREAKPAD_ROOT}/client/windows/sender/crash_report_sender.h
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
    ${CMAKE_CURRENT_LIST_DIR}/crash_report_sender.cmake
)
IF(WIN32)
    #  dependencies
    set(DEPENDENCIES
        ${DEPENDENCIES}
        Wininet.lib
    )
ENDIF()
#依赖项目
SET(CS_MODULE_DEPS
    ${CS_MODULE_DEPS}
#    common
)
