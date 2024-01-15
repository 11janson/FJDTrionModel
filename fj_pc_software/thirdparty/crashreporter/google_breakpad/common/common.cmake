colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad common library:")
#####################################################
# Source files.
SET(SOURCES
    ${SOURCES}
    ${GBREAKPAD_ROOT}/common/windows/guid_string.cc
    ${GBREAKPAD_ROOT}/common/windows/string_utils.cc
    ${GBREAKPAD_ROOT}/common/windows/http_upload.cc
)
#####################################################
# Header files.
SET(HEADERS
    ${HEADERS}
    ${GBREAKPAD_ROOT}/common/windows/guid_string.h
    ${GBREAKPAD_ROOT}/common/windows/http_upload.h
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
    ${CMAKE_CURRENT_LIST_DIR}/common.cmake
)
# dependencies
IF(WIN32)
    set(DEPENDENCIES
        ${DEPENDENCIES}
        Wininet.lib
    )
ENDIF()
#依赖项目
SET(CS_MODULE_DEPS
    ${CS_MODULE_DEPS}
)
