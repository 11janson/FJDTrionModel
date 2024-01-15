colormsg(_HIBLUE_ "---Start---Configuring Google Breakpad common library:")
#####################################################
# Source files.
SET(SOURCES
    ${SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/crashhandler.cpp
)
#####################################################
# Header files.
SET(HEADERS
    ${HEADERS}
    ${CMAKE_CURRENT_LIST_DIR}/crashhandler.h
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
    ${CMAKE_CURRENT_LIST_DIR}/crashhandler.cmake
)