colormsg(_HIBLUE_ "---Start---Configuring NCQFC CrashReporter App:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/crashreporter_dependencies.cmake)
## 项目名称
PROJECT(${WIS_PROJECT_NAME})
wis_version(MAJOR 1 MINOR 0 PATCH 0)
## Include路径
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}
    )
set(${PROJECTNAMEU}_VERSION_MAJOR ${${PROJECTNAMEU}_VERSION_MAJOR} PARENT_SCOPE)
set(${PROJECTNAMEU}_VERSION_MINOR ${${PROJECTNAMEU}_VERSION_MINOR} PARENT_SCOPE) 
set(${PROJECTNAMEU}_VERSION_PATCH ${${PROJECTNAMEU}_VERSION_PATCH} PARENT_SCOPE)
########################################################
configure_file(${CMAKE_CURRENT_LIST_DIR}/version.h.cm ${PROJECT_BINARY_DIR}/version.h)

EXECUTE_PROCESS(
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/crashreporter.rc ${PROJECT_SOURCE_DIR}/crashreporter.rc
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/crashreporter.ico ${PROJECT_SOURCE_DIR}/crashreporter.ico
    )
# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
SET(HEADERS
    ${HEADERS}
    ${PROJECT_BINARY_DIR}/version.h
    )
#####################################################
# UI files
FILE(GLOB_RECURSE FORMS "${CMAKE_CURRENT_LIST_DIR}/*.ui")

#####################################################
# Resource files.
SET (RESOURCES
    
)
########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/crashreporter_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/crashreporter.cmake
    ${CMAKE_CURRENT_LIST_DIR}/crashreporter_dependencies.pri
    ${CMAKE_CURRENT_LIST_DIR}/crashreporter_lib.pri
    ${CMAKE_CURRENT_LIST_DIR}/version.h.cm
)
########################################################
IF(WIN32)
    LIST(APPEND OTHER_FILES ${PROJECT_SOURCE_DIR}/crashreporter.rc)
ENDIF(WIN32)
#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )
