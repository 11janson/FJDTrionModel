colormsg(_HIBLUE_ "---Start---Configuring  csutils library:")
#############################################################

IF(APPLE)
    add_compile_options(-x objective-c++)
ENDIF()

## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/csutils_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()


#################配置头文件###################
include_directories(${THIRDPARTY_ROOT}/csutils)


# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
#####################################################
# UI files
FILE(GLOB_RECURSE FORMS "${CMAKE_CURRENT_LIST_DIR}/*.ui")

# Files
IF(WIN32)
    LIST(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/consoleprocess_unix.cpp")
ELSE(WIN32)
    LIST(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/consoleprocess_win.cpp")
    LIST(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/winutils.cpp")
    LIST(REMOVE_ITEM HEADERS "${CMAKE_CURRENT_LIST_DIR}/winutils.h")
ENDIF(WIN32)
#####################################################
FILE(GLOB_RECURSE images "${CMAKE_CURRENT_LIST_DIR}/images/*.png")

#####################################################
# Resource files.
SET (RESOURCES
	${CMAKE_CURRENT_LIST_DIR}/csutils.qrc
)
########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/csutils_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/csutils.cmake
    ${CMAKE_CURRENT_LIST_DIR}/csutils_lib.pri
    ${CMAKE_CURRENT_LIST_DIR}/csutils_dependencies.pri
	${CMAKE_CURRENT_LIST_DIR}/tooltip/images/f1.svg
    ${CMAKE_CURRENT_LIST_DIR}/tooltip/images/f1.png
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )