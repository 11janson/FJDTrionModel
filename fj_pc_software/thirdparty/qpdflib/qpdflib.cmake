colormsg(_HIBLUE_ "---Start---Configuring  qpdflib library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/qpdflib_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()
#################配置头文件###################
include_directories(${THIRDPARTY_ROOT}/qpdflib)


# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
#####################################################
# UI files
FILE(GLOB_RECURSE FORMS "${CMAKE_CURRENT_LIST_DIR}/*.ui")

#####################################################
FILE(GLOB_RECURSE images "${CMAKE_CURRENT_LIST_DIR}/pdfview/images/*.png")
#####################################################
# Resource files.
SET (RESOURCES
	${CMAKE_CURRENT_LIST_DIR}/pdfview.qrc
)
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/qpdflib_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/qpdflib.cmake
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
)


	
