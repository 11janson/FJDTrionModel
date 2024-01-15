colormsg(_HIBLUE_ "---Start---Configuring  cloudcompareutils library:")

#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils_dependencies.cmake)
include(${THIRDPARTY_CMAKE}/qxlsx.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})
#INCLUDE(${THIRDPARTY_CMAKE}/cos.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/yaml.cmake)

find_package(yaml-cpp REQUIRED)

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()


# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
#####################################################

#####################################################
# Resource files.
SET (RESOURCES
)
########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils.cmake
    ${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils_lib.pri
    ${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils_dependencies.pri
	${CMAKE_CURRENT_LIST_DIR}/cloudcompareutils.pro
    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )