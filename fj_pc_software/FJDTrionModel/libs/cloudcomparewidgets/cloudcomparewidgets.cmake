colormsg(_HIBLUE_ "---Start---Configuring  cloudcomparewidgets library:")

#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

##添加依赖库
INCLUDE(ccdb_dependencies)
#INCLUDE(${THIRDPARTY_CMAKE}/pcl.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/vtk.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/opencv.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/opencl.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/yaml.cmake)

find_package(yaml-cpp REQUIRED)

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()
# Source files.
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/sentinelldkencrypt.lib debug ${LIBRARY_OUTPUT_PATH}/sentinelldkencryptd.lib)
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
    ${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets.cmake
    ${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets_lib.pri
    ${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets_dependencies.pri
	${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets.pro
    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )