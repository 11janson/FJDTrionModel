colormsg(_HIBLUE_ "---Start---Configuring  tritonmetahub library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/metahubcomputecl_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

##添加依赖库
INCLUDE(ccdb_dependencies)
#INCLUDE(${THIRDPARTY_CMAKE}/pcl.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/vtk.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/opencv.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/boostcomputecl.cmake)

INCLUDE(${THIRDPARTY_CMAKE}/opencl.cmake)

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


########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/metahubcomputecl_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/metahubcomputecl.cmake
    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )