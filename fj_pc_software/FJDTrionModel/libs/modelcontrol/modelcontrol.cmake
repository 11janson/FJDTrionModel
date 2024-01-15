colormsg(_HIBLUE_ "---Start---Configuring  smartweldres library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/modelcontrol_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})



INCLUDE(ccdb_dependencies)
#INCLUDE(${THIRDPARTY_CMAKE}/pcl.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/vtk.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/opencv.cmake)
INCLUDE(${THIRDPARTY_CMAKE}/opencl.cmake)
#INCLUDE(${THIRDPARTY_CMAKE}/yaml.cmake)



#link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/fj_pc_alg.lib debug ${LIBRARY_OUTPUT_PATH}/fj_pc_algd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/QFJD_DB_LIB.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/QFJD_DB_LIBd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/QCSF_PLUGIN.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/QCSF_PLUGINd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/FJDCoreLib.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/FJDCoreLibd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/QPCL_2ccCloud_LIB.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/QPCL_2ccCloud_LIBd.lib)
#link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/fj_forest_alg.lib debug ${LIBRARY_OUTPUT_PATH}/fj_forest_algd.lib)
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
    ${CMAKE_CURRENT_LIST_DIR}/modelcontrol_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/modelcontrol.cmake
    ${CMAKE_CURRENT_LIST_DIR}/modelcontrol_lib.pri
    ${CMAKE_CURRENT_LIST_DIR}/modelcontrol_dependencies.pri
	${CMAKE_CURRENT_LIST_DIR}/modelcontrol.pro
    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )