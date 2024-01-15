#############################################################################################################################
message("-----------------------------config ccqfc dependencies-----------------------------------")
##############################配置##############################################################
#######################配置头文件###############################################################
include_directories(${THIRDPARTY_ROOT}/csaggregation)
include_directories(${THIRDPARTY_ROOT}/csutils)
include_directories(${THIRDPARTY_ROOT}/cswidgets)
include_directories(${THIRDPARTY_ROOT}/sentinelldkencrypt)
include_directories(${VCPKG_HOME}/installed/x64-windows/include)
include_directories(${THIRDPARTY_ROOT}/qpdflib)
include_directories(${THIRDPARTY_ROOT}/quazip)

#include(${THIRDPARTY_CMAKE}/pcl.cmake)
#include(${THIRDPARTY_CMAKE}/vtk.cmake)
#include(${THIRDPARTY_CMAKE}/opencv.cmake)


#################链接库####################
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/csaggregation.lib debug ${LIBRARY_OUTPUT_PATH}/csaggregationd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/csutils.lib debug ${LIBRARY_OUTPUT_PATH}/csutilsd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/cswidgets.lib debug ${LIBRARY_OUTPUT_PATH}/cswidgetsd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/crashreporter.lib debug ${LIBRARY_OUTPUT_PATH}/crashreporterd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/ncspdlog.lib debug ${LIBRARY_OUTPUT_PATH}/ncspdlogd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/sentinelldkencrypt.lib debug ${LIBRARY_OUTPUT_PATH}/sentinelldkencryptd.lib)
#link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/qpdflib.lib debug ${LIBRARY_OUTPUT_PATH}/qpdflibd.lib)



