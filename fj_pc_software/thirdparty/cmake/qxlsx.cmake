#######################################config thirdparty qxlsx sdk###################################
message("-------------------------------config thirdparty qxlsx sdk---------------------------------")

add_compile_definitions(qxlsx_EXPORTS) 

#################配置路径#####################
SET(QXLSX_INCLUDE ${THIRDPARTY_ROOT}/qxlsx/header)
SET(QXLSX_LIB ${THIRDPARTY_ROOT}/qxlsx/lib)

#################配置头文件###################
include_directories(${QXLSX_INCLUDE})




#################链接库####################
link_libraries(optimized ${QXLSX_LIB}/QXlsxQt5.lib debug ${QXLSX_LIB}/QXlsxQt5d.lib)

file(COPY ${THIRDPARTY_ROOT}/qxlsx/lib/QXlsxQt5.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${THIRDPARTY_ROOT}/qxlsx/lib/QXlsxQt5.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)