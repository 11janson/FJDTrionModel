#######################################config thirdparty laslib sdk###################################
message("-------------------------------config thirdparty laslib sdk---------------------------------")


#################配置路径#####################
SET(LASLIB_INCLUDE ${VCPKG_HOME}/installed/x64-windows/include/laslib)
SET(LASLIB_LIB ${VCPKG_HOME}/installed/x64-windows/lib)

#################配置头文件###################
include_directories(${LASLIB_INCLUDE})




#################链接库####################
link_libraries(optimized ${LASLIB_LIB}/LASlib.lib debug ${LASLIB_LIB}/LASlibD.lib)
