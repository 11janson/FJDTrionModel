#######################################config thirdparty liblas sdk###################################
message("-------------------------------config thirdparty liblas sdk---------------------------------")


#################配置路径#####################
SET(LIBLAS_INCLUDE ${VCPKG_HOME}/installed/x64-windows/include/liblas)
SET(LIBLAS_LIB ${VCPKG_HOME}/installed/x64-windows/lib)

#################配置头文件###################
include_directories(${LIBLAS_INCLUDE})




#################链接库####################
link_libraries(optimized ${LIBLAS_LIB}/liblas.lib debug ${LIBLAS_LIB}/liblas.lib)
