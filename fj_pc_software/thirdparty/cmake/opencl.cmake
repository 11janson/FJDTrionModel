#######################################config thirdparty opencl###################################
message("-------------------------------config thirdparty opencl---------------------------------")

#################配置路径#####################
SET(OPENCL_INCLUDE ${THIRDPARTY_ROOT}/opencl)
SET(OPENCL_LIB ${THIRDPARTY_ROOT}/opencl)

#################配置头文件###################
include_directories(${OPENCL_INCLUDE})
link_libraries(optimized ${OPENCL_LIB}/OpenCL.lib debug ${OPENCL_LIB}/OpenCL.lib)