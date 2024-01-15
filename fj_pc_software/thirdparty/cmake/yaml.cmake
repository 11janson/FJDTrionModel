#######################################config thirdparty yaml-cpp sdk###################################
message("-------------------------------config thirdparty yaml-cpp sdk---------------------------------")

add_compile_definitions(yaml_cpp_EXPORTS) 

#################配置路径#####################
SET(YAML_INCLUDE ${THIRDPARTY_ROOT}/yaml-cpp/include)
SET(YANL_LIB ${THIRDPARTY_ROOT}/yaml-cpp/lib)

#################配置头文件###################
include_directories(${YAML_INCLUDE})




#################链接库####################
link_libraries(optimized ${YANL_LIB}/yaml-cpp.lib debug ${YANL_LIB}/yaml-cppd.lib)

