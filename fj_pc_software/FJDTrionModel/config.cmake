######################配置第三方库路径################################
#cmake
#fj_pc_alg
#vcpkg
###################################################################
message("-----------------------------------thirdparty path--------------------------------")
###选择第三方库路径
SET(THIRDPARTY_ROOT CACHE PATH "thirdparty root path")
IF("${THIRDPARTY_ROOT}" STREQUAL "")
    MESSAGE(FATAL_ERROR "Please select thirdparty root path")	
ENDIF()
MESSAGE("Current thirdparty path is ${THIRDPARTY_ROOT}")

##设置第三方库cmake 路径
SET(THIRDPARTY_CMAKE ${THIRDPARTY_ROOT}/cmake)
MESSAGE("thirdparty cmake is ${THIRDPARTY_CMAKE}")

##设置fjpcl库路径
SET(FJPCL_HOME ${THIRDPARTY_ROOT}/fj_pc_alg)
MESSAGE("thirdparty fjpcl sdk is ${FJPCL_HOME}")


##选择Vcpkg目录
SET(VCPKG_HOME CACHE PATH "vcpkg root path")
IF("${VCPKG_HOME}" STREQUAL "")
    MESSAGE(FATAL_ERROR "Please select vcpkg root path")	
ENDIF()
MESSAGE("thirdparty vcpkg is ${VCPKG_HOME}")


#添加包含目录
include_directories(${THIRDPARTY_ROOT})
