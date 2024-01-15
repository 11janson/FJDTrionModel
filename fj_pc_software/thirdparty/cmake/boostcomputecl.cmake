#######################################config thirdparty boost compute###################################
message("-------------------------------config thirdparty boost compute---------------------------------")


#######################配置路径########################################
SET(COMPUTECL_INCLUDE ${VCPKG_HOME}/installed/x64-windows/include)
SET(COMPUTECL_LIB ${VCPKG_HOME}/installed/x64-windows/lib)
SET(COMPUTECL_DLIB ${VCPKG_HOME}/installed/x64-windows/debug/lib)


#######################配置头文件######################################
include_directories(${COMPUTECL_INCLUDE})
include_directories(${COMPUTECL_INCLUDE}/boost)
include_directories(${COMPUTECL_INCLUDE}/compute)
include_directories(${THIRDPARTY_ROOT}/opencl)
INCLUDE(${THIRDPARTY_CMAKE}/opencl.cmake)

########################链接库#########################################

link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_atomic-vc140-mt-gd.lib)

link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_math_c99l-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_serialization-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_thread-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_system-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_filesystem-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_iostreams-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_container-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_container-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_exception-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_exception-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_context-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_context-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_exception-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_exception-vc140-mt-gd.lib)
link_libraries(optimized ${COMPUTECL_LIB}/boost_exception-vc140-mt.lib debug ${COMPUTECL_DLIB}/boost_exception-vc140-mt-gd.lib)