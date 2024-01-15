#######################################config thirdparty cos sdk###################################
message("-------------------------------config thirdparty cos sdk---------------------------------")

SET(COS_INCLUDE ${THIRDPARTY_ROOT}/cos/include)
SET(COS_LIB ${THIRDPARTY_ROOT}/cos/lib)
SET(COS_BIN ${THIRDPARTY_ROOT}/cos/bin)
#
#
##########################配置头文件包含路径
include_directories(${COS_INCLUDE})
include_directories(${COS_INCLUDE}/gtest)
include_directories(${COS_INCLUDE}/op)
include_directories(${COS_INCLUDE}/openssl)
#include_directories(${COS_INCLUDE}/Poco)
include_directories(${COS_INCLUDE}/rapidxml)
include_directories(${COS_INCLUDE}/request)
include_directories(${COS_INCLUDE}/response)
include_directories(${COS_INCLUDE}/trsf)
include_directories(${COS_INCLUDE}/util)


#################链接库####################
link_libraries(optimized ${COS_LIB}/cossdk.lib debug ${COS_LIB}/cossdk.lib)
link_libraries(optimized ${COS_LIB}/gtest.lib debug ${COS_LIB}/gtest.lib)
link_libraries(optimized ${COS_LIB}/gtest_main.lib debug ${COS_LIB}/gtest_main.lib)
link_libraries(optimized ${COS_LIB}/libcrypto.lib debug ${COS_LIB}/libcrypto.lib)
link_libraries(optimized ${COS_LIB}/libssl.lib debug ${COS_LIB}/libssl.lib)
link_libraries(optimized ${COS_LIB}/openssl.lib debug ${COS_LIB}/openssl.lib)
link_libraries(optimized ${COS_LIB}/PocoCrypto.lib debug ${COS_LIB}/PocoCrypto.lib)
link_libraries(optimized ${COS_LIB}/PocoFoundation.lib debug ${COS_LIB}/PocoFoundation.lib)
link_libraries(optimized ${COS_LIB}/PocoJSON.lib debug ${COS_LIB}/PocoJSON.lib)
link_libraries(optimized ${COS_LIB}/PocoNet.lib debug ${COS_LIB}/PocoNet.lib)
link_libraries(optimized ${COS_LIB}/PocoNetSSL.lib debug ${COS_LIB}/PocoNetSSL.lib)
link_libraries(optimized ${COS_LIB}/PocoUtil.lib debug ${COS_LIB}/PocoUtil.lib)
link_libraries(optimized ${COS_LIB}/PocoXML.lib debug ${COS_LIB}/PocoXML.lib)

#############拷贝动态库到运行目录下面
file(COPY ${COS_BIN}/libcrypto-1_1-x64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/libcrypto-1_1-x64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/libssl-1_1-x64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/libssl-1_1-x64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoCrypto.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoCrypto.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoFoundation.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoFoundation.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoJSON.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoJSON.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoNet.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoNet.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoNetSSL.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoNetSSL.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoUtil.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoUtil.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
file(COPY ${COS_BIN}/PocoXML.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
file(COPY ${COS_BIN}/PocoXML.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)



