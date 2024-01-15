#######################################config thirdparty encryption sdk###################################
message("-------------------------------config thirdparty encryption sdk---------------------------------")


#################配置路径#####################
SET(ENCRYPTION_INCLUDE ${THIRDPARTY_ROOT}/encryption/src/encryptclr/encryptclr)
SET(ENCRYPTION_LIB ${THIRDPARTY_ROOT}/encryption/bin)

#################配置头文件###################
include_directories(${ENCRYPTION_INCLUDE})


#################链接库####################
link_libraries(optimized ${ENCRYPTION_LIB}/release/encryptclr.lib debug ${ENCRYPTION_LIB}/debug/encryptclrd.lib)

file(COPY ${ENCRYPTION_LIB}/debug/encrypt.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug )
file(COPY ${ENCRYPTION_LIB}/debug/encryptclrd.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug )
file(COPY ${ENCRYPTION_LIB}/release/encrypt.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release )
file(COPY ${ENCRYPTION_LIB}/release/encryptclr.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release )
	 
