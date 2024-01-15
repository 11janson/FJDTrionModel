#######################################config thirdparty curl sdk###################################
message("-------------------------------config thirdparty curl sdk---------------------------------")



#################配置路径#####################
SET(CURL_INCLUDE ${THIRDPARTY_ROOT}/curl/include)
SET(CURL_LIB ${THIRDPARTY_ROOT}/curl/bin)

#################配置头文件###################
include_directories(${CURL_INCLUDE})




#################链接库####################
link_libraries(optimized ${CURL_LIB}/libcurl_imp.lib debug ${CURL_LIB}/libcurl-d_imp.lib)


file(COPY ${CURL_LIB}/libcurl-d.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug )
file(COPY ${CURL_LIB}/libcurl.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release )
