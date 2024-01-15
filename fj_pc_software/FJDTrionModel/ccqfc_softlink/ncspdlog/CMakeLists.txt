########################################################
## 包含源文�?
include(${THIRDPARTY_ROOT}/ncspdlog/ncspdlog.cmake)
########################################################
## 生成的目�?
if(BUILD_STATIC_NCSPDLOG_LIB)
    ADD_DEFINITIONS(-DNCSPDLOG_STATIC)
    ## 项目分组
    SET (PROJECT_FOLDER 
        ccqfc
        )
    ## 生成静态库
    # LOG4QT_STATIC must also be set when linking against the lib
    CS_QT_INIT_STATIC_MODULE(TRUE FALSE)
else()
    ADD_DEFINITIONS(-DNCSPDLOG_STATIC)
    ## 项目分组
    SET (PROJECT_FOLDER 
        ccqfc
        )
    ## 生成插件
    # only set LOG4QT_LIBRARY when building the lib, not when linking against
    CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
endif()

SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")
