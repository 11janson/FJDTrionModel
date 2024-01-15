########################################################
## 包含源文件
include(${THIRDPARTY_ROOT}/nqspdlog/nqspdlog.cmake)
########################################################
## 生成的目标
if(BUILD_STATIC_NQSPDLOG_LIB)
    ADD_DEFINITIONS(-DNQSPDLOG_STATIC)
    ## 项目分组
    SET (PROJECT_FOLDER 
        ccqfc
        )
    ## 生成静态库
    # LOG4QT_STATIC must also be set when linking against the lib
    CS_QT_INIT_STATIC_MODULE(TRUE FALSE)
else()
    ADD_DEFINITIONS(-DNQSPDLOG_STATIC)
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
