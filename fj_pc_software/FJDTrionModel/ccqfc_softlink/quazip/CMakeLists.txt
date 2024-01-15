########################################################
include(${THIRDPARTY_ROOT}/quazip/quazip.cmake)
########################################################
## 项目分组
SET (PROJECT_FOLDER 
    ccqfc
    )
## 生成动态库
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")
