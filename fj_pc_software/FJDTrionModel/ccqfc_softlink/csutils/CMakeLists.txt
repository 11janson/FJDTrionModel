########################################################
## 包含源文件
include(${THIRDPARTY_ROOT}/csutils/csutils.cmake)
########################################################

#list(APPEND CMAKE_INCLUDE_PATH "${THIRDPARTY_ROOT}/csutils")

## 项目分组
SET (PROJECT_FOLDER 
    ccqfc
    )
## 生成的目标
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")