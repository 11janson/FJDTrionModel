########################################################
## 包含源文件

include(${CMAKE_CURRENT_LIST_DIR}/cloudcompareres.cmake)
########################################################
## 项目分组
SET (PROJECT_FOLDER 
    libs
    )

## 生成动态库
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")