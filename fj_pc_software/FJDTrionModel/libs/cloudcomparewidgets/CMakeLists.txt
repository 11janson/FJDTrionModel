########################################################
## ����Դ�ļ�

include(${CMAKE_CURRENT_LIST_DIR}/cloudcomparewidgets.cmake)

## 项目分组
SET (PROJECT_FOLDER 
    libs
    )
    
## 生成动态库
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)

target_link_libraries(${CS_PROJECT_NAME} PRIVATE yaml-cpp)

colormsg(_HIBLUE_ "---End---")