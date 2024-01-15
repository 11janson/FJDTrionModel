########################################################
## ����Դ�ļ�

include(${THIRDPARTY_ROOT}/cswidgets/cswidgets.cmake)
########################################################
## ��Ŀ����
SET (PROJECT_FOLDER 
    ccqfc
    )

## ���ɶ�̬��
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)

# 配置头文件
target_include_directories(${CS_PROJECT_NAME} PUBLIC ${THIRDPARTY_ROOT}/cswidgets)

colormsg(_HIBLUE_ "---End---")