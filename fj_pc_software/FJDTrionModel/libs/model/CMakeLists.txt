########################################################
## ����Դ�ļ�

include(${CMAKE_CURRENT_LIST_DIR}/model.cmake)
########################################################
## ��Ŀ����
SET (PROJECT_FOLDER 
    libs
    )

## ���ɶ�̬��
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")