########################################################
## ����Դ�ļ�

include(${THIRDPARTY_ROOT}/qpdflib/qpdflib.cmake)
########################################################
## ��Ŀ����
SET (PROJECT_FOLDER 
    ccqfc
    )

## ���ɶ�̬��
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)
colormsg(_HIBLUE_ "---End---")