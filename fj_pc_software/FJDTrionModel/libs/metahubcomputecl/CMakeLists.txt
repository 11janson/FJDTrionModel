########################################################
## ����Դ�ļ�

include(${CMAKE_CURRENT_LIST_DIR}/metahubcomputecl.cmake)
########################################################
## ��Ŀ����
SET (PROJECT_FOLDER 
    libs
    )

## ���ɶ�̬��
CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)

target_link_libraries(${CS_PROJECT_NAME} PRIVATE
    ${Boost_LIBRARIES})

colormsg(_HIBLUE_ "---End---")