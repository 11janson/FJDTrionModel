#######################################################
include(${CMAKE_CURRENT_LIST_DIR}/modelcontrol.cmake)
########################################################


SET (PROJECT_FOLDER 
    libs
    )


CS_QT_INIT_LIBRARY_MODULE(TRUE FALSE)
SET(${PROJECTNAMEU}_TARGET_LINK ${${PROJECTNAMEU}_TARGET_LINK} PARENT_SCOPE)

target_link_libraries(${CS_PROJECT_NAME} PUBLIC 
QPCL_2ccCloud_LIB
QCANUPO_PLUGIN
)
##添加公共库
find_package(PROJ4 REQUIRED) #地理信息
find_package(yaml-cpp REQUIRED)
set(CMAKE_PREFIX_PATH ${VCPKG_HOME}/installed/x64-windows/share/ceres 
                      ${VCPKG_HOME}/installed/x64-windows/share/eigen3
                      ${VCPKG_HOME}/installed/x64-windows/share/glog  
                      ${VCPKG_HOME}/installed/x64-windows/share/gflags  
                      ${VCPKG_HOME}/installed/x64-windows/share
                      ${CMAKE_PREFIX_PATH})
message( 1111111111111111111122222222222222222222 ${VCPKG_HOME}/installed/x64-windows/share/ceres)
find_package(Ceres REQUIRED)
target_link_libraries(${CS_PROJECT_NAME} PUBLIC PROJ4::proj yaml-cpp Ceres::ceres)
target_include_directories(${CS_PROJECT_NAME} PUBLIC ${PROJ4_INCLUDE_DIRS})
target_include_directories(${CS_PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

colormsg(_HIBLUE_ "---End---")

