#############################################################################################################################
message("-----------------------------config ccdb_dependencies-----------------------------------")
##############################配置##############################################################
#######################配置头文件###############################################################
include_directories(${LIBS_HOME}/qCC_db/include)
include_directories(${LIBRARY_OUTPUT_PATH}/../libs/qCC_db/extern/CCCoreLib/exports)
include_directories(${LIBS_HOME}/qCC_db/extern/CCCoreLib/include)

#添加配置定义
ADD_DEFINITIONS(-DCC_CORE_LIB_USES_FLOAT)
ADD_DEFINITIONS(-DCC_CORE_LIB_USES_QT_CONCURRENT)


include_directories(${LIBS_HOME}/CCFbo/include)
include_directories(${LIBS_HOME}/qCC_io/include)
include_directories(${LIBS_HOME}/qCC_io/extern/dxflib/src)
include_directories(${LIBS_HOME}/qCC_io/extern/shapelib)



#################链接库####################
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/QFJD_DB_LIB.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/QFJD_DB_LIBd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/FJDCoreLib.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/FJDCoreLibd.lib)

