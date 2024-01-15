################################################################################
MESSAGE("set saribbon library config")

#################配置路径#####################
SET(SARIBBON_INCLUDE ${LIBS_HOME}/Saribbon/src/SARibbonBar)


#################配置头文件###################
include_directories(${SARIBBON_INCLUDE})
################配置库目录路径##############

#################链接库####################
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/Release/SARibbonBar.lib debug ${LIBRARY_OUTPUT_PATH}/Debug/SARibbonBard.lib)












