#############################################################################################################################
message("-----------------------------config libs dependencies-----------------------------------")
##############################配置##############################################################
#######################配置头文件###############################################################
include_directories(${LIBS_HOME})


#################链接库####################
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/modelcontrol.lib debug ${LIBRARY_OUTPUT_PATH}/modelcontrold.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/graphicsviewvector.lib debug ${LIBRARY_OUTPUT_PATH}/graphicsviewvectord.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/model.lib debug ${LIBRARY_OUTPUT_PATH}/modeld.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubres.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubresd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubutils.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubutilsd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/trionmetahubwidgets.lib debug ${LIBRARY_OUTPUT_PATH}/trionmetahubwidgetsd.lib)
link_libraries(optimized ${LIBRARY_OUTPUT_PATH}/registeractivate.lib debug ${LIBRARY_OUTPUT_PATH}/registeractivated.lib)
