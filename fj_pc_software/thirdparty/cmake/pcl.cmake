#######################################config thirdparty pcl###################################
message("-------------------------------config thirdparty pcl---------------------------------")


#################配置路径#####################
SET(PCL_INCLUDE ${VCPKG_HOME}/installed/x64-windows/include)
SET(PCL_LIB ${VCPKG_HOME}/installed/x64-windows/lib)
SET(PCL_DLIB ${VCPKG_HOME}/installed/x64-windows/debug/lib)
#################配置头文件###################
include_directories(${PCL_INCLUDE})


link_libraries(optimized ${PCL_LIB}/pcl_common.lib debug ${PCL_DLIB}/pcl_commond.lib)
link_libraries(optimized ${PCL_LIB}/pcl_features.lib debug ${PCL_DLIB}/pcl_featuresd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_filters.lib debug ${PCL_DLIB}/pcl_filtersd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_io.lib debug ${PCL_DLIB}/pcl_iod.lib)
link_libraries(optimized ${PCL_LIB}/pcl_io_ply.lib debug ${PCL_DLIB}/pcl_io_plyd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_kdtree.lib debug ${PCL_DLIB}/pcl_kdtreed.lib)
link_libraries(optimized ${PCL_LIB}/pcl_keypoints.lib debug ${PCL_DLIB}/pcl_keypointsd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_ml.lib debug ${PCL_DLIB}/pcl_mld.lib)
link_libraries(optimized ${PCL_LIB}/pcl_octree.lib debug ${PCL_DLIB}/pcl_octreed.lib)
link_libraries(optimized ${PCL_LIB}/pcl_outofcore.lib debug ${PCL_DLIB}/pcl_outofcored.lib)
link_libraries(optimized ${PCL_LIB}/pcl_people.lib debug ${PCL_DLIB}/pcl_peopled.lib)
link_libraries(optimized ${PCL_LIB}/pcl_recognition.lib debug ${PCL_DLIB}/pcl_recognitiond.lib)
link_libraries(optimized ${PCL_LIB}/pcl_registration.lib debug ${PCL_DLIB}/pcl_registrationd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_sample_consensus.lib debug ${PCL_DLIB}/pcl_sample_consensusd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_search.lib debug ${PCL_DLIB}/pcl_searchd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_segmentation.lib debug ${PCL_DLIB}/pcl_segmentationd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_stereo.lib debug ${PCL_DLIB}/pcl_stereod.lib)
link_libraries(optimized ${PCL_LIB}/pcl_surface.lib debug ${PCL_DLIB}/pcl_surfaced.lib)
link_libraries(optimized ${PCL_LIB}/pcl_tracking.lib debug ${PCL_DLIB}/pcl_trackingd.lib)
link_libraries(optimized ${PCL_LIB}/pcl_visualization.lib debug ${PCL_DLIB}/pcl_visualizationd.lib)
