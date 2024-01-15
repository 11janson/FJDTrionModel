
# 设置配置路径
set(OPENCV_WORLD_LIB ${THIRDPARTY_ROOT}/opencv_world)

# 链接库
link_libraries(optimized ${OPENCV_WORLD_LIB}/opencv_world3416.lib debug ${OPENCV_WORLD_LIB}/opencv_world3416d.lib)
file(COPY ${OPENCV_WORLD_LIB}/opencv_world3416d.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug )
file(COPY ${OPENCV_WORLD_LIB}/opencv_ffmpeg3416_64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug )

file(COPY ${OPENCV_WORLD_LIB}/opencv_world3416.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release )
file(COPY ${OPENCV_WORLD_LIB}/opencv_ffmpeg3416_64.dll DESTINATION ${CMAKE_BINARY_DIR}/bin/Release )