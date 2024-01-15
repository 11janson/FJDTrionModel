colormsg(_HIBLUE_ "---Start---Configuring  sentinelldkencrypt library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/sentinelldkencrypt_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()


# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")
#####################################################
# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
#####################################################


#####################################################
# Resource files.
SET (RESOURCES
)
########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/sentinelldkencrypt_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/sentinelldkencrypt.cmake
)

#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )



#######################依赖库##############################
include_directories(${THIRDPARTY_ROOT}/sentinelldkencrypt)
link_directories(${THIRDPARTY_ROOT}/sentinelldkencrypt)
link_libraries(optimized libhasp_windows_x64_28714 debug libhasp_windows_x64_28714)
link_libraries(optimized hasp_windows_x64_28714 debug hasp_windows_x64_28714)

file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/hasp_windows_x64_28714.lib DESTINATION ${LIBRARY_OUTPUT_PATH} )
file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/libhasp_windows_x64_28714.lib DESTINATION ${LIBRARY_OUTPUT_PATH} )

file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/hasp_windows_x64_28714.dll DESTINATION ${BIN_ROOT}/Release )
file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/haspvlib_28714.dll DESTINATION ${BIN_ROOT}/Release )
file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/hasp_windows_x64_28714.dll DESTINATION ${BIN_ROOT}/Debug )
file(COPY ${THIRDPARTY_ROOT}/sentinelldkencrypt/haspvlib_28714.dll DESTINATION ${BIN_ROOT}/Debug )