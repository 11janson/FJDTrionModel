#colormsg(_HIBLUE_ "---Start---Configuring quazip library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/quazip_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})
# Source files.
#FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
#FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")
#####################################################
# UI files
#FILE(GLOB_RECURSE FORMS "${CMAKE_CURRENT_LIST_DIR}/*.ui")

SET(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/JlCompress.cpp
    ${CMAKE_CURRENT_LIST_DIR}/qioapi.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quaadler32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quacrc32.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quagzipfile.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quaziodevice.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quazip.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quazipdir.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quazipfile.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quazipfileinfo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/quazipnewinfo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/unzip.c
    ${CMAKE_CURRENT_LIST_DIR}/zip.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/adler32.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/compress.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/crc32.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/deflate.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/gzclose.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/gzlib.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/gzread.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/gzwrite.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/infback.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inffast.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inflate.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inftrees.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/trees.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/uncompr.c
    ${CMAKE_CURRENT_LIST_DIR}/zlib/zutil.c
    )
SET(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/crypt.h
    ${CMAKE_CURRENT_LIST_DIR}/ioapi.h
    ${CMAKE_CURRENT_LIST_DIR}/JlCompress.h
    ${CMAKE_CURRENT_LIST_DIR}/quaadler32.h
    ${CMAKE_CURRENT_LIST_DIR}/quachecksum32.h
    ${CMAKE_CURRENT_LIST_DIR}/quacrc32.h
    ${CMAKE_CURRENT_LIST_DIR}/quagzipfile.h
    ${CMAKE_CURRENT_LIST_DIR}/quaziodevice.h
    ${CMAKE_CURRENT_LIST_DIR}/quazip.h
    ${CMAKE_CURRENT_LIST_DIR}/quazip_global.h
    ${CMAKE_CURRENT_LIST_DIR}/quazipdir.h
    ${CMAKE_CURRENT_LIST_DIR}/quazipfile.h
    ${CMAKE_CURRENT_LIST_DIR}/quazipfileinfo.h
    ${CMAKE_CURRENT_LIST_DIR}/quazipnewinfo.h
    ${CMAKE_CURRENT_LIST_DIR}/unzip.h
    ${CMAKE_CURRENT_LIST_DIR}/zip.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/crc32.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/deflate.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/gzguts.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inffast.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inffixed.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inflate.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/inftrees.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/trees.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/zconf.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/zlib.h
    ${CMAKE_CURRENT_LIST_DIR}/zlib/zutil.h
    )
# Resource files.
SET (RESOURCES

    )
########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/quazip.cmake
    ${CMAKE_CURRENT_LIST_DIR}/quazip_dependencies.cmake
    )
#####################################################
# 指定当前目录下源文件需要翻译，生成的ts文件会放在当前目录下
SET(TRANSLATE_SOURCE_DIR
    ${CMAKE_CURRENT_LIST_DIR}
    )

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_LIST_DIR}
    #    ${GBREAKPAD_ROOT}/src
    ${CMAKE_CURRENT_LIST_DIR}/zlib
    )
