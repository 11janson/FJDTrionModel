DEFINES += QUAZIP_LIBRARY

INCLUDEPATH += $$PWD/zlib
DEPENDPATH += $$PWD/zlib

HEADERS += \
    $$PWD/crypt.h \
    $$PWD/ioapi.h \
    $$PWD/JlCompress.h \
    $$PWD/quaadler32.h \
    $$PWD/quachecksum32.h \
    $$PWD/quacrc32.h \
    $$PWD/quagzipfile.h \
    $$PWD/quaziodevice.h \
    $$PWD/quazip.h \
    $$PWD/quazip_global.h \
    $$PWD/quazipdir.h \
    $$PWD/quazipfile.h \
    $$PWD/quazipfileinfo.h \
    $$PWD/quazipnewinfo.h \
    $$PWD/unzip.h \
    $$PWD/zip.h \
    $$PWD/zlib/crc32.h \
    $$PWD/zlib/deflate.h \
    $$PWD/zlib/gzguts.h \
    $$PWD/zlib/inffast.h \
    $$PWD/zlib/inffixed.h \
    $$PWD/zlib/inflate.h \
    $$PWD/zlib/inftrees.h \
    $$PWD/zlib/trees.h \
    $$PWD/zlib/zconf.h \
    $$PWD/zlib/zlib.h \
    $$PWD/zlib/zutil.h

SOURCES += \
    $$PWD/JlCompress.cpp \
    $$PWD/qioapi.cpp \
    $$PWD/quaadler32.cpp \
    $$PWD/quacrc32.cpp \
    $$PWD/quagzipfile.cpp \
    $$PWD/quaziodevice.cpp \
    $$PWD/quazip.cpp \
    $$PWD/quazipdir.cpp \
    $$PWD/quazipfile.cpp \
    $$PWD/quazipfileinfo.cpp \
    $$PWD/quazipnewinfo.cpp \
    $$PWD/unzip.c \
    $$PWD/zip.c \
    $$PWD/zlib/adler32.c \
    $$PWD/zlib/compress.c \
    $$PWD/zlib/crc32.c \
    $$PWD/zlib/deflate.c \
    $$PWD/zlib/gzclose.c \
    $$PWD/zlib/gzlib.c \
    $$PWD/zlib/gzread.c \
    $$PWD/zlib/gzwrite.c \
    $$PWD/zlib/infback.c \
    $$PWD/zlib/inffast.c \
    $$PWD/zlib/inflate.c \
    $$PWD/zlib/inftrees.c \
    $$PWD/zlib/trees.c \
    $$PWD/zlib/uncompr.c \
    $$PWD/zlib/zutil.c

DISTFILES +=
