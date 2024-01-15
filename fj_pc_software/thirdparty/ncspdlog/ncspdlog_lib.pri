dll {
    DEFINES += NCSPDLOG_LIBRARY
} else {
    DEFINES += NCSPDLOG_STATIC_LIB
}

win32{
    DEFINES += SPDLOG_WCHAR_FILENAMES
}

CONFIG -= qt ### 非Q项目

HEADERS += \
    $$PWD/ncspdlog_global.h \
    $$PWD/consoleformatter.h \
    $$PWD/spdlogbase.h \
    $$PWD/spdlogapplication.h \
    $$PWD/spdlogmodule.h

SOURCES += \
    $$PWD/spdlogbase.cpp \
    $$PWD/spdlogapplication.cpp \
    $$PWD/spdlogmodule.cpp


OTHER_FILES += 


