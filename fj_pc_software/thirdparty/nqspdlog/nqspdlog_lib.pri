dll {
    DEFINES += NQSPDLOG_LIBRARY
} else {
    DEFINES += NQSPDLOG_STATIC_LIB
}

HEADERS += \
    $$PWD/nqspdlog_global.h \
    $$PWD/nqspdlog.h \
    $$PWD/spdlogview.h \
    $$PWD/spdlogview_sink.h

SOURCES += \
    $$PWD/spdlogview.cpp

OTHER_FILES += 


