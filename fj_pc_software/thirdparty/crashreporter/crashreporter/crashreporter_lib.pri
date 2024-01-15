QT += network
SOURCES += \
    $$PWD/*.cpp 

HEADERS  += \
    $$PWD/*.h

FORMS += \
    $$PWD/*.ui 

RESOURCES +=

DISTFILES += \
    $$PWD/version.h.in

QMAKE_SUBSTITUTES += \
    $$PWD/version.h.in

CONFIG += no_batch

INCLUDEPATH += \
    $${PWD} \
    $$IDE_OUTPUT_PATH/ncqfc/crashreporter/crashreporter

win32 {
    LIBS += -lws2_32 -lkernel32 -luser32 -lshell32 -luuid -lole32 -ladvapi32
}

unix:!macx {
    LIBS += -Wl,-rpath=\\\$$ORIGIN/../lib #don't remove!!!
    CONFIG += static
    QMAKE_LFLAGS += -static-libgcc -static-libstdc++
}
