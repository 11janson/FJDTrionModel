
SOURCES += \
    $$PWD/devicecontrol.cpp \
    $$PWD/robotcamerapaircontrol.cpp \
    $$PWD/WeldUtilsParamControl.cpp \
    $$PWD/workpiececameraposecontrol.cpp

HEADERS += \
        modelcontrol_global.h \ 
    $$PWD/devicecontrol.h \
    $$PWD/robotcamerapaircontrol.h \
    $$PWD/WeldUtilsParamControl.h \
    $$PWD/workpiececameraposecontrol.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
