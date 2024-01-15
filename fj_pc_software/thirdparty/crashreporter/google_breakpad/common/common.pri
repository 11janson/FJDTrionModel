SOURCES += \
    $$GBREAKPAD_ROOT/common/windows/guid_string.cc \
    $$GBREAKPAD_ROOT/common/windows/string_utils.cc \
    $$GBREAKPAD_ROOT/common/windows/http_upload.cc
)
HEADERS += \
    $$GBREAKPAD_ROOT/common/windows/guid_string.h \
    $$GBREAKPAD_ROOT/common/windows/http_upload.h
)
OTHER_FILES += \
    $$PWD/common.pri
)
win32{
	LIBS *= -lWininet
}
