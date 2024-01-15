SOURCES += \
    $$GBREAKPAD_ROOT/client/windows/sender/crash_report_sender.cc
HEADERS += \
    $$GBREAKPAD_ROOT/client/windows/sender/crash_report_sender.h
OTHER_FILES += \
    ${CMAKE_CURRENT_LIST_DIR}/crash_report_sender.pri

win32{
	LIBS *= -lWininet
}
