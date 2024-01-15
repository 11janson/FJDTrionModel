SOURCES += \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/client_info.cc \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/crash_generation_server.cc \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/minidump_generator.cc
HEADERS += \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/crash_generation_server.h \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/client_info.h \
	$$GBREAKPAD_ROOT/client/windows/crash_generation/minidump_generator.h
OTHER_FILES +=
	$$PWD/crash_generation_server.pri