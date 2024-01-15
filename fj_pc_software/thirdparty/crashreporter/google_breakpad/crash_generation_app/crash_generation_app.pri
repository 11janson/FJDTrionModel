SOURCES += \
	$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/abstract_class.cc \
	$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/crash_generation_app.cc
HEADERS += \
	$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/crash_generation_app.h
OTHER_FILES +=
	$$PWD/crash_generation_app.cmake \
	$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/small.ico \
	$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/crash_generation_app.ico
)
win32{
	OTHER_FILES += \
		$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/crash_generation_app.rc
	HEADERS += \
		$$GBREAKPAD_ROOT/client/windows/tests/crash_generation_app/resource.h
}
win32{
	LIBS *= -lWininet
}

