file(GLOB sources CMAKE_CONFIGURE_DEPENDS *.cpp)

target_sources( ${PROJECT_NAME}
	PRIVATE ${sources})