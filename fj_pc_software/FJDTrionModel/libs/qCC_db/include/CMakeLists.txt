file(GLOB headerFiles CMAKE_CONFIGURE_DEPENDS *.h )
target_sources( ${PROJECT_NAME}
	PRIVATE ${headerFiles})

target_include_directories( ${PROJECT_NAME}
	PUBLIC
	    ${CMAKE_CURRENT_LIST_DIR}
)
