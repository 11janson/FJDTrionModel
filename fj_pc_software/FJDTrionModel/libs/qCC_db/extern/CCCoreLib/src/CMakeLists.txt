# SPDX-License-Identifier: MIT
# Copyright © Andy Maloney <asmaloney@gmail.com>
file(GLOB sources CMAKE_CONFIGURE_DEPENDS *.cpp)

target_sources( ${PROJECT_NAME}
	PRIVATE ${sources})

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
