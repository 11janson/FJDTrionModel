#
# 项目名称
# 
SET(CS_PROJECT_NAME csutils)
#
# 依赖非Qt库，非本项库
#
IF(APPLE)
    SET(FRAMEWORKS
        Foundation AppKit
    )
    SET(DEPENDENCIES

     )
ELSE()
    IF(WIN32)
        SET(DEPENDENCIES
            user32 iphlpapi ws2_32 shell32 psapi
        )
    ENDIF()
    IF(MSVC)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -wd4250 -wd4290")
    ENDIF(MSVC)
ENDIF()
#
# 依赖的Qt库
#
SET(QT_MODULES
    Gui
    Widgets
	Network
    )
IF(Qt4_FOUND)

ELSE()

ENDIF()
#
# 依赖的本项目内的库
#
SET(CS_MODULE_DEPS

    )