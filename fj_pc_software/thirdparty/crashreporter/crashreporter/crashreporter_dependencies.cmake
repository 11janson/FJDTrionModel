#
# 项目名称
# 
SET(CS_PROJECT_NAME CrashReporter)
#
# 依赖非Qt库，非本项库
#
set(DEPENDENCIES
    )
IF (WIN32)
    set(DEPENDENCIES
        ${DEPENDENCIES}
        ws2_32 
        kernel32 
        user32 
        shell32 
        uuid 
        ole32 
        advapi32
        )
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