#
# 项目名称
# 
SET(CS_PROJECT_NAME csextensionsystem)
#
# 依赖非Qt库，非本项库
#
set(DEPENDENCIES

    )
#
# 依赖的Qt库
#
SET(QT_MODULES
    Gui
    Widgets
    )
IF(Qt4_FOUND)

ELSE()

ENDIF()
#
# 依赖的本项目内的库
#
SET(CS_MODULE_DEPS
    csaggregation
    csutils
    )