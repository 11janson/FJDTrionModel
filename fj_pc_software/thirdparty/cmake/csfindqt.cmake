﻿OPTION(USE_QT4 "Use Qt 4.x.x lib " OFF)
OPTION(USE_QT5 "Use Qt 5.x.x lib " ON)
IF(USE_QT4 AND USE_QT5)
  MESSAGE(FATAL_ERROR "Only choose one Qt Version")
ENDIF()
IF(NOT USE_QT4 AND NOT USE_QT5)
    MESSAGE(FATAL_ERROR "Must choose Qt Version")
ENDIF()

IF ("$Qt5_DIR}" STREQUAL "")
	SET(QT_DIR    ## Qt安装路径的环境变量
    ${QT_DIR} CACHE PATH  "Qt root path")
ELSE()
	SET(QT_DIR ${Qt5_DIR}/../../../)
ENDIF()

IF("${QT_DIR}" STREQUAL "")
    MESSAGE(FATAL_ERROR "Please set the Qt install path first. -DQT_DIR=XXXX ")
ENDIF()
boost_report_value(QT_DIR)
IF(USE_QT5)
    ##    Set Qt 5 Install path 
    SET(CMAKE_PREFIX_PATH
        ${CMAKE_PREFIX_PATH}
        ${QT_DIR}
    )
    #message(STATUS ${CMAKE_PREFIX_PATH})
    ##
    FIND_PACKAGE(Qt5 REQUIRED Core)
    IF(Qt5Core_FOUND)
        boost_report_value(Qt5Core_VERSION_STRING)
        SET(CMAKE_MODULE_PATH 
            ${CMAKE_MODULE_PATH}
            ${QT_DIR}/lib/cmake/Qt5Core 
            )
        ##MESSAGE(STATUS "${CMAKE_MODULE_PATH}")
                #INCLUDE(Qt5CTestMacros)
        SET(QT_INSTALL_BINS ${QT_DIR}/bin)
        SET(QT_INSTALL_TRANSLATIONS ${QT_DIR}/translations)
        SET(QT_INSTALL_PLUGINS ${QT_DIR}/plugins)
        SET(QT_INSTALL_IMPORTS ${QT_DIR}/imports)
        SET(QT_INSTALL_QML ${QT_DIR}/qml)
        ##
        INCLUDE(${THIRDPARTY_CMAKE}/csqtconfig.cmake)
        SET(QT_VERSION_MAJOR ${Qt5Core_VERSION_MAJOR})
        SET(QT_VERSION_MINOR ${Qt5Core_VERSION_MINOR})
        SET(QT_VERSION_PATCH ${Qt5Core_VERSION_PATCH})
    ELSE()
        MESSAGE(FATAL_ERROR "Qt 5 install path invalid!")
    ENDIF()
ENDIF()

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
## Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set_property(GLOBAL PROPERTY USE_FOLDERS ON)
#set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

#######################翻译相关############################################
#
# 是否生成ts文件
#
option(RUN_TS "Do you want to generate TS files (default: on)" OFF)
#
# 是否生成ts文件
#
option(RUN_QM "Do you want to generate QM files (default: on)" OFF)
#
# 翻译的目标语言,空格分割的字符串
#
SET(LANGUAGES "zh_CN en zh_TW ja ru it es" CACHE STRING  "Need to be translated in the target language")
message(${LANGUAGES})


### 高分屏支持
ADD_DEFINITIONS (-DQT_AUTO_SCREEN_SCALE_FACTOR=1)