colormsg(_HIBLUE_ "---Start---Configuring NCQFC nqspdlog library:")
#############################################################
## 加载项目名称和项目依赖项的文件
INCLUDE(${CMAKE_CURRENT_LIST_DIR}/nqspdlog_dependencies.cmake)
## 项目名称
PROJECT(${CS_PROJECT_NAME})

IF(MSVC)
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ELSE()
    ADD_DEFINITIONS(-DIDE_TEST_DIR="${PROJECT_SOURCE_DIR}")
ENDIF()

# Source files.
FILE(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
#####################################################
# Header files.
FILE(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_LIST_DIR}/*.h")

########################################################
# Other files
SET(OTHER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/nqspdlog_dependencies.cmake
    ${CMAKE_CURRENT_LIST_DIR}/nqspdlog.cmake
    )
