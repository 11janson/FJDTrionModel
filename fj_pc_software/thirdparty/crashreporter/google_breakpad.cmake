colormsg(_HIBLUE_ "---Start---Configuring NCQFC google_breakpad resource:")

## 项目名称
#
# 项目名称
# 
SET(CS_PROJECT_NAME crashreporter)
PROJECT(${CS_PROJECT_NAME})
include(${CMAKE_CURRENT_LIST_DIR}/google_breakpad/common/common.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/google_breakpad/crash_generation_client/crash_generation_client.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/google_breakpad/exception_handler/exception_handler.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/google_breakpad/crash_report_sender/crash_report_sender.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/crashhandler/crashhandler.cmake)