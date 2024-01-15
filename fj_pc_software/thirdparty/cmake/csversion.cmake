################################################################################

# Defines version information for 
#
# cs_version(MAJOR major_version MINOR minor_version PATCH patch_level)
#
#    MAJOR.MINOR version is used to set SOVERSION

macro(cs_version)
    parse_arguments(THIS_VERSION "MAJOR;MINOR;PATCH;"
    ""
    ${ARGN})
    string(TOUPPER "${PROJECT_NAME}" PROJECTNAMEU)
    string(TOLOWER "${PROJECT_NAME}" PROJECTNAMEL)
     
    # Set version components
    set(${PROJECTNAMEU}_VERSION_MAJOR ${THIS_VERSION_MAJOR})
    set(${PROJECTNAMEU}_VERSION_MINOR ${THIS_VERSION_MINOR}) 
    set(${PROJECTNAMEU}_VERSION_PATCH ${THIS_VERSION_PATCH})

    # Set VERSION string
    set(${PROJECTNAMEU}_VERSION
    ${${PROJECTNAMEU}_VERSION_MAJOR}.${${PROJECTNAMEU}_VERSION_MINOR}.${${PROJECTNAMEU}_VERSION_PATCH})

    # Set SOVERSION based on major and minor
    set(${PROJECTNAMEU}_SOVERSION
    ${${PROJECTNAMEU}_VERSION_MAJOR}.${${PROJECTNAMEU}_VERSION_MINOR})

    # Set ABI version string used to name binary output and, by RMS loader, to find binaries.
    # On Windows, ABI version is specified using binary file name suffix.
    # On Unix, suffix ix empty and SOVERSION is used instead.
    if (UNIX)
        set(${PROJECTNAMEU}_ABI_VERSION 
            ${${PROJECTNAMEU}_SOVERSION})
    elseif(WIN32)
        set(${PROJECTNAMEU}_ABI_VERSION
            ${${PROJECTNAMEU}_VERSION_MAJOR}_${${PROJECTNAMEU}_VERSION_MINOR}
        )
    else()
        message(FATAL_ERROR "Ambiguous target platform with unknown ABI version scheme. Giving up.")
    endif()

    boost_report_value(${PROJECTNAMEU}_VERSION)
    boost_report_value(${PROJECTNAMEU}_ABI_VERSION)
    ## 目标名称
    SET(TARGET_U
        ${PROJECTNAMEU}
    )
    SET(TARGET_L
        ${PROJECTNAMEL}
    )
    SET(TARGET
        ${PROJECT_NAME}
    )
    ADD_DEFINITIONS(-DTARGET_U="${TARGET_U}")
    ADD_DEFINITIONS(-DTARGET_L="${TARGET_L}")
    ADD_DEFINITIONS(-DTARGET="${TARGET}")
    add_definitions(-D${PROJECTNAMEU}_ABI_VERSION=${${PROJECTNAMEU}_ABI_VERSION})
    add_definitions(-D${PROJECTNAMEU}_VERSION=${${PROJECTNAMEU}_VERSION})
    add_definitions(-D${PROJECTNAMEU}_VERSION_MAJOR=${${PROJECTNAMEU}_VERSION_MAJOR})
    add_definitions(-D${PROJECTNAMEU}_VERSION_MINOR=${${PROJECTNAMEU}_VERSION_MINOR})
    add_definitions(-D${PROJECTNAMEU}_VERSION_PATCH=${${PROJECTNAMEU}_VERSION_PATCH})
endmacro()


# cs_version(MAJOR major_version MINOR minor_version PATCH patch_level)
#
#    MAJOR.MINOR version is used to set SOVERSION
#
macro(cs_version_file)
  # get version from version.h
  file(
    STRINGS
    "${PROJECT_SOURCE_DIR}/version.h"
    _VERSION
    REGEX
    "#define CS_VERSION ([0-9]+)"
  )
  string(REGEX MATCH "([0-9]+)" _VERSION "${_VERSION}")

  math(EXPR ${PROJECTNAMEU}_VERSION_MAJOR "${_VERSION} / 100000")
  math(EXPR ${PROJECTNAMEU}_VERSION_MINOR "${_VERSION} / 100 % 1000")
  math(EXPR ${PROJECTNAMEU}_VERSION_PATCH "${_VERSION} % 100")

  # Set VERSION string
  set(${PROJECTNAMEU}_VERSION
    "${${PROJECTNAMEU}_VERSION_MAJOR}.${${PROJECTNAMEU}_VERSION_MINOR}.${${PROJECTNAMEU}_VERSION_PATCH}")

  # Set SOVERSION based on major and minor
  set(${PROJECTNAMEU}_SOVERSION
    "${${PROJECTNAMEU}_VERSION_MAJOR}.${${PROJECTNAMEU}_VERSION_MINOR}")

  # Set ABI version string used to name binary output and, by SOCI loader, to find binaries.
  # On Windows, ABI version is specified using binary file name suffix.
  # On Unix, suffix ix empty and SOVERSION is used instead.
  if (UNIX)
    set(${PROJECTNAMEU}_ABI_VERSION ${${PROJECTNAMEU}_SOVERSION})
  elseif(WIN32)
    set(${PROJECTNAMEU}_ABI_VERSION
      "${${PROJECTNAMEU}_VERSION_MAJOR}_${${PROJECTNAMEU}_VERSION_MINOR}")
  else()
    message(FATAL_ERROR "Ambiguous target platform with unknown ABI version scheme. Giving up.")
  endif()

  boost_report_value(${PROJECTNAMEU}_VERSION)
  boost_report_value(${PROJECTNAMEU}_ABI_VERSION)

  add_definitions(-D${PROJECTNAMEU}_ABI_VERSION="${${PROJECTNAMEU}_ABI_VERSION}")

endmacro()