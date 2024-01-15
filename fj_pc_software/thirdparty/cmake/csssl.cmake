# Usage: 增加项目 cmake 框架对camera sdk库的支持
# CSCRYPTO_INCLUDE_DIR, where to find camera sdk headers
# CSCRYPTO_LIBRARIES,  Where to find dependencies
# CSCRYPTO_FOUND, If false, do not try to use camera sdk

MESSAGE("set ssl config ---")
IF (WIN32)

SET(SSL_HOME ${SSL_HOME} CACHE PATH  "camera crypto root path")
IF (NOT (EXISTS "${SSL_HOME}"))
    SET(SSL_HOME
        ${THIRDPARTY_ROOT}/ssl
    )
    IF (NOT (EXISTS "${SSL_HOME}"))
        MESSAGE(FATAL_ERROR "Please set the camera sdk library dir first. -DSSL_HOME=XXXX ")
    ENDIF()
ENDIF()

IF (MSVC)
	set(PROJECT_PATH "${PROJECT_PATH}" "${SSL_HOME};")
	set(PROJECT_PATH ${PROJECT_PATH} PARENT_SCOPE)
ENDIF()

ENDIF()