MESSAGE("set encryptionutils library config ---")
SET(ENCRYPTIONUTILS_HOME ${THIRDPARTY_ROOT}/encryptionutils/sdk)

include_directories(${ENCRYPTIONUTILS_HOME}/include)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
	  link_directories(${ENCRYPTIONUTILS_HOME}/debug)
	 link_libraries(csencryption)
	 file(COPY ${ENCRYPTIONUTILS_HOME}/debug/csencryptiond.lib DESTINATION ${LIB_DIR} )
	 file(COPY ${ENCRYPTIONUTILS_HOME}/debug/csencryptiond.dll DESTINATION ${BIN_DIR} )
	 file(COPY ${ENCRYPTIONUTILS_HOME}/debug/EncryptionUtils.dll DESTINATION ${BIN_DIR} )

ELSE()
	 link_directories(${ENCRYPTIONUTILS_HOME}/release)
	 link_libraries(csencryption)
	 file(COPY ${ENCRYPTIONUTILS_HOME}/release/csencryption.lib DESTINATION ${LIB_DIR} )
	 file(COPY ${ENCRYPTIONUTILS_HOME}/release/csencryption.dll DESTINATION ${BIN_DIR} )
	 file(COPY ${ENCRYPTIONUTILS_HOME}/release/EncryptionUtils.dll DESTINATION ${BIN_DIR} )

ENDIF()

