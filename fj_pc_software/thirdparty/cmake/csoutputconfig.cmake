# macro that sets a default (path) if one wasn't specified
MACRO(SET_PATH variable default)
    IF(NOT ${variable})
        SET(${variable} ${default})
    ENDIF(NOT ${variable})
ENDMACRO(SET_PATH)
# 生成文件的路径
IF(APPLE)
    # enable @rpath in the install name for any shared library being built
    # note: it is planned that a future version of CMake will enable this by default
    SET(CMAKE_INSTALL_RPATH @loader_path/../Frameworks @loader_path)
    #SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    SET(CMAKE_MACOSX_RPATH ON)
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    #SET(CMAKE_SKIP_BUILD_RPATH TRUE)
    # most paths are irrelevant since the items will be bundled with application
    SET(IDE_APPBUNDLE_NAME ${IDE_TARGET_NAME})
    SET_PATH(IDE_APP_BUNDLE "${CMAKE_BINARY_DIR}/../${IDE_APPBUNDLE_NAME}.app")
    SET_PATH(IDE_OUTPUT_PATH "${IDE_APP_BUNDLE}/Contents")
    SET_PATH(BIN_DIR "${IDE_OUTPUT_PATH}/MacOS")
    SET_PATH(LIB_DIR "${IDE_OUTPUT_PATH}/Frameworks")
    SET_PATH(PLUGIN_DIR "${IDE_OUTPUT_PATH}/Plugins")
    SET_PATH(SHARE_DIR "${IDE_OUTPUT_PATH}/Resources")
    SET(LIBRARY_OUTPUT_PATH ${LIB_DIR})
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/..)
ELSE()
    SET(BIN_DIR         ${CMAKE_BINARY_DIR}/bin/$<CONFIG>)
    SET(LIB_DIR         ${CMAKE_BINARY_DIR}/lib)
	SET(PDB_DIR			${CMAKE_BINARY_DIR}/pdb/$<CONFIG>)
    EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory  ${CMAKE_BINARY_DIR}/bin
            COMMAND ${CMAKE_COMMAND} -E make_directory ${LIB_DIR}
			COMMAND ${CMAKE_COMMAND} -E make_directory  ${CMAKE_BINARY_DIR}/bin/Debug
			COMMAND ${CMAKE_COMMAND} -E make_directory  ${CMAKE_BINARY_DIR}/bin/Release
			COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/pdb
			COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_DIR}/plugins
    )
	
	set(CMAKE_ILK_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	set(CMAKE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/pdb)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

	

	
	SET(LIBRARY_OUTPUT_PATH         ${CMAKE_BINARY_DIR}/lib)
    SET(PDB_OUTPUT_DIR         ${CMAKE_BINARY_DIR}/pdb)
	SET(BIN_OUTPUT_PATH		   ${CMAKE_BINARY_DIR}/bin/$<CONFIG>)
	SET(BIN_ROOT		   ${CMAKE_BINARY_DIR}/bin)
	SET(SHARE_DIR		   ${CMAKE_BINARY_DIR}/bin/$<CONFIG>)
	
	
    EXECUTE_PROCESS(
            COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARY_OUTPUT_PATH}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${PDB_OUTPUT_DIR}
			COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_OUTPUT_PATH}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${BIN_OUTPUT_PATH}/share
			)
    
ENDIF()

