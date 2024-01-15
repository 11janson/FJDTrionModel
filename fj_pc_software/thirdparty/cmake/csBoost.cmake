# BOOST include directory
SET(BOOST_INCLUDEDIR $ENV{BOOST_INCLUDEDIR} CACHE PATH  "Boost include path ")
# BOOST library directory
SET(BOOST_LIBRARYDIR $ENV{BOOST_LIBRARYDIR} CACHE PATH  "Boost librarydir path ")
IF (NOT (EXISTS "${BOOST_INCLUDEDIR}"))
    ## 查找home目录
	SET(BOOST_HOME $ENV{BOOST_HOME} PATH  "Boost root path ")
	IF (NOT (EXISTS "${BOOST_HOME}"))
		SET(BOOST_HOME
			${THIRDPARTY_ROOT}/boost
		)
		IF (NOT (EXISTS "${BOOST_HOME}"))
			MESSAGE(WARNING "Please set the Boost library dir first. -DBOOST_HOME=XXXX ")
		ELSE()
           ### boost 支持
			SET(BOOST_INCLUDEDIR ${BOOST_HOME}/include)
			SET(BOOST_LIBRARYDIR ${BOOST_HOME}/${OSName}/${CompilerName}/${CompilerArch}/${CompilerVersion})
		ENDIF()
	ELSE()
		# Thrift include directory
		SET(BOOST_INCLUDEDIR ${BOOST_HOME}/include)
		SET(BOOST_LIBRARYDIR ${BOOST_HOME}/lib)
	ENDIF()
ENDIF()


#SET(Boost_HOME $ENV{Boost_HOME} CACHE PATH  "Boost root path ")
#IF (NOT (EXISTS "${Boost_HOME}"))
#	SET(Boost_HOME
#		${THIRDPARTY_ROOT}/boost
#	)
#	IF (NOT (EXISTS "${Boost_HOME}"))
#		MESSAGE(WARNING "Please set the Boost library dir first. -DBOOST_HOME=XXXX ")
#	ENDIF()
#ENDIF()

#### boost 支持
#SET(BOOST_INCLUDEDIR ${Boost_HOME}/include)
#SET(BOOST_LIBRARYDIR ${Boost_HOME}/${OSName}/${CompilerName}/${CompilerArch}/${CompilerVersion})

SET (Boost_USE_STATIC_LIBS ON)
SET (Boost_USE_MULTITHREADED ON)
# 头文件搜索目录
INCLUDE_DIRECTORIES(
	${BOOST_INCLUDEDIR}
)
# 链接库搜索目录
LINK_DIRECTORIES(
	${BOOST_LIBRARYDIR}
)
### 移除Boost的自动链接的规则
ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
FIND_PACKAGE(Boost 1.53 REQUIRED ${BOOST_COMPONENTS})
mark_as_advanced (Boost_DIR)
if (WIN32)
	mark_as_advanced (Boost_LIB_DIAGNOSTIC_DEFINITIONS)
endif ()

ADD_DEFINITIONS(-DBOOST_BEAST_NO_THREAD_LOCAL)