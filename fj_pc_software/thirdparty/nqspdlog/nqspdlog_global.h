#pragma once
#include <QtGlobal>

#if defined(NQSPDLOG_LIBRARY)
#  define NQSPDLOG_EXPORT Q_DECL_EXPORT
#elif  defined(NQSPDLOG_STATIC_LIB) // Abuse single files for manual tests
#  define NQSPDLOG_EXPORT
#else
#  define NQSPDLOG_EXPORT Q_DECL_IMPORT
#endif


