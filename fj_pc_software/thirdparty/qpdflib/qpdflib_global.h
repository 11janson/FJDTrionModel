#ifndef QPDFLIB_GLOBAL_H
#define QPDFLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(QPDFLIB_LIBRARY)
#  define QPDFLIB_EXPORT Q_DECL_EXPORT
#else
#  define QPDFLIB_EXPORT Q_DECL_IMPORT
#endif

#endif // QPDFLIB_GLOBAL_H
