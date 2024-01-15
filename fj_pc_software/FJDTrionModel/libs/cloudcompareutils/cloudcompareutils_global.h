#ifndef TRIONMETAHUBUTILS_GLOBAL_H
#define TRIONMETAHUBUTILS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(TRIONMETAHUBUTILS_LIBRARY)
#  define TRIONMETAHUBUTILS_EXPORT Q_DECL_EXPORT
#else
#  define TRIONMETAHUBUTILS_EXPORT Q_DECL_IMPORT
#endif

#endif // TRIONMETAHUBUTILS_GLOBAL_H
