#ifndef CLOUDCOMPARERES_GLOBAL_H
#define CLOUDCOMPARERES_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CLOUDCOMPARERES_LIBRARY)
#  define CLOUDCOMPARERES_EXPORT Q_DECL_EXPORT
#else
#  define CLOUDCOMPARERES_EXPORT Q_DECL_IMPORT
#endif

#endif // CLOUDCOMPARERES_GLOBAL_H
