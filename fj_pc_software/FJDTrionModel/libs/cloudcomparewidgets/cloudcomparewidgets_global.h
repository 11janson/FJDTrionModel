#ifndef TRIONMETAHUBWIDGETS_GLOBAL_H
#define TRIONMETAHUBWIDGETS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(TRIONMETAHUBWIDGETS_LIBRARY)
#  define TRIONMETAHUBWIDGETS_EXPORT Q_DECL_EXPORT
#else
#  define TRIONMETAHUBWIDGETS_EXPORT Q_DECL_IMPORT
#endif

#endif // TRIONMETAHUBWIDGETS_GLOBAL_H
