/****************************************************************************
**
****************************************************************************/

#pragma once

// #include <QLoggingCategory>
#include <qglobal.h>

#if defined(CSWIDGETS_LIBRARY)
#  define CSWIDGETS_EXPORT Q_DECL_EXPORT
#else
#  define CSWIDGETS_EXPORT Q_DECL_IMPORT
#endif

// Q_DECLARE_LOGGING_CATEGORY(pluginLog)
