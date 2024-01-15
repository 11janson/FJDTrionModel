#pragma once
#include "cloudcompareutils_global.h"
#include <QVariantMap>

namespace Utils{
class TRIONMETAHUBUTILS_EXPORT Theming
{
public:
    static const QVariantMap &theme();
    static QString replaceCssColors(const QString &input);
    static QString replaceCssImages(const QString &input);
};
}
