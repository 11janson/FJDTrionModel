#include "theming.h"
#include "csutils/theme/theme.h"
#include "stylehelper.h"
using namespace Utils;
#include <QRegExp>
#include <QFileInfo>
#include <QDebug>

const QVariantMap &Theming::theme()
{
    static QVariantMap map;
    if (!map.isEmpty())
        return map;

    map = creatorTheme()->values();
    return map;
}

QString Theming::replaceCssColors(const QString &input)
{
    const QVariantMap &map = theme();
    QRegExp rx("chishineTheme.color\\.(\\w+);");

    int pos = 0;
    QString output = input;
    while ((pos = rx.indexIn(output, pos)) != -1)
    {
          const QString color = rx.cap(1);
          output.replace(pos, QString("chishineTheme.color." + rx.cap(1)).length(), map.value(color).toString());
          pos += rx.matchedLength();
    }
    return output;
}

QString Theming::replaceCssImages(const QString &input)
{
    const QVariantMap &map = theme();
    QRegExp rx("chishineTheme\\.image\\.(\\w+)");
    int pos = 0;
    QString output = input;
    while ((pos = rx.indexIn(output, pos)) != -1)
    {
        const QString image = rx.cap(1);		
        QString imageFile = StyleHelper::dpiSpecificImageFile(map.value(image).toString());
        if(!QFileInfo::exists(imageFile))
            qDebug() << rx.cap(1) <<"\t" << imageFile <<" is not exist!" ;
        output.replace(pos,QString("chishineTheme.image." + rx.cap(1)).length(), imageFile);
        pos += rx.matchedLength();
    }
    return output;
}
