#pragma once

#include "utils_global.h"

#include <QMetaType>
#include <QString>

QT_BEGIN_NAMESPACE
class QDataStream;
class QVariant;
QT_END_NAMESPACE
namespace Utils {

class CSUTILS_EXPORT Id
{
public:
    Id() : m_id(0) {}
    Id(const char *name); // Good to use.

    Id withSuffix(int suffix) const;
    Id withSuffix(const char *suffix) const;
    Id withSuffix(const QString &suffix) const;
    Id withPrefix(const char *prefix) const;

    QByteArray name() const;
    QString toString() const; // Avoid.
    QVariant toSetting() const; // Good to use.
    QString suffixAfter(Id baseId) const;
    bool isValid() const { return m_id; }
    bool operator==(Id id) const { return m_id == id.m_id; }
    bool operator==(const char *name) const;
    bool operator!=(Id id) const { return m_id != id.m_id; }
    bool operator!=(const char *name) const { return !operator==(name); }
    bool operator<(Id id) const { return m_id < id.m_id; }
    bool operator>(Id id) const { return m_id > id.m_id; }
    bool alphabeticallyBefore(Id other) const;

    quintptr uniqueIdentifier() const { return m_id; } // Avoid.
    static Id fromString(const QString &str); // FIXME: avoid.
    static Id fromName(const QByteArray &ba); // FIXME: avoid.
    static Id fromSetting(const QVariant &variant); // Good to use.

    static Id versionedId(const QByteArray &prefix, int major, int minor = -1);

    static QSet<Id> fromStringList(const QStringList &list);
    static QStringList toStringList(const QSet<Id> &ids);

private:
    // Intentionally unimplemented
    Id(const QLatin1String &) = delete;
    explicit Id(quintptr uid) : m_id(uid) {}

    quintptr m_id;
};

inline uint qHash(Id id) { return static_cast<uint>(id.uniqueIdentifier()); }

} // namespace Utils
Q_DECLARE_METATYPE(Utils::Id)
Q_DECLARE_METATYPE(QList<Utils::Id>)

QT_BEGIN_NAMESPACE
QDataStream &operator<<(QDataStream &ds, Utils::Id id);
QDataStream &operator>>(QDataStream &ds, Utils::Id &id);
CSUTILS_EXPORT QDebug operator<<(QDebug dbg, const Utils::Id &id);
QT_END_NAMESPACE
