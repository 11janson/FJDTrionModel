#pragma once
#include "cswidgets_global.h"
#include <QFrame>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>
#include <QIcon>
#include <QHBoxLayout>
namespace CS{
namespace Widgets{

class CSWIDGETS_EXPORT FrameLine : public QObject
{
    Q_OBJECT
public:
    explicit FrameLine(QObject *parent = 0);
    ~FrameLine();
    /**
    * @brief 创建一个水平线
    */
    static QFrame* createHorizontalLine(
            const QString &strLinePropertyName,
            QWidget*parent = nullptr,
            const quint32 &lineWidth = 1,
            const quint32 &marginTB = 8,
            const quint32 &marginLR = 0);
    /**
    * @brief 创建一个垂直线
    */
    static QFrame* createVerticalLine(
            const QString &strLinePropertyName,
            QWidget*parent = nullptr,
            const quint32 &lineWidth = 1,
            const quint32 &marginLR = 8,
            const quint32 &marginTB = 0);
private:

};
}
}


