TEMPLATE = lib
TARGET = qpdf

CONFIG += dll
CONFIG += c++11

QT += core gui  widgets webchannel webview

lessThan(QT_VERSION, 0x050600)
{
QT += webkitwidgets
# do something else
}
greaterThan(QT_VERSION, 0x050500)
{
QT += webenginewidgets
# do something else
}


DEFINES += QPDFLIB_BUILD

HEADERS =\
    qpdfwidget.h \
    pdfjsbridge.h

SOURCES =\
    qpdfwidget.cpp \
    pdfjsbridge.cpp

RESOURCES += pdfview.qrc
