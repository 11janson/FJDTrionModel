#include "outputlogview.h"
#include "outputwindow.h"
using namespace Utils;
#include <QDebug>
#include <QTextBrowser>
#include <QLayout>
#include <QVBoxLayout>
#include <QTextBlock>
#include <QTextCursor>
#include <QPalette>
#include <QMenu>
#include <QColor>
#include <QMutexLocker>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>


OutputLogView::OutputLogView(QWidget *parent) :
    QWidget(parent)
{
    qRegisterMetaType<QTextBlock>("QTextBlock");
    qRegisterMetaType<QTextCursor>("QTextCursor");
    createWidgets();
    createConnects();
    showMaximized();
    setWindowIcon(QIcon(QStringLiteral(":/logview/logviewer")));
}
OutputLogView::~OutputLogView()
{
}

void OutputLogView::createWidgets()
{
    m_pLogView = new OutputWindow(this);
    m_pLogView->setReadOnly(true);
    m_pLogView->setUndoRedoEnabled(false);
    m_pLogView->setWindowTitle(tr("Log Output"));
    m_pLogView->setContextMenuPolicy(Qt::CustomContextMenu);

    m_pLogToolBar = new QToolBar(tr("Log Output ToolBar"), this);
    m_pLogToolBar->setIconSize(QSize(16, 16));
    m_pLogToolBar->addAction(m_pLogView->copyAction);
    m_pLogToolBar->addAction(m_pLogView->selectAllAction);
    m_pLogToolBar->addSeparator();
    m_pLogToolBar->addAction(m_pLogView->clearAction);
    QVBoxLayout*vLayout = new QVBoxLayout(this);
    vLayout->setMargin(0);
    vLayout->setSpacing(0);
    vLayout->addWidget(m_pLogToolBar);
    vLayout->addWidget(m_pLogView);
    // Let selected text be colored as if the text edit was editable,
    // otherwise the highlight for searching is too light
    QPalette p = m_pLogView->palette();
    QColor activeHighlight = p.color(QPalette::Active, QPalette::Highlight);
    p.setColor(QPalette::Highlight, activeHighlight);
    QColor activeHighlightedText = p.color(QPalette::Active, QPalette::HighlightedText);
    p.setColor(QPalette::HighlightedText, activeHighlightedText);
    m_pLogView->setPalette(p);
    m_pLogView->setWordWrapEnabled(false);
}

void OutputLogView::createConnects()
{
    connect(this, SIGNAL(signalAppendLog(const QString &, int)),
            this, SLOT(onAppendLog(const QString &, int)));
    connect(m_pLogView, SIGNAL(customContextMenuRequested(const QPoint&)),
            this, SLOT(slotShowLogContextMenu(const QPoint&)));
}

void OutputLogView::onAppendLog(QString pMessage, int _severity_level)
{
    QMutexLocker lock(&mMutex);
    m_pLogView->appendText(pMessage, m_LevelForFormat[_severity_level]);
}

QColor OutputLogView::mixColors(const QColor &a, const QColor &b)
{
    return QColor((a.red() + 2 * b.red()) / 3, (a.green() + 2 * b.green()) / 3,
                  (a.blue() + 2 * b.blue()) / 3, (a.alpha() + 2 * b.alpha()) / 3);
}

void OutputLogView::submitAppendLog(const QString &pMessage, int _severity_level)
{
    emit signalAppendLog(pMessage, _severity_level);
}

void OutputLogView::slotShowLogContextMenu(const QPoint &pt)
{
    QMenu _menu(this);
    _menu.addAction(m_pLogView->copyAction);
    _menu.addAction(m_pLogView->selectAllAction);
    _menu.addSeparator();
    _menu.addAction(m_pLogView->clearAction);
    _menu.exec(m_pLogView->mapToGlobal(pt));
}
