#include "spdlogview.h"
#include "csutils/outputwindow.h"
using namespace Utils;
#include "spdlog/logger.h"
using namespace spdlog;
//------------------------------------------------------------------------------
#include <QDebug>
#include <QTextBrowser>
#include <QLayout>
#include <QVBoxLayout>
#include <QTextBlock>
#include <QTextCursor>
#include <QPalette>
#include <QMenu>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <QMutexLocker>

SpdLogView::SpdLogView(QWidget *parent)
    : OutputLogView(parent)
{
    initFormats();
}
SpdLogView::~SpdLogView()
{
}

void SpdLogView::initFormats()
{
    /*QPalette p = m_pLogView->palette();

    m_LevelForFormat[spdlog::level::trace].setFont(m_font);
    m_LevelForFormat[spdlog::level::trace].setForeground(mixColors(p.color(QPalette::Text), QColor(Qt::blue)));

    m_LevelForFormat[spdlog::level::debug].setFont(m_font);
    m_LevelForFormat[spdlog::level::debug].setForeground(QColor(Qt::blue));

    m_LevelForFormat[spdlog::level::info].setFont(m_font);
    m_LevelForFormat[spdlog::level::info].setForeground(QColor(Qt::black));

    m_LevelForFormat[spdlog::level::warn].setFont(m_font);
    m_LevelForFormat[spdlog::level::warn].setForeground(mixColors(p.color(QPalette::Text), QColor(Qt::green)));

    m_LevelForFormat[spdlog::level::err].setFont(m_font);
    m_LevelForFormat[spdlog::level::err].setForeground(QColor(Qt::red));

    m_LevelForFormat[spdlog::level::critical].setFont(m_font);
    m_LevelForFormat[spdlog::level::critical].setForeground(mixColors(p.color(QPalette::Text), QColor(Qt::red)));*/

}
