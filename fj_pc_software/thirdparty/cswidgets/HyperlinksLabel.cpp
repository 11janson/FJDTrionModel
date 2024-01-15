#include "HyperlinksLabel.h"
#include <QVariant>
#include "cswidgets/framelessfiledialog.h"
#include <QDesktopServices>
#include <QDebug>

using namespace CS::Widgets;
HyperlinksLabel::HyperlinksLabel(QWidget *parent)
    : QLabel(parent)
{
	createConnect();
}

void HyperlinksLabel::createConnect()
{
    connect(this, &QLabel::linkActivated, this, &HyperlinksLabel::slotOpenUrl);
}

void HyperlinksLabel::setBaseProperty(bool status)
{   
    this->setOpenExternalLinks(status);
    this->setProperty("about", QVariant(true));
}

void HyperlinksLabel::setHyperlinksText(QString linkName)
{
    QString authorizeFileLink = QString("<a href=hello>%1</a>").arg(linkName);
    this->setText(authorizeFileLink);
}

void HyperlinksLabel::setFileDir(QString strDir)
{
    m_strDir = strDir;
}

void HyperlinksLabel::slotOpenUrl(QString strUrl)
{
    qDebug() << "strUrl" << m_strDir;
    QString url = "file:///"+ m_strDir;
    QDesktopServices::openUrl(QUrl(url, QUrl::TolerantMode));
}