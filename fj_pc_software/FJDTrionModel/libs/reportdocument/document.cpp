#include "document.h"

using namespace CS::TrionDocument;


CS::TrionDocument::Document::Document(QObject *parent)
    :QObject(parent)
{

}
CS::TrionDocument::Document::~Document()
{
}

bool Document::createDocument(const QString & strDocumentName)
{
    m_strFileName = strDocumentName;
    return true;
}



bool Document::writeDocumentTableContent(const std::vector<QStringList> tableContent)
{
    return false;
}

bool Document::writeDocumentLineContent(const QStringList & lineContent)
{
    return false;
}

bool Document::saveDocument()
{
    return false;
}
