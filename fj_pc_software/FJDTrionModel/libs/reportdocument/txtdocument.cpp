#include "txtdocument.h"

using namespace CS::TrionDocument;


CS::TrionDocument::TxtDocument::TxtDocument(QObject *parent)
    :Document(parent)
{

}
CS::TrionDocument::TxtDocument::~TxtDocument()
{
    if (m_pDocFile){
        delete m_pDocFile;
        m_pDocFile = nullptr;
    }

    if (m_pDocStream){
        delete m_pDocStream;
        m_pDocStream = nullptr;
    }
}

bool TxtDocument::createDocument(
    const QString & strDocumentName)
{
    Document::createDocument(strDocumentName);
    m_pDocFile = new QFile(strDocumentName);
    if (!m_pDocFile->open(QIODevice::WriteOnly)) {
        return false;
    }

    m_pDocStream = new QTextStream(m_pDocFile);
    m_pDocStream->setCodec("UTF-8");

    return true;
}



bool TxtDocument::writeDocumentTableContent(
    const std::vector<QStringList> tableContent)
{
    if (!m_pDocStream) {
        return false;
    }

    m_pDocStream->setFieldWidth(12);
    m_pDocStream->setFieldAlignment(QTextStream::AlignRight);
    for (auto itR : tableContent) {
       
        for (auto itC : itR){
            *m_pDocStream << itC ;
        }
        *m_pDocStream << endl;
    }
    
    return true;
}

bool TxtDocument::writeDocumentLineContent(
    const QStringList & lineContent)
{
    if (!m_pDocStream) {
        return false;
    }

    for (auto it : lineContent) {
        *m_pDocStream << it << " ";
    }
    *m_pDocStream << endl;
    return true;
}

bool TxtDocument::saveDocument()
{
    m_pDocStream->flush();
    m_pDocFile->close();
    delete m_pDocFile;
    delete m_pDocStream;
    m_pDocFile = nullptr;
    m_pDocStream = nullptr;
    return true;
}
