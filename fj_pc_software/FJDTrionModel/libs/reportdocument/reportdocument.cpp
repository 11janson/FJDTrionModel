#include "reportdocument.h"
#include "txtdocument.h"
#include "excelaxdocument.h"
#include "excelxlsdocument.h"
#include "document.h"
#include <QDebug>


using namespace CS;
using namespace CS::TrionDocument;

namespace CS {
    namespace TrionDocument {

        class ReportDocumentPrivate : public QObject
        {
            Q_OBJECT
        public:
            explicit ReportDocumentPrivate(CS::TrionDocument::ReportDocument *pQptr);
            virtual ~ReportDocumentPrivate();
        private:
            friend class CS::TrionDocument::ReportDocument;
            CS::TrionDocument::ReportDocument *m_pQptr = nullptr;
        private:
            CS::TrionDocument::Document *m_pDocument = nullptr;

        };
    }
}

ReportDocumentPrivate::ReportDocumentPrivate(CS::TrionDocument::ReportDocument *pQptr)
    :QObject(nullptr)
    ,m_pQptr(pQptr)
{

}


ReportDocumentPrivate::~ReportDocumentPrivate()
{

}

ReportDocument::ReportDocument(QObject *parent)
    :QObject(nullptr)
    , m_pDptr(new ReportDocumentPrivate(this))
{

}


ReportDocument::~ReportDocument()
{
    if (m_pDptr){
        delete m_pDptr;
        m_pDptr = nullptr;
    }
}

ReportDocument * CS::TrionDocument::ReportDocument::instance(void)
{
    static CS::TrionDocument::ReportDocument instance;
    return &instance;
}

bool ReportDocument::createDocument(
    const CS::TrionDocument::TrionDocType & docType, 
    const QString & strDocumentName)
{
    switch (docType)
    {
    case CS::TrionDocument::TrionDocType::Txt:
        m_pDptr->m_pDocument = new CS::TrionDocument::TxtDocument();
        break;
    case CS::TrionDocument::TrionDocType::ExcelAx:
        m_pDptr->m_pDocument = new CS::TrionDocument::ExcelAxDocument();
        break;
    case CS::TrionDocument::TrionDocType::ExcelQXls:
        m_pDptr->m_pDocument = new CS::TrionDocument::ExcelXlsDocument();
        break;
    default:
        m_pDptr->m_pDocument = new CS::TrionDocument::TxtDocument();
        break;
    }

    m_pDptr->m_pDocument->createDocument(strDocumentName);
    return true;
}



bool ReportDocument::writeDocumentTableContent(
    const std::vector<QStringList> tableContent)
{
    return m_pDptr->m_pDocument->writeDocumentTableContent(tableContent);
}

bool ReportDocument::writeDocumentLineContent(
    const QStringList & lineContent)
{
    return m_pDptr->m_pDocument->writeDocumentLineContent(lineContent);
}

bool ReportDocument::saveDocument()
{
    return m_pDptr->m_pDocument->saveDocument();
}

#include "reportdocument.moc"