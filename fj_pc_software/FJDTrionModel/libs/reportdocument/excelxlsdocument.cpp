#include "excelxlsdocument.h"

using namespace CS::TrionDocument;


CS::TrionDocument::ExcelXlsDocument::ExcelXlsDocument(QObject *parent)
    :Document(parent)
{

}
CS::TrionDocument::ExcelXlsDocument::~ExcelXlsDocument()
{
}

bool ExcelXlsDocument::createDocument(
    const QString & strDocumentName)
{
    return false;
}


bool ExcelXlsDocument::writeDocumentTableContent(
    const std::vector<QStringList> tableContent)
{
    return false;
}

bool ExcelXlsDocument::writeDocumentLineContent(
    const QStringList & lineContent)
{
    return false;
}

bool ExcelXlsDocument::saveDocument()
{
    return false;
}
