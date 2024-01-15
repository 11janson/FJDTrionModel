#include "excelaxdocument.h"
#include <QAxObject>
#include <QDir>


using namespace CS::TrionDocument;


CS::TrionDocument::ExcelAxDocument::ExcelAxDocument(QObject *parent)
    :Document(parent)
{

}
CS::TrionDocument::ExcelAxDocument::~ExcelAxDocument()
{
}

bool ExcelAxDocument::createDocument(
    const QString & strDocumentName)
{
    Document::createDocument(strDocumentName);
    m_pExcel = new QAxObject;
    if (!m_pExcel->setControl("ket.Application") && !m_pExcel->setControl("Excel.Application")) {
        return false;
    }
    m_pExcel->setProperty("DisplayAlerts", false);
    return true;
}



bool ExcelAxDocument::writeDocumentTableContent(
    const std::vector<QStringList> tableContent)
{
    if (!m_pExcel){
        return false;
    }

    QAxObject *pWorkbooks = m_pExcel->querySubObject("WorkBooks");
    if (!pWorkbooks){

        m_pExcel->dynamicCall("Quit()");
        delete m_pExcel;
        m_pExcel = nullptr;
    }
    pWorkbooks->dynamicCall("Add");
    QAxObject* pWorkbook = m_pExcel->querySubObject("ActiveWorkBook");
    QAxObject* pSheet = pWorkbook->querySubObject("WorkSheets(int)", 1);
    QString range = "A1:G" + QString::number(tableContent.size());
    QAxObject* user_range = pSheet->querySubObject("Range(const QString&)", range);
    if (!user_range || user_range->isNull()) {
        m_pExcel->dynamicCall("Quit(void)");  
        delete m_pExcel;
        m_pExcel = NULL;
        return false;
    }
    user_range->setProperty("HorizontalAlignment", -4152);
    QList<QVariant> dataList;
    for (int i = 0; i < tableContent.size(); i++) {
        dataList.push_back(tableContent[i]);
    }
    user_range->dynamicCall("SetValue(const QVariant&)", QVariant(dataList));
    pWorkbook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(m_strFileName)); 
    pWorkbook->dynamicCall("Close (Boolean)", false);  
    return true;
}

bool ExcelAxDocument::writeDocumentLineContent(
    const QStringList & lineContent)
{
    return false;
}

bool ExcelAxDocument::saveDocument()
{
  
    m_pExcel->dynamicCall("Quit(void)"); 
    delete m_pExcel;
    m_pExcel = nullptr;
    return true;
}
