#pragma  once
#include <QObject>
#include <vector>
#include "reportdocument_global.h"


namespace CS {
    namespace TrionDocument {
        /**
        *@brief 文档类型
        */
        enum TrionDocType{
            Txt,
            ExcelAx,
            ExcelQXls,
            Pdf,
        };
        class ReportDocumentPrivate;
        class REPORTDOCUMENT_EXPORT ReportDocument :public QObject
        {
            Q_OBJECT
        private:
            explicit ReportDocument(QObject *parent = Q_NULLPTR);
        public:
            virtual ~ReportDocument();
        public:
            /**
            *@brief 获取实例
            */
            static ReportDocument *instance(void);

            /**
            *@brief 创建文档
            *@param 文档类型
            *@param 文档名称
            */
            bool createDocument(const CS::TrionDocument::TrionDocType &docType, const QString &strDocumentName);
            /**
            *@brief 创建文档表头
            */
            bool createDocumentHeader(const QStringList &docHeader);
            /**
            *@brief 写文档表格内容
            */
            bool writeDocumentTableContent(const std::vector<QStringList> tableContent);

            /**
            *@brief 写一行数据
            */
            bool writeDocumentLineContent(const QStringList &lineContent);

            /**
            *@brief 保存文档
            */
            bool saveDocument();
        private:
            friend class CS::TrionDocument::ReportDocumentPrivate;
            CS::TrionDocument::ReportDocumentPrivate *m_pDptr = nullptr;
        };

    }
}



