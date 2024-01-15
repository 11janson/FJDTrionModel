#pragma  once
#include "document.h"


namespace CS {
    namespace TrionDocument {
        class ExcelXlsDocument :public Document
        {
            Q_OBJECT
        public:
            explicit ExcelXlsDocument(QObject *parent = Q_NULLPTR);
            virtual ~ExcelXlsDocument();

        public:
            /**
            *@brief 创建文档
            *@param 文档名称
            */
            virtual bool createDocument(const QString &strDocumentName) override;
            /**
            *@brief 写文档表格内容
            */
            virtual bool writeDocumentTableContent(const std::vector<QStringList> tableContent) override;

            /**
            *@brief 写一行数据
            */
            virtual bool writeDocumentLineContent(const QStringList &lineContent) override;

            /**
            *@brief 保存文档
            */
            virtual  bool saveDocument() override;


        };
    }
}