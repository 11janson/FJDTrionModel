#pragma  once
#include <QObject>
#include <vector>

namespace CS {
    namespace TrionDocument {
        class Document :public QObject
        {
            Q_OBJECT
        public:
            explicit Document(QObject *parent = Q_NULLPTR);
            virtual ~Document();

        public:
            /**
            *@brief 创建文档
            *@param 文档名称
            */
            virtual bool createDocument(const QString &strDocumentName);
            /**
            *@brief 写文档表格内容
            */
            virtual bool writeDocumentTableContent(const std::vector<QStringList> tableContent);

            /**
            *@brief 写一行数据
            */
            virtual bool writeDocumentLineContent(const QStringList &lineContent);

            /**
            *@brief 保存文档
            */
            virtual  bool saveDocument();
        protected:
            QString m_strFileName;
        };
    }
}
