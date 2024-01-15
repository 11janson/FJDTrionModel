#pragma once
#include <QOBject>
#include "cloudcompareutils_global.h"
#include "qxlsx/header/xlsxdocument.h"
#include "qxlsx/header/xlsxchartsheet.h"
#include "qxlsx/header/xlsxcellrange.h"
#include "qxlsx/header/xlsxchart.h"
#include "qxlsx/header/xlsxrichstring.h"
#include "qxlsx/header/xlsxworkbook.h"

namespace Utils {

	enum FileType {
		TxtType,
		ExeclType,
		CsvType,
	};
	enum FileInteractionStatus{
		Write,
		Read
	};
    enum TargetLanguage {
        CN		=	2,
        JA		=	4,
        CN_TW	=	5,
		Russian =	6,
		Spanish =	7,
		Italian =	8,
    };

	class TRIONMETAHUBUTILS_EXPORT publicExportFile : public QObject
	{
		Q_OBJECT
	public:
		//[!]输入的参数:文件路径,文件内容，列数，最后几行如何输出 如最后3 行按照两列输出,文件类型
		static bool writeDataToFile(QString filePath, QStringList list, int numColumns, int lastRows, int lastColumns, FileType);
        //[!]获取这个execl的所有内容 //三层，外层代表表格的数据，二层代表单个表格数据，三层代表每行的数据 
		//[!]如果某行某列的数据没有，将会用空字符填充。所以读出来不管数据是什么样子，实际排列都是类似矩阵
        static  QList<QList<QList<QString>>>  readDataFromFile(QString filePath);
		//[!]获取单个excel中的所有数据
        static QList<QStringList> readForExecl(const QString excelPath);

		//[!]获取当前字符串有几个汉字
		static int calculateWidth(QString);
		//[!]写
		void writeToExecl(QString);

		//[!]读xlsx
        QList<QList<QString>> readForExecl();

		//[!]读ts文件
        void getTsFileStream(TargetLanguage);
	};
}

