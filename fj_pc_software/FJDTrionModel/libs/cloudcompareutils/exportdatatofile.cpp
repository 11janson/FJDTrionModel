#include "exportdatatofile.h"
#include <QtGlobal>
#include <QtCore>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QFontMetrics>
#include <QApplication>
#include <QTextDocument>
#include <QTextCursor>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QTextTable>
#include <QFileDialog>

#define STRINGSPACING 3
#define TargetLanguageColumns 3

using namespace Utils;

void publicExportFile::writeToExecl(QString filepath)
{
	QXlsx::Document xlsx;
	xlsx.write("A1", "Hello QtXlsx!");
	xlsx.write("A2", 12345);
	xlsx.write("A3", "=44+33");
	xlsx.write("A4", true);
	xlsx.write("A5", QDate(2013, 12, 27));
	if (!xlsx.saveAs(filepath))
	{
		qDebug() << "[WriteExcel1] failed to save excel file";
		return ;
	}
}

QList<QStringList> publicExportFile::readForExecl(const QString excelPath)
{
    QList<QStringList> allData; // 用于存储所有工作表数据的列表

    QXlsx::Document xlsx(excelPath);
    if (xlsx.load()) {
        QStringList sheetNames = xlsx.sheetNames(); // 获取工作表数量

        foreach(const QString & sheetName, sheetNames) {
            xlsx.selectSheet(sheetName); // 选择当前工作表

            for (int row = 1; row <= xlsx.dimension().rowCount(); ++row) {
                QStringList rowData; // 每一行的数据都存储在这个字符串列表中
                for (int col = 1; col <= xlsx.dimension().columnCount(); ++col) {
                    QString gird = xlsx.read(row, col).toString();
                    QRegularExpression regex("[\\t\\n ]"); 
                    QRegularExpressionMatch match = regex.match(gird);
                    if (match.hasMatch()) {
                        allData.clear();
                        return allData;
                    }

                    rowData.append(gird);
                }
                allData.append(rowData); // 将每行数据添加到所有工作表数据中
            }
            break;
        }
    }
    else {
        allData.clear();
    }

    return allData;
}
QList<QList<QString>> publicExportFile::readForExecl()
{
    QStringList list = QFileDialog::getOpenFileNames(nullptr, "Open xlsx File", "", "Excel Files (*.xlsx)");
    QList<QList<QString>> data;
    for (QString path : list) {
        QXlsx::Document xlsx(path);
        QStringList sheetNames = xlsx.sheetNames();
        foreach(const QString &sheetName, sheetNames) {
            xlsx.selectSheet(sheetName);
            int rowCount = xlsx.dimension().lastRow();
            int columnCount = xlsx.dimension().lastColumn();
            for (int row = 1; row <= rowCount; ++row) {
                QList<QString> rowData;
                for (int column = 2; column <= columnCount; ++column) {
                    QXlsx::Cell *cell = xlsx.cellAt(row, column);
                    if (cell) {
                        rowData.append(cell->value().toString());
                        QXlsx::Format format;
                        format.setPatternBackgroundColor(QColor(Qt::green));
                        if (!xlsx.write(row, column, cell->value(), format))
                        {
                            qDebug() << "write cell color error";
                        }
                    }
                    else {
                        rowData.append("");
                    }
                }
                 data.append(rowData);
            }
        }
        xlsx.save();
    }
    return data;
}
void Utils::publicExportFile::getTsFileStream(TargetLanguage target)
{
    QStringList filePath = QFileDialog::getOpenFileNames(nullptr, "Open Ts File", "", "Ts Files (*.ts)");
    if (filePath.isEmpty()) {
        return;
    }
    QFile file(filePath.at(0));
    if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        qDebug() << "open ts error";
        return;
    }
    QTextStream in(&file);
    QStringList updatedLines;
    while (!in.atEnd()) {
        updatedLines.append(in.readLine().trimmed());
    }
    QStringList list = QFileDialog::getOpenFileNames(nullptr, "Open xlsx File", "", "Excel Files (*.xlsx)");
    QList<QList<QString>> data;
    for (QString path : list) {
        QXlsx::Document xlsx(path);
        QStringList sheetNames = xlsx.sheetNames();
        foreach(const QString &sheetName, sheetNames) {
            xlsx.selectSheet(sheetName);
            int rowCount = xlsx.dimension().lastRow();
            int columnCount = xlsx.dimension().lastColumn();
            for (int row = 1; row <= rowCount; ++row) {
                QList<QString> rowData;
                for (int column = 2; column <= columnCount; ++column) {
                    QXlsx::Cell *cell = xlsx.cellAt(row, column);
                    if (cell) {
                        QString cellData = cell->value().toString().trimmed();
                        for (int i = 0; i < updatedLines.size(); ++i) {
                            const QString &line = updatedLines.at(i);
                            if (line.contains("<source>") && line.contains("</source>")) {
                                int startIdx = line.indexOf("<source>");
                                int endIdx = line.indexOf("</source>");
                                if (startIdx != -1 && endIdx != -1) {
                                    int contentStart = startIdx + QString("<source>").length();
                                    int contentLength = endIdx - contentStart;
                                    QString desiredContent = line.mid(contentStart, contentLength).trimmed();
                                    if (desiredContent == cellData)
                                    {
                                        if (i + 1 >= updatedLines.size()) {
                                            return;
                                        }
                                        QString &nextLine = updatedLines[i + 1];
                                        QXlsx::Cell *targetCell;
                                        if (target == CN)
                                        {
                                            targetCell = xlsx.cellAt(row, int(CN));
                                        }
                                        else if (target == CN_TW)
                                        {
                                            targetCell = xlsx.cellAt(row, int(CN_TW));
                                        }
                                        else if (target == JA)
                                        {
                                            targetCell = xlsx.cellAt(row, int(JA));
                                        }
                                        else if (target == Russian)
                                        {
                                            targetCell = xlsx.cellAt(row, int(Russian));
                                        }
                                        else if (target == Spanish)
                                        {
                                            targetCell = xlsx.cellAt(row, int(Spanish));
                                        }
                                        else if (target == Italian)
                                        {
                                            targetCell = xlsx.cellAt(row, int(Italian));
                                        }
                                        if (!targetCell)
                                        {
                                            qDebug() << "targetCell error:" << sheetName << row << int(target);
                                            continue;
                                        }
                                        nextLine.clear();
                                        nextLine.append("<translation>");
                                        nextLine.append(targetCell->value().toString());
                                        nextLine.append("</translation>");
                                        QXlsx::Format format;
                                        format.setPatternBackgroundColor(QColor(Qt::green));
                                        if (!xlsx.write(row, int(target), targetCell->value(), format))
                                        {
                                            qDebug() << "write cell color error" << "row , column" << row << column;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        xlsx.saveAs(path);
    }
    
    file.resize(0);
    for(QString &line :updatedLines) {

        if (line.contains("<name>") && line.contains("</name>"))
        {
            line.prepend("\t");
        }
        else if (line == "<message>" || line == "</message>")
        {
            line.prepend("\t");
        }
        else if (line.contains("<source>") && line.contains("</source>"))
        {
            line.prepend("\t\t");
        }
        else if (line.contains("<translation") && line.contains("</translation>"))
        {
            line.prepend("\t\t");
        }
        else if (line.contains("<location filename="))
        {
            line.prepend("\t\t");
        }
        in << line << "\n";
    }
    file.close();

}
bool Utils::publicExportFile::writeDataToFile(QString filePath, QStringList list, int numColumns,int lastRows,int lastColumns, FileType Type)
{

	if (Type == TxtType) {
		QFile file(filePath);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			qWarning("无法打开文件以进行写入操作。");
			return false;
		}

		QTextStream out(&file);
		out.setCodec("UTF-8");

		// 计算每个字符串的最大宽度
		int maxWidth = 0;
		for (const QString &str : list) {
			int width = calculateWidth(str);
			if (width > maxWidth) {
				maxWidth = width;
			}
		}

		int totalElements = list.size();
		int elementsBeforeLastBlock = totalElements - (lastRows * lastColumns);
		int fullRowsBeforeLastBlock = elementsBeforeLastBlock / numColumns;
		
		// 写入完整的行，直到最后的 lastRows * lastColumns 个元素
		for (int i = 0; i < fullRowsBeforeLastBlock; i++) {
			for (int j = 0; j < numColumns; j++) {
				int index = i * numColumns + j;
				QString str = list.at(index);
				int strLenth = str.size();
				int size = strLenth / 8;
				QString tab = " \t";
				if(!size && (calculateWidth(str) < 5)|| str.contains(" ")){
					tab += "\t";
				}
				out << str + tab;
				if (j == numColumns - 1) {
					out << "\n";
				}
			}
		}
		// 将剩余的数据按照 lastRows 行，lastColumns 列排序
		int index = elementsBeforeLastBlock;
		for (int i = 0; i < lastRows; i++) {
			for (int j = 0; j < lastColumns; j++) {
				if (index < totalElements) {
					QString str = list.at(index);
					int strLenth = str.size();
					int size = strLenth / 8;
					QString tab = " \t";
					if (!size && (calculateWidth(str) < 5)) {
						tab += "\t";
					}
					out << str + tab;
					index++;
				}
				if (j == lastColumns - 1 && i != lastRows - 1) {
					out << "\n";
				}
			}
		}

		file.close();
		return true;
	}
	else if (Type == ExeclType) {
		
		int totalElements = list.size();
		int elementsBeforeLastBlock = totalElements - (lastRows * lastColumns);
		int fullRowsBeforeLastBlock = elementsBeforeLastBlock / numColumns;

		QXlsx::Document xlsx;
		// 写入完整的行，直到最后的 lastRows * lastColumns 个元素
		for (int i = 0; i < fullRowsBeforeLastBlock; i++) {
			for (int j = 0; j < numColumns; j++) {
				int index = i * numColumns + j;
				int AsciiNum = 65 + j;
				QString str = QString(QChar(AsciiNum)) + QString::number(i+1);
				xlsx.write(str, list.at(index));
			}
		}
		// 将剩余的数据按照 lastRows 行，lastColumns 列排序
		int index = elementsBeforeLastBlock;
		for (int i = 0; i < lastRows; i++) {
			for (int j = 0; j < lastColumns; j++) {
				if (index < totalElements) {
					int AsciiNum = 65 + j;
					QString str = QString(QChar(AsciiNum)) + QString::number(fullRowsBeforeLastBlock + 1);
					xlsx.write(str, list.at(index));
					index++;
				}
			}
		}
		if (!xlsx.saveAs(filePath))
		{
			qDebug() << "[WriteExcel1] failed to save excel file";
			return false;
		}

		return true;
	}
	else if (Type == CsvType)
	{
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qWarning("无法打开文件以进行写入操作。");
            return false;
        }

        int totalElements = list.size();
        int elementsBeforeLastBlock = totalElements - (lastRows * lastColumns);
        int fullRowsBeforeLastBlock = elementsBeforeLastBlock / numColumns;

        QTextStream out(&file);

        for (int i = 0; i < fullRowsBeforeLastBlock; i++) {
            for (int j = 0; j < numColumns; j++) {
                if (list.isEmpty())
                {
                    return false;
                }
                out << list.first();
                out << ",";
                list.removeFirst();
            }
			out << "\n";
        }
        // 将剩余的数据按照 lastRows 行，lastColumns 列排序
        int index = elementsBeforeLastBlock;
        for (int i = 0; i < lastRows; i++) {
            for (int j = 0; j < numColumns; j++) {
				if (list.isEmpty())
				{
					return false;
				}
				if (j < (numColumns - lastColumns))
				{
					out << ",";
					continue;
				}
                out << list.first();
                out << ",";
                list.removeFirst();
            }
			out << "\n";
        }

        file.close();
        return true;

	}else{
		return false;
	}
}

QList<QList<QList<QString>>> publicExportFile::readDataFromFile(QString filePath)
{
    QList<QList<QList<QString>>> mainList;
    QList<QList<QString>> data;
    QXlsx::Document xlsx(filePath);
    QStringList sheetNames = xlsx.sheetNames();
    foreach(const QString & sheetName, sheetNames) {
        xlsx.selectSheet(sheetName);
        int rowCount = xlsx.dimension().lastRow();
        int columnCount = xlsx.dimension().lastColumn();
        for (int row = 1; row <= rowCount; ++row) {
            QList<QString> rowData;
            for (int column = 2; column <= columnCount; ++column) {
                QXlsx::Cell* cell = xlsx.cellAt(row, column);
                if (cell) {
                    rowData.append(cell->value().toString());
                }
                else {
                    rowData.append("");
                }
            }
            data.append(rowData);
        }
        mainList.append(data);
    }
    return mainList;
}

int Utils::publicExportFile::calculateWidth(QString str)
{
	int count = 0;
	QRegExp regex = QRegExp(QString("^[\u4E00-\u9FA5]{0,}$"));
	for (int i = 0; i < str.length(); i++)
	{
		if (regex.exactMatch(QString(str[i]))) {

			count++;
		}
	}
	return count;
}

