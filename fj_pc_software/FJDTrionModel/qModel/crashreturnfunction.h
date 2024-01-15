#pragma once

#include <QtWidgets/QWidget>
#include<QProcess>
#include<QStringList>
#include<QMap>
class crashreturnfunction : public QWidget
{
    Q_OBJECT

public:
    explicit crashreturnfunction(QWidget *parent = Q_NULLPTR);
public:
    void setPostInformation(QString);
public:
    //[!]获取软件dump目录
    QString getFileName(QString);


    QString getComputerMac();
    QString getComputerUUID();
    bool setConfigJsonFile(QMap<QString, QString>);

    bool getCosParameter(QString);
    //[!]复制config文件到配置目录
    bool copyConfigJsonFile();
    //[!]获取程序路径
    QString getCrashExePath();
    //[!]获取程序参数
    QStringList getCrashExePram();
    //[!]文件处理流程,参数返回需要做检查
    QString dmpFilePath();
    //[!]将本地log路径输入
    QString getLogFilePath();
    //[!]获取temp文件夹路径
    QString getCrashTempPath();
private:
    QString m_sDumpPath = "";
    QProcess *m_pProcess = nullptr;
    QStringList m_Param;
    QMap<QString, QString>map;
    QString m_currentTime = "";
    QString newVersion = "";
};
