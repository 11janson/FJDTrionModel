#include "crashreturnfunction.h"
#include<QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include<QApplication>
#include<QJsonArray>
#include<QFIle>
#include<QDir>
#include "cloudcompareutils/icore.h"
#include<QTime>
#include<QDate>
#include<QDateTime>
#include<QNetworkInterface>
using namespace CS;
#ifndef CrashApplication
#define CrashApplication "http://scanner.fjdac.cn"
#else
#define CrashApplication "http://scanner-test.fjdac.cn"
#endif
crashreturnfunction::crashreturnfunction(QWidget *parent)
    : QWidget(parent)
{

    QSettings configSettings(QApplication::applicationDirPath() + "/share/version/versionConfig.ini",  QSettings::IniFormat);
    QString oldVersion(FJDTRITONMETAHUB_VERSIONS);
    oldVersion += "-error";
    newVersion = configSettings.value("TrionModelStart/version", oldVersion).toString();

    if (newVersion.isEmpty())
    {
        QString version(FJDTRITONMETAHUB_VERSIONS);
        newVersion = version;
    }
    QString currentTime = "";
    QDate date = QDate::currentDate();
    currentTime.append(date.toString("yyyy-MM-dd"));
    currentTime.append("-");
    QTime time = QTime::currentTime();
    currentTime.append(time.toString("hh:mm:ss tt"));
    m_currentTime = QString::number(QDateTime::currentMSecsSinceEpoch());
    qDebug() << "m_currentTime" << QDateTime::currentMSecsSinceEpoch();
    m_pProcess = new QProcess(this);
    if (!copyConfigJsonFile())
    {
        return;
    }
    QString dmpfilepath = dmpFilePath();
    QString logfilepath = getLogFilePath();

    QString filename = getFileName(dmpfilepath);
    QString logfilename = getFileName(logfilepath);
    if (filename.isEmpty() || logfilename.isEmpty())
    {
        return;
    }
    m_Param.append(dmpfilepath);
    m_Param.append(logfilepath);
    m_Param.append(newVersion);
    m_Param.append(currentTime);
    if (!getCosParameter(CrashApplication))
    {
        return;
    }
    m_Param.append(filename);
    m_Param.append(logfilename);
    m_Param.append(m_currentTime);
    m_Param.append(getCrashTempPath());
    QString strMac = getComputerMac();
    if (strMac.isEmpty())
    {
       return;
    }
    m_Param.append(strMac);
    if (m_Param.size() == 13)
    {
        m_Param.append("program_post_success");
    }
}

void crashreturnfunction::setPostInformation(QString JSONtext)
{
    qDebug() << JSONtext;
    if (JSONtext.isEmpty())
    {
        qDebug() << " crash:  JSONtext isEmpty! ";
        return;
    }
    QByteArray bit = JSONtext.toUtf8();
    QJsonParseError jsonError;
    QJsonDocument jsonDoc(QJsonDocument::fromJson(bit, &jsonError));
    QString SecretId = "";
    QString SecretKey = "";
    QString sessionToken = "";
    if (jsonError.error == QJsonParseError::NoError) {
        QJsonObject jsonObject = jsonDoc.object();

        int codeData = jsonObject["code"].toInt();
        QString msgData = jsonObject["msg"].toString();
        QJsonObject dataObj = jsonObject.value("data").toObject();
        SecretId = dataObj.value("tmpSecretId").toString();
        SecretKey = dataObj.value("tmpSecretKey").toString();
        sessionToken = dataObj.value("sessionToken").toString();
    }
    else {
        qDebug() << "json data error!" << jsonError.errorString();
        return;
    }
    QMap<QString, QString>map;
    map.insert("SecretId", SecretId);
    map.insert("SecretKey", SecretKey);
    m_Param.append(SecretId);
    m_Param.append(SecretKey);
    m_Param.append(sessionToken);
    if (!setConfigJsonFile(map))
    {
        qDebug() << "setConfigJsonFile error!" << endl;
        return;
    }
}
QString crashreturnfunction::getFileName(QString filePath)
{
    if (filePath.isEmpty())
    {
        return "";
    }
    QFileInfo fileInfo = QFileInfo(filePath);
    if (fileInfo.exists())
    {

        QString fileName = fileInfo.fileName();
        return fileName;
    }
    return "";
}


bool crashreturnfunction::getCosParameter(QString serveAdress)
{
    QStringList sParams;
    QString Interface = "/scanner/file/getTmpSecret";
    sParams.append(serveAdress + Interface);
    QString exePath = QApplication::applicationDirPath() +"/share/curl/curl.exe";
    m_pProcess->start(exePath, sParams);
    m_pProcess->waitForStarted();
    m_pProcess->waitForFinished();
    QString strReply = m_pProcess->readAllStandardOutput();
    if (!strReply.isEmpty())
    {
        qDebug() << "get key success !";
        setPostInformation(strReply);
        return true;
    }
    return false;
}


bool crashreturnfunction::setConfigJsonFile(QMap<QString, QString>map)
{
    QJsonObject setJSON;
    setJSON.insert("SecretId", map.value("SecretId"));
    setJSON.insert("SecretKey", map.value("SecretKey"));

    //[!]其他数据配置

    setJSON.insert("Region", "ap-beijing");
    setJSON.insert("SignExpiredTime", 360);
    setJSON.insert("ConnectTimeoutInms", 6000);
    setJSON.insert("ReceiveTimeoutInms", 5000);
    setJSON.insert("AsynThreadPoolSize", 2);
    setJSON.insert("UploadPartSize", 10485760);
    setJSON.insert("UploadCopyPartSize", 10485760);
    setJSON.insert("UploadThreadPoolSize", 5);
    setJSON.insert("LogoutType", 2);
    setJSON.insert("LogLevel", 3);
    setJSON.insert("DownloadThreadPoolSize", 5);
    setJSON.insert("DownloadSliceSize", 4194304);
    setJSON.insert("IsDomainSameToHost", false);
    setJSON.insert("DestDomain", "");
    setJSON.insert("IsUseIntranet", false);
    setJSON.insert("IntranetAddr", "");


    QJsonDocument jDoc(setJSON);
    if (m_Param.isEmpty())
    {
        return false;
    }
    QFile file(m_Param.at(0));
    if (!file.open(QIODevice::WriteOnly))
        return false;
    QByteArray data(jDoc.toJson());
    file.write(data);
    file.close();
    return true;
}

QString crashreturnfunction::getLogFilePath()
{
    QString logpath = QDir::fromNativeSeparators(CS::Core::ICore::getDefaultPath()) + "/log/";
    QString logfile = logpath + "FJDTrionModel.log";
    if (m_currentTime.isEmpty())
    {
        return "";
    }
    QString newName = logpath + newVersion + "-" +m_currentTime + ".log";

    QFile file(logfile);
    if (!file.exists())
    {
        qDebug() << "logfile not find;";
        return "";
    }
    //开始复制
    if (!QFile::copy(logfile, newName))
    {
        qDebug() << "crash: log copy error !";
        return "";
    }
    QFile fileNew(newName);
    if (!fileNew.exists())
    {
        return "";
    }
    QString newLogfileName = getCrashTempPath()+"/" + newVersion + "-" + m_currentTime + ".log";
    if (!QFile::rename(newName, newLogfileName))
    {
        qDebug() << "crash: rename error !";
        return "";
    }
    return newLogfileName;
   
}
bool crashreturnfunction::copyConfigJsonFile()
{
    QString inpath = QDir::fromNativeSeparators(QApplication::applicationDirPath()) + "/share/Crash/";
    QString topath = QDir::fromNativeSeparators(CS::Core::ICore::getDefaultPath()) + "/config";
    QDir dir(topath);
    QStringList names = dir.entryList(QDir::Files);
    if (dir.exists())
    {
        QFile::copy(inpath + "/crash.json", topath + "/crash.json");
        m_Param.append(topath + "/crash.json");
        return true;
    }
    return false;
}
QStringList crashreturnfunction::getCrashExePram()
{
    return m_Param;
}
QString crashreturnfunction::getCrashExePath()
{
    QString Filepath = QApplication::applicationDirPath();
    QString cmd = Filepath + "/CrashBack.exe";
    cmd = "\"" + cmd + "\"";
    return cmd;
}
QString crashreturnfunction::dmpFilePath()
{
    QString path = QDir::fromNativeSeparators(CS::Core::ICore::getDefaultPath()) + "/dump/";
    QDir dir(path);
    dir.setFilter(QDir::NoDotAndDotDot | QDir::AllEntries);
    dir.setSorting(QDir::Time | QDir::DirsFirst);
    QFileInfoList list = dir.entryInfoList();
    QString dmpfile = "";
  
   for (int i = 0; i < list.size(); i++)
    {
        if (list.at(i).isFile())
        {
            if (list.at(i).suffix() == "dmp" || list.at(i).suffix() == "DMP")
            {
                dmpfile = QDir::fromNativeSeparators(list.at(0).absoluteFilePath());
                QString filePath = QDir::fromNativeSeparators(list.at(0).absolutePath());
                QString fileName = list.at(0).fileName();
                QString tempPath = getCrashTempPath();
                if (fileName.contains(newVersion + "-" + m_currentTime))
                {
                    filePath = tempPath +"/"+ list.at(i).fileName();
                }
                else
                {
                    filePath = tempPath +"/"+ newVersion + "-" + m_currentTime + "." + list.at(i).suffix();
                }
               
                if (!QFile::rename(dmpfile, filePath))
                {
                    qDebug() << "crash: dmp rename file  error !";
                    return "";
                }
               
                
                dmpfile = QDir::fromNativeSeparators(filePath);
                break;
            }
        }
    }

    return dmpfile;
}

QString crashreturnfunction::getCrashTempPath()
{
    QString path = QDir::fromNativeSeparators(CS::Core::ICore::getDefaultPath());
    QStringList tempFiles = path.split("/");
    QString temppath;
    if (tempFiles.size() > 4)
    {
        for (int i = 0; i < 4; i++)
        {
            temppath += tempFiles.at(i) + "/";
        }
        temppath += "Local/Temp/fjcrashtemp";

        QDir dir;
        if (!dir.exists(temppath))
        {
            dir.mkdir(temppath);
        }
    }
    else
    {
        qDebug() << "get temppath error!" << Qt::endl;
        return "";
    }
    return temppath;
}

QString crashreturnfunction::getComputerMac()
{
    
    QList<QNetworkInterface> nets = QNetworkInterface::allInterfaces();// 获取所有网络接口列表
    int nCnt = nets.count();
    qDebug() << nets;
    QString strMacAddr = "";
    for (int i = 0; i < nCnt; i++)
    {
        if (nets[i].flags().testFlag(QNetworkInterface::IsUp) &&
            nets[i].flags().testFlag(QNetworkInterface::IsRunning)
            && !nets[i].flags().testFlag(QNetworkInterface::IsLoopBack))
        {
            for (int j = 0; j < nets[i].addressEntries().size(); j++) {
                if (nets[i].addressEntries().at(j).ip() != QHostAddress::LocalHost&&nets[i].addressEntries().at(j).ip().protocol() == QAbstractSocket::IPv4Protocol
                    ) {
                    strMacAddr = nets[i].hardwareAddress();
                }
            }
        }
    }
    qDebug() << "strMacAddr" << strMacAddr;
    return strMacAddr;
}
QString crashreturnfunction::getComputerUUID()
{
    QProcess process;

#ifdef Q_OS_WIN
    //    wmic csproduct get uuid

    QStringList mList;
    mList << "csproduct" << "get" << "uuid";
    process.start("wmic", mList);

    process.waitForFinished();
    QString ret = process.readAll();
    ret = ret.trimmed();

    QStringList dataList = ret.split("\r\n");

    if (dataList.length() != 2)
    {
        return "";
    }

    return dataList[1].trimmed();
#endif

#ifdef Q_OS_LINUX
    QStringList mList;
    mList << "-s" << "system-uuid";
    process.start("dmidecode", mList);

    process.waitForFinished();
    QString ret = process.readAll();
    ret = ret.trimmed();

    return ret;
#endif

    return "";
}
