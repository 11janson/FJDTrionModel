#include "sentinelldkencrypt.h"

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <QDateTime>
#include <QMap>
#include <time.h>
#include <string.h>
#include <QList>
#include <QDomDocument>
#include <QDomNodeList>
#include <QTextCodec>
#include <QDebug>
#include <map>
#include <QFile>
#include <QDate>
#include <QDataStream>
#include "hasp_api.h"
#include "hasp_vcode.h"
#include "cloudcompareutils/publicutils.h"




using namespace CS::SentinelLdkEncrypt;
namespace CS {
	namespace SentinelLdkEncrypt {
		class SentinelldkencryptPrivate :public QObject
		{
			Q_OBJECT
		public:
			explicit SentinelldkencryptPrivate(Sentinelldkencrypt *pQptr);


		private:
			/**
			*@brief 初始化解密狗特征内容
			*/
			void initSentinelLdkContent(void);


			
		private:
			friend class Sentinelldkencrypt;
			CS::SentinelLdkEncrypt::Sentinelldkencrypt *m_pQptr = nullptr;

		private:
			std::map<CS::SentinelLdkEncrypt::FeaturID, int> m_SentinelLdkContent;
			std::map<QString, QString> m_mapSentinelldkHint;
			QDateTime	m_licensingData;
			int	 m_lockID = -1;
		};
		
	

	}
}


SentinelldkencryptPrivate::SentinelldkencryptPrivate(Sentinelldkencrypt *pQptr)
{
	m_pQptr = pQptr;

	//[!].初始化加密内容
	initSentinelLdkContent();
}

void SentinelldkencryptPrivate::initSentinelLdkContent(void)
{
	//[!].初始化加密字段
	m_SentinelLdkContent.clear();
	for (int i = 0; i < int(CS::SentinelLdkEncrypt::FeaturID::Count); i++) {
		m_SentinelLdkContent.insert(std::pair<CS::SentinelLdkEncrypt::FeaturID, int>(CS::SentinelLdkEncrypt::FeaturID(i), 0));
	}


	//[!].注册提示语句
	m_mapSentinelldkHint.clear();

	//[!].未找到加密锁
	QString strHint = tr("Encryption lock was not found!");
	m_mapSentinelldkHint.insert(std::pair<QString, QString>("lock", strHint));
	return;
}




Sentinelldkencrypt::Sentinelldkencrypt(QObject *parent)
	:QObject(parent),
	m_pDptr(new SentinelldkencryptPrivate(this))
{
}

Sentinelldkencrypt * Sentinelldkencrypt::instance(void)
{
	static Sentinelldkencrypt instance;
	return &instance;
}
bool Sentinelldkencrypt::checkExists(void)
{
	bool bRet = false;
	hasp_status_t status;
	hasp_handle_t handle;

	//[!].登录加密狗
	status = hasp_login(hasp_feature_t(0), vendor_code, &handle);
	if (status != HASP_STATUS_OK) {
		qDebug("Login to feature: %d failed with status %d\n", 0, status);
		return false;
	}

	//[!].退出加密狗
	hasp_logout(handle);
	return true;
}
void Sentinelldkencrypt::readSentinelLdkFeatureContent(void)
{
	for (auto id : m_pDptr->m_SentinelLdkContent) {
		int nRet = readSentinelLdkFeature(id.first);
		m_pDptr->m_SentinelLdkContent[id.first] = nRet;
	}

	return;
}
int Sentinelldkencrypt::readSentinelLdkFeature(CS::SentinelLdkEncrypt::FeaturID id)
{
	return readSentinelLdkFeature(int(id));
}
int Sentinelldkencrypt::readSentinelLdkFeature(int id)
{
	hasp_status_t status;
	hasp_handle_t handle;

	//[!].获取特征数据
	status = hasp_login(hasp_feature_t(id), vendor_code, &handle);
	if (status != HASP_STATUS_OK) {
		qDebug("Login to feature: %d failed with status %d\n", id, status);
		return -1;
	}

	qDebug() << QString("Feature id:%1, status:%2").arg(int(id)).arg(status);
	//[!].退出加密狗
	hasp_logout(handle);
	return 0;
}


int Sentinelldkencrypt::readSentinelLdkFeatureData(const int & Featrueid, int & nResult)
{
    nResult = 0;
	//[!].获取授权日期
	QDateTime dogData;
	int nRet = getSentinelLicensingData(Featrueid, dogData);
	if (nRet == 1) {
		//[!].永久
		return nRet;
	}
	else if (nRet < 0){
		//[!].失败
		qDebug() << "error!";
		return -1;
	}

    //[!].获取加密狗的时间
    QDateTime curData;
    nRet = readerLockCurrentData(curData);
    if (nRet < 0){
        return -1;
    }
	
	//[!].日期授权
	uint stime = curData.toTime_t();
	uint etime = dogData.toTime_t();
	int ndaysec = 24 * 60 * 60;
	nResult = (etime - stime) / (ndaysec)+((etime - stime) % (ndaysec)+(ndaysec - 1)) / (ndaysec)-1;
	if (nResult < 0) {
		return -1;
	}
	return 0;
}

int CS::SentinelLdkEncrypt::Sentinelldkencrypt::getSentinelLockID(void)
{

	const char* format =
		"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
		"<haspformat root=\"hasp_info\">"
		"    <feature>"
		"        <attribute name=\"id\" />"
		"        <element name=\"license\" />"
		"        <hasp>"
		"          <attribute name=\"id\" />"
		"          <attribute name=\"type\" />"
		"        </hasp>"
		"    </feature>"
		"</haspformat>"
		"";

	bool bRet = false;
	hasp_status_t status;
	hasp_handle_t handle;
	char ** info = (char **)malloc(1024 * 2 * sizeof(char *));;

	//[!].登录加密狗
	status = hasp_login(0,
		vendor_code,
		&handle);

	if (status != HASP_STATUS_OK) {
		qDebug("Login to feature: %d failed with status %d\n", 0, status);
		return -1;
	}

	//[!].查询状态
	status = hasp_get_sessioninfo(handle, format, info);
	if (HASP_STATUS_OK != status) {
		qDebug() << "hasp_get sessioninfo is error! Login to feature: " << 0;
		return -1;
	}
    hasp_logout(handle);
	qDebug() << "sessioninfo data:" << info;
	QDomDocument document;
	if (!document.setContent(QByteArray(*info))) {
		qDebug() << "Error parsing information";
		return -1;
	}
   
	//[!].获取日期
	QDomNode RootNode;
	QDomNode TempNode;
	RootNode = document.documentElement();
	if (RootNode.isNull()) {
		qDebug() << "Error parsing information";
		return -1;
	}

	bool bHasp = false;
	int nID = -1;
	for (int i = 0; i < RootNode.childNodes().size(); i++) {
		if (RootNode.childNodes().at(i).toElement().nodeName() == "feature") {
			TempNode = RootNode.childNodes().at(i);
			QDomNodeList list = TempNode.childNodes();
			for (int j = 0; j < list.size(); j++) {
				TempNode = list.at(j);
				QString strName = TempNode.nodeName();
				if (strName == "hasp") {
					bHasp = true;
					nID = TempNode.toElement().attribute("id").toInt();
				}

			}

		}

	}
	if (!bHasp) {
		return -1;
	}

	return nID;
}

/**
			*@brief 读取特征加密
			*/
int Sentinelldkencrypt::readSentinelLdkEncrypt(CS::SentinelLdkEncrypt::FeaturID id)
{
	auto it = m_pDptr->m_SentinelLdkContent.find(id);
	if (it == m_pDptr->m_SentinelLdkContent.end()) {
		return -1;
	}
	return it->second;
}

QString Sentinelldkencrypt::getSentinelldkHint(const QString key)
{
	auto it = m_pDptr->m_mapSentinelldkHint.find(key);
	if (it == m_pDptr->m_mapSentinelldkHint.end()) {
		return QString::null;
	}

	return it->second;
}

int Sentinelldkencrypt::getSentinelLockType(QString &strAddress)
{

    char *info = 0;

    hasp_status_t status;
    hasp_handle_t handle;
   
    //[!].登录加密狗
    status = hasp_login(0,
        vendor_code,
        &handle);

    if (status != HASP_STATUS_OK) {
        qDebug("Login to feature: %d failed with status %d\n", 0, status);
        return -1;
    }
    status = hasp_get_sessioninfo(handle, HASP_KEYINFO, &info);

    /* check if operation was successful */
    if (status != HASP_STATUS_OK){
        return -1;
       
    }
    /* use the information, free it afterwards */

    QDomDocument document;
    if (!document.setContent(QByteArray(info))) {
        qDebug() << "Error parsing information";
        return -1;
    }
    hasp_free(info);
    hasp_logout(handle);

    //[!].获取日期
    QDomNode RootNode;
    QDomNode TempNode;
    RootNode = document.documentElement();
    if (RootNode.isNull()) {
        qDebug() << "Error parsing information";
        return -1;
    }

    QString strResult = QString::null;
    strAddress = QString::null;
    for (int i = 0; i < RootNode.childNodes().size(); i++) {
        QString strNodeName = RootNode.childNodes().at(i).toElement().nodeName();
        if (RootNode.childNodes().at(i).toElement().nodeName() == "keyspec") {

            TempNode = RootNode.childNodes().at(i);
            QDomNodeList list = TempNode.childNodes();
            for (int j = 0; j < list.size(); j++) {
                TempNode = list.at(j);
                QString strName = TempNode.nodeName();
                if (strName == "port") {
                    QDomNodeList nodelist = TempNode.childNodes();
                    for (int k = 0; k < nodelist.size(); k++) {
                        TempNode = nodelist.at(k);
                        strName = TempNode.nodeName();
                        if (strName.compare("type") == 0){
                            strResult =  TempNode.toElement().text();
                        } else if (strName.compare("address") == 0){
                            strAddress = TempNode.toElement().text();
                        }
                    }

                }

            }

        }

    }
    if (strResult.compare("USB") == 0){
        return 0;
    }
    else if (strResult.compare("IP") == 0)
    {
        return 1;
    }
   
    return -1;
}

int Sentinelldkencrypt::createSentinelLicenseC2VFile(const QString &strFileName)
{
    hasp_status_t status;
    hasp_handle_t handle;

    //[!].登录加密狗
    status = hasp_login(0,
        vendor_code,
        &handle);

    if (status != HASP_STATUS_OK) {
        qDebug("Login to feature: %d failed with status %d\n", 0, status);
        return -1;
    }

    char *info = 0;
    const char* format =
        "<haspformat format=\"updateinfo\"/>";
    status = hasp_get_sessioninfo(handle, format, &info);
    /* check if operation was successful */
    if (status != HASP_STATUS_OK){
        qDebug() << "get sessioninfo is error!";
        return status;
    }

    QByteArray byte(info);
    /* use the information, free it afterwards */
    hasp_free(info);
    hasp_logout(handle);

    QFile file(strFileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return -1;
    }
    QTextStream out(&file);
    out << byte;
    file.close();
    return 0;
}

int Sentinelldkencrypt::updateSentinelLicenseV2CFile(const QString & strFileName)
{

    hasp_status_t status;
    char *ack = 0;
    QTextCodec *code = QTextCodec::codecForName("gb18030");
    QString strPath=  code->fromUnicode(strFileName);

    QFile licenseV2C(strPath);
    if (!licenseV2C.exists()){
        qDebug() << "licnese file is exists";
        return -1;
    }

    QString strData;
    if (!licenseV2C.open(QIODevice::ReadOnly)){
        qDebug() << "open license file is error!";
        return -1;
    }
    strData = licenseV2C.readAll();
    licenseV2C.close();

    status = hasp_update(strData.toStdString().c_str(), &ack);

    /* check if operation was successful */
    if (status != HASP_STATUS_OK){
        qDebug() << "update file error！ " << status;
       
    }
    /* use the acknowledge data, free it afterwards */
    hasp_free(ack);
    return int(status);
}

int CS::SentinelLdkEncrypt::Sentinelldkencrypt::readerLockCurrentData(QDateTime & currenttime)
{
    hasp_status_t status;
    hasp_handle_t handle;
    char ** info = (char **)malloc(1024 * 2 * sizeof(char *));;

    //[!].登录加密狗
    status = hasp_login(0,
        vendor_code,
        &handle);

    if (status != HASP_STATUS_OK) {
        qDebug("Login to feature: %d failed with status %d\n", 0, status);
        return -1;
    }

    //[!].获取加密狗的时间
    hasp_time_t time;
    unsigned int day, month, year, hour, minute, second;
    status = hasp_get_rtc(handle, &time);

    /* check if operation was successful */
    if (status != HASP_STATUS_OK){
        return -1;
       
    }
    hasp_hasptime_to_datetime(time, &day, &month, &year, &hour, &minute, &second);
    QDate date;
    date.setDate(year, month, day);
    QTime timeNew;
    timeNew.setHMS(hour, minute, second);
    QDateTime dogData(date, timeNew);
    currenttime = dogData;
    return 0;
}

int Sentinelldkencrypt::getSentinelLicensingData(const int & Featrueid, QDateTime &data)
{
	const char* format =
		"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
		"<haspformat root=\"hasp_info\">"
		"    <feature>"
		"        <attribute name=\"id\" />"
		"        <element name=\"license\" />"
		"        <hasp>"
		"          <attribute name=\"id\" />"
		"          <attribute name=\"type\" />"
		"        </hasp>"
		"    </feature>"
		"</haspformat>"
		"";

	bool bRet = false;
	hasp_status_t status;
	hasp_handle_t handle;
	char ** info = (char **)malloc(1024 * 2 * sizeof(char *));;

	//[!].登录加密狗
	status = hasp_login(Featrueid,
		vendor_code,
		&handle);

	if (status != HASP_STATUS_OK) {
		qDebug("Login to feature: %d failed with status %d\n", Featrueid, status);
		return -1;
	}

	//[!].查询状态
	status = hasp_get_sessioninfo(handle, format, info);
	if (HASP_STATUS_OK != status) {
		qDebug() << "hasp_get sessioninfo is error! Login to feature: " << Featrueid;
		return -1;
	}

	qDebug() << "sessioninfo data:" << info;
	QDomDocument document;
	if (!document.setContent(QByteArray(*info))) {
		qDebug() << "Error parsing information";
		return -1;
	}

	//[!].获取日期
	QDomNode RootNode;
	QDomNode TempNode;
	RootNode = document.documentElement();
	if (RootNode.isNull()) {
		qDebug() << "Error parsing information";
		return -1;
	}

	hasp_time_t featureTime;
	bool bLicense = false;
	bool bPerpetual = false;
	for (int i = 0; i < RootNode.childNodes().size(); i++) {
		if (RootNode.childNodes().at(i).toElement().nodeName() == "feature") {
			TempNode = RootNode.childNodes().at(i).firstChild();
			if (TempNode.nodeName() == "license") {
				//[!].查看类型是否为日期类型
				for (int j = 0; j < TempNode.childNodes().size(); j++) {
					QString strNodeName = TempNode.childNodes().at(j).nodeName();
					if (strNodeName == "exp_date") {
						bLicense = true;
						featureTime = TempNode.childNodes().at(j).toElement().text().toInt();
					}
					else if (strNodeName == "license_type"){
						QString strValue = TempNode.childNodes().at(j).toElement().text();
						if (strValue.compare("perpetual") == 0) {
							//[!].永久
							bPerpetual = true;
						}
					}
				}

			}

		}

	}

	//[!].判断找到了没
	if (!bLicense && !bPerpetual) {
		qDebug() << "Error parsing information";
		return -1;
	}

	if (bPerpetual) {
		return 1;
	}

	unsigned int day;
	unsigned int month;
	unsigned int year;
	unsigned int hour;
	unsigned int minute;
	unsigned int second;
	hasp_hasptime_to_datetime(featureTime, &day, &month, &year, &hour, &minute, &second);
	QDate date;
	date.setDate(year, month, day);
	QTime time;
	time.setHMS(hour, minute, second);
	QDateTime dogData(date, time);
	data = dogData;
	return 0;
}





#include "sentinelldkencrypt.moc"