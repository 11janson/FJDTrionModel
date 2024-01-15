#include "publicutils.h"
#include <QSettings>
#include <QDir>
#include <QTextCodec>
#include <QDateTime>
#include <QCoreApplication>
#include <windows.h>
#include <time.h>
#include <QSharedMemory>
#include <locale>
#include <codecvt>
#include <string>
#include <QDebug>

using namespace Utils;
int  PublicUtils::m_Colorvaluse[256];
int PublicUtils::m_nColorBright = 15;
int PublicUtils::m_nColorContrast = 25;

PublicUtils::PublicUtils(QObject *parent) : QObject(parent)
{

}


PublicUtils* Utils::PublicUtils::instance(void)
{
	static PublicUtils instance;
	return &instance;
}

bool Utils::PublicUtils::checkProcessActivationRuning(void)
{
    static QSharedMemory sharedMemory;
    bool bRun = false;
    QString strPid = "TrionModelActivation";
    //设置共享内存的标识，这个标识是确定的
    sharedMemory.setKey(strPid);
    if (sharedMemory.attach()){

        bRun = true;
    }
    else
    {
        sharedMemory.create(1);
    }

    return bRun;
}

bool Utils::PublicUtils::setApplicationAutoBoot(bool bBoot)
{
    QSettings reg("HKEY_CURRENT_USER\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Run", QSettings::NativeFormat);
    QString strAppPath = QDir::toNativeSeparators(QCoreApplication::applicationFilePath());
    QString strAppName = QFileInfo(strAppPath).baseName();

#ifdef Q_OS_WIN32
    if (bBoot) {
        reg.setValue(strAppName, strAppPath);
    }
    else {
        reg.remove(strAppName);
    }
#endif
    return false;
}

QString Utils::PublicUtils::GBKtoUTF8(QString strGBK)
{
    QTextCodec * codec = QTextCodec::codecForName("UTF-8");
    return codec->toUnicode(strGBK.toUtf8());
}

QByteArray Utils::PublicUtils::GBKtoGb(QString strGBK)
{
    QTextCodec *code = QTextCodec::codecForName("gb18030");
    return code->fromUnicode(strGBK);
}

QString  Utils::PublicUtils::GBKtoGb(QByteArray byteArray)
{
    QTextCodec *code = QTextCodec::codecForName("gb18030");
    return code->toUnicode(byteArray);
}




qint64 Utils::PublicUtils::currentMSecsSinceEpoch(void)
{
    SYSTEMTIME  systime;
    ::GetLocalTime(&systime);
    time_t curr_t = time(NULL);
    unsigned long long tld = (unsigned long long)curr_t;
    unsigned long millSec = (unsigned long)(systime.wMilliseconds);
    tld = tld * 1000 + millSec;
    return tld;

}

qint64 Utils::PublicUtils::getCurrentMSecsDifference(qint64 timeValue)
{
    SYSTEMTIME  systime;
    ::GetLocalTime(&systime);
    time_t curr_t = time(NULL);
    unsigned long long tld = (unsigned long long)curr_t;
    unsigned long millSec = (unsigned long)(systime.wMilliseconds);
    tld = tld * 1000 + millSec;
    return tld - timeValue;
}


std::string Utils::PublicUtils::convertUnicodeUTF8Code(const QString &strCode)
{

    std::wstring szRetDst = strCode.toStdWString();
    std::string strRet;
    char *psCharText;



    ////[!].检查字符编码格式 是否是否包含字符
    if (isGBKCode(strCode)) {
        //[!].判断是否为中文编码方式
        DWORD dwNum = WideCharToMultiByte(CP_ACP, NULL, szRetDst.c_str(), -1, NULL, 0, NULL, FALSE);
        psCharText = new char[dwNum];
        WideCharToMultiByte(CP_ACP, NULL, szRetDst.c_str(), -1, psCharText, dwNum, NULL, FALSE);
    }
    else if (isKoreanCode(strCode))
    {//[!].韩语朝鲜语

        DWORD dwNum = WideCharToMultiByte(51949, NULL, szRetDst.c_str(), -1, NULL, 0, NULL, FALSE);
        psCharText = new char[dwNum];
        WideCharToMultiByte(51949, NULL, szRetDst.c_str(), -1, psCharText, dwNum, NULL, FALSE);
    }
    else
    {
        //[!].其他一律采用UTF8编码
        DWORD dwNum = WideCharToMultiByte(65001, NULL, szRetDst.c_str(), -1, NULL, 0, NULL, FALSE);
        psCharText = new char[dwNum];
        WideCharToMultiByte(65001, NULL, szRetDst.c_str(), -1, psCharText, dwNum, NULL, FALSE);
    }


    strRet = psCharText;
    delete psCharText;

    psCharText = nullptr;
    return strRet;
}
bool Utils::PublicUtils::isGBKCode(const QString &strCode)
{
    if (strCode.contains(QRegExp("[\\x4e00-\\x9fa5]+"))) {
        return true;
    }
    return false;
}

bool Utils::PublicUtils::isKoreanCode(const QString & strCode)
{
    if (strCode.contains(QRegExp("[\\AC00-\\D7FFh]+"))) {
        return true;
    }
    return false;
}

bool Utils::PublicUtils::isUTF8Code(const QString &strCode)
{
    if (strCode.isEmpty()) {
        return false;
    }
    const char* str = strCode.toStdString().c_str();
    unsigned int nBytes = 0;//UFT8可用1-6个字节编码,ASCII用一个字节
    unsigned char chr = *str;
    bool bAllAscii = true;
    for (unsigned int i = 0; str[i] != '\0'; ++i) {
        chr = *(str + i);
        if (nBytes == 0 && (chr & 0x80) != 0) {
            bAllAscii = false;
        }
        if (nBytes == 0) {
            //如果不是ASCII码,应该是多字节符,计算字节数
            if (chr >= 0x80) {
                if (chr >= 0xFC && chr <= 0xFD) {
                    nBytes = 6;
                }
                else if (chr >= 0xF8) {
                    nBytes = 5;
                }
                else if (chr >= 0xF0) {
                    nBytes = 4;
                }
                else if (chr >= 0xE0) {
                    nBytes = 3;
                }
                else if (chr >= 0xC0) {
                    nBytes = 2;
                }
                else {
                    return false;
                }
                nBytes--;
            }
        }
        else {
            //多字节符的非首字节,应为 10xxxxxx
            if ((chr & 0xC0) != 0x80) {
                return false;
            }
            //减到为零为止
            nBytes--;
        }
    }
    //违返UTF8编码规则
    if (nBytes != 0) {
        return false;
    }
    if (bAllAscii) { //如果全部都是ASCII, 也是UTF8
        return true;
    }
    return true;
}

std::string Utils::PublicUtils::recDecimalUshortToHex(ushort num)
{
    std::string str;
    ushort tempNum = num / 16;
    ushort left = num % 16;
    if (tempNum > 0)
    {
        str += recDecimalUshortToHex(tempNum);
    }
    if (left < 10)
    {
        str += (left + '0');
    }
    else
    {
        str += ('A' + left - 10);
    }
    return str;
}

std::string Utils::PublicUtils::QStringToStdStringUnicode(QString str)
{

	std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;

	// 输入韩文字符串
	std::wstring korean_str = str.toStdWString();

	// 将韩文字符串转换为 std::string 类型
	std::string utf8_str = converter.to_bytes(korean_str);

    return utf8_str;
}

std::string Utils::PublicUtils::unicodeToCharacters(std::string str)
{
    return str;
}

int Utils::PublicUtils::exeCheckColorValue(int nValue)
{
    return nValue <= 0 ? 0 : nValue >= 255 ? 255 : nValue;
}

int Utils::PublicUtils::updateRGBColorBrightContrastValue(int iBright, int iContrast)
{

	m_nColorBright = std::clamp(iBright, 0, 100);
	m_nColorContrast = std::clamp(iContrast, 0, 100);
	return 0;
}


int Utils::PublicUtils::exeInitRGBColorBrightContrastValue(int iBright, int iContrast, std::byte threshold)
{
    float cv = iContrast <= -255 ? -1.0f : iContrast / 255.0f;
    if (iContrast > 0 && iContrast < 255) {
        cv = 1.0f / (1.0f - cv) - 1.0f;
    }

    for (int i = 0; i < 256; i++) {
        int v = iContrast > 0 ? exeCheckColorValue(i + iBright) : i;
        if (iContrast >= 255) {
            v = v >= int(threshold) ? 255 : 0;
        }
        else {
            v = exeCheckColorValue(v + (int)((v - int(threshold)) * cv + 0.5f));
        }
        m_Colorvaluse[i] = iContrast < 0 ? exeCheckColorValue(v + iBright) : v;
    }
    return 0;
}


float Utils::PublicUtils::getRGBColorBright(void)
{
	return std::clamp(m_nColorBright / 100.0, 0.0, 1.0);
}

float Utils::PublicUtils::getRGBColorContrast(void)
{
	return std::clamp(m_nColorContrast / 100.0, 0.0, 1.0);
}







