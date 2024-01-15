#pragma once
#include <QOBject>
#include "cloudcompareutils_global.h"
#include <cstddef> 

namespace Utils {
	class TRIONMETAHUBUTILS_EXPORT PublicUtils : public QObject
	{
		Q_OBJECT
	private:
		explicit PublicUtils(QObject *parent = nullptr);
	public:
	
		static PublicUtils* instance(void);


        static bool checkProcessActivationRuning(void);

        /**
      *@brief 设置软件自启动
      */
        bool setApplicationAutoBoot(bool bBoot);
        /**
        *@brief 中文转化为UTF-8
        */
        static  QString GBKtoUTF8(QString strGBK);

        /**
        *@brief 中文字符转换
        */
        static QByteArray GBKtoGb(QString strGBK);

        /**
       *@brief 中文字符转换
       */
        static QString  GBKtoGb(QByteArray byteArray);
        /**
        *@brief 获取当前时间锉
        */
        static qint64 currentMSecsSinceEpoch(void);

        /**
        *@brief 获取当前时间差
        */
        static qint64 getCurrentMSecsDifference(qint64 time);

        /**
        *@brief 转换字符编码到
        */
        static std::string convertUnicodeUTF8Code(const QString &strCode);

        /**
        *@brief 检查是否为UTF8编码字符串
        */
        static bool isUTF8Code(const QString &strCode);

        /**
        *@brief 检查是否包含中文编码
        */
        static bool isGBKCode(const QString &strCode);

        /**
        *@brief 检查是否包含韩语编码
        */
        static bool isKoreanCode(const QString &strCode);
        std::string recDecimalUshortToHex(ushort);
        //[!]将QString 导出为 16进制 unicode 编码
		static std::string QStringToStdStringUnicode(QString);
        //[!]将unicode编码转换为文字
        std::string unicodeToCharacters(std::string);

        /**
         *@brief 颜色值
        */
        static int exeCheckColorValue(int nValue);

		/**
		*@brief gengx 对比度亮度
		*/
		static int updateRGBColorBrightContrastValue(int iBright, int iContrast);

        /**
		*@brief 执行RGB 对比度和亮度颜色值
		*/
        static  int exeInitRGBColorBrightContrastValue(int iBright, int iContrast, std::byte threshold);

		/**
		*@brief 获取RGB颜色亮度值
		*/
		static float getRGBColorBright(void);

		/**
		*@brief 获取RGB颜色对比度
		*/
		static float getRGBColorContrast(void);

	
    protected:
		static int m_nColorBright;
		static int m_nColorContrast;
		static int  m_Colorvaluse[256];
	   
	};

}

