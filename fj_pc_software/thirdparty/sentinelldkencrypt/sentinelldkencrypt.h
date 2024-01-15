#pragma once
#include <QObject>
#include "sentinelldkencrypt_global.h"
#include "sentinelldkfeaturecontent.h"

namespace CS {
	namespace SentinelLdkEncrypt {
		class SentinelldkencryptPrivate;
		class SENTINELLDKENCRYPT_EXPORT Sentinelldkencrypt :public QObject
		{
			Q_OBJECT
		private:
			explicit Sentinelldkencrypt(QObject *parent = Q_NULLPTR);
		public:
			/**
			*@ 获取解密狗实例
			*/
			static Sentinelldkencrypt* instance(void);

			/**
			*@brief 检测加密狗是否存在
			*bool true 存在， false 失败
			*/
			bool checkExists(void);

			/**
			*@brief 读取解密狗特征内容
			*return int 大于等于零成功 小于零失败
			*/
			void readSentinelLdkFeatureContent(void);

			/**
			*@brief 读取特征id
			*/
			int readSentinelLdkFeature(CS::SentinelLdkEncrypt::FeaturID id);

			/**
			*@brief 读取特征id
			*/
			int readSentinelLdkFeature(int id);


			/**
			*@brief 读取特征加密
			*/
			int readSentinelLdkEncrypt(CS::SentinelLdkEncrypt::FeaturID id);

			/**
			*@brief 读取特征数据
			*/
			int readSentinelLdkFeatureData(const int &Featrueid, int &nResult);

			/**
			*@brief 获取主锁id
			*@return 大于0成功 小于零失败
			*/
			int getSentinelLockID(void);

			/**
			*@brief 获取加密狗授权日期
			*@return 0日期授权，1永久， -1失败
			*/
			int getSentinelLicensingData(const int & Featrueid, QDateTime &time);
			/**
			*@brief 获取加密狗提示语
			*/
			QString getSentinelldkHint(const QString key);

            /**
            *@brief 获取加密锁类型 小于0失败，0本地锁， 1网络锁
            *@brief 并且返回地址
            */
            int getSentinelLockType(QString &strAddress);

            /**
            *@brief 创建C2V文件
            */
            int createSentinelLicenseC2VFile(const QString &strFileName);
            /**
            *@brief 
            */
            int updateSentinelLicenseV2CFile(const QString &strFileName);

            /**
            *@brief 读取加密锁当前时间
            */
            int readerLockCurrentData(QDateTime &time);
		private:
			friend class SentinelldkencryptPrivate;
			CS::SentinelLdkEncrypt::SentinelldkencryptPrivate *m_pDptr = nullptr;
		};
	}
}




