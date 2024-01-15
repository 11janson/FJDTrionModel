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
			*@ ��ȡ���ܹ�ʵ��
			*/
			static Sentinelldkencrypt* instance(void);

			/**
			*@brief �����ܹ��Ƿ����
			*bool true ���ڣ� false ʧ��
			*/
			bool checkExists(void);

			/**
			*@brief ��ȡ���ܹ���������
			*return int ���ڵ�����ɹ� С����ʧ��
			*/
			void readSentinelLdkFeatureContent(void);

			/**
			*@brief ��ȡ����id
			*/
			int readSentinelLdkFeature(CS::SentinelLdkEncrypt::FeaturID id);

			/**
			*@brief ��ȡ����id
			*/
			int readSentinelLdkFeature(int id);


			/**
			*@brief ��ȡ��������
			*/
			int readSentinelLdkEncrypt(CS::SentinelLdkEncrypt::FeaturID id);

			/**
			*@brief ��ȡ��������
			*/
			int readSentinelLdkFeatureData(const int &Featrueid, int &nResult);

			/**
			*@brief ��ȡ����id
			*@return ����0�ɹ� С����ʧ��
			*/
			int getSentinelLockID(void);

			/**
			*@brief ��ȡ���ܹ���Ȩ����
			*@return 0������Ȩ��1���ã� -1ʧ��
			*/
			int getSentinelLicensingData(const int & Featrueid, QDateTime &time);
			/**
			*@brief ��ȡ���ܹ���ʾ��
			*/
			QString getSentinelldkHint(const QString key);

            /**
            *@brief ��ȡ���������� С��0ʧ�ܣ�0�������� 1������
            *@brief ���ҷ��ص�ַ
            */
            int getSentinelLockType(QString &strAddress);

            /**
            *@brief ����C2V�ļ�
            */
            int createSentinelLicenseC2VFile(const QString &strFileName);
            /**
            *@brief 
            */
            int updateSentinelLicenseV2CFile(const QString &strFileName);

            /**
            *@brief ��ȡ��������ǰʱ��
            */
            int readerLockCurrentData(QDateTime &time);
		private:
			friend class SentinelldkencryptPrivate;
			CS::SentinelLdkEncrypt::SentinelldkencryptPrivate *m_pDptr = nullptr;
		};
	}
}




