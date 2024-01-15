#pragma once
#include <iostream>
using namespace std;
#if defined(CSENCRYPTCLR_LIBRARY)
#  define CSENCRYPTCLR_LIBRARY __declspec(dllexport)
#else
#  define CSENCRYPTCLR_LIBRARY __declspec(dllimport)
#endif

namespace CS {

    namespace EncryptClr
    {
        class CSENCRYPTCLR_LIBRARY EncryptUtils
        {
        public:
            EncryptUtils();

        public:
            /**
            *@brief Aes加密
            */
            static string AesEncrypt(string sData, string strKey);

            /**
            *@brief Aes解密
            */
            static string AesDecrypt(string sData, string strKey);

            /**
            *@brief Des加密
            */
            static string DesEncrypt(string sData, string strKey);

            /**
            *@brief Des解密
            */
            static string DesDecrypt(string sData, string strKey);

            /*
            *@brief 获取Md5值
            */
            static string getMd5(string sData);


        };

    }
}
