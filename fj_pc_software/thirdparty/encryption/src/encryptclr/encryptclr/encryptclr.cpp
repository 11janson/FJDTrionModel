#include "pch.h"
#include "encryptclr.h"
using namespace std;
using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;
using namespace System;
using namespace System::Collections::Generic;
using namespace System::IO;
using namespace System::Security::Cryptography;
using namespace System::Text;
using namespace System::Threading::Tasks;
using namespace Runtime::InteropServices;
using namespace Encrypt;



CS::EncryptClr::EncryptUtils::EncryptUtils()
{
}

string CS::EncryptClr::EncryptUtils::AesEncrypt(string sData, string strKey)
{
    
    System::String ^strSourceData = gcnew String(sData.c_str());
    System::String ^Key = gcnew String(strKey.c_str());
    System::String ^strData = Encrypt::EncryptUtils::AesEncrypt(strSourceData, Key);
    const char* chars = (const char*)(Marshal::StringToHGlobalAnsi(strData)).ToPointer();
    std::string strRet = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return strRet;
}

string CS::EncryptClr::EncryptUtils::AesDecrypt(string sData, string strKey)
{
   
    System::String ^strSourceData = gcnew String(sData.c_str());
    System::String ^Key = gcnew String(strKey.c_str());
    System::String ^strData = Encrypt::EncryptUtils::AesDecrypt(strSourceData, Key);
    const char* chars = (const char*)(Marshal::StringToHGlobalAnsi(strData)).ToPointer();
    std::string strRet = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return strRet;
}


string CS::EncryptClr::EncryptUtils::DesEncrypt(string sData, string strKey)
{
    System::String ^strSourceData = gcnew String(sData.c_str());
    System::String ^Key = gcnew String(strKey.c_str());
    System::String ^strData = Encrypt::EncryptUtils::DesEncrypt(strSourceData, Key);
    const char* chars = (const char*)(Marshal::StringToHGlobalAnsi(strData)).ToPointer();
    std::string strRet = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return strRet;
}

string CS::EncryptClr::EncryptUtils::DesDecrypt(string sData, string strKey)
{
    System::String ^strSourceData = gcnew String(sData.c_str());
    System::String ^Key = gcnew String(strKey.c_str());
    System::String ^strData = Encrypt::EncryptUtils::DesDecrypt(strSourceData, Key);
    const char* chars = (const char*)(Marshal::StringToHGlobalAnsi(strData)).ToPointer();
    std::string strRet = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return strRet;
}

string CS::EncryptClr::EncryptUtils::getMd5(string sData)
{
    System::String ^strSourceData = gcnew String(sData.c_str());
    System::String ^strData = Encrypt::EncryptUtils::getMd5(strSourceData);
    const char* chars = (const char*)(Marshal::StringToHGlobalAnsi(strData)).ToPointer();
    std::string strRet = chars;
    Marshal::FreeHGlobal(IntPtr((void*)chars));
    return strRet;
}
