#pragma once

#include "cloudcomparewidgets_global.h"
#include <windows.h>
#include<iostream>
#include<vector>
#include<QString>
#include<QMap>
namespace CS
{
    namespace RosMemory
    {

        class TRIONMETAHUBWIDGETS_EXPORT RosMemorySharing
        {
        public:
            /**
            * @brief 创建S1yaml文件
            */
            bool createMainYamlFiles(QMap<QString, QString>);
            /**
            * @brief 创建S1yaml文件
            */
            void createYamlFiles(QMap<QString,QString>, std::istream& input);
            /**
            * @brief 创建P1yaml文件
            */
            void createP1YamlFiles(QMap<QString, QString>, std::istream& input);
            /**
            * @brief 写入共享内存
            */
            void setSharedMemory(std::string);
            /**
            * @brief 读入共享内存
            */
            char* getSharedMemory();
            /**
            * @brief 关闭共享内存
            */
            void closeSharedMemory();
            /**
             * @brief 深拷贝
             */
            void deepCopyChar(char*, char*);
            /**
             * @brief 调试算法读取yaml文件字符编码
             */
            void slamGetyamlData(std::string yamlpath);

            /**
             * @brief 编码
             */
            std::wstring UtfToGbk(std::string strValue);

            void removeTrailingNewline(std::string& str);
        private:
            HANDLE hFileMap_Handle;
            LPVOID pBuffer;
        };
    }
}




