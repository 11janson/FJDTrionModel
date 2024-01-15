#pragma once

#include "ncshared/compilerdetection.h"

#if defined(NCSPDLOG_LIBRARY)
#  define NCSPDLOG_EXPORT NC_DECL_EXPORT
#elif  defined(NCSPDLOG_STATIC_LIB) // Abuse single files for manual tests
#  define NCSPDLOG_EXPORT
#else
#  define NCSPDLOG_EXPORT NC_DECL_IMPORT
#endif


#include <string>

//*****************************************************
//注意：
//文件名 __FILE__ ,函数名 __func__ ，行号__LINE__ 是编译器实现的
//并非C++头文件中定义的 
//前两个变量是string类型，且__LINE__是整形，所以需要转为string类型
//******************************************************
 
//整数类型文件行号 ->转换为string类型  std::to_string 处理
#ifdef _WIN32
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#else
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif
 
//定义一个在日志后添加 文件名 函数名 行号 的宏定义
#ifndef TRACE_SUFFIX
#ifndef NOVALOG_NO_OUTPUT_CONTEXT
#define TRACE_SUFFIX(msg)  std::string(msg).append(" \n //") \
        .append(__FILENAME__).append(":").append(__func__) \
        .append("()#").append(std::to_string(__LINE__)) \
        .append(".").c_str()
#else
#define TRACE_SUFFIX(msg)  msg
#endif
#endif

#if defined(SPDLOG_LEVEL_NAMES)
#undef SPDLOG_LEVEL_NAMES
#endif
#if !defined(SPDLOG_LEVEL_NAMES)
#define SPDLOG_LEVEL_NAMES                                                                                                                 \
    {                                                                                                                                      \
        "TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL", "OFF  "                                                                    \
    }
#endif
