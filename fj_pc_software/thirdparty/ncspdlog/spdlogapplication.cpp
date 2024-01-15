#include "spdlogapplication.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#ifdef WIN32
#include <direct.h>
#include <io.h>
#endif
#ifdef linux
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

//#include "spdlog/sinks/daily_file_sink.h"
//#include "spdlog/sinks/rotating_file_sink.h"
//#include "spdlog/sinks/stdout_color_sinks.h"

using namespace std;
using namespace NC;

// 静态成员变量
//SpdlogApplication *SpdlogApplication::m_pInstance = nullptr;

SpdlogApplication *SpdlogApplication::instance()
{
    static SpdlogApplication instance;
    return &instance;
}
SpdlogApplication::SpdlogApplication()
{
    
    setLoggerName("root");
}
SpdlogApplication::~SpdlogApplication()
{
    
}
