#include "spdlogmodule.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

using namespace std;
using namespace NC;

SpdlogModule::SpdlogModule(const std::string &loggerName)
{
    setLoggerName(loggerName);
}

SpdlogModule::~SpdlogModule()
{
    
}

std::shared_ptr<spdlog::logger> SpdlogModule::getLogger(const std::string &name)
{
    return spdlog::get(name);
}

bool SpdlogModule::containsLogger(const string &name)
{
    return spdlog::get(name) != nullptr;
}
