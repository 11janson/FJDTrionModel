#pragma once

#if defined(_WIN32)

#include "spdlog/sinks/base_sink.h"
#include "spdlog/details/null_mutex.h"

#include <mutex>
#include <string>
#include <QString>

namespace spdlog
{
    namespace sinks
    {
        using logviewer_cb_handler = std::function<void(const QString &err_msg, const int &_severity_level)>;
        /*
        * MSVC sink (logging using OutputDebugStringA)
        */
        template<class Mutex>
        class logviewer_sink : public base_sink < Mutex >
        {
        public:
            logviewer_sink(logviewer_cb_handler cb)
                :_cb(cb)
            {
            }

        protected:
            virtual void sink_it_(const details::log_msg &msg)  override
            {
                if(nullptr!=_cb)
                {
                    fmt::memory_buffer formatted;
                    base_sink<Mutex>::formatter_->format(msg, formatted);
                    _cb(QString::fromLocal8Bit(formatted.data(),formatted.size()),msg.level);
                }
            }

            virtual void flush_() override
            {

            }

            logviewer_cb_handler _cb;
        };

        typedef logviewer_sink<std::mutex> logviewer_sink_mt;
        typedef logviewer_sink<details::null_mutex> logviewer_sink_st;

    }
}

#endif
