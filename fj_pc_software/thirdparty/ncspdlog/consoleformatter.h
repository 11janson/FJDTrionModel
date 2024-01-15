#pragma once
#include "ncspdlog_global.h"
#include "spdlog/formatter.h"
#include "spdlog/fmt/fmt.h"
#include "spdlog/details/log_msg.h"
#include "spdlog/details/fmt_helper.h"
#include "spdlog/details/pattern_formatter.h"
namespace spdlog{
#ifdef WIN32
//UTF-8格式转成GB2312格式
static char* Utf8ToGB2312(const char* utf8)
{
    int len = MultiByteToWideChar(CP_UTF8, 0, utf8, -1, NULL, 0);
    wchar_t* wstr = new wchar_t[len + 1];
    memset(wstr, 0, len + 1);
    MultiByteToWideChar(CP_UTF8, 0, utf8, -1, wstr, len);
    len = WideCharToMultiByte(CP_ACP, 0, wstr, -1, NULL, 0, NULL, NULL);
    char* str = new char[len + 1];
    memset(str, 0, len + 1);
    WideCharToMultiByte(CP_ACP, 0, wstr, -1, str, len, NULL, NULL);
    if (wstr) delete[] wstr;
    return str;
}
//Unicode到GB2312的转换
static int UnicodeToGB2312(const char* unicode, int size, char*gb2312)
{
    UINT nCodePage = 936; //GB2312
    wchar_t* wstr = new wchar_t[size/2+1];
    memcpy(wstr, unicode, size);
    int len = WideCharToMultiByte(nCodePage, 0, wstr, -1, NULL, 0, NULL, NULL);
    WideCharToMultiByte(nCodePage, 0, wstr, -1, gb2312, len, NULL, NULL);
    if(wstr) delete[] wstr;
    return len;
}
#endif
class console_formatter final : public formatter
{
public:
    explicit console_formatter(
        std::string pattern, pattern_time_type time_type = pattern_time_type::local, std::string eol = spdlog::details::os::default_eol)
        : pattern_(std::move(pattern))
        , eol_(std::move(eol))
        , pattern_time_type_(time_type)
        , last_log_secs_(0)
    {
        std::memset(&cached_tm_, 0, sizeof(cached_tm_));
        compile_pattern_(pattern_);
    }

    console_formatter(const console_formatter &other) = delete;
    console_formatter &operator=(const console_formatter &other) = delete;

    std::unique_ptr<formatter> clone() const override
    {
        return details::make_unique<console_formatter>(pattern_, pattern_time_type_, eol_);
    }

    void format(const details::log_msg &msg, memory_buf_t &dest)
    {
#ifndef SPDLOG_NO_DATETIME
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(msg.time.time_since_epoch());
        if (secs != last_log_secs_)
        {
            cached_tm_ = get_time_(msg);
            last_log_secs_ = secs;
        }
#endif
        for (auto &f : formatters_)
        {
            f->format(msg, cached_tm_, dest);
        }
        // write eol
        details::fmt_helper::append_str(eol_, dest);
#ifdef WIN32
        // 中文转换
        char* chGB2312 = Utf8ToGB2312(dest.data());               
        dest.clear();
        dest.resize(0);
        fmt::format_to(dest,chGB2312);
        delete []chGB2312;
        chGB2312=nullptr;                
#endif
    }

private:
    std::string pattern_;
    std::string eol_;
    pattern_time_type pattern_time_type_;
    std::tm cached_tm_;
    std::chrono::seconds last_log_secs_;

    std::vector<std::unique_ptr<details::flag_formatter>> formatters_;

    std::tm get_time_(const details::log_msg &msg)
    {
        if (pattern_time_type_ == pattern_time_type::local)
        {
            return details::os::localtime(log_clock::to_time_t(msg.time));
        }
        return details::os::gmtime(log_clock::to_time_t(msg.time));
    }

    void handle_flag_(char flag)
    {
        switch (flag)
        {
        // logger name
        case 'n':
            formatters_.push_back(details::make_unique<details::name_formatter>());
            break;

        case 'l':
            formatters_.push_back(details::make_unique<details::level_formatter>());
            break;

        case 'L':
            formatters_.push_back(details::make_unique<details::short_level_formatter>());
            break;

        case ('t'):
            formatters_.push_back(details::make_unique<details::t_formatter>());
            break;

        case ('v'):
            formatters_.push_back(details::make_unique<details::v_formatter>());
            break;

        case ('a'):
            formatters_.push_back(details::make_unique<details::a_formatter>());
            break;

        case ('A'):
            formatters_.push_back(details::make_unique<details::A_formatter>());
            break;

        case ('b'):
        case ('h'):
            formatters_.push_back(details::make_unique<details::b_formatter>());
            break;

        case ('B'):
            formatters_.push_back(details::make_unique<details::B_formatter>());
            break;
        case ('c'):
            formatters_.push_back(details::make_unique<details::c_formatter>());
            break;

        case ('C'):
            formatters_.push_back(details::make_unique<details::C_formatter>());
            break;

        case ('Y'):
            formatters_.push_back(details::make_unique<details::Y_formatter>());
            break;

        case ('D'):
        case ('x'):
            formatters_.push_back(details::make_unique<details::D_formatter>());
            break;

        case ('m'):
            formatters_.push_back(details::make_unique<details::m_formatter>());
            break;

        case ('d'):
            formatters_.push_back(details::make_unique<details::d_formatter>());
            break;

        case ('H'):
            formatters_.push_back(details::make_unique<details::H_formatter>());
            break;

        case ('I'):
            formatters_.push_back(details::make_unique<details::I_formatter>());
            break;

        case ('M'):
            formatters_.push_back(details::make_unique<details::M_formatter>());
            break;

        case ('S'):
            formatters_.push_back(details::make_unique<details::S_formatter>());
            break;

        case ('e'):
            formatters_.push_back(details::make_unique<details::e_formatter>());
            break;

        case ('f'):
            formatters_.push_back(details::make_unique<details::f_formatter>());
            break;
        case ('F'):
            formatters_.push_back(details::make_unique<details::F_formatter>());
            break;

        case ('E'):
            formatters_.push_back(details::make_unique<details::E_formatter>());
            break;

        case ('p'):
            formatters_.push_back(details::make_unique<details::p_formatter>());
            break;

        case ('r'):
            formatters_.push_back(details::make_unique<details::r_formatter>());
            break;

        case ('R'):
            formatters_.push_back(details::make_unique<details::R_formatter>());
            break;

        case ('T'):
        case ('X'):
            formatters_.push_back(details::make_unique<details::T_formatter>());
            break;

        case ('z'):
            formatters_.push_back(details::make_unique<details::z_formatter>());
            break;

        case ('+'):
            formatters_.push_back(details::make_unique<details::full_formatter>());
            break;

        case ('P'):
            formatters_.push_back(details::make_unique<details::pid_formatter>());
            break;
#ifdef SPDLOG_ENABLE_MESSAGE_COUNTER
        case ('i'):
            formatters_.push_back(details::make_unique<details::i_formatter>());
            break;
#endif
        case ('^'):
            formatters_.push_back(details::make_unique<details::color_start_formatter>());
            break;

        case ('$'):
            formatters_.push_back(details::make_unique<details::color_stop_formatter>());
            break;

        default: // Unknown flag appears as is
            formatters_.push_back(details::make_unique<details::ch_formatter>('%'));
            formatters_.push_back(details::make_unique<details::ch_formatter>(flag));
            break;
        }
    }

    void compile_pattern_(const std::string &pattern)
    {
        auto end = pattern.end();
        std::unique_ptr<details::aggregate_formatter> user_chars;
        formatters_.clear();
        for (auto it = pattern.begin(); it != end; ++it)
        {
            if (*it == '%')
            {
                if (user_chars) // append user chars found so far
                {
                    formatters_.push_back(std::move(user_chars));
                }
                if (++it != end)
                {
                    handle_flag_(*it);
                }
                else
                {
                    break;
                }
            }
            else // chars not following the % sign should be displayed as is
            {
                if (!user_chars)
                {
                    user_chars = details::make_unique<details::aggregate_formatter>();
                }
                user_chars->add_ch(*it);
            }
        }
        if (user_chars) // append raw chars found so far
        {
            formatters_.push_back(std::move(user_chars));
        }
    }
};
}//namespace NC