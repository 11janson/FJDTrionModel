#pragma once

// standard libraries
#include <vector>
#include <memory>
#include <thread>
#include <list>

#include <boost/utility.hpp>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
namespace NC
{
using io_service_t = boost::asio::io_service;
/**
 * @brief The io_service_pool class
 */
class io_service_pool : boost::noncopyable
{
public:
    using ios_work_ptr = std::unique_ptr<io_service_t::work>;

    class ios_worker
    {
    public:
        ios_worker()
            : ios_()
            , work_(boost::make_unique<io_service_t::work>(ios_))
        {}

        void start()
        {
            worker_ = boost::move(boost::thread{ boost::bind(&io_service_t::run, &ios_) });
        }

        void stop()
        {
            work_.reset();
            if (!ios_.stopped())
                ios_.stop();
        }

        void wait()
        {
            if (worker_.joinable())
                worker_.join();
        }

        io_service_t& get_io_service()
        {
            return ios_;
        }

    private:
        io_service_t ios_;
        ios_work_ptr    work_;
        boost::thread    worker_;
    };

    using iterator = std::list<ios_worker>::iterator;

public:
    explicit io_service_pool(size_t pool_size)
        : ios_workers_(pool_size)
        , next_io_service_(ios_workers_.begin())
    {
    }

    ~io_service_pool()
    {
        stop();
    }

    void start()
    {
        for (auto& ios_worker : ios_workers_)
            ios_worker.start();
    }

    void stop()
    {
        for (auto& ios : ios_workers_)
            ios.stop();

        for (auto& ios : ios_workers_)
            ios.wait();
    }

    io_service_t& get_io_service()
    {
        auto current = next_io_service_++;
        if (ios_workers_.end() == next_io_service_)
        {
            next_io_service_ = ios_workers_.begin();
        }

        return current->get_io_service();
    }

private:
    std::list<ios_worker>        ios_workers_;
    iterator                    next_io_service_;
};
}//namespace NC
