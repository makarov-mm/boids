#include "thread_pool.hpp"

ThreadPool::ThreadPool(unsigned threads) {
    if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
    workers_.reserve(threads - 1);
    for (unsigned t = 1; t < threads; ++t) workers_.emplace_back([this] { workerLoop(); });
}

ThreadPool::~ThreadPool() {
    {
        const std::lock_guard lock(mutex_);
        stop_ = true;
    }
    wake_.notify_all();
    for (std::thread& w : workers_) w.join();
}

void ThreadPool::dispatch(const Job& job) {
    {
        const std::lock_guard lock(mutex_);
        job_ = &job;
        next_.store(0, std::memory_order_relaxed);
        busy_ = workers_.size();
        ++generation_;
    }
    wake_.notify_all();

    work(job);

    // The workers publish their writes by leaving the job under the mutex.
    std::unique_lock lock(mutex_);
    done_.wait(lock, [this] { return busy_ == 0; });
}

void ThreadPool::work(const Job& job) {
    for (;;) {
        const int begin = next_.fetch_add(job.block, std::memory_order_relaxed);
        if (begin >= job.n) return;
        job.call(job.ctx, begin, std::min(begin + job.block, job.n));
    }
}

void ThreadPool::workerLoop() {
    std::uint64_t seen = 0;
    std::unique_lock lock(mutex_);
    for (;;) {
        wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
        if (stop_) return;
        seen = generation_;
        const Job* job = job_;

        lock.unlock();
        work(*job);
        lock.lock();

        if (--busy_ == 0) done_.notify_one();
    }
}
