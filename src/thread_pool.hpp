#pragma once
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

// Persistent worker pool: threads are created once and parked on a condition
// variable between jobs. parallelFor hands out blocks of the index range
// through an atomic counter, so uneven work (dense flocks next to empty
// space) stays balanced; the calling thread takes blocks too.
class ThreadPool {
public:
    // `threads` counts the caller; 0 means one per hardware thread.
    explicit ThreadPool(unsigned threads = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    [[nodiscard]] unsigned size() const noexcept { return static_cast<unsigned>(workers_.size()) + 1; }

    // Calls fn(begin, end) for consecutive blocks of [0, n), `block` indices
    // each, and returns once all of them are done.
    template <class Fn>
    void parallelFor(int n, int block, Fn&& fn) {
        if (n <= 0) return;
        if (workers_.empty() || n <= block) { fn(0, n); return; }
        using F = std::remove_reference_t<Fn>;
        const Job job{
            .n     = n,
            .block = std::max(block, 1),
            .ctx   = std::addressof(fn),
            .call  = [](const void* ctx, int begin, int end) {
                (*const_cast<F*>(static_cast<const F*>(ctx)))(begin, end);
            },
        };
        dispatch(job);
    }

private:
    // A type-erased reference to the caller's callable; it outlives the job.
    struct Job {
        int         n     = 0;
        int         block = 1;
        const void* ctx   = nullptr;
        void (*call)(const void*, int, int) = nullptr;
    };

    void dispatch(const Job& job);
    void work(const Job& job);   // takes blocks until the range is exhausted
    void workerLoop();

    std::vector<std::thread> workers_;
    std::mutex               mutex_;
    std::condition_variable  wake_, done_;
    const Job*               job_  = nullptr;
    std::atomic<int>         next_ = 0;
    std::size_t              busy_ = 0;   // workers still on the current job
    std::uint64_t            generation_ = 0;
    bool                     stop_ = false;
};
