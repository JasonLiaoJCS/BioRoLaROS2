#pragma once
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>

namespace lifecycle {
extern volatile sig_atomic_t stop_signal;
extern std::atomic<bool> terminal_lost;
extern bool headless;
inline bool stopping() { return stop_signal != 0 || terminal_lost.load(); }
inline void signal_stop(int value) { stop_signal = value; }
inline void install_signals() {
    struct sigaction action{};
    action.sa_handler = signal_stop;
    sigemptyset(&action.sa_mask);
    for (int sig : {SIGINT, SIGTERM, SIGHUP, SIGPIPE})
        if (sigaction(sig, &action, nullptr) != 0)
            throw std::runtime_error("Cannot install stop signal handler");
}
inline void event(const char* message) {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    std::string boot;
    std::ifstream("/proc/sys/kernel/random/boot_id") >> boot;
    fprintf(stderr, "LIFECYCLE pid=%ld boot=%s mono=%ld.%03ld %s\n",
            static_cast<long>(getpid()), boot.c_str(), now.tv_sec,
            now.tv_nsec / 1000000, message);
    fflush(stderr);
}
inline void lose_terminal(const char* reason) {
    if (!terminal_lost.exchange(true)) event(reason);
}
// poll's error bits are reported even if no input was requested.
inline bool endpoint_lost(int input, int output) {
    pollfd fds[2] = {{input, POLLIN, 0}, {output, 0, 0}};
    int rc = poll(fds, 2, 0);
    if (rc < 0) return errno != EINTR;
    return ((fds[0].revents | fds[1].revents) & (POLLHUP | POLLERR | POLLNVAL)) != 0;
}
// Held before constructing FpgaHandler, through its destructor. Never unlink
// the lock file: doing so would let two processes lock different inodes.
class DriverLock {
    int fd_ = -1;
public:
    explicit DriverLock(const char* path = "/tmp/rslip-fpga-driver.lock") {
        fd_ = open(path, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            if (fd_ >= 0) close(fd_);
            throw std::runtime_error("Driver lock unavailable; no FPGA initialization");
        }
        // Also refuse an already-running legacy binary that predates this lock.
        DIR* proc = opendir("/proc");
        if (!proc) { close(fd_); throw std::runtime_error("Cannot inspect existing drivers"); }
        bool duplicate = false;
        while (dirent* entry = readdir(proc)) {
            char* end = nullptr;
            long pid = strtol(entry->d_name, &end, 10);
            if (!end || *end || pid <= 0 || pid == getpid()) continue;
            std::string comm;
            std::ifstream(std::string("/proc/") + entry->d_name + "/comm") >> comm;
            if (comm == "fpga_driver") { duplicate = true; break; }
        }
        closedir(proc);
        if (duplicate) { close(fd_); throw std::runtime_error("Existing fpga_driver; no FPGA initialization"); }
    }
    ~DriverLock() { if (fd_ >= 0) close(fd_); }
    DriverLock(const DriverLock&) = delete;
    DriverLock& operator=(const DriverLock&) = delete;
};
}
