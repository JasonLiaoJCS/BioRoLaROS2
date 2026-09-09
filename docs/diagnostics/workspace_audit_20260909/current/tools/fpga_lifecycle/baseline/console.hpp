#ifndef RSLIP_CONSOLE_HPP
#define RSLIP_CONSOLE_HPP

#include "fpga_handler.hpp"
#include <atomic>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Console {
public:
    Console() = default;
    ~Console();
    void init(FpgaHandler*, std::vector<bool>*, std::mutex*);
    void stop();
private:
    void run();
    std::string execute(const std::string&);
    FpgaHandler* fpga_ = nullptr;
    std::vector<bool>* power_ = nullptr;
    std::mutex* device_mutex_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread ui_;
    FILE* terminal_out_ = nullptr;
    FILE* terminal_in_ = nullptr;
    int saved_stdout_ = -1;
    int saved_stderr_ = -1;
};
#endif
