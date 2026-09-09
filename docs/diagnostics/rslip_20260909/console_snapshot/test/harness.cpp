#include "console.hpp"
#include <csignal>
#include <chrono>
#include <iostream>
volatile sig_atomic_t done = 0;
void finish(int) { done = 1; }
int main() {
    std::signal(SIGINT, finish);
    std::signal(SIGTERM, finish);
    FpgaHandler fpga;
    std::vector<bool> power{false,false,false};
    std::mutex mutex;
    Console console;
    console.init(&fpga, &power, &mutex);
    for (int n=0; n<300 && !done; ++n) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            std::cout << "TEST_LOG_NOISE STATE=" << power[0] << power[1] << power[2] << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    console.stop();
    std::cout << "HARNESS_EXIT_OK" << std::endl;
}
