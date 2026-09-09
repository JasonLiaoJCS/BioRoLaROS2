#pragma once
#include <cstdint>
#include <cstdio>
constexpr bool NiFpga_True = true;
constexpr bool NiFpga_False = false;
struct ModuleIO {
    int32_t read_ep_(int i) { return 100+i; }
    uint32_t read_tc_(int i) { return 200+i; }
    bool read_he_(int i) { return i == 2; }
    uint16_t read_position_encoder_(int i) { return 300+i; }
    uint16_t read_cm_() { return 1; }
    void write_en_(int i, bool v) { printf("WRITE E %d %d\n",i,v); }
    void write_dir_(int i, bool v) { printf("WRITE D %d %d\n",i,v); }
    void write_iv_(int i, uint16_t v) { printf("WRITE I %d %u\n",i,v); }
    void write_state_(int i, bool v) { printf("WRITE S %d %d\n",i,v); }
    void write_rp_(int i, bool v) { printf("WRITE R %d %d\n",i,v); }
    void write_position_bus_(int i, uint16_t v) { printf("WRITE P %d %u\n",i,v); }
};
struct FpgaHandler {
    ModuleIO moduleIO;
    double powerboard_V_list_[8] = {23.1};
    double powerboard_I_list_[8] = {0.8};
};
