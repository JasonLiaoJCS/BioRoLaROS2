#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include "NiFpga.h"
#include "NiFpga_FPGA_POWER_RS485_v2.h"
#include "Power.pb.h"
#include "Motor.pb.h"
#include "color.hpp"

#include <unistd.h>
#include <iostream>
#include <functional>
#include <signal.h>
#include <dlfcn.h>
#include <vector>
#include <ncurses.h>
#include <curses.h>
#include <iostream>
#include <bitset>
#include <string>
#undef OK

class ModuleIO{
public:
    ModuleIO(NiFpga_Status status_, NiFpga_Session fpga_session_);
    ModuleIO(){};

    NiFpga_Status status_;
    NiFpga_Session fpga_session_;
    void setSession(NiFpga_Session session);

    NiFpga_FPGA_POWER_RS485_v2_ControlU32 r_motor_;

    // tx buffer
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8 r_tx_;
    NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size r_tx_size_;

    // rx buffer
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8 r_rx_;
    NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size r_rx_size_;

    NiFpga_FPGA_POWER_RS485_v2_ControlBool r_dir_[6];
    NiFpga_FPGA_POWER_RS485_v2_ControlBool r_en_[6];
    NiFpga_FPGA_POWER_RS485_v2_IndicatorI32 r_ep_[6];
    NiFpga_FPGA_POWER_RS485_v2_ControlU16 r_iv_[6];
    NiFpga_FPGA_POWER_RS485_v2_ControlBool r_rp_[6];
    NiFpga_FPGA_POWER_RS485_v2_IndicatorBool r_he_[6];
    NiFpga_FPGA_POWER_RS485_v2_ControlBool r_state_[6];
    NiFpga_FPGA_POWER_RS485_v2_IndicatorU32 r_tc_[6];

    NiFpga_FPGA_POWER_RS485_v2_ControlU16 r_cm_;
    NiFpga_FPGA_POWER_RS485_v2_ControlU16 r_position_bus_[6];
    NiFpga_FPGA_POWER_RS485_v2_IndicatorU16 r_position_encoder_[6];

    NiFpga_FPGA_POWER_RS485_v2_ControlBool r_stop_;

    // read write function
    void write_motor_(uint32_t value);
    void write_dir_(int index, NiFpga_Bool value);
    void write_en_(int index, NiFpga_Bool value);
    void write_iv_(int index, uint16_t value);
    void write_rp_(int index, NiFpga_Bool value);
    void write_state_(int index, NiFpga_Bool value);
    void write_tx_data_(const uint8_t *tx_arr);

    void write_cm_(uint16_t value);
    void write_position_bus_(int index, uint16_t value);
    
    int32_t read_ep_(int index);
    NiFpga_Bool read_dir_(int index);
    NiFpga_Bool read_he_(int index);
    uint32_t read_tc_(int index);
    void read_rx_data_(uint8_t *rx_arr);
    uint16_t read_position_encoder_(int index);
    NiFpga_Bool read_stop_();
    uint16_t read_cm_();
};

class FpgaHandler
{
public:
  FpgaHandler();
  ~FpgaHandler();
  ModuleIO moduleIO;
  NiFpga_Session session_;
  NiFpga_Status status_;

  // powerboard
  NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_digital_;
  NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_signal_;
  NiFpga_FPGA_POWER_RS485_v2_ControlBool w_pb_power_;

  NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16 r_powerboard_data_;
  NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size size_powerboard_data_;

  void write_powerboard_(std::vector<bool> *powerboard_state_);
  void read_powerboard_data_();

  double powerboard_Ifactor[8];
  double powerboard_Ioffset[8];
  double powerboard_Vfactor[8];
  double powerboard_Voffset[8];

  double powerboard_I_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  double powerboard_V_list_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
