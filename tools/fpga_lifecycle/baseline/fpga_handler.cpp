#include <fpga_handler.hpp>

ModuleIO::ModuleIO(NiFpga_Status _status, NiFpga_Session _fpga_session){

    status_ = _status;
    fpga_session_ = _fpga_session;

    r_motor_ = NiFpga_FPGA_POWER_RS485_v2_ControlU32_DCMotor;

    r_tx_ = NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8_DataTx;
    r_tx_size_ = NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size_DataTx;

    r_rx_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRX;
    r_rx_size_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRX;

    r_dir_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR;
    r_dir_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR2;
    r_dir_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR3;
    r_dir_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR4;
    r_dir_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR5;
    r_dir_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR6;

    r_en_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN;
    r_en_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN2;
    r_en_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN3;
    r_en_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN4;
    r_en_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN5;
    r_en_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN6;

    r_ep_[0] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition;
    r_ep_[1] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition2;
    r_ep_[2] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition3;
    r_ep_[3] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition4;
    r_ep_[4] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition5;
    r_ep_[5] = NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition6;

    r_iv_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage;
    r_iv_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage2;
    r_iv_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage3;
    r_iv_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage4;
    r_iv_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage5;
    r_iv_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage6;

    r_rp_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition;
    r_rp_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition2;
    r_rp_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition3;
    r_rp_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition4;
    r_rp_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition5;
    r_rp_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition6;

    r_he_[0] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect;
    r_he_[1] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect2;
    r_he_[2] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect3;
    r_he_[3] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect4;
    r_he_[4] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect5;   
    r_he_[5] = NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect6;

    r_state_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state;
    r_state_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state2;
    r_state_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state3;
    r_state_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state4;
    r_state_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state5;
    r_state_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlBool_state6;

    r_tc_[0] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount;
    r_tc_[1] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount2;
    r_tc_[2] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount3;
    r_tc_[3] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount4;
    r_tc_[4] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount5;
    r_tc_[5] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount6;

    r_cm_ = NiFpga_FPGA_POWER_RS485_v2_ControlU16_Controlmode;
    r_position_bus_[0] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_L1PositionBus1;
    r_position_bus_[1] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_L2PositionBus2;
    r_position_bus_[2] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_L3PositionBus3;
    r_position_bus_[3] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_R1PositionBus1;
    r_position_bus_[4] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_R2PositionBus2;
    r_position_bus_[5] = NiFpga_FPGA_POWER_RS485_v2_ControlU16_R3PositionBus3;

    r_position_encoder_[0] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus1;
    r_position_encoder_[1] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus2;
    r_position_encoder_[2] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus3;
    r_position_encoder_[3] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus1;
    r_position_encoder_[4] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus2;
    r_position_encoder_[5] = NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus3;

    r_stop_ = NiFpga_FPGA_POWER_RS485_v2_ControlBool_stop;

}


// Write FPGA status
void ModuleIO::write_motor_(uint32_t value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_motor_, value));
}
void ModuleIO::write_dir_(int index, NiFpga_Bool value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_dir_[index], value));
}

void ModuleIO::write_en_(int index, NiFpga_Bool value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_en_[index], value));
}

void ModuleIO::write_iv_(int index, uint16_t value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(fpga_session_, r_iv_[index], value));
}

void ModuleIO::write_rp_(int index, NiFpga_Bool value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_rp_[index], value));
}

void ModuleIO::write_state_(int index, NiFpga_Bool value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_state_[index], value));
}

void ModuleIO::write_tx_data_(const uint8_t *tx_arr){
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_, tx_arr, r_tx_size_));
}

void ModuleIO::write_cm_(uint16_t value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(fpga_session_, r_cm_, value));
}

void ModuleIO::write_position_bus_(int index, uint16_t value){
    NiFpga_MergeStatus(&status_, NiFpga_WriteU16(fpga_session_, r_position_bus_[index], value));
}

NiFpga_Bool ModuleIO::read_dir_(int index){
    NiFpga_Bool value;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_dir_[index], &value));
    return value;
}

NiFpga_Bool ModuleIO::read_he_(int index){
    NiFpga_Bool value;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_he_[index], &value));
    return value;
}
int32_t ModuleIO::read_ep_(int index){
    int32_t value;
    NiFpga_MergeStatus(&status_, NiFpga_ReadI32(fpga_session_, r_ep_[index], &value));
    return value;
}   

uint32_t ModuleIO::read_tc_(int index){
    uint32_t value;
    NiFpga_MergeStatus(&status_, NiFpga_ReadU32(fpga_session_, r_tc_[index], &value));
    return value;
}

void ModuleIO::read_rx_data_(uint8_t *rx_arr){
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_, rx_arr, r_rx_size_));
}

uint16_t ModuleIO::read_position_encoder_(int index){
    
    uint16_t result_value = 0; 
    NiFpga_MergeStatus(&status_, NiFpga_ReadU16(fpga_session_, r_position_encoder_[index], &result_value));

    return result_value;
}

NiFpga_Bool ModuleIO::read_stop_(){
    NiFpga_Bool value;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_stop_, &value));
    return value;
}

void ModuleIO::setSession(NiFpga_Session session){
    fpga_session_ = session;
}

uint16_t ModuleIO::read_cm_(){
    uint16_t value = 0;
    
    NiFpga_Status s = NiFpga_ReadU16(fpga_session_, r_cm_, &value);
    
    NiFpga_MergeStatus(&status_, s);

    if (NiFpga_IsError(s)) {
        return 9999;
    }

    return value;
}

FpgaHandler::FpgaHandler()
{
    status_ = NiFpga_Initialize();
    important_message("[FPGA Handler] Fpga Initialized");
    
    NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGA_POWER_RS485_v2_Bitfile,
                                             NiFpga_FPGA_POWER_RS485_v2_Signature, "RIO0", 0, &session_));

    if (NiFpga_IsError(status_)) 
    {
        std::cout << red << "[Fatal Error] Open Failed! Status: " << status_ << reset << std::endl;
        return; 
    }
    
    important_message("[FPGA Handler] Session opened (Success)");

    important_message("[FPGA Handler] Starting FPGA VI execution...");
    NiFpga_MergeStatus(&status_, NiFpga_Run(session_, 0));
    
    if (NiFpga_IsNotError(status_)) 
    {
        NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, NiFpga_FPGA_POWER_RS485_v2_ControlBool_stop, NiFpga_False));
        important_message("[FPGA Handler] Stop button reset to False.");
    }
    else
    {
         std::cout << red << "[Fatal Error] Run Failed! Status: " << status_ << reset << std::endl;
    }

    moduleIO = ModuleIO(status_, session_);
    
    w_pb_digital_ = NiFpga_FPGA_POWER_RS485_v2_ControlBool_Digital;
    w_pb_signal_ = NiFpga_FPGA_POWER_RS485_v2_ControlBool_Signal;
    w_pb_power_ = NiFpga_FPGA_POWER_RS485_v2_ControlBool_Power;

    r_powerboard_data_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16_Data;
    size_powerboard_data_ = NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size_Data;

    for (int i = 0; i < 8; i++)
    {
        powerboard_V_list_[i] = 0;
        powerboard_I_list_[i] = 0;
    }
}

FpgaHandler::~FpgaHandler()
{

    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    important_message("[FPGA Handler] Session Closed");

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    important_message("[FPGA Handler] Fpga Finalized");
}

void FpgaHandler::write_powerboard_(std::vector<bool> *powerboard_state_)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, powerboard_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, powerboard_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, powerboard_state_->at(2)));
}

void FpgaHandler::read_powerboard_data_()
{
    
    uint16_t rx_arr[16];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16_Data, rx_arr, NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size_Data));

    for (int i = 0; i < 16; i++)
    {
        if (i % 2 == 0)powerboard_I_list_[i / 2] = rx_arr[i] * powerboard_Ifactor[i / 2] + powerboard_Ioffset[i / 2];
        if (i % 2 == 1)powerboard_V_list_[(i - 1) / 2] = rx_arr[i] * powerboard_Vfactor[(i - 1) / 2] + powerboard_Voffset[(i - 1) / 2];
    }
}
