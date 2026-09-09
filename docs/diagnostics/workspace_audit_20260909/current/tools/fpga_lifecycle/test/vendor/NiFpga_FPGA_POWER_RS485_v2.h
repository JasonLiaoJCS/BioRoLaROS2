/*
 * Generated with the FPGA Interface C API Generator 19.0
 * for NI-RIO 19.0 or later.
 */
#ifndef __NiFpga_FPGA_POWER_RS485_v2_h__
#define __NiFpga_FPGA_POWER_RS485_v2_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 190
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGA_POWER_RS485_v2_Bitfile;
 */
#define NiFpga_FPGA_POWER_RS485_v2_Bitfile "NiFpga_FPGA_POWER_RS485_v2.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGA_POWER_RS485_v2_Signature = "8C0C6BDE5024BA4053EF4E6ADBF5DB00";

#if NiFpga_Cpp
extern "C"
{
#endif

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_ChecksumOK = 0x1802E,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect = 0x180A6,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect2 = 0x180BE,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect3 = 0x180FE,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect4 = 0x1812A,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect5 = 0x18156,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_Hall_effect6 = 0x18182,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinish = 0x18032,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus1 = 0x1804A,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus2 = 0x18062,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorBool_RXfinishBus3 = 0x1807A,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorBool;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU8_Information = 0x18016,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorU8;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus1 = 0x18042,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus2 = 0x1805A,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID1EncoderBus3 = 0x18072,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus1 = 0x1803E,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus2 = 0x18056,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU16_ID2EncoderBus3 = 0x1806E,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorU16;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition = 0x18098,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition2 = 0x180C8,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition3 = 0x180F0,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition4 = 0x1811C,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition5 = 0x18148,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_EncoderPosition6 = 0x18174,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset = 0x180A0,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset2 = 0x180D0,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset3 = 0x180F8,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset4 = 0x18124,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset5 = 0x18150,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorI32_countsnoreset6 = 0x1817C,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorI32;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount = 0x1809C,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount2 = 0x180CC,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount3 = 0x180F4,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount4 = 0x18120,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount5 = 0x1814C,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorU32_tickscount6 = 0x18178,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorU32;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR = 0x180B6,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR2 = 0x180E2,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR3 = 0x1810E,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR4 = 0x1813A,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR5 = 0x18166,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_DIR6 = 0x18192,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_Digital = 0x18026,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN = 0x180BA,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN2 = 0x180E6,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN3 = 0x18112,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN4 = 0x1813E,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN5 = 0x1816A,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_EN6 = 0x18196,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_Power = 0x1801E,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition = 0x18092,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition2 = 0x180C2,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition3 = 0x180EA,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition4 = 0x18116,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition5 = 0x18142,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_ResetPosition6 = 0x1816E,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_Signal = 0x18022,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state = 0x180B2,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state2 = 0x180DE,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state3 = 0x1810A,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state4 = 0x18136,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state5 = 0x18162,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_state6 = 0x1818E,
   NiFpga_FPGA_POWER_RS485_v2_ControlBool_stop = 0x18006,
} NiFpga_FPGA_POWER_RS485_v2_ControlBool;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_Controlmode = 0x1800A,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_L1PositionBus1 = 0x1804E,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_L2PositionBus2 = 0x1806A,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_L3PositionBus3 = 0x18082,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_R1PositionBus1 = 0x18052,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_R2PositionBus2 = 0x18066,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_R3PositionBus3 = 0x1807E,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage = 0x180AA,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage2 = 0x180D6,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage3 = 0x18102,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage4 = 0x1812E,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage5 = 0x1815A,
   NiFpga_FPGA_POWER_RS485_v2_ControlU16_input_voltage6 = 0x18186,
} NiFpga_FPGA_POWER_RS485_v2_ControlU16;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue = 0x18094,
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue2 = 0x180C4,
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue3 = 0x180EC,
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue4 = 0x18118,
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue5 = 0x18144,
   NiFpga_FPGA_POWER_RS485_v2_ControlI32_ResetPositionValue6 = 0x18170,
} NiFpga_FPGA_POWER_RS485_v2_ControlI32;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_CMDTimeus = 0x18088,
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_DCMotor = 0x18010,
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_IdleTimeus = 0x18084,
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_PowerBoard = 0x18000,
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_RSBLServo = 0x1800C,
   NiFpga_FPGA_POWER_RS485_v2_ControlU32_ReadTimeus = 0x1808C,
} NiFpga_FPGA_POWER_RS485_v2_ControlU32;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray = 0x180AE,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray2 = 0x180DA,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray3 = 0x18106,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray4 = 0x18132,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray5 = 0x1815E,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool_outputarray6 = 0x1818A,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBool;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray2 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray3 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray4 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray5 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize_outputarray6 = 16,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataArray = 0x18034,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRX = 0x18038,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus1 = 0x18044,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus2 = 0x1805C,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8_DataRXBus3 = 0x18074,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataArray = 33,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRX = 64,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus1 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus2 = 16,
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size_DataRXBus3 = 16,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU8Size;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16_Data = 0x18018,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size_Data = 16,
} NiFpga_FPGA_POWER_RS485_v2_IndicatorArrayU16Size;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8_DataTx = 0x18028,
} NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8;

typedef enum
{
   NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size_DataTx = 12,
} NiFpga_FPGA_POWER_RS485_v2_ControlArrayU8Size;


#if NiFpga_Cpp
}
#endif

#endif
