#pragma once
#include <cstdint>
struct Stamp { void set_sec(long) {} void set_usec(long) {} };
struct Header { Stamp stamp; void set_seq(unsigned) {} Stamp* mutable_stamp(){return &stamp;} };
namespace power_msg {
struct PowerCmdStamped {
 Header header_; Header* mutable_header(){return &header_;}
 double digital_ = 0; double digital() const {return digital_;} void set_digital(double v){digital_ = v;}
 double signal_ = 0; double signal() const {return signal_;} void set_signal(double v){signal_ = v;}
 double power_ = 0; double power() const {return power_;} void set_power(double v){power_ = v;}
 double v_0_ = 0; double v_0() const {return v_0_;} void set_v_0(double v){v_0_ = v;}
 double v_1_ = 0; double v_1() const {return v_1_;} void set_v_1(double v){v_1_ = v;}
 double v_2_ = 0; double v_2() const {return v_2_;} void set_v_2(double v){v_2_ = v;}
 double v_3_ = 0; double v_3() const {return v_3_;} void set_v_3(double v){v_3_ = v;}
 double v_4_ = 0; double v_4() const {return v_4_;} void set_v_4(double v){v_4_ = v;}
 double v_5_ = 0; double v_5() const {return v_5_;} void set_v_5(double v){v_5_ = v;}
 double v_6_ = 0; double v_6() const {return v_6_;} void set_v_6(double v){v_6_ = v;}
 double v_7_ = 0; double v_7() const {return v_7_;} void set_v_7(double v){v_7_ = v;}
 double i_0_ = 0; double i_0() const {return i_0_;} void set_i_0(double v){i_0_ = v;}
 double i_1_ = 0; double i_1() const {return i_1_;} void set_i_1(double v){i_1_ = v;}
 double i_2_ = 0; double i_2() const {return i_2_;} void set_i_2(double v){i_2_ = v;}
 double i_3_ = 0; double i_3() const {return i_3_;} void set_i_3(double v){i_3_ = v;}
 double i_4_ = 0; double i_4() const {return i_4_;} void set_i_4(double v){i_4_ = v;}
 double i_5_ = 0; double i_5() const {return i_5_;} void set_i_5(double v){i_5_ = v;}
 double i_6_ = 0; double i_6() const {return i_6_;} void set_i_6(double v){i_6_ = v;}
 double i_7_ = 0; double i_7() const {return i_7_;} void set_i_7(double v){i_7_ = v;}
};
struct PowerStateStamped {
 Header header_; Header* mutable_header(){return &header_;}
 double digital_ = 0; double digital() const {return digital_;} void set_digital(double v){digital_ = v;}
 double signal_ = 0; double signal() const {return signal_;} void set_signal(double v){signal_ = v;}
 double power_ = 0; double power() const {return power_;} void set_power(double v){power_ = v;}
 double v_0_ = 0; double v_0() const {return v_0_;} void set_v_0(double v){v_0_ = v;}
 double v_1_ = 0; double v_1() const {return v_1_;} void set_v_1(double v){v_1_ = v;}
 double v_2_ = 0; double v_2() const {return v_2_;} void set_v_2(double v){v_2_ = v;}
 double v_3_ = 0; double v_3() const {return v_3_;} void set_v_3(double v){v_3_ = v;}
 double v_4_ = 0; double v_4() const {return v_4_;} void set_v_4(double v){v_4_ = v;}
 double v_5_ = 0; double v_5() const {return v_5_;} void set_v_5(double v){v_5_ = v;}
 double v_6_ = 0; double v_6() const {return v_6_;} void set_v_6(double v){v_6_ = v;}
 double v_7_ = 0; double v_7() const {return v_7_;} void set_v_7(double v){v_7_ = v;}
 double i_0_ = 0; double i_0() const {return i_0_;} void set_i_0(double v){i_0_ = v;}
 double i_1_ = 0; double i_1() const {return i_1_;} void set_i_1(double v){i_1_ = v;}
 double i_2_ = 0; double i_2() const {return i_2_;} void set_i_2(double v){i_2_ = v;}
 double i_3_ = 0; double i_3() const {return i_3_;} void set_i_3(double v){i_3_ = v;}
 double i_4_ = 0; double i_4() const {return i_4_;} void set_i_4(double v){i_4_ = v;}
 double i_5_ = 0; double i_5() const {return i_5_;} void set_i_5(double v){i_5_ = v;}
 double i_6_ = 0; double i_6() const {return i_6_;} void set_i_6(double v){i_6_ = v;}
 double i_7_ = 0; double i_7() const {return i_7_;} void set_i_7(double v){i_7_ = v;}
};
}
namespace motor_msg {
struct LegCmd {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
};
struct LegState {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
};
struct ServoCmd {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
};
struct ServoState {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
};
struct MotorCmdStamped {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
 LegCmd l1_; const LegCmd& l1() const {return l1_;} LegCmd* mutable_l1() {return &l1_;}
 LegCmd l2_; const LegCmd& l2() const {return l2_;} LegCmd* mutable_l2() {return &l2_;}
 LegCmd l3_; const LegCmd& l3() const {return l3_;} LegCmd* mutable_l3() {return &l3_;}
 LegCmd r1_; const LegCmd& r1() const {return r1_;} LegCmd* mutable_r1() {return &r1_;}
 LegCmd r2_; const LegCmd& r2() const {return r2_;} LegCmd* mutable_r2() {return &r2_;}
 LegCmd r3_; const LegCmd& r3() const {return r3_;} LegCmd* mutable_r3() {return &r3_;}
 ServoCmd sl1_; const ServoCmd& sl1() const {return sl1_;} ServoCmd* mutable_sl1() {return &sl1_;}
 ServoCmd sl2_; const ServoCmd& sl2() const {return sl2_;} ServoCmd* mutable_sl2() {return &sl2_;}
 ServoCmd sl3_; const ServoCmd& sl3() const {return sl3_;} ServoCmd* mutable_sl3() {return &sl3_;}
 ServoCmd sr1_; const ServoCmd& sr1() const {return sr1_;} ServoCmd* mutable_sr1() {return &sr1_;}
 ServoCmd sr2_; const ServoCmd& sr2() const {return sr2_;} ServoCmd* mutable_sr2() {return &sr2_;}
 ServoCmd sr3_; const ServoCmd& sr3() const {return sr3_;} ServoCmd* mutable_sr3() {return &sr3_;}
};
struct MotorStateStamped {
 Header header_; Header* mutable_header(){return &header_;}
 double enable_ = 0; double enable() const {return enable_;} void set_enable(double v){enable_ = v;}
 double direction_ = 0; double direction() const {return direction_;} void set_direction(double v){direction_ = v;}
 double voltage_ = 0; double voltage() const {return voltage_;} void set_voltage(double v){voltage_ = v;}
 double reset_position_ = 0; double reset_position() const {return reset_position_;} void set_reset_position(double v){reset_position_ = v;}
 double position_ = 0; double position() const {return position_;} void set_position(double v){position_ = v;}
 double tick_count_ = 0; double tick_count() const {return tick_count_;} void set_tick_count(double v){tick_count_ = v;}
 double hall_effect_ = 0; double hall_effect() const {return hall_effect_;} void set_hall_effect(double v){hall_effect_ = v;}
 double position_encoder_ = 0; double position_encoder() const {return position_encoder_;} void set_position_encoder(double v){position_encoder_ = v;}
 double servo_control_mode_ = 0; double servo_control_mode() const {return servo_control_mode_;} void set_servo_control_mode(double v){servo_control_mode_ = v;}
 LegState l1_; const LegState& l1() const {return l1_;} LegState* mutable_l1() {return &l1_;}
 LegState l2_; const LegState& l2() const {return l2_;} LegState* mutable_l2() {return &l2_;}
 LegState l3_; const LegState& l3() const {return l3_;} LegState* mutable_l3() {return &l3_;}
 LegState r1_; const LegState& r1() const {return r1_;} LegState* mutable_r1() {return &r1_;}
 LegState r2_; const LegState& r2() const {return r2_;} LegState* mutable_r2() {return &r2_;}
 LegState r3_; const LegState& r3() const {return r3_;} LegState* mutable_r3() {return &r3_;}
 ServoState sl1_; const ServoState& sl1() const {return sl1_;} ServoState* mutable_sl1() {return &sl1_;}
 ServoState sl2_; const ServoState& sl2() const {return sl2_;} ServoState* mutable_sl2() {return &sl2_;}
 ServoState sl3_; const ServoState& sl3() const {return sl3_;} ServoState* mutable_sl3() {return &sl3_;}
 ServoState sr1_; const ServoState& sr1() const {return sr1_;} ServoState* mutable_sr1() {return &sr1_;}
 ServoState sr2_; const ServoState& sr2() const {return sr2_;} ServoState* mutable_sr2() {return &sr2_;}
 ServoState sr3_; const ServoState& sr3() const {return sr3_;} ServoState* mutable_sr3() {return &sr3_;}
};
}
