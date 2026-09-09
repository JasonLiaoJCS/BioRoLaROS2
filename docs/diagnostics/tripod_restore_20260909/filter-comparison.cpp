#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include "/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/tripod_reference.hpp"
#include "/home/jetson/rinbo_ros_ws/src/rinbo_fsm/src/control_velocity.hpp"
struct Trajectory {
    std::vector<double> poly_matlab_, poly_;
    double t_stance_, t_flight_, period_, br_, t_center_;
    double theta_LO_, theta_dot_LO_, theta_TD_, theta_dot_TD_, w_top_, a1_, a2_;
    double current_ratio_ = 8;
    Trajectory() {
      poly_matlab_ = {499226.851105064, -124169.690772846, 2572.43821028774, 
                        -2180.08626086051, 1013.75966324102, -48.7991629707687};
t_stance_ = 0.175887413151118;
t_flight_ = 0.227629442432964;
br_ = 0.3;
      poly_.assign(poly_matlab_.rbegin(), poly_matlab_.rend());
      period_=t_stance_+t_flight_; t_center_=find_poly_zero();
      theta_LO_=eval_poly_rad(t_stance_); theta_dot_LO_=eval_poly_derivative_rad(t_stance_);
      theta_TD_=eval_poly_rad(0)+2*M_PI; theta_dot_TD_=eval_poly_derivative_rad(0);
      compute_trapezoid_params();
    }
    double eval_poly_deg(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 0; i < poly_.size(); i++) {
            result += poly_[i] * t_pow;
            t_pow *= t;
        }
        return result;
    } double eval_poly_rad(double t) {
        return eval_poly_deg(t) * M_PI / 180.0;
    } double eval_poly_derivative_deg(double t) {
        double result = 0.0;
        double t_pow = 1.0;
        for (size_t i = 1; i < poly_.size(); i++) {
            result += i * poly_[i] * t_pow;
            t_pow *= t;
        }
        return result;
    } double eval_poly_derivative_rad(double t) {
        return eval_poly_derivative_deg(t) * M_PI / 180.0;
    } double find_poly_zero() {
        double t = t_stance_ / 2.0;  // 初始猜測：Stance 中間
        
        for (int i = 0; i < 20; i++) {
            double f = eval_poly_deg(t);
            double df = eval_poly_derivative_deg(t);
            
            if (std::fabs(df) < 1e-10) break;
            
            double t_new = t - f / df;
            
            // 確保在 [0, t_stance] 範圍內
            t_new = std::clamp(t_new, 0.0, t_stance_);
            
            if (std::fabs(t_new - t) < 1e-10) break;
            t = t_new;
        }
        
        return t;
    } void compute_trapezoid_params() {
        double area = theta_TD_ - theta_LO_; 
        
        w_top_ = (area - (theta_dot_LO_ + theta_dot_TD_) * br_ * t_flight_ / 2.0) 
                 / ((1.0 - br_) * t_flight_);
        a1_ = (w_top_ - theta_dot_LO_) / (br_ * t_flight_);
        a2_ = (theta_dot_TD_ - w_top_) / (br_ * t_flight_);
    } void eval_trapezoid(double t, double& theta, double& theta_dot) {
        double t_f = t_flight_;
        double t_br = br_ * t_f;
        double t_1_br = (1.0 - br_) * t_f;
        
        if (t <= t_br) {
            theta = theta_LO_ + theta_dot_LO_ * t + 0.5 * a1_ * t * t;
            theta_dot = theta_dot_LO_ + a1_ * t;
        } else if (t <= t_1_br) {
            double theta_at_br = theta_LO_ + theta_dot_LO_ * t_br + 0.5 * a1_ * t_br * t_br;
            theta = theta_at_br + w_top_ * (t - t_br);
            theta_dot = w_top_;
        } else {
            double theta_at_br = theta_LO_ + theta_dot_LO_ * t_br + 0.5 * a1_ * t_br * t_br;
            double theta_at_1_br = theta_at_br + w_top_ * (t_1_br - t_br);
            double dt = t - t_1_br;
            theta = theta_at_1_br + w_top_ * dt + 0.5 * a2_ * dt * dt;
            theta_dot = w_top_ + a2_ * dt;
        }
    } void compute_trajectory(double tau_local, double& target_rad, double& target_vel_rad, bool& stance_phase) {
        int n = static_cast<int>(std::floor(tau_local / period_));
        double t_local = std::fmod(tau_local, period_);
        
        if (t_local < 0) {
            t_local += period_;
            // floor() already accounts for the previous cycle.
        }
        
        if (t_local < t_stance_) {
            stance_phase = true;
            target_rad = eval_poly_rad(t_local);
            target_vel_rad = eval_poly_derivative_rad(t_local);
        } else {
            stance_phase = false;
            double t_in_flight = t_local - t_stance_;
            eval_trapezoid(t_in_flight, target_rad, target_vel_rad);
        }
        
        target_rad += n * 2.0 * M_PI;
        target_vel_rad /= current_ratio_;
    } void compute_group_reference(double phase, double& position, double& velocity) {
        const auto launch = rinbo_fsm::tripod::launch_phase(
            phase - t_center_, period_ / 4.0);
        bool stance;
        compute_trajectory(t_center_ + launch.position, position, velocity, stance);
        // Anchor to the numerical polynomial root, not a measured/rebased zero.
        position -= eval_poly_rad(t_center_);
        velocity *= launch.velocity;
    }
};
int main() {
  std::cout << "ratio,packet_ms,filter_ms,velocity_rms_error,kd_pwm_rms_error,peak_velocity_error\n";
  for (double ratio: {8.,5.9,1.}) for (int packet: {1,5}) for (double filter: {0.,.005,.02}) {
    Trajectory gait; gait.current_ratio_=ratio;
    rinbo_fsm::ControlVelocity estimator;
    double previous=0, held=0, squares=0, peak=0;
    int count=0;
    for(int n=0;n<=10000;++n) {
      double p,v;
      gait.compute_group_reference(gait.t_center_+gait.period_+n*.001/ratio,p,v);
      p*=rinbo_fsm::tripod::kCountsPerRevolution/(2*M_PI);
      v*=rinbo_fsm::tripod::kCountsPerRevolution/(2*M_PI);
      if(n%packet==0) held=std::round(p);
      if(n>0) {
        const double estimate=estimator.update((held-previous)/.001,.001,filter);
        if(n>1000) {const double e=estimate-v;squares+=e*e;peak=std::max(peak,std::abs(e));++count;}
      }
      previous=held;
    }
    const double rms=std::sqrt(squares/count);
    std::cout << std::setprecision(12) << ratio << ',' << packet << ',' << filter*1000
              << ',' << rms << ',' << rms*.003 << ',' << peak << '\n';
  }
}
