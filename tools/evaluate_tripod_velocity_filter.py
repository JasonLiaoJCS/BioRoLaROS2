#!/usr/bin/env python3
"""Offline nominal-trajectory/encoder delivery comparison. No ROS or hardware.

This measures estimator error, not physical closed-loop stability. A 5 ms
packet-hold case is a synthetic delivery scenario, not a measured device spec.
"""
from pathlib import Path
import importlib.util
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/diagnostics/tripod_restore_20260909'


def main():
    spec = importlib.util.spec_from_file_location('compare', ROOT/'tools/analyze_tripod_gait_compare.py')
    compare = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compare)
    source = (ROOT/'src/rinbo_fsm/src/rinbo_tripod.cpp').read_text()
    cpp = '''#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
'''
    for name in ('tripod_reference.hpp', 'control_velocity.hpp'):
        cpp += '#include ' + json.dumps(str(ROOT/'src/rinbo_fsm/src'/name)) + '\n'
    cpp += compare.arithmetic_class(source, 'Trajectory', True)
    cpp += r'''
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
'''
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/'filter-comparison.cpp').write_text(cpp)
    with tempfile.TemporaryDirectory(prefix='tripod-filter-offline-') as tmp:
        binary = str(Path(tmp)/'compare')
        subprocess.run(['g++','-std=c++17','-O2',str(OUT/'filter-comparison.cpp'),'-o',binary], check=True)
        result = subprocess.run([binary],check=True,text=True,capture_output=True).stdout
    (OUT/'filter-comparison.csv').write_text(result)
    print(result, end='')


if __name__ == '__main__':
    main()
