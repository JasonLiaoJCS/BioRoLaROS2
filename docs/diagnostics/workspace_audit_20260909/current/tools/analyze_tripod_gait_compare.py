#!/usr/bin/env python3
"""Offline comparison of archived GitHub/local C++ trajectory methods and logs.

Never initializes ROS, connects to hardware, or runs a motion executable.
Compile only the extracted, arithmetic-only trajectory functions.
"""
from pathlib import Path
import csv
import difflib
import hashlib
import io
import json
import math
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'docs/diagnostics/tripod_gait_compare_20260909'


def method(source, name):
    match = re.search(r'\b(?:double|void)\s+'+name+r'\(', source)
    assert match, name
    start = source.index('{', match.start())
    depth = 1
    end = start+1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def assignment(source, name):
    match = re.search(r'\b'+name+r'\s*=\s*[^;]+;', source)
    assert match, name
    return match.group()


def arithmetic_class(source, name, current=False):
    methods = ('eval_poly_deg', 'eval_poly_rad', 'eval_poly_derivative_deg',
               'eval_poly_derivative_rad', 'find_poly_zero', 'compute_trapezoid_params',
               'eval_trapezoid', 'compute_trajectory')
    if current:
        methods += ('compute_group_reference',)
    setup = '\n'.join(assignment(source, v) for v in
                      ('poly_matlab_', 't_stance_', 't_flight_', 'br_'))
    return f'''struct {name} {{
    std::vector<double> poly_matlab_, poly_;
    double t_stance_, t_flight_, period_, br_, t_center_;
    double theta_LO_, theta_dot_LO_, theta_TD_, theta_dot_TD_, w_top_, a1_, a2_;
    double current_ratio_ = 8;
    {name}() {{
      {setup}
      poly_.assign(poly_matlab_.rbegin(), poly_matlab_.rend());
      period_=t_stance_+t_flight_; t_center_=find_poly_zero();
      theta_LO_=eval_poly_rad(t_stance_); theta_dot_LO_=eval_poly_derivative_rad(t_stance_);
      theta_TD_=eval_poly_rad(0)+2*M_PI; theta_dot_TD_=eval_poly_derivative_rad(0);
      compute_trapezoid_params();
    }}
    {' '.join(method(source, v) for v in methods)}
}};'''


def main():
    upstream = (OUT/'upstream-rinbo_tripod.cpp').read_text()
    current = (OUT/'current-rinbo_tripod.cpp').read_text()
    (OUT/'upstream-to-current.diff').write_text(''.join(difflib.unified_diff(
        upstream.splitlines(True), current.splitlines(True),
        fromfile='GitHub-ddcbce9/rinbo_tripod.cpp', tofile='Orin/rinbo_tripod.cpp')))
    # Compilation intentionally has no ROS/controller construction or hardware APIs.
    cpp = '''#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
'''
    cpp += '#include '+json.dumps(str(OUT/'current-tripod_reference.hpp'))+'\n'
    cpp += arithmetic_class(upstream, 'Original')+'\n'+arithmetic_class(current, 'Current', True)
    cpp += '''
int main() {
  Original old; Current now;
  double ratio, phase;
  std::cout << std::setprecision(17);
  while (std::cin >> ratio >> phase) {
    old.current_ratio_=now.current_ratio_=ratio;
    double op, ov, np, nv; bool stance;
    old.compute_trajectory(phase,op,ov,stance);
    now.compute_group_reference(phase,np,nv);
    std::cout << op << ',' << ov << ',' << np << ',' << nv << '\\n';
  }
}
'''
    (OUT/'arithmetic-comparison.cpp').write_text(cpp)
    rows = json.loads((OUT/'trace-rows.json').read_text())
    running = [r for r in rows if r['phase'] == 'RUNNING']
    center = running[-1]['tau']-running[-1]['phase_A']
    period = .175887413151118+.227629442432964
    blend = period/4
    counts = 54984.83/(2*math.pi)
    grid = [(ratio, center+blend+period*i/10000)
            for ratio in (8, 6, 5.9, 5, 4) for i in range(50001)]
    trace_queries = [(r['ratio'], r['tau']-(period/2 if leg['group']=='B' else 0))
                     for r in running for leg in r['legs'].values()
                     if not leg['masked'] and (leg['group']!='B' or r['group_b_active'])]
    # Include both sides of stance/flight and full-cycle joins, plus negative
    # phase as a diagnostic (not an enabled production command).
    joins = [(5.9, value+epsilon) for value in (.175887413151118, period, 10*period)
             for epsilon in (-1e-9, 0, 1e-9)]
    with tempfile.TemporaryDirectory(prefix='tripod-arithmetic-') as tmp:
        exe = Path(tmp)/'compare'
        subprocess.run(['g++', '-std=c++17', '-O2', str(OUT/'arithmetic-comparison.cpp'),
                        '-o', str(exe)], check=True)
        queries = grid+trace_queries+joins+[(5.9, -.01)]
        output = subprocess.check_output([str(exe)], text=True,
                                        input=''.join(f'{r:.17g} {p:.17g}\n' for r,p in queries))
    values = [tuple(map(float, row)) for row in csv.reader(io.StringIO(output))]
    geometry = values[:len(grid)]
    max_p = max(abs(op-np)*counts for op,ov,np,nv in geometry)
    max_v = max(abs(ov-nv)*counts for op,ov,np,nv in geometry)
    assert max_p < 1e-6 and max_v < 1e-6, (max_p,max_v)
    cursor = len(grid)
    target_errors, velocity_errors, group_spread = [], [], []
    for row in running:
        group_targets = {'A':[], 'B':[]}
        for leg in row['legs'].values():
            if leg['masked'] or (leg['group']=='B' and not row['group_b_active']):
                continue
            _, _, position, velocity = values[cursor]; cursor += 1
            target_errors.append(abs(leg['target_counts']-(leg['home_counts']+position*counts)))
            velocity_errors.append(abs(leg['target_v']-velocity*counts))
            group_targets[leg['group']].append(leg['target_counts']-leg['home_counts'])
        for group in group_targets.values():
            if group: group_spread.append(max(group)-min(group))
    # Trace positions are float32 at ~1e6 counts. Differences of 1/16 count
    # are expected rounding, not phase/leg swaps. Text timestamps are rounded.
    assert max(target_errors) < .2, max(target_errors)
    assert max(velocity_errors) < .02, max(velocity_errors)
    assert max(group_spread) < .2, max(group_spread)
    joint_values = values[cursor:cursor+len(joins)]
    joint_metrics = []
    for i in range(0,len(joins),3):
        left, _, right = joint_values[i:i+3]
        joint_metrics.append({'phase':joins[i+1][1],
                              'position_delta_counts':(right[2]-left[2])*counts,
                              'velocity_delta_counts_s':(right[3]-left[3])*counts})
    assert max(abs(m['position_delta_counts']) for m in joint_metrics) < .01
    assert max(abs(m['velocity_delta_counts_s']) for m in joint_metrics) < .01
    def eval_v(t):
        co = [-48.7991629707687, 1013.75966324102, -2180.08626086051,
              2572.43821028774, -124169.690772846, 499226.851105064]
        return sum(i*a*t**(i-1) for i,a in enumerate(co) if i)*math.pi/180
    # Read plateau directly from extracted C++, then differentiate the actual
    # flight ramp analytically. This is nominal controller demand, not a motor model.
    index = min(range(len(grid)), key=lambda i:abs(grid[i][0]-5.9)*100+
                abs((grid[i][1]%period)-(.175887413151118+.227629442432964/2)))
    w_top = geometry[index][1]*grid[index][0]
    a1 = (w_top-eval_v(.175887413151118))/(.3*.227629442432964)
    required = [{'ratio':r,'cycle_seconds':r*period,
                 'feedforward_flight_ramp_pwm_per_s':abs(a1)*counts*.005/r**2}
                for r in (8, 6, 5.9, 5, 4)]
    metrics = {'positive_phase_comparisons':len(grid), 'max_reference_difference_counts':max_p,
               'max_reference_velocity_difference_counts_s':max_v,
               'actual_trace_leg_reference_comparisons':len(target_errors),
               'max_trace_target_reconstruction_error_counts':max(target_errors),
               'max_trace_velocity_reconstruction_error_counts_s':max(velocity_errors),
               'max_same_group_relative_target_spread_counts':max(group_spread),
               'join_continuity':joint_metrics,
               'negative_phase_original_minus_current_counts':(values[-1][0]-values[-1][2])*counts,
               'nominal_feedforward_ramp':required,
               'files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in OUT.glob('current-*')}}
    (OUT/'arithmetic-results.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(json.dumps(metrics,indent=2))


if __name__ == '__main__':
    main()
