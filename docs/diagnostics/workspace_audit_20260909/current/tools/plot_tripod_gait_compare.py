#!/usr/bin/env python3
"""Plot archived pre-change Tripod telemetry; no hardware access."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parents[1]
out=root/'docs/diagnostics/tripod_gait_compare_20260909'
rows=[r for r in json.loads((out/'trace-rows.json').read_text()) if r['phase']=='RUNNING']
ratio=np.array([r['ratio'] for r in rows])
fig,axes=plt.subplots(3,1,figsize=(11,9),layout='constrained')
for leg in ('L2','R1','R2'):
    axes[0].plot(ratio,[r['legs'][leg]['signed_error'] for r in rows],label=leg,lw=1.3)
axes[0].set(ylabel='Target - actual (counts)',title='Tracking diverges as the requested gait speeds up',xlim=(8,5.65))
axes[0].axhline(0,color='grey',lw=.6)
axes[0].legend(loc='upper right',ncol=3)
raw=np.array([r['legs']['R1']['pwm_raw'] for r in rows])
sent=np.array([r['legs']['R1']['pwm_output'] for r in rows])
axes[1].plot(ratio,raw,label='R1 controller request',color='tab:red',lw=1.4)
axes[1].plot(ratio,sent,label='R1 sent after 250 PWM/s limit',color='tab:blue',lw=1.4)
axes[1].fill_between(ratio,raw,sent,where=raw*sent<0,color='tab:red',alpha=.15,label='Opposite signs')
axes[1].set(ylabel='Signed PWM command',xlim=(6.4,5.65),title='The rate limiter delays correction, including reversal')
axes[1].axhline(0,color='grey',lw=.6)
axes[1].legend(loc='lower left',fontsize=9)
axes[2].plot(ratio,[r['legs']['R1']['target_v'] for r in rows],label='R1 target velocity',lw=1.4)
axes[2].plot(ratio,[r['legs']['R1']['filtered_v'] for r in rows],label='R1 encoder velocity (20 ms filter)',lw=1.2)
axes[2].set(ylabel='Velocity (counts/s)',xlim=(6.4,5.65),xlabel='T-ratio decreases to the right (faster gait)',title='Actual tracking does not follow the smooth target velocity')
axes[2].legend(loc='upper left',fontsize=9)
for ax in axes:
    ax.axvline(5.9,color='black',ls='--',lw=.8)
    ax.grid(alpha=.2)
    ax.ticklabel_format(axis='y',style='plain',useOffset=False)
fig.suptitle('Recorded BEFORE change: Tripod 2026-09-09, PWM cap 3300, slew 250/s',fontsize=13)
fig.savefig(out/'tripod-before-change.png',dpi=170)
fig.savefig(out/'tripod-before-change.pdf')
print(out/'tripod-before-change.png')
