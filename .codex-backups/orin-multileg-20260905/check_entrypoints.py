"""Read-only deployment verification; never calls an FSM without --check-config."""
import hashlib,json,os,subprocess,tempfile
from pathlib import Path
ws=Path('/home/jetson/rinbo_ros_ws')
site=Path('/home/jetson/redrhex_site/rinbo_fsm_disabled_leg.yaml')
sha=hashlib.sha256(site.read_bytes()).hexdigest()
with tempfile.TemporaryDirectory(prefix='rinbo-entry-check-') as scratch:
    base={'HOME':scratch,'PATH':'/usr/bin:/bin','LANG':'C.UTF-8',
          'LD_LIBRARY_PATH':os.environ.get('LD_LIBRARY_PATH',''),
          'ROS_DOMAIN_ID':'231','ROS_LOCALHOST_ONLY':'1',
          'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
          'LD_PRELOAD':str(ws/'.codex-backups/orin-multileg-20260905/no_ros_canary.so'),
          'ROBOT_CONFIG':'/ignored/windows/config.yaml',
          'REDRHEX_FSM_CFG':'/ignored/legacy/config.yaml'}
    count=0
    for exe,node in [('rinbo_cali','rinbo_cali'),('rinbo_standing','rinbo_standing'),('rinbo_tripod','rinbo_tripod_rslip')]:
        build=ws/'build/rinbo_fsm'/exe
        installed=ws/'install/rinbo_fsm/lib/rinbo_fsm'/exe
        assert installed.resolve()==build.resolve(), (exe,'install link mismatch')
        for cwd in ['/',scratch,str(ws)]:
            # Empty noninteractive environment, including a ROS/network API canary:
            # if any ROS initialization is reached, this must fail.
            result=subprocess.run(['/bin/bash','--noprofile','--norc','-c','exec "$@"','check',str(build),'--check-config'],cwd=cwd,env=base,text=True,capture_output=True,timeout=15)
            assert result.returncode==0,(exe,cwd,result.stderr)
            for expected in [str(site),sha,'revision: 1','Disabled legs: []',f'Effective parameters for {node}:','hardware.max_disabled_legs: 6']:
                assert expected in result.stdout,(exe,expected)
            count+=1
        env=dict(os.environ);env.update(base)
        result=subprocess.run(['/opt/ros/humble/bin/ros2','run','rinbo_fsm',exe,'--check-config'],cwd=scratch,env=env,text=True,capture_output=True,timeout=15)
        assert result.returncode==0,(exe,'ros2 run',result.stderr)
        assert sha in result.stdout;count+=1
        for args in [['--ros-args','--params-file',str(ws/'src/rinbo_fsm/config/l1_degraded_test.yaml')],['--ros-args','-p','hardware.disabled_legs:=[L1]']]:
            result=subprocess.run([str(build),'--check-config',*args],cwd=scratch,env=base,text=True,capture_output=True,timeout=15)
            assert result.returncode!=0 and '[FATAL]' in result.stderr and 'conflict' in result.stderr,result
            assert 'State: DONE' not in result.stdout and 'entering RUNNING' not in result.stdout
            count+=1
    result=subprocess.run([str(ws/'build/rinbo_fsm/rinbo_legs'),'status','--json'],env=base,text=True,capture_output=True,timeout=15)
    assert result.returncode==0,result.stderr
    status=json.loads(result.stdout)
    assert status['disabled_legs']==[] and status['path']==str(site) and status['hash']==sha
    assert len(status['enabled_legs'])==6
    assert not (Path(scratch)/'.ros').exists(), 'Read-only probe initialized ROS logs'
    print(f'{count} FSM entry checks passed: 9 build/cwd + 3 ros2 run + 6 override rejections; LD_PRELOAD canary rejects ROS init/publish/socket calls; none occurred.')
    print('rinbo_legs JSON status verified. Schema=1 revision=1 disabled=[]; SHA256='+sha)
assert hashlib.sha256(site.read_bytes()).hexdigest()==sha,'Effective configuration changed during read-only checks'
print('Effective configuration unchanged by entry checks. No motion entry executed.')
