from pathlib import Path
import shlex,subprocess
root=Path('/home/jetson/rinbo_ros_ws');out=root/'docs/diagnostics/restore_choices_20260909';build=Path('/tmp/rinbo-tripod-slew-build')
flags=(build/'CMakeFiles/robot_config.dir/flags.make').read_text()
includes=next(line.split('=',1)[1] for line in flags.splitlines() if line.startswith('CXX_INCLUDES ='))
obj=out/'validate_candidate.o'
subprocess.run(['/usr/bin/c++','-std=c++17',*shlex.split(includes),'-c',str(out/'validate_candidate.cpp'),'-o',str(obj)],check=True)
link=shlex.split((build/'CMakeFiles/rinbo_legs.dir/link.txt').read_text())
link=[str(obj) if s=='CMakeFiles/rinbo_legs.dir/src/rinbo_legs.cpp.o' else s for s in link]
link[link.index('-o')+1]=str(out/'validate_candidate')
subprocess.run(link,cwd=build,check=True)
obj.unlink()
