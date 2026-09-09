import json, os, shlex, shutil, subprocess
stage = os.path.dirname(os.path.abspath(__file__))
root = '/home/admin/rinbo_sbRIO_ws/rinbo_fpga_driver'
if not os.path.isdir(stage+'/include'):
    shutil.copytree(root+'/include', stage+'/include')
shutil.copy2(stage+'/console.hpp', stage+'/include/console.hpp')
with open(root+'/build/compile_commands.json') as handle:
    database=json.load(handle)
objects={}
for name in ['console.cpp','fpga_server.cpp']:
    entry=next(item for item in database if os.path.basename(item['file'])==name)
    argv=shlex.split(entry['command'])
    argv=[('-I'+stage+'/include') if arg=='-I'+root+'/include' else arg for arg in argv]
    argv[argv.index('-c')+1]=stage+'/'+name
    argv[argv.index('-o')+1]=stage+'/'+name+'.o'
    argv=[arg for arg in argv if arg!='-w']
    print('Compiling '+name,flush=True)
    subprocess.check_call(argv,cwd=entry['directory'])
    objects['CMakeFiles/fpga_driver.dir/'+name+'.o']=stage+'/'+name+'.o'
with open(root+'/build/src/CMakeFiles/fpga_driver.dir/link.txt') as handle:
    link=shlex.split(handle.read())
link=[objects.get(arg,arg) for arg in link]
link[link.index('-o')+1]=stage+'/fpga_driver.candidate'
print('Linking candidate (active driver untouched)',flush=True)
subprocess.check_call(link,cwd=root+'/build/src')
test=stage+'/test'
shutil.copy2(stage+'/console.hpp',test+'/console.hpp')
with open(stage+'/console.cpp') as handle:
    source=handle.read().replace('/tmp/fpga_driver_console.log',test+'/console.log')
with open(test+'/console.cpp','w') as handle:
    handle.write(source)
subprocess.check_call(['g++','-std=c++14','-Wall','-Wextra','-pthread','-I'+test,test+'/console.cpp',test+'/harness.cpp','-lncurses','-o',test+'/harness'])
print('BUILD_OK',flush=True)