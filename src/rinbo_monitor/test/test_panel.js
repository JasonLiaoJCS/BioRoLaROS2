// Browser-script smoke check without installing a browser on the robot.
// node src/rinbo_monitor/test/test_panel.js
const fs=require('fs'),vm=require('vm'),assert=require('assert');
class Element {
 constructor(){this.children=[];this.textContent='';this.value='l1';this.clientWidth=600;this.clientHeight=300;this.scrollTop=0;this.scrollHeight=0;this.className='';this.classList={add:()=>{}};}
 append(...nodes){this.children.push(...nodes);}
 replaceChildren(...nodes){this.children=nodes;}
 getContext(){return new Proxy({}, {get:()=>()=>{}});}
}
const ids={};const document={getElementById:id=>ids[id]??=(new Element()),createElement:()=>new Element(),querySelectorAll:()=>[]};
const requests=[];
const names=['l1','l2','l3','r1','r2','r3'];
const command={header:{seq:123},servo_control_mode:2};
const motor={};
for(const [i,n] of names.entries()){command[n]={enable:true,direction:i>2,voltage:30,state:1,reset_position:false};command['s'+n]={position_encoder:1000};motor[n]={position:1234+i,hall_effect:i!==1,tick_count:200};motor['s'+n]={position_encoder:999};}
const sample={ros_domain_id:'232',sources:{},reset_counts:{requested:{},forwarded:{}},events:[{t:1,kind:'reset',text:'L2 reset'},{t:2,kind:'error',text:'CALIBRATION SAFETY STOP: position reset timeout: L2'}],history:[{t:1,legs:names.map(()=>[1234,30,1000,999])}]};
for(const k of ['requested','forwarded','motor','power','output','ready'])sample.sources[k]={stale:false,age_s:.01,count:2,source_age_s:null,data:k==='requested'||k==='forwarded'?command:k==='motor'?motor:k==='power'?{v_7:24,i_7:1,power:true}:{data:true}};
for(const n of names){sample.reset_counts.requested[n]=1;sample.reset_counts.forwarded[n]=1;}
const context=vm.createContext({document,window:{devicePixelRatio:1},console,AbortSignal,setTimeout:()=>{},fetch:async(url,options)=>{requests.push({url,options});return {ok:true,json:async()=>url.includes('/recording/')?{state:'recording',samples:0,events:0,duration_s:0}:sample};}});
const page=fs.readFileSync('src/rinbo_monitor/rinbo_monitor/panel.html','utf8');
vm.runInContext(page.split('<script>')[1].split('</script>')[0],context);
setImmediate(async()=>{
 assert.equal(ids.legs.children.length,6);
 assert.equal(ids.events.children.length,2);
 ids.pause.onclick();assert.match(ids.connection.textContent,/已暫停/);
 ids.pause.onclick();
 sample.sources.motor.stale=true;
 context.fixture=sample;vm.runInContext('render(fixture)',context);
 assert.match(ids.connection.textContent,/過期/);
 for(const s of Object.values(sample.sources)){s.stale=true;s.data=null;}
 vm.runInContext('render(fixture)',context);assert.equal(ids.legs.children.length,6);
 vm.runInContext("renderRecording({state:'recording',duration_s:2,samples:20,events:1})",context);
 assert.equal(ids['record-start'].disabled,true);assert.equal(ids['record-stop'].disabled,false);
 vm.runInContext("renderRecording({state:'stopped',duration_s:2,samples:20,events:1,download_ready:true})",context);
 assert.equal(ids['record-download'].hidden,false);assert.equal(ids['record-start'].disabled,false);
 await ids['record-start'].onclick();
 assert.equal(requests.at(-1).url,'/api/recording/start');assert.equal(requests.at(-1).options.method,'POST');
 const text=[];ids.mainplot.getContext=()=>new Proxy({}, {get:(_,name)=>name==='fillText'?(value)=>text.push(value):()=>{}});
 context.rawHistory=[{t:1,legs:names.map(()=>[-55297,80,1000,999])},{t:1.1,legs:names.map(()=>[55297,80,1000,999])}];
 context.resetEvents=[{t:1.05,kind:'reset',text:'requested L1 reset=true seq=1'}];
 vm.runInContext("draw($('mainplot'),rawHistory,0,0,null,false,resetEvents)",context);
 assert(text.includes('reset'));assert(text.some(t=>Number(t)>55000));
 console.log('Panel script: rendering, stale data, recording controls, independent raw-count plot and reset markers passed.');
});
