// Headless conformance run of all five scenarios: node test/scenarios.test.js
const {DOMParser}=require('@xmldom/xmldom'); const fs=require('fs'); const vm=require('vm');
const ctx={RIL:{},Math,console,Object,Array,Number,Error,JSON,String,Date,Promise,setTimeout,setInterval,clearInterval,performance}; vm.createContext(ctx);
for(const f of ['contract','kinematics','sim_ros2','adapters','manifests','platform']) vm.runInContext(fs.readFileSync(require('path').join(__dirname,'..','src',`${f}.js`),'utf8'),ctx,{filename:f});
const R=ctx.RIL, D=JSON.parse(fs.readFileSync(require('path').join(__dirname,'..','assets','robots.json')));
const trace=[];
function boot(){
  const world=new R.Platform.World(R.Site.world);
  const ril=new R.Platform.RIL({world,parseXML:s=>new DOMParser().parseFromString(s,'text/xml')});
  const graphs={};
  for(const id of ['crx10ia','ur10e']){
    const m=R.Manifests[id]; const kin=new R.Kin.KinModel(new DOMParser().parseFromString(D[id].urdf,'text/xml'),m.description.root_link);
    const names=kin.chain(m.tool_frame.link).filter(j=>j.type!=='fixed').map(j=>j.name);
    graphs[id]=new R.SimRos2.SimRobotGraph({ns:'/'+id,vendor:id==='ur10e'?'ur':'fanuc',jointNames:names,q0:m.home,onTrace:(d,l)=>trace.push(`[${id}] ${d} ${l}`)});
    ril.register({manifest:m,urdf:D[id].urdf,site:R.Site.robots[id],adapter:new R.Adapters.byName[m.adapter]({manifest:m,graph:graphs[id]})});
  }
  ril.audit.on(e=>console.log(`  ${e.level.padEnd(5)} ${e.type.padEnd(22)} ${e.msg}`));
  return {ril,world,graphs};
}
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
  let {ril,world,graphs}=boot(); await sleep(50);
  console.log('== S1 pick&place on crx10ia'); let t0=Date.now();
  let t=ril.submit({skill:'pick_place',object:'part_1',from:'station_a',to:'station_b'},{robot:'crx10ia'}); await t.done;
  console.log('  ->',t.state,(Date.now()-t0)/1000+'s', 'obj at',world.objects.part_1.pos);
  console.log('== S2 same task on ur10e (reset)'); world.reset();
  t=ril.submit({skill:'pick_place',object:'part_1',from:'station_a',to:'station_b'},{robot:'ur10e'}); await t.done; console.log('  ->',t.state);
  console.log('== S3 failover: link loss on crx during approach'); world.reset();
  t=ril.submit({skill:'pick_place',object:'part_1',from:'station_a',to:'station_b'},{robot:'crx10ia'});
  await sleep(700); graphs.crx10ia.injectLinkLoss(); t0=Date.now(); await t.done; console.log('  ->',t.state,'history',t.history.map(h=>h.robot),'total',(Date.now()-t0)/1000);
  graphs.crx10ia.restoreLink(); await sleep(700);
  console.log('== S4 protective stop on ur10e mid-transfer -> auto recover'); world.reset();
  t=ril.submit({skill:'pick_place',object:'part_1',from:'station_a',to:'station_b'},{robot:'ur10e'});
  while(t.current!=='transfer') await sleep(20); await sleep(300); graphs.ur10e.injectProtectiveStop(); await t.done; console.log('  ->',t.state);
  console.log('== S5 link loss on ur10e while holding -> blocked'); world.reset();
  t=ril.submit({skill:'pick_place',object:'part_1',from:'station_a',to:'station_b'},{robot:'ur10e'});
  while(t.current!=='transfer') await sleep(20); graphs.ur10e.injectLinkLoss();
  while(t.state!=='BLOCKED_ON_OPERATOR') await sleep(50); graphs.ur10e.restoreLink(); await sleep(1200); ril.operatorDecision(t.id,'resume'); await t.done; console.log('  ->',t.state);
  process.exit(0);
})();
