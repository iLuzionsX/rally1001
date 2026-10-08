import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
import {execFileSync} from 'node:child_process';

// Repeatable collision-mesh fixtures, using the shipped vehicle and timestep.
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'rough-road-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/240;
const round=n=>+n.toFixed(4);
try{
  for(const name of ['course','vehicle-config','vehicle','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
    const source=process.env.RALLY_ROUGH_BEFORE&&name==='wheel-model'
      ?execFileSync('git',['show','5bf089441a4b010bf2df931be363fb2bf8b97409:lib/rally/wheel-model.ts'],{cwd:root,encoding:'utf8'})
      :readFileSync(join(root,'lib/rally',name+'.ts'),'utf8');
    const {outputText}=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
    mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});
    writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
  }
  const course=require(join(temp,'course.cjs')),{RallyVehicle}=require(join(temp,'vehicle.cjs'));
  const {advanceVehicle}=require(join(temp,'simulation.cjs'));
  await RAPIER.init();
  course.surfaceAt=()=>({grip:.59,rollingResistance:.025,rollingDrag:4,mud:false,label:'LOOSE DIRT'});
  // A wheel moving tangent to a plane has no damper compression velocity,
  // even if its world-space vertical velocity is nonzero.
  if(!process.env.RALLY_ROUGH_BEFORE)for(const kind of ['suv','truck']){
    const world=new RAPIER.World({x:0,y:-9.81,z:0}),angle=.05;
    world.createCollider(RAPIER.ColliderDesc.cuboid(30,.5,30).setTranslation(0,-.5,0).setRotation({x:Math.sin(angle/2),y:0,z:0,w:Math.cos(angle/2)}));
    const v=new RallyVehicle(world,kind,'rally');
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<480;i++)advanceVehicle(v,world,zero,dt,false);
    v.body.setAngvel({x:0,y:0,z:0},true);
    v.body.setLinvel({x:0,y:10*Math.tan(angle),z:-10},true);
    const w=v.wheels[0];w.contactStep();assert(w.contact,'Plane contact');
    const compression=v.config.suspension-w.suspensionLength;
    const spring=v.config.springRate*compression+Math.max(0,compression-v.config.suspensionTravel)*v.config.springRate*3;
    assert(Math.abs(w.force-spring)<.1,'Tangent travel must not create damping');
    const tangentLoad=w.force;
    v.body.setLinvel({x:0,y:0,z:-10},true);w.contactStep();
    assert(w.force>tangentLoad+500,'Approaching a rising surface must compress the damper');
    world.free();
  }
  const hump=(s,start,length,height)=>s>start&&s<start+length?height*Math.sin(Math.PI*(s-start)/length)**2:0;
  function fixture(world,shape){
    const vertices=[],indices=[],nx=20,nz=1400;
    for(let iz=0;iz<=nz;iz++)for(let ix=0;ix<=nx;ix++){
      const x=ix-10,s=iz*.1-10;
      let y=0;
      if(shape==='bumps'||shape==='corner-bumps')y=hump(s,12,2,.10)+hump(s,17,2,.10)+hump(s,22,2,.10);
      if(shape==='split')y=(x<0?1:0)*(hump(s,12,3,.14)+hump(s,20,3,.14));
      if(shape==='crest')y=hump(s,12,20,.6);
      vertices.push(x,y,-s);
    }
    for(let iz=0;iz<nz;iz++)for(let ix=0;ix<nx;ix++){
      const a=iz*(nx+1)+ix,b=a+1,c=a+nx+1,d=c+1;
      indices.push(a,b,c,b,d,c);
    }
    world.createCollider(RAPIER.ColliderDesc.trimesh(new Float32Array(vertices),new Uint32Array(indices)));
  }
  const results=[];
  for(const kind of ['suv','truck'])for(const shape of ['flat','bumps','corner-bumps','split','crest','landing'])for(const kmh of [40,70]){
    const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
    fixture(world,shape);
    const v=new RallyVehicle(world,kind,'rally');
    v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
    for(let i=0;i<480;i++)advanceVehicle(v,world,zero,dt,false);
    const rideHeight=v.position.y;
    if(shape==='landing')v.body.setTranslation({x:0,y:rideHeight+.6,z:0},true);
    v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
    v.brake=0;
    let peakLoad=0,peakVertical=0,peakRoll=0,peakPitch=0,airtime=0,unloaded=0,settleTime=0,lastObstacleTime=0;
    let minTravel=Infinity,maxTravel=0,tailVertical=0,peakYaw=0,minUp=1;
    const contacts=[0,0,0,0];
    for(let i=0;i<240*7;i++){
      const steer=shape==='corner-bumps'&&-v.position.z>8&&-v.position.z<28?.055:0;
      advanceVehicle(v,world,{...zero,steer},dt,true);
      assert(Number.isFinite(v.speed)&&Number.isFinite(v.position.y),'Finite motion');
      assert(v.wheels.every(w=>Number.isFinite(w.force)&&w.force>=0),'Finite nonnegative wheel loads');
      const up=1-2*(v.rotation.x**2+v.rotation.z**2);
      minUp=Math.min(minUp,up);
      const active=v.wheels.filter(w=>w.contact&&w.force>1).length;
      if(active===0)airtime+=dt;
      unloaded+=(4-active)*dt;
      v.wheels.forEach((w,j)=>{
        if(w.contact&&w.force>1)contacts[j]+=dt;
        peakLoad=Math.max(peakLoad,w.force/(v.config.mass*9.81/4));
        minTravel=Math.min(minTravel,w.suspensionLength);maxTravel=Math.max(maxTravel,w.suspensionLength);
      });
      peakVertical=Math.max(peakVertical,Math.abs(v.velocity.y));
      peakRoll=Math.max(peakRoll,Math.abs(v.body.angvel().z));
      peakPitch=Math.max(peakPitch,Math.abs(v.body.angvel().x));
      peakYaw=Math.max(peakYaw,Math.abs(v.body.angvel().y));
      const past=shape==='landing'?i*dt>.7:-v.position.z>36;
      if(!past)lastObstacleTime=i*dt;
      if(past&&Math.abs(v.velocity.y)<.05&&active===4&&!settleTime)settleTime=i*dt-lastObstacleTime;
      if(i>240*6)tailVertical=Math.max(tailVertical,Math.abs(v.velocity.y));
    }
    results.push({kind,shape,kmh,minUp:round(minUp),peakLoad:round(peakLoad),peakVertical:round(peakVertical),peakRoll:round(peakRoll),peakPitch:round(peakPitch),peakYaw:round(peakYaw),airtime:round(airtime),unloadedWheelSeconds:round(unloaded),minTravel:round(minTravel),maxTravel:round(maxTravel),tailVertical:round(tailVertical),settleTime:round(settleTime),distance:round(-v.position.z),contacts:contacts.map(round)});
    world.free();
  }
  const report={results,limitations:'Scripted collision-mesh tests; no browser/GPU or hands-on DiRT Rally comparison.'};
  const file=process.env.RALLY_ROUGH_BEFORE?'rough-road-before.json':'rough-road-validation.json';
  writeFileSync(join(root,'reference',file),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify(report,null,2));
  // The retained pre-change implementation is diagnostic; it is expected to
  // fail the fast crest. The shipped solver must meet all acceptance bounds.
  if(!process.env.RALLY_ROUGH_BEFORE)for(const r of results){
    assert(r.minUp>.95,`${r.kind} ${r.shape} ${r.kmh}: remain upright`);
    assert(r.distance>40,`${r.kind} ${r.shape} ${r.kmh}: clear the test section`);
    assert(r.tailVertical<.05,`${r.kind} ${r.shape} ${r.kmh}: settle without persistent bouncing`);
    assert(r.minTravel>=0&&r.maxTravel<=(r.kind==='suv'?.42:.46),'Suspension travel stays within bounds');
    if(r.shape==='crest'&&r.kmh===70)assert(r.airtime>.05&&r.airtime<.6,'Fast crest should unload and regain contact promptly');
    if(r.shape==='flat')assert(r.airtime===0,'No false airborne state on level ground');
  }
}finally{rmSync(temp,{recursive:true,force:true});}
