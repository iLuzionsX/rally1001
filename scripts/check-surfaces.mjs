import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,mkdirSync,mkdtempSync,rmSync} from 'node:fs';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
const root=dirname(dirname(fileURLToPath(import.meta.url))),cache=join(root,'node_modules/.cache');
mkdirSync(cache,{recursive:true});const temp=mkdtempSync(join(cache,'surfaces-'));
const require=createRequire(import.meta.url),RAPIER=require('@dimforge/rapier3d-compat');
const zero={steer:0,throttle:0,brake:0,handbrake:0},dt=1/240,round=n=>+n.toFixed(4);
try{
 for(const name of ['course','vehicle-config','vehicle','simulation','drive-model','wheel-model','vendor/ecctrl/CurveLUT','vendor/stunt-rally/gravel','vendor/stunt-rally/pacejka','vendor/stunt-rally/abs','vendor/stunt-rally/engine-friction','vendor/stunt-rally/differential']){
  const {outputText}=ts.transpileModule(readFileSync(join(root,'lib/rally',name+'.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022,esModuleInterop:true}});
  mkdirSync(dirname(join(temp,name+'.cjs')),{recursive:true});writeFileSync(join(temp,name+'.cjs'),outputText.replace(/require\("\.\/(.*?)"\)/g,'require("./$1.cjs")'));
 }
 const course=require(join(temp,'course.cjs')),{RallyVehicle}=require(join(temp,'vehicle.cjs')),{advanceVehicle}=require(join(temp,'simulation.cjs'));
 const {gravelForce}=require(join(temp,'vendor/stunt-rally/pacejka.cjs'));
 const surfaces=course.SURFACE_TIRES,productionSurface=course.surfaceAt;
 await RAPIER.init();
 const beta=v=>Math.atan2(-v.velocity.x*v.forward.z+v.velocity.z*v.forward.x,v.forwardSpeed);
 const sample=name=>({...surfaces[name],mud:name==='mud',tire:surfaces[name]});
 function rig(kind,sampler,kmh=60,mode='rally'){
  course.surfaceAt=(x,z)=>sample(sampler(x,z));
  const world=new RAPIER.World({x:0,y:-9.81,z:0});world.integrationParameters.numSolverIterations=8;
  world.createCollider(RAPIER.ColliderDesc.cuboid(2000,.5,2000).setTranslation(0,-.5,0));
  const v=new RallyVehicle(world,kind,mode);
  v.reset({x:0,y:0,z:0,tx:0,tz:-1,width:12,s:0,distance:0});
  for(let i=0;i<480;i++)advanceVehicle(v,world,zero,dt,false);
  v.brake=0;v.body.setLinvel({x:0,y:0,z:-kmh/3.6},true);v.wheels.forEach(w=>w.angularSpeed=kmh/3.6/v.config.radius);
  const c=v.config.powertrain,rpm=kmh/3.6/v.config.radius*30/Math.PI;
  let gear=0;for(let i=0;i<c.gearRatios.length;i++)if(rpm*c.gearRatios[i]*c.finalDriveRatio>c.shiftDownRPM+1200)gear=i;
  v.driveModel.gearIndex=gear;v.driveModel.driveRatio=c.gearRatios[gear]*c.finalDriveRatio;
  function step(input=zero,frame=dt){
   advanceVehicle(v,world,input,frame,true);
   assert(Number.isFinite(v.speed)&&v.wheels.every(w=>Number.isFinite(w.longForce)&&Number.isFinite(w.sideForce)&&Number.isFinite(w.angularSpeed)),'Finite state');
   assert(1-2*(v.rotation.x**2+v.rotation.z**2)>.9,'Remain upright');
   assert(Math.abs(v.wheels.reduce((s,w)=>s+w.driveTorque,0)-v.engineWheelTorque)<.2,'Torque conservation');
  }
  return {v,world,step};
 }
 // Continuous physical lane/material sampling, including seam and mud edges.
 const mapping=[];
 for(const s of [0,.1,.345,.39,.64,.682,.9999]){
  const p=course.courseAt(s);let previous=null,maxJump=0;
  for(let l=0;l<9;l+=.02){const tire=productionSurface(p.x-p.tz*l,p.z+p.tx*l).tire;if(previous)maxJump=Math.max(maxJump,Math.abs(tire.grip-previous.grip));previous=tire;}
  assert(maxJump<.025,'No sample-index grip step across the shoulder');mapping.push({s,maxGripStep:round(maxJump)});
 }
 for(const boundary of [.345,.39,.64,.682]){
  let previous=null;
  for(let distance=-4;distance<=4;distance+=.02){const p=course.courseAt(boundary+distance/course.COURSE_LENGTH),tire=productionSurface(p.x,p.z).tire;if(previous)assert(Math.abs(tire.grip-previous.grip)<.015,'Continuous mud entry/exit');previous=tire;}
 }
 // Force envelopes: no force without load, no tire force driving slip away from
 // equilibrium, and material-specific slip curves while preserving pure limits.
 const curves=[];
 for(const [surface,tire] of Object.entries(surfaces)){
  let peakX=0,peakY=0,peakSlip=0;
  for(const load of [0,1500,4000,9000])for(const slip of [-1,-.3,-.1,0,.1,.3,1])for(const angle of [-.4,-.1,0,.1,.4]){
   const f=gravelForce(load,tire.grip,20,-Math.tan(angle)*20,20*(1+slip),1,angle,tire);
   assert(Number.isFinite(f.long)&&Number.isFinite(f.side));
   assert(f.long*slip>=-1e-8&&f.side*angle>=-1e-8,'Steady tire force must oppose relative slip');
   if(load===0)assert(f.long===0&&f.side===0,'No airborne tire force');
  }
  for(let slip=0;slip<=1;slip+=.002){const f=gravelForce(4000,tire.grip,20,0,20*(1+slip),1,0,tire);if(f.long>peakX){peakX=f.long;peakSlip=slip;}}
  for(let angle=0;angle<=.6;angle+=.002)peakY=Math.max(peakY,gravelForce(4000,tire.grip,20,-Math.tan(angle)*20,20,1,angle,tire).side);
  curves.push({surface,peakX:round(peakX),peakY:round(peakY),peakSlip:round(peakSlip)});
 }
 assert(curves[0].peakX>curves[1].peakX&&curves[1].peakX>curves[2].peakX,'Packed/gravel/mud traction order');
 assert(curves[0].peakSlip<curves[1].peakSlip&&curves[1].peakSlip<curves[2].peakSlip,'Materials have distinct optimum slip, not just different friction');
 const braking=[],slides=[],transitions=[],cadence=[];
 for(const kind of ['suv','truck']){
  for(const surface of ['packed','gravel','mud','shoulder']){
   const {v,world,step}=rig(kind,()=>surface,80);let stopped=0,peakYaw=0;
   for(let i=0;i<240*7;i++){step({...zero,brake:1});peakYaw=Math.max(peakYaw,Math.abs(v.body.angvel().y));if(v.speed<.15){stopped=(i+1)*dt;break;}}
   assert(stopped>0&&Math.abs(v.position.x)<.1,'Straight stop on every material');
   braking.push({kind,surface,metres:round(-v.position.z),seconds:round(stopped),peakYaw:round(peakYaw)});world.free();
   for(const maneuver of ['coast','trail','power']){
    const {v,world,step}=rig(kind,()=>surface,55);let peakSlip=0,heading=0,frontShare=0,entrySlip=0,spin=0;
    for(let i=0;i<480;i++){
     const t=i*dt,throttle=maneuver==='power'?.65:t<.4?.3:maneuver==='trail'&&t>1.2?.45:0;
     const brake=maneuver==='trail'&&t>=.4&&t<1.1?.25*(1-(t-.4)/.7):0;
     step({...zero,steer:.22,throttle,brake});
     peakSlip=Math.max(peakSlip,Math.abs(beta(v))*180/Math.PI);heading+=v.body.angvel().y*dt*180/Math.PI;
     if(i===215)frontShare=(v.wheels[0].force+v.wheels[1].force)/v.wheels.reduce((sum,w)=>sum+w.force,0);
     spin+=v.wheels.reduce((sum,w)=>sum+Math.abs(w.longSlip),0)/4/480;
    }
    entrySlip=beta(v)*180/Math.PI;const exitSpeed=v.speed*3.6;let recovered=0;
    for(let i=0;i<960;i++){
     step({...zero,steer:-course.clamp(beta(v)*2+v.body.angvel().y*.35,-1,1),throttle:.15});
     if(!recovered&&Math.abs(beta(v))<5*Math.PI/180&&Math.abs(v.body.angvel().y)<.25)recovered=(i+1)*dt;
    }
    assert(peakSlip<40&&recovered>0&&recovered<3,'Controlled corner and prompt recovery');
    assert(Math.abs(beta(v))<3*Math.PI/180&&v.forwardSpeed>4,'Slide settles with forward travel');
    slides.push({kind,surface,maneuver,peakSlip:round(peakSlip),exitSlip:round(entrySlip),heading:round(heading),frontShare:round(frontShare),meanWheelSlip:round(spin),exitKmh:round(exitSpeed),recoverySeconds:round(recovered)});world.free();
   }
  }
  for(const shape of ['mud-entry','mud-exit','mud-corner','split-left','split-right','shoulder-corner']){
   const sampler=(x,z)=>shape==='mud-exit'?(z>-12?'mud':'packed'):shape.startsWith('split')?((shape==='split-left'?x<0:x>0)?'mud':'packed'):shape==='shoulder-corner'?(x<0?'shoulder':'packed'):(z<-12?'mud':'packed');
   const {v,world,step}=rig(kind,sampler,60);let peakYaw=0,peakSlip=0,firstFront=0,firstRear=0,peakYawStep=0,lastYaw=0,mixed=false,maxDelta=0;
   for(let i=0;i<960;i++){
    const corner=shape.endsWith('corner'),brake=!corner&&i>24?.55:0;
    step({...zero,steer:corner?.16:0,brake,throttle:corner?.25:0});
    const yaw=v.body.angvel().y;peakYaw=Math.max(peakYaw,Math.abs(yaw));peakYawStep=Math.max(peakYawStep,Math.abs(yaw-lastYaw));lastYaw=yaw;
    peakSlip=Math.max(peakSlip,Math.abs(beta(v))*180/Math.PI);
    const mus=v.wheels.map(w=>w.friction/v.config.tireGrip);maxDelta=Math.max(maxDelta,Math.max(...mus)-Math.min(...mus));mixed ||=maxDelta>.1;
    if(shape==='mud-entry'){
     if(!firstFront&&mus[0]<.4)firstFront=(i+1)*dt;
     if(!firstRear&&mus[2]<.4)firstRear=(i+1)*dt;
    }
    for(const w of v.wheels)if(w.contact){const target=sample(sampler(w.contactPoint.x,w.contactPoint.z));assert(w.friction<=target.grip*v.config.tireGrip+1e-9,'No delayed loss of grip entering a slick surface');}
    if(v.speed<.2)break;
   }
   assert(mixed,'Different wheels must experience different surfaces');
   assert(peakYaw<1.2&&peakSlip<45&&peakYawStep<.06,'No abrupt transition spin');
   if(shape==='mud-entry')assert(firstFront>0&&firstRear>firstFront,'Front wheels encounter transverse boundary before rear wheels');
   transitions.push({kind,shape,peakYaw:round(peakYaw),peakSlip:round(peakSlip),peakYawStep:round(peakYawStep),firstFront:round(firstFront),firstRear:round(firstRear),maxWheelGripDifference:round(maxDelta),heading:round(Math.atan2(-v.forward.x,-v.forward.z))});world.free();
  }
  const runs=[];
  for(const hz of [30,60,120]){
   const {v,world,step}=rig(kind,(x,z)=>z<-18?'mud':x<0?'gravel':'packed',60);
   for(let i=0;i<hz*4;i++)step({...zero,steer:i<hz?.18:-.12,throttle:.3},1/hz);
   runs.push({hz,x:v.position.x,z:v.position.z,speed:v.speed});
   v.setHandlingMode('rally-legacy');v.setHandlingMode('rally');assert(v.wheels.every(w=>!w.surfaceReady&&w.relaxedSlipAngle===0),'Mode change clears terrain/contact memory');
   v.reset({x:0,y:8,z:0,tx:0,tz:-1,width:12,s:0,distance:0});step();assert(v.wheels.every(w=>!w.contact&&!w.surfaceReady&&w.longForce===0&&w.sideForce===0),'Airborne has no stale contact forces');world.free();
  }
  for(const run of runs)assert(Math.hypot(run.x-runs[0].x,run.z-runs[0].z)<.01&&Math.abs(run.speed-runs[0].speed)<.01,'Surface response independent of render cadence');
  cadence.push({kind,runs});
 }
 for(const kind of ['suv','truck']){
  const b=Object.fromEntries(braking.filter(r=>r.kind===kind).map(r=>[r.surface,r.metres]));assert(b.packed<b.gravel&&b.gravel<b.mud,'Braking distinguishes packed, gravel, mud');
  for(const surface of ['packed','gravel','mud','shoulder']){
   const r=Object.fromEntries(slides.filter(r=>r.kind===kind&&r.surface===surface).map(r=>[r.maneuver,r]));
   assert(r.trail.frontShare>r.power.frontShare+.04,'Braking vs throttle must alter physical axle loading');
   assert(r.power.meanWheelSlip>r.coast.meanWheelSlip*1.5,'Throttle must change tire traction demand');
   assert(r.power.exitKmh>r.coast.exitKmh+5,'Power exit must preserve drive');
  }
  const l=transitions.find(r=>r.kind===kind&&r.shape==='split-left'),r=transitions.find(r=>r.kind===kind&&r.shape==='split-right');
  assert(l.heading*r.heading<0&&Math.abs(l.heading+r.heading)<.02,'Split-grip response reverses with side; no fixed yaw bias');
 }
 const report={curves,mapping,braking,slides,transitions,cadence,limitations:'Production solver and synthetic surface fixtures; game tuning, no measured DiRT Rally telemetry or hands-on/browser comparison.'};
 writeFileSync(join(root,'reference/surface-validation.json'),JSON.stringify(report,null,2)+'\n');
 console.log(JSON.stringify({result:'passed',braking,slides:slides.length,transitions:transitions.length,cadence:cadence.length},null,2));
}finally{rmSync(temp,{recursive:true,force:true});}
